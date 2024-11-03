use alloc::boxed::Box;

use vexide::prelude::*;

// NOTE: do i really need to be doing an allocation for state every 10 ms update?
// convenience of isolating the update implementations
// Box::new doesnt allocate if it's zero-sized

const LIFT_THRESHOLD: f64 = 4.0;
const WRIST_THRESHOLD: f64 = 3.0;

const LIFT_VEL: i32 = 200;
const WRIST_VEL: i32 = 70;

const ACCEPT_LIFT_POS: f64 = 390.0;
const ACCEPT_WRIST_POS: f64 = -130.0;

const READY_LIFT_POS: f64 = ACCEPT_LIFT_POS;
const READY_WRIST_POS: f64 = -50.0;

const SCORE_LIFT_POS: f64 = 320.0;
const SCORE_WRIST_POS: f64 = 90.0;

const RELEASE_LIFT_POS: f64 = 640.0;
const RELEASE_WRIST_POS: f64 = 100.0;

pub enum ArmSignal {
    Empty,
    Score,
    Return,
}

// ! transition from target to target in the ready state? different ready threshold in ready state

// ! change this to take from the motor target

fn motor_ready(mtr: &Motor, thres: f64) -> bool {
    let target: MotorControl = mtr
        .target()
        .unwrap_or(MotorControl::Brake(BrakeMode::Brake));
    match target {
        MotorControl::Brake(_) => true,
        MotorControl::Position(pos, _) => {
            (mtr.position().unwrap_or_default() - pos)
                .as_degrees()
                .abs()
                < thres
        }
        _ => false,
    }
}

fn arm_move(
    lift: &mut Motor,
    lift_deg: f64,
    lift_vel: i32,
    wrist: &mut Motor,
    wrist_deg: f64,
    wrist_vel: i32,
) {
    lift.set_position_target(Position::from_degrees(lift_deg), lift_vel)
        .ok();
    wrist
        .set_position_target(Position::from_degrees(wrist_deg), wrist_vel)
        .ok();
}

pub struct Arm {
    state: Option<Box<dyn ArmState>>,
    lift: Motor,
    wrist: Motor,
}

impl Arm {
    pub fn new(lift: Motor, wrist: Motor) -> Self {
        Arm {
            state: Some(Box::new(Returning {})),
            lift,
            wrist,
        }
    }

    pub fn act(&mut self) {
        self.state
            .as_mut()
            .unwrap()
            .act(&mut self.lift, &mut self.wrist);
    }

    pub fn update(&mut self, signal: ArmSignal) {
        if let Some(s) = self.state.take() {
            self.state = Some(s.update(&self.lift, &self.wrist, signal));
        }
    }

    pub fn state(&self) -> &str {
        self.state.as_ref().unwrap().name()
    }
}

trait ArmState {
    fn act(&self, lift: &mut Motor, wrist: &mut Motor);
    fn update(self: Box<Self>, lift: &Motor, wrist: &Motor, signal: ArmSignal)
        -> Box<dyn ArmState>;
    fn name(&self) -> &str;
}

struct Returning {}

impl ArmState for Returning {
    fn act(&self, lift: &mut Motor, wrist: &mut Motor) {
        if wrist
            .position()
            .unwrap_or(Position::from_degrees(0.0))
            .as_degrees()
            <= 0.0
        {
            arm_move(lift, ACCEPT_LIFT_POS, 200, wrist, ACCEPT_WRIST_POS, 100);
        } else {
            arm_move(lift, RELEASE_LIFT_POS, 200, wrist, ACCEPT_WRIST_POS, 100);
        }
    }

    fn update(
        self: Box<Self>,
        lift: &Motor,
        wrist: &Motor,
        _signal: ArmSignal,
    ) -> Box<dyn ArmState> {
        let lift_ready = motor_ready(lift, LIFT_THRESHOLD);
        let wrist_ready = motor_ready(wrist, WRIST_THRESHOLD);
        if lift_ready && wrist_ready {
            Box::new(Accepting {})
        } else {
            self
        }
    }

    fn name(&self) -> &str {
        "returning"
    }
}

struct Accepting {}

impl ArmState for Accepting {
    fn act(&self, lift: &mut Motor, wrist: &mut Motor) {
        arm_move(
            lift,
            ACCEPT_LIFT_POS,
            LIFT_VEL,
            wrist,
            ACCEPT_WRIST_POS,
            WRIST_VEL,
        );
    }

    fn update(
        self: Box<Self>,
        _lift: &Motor,
        _wrist: &Motor,
        signal: ArmSignal,
    ) -> Box<dyn ArmState> {
        match signal {
            ArmSignal::Score => Box::new(Ready {}),
            _ => self,
        }
    }

    fn name(&self) -> &str {
        "accepting"
    }
}

struct Ready {}

impl ArmState for Ready {
    fn act(&self, lift: &mut Motor, wrist: &mut Motor) {
        arm_move(
            lift,
            READY_LIFT_POS,
            LIFT_VEL,
            wrist,
            READY_WRIST_POS,
            WRIST_VEL,
        );
    }

    fn update(
        self: Box<Self>,
        _lift: &Motor,
        wrist: &Motor,
        _signal: ArmSignal,
    ) -> Box<dyn ArmState> {
        let lift_ready = true; // motor_ready(lift, LIFT_THRESHOLD + 20.0);
        let wrist_ready = motor_ready(wrist, WRIST_THRESHOLD + 30.0);
        if lift_ready && wrist_ready {
            Box::new(Scoring {})
        } else {
            self
        }
    }

    fn name(&self) -> &str {
        "ready"
    }
}

struct Scoring {}

impl ArmState for Scoring {
    fn act(&self, lift: &mut Motor, wrist: &mut Motor) {
        arm_move(
            lift,
            SCORE_LIFT_POS,
            LIFT_VEL,
            wrist,
            SCORE_WRIST_POS,
            WRIST_VEL,
        );
    }

    fn update(
        self: Box<Self>,
        lift: &Motor,
        wrist: &Motor,
        signal: ArmSignal,
    ) -> Box<dyn ArmState> {
        if matches!(signal, ArmSignal::Return) {
            return Box::new(Returning {});
        }

        let lift_ready = motor_ready(lift, LIFT_THRESHOLD);
        let wrist_ready = motor_ready(wrist, WRIST_THRESHOLD);
        if lift_ready && wrist_ready {
            Box::new(Releasing {})
        } else {
            self
        }
    }

    fn name(&self) -> &str {
        "scoring"
    }
}

struct Releasing {}

impl ArmState for Releasing {
    fn act(&self, lift: &mut Motor, wrist: &mut Motor) {
        if lift
            .position()
            .unwrap_or(Position::from_degrees(0.0))
            .as_degrees()
            <= 590.0
        {
            arm_move(
                lift,
                RELEASE_LIFT_POS,
                LIFT_VEL,
                wrist,
                SCORE_WRIST_POS,
                100,
            );
        } else {
            arm_move(
                lift,
                RELEASE_LIFT_POS,
                LIFT_VEL,
                wrist,
                RELEASE_WRIST_POS,
                100,
            );
        }
    }

    fn update(
        self: Box<Self>,
        lift: &Motor,
        wrist: &Motor,
        _signal: ArmSignal,
    ) -> Box<dyn ArmState> {
        let lift_ready = motor_ready(lift, LIFT_THRESHOLD + 5.0);
        let wrist_ready = motor_ready(wrist, WRIST_THRESHOLD + 10.0);
        if lift_ready && wrist_ready {
            Box::new(Returning {})
        } else {
            self
        }
    }

    fn name(&self) -> &str {
        "releasing"
    }
}

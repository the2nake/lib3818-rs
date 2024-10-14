use alloc::boxed::Box;

use vexide::prelude::*;

// NOTE: do i really need to be doing an allocation for state every 10 ms update?
// convenience of isolating the update implementations
// Box::new doesnt allocate if it's zero-sized

const LIFT_THRESHOLD: f64 = 4.0;
const WRIST_THRESHOLD: f64 = 3.0;

const LIFT_VEL: i32 = 120;
const WRIST_VEL: i32 = 70;

const READY_LIFT_POS: f64 = 380.0;
const READY_WRIST_POS: f64 = -130.0;

const SCORE_LIFT_POS: f64 = 580.0;
const SCORE_WRIST_POS: f64 = 100.0;

pub enum ArmSignal {
    None,
    Score,
    Return,
}

fn motor_ready(mtr: &Motor, deg: f64, thres: f64) -> bool {
    (mtr.position().unwrap_or_default().as_degrees() - deg).abs() < thres
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
            state: Some(Box::new(Readying {})),
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
        self.state.as_ref().unwrap().state_name()
    }
}

trait ArmState {
    fn act(&self, lift: &mut Motor, wrist: &mut Motor);
    fn update(self: Box<Self>, lift: &Motor, wrist: &Motor, signal: ArmSignal)
        -> Box<dyn ArmState>;
    fn state_name(&self) -> &str;
}

struct Readying {}

impl ArmState for Readying {
    fn act(&self, lift: &mut Motor, wrist: &mut Motor) {
        arm_move(lift, READY_LIFT_POS, 200, wrist, READY_WRIST_POS, 100);
    }

    fn update(
        self: Box<Self>,
        lift: &Motor,
        wrist: &Motor,
        _signal: ArmSignal,
    ) -> Box<dyn ArmState> {
        let lift_ready = motor_ready(lift, READY_LIFT_POS, LIFT_THRESHOLD);
        let wrist_ready = motor_ready(wrist, READY_WRIST_POS, WRIST_THRESHOLD);
        if lift_ready && wrist_ready {
            Box::new(Ready {})
        } else {
            self
        }
    }

    fn state_name(&self) -> &str {
        "readying"
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
        _wrist: &Motor,
        signal: ArmSignal,
    ) -> Box<dyn ArmState> {
        match signal {
            ArmSignal::Score => Box::new(Scoring {}),
            _ => self,
        }
    }

    fn state_name(&self) -> &str {
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
            return Box::new(Readying {});
        }

        let lift_ready = motor_ready(lift, SCORE_LIFT_POS, LIFT_THRESHOLD);
        let wrist_ready = motor_ready(wrist, SCORE_WRIST_POS, WRIST_THRESHOLD);
        if lift_ready && wrist_ready {
            Box::new(Readying {})
        } else {
            self
        }
    }

    fn state_name(&self) -> &str {
        "scoring"
    }
}

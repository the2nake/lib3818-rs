use vexide::prelude::*;

// TOOD:  swtich to list of motors
pub struct TankChassis {
    l1: Motor,
    l2: Motor,
    lt: Motor,
    r1: Motor,
    r2: Motor,
    rt: Motor,
}

impl TankChassis {
    pub fn new(l1: Motor, l2: Motor, lt: Motor, r1: Motor, r2: Motor, rt: Motor) -> Self {
        TankChassis {
            l1,
            l2,
            lt,
            r1,
            r2,
            rt,
        }
    }

    pub fn move_tank(&mut self, mut left: f32, mut right: f32) {
        if left.abs() > 1.0 || right.abs() > 1.0 {
            let mut max = left.abs();
            if right.abs() > left.abs() {
                max = right.abs();
            }
            left /= max;
            right /= max;
        }
        left *= 12.0;
        right *= 12.0;
        self.l1.set_voltage(left.into()).ok();
        self.l2.set_voltage(left.into()).ok();
        self.lt.set_voltage(left.into()).ok();
        self.r1.set_voltage(right.into()).ok();
        self.r2.set_voltage(right.into()).ok();
        self.rt.set_voltage(right.into()).ok();
    }

    pub fn move_arcade(&mut self, mut throttle: f32, mut steer: f32) {
        throttle = throttle.clamp(-1.0, 1.0);
        steer = steer.clamp(-1.0, 1.0);
        self.move_tank(throttle - steer, throttle + steer);
    }

    pub fn stop(&mut self, mode: BrakeMode) {
        self.l1.brake(mode).ok();
        self.l2.brake(mode).ok();
        self.lt.brake(mode).ok();
        self.r1.brake(mode).ok();
        self.r2.brake(mode).ok();
        self.rt.brake(mode).ok();
    }
}

pub mod model;
pub mod mp;

use alloc::vec::Vec;

use vexide::prelude::*;

pub struct TankChassis {
    left: Vec<Motor>,
    right: Vec<Motor>,
}

impl TankChassis {
    pub fn new(l1: Motor, l2: Motor, lt: Motor, r1: Motor, r2: Motor, rt: Motor) -> Self {
        let vec_left: Vec<Motor> = vec![l1, l2, lt];
        let vec_right: Vec<Motor> = vec![r1, r2, rt];
        TankChassis {
            left: vec_left,
            right: vec_right,
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
        for motor in self.left.iter_mut() {
            motor.set_voltage(left.into()).ok();
        }
        for motor in self.right.iter_mut() {
            motor.set_voltage(right.into()).ok();
        }
    }

    pub fn move_arcade(&mut self, throttle: f32, steer: f32) {
        self.move_tank(throttle - steer, throttle + steer);
    }

    pub fn brake(&mut self, mode: BrakeMode) {
        for motor in self.left.iter_mut() {
            motor.brake(mode).ok();
        }
        for motor in self.right.iter_mut() {
            motor.brake(mode).ok();
        }
    }
}

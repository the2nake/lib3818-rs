#![no_main]
#![no_std]

#[macro_use]
extern crate alloc;

mod chassis;

use core::time::Duration;

use vexide::{core::time::Instant, devices::screen::*, prelude::*};

use crate::chassis::*;

struct Robot {
    controller: Controller,
    chassis: TankChassis,

    intake: Motor,
    lift: Motor,
    wrist: Motor,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous!");
        self.chassis.move_tank(1.0, 1.0);
        sleep(Duration::new(0, 500_000_000)).await;
        self.chassis.stop(BrakeMode::Brake);
    }

    async fn driver(&mut self) {
        println!("Driver!");
        loop {
            let time_start = Instant::now();

            if self.controller.right_trigger_2.is_pressed().unwrap_or(false) {
                self.intake.set_voltage(12.0).ok();
            } else if self.controller.right_trigger_1.is_pressed().unwrap_or(false) {
                self.intake.set_voltage(-12.0).ok();
            } else {
                self.intake.brake(BrakeMode::Brake).ok();
            }

            let throttle: f32 = self.controller.left_stick.y().unwrap_or(0.0) as f32;
            let steer: f32 = self.controller.right_stick.x().unwrap_or(0.0) as f32;
            self.chassis.move_arcade(throttle, -steer);

            sleep_until(time_start + Duration::new(0, 20_000_000)).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let m_l1 = Motor::new(peripherals.port_6, Gearset::Blue, Direction::Reverse);
    let m_l2 = Motor::new(peripherals.port_7, Gearset::Blue, Direction::Reverse);
    let m_lt = Motor::new(peripherals.port_8, Gearset::Blue, Direction::Forward);

    let m_r1 = Motor::new(peripherals.port_20, Gearset::Blue, Direction::Forward);
    let m_r2 = Motor::new(peripherals.port_19, Gearset::Blue, Direction::Forward);
    let m_rt = Motor::new(peripherals.port_18, Gearset::Blue, Direction::Reverse);

    let m_h_lift = Motor::new(peripherals.port_3, Gearset::Green, Direction::Forward);
    let m_wrist = Motor::new(peripherals.port_4, Gearset::Red, Direction::Forward);

    let m_h_intake = Motor::new(peripherals.port_10, Gearset::Green, Direction::Reverse);

    /*
    let mut odom_x = RotationSensor::new(peripherals.port_11, Direction::Forward);
    odom_x.set_data_rate(Duration::from_millis(5)).ok();
    */

    let drive = TankChassis::new(m_l1, m_l2, m_lt, m_r1, m_r2, m_rt);

    let mut master = peripherals.primary_controller;
    let mut scr = peripherals.screen;

    while !master.button_y.was_pressed().unwrap_or(false) {
        let obj = Text::new("press y", TextSize::Small, (0, 0));
        scr.fill(&obj, Rgb::WHITE);
        sleep(Duration::new(0, 20_000_000)).await;
    }

    let robot = Robot {
        controller: master,
        chassis: drive,
        intake: m_h_intake,
        lift: m_h_lift,
        wrist: m_wrist,
    };

    robot.compete().await;
}

#![no_main]
#![no_std]

#[macro_use]
extern crate alloc;

mod arm;
mod piston;
mod tank_chassis;

use alloc::string::{String, ToString};
use core::time::Duration;

use vexide::{core::time::Instant, devices::screen::*, prelude::*};

use crate::{arm::*, piston::Piston, tank_chassis::TankChassis};

struct Robot {
    screen: Screen,
    controller: Controller,
    chassis: TankChassis,

    intake: Motor,

    arm: Arm,
    clamp: Piston,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous!");
        self.chassis.move_tank(1.0, 1.0);
        sleep(Duration::from_secs_f32(0.5)).await;
        self.chassis.brake(BrakeMode::Brake);
    }

    async fn driver(&mut self) {
        println!("Driver!");

        let mut scoring_millis = 0.0;

        loop {
            let time_start = Instant::now();

            // drive the intake using right triggers
            if self
                .controller
                .right_trigger_2
                .is_pressed()
                .unwrap_or(false)
            {
                self.intake.set_voltage(12.0).ok();
            } else if self
                .controller
                .right_trigger_1
                .is_pressed()
                .unwrap_or(false)
            {
                self.intake.set_voltage(-12.0).ok();
            } else {
                self.intake.brake(BrakeMode::Brake).ok();
            }

            // send score signal if left trigger is pressed
            let mut signal = ArmSignal::Empty;
            if self.controller.left_trigger_2.is_pressed().unwrap_or(false) {
                signal = ArmSignal::Score;
            }
            // scoring timeout
            if self.arm.state() == "scoring" {
                scoring_millis += 20.0;
            } else {
                scoring_millis = 0.0;
            }
            if scoring_millis >= 1000.0 {
                signal = ArmSignal::Return;
            }
            self.arm.update(signal);
            // perform the action
            self.arm.act();

            if self
                .controller
                .left_trigger_1
                .was_pressed()
                .unwrap_or(false)
            {
                self.clamp.toggle();
            }

            // display arm state
            let obj = Text::new(
                (String::from("arm state: ") + self.arm.state()).as_str(),
                TextSize::Small,
                (0, 0),
            );
            self.screen.fill(&obj, Rgb::WHITE);

            let text_height = obj.height();
            // display clamp state
            let obj = Text::new(
                (String::from("clamp state: ")
                    + self.clamp.activated().to_string().as_str()
                    + "      ")
                    .as_str(),
                TextSize::Small,
                (0, text_height as i16),
            );
            self.screen.fill(&obj, Rgb::WHITE);

            let throttle: f32 = self.controller.left_stick.y().unwrap_or(0.0) as f32;
            let steer: f32 = self.controller.right_stick.x().unwrap_or(0.0) as f32;
            self.chassis.move_arcade(throttle, -steer);

            sleep_until(time_start + Duration::from_millis(20)).await;
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

    let adi_clamp = AdiDigitalOut::new(peripherals.adi_a);

    /*
    let mut odom_x = RotationSensor::new(peripherals.port_11, Direction::Forward);
    odom_x.set_data_rate(Duration::from_millis(5)).ok();
    */

    let drive = TankChassis::new(m_l1, m_l2, m_lt, m_r1, m_r2, m_rt);

    let master = peripherals.primary_controller;
    let scr = peripherals.screen;

    let mut robot = Robot {
        screen: scr,
        controller: master,
        chassis: drive,
        intake: m_h_intake,
        arm: Arm::new(m_h_lift, m_wrist),
        clamp: Piston::new(adi_clamp, false),
    };

    while robot.arm.state() != "ready" {
        robot.arm.update(ArmSignal::Empty);
        robot.arm.act();
        sleep(Duration::from_millis(20)).await;
    }

    robot.compete().await;
}

// ! fix the bug where things start actuated

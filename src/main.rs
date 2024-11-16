#![no_main]
#![no_std]

#[macro_use]
extern crate alloc;
mod arm;
mod localisation;
mod math;
mod piston;
mod tank_chassis;

use alloc::sync::Arc;
use core::{f64::consts, time::Duration};

use vexide::{
    core::{sync::Mutex, time::Instant},
    devices::screen::*,
    prelude::*,
};

use crate::{
    arm::*,
    localisation::*,
    math::*,
    piston::Piston,
    tank_chassis::{boomerang::*, TankChassis},
};

struct Robot {
    scr: Screen,
    controller: Controller,
    chassis: Arc<Mutex<TankChassis>>,

    intake: Motor,

    arm: Arm,
    clamp: Piston,
    distance_cage: DistanceSensor,

    localiser: Arc<Mutex<TrackingWheelLocaliser<DeadWheelTrackingAxis, DriveTrackingAxis>>>,
}

impl Robot {
    fn ring_in_cage(&self) -> bool {
        /*
        self.distance_cage
            .object()
            .unwrap_or(None)
            .is_some_and(|x| x.distance < 35)
            && self.arm.state() == "accepting";
            */
        true
    }
    fn ring_ready_auto(&self) -> bool {
        self.distance_cage
            .object()
            .unwrap_or(None)
            .is_some_and(|x| x.distance < 35)
            && self.arm.state() == "accepting"
    }
    fn cage_ready(&self) -> bool {
        self.distance_cage
            .object()
            .unwrap_or(None)
            .is_none_or(|x| x.distance > 40)
            && self.arm.state() == "accepting"
    }

    async fn forward(&mut self, metres: f64) {
        let pose0 = self.localiser.lock().await.pose();
        let mut curr = pose0;
        while curr.dist(&pose0) < metres {
            curr = self.localiser.lock().await.pose();
            self.chassis
                .lock()
                .await
                .move_arcade((curr.dist(&pose0)) / metres.abs(), 0.0);
            sleep(Duration::from_millis(20)).await;
        }
        self.chassis.lock().await.brake(BrakeMode::Brake);
    }

    async fn turn(&mut self, target: f64) {
        let mut curr = self.localiser.lock().await.pose();
        while shorter_rad(curr.h.as_rad(AngleSystem::Cartesian), target).abs() > 0.1 {
            curr = self.localiser.lock().await.pose();
            let delta = shorter_rad(curr.h.as_rad(AngleSystem::Cartesian), target);
            self.chassis.lock().await.move_arcade(0.0, delta * 0.3);
            sleep(Duration::from_millis(20)).await;
        }
        self.chassis.lock().await.brake(BrakeMode::Brake);
    }
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous!");
        self.localiser
            .lock()
            .await
            .set_pose(Pose::new(0.0, 0.0, Heading::new(consts::FRAC_PI_2)));

        self.forward(1.0).await;
        /*
        let mut alg = TankPid::new(self.chassis.clone(), self.localiser.clone(), 0.5, 0.3);
        let target = Pose::new(-0.5, 0.5, Heading::new(0.0));

        if self.ring_ready_auto() {
            self.arm.update(ArmSignal::Score);
        } else {
            self.arm.update(ArmSignal::Empty);
        }
        self.arm.act();

        let mut pose = self.localiser.lock().await.pose();
        while pose.dist(&target) > 0.05 {
            let mut localiser_lock = self.localiser.lock().await;
            localiser_lock.update().await;
            pose = localiser_lock.pose();
            let obj = Text::new(
                format!("pose: {}                       ", pose).as_str(),
                TextSize::Small,
                (0, 0),
            );
            self.scr.fill(&obj, Rgb::WHITE);
            drop(localiser_lock);
            self.arm.update(ArmSignal::Empty);
            self.arm.act();
            alg.approach_point(target).await;
            sleep(Duration::from_millis(10)).await;
        }
        alg.brake(BrakeMode::Brake).await;
        */
        /*
        let mut boom = Boomerang::new(alg, self.localiser.clone(), 0.5, 0.5, 0.02, 100);

        // ! update odometry

        // TODO: implement angle as an enum
        boom.drive(
            Pose::new(1.0, 1.0, Heading::from_deg(90.0, AngleSystem::Cartesian)),
            BrakeMode::Brake,
            1000,
        )
        .await;*/
    }

    async fn driver(&mut self) {
        //self.autonomous().await;
        println!("Driver!");

        let mut scoring_millis = 0.0;

        loop {
            let time_start = Instant::now();

            // sensor updates
            // TODO: move to task
            // ! move to a task
            self.localiser.lock().await.update().await;

            // drive the intake using right triggers
            // block the intake if cage is full
            if self
                .controller
                .right_trigger_2
                .is_pressed()
                .unwrap_or(false)
                && self.cage_ready()
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
            if self.controller.left_trigger_2.is_pressed().unwrap_or(false) && self.ring_in_cage() {
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

            if self.controller.button_left.was_pressed().unwrap_or(false) {
                self.clamp.toggle();
            }

            // display stuff
            // TODO: move to tasks
            // display arm state
            let obj = Text::new(
                format!("arm state: {}     ", self.arm.state()).as_str(),
                TextSize::Small,
                (0, 0),
            );
            self.scr.fill(&obj, Rgb::WHITE);

            let text_height = obj.height();
            // display clamp state
            let obj = Text::new(
                format!("clamp state: {}     ", self.clamp.activated()).as_str(),
                TextSize::Small,
                (0, text_height as i16),
            );
            self.scr.fill(&obj, Rgb::WHITE);

            // TODO: get better string concatenation
            // display position
            let pose = self.localiser.lock().await.pose();
            let obj = Text::new(
                format!("pose: {}                       ", pose).as_str(),
                TextSize::Small,
                (0, 2 * text_height as i16),
            );
            self.scr.fill(&obj, Rgb::WHITE);

            // arcade control
            let throttle: f64 = self.controller.left_stick.y().unwrap_or(0.0);
            let steer: f64 = 0.7 * self.controller.right_stick.x().unwrap_or(0.0);
            self.chassis.lock().await.move_arcade(throttle, -steer);

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

    let intake = Motor::new(peripherals.port_10, Gearset::Green, Direction::Reverse);

    let adi_clamp = AdiDigitalOut::new(peripherals.adi_a);

    let distance_cage = DistanceSensor::new(peripherals.port_12);

    let mut odom_x = RotationSensor::new(peripherals.port_11, Direction::Reverse);
    odom_x.set_data_rate(Duration::from_millis(5)).ok();

    let imu = InertialSensor::new(peripherals.port_13);

    let chassis = Arc::new(Mutex::new(TankChassis::new(
        m_l1, m_l2, m_lt, m_r1, m_r2, m_rt,
    )));

    let localiser = Arc::new(Mutex::new(TrackingWheelLocaliser::from_chassis_and_wheel(
        imu,
        DeadWheelTrackingAxis::new(odom_x, 0.220 / 360.0, 0.0),
        DriveTrackingAxis::new(chassis.clone(), (48.0 / 60.0) * 0.220 / 360.0),
        Pose::new(0.0, 0.0, Heading::new(0.0)),
    )));

    let mut master = peripherals.primary_controller;
    let scr = peripherals.screen;

    master.button_left.was_pressed().ok();

    let mut robot = Robot {
        scr,
        controller: master,
        chassis,
        localiser,
        intake,
        arm: Arm::new(m_h_lift, m_wrist),
        clamp: Piston::new(adi_clamp, false),
        distance_cage,
    };

    sleep(Duration::new(2, 0)).await;
    while robot.arm.state() != "accepting" {
        robot.arm.update(ArmSignal::Empty);
        robot.arm.act();
        sleep(Duration::from_millis(20)).await;
    }

    // ! double exec

    robot.compete().await;
}

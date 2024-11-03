use alloc::sync::Arc;
use core::f64::*;

use vexide::{
    core::sync::Mutex,
    prelude::{Position, RotationSensor},
};

use crate::tank_chassis::TankChassis;

pub enum AngleSystem {
    Cartesian,
    Bearing,
}

// * internally stored as radians from the positive x axis
// ? is this even needed
#[derive(Copy, Clone)]
pub struct Heading {
    rad: f64,
}

impl Heading {
    pub fn new(rad: f64) -> Self {
        Heading { rad }
    }

    pub fn from_deg(deg: f64, system: AngleSystem) -> Self {
        let mut heading = Heading { rad: 0.0 };
        heading.set_deg(deg, system);
        heading
    }

    pub fn from_rad(rad: f64, system: AngleSystem) -> Self {
        let mut heading = Heading { rad: 0.0 };
        heading.set_rad(rad, system);
        heading
    }

    pub fn set_deg(&mut self, mut deg: f64, system: AngleSystem) {
        if matches!(system, AngleSystem::Bearing) {
            deg = 90.0 - deg;
        }
        self.rad = deg.to_radians();
    }

    pub fn set_rad(&mut self, mut rad: f64, system: AngleSystem) {
        if matches!(system, AngleSystem::Bearing) {
            rad = consts::FRAC_PI_2 - rad;
        }
        self.rad = rad;
    }

    pub fn as_deg(&self, system: AngleSystem) -> f64 {
        if matches!(system, AngleSystem::Bearing) {
            90.0 - self.rad.to_degrees()
        } else {
            self.rad.to_degrees()
        }
    }

    pub fn as_rad(&self, system: AngleSystem) -> f64 {
        if matches!(system, AngleSystem::Bearing) {
            consts::FRAC_PI_2 - self.rad
        } else {
            self.rad
        }
    }
}
#[derive(Copy, Clone)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub h: Heading,
}

impl Pose {
    pub fn new(x: f64, y: f64, h: Heading) -> Self {
        Pose { x, y, h }
    }
}

impl core::fmt::Display for Pose {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "({}, {}, {})",
            self.x,
            self.y,
            self.h.as_rad(AngleSystem::Cartesian)
        )
    }
}

pub trait Localiser {
    fn pose(&self) -> Pose;
    fn set_pose(&mut self, pose: Pose);
}

// TODO: make pose thread-safe
pub struct TrackingWheelLocaliser<TX: TrackingAxis, TY: TrackingAxis> {
    x_axis: TX,
    y_axis: TY,
    pose: Pose,
}

impl<TX: TrackingAxis, TY: TrackingAxis> TrackingWheelLocaliser<TX, TY> {
    pub fn from_chassis_and_wheel(x_tracker: TX, y_tracker: TY, init_pose: Pose) -> Self {
        TrackingWheelLocaliser {
            x_axis: x_tracker,
            y_axis: y_tracker,
            pose: init_pose,
        }
    }

    pub async fn update(&mut self) {
        self.pose.x = self.x_axis.deg().await;
        self.pose.y = self.y_axis.deg().await;
        self.pose.h.set_deg(0.0, AngleSystem::Cartesian);
    }
}

impl<TX: TrackingAxis, TY: TrackingAxis> Localiser for TrackingWheelLocaliser<TX, TY> {
    fn pose(&self) -> Pose {
        self.pose
    }
    fn set_pose(&mut self, pose: Pose) {
        self.pose = pose;
    }
}

pub trait TrackingAxis {
    async fn deg(&self) -> f64;
    fn pos(&self) -> f64;
}

pub struct TrackerAxisWheel {
    sensor: RotationSensor,
    pos: f64,
}

impl TrackerAxisWheel {
    pub fn new(sensor: RotationSensor, pos: f64) -> Self {
        TrackerAxisWheel { sensor, pos }
    }
}

impl TrackingAxis for TrackerAxisWheel {
    async fn deg(&self) -> f64 {
        self.sensor
            .position()
            .unwrap_or(Position::from_degrees(0.0))
            .as_degrees()
    }

    fn pos(&self) -> f64 {
        self.pos
    }
}

pub struct TrackerAxisDrive {
    chassis: Arc<Mutex<TankChassis>>,
    track_width: f64,
}

impl TrackerAxisDrive {
    pub fn new(chassis: Arc<Mutex<TankChassis>>, track_width: f64) -> Self {
        TrackerAxisDrive {
            chassis,
            track_width,
        }
    }
}

impl TrackingAxis for TrackerAxisDrive {
    async fn deg(&self) -> f64 {
        self.chassis.lock().await.right_deg()
    }

    // cartesian coordinate
    fn pos(&self) -> f64 {
        self.track_width / 2.0
    }
}

use alloc::sync::Arc;
use core::f64::*;

use vexide::{core::sync::Mutex, prelude::*};

use crate::{math::*, tank_chassis::TankChassis};

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

    pub fn set_deg(&mut self, deg: f64, system: AngleSystem) {
        self.set_rad(deg.to_radians(), system);
    }

    pub fn set_rad(&mut self, rad: f64, system: AngleSystem) {
        if matches!(system, AngleSystem::Bearing) {
            self.rad = consts::FRAC_PI_2 - rad;
        } else {
            self.rad = rad;
        }
    }

    pub fn as_deg(&self, system: AngleSystem) -> f64 {
        match system {
            AngleSystem::Bearing => 90.0 - self.rad.to_degrees(),
            AngleSystem::Cartesian => self.rad.to_degrees(),
        }
    }

    pub fn as_rad(&self, system: AngleSystem) -> f64 {
        match system {
            AngleSystem::Bearing => consts::FRAC_PI_2 - self.rad,
            AngleSystem::Cartesian => self.rad,
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
            "({:.3}, {:.3}, {:.3})",
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
    pub imu: InertialSensor,
    x_axis: TX,
    y_axis: TY,
    pose: Pose,

    prev_heading: f64,
    prev_deg_x: f64,
    prev_deg_y: f64,
}

impl<TX: TrackingAxis, TY: TrackingAxis> TrackingWheelLocaliser<TX, TY> {
    pub fn from_chassis_and_wheel(
        imu: InertialSensor,
        x_axis: TX,
        y_axis: TY,
        init_pose: Pose,
    ) -> Self {
        TrackingWheelLocaliser {
            imu,
            x_axis,
            y_axis,
            pose: init_pose,
            prev_heading: f64::NAN,
            prev_deg_x: f64::NAN,
            prev_deg_y: f64::NAN,
        }
    }

    pub async fn update(&mut self) {
        let deg_x = self.x_axis.deg().await;
        let mut dx = (deg_x - self.prev_deg_x) * self.x_axis.ticks_per_deg();
        let deg_y = self.y_axis.deg().await;
        let mut dy = (deg_y - self.prev_deg_y) * self.y_axis.ticks_per_deg();

        let raw_h = self.imu.heading().unwrap_or(f64::NAN);
        let mut dh = -shorter_deg(self.prev_heading, raw_h).to_radians();

        if self.prev_deg_x.is_nan() {
            self.prev_deg_x = deg_x;
            dx = 0.0;
        }

        if self.prev_deg_y.is_nan() {
            self.prev_deg_y = deg_y;
            dy = 0.0;
        }

        if dh.is_nan() || self.prev_heading.is_nan() {
            self.prev_heading = raw_h;
            dh = 0.0;
        }

        print!("imu raw: {}\n", raw_h);
        print!("raw internal: {}\n", self.pose.h.rad);

        self.pose.x += dx;
        self.pose.y += dy;
        self.pose.h.rad += dh;

        self.prev_deg_x = deg_x;
        self.prev_deg_y = deg_y;
        self.prev_heading = raw_h;
        /*
        let dh = self.pose.h.as_rad(AngleSystem::Cartesian) - self.prev_heading;

        self.x_axis.pos();
        self.y_axis.pos();

        let mut x_impact_lx = 0.0;
        let mut x_impact_ly = 0.0;
        let mut y_impact_lx = 0.0;
        let mut y_impact_ly = 0.0;

        let is_low_turn = dh.abs().to_degrees() < 0.3;

        if is_low_turn {
            x_impact_lx = dx;
            y_impact_ly = dy;
        } else {
            let tmp = -dx / dh - self.x_axis.pos();
            x_impact_lx = tmp * dh.sin();
            x_impact_ly = (dh.cos() - 1.0) * tmp;
            let tmp = -dy / dh + self.y_axis.pos();
            y_impact_lx = (1.0 - dh.cos()) * tmp;
            y_impact_ly = dh.sin() * tmp;
        }

        let dx_l = x_impact_lx + y_impact_lx;
        let dy_l = x_impact_ly + y_impact_ly;
        let dx_g = dx_l * self.prev_heading.to_radians().cos()
            + dy_l * self.prev_heading.to_radians().sin();
        let dy_g = -dx_l * self.prev_heading.to_radians().sin()
            + dy_l * self.prev_heading.to_radians().cos();

        self.pose.x += dx_g;
        self.pose.y += dy_g;
        self.prev_heading = self.pose.h.rad;
        self.prev_deg_x = deg_x;
        self.prev_deg_y = deg_y;*/
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
    fn ticks_per_deg(&self) -> f64;
    fn pos(&self) -> f64;
}

pub struct TrackerAxisWheel {
    sensor: RotationSensor,
    dist_per_deg: f64,
    pos: f64,
}

impl TrackerAxisWheel {
    pub fn new(sensor: RotationSensor, dist_per_deg: f64, pos: f64) -> Self {
        TrackerAxisWheel {
            sensor,
            dist_per_deg,
            pos,
        }
    }
}

impl TrackingAxis for TrackerAxisWheel {
    async fn deg(&self) -> f64 {
        self.sensor
            .position()
            .unwrap_or(Position::from_degrees(0.0))
            .as_degrees()
    }

    fn ticks_per_deg(&self) -> f64 {
        self.dist_per_deg
    }

    fn pos(&self) -> f64 {
        self.pos
    }
}

pub struct TrackerAxisDrive {
    chassis: Arc<Mutex<TankChassis>>,
    ticks_per_deg: f64,
}

impl TrackerAxisDrive {
    pub fn new(chassis: Arc<Mutex<TankChassis>>, ticks_per_deg: f64) -> Self {
        TrackerAxisDrive {
            chassis,
            ticks_per_deg,
        }
    }
}

impl TrackingAxis for TrackerAxisDrive {
    async fn deg(&self) -> f64 {
        let chassis = self.chassis.lock().await;
        (chassis.right_deg() + chassis.left_deg()) / 2.0
    }

    fn ticks_per_deg(&self) -> f64 {
        self.ticks_per_deg
    }

    // cartesian coordinate
    fn pos(&self) -> f64 {
        0.0
    }
}

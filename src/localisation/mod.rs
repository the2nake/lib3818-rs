use core::f64::*;

pub enum AngleSystem {
    Cartesian,
    Bearing,
}

// * internally stored as radians from the positive x axis
// ? is this even needed
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

pub struct Pose {
    x: f64,
    y: f64,
    h: Heading,
}

trait Localiser {
    fn pose(&self) -> Pose;
    fn update(&mut self);
}

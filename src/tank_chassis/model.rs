use crate::tank_chassis::mp::TankConstraints;

pub struct TankModel {
    mass: f64,
    lateral_mu: f64,
    track_width: f64,
    lin_speed: f64,
    force: f64,
}

pub struct TankVelocities {
    pub left: f64,
    pub right: f64,
}

impl TankModel {
    fn wheel_vels(&self, linear_velocity: f64, curvature: f64) -> TankVelocities {
        // if turning left is positive curvature
        // TODO: put this as -curvature if turning right is positive
        let angular_velocity = linear_velocity * curvature;

        let left_r = 1.0 / curvature - self.track_width / 2.0;
        let right_r = 1.0 / curvature + self.track_width / 2.0;

        let left = angular_velocity * left_r;
        let right = angular_velocity * right_r;

        TankVelocities { left, right }
    }

    fn constraints(&self, curvature: f64) -> TankConstraints {
        let mut linear_velocity = self.lin_speed;
        if curvature > 0.0 {
            // left turn, right > left
            let right_r = 1.0 / curvature + self.track_width / 2.0;
            linear_velocity = self.lin_speed / (curvature * right_r);
        } else if curvature < 0.0 {
            // right turn, left > right
            let left_r = 1.0 / curvature - self.track_width / 2.0;
            linear_velocity = self.lin_speed / (curvature * left_r);
        }
        TankConstraints {
            max_vel: linear_velocity,
            max_accel: self.force / self.mass,
        }
    }
}

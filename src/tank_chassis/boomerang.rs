use alloc::sync::Arc;
use core::{f64::consts, time::Duration};

use vexide::core::{sync::Mutex, time::Instant};

use crate::{localisation::*, math::shorter_rad, tank_chassis::*};

pub struct TankPid<T: Localiser> {
    chassis: Arc<Mutex<TankChassis>>,
    localiser: Arc<Mutex<T>>,
    lin_gain: f64,
    ang_gain: f64,
}

pub trait MoveToPoint {
    async fn approach_point<T: Point>(&mut self, point: T);
    async fn brake(&mut self, mode: BrakeMode);
}

impl<T: Localiser> TankPid<T> {
    pub fn new(
        chassis: Arc<Mutex<TankChassis>>,
        localiser: Arc<Mutex<T>>,
        lin_gain: f64,
        ang_gain: f64,
    ) -> Self {
        TankPid {
            chassis,
            localiser,
            lin_gain,
            ang_gain,
        }
    }
}

impl<T: Localiser> MoveToPoint for TankPid<T> {
    async fn approach_point<T2: Point>(&mut self, point: T2) {
        let pose = self.localiser.lock().await.pose();
        let mut lin_err = pose.dist(&point);
        let err_x = point.pos().0 - pose.x;
        let err_y = point.pos().1 - pose.y;
        let ang_target = err_y.atan2(err_x);
        let mut ang_err = shorter_rad(pose.h.as_rad(AngleSystem::Cartesian), ang_target);

        println!("before correct {} {}", lin_err, ang_err);

        if ang_err.abs() > consts::FRAC_PI_2 {
            ang_err = shorter_rad(
                pose.h.as_rad(AngleSystem::Cartesian),
                ang_target + consts::PI,
            );
            lin_err *= -1.0;
        }

        println!("after correct {} {}", lin_err, ang_err);

        self.chassis
            .lock()
            .await
            .move_arcade(lin_err * self.lin_gain, ang_err * self.ang_gain);
    }

    async fn brake(&mut self, mode: BrakeMode) {
        self.chassis.lock().await.brake(mode);
    }
}

pub struct Boomerang<T1: MoveToPoint, T2: Localiser> {
    algorithm: T1,
    localiser: Arc<Mutex<T2>>,
    dist_stop_range: f64,
    min_stop_millis: u64,

    dlead: f64,
    glead: f64,

    target: Option<Pose>,
    millis_in_range: u64,
}

impl<T1: MoveToPoint, T2: Localiser> Boomerang<T1, T2> {
    pub fn new(
        algorithm: T1,
        localiser: Arc<Mutex<T2>>,
        dlead: f64,
        glead: f64,
        dist_stop_range: f64,
        min_stop_millis: u64,
    ) -> Self {
        Boomerang {
            algorithm,
            localiser,
            dlead: dlead.clamp(0.0, 1.0),
            glead: glead.clamp(0.0, 1.0),
            dist_stop_range,
            min_stop_millis,
            target: None,
            millis_in_range: 0,
        }
    }
    pub fn settled(&self) -> bool {
        self.min_stop_millis < self.millis_in_range
    }

    pub async fn drive(&mut self, target: Pose, brake_mode: BrakeMode, ms_timeout: u128) {
        self.target = Some(target);
        self.millis_in_range = 0;

        let start = Instant::now();
        let mut prev_update = Instant::now();

        while !self.settled() || prev_update.duration_since(start).as_millis() < ms_timeout {
            let pose = self.localiser.lock().await.pose();
            let err = match (self.target) {
                None => self.dist_stop_range + 1.0,
                Some(s) => pose.dist(&s),
            };
            if err < self.dist_stop_range {
                let now = Instant::now();
                self.millis_in_range += now.duration_since(prev_update).as_millis() as u64;
                prev_update = now;
            } else {
                self.millis_in_range = 0;
            }

            // calculate 1st ghost
            let lead = Pose::new(
                target.x - self.dlead * err * target.h.as_rad(AngleSystem::Cartesian).cos(),
                target.y - self.dlead * err * target.h.as_rad(AngleSystem::Cartesian).sin(),
                target.h,
            );
            // calculate 2nd ghost
            // move towards ghost
            self.algorithm.approach_point(lead).await;
            sleep(Duration::from_millis(10)).await;
        }

        self.algorithm.brake(brake_mode).await;
    }
}

use core::f64::consts;

use vexide::prelude::Float;

pub fn shorter_rad(h0: f64, hf: f64) -> f64 {
    shorter_turn(h0, hf, 2.0 * consts::PI)
}

pub fn shorter_deg(h0: f64, hf: f64) -> f64 {
    shorter_turn(h0, hf, 360.0)
}

fn shorter_turn(h0: f64, hf: f64, modulo: f64) -> f64 {
    let dir = (hf % modulo - h0 % modulo) % modulo;
    let half = modulo.abs() / 2.0;
    if dir.abs() < half {
        dir
    } else if dir > 0.0 {
        dir - modulo
    } else {
        dir + modulo
    }
}

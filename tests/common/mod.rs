use std::f64::consts::PI;

pub mod graphing;

pub fn mag(v: (f64, f64)) -> f64 {
    (v.0 * v.0 + v.1 * v.1).sqrt()
}
pub fn deg_to_rad(v: f64) -> f64 {
    v * PI / 180.0
}
pub fn polar(r: f64, theta: f64) -> (f64, f64) {
    (r * theta.cos(), r * theta.sin())
}

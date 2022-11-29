use robot_algorithms::math::vec::Vec2;
use std::f64::consts::PI;

pub mod graphing;

pub fn deg_to_rad(v: f64) -> f64 {
	v * PI / 180.0
}
pub fn polar(r: f64, theta: f64) -> Vec2 {
	Vec2::new(r * theta.cos(), r * theta.sin())
}

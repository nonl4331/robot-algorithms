#![allow(unused)]

pub mod quat;
pub mod vec;

mod cmath {
	extern "C" {
		pub fn sqrt(x: f64) -> f64;
		pub fn sqrtf(x: f32) -> f32;
		pub fn abs(x: f64) -> f64;
		pub fn absf(x: f32) -> f32;
		pub fn sin(x: f64) -> f64;
		pub fn sinf(x: f32) -> f32;
		pub fn cos(x: f64) -> f64;
		pub fn cosf(x: f32) -> f32;
		pub fn tan(x: f64) -> f64;
		pub fn tanf(x: f32) -> f32;
		pub fn asin(x: f64) -> f64;
		pub fn asinf(x: f32) -> f32;
		pub fn acos(x: f64) -> f64;
		pub fn acosf(x: f32) -> f32;
		pub fn atan(x: f64) -> f64;
		pub fn atanf(x: f32) -> f32;
	}
}

pub trait FloatMath {
	fn sqrt(self) -> Self;
	fn abs(self) -> Self;
	fn sin(self) -> Self;
	fn cos(self) -> Self;
	fn tan(self) -> Self;
	fn asin(self) -> Self;
	fn acos(self) -> Self;
	fn atan(self) -> Self;
}

impl FloatMath for f64 {
	fn sqrt(self) -> Self {
		unsafe { cmath::sqrt(self) }
	}
	fn abs(self) -> Self {
		unsafe { cmath::abs(self) }
	}
	fn sin(self) -> Self {
		unsafe { cmath::sin(self) }
	}
	fn cos(self) -> Self {
		unsafe { cmath::cos(self) }
	}
	fn tan(self) -> Self {
		unsafe { cmath::tan(self) }
	}
	fn asin(self) -> Self {
		unsafe { cmath::asin(self) }
	}
	fn acos(self) -> Self {
		unsafe { cmath::acos(self) }
	}
	fn atan(self) -> Self {
		unsafe { cmath::atan(self) }
	}
}

impl FloatMath for f32 {
	fn sqrt(self) -> Self {
		unsafe { cmath::sqrtf(self) }
	}
	fn abs(self) -> Self {
		unsafe { cmath::absf(self) }
	}
	fn sin(self) -> Self {
		unsafe { cmath::sinf(self) }
	}
	fn cos(self) -> Self {
		unsafe { cmath::cosf(self) }
	}
	fn tan(self) -> Self {
		unsafe { cmath::tanf(self) }
	}
	fn asin(self) -> Self {
		unsafe { cmath::asinf(self) }
	}
	fn acos(self) -> Self {
		unsafe { cmath::acosf(self) }
	}
	fn atan(self) -> Self {
		unsafe { cmath::atanf(self) }
	}
}

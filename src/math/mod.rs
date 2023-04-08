#![allow(unused)]

pub mod quat;
pub mod vec;

mod cmath {
	extern "C" {
		pub fn abs(x: f64) -> f64;
		pub fn absf(x: f32) -> f32;
		pub fn acos(x: f64) -> f64;
		pub fn acosf(x: f32) -> f32;
		pub fn asin(x: f64) -> f64;
		pub fn asinf(x: f32) -> f32;
		pub fn atan(x: f64) -> f64;
		pub fn atan2(y: f64, x: f64) -> f64;
		pub fn atan2f(y: f32, x: f32) -> f32;
		pub fn atanf(x: f32) -> f32;
		pub fn cos(x: f64) -> f64;
		pub fn cosf(x: f32) -> f32;
		pub fn exp(x: f64) -> f64;
		pub fn expf(x: f32) -> f32;
		pub fn sin(x: f64) -> f64;
		pub fn sinf(x: f32) -> f32;
		pub fn sqrt(x: f64) -> f64;
		pub fn sqrtf(x: f32) -> f32;
		pub fn tan(x: f64) -> f64;
		pub fn tanf(x: f32) -> f32;
	}
}

pub trait FloatMath {
	fn abs(self) -> Self;
	fn acos(self) -> Self;
	fn asin(self) -> Self;
	fn atan(self) -> Self;
	fn atan2(self, x: Self) -> Self;
	fn cos(self) -> Self;
	fn exp(self) -> Self;
	fn sin(self) -> Self;
	fn sqrt(self) -> Self;
	fn tan(self) -> Self;
}

impl FloatMath for f64 {
	fn abs(self) -> Self {
		unsafe { cmath::abs(self) }
	}
	fn acos(self) -> Self {
		unsafe { cmath::acos(self) }
	}
	fn asin(self) -> Self {
		unsafe { cmath::asin(self) }
	}
	fn atan(self) -> Self {
		unsafe { cmath::atan(self) }
	}
	fn atan2(self, x: Self) -> Self {
		unsafe { cmath::atan2(self, x) }
	}
	fn cos(self) -> Self {
		unsafe { cmath::cos(self) }
	}
	fn exp(self) -> Self {
		unsafe { cmath::exp(self) }
	}
	fn sin(self) -> Self {
		unsafe { cmath::sin(self) }
	}
	fn sqrt(self) -> Self {
		unsafe { cmath::sqrt(self) }
	}
	fn tan(self) -> Self {
		unsafe { cmath::tan(self) }
	}
}

impl FloatMath for f32 {
	fn abs(self) -> Self {
		unsafe { cmath::absf(self) }
	}
	fn acos(self) -> Self {
		unsafe { cmath::acosf(self) }
	}
	fn asin(self) -> Self {
		unsafe { cmath::asinf(self) }
	}
	fn atan(self) -> Self {
		unsafe { cmath::atanf(self) }
	}
	fn atan2(self, x: Self) -> Self {
		unsafe { cmath::atan2f(self, x) }
	}
	fn cos(self) -> Self {
		unsafe { cmath::cosf(self) }
	}
	fn exp(self) -> Self {
		unsafe { cmath::expf(self) }
	}
	fn sin(self) -> Self {
		unsafe { cmath::sinf(self) }
	}
	fn sqrt(self) -> Self {
		unsafe { cmath::sqrtf(self) }
	}
	fn tan(self) -> Self {
		unsafe { cmath::tanf(self) }
	}
}

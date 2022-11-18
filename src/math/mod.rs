#![allow(unused)]

pub mod quat;
pub mod vec;

mod cmath {
	extern "C" {
		pub fn sqrt(x: f64) -> f64;
		pub fn sqrtf(x: f32) -> f32;
		pub fn abs(x: f64) -> f64;
		pub fn absf(x: f32) -> f32;
	}
}

pub trait FloatMath {
	fn sqrt(&self) -> Self;
	fn abs(&self) -> Self;
}

impl FloatMath for f64 {
	fn sqrt(&self) -> Self {
		unsafe { cmath::sqrt(*self) }
	}

	fn abs(&self) -> Self {
		unsafe { cmath::abs(*self) }
	}
}

impl FloatMath for f32 {
	fn sqrt(&self) -> Self {
		unsafe { cmath::sqrtf(*self) }
	}

	fn abs(&self) -> Self {
		unsafe { cmath::absf(*self) }
	}
}

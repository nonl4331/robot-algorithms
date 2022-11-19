#![cfg_attr(not(std), no_std)]
pub mod algorithms;
pub mod math;

pub mod prelude {
	pub use crate::math::{quat::*, vec::*, FloatMath};
}

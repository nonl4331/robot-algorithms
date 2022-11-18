#![cfg_attr(not(std), no_std)]
pub mod algorithms;
pub mod math;

mod prelude {
	pub use crate::math::*;
}

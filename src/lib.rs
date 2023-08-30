#![cfg_attr(feature = "no_std", no_std)]

#[cfg(all(feature = "std", feature = "no_std"))]
compile_error!("feature \"std\" and feature \"no_std\" cannot be enabled at the same time");

#[cfg(not(any(feature = "std", feature = "no_std")))]
compile_error!("either feature \"std\" or feature \"no_std\" must enabled but not both");

#[cfg(feature = "no_std")]
extern crate alloc;

#[cfg(all(not(feature = "no_std"), test))]
#[macro_use]
extern crate std;

pub mod path_planning;
pub mod path_tracking;

#[cfg(feature = "no_std")]
pub mod no_std_stuff {
	pub use alloc::vec::Vec;
	pub use nalgebra::{ComplexField, RealField};
}

pub mod prelude {
	use core::cmp::Ordering;

	pub use nalgebra::{Point2, Point3, Rotation2, Vector2, Vector3};

	#[derive(Debug, Copy, Clone, PartialEq)]
	pub enum Error {
		PathNotFound,
		NaNInCalculation,
	}

	pub type Vec2 = Vector2<f64>;
	pub type Vec3 = Vector3<f64>;
	pub type Pos2 = Point2<f64>;
	pub type Pos3 = Point3<f64>;

	#[cfg(feature = "no_std")]
	pub use crate::no_std_stuff::*;

	#[derive(Debug, Copy, Clone, PartialEq)]
	#[must_use]
	pub struct Ray {
		pub pos: Pos2,
		pub angle: f64,
	}
	impl Ray {
		pub const ZERO: Self = Self::new(Pos2::new(0.0, 0.0), 0.0);
		pub const fn new(pos: Pos2, angle: f64) -> Self {
			Self { pos, angle }
		}
		pub fn translated(&self, offset: Vec2) -> Self {
			Self {
				pos: self.pos + offset,
				angle: self.angle,
			}
		}
		pub fn rotated(&self, rotation: f64) -> Self {
			let rot = Rotation2::new(rotation);

			Self {
				pos: rot * self.pos,
				angle: self.angle + rotation,
			}
		}
		pub fn scale(&mut self, scale: f64) {
			self.pos *= scale;
		}
		pub fn scaled(&self, scale: f64) -> Self {
			Self {
				pos: scale * self.pos,
				angle: self.angle,
			}
		}
		pub fn ray_to_local(&self, other: Self) -> Self {
			other.translated(-self.pos.coords).rotated(-self.angle)
		}
		pub fn ray_from_local(&self, other: Self) -> Self {
			other.rotated(self.angle).translated(self.pos.coords)
		}
        #[must_use]
        pub fn at(&self, t: f64) -> Pos2 {
            self.pos + t * Vec2::new(self.angle.cos(), self.angle.sin())
        }
	}

	#[allow(clippy::float_cmp)]
	pub(crate) fn float_cmp(a: f64, b: f64) -> Ordering {
		if a < b {
			Ordering::Less
		} else if a == b {
			Ordering::Equal
		} else {
			Ordering::Greater
		}
	}
}

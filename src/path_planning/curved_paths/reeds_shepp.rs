use super::PathSegmentType::{self, Left, Nill, Right, Straight};
use crate::{path_planning::curved_paths, prelude::*};
use core::f64::consts::{FRAC_PI_2, PI, TAU};

// references:
// https://projecteuclid.org/journals/pacific-journal-of-mathematics/volume-145/issue-2/Optimal-paths-for-a-car-that-goes-both-forwards-and/pjm/1102645450.pdf
// https://ompl.kavrakilab.org/ReedsSheppStateSpace_8cpp_source.html
// --------
// notation in paper:
// {l, r, s}{p, m}
// l = left curve
// r = right curve
// s = straight
// p = plus (forward)
// m = minus (backwards)
//
// we can derive all possible optimal solutions from the 9 "words" below
//
// number references refer to sections in the paper
// lengths of segments are in order of the segments in the function name
// and are named {s1, s2, s3, s4}
// a comment above the function notates what they are refered to in the paper
// --------

pub(crate) trait ReedsSheppSegments {
	fn timeflip(self) -> Self;
	fn reflect(self) -> Self;
	fn distance(&self) -> f64;
}

impl<const N: usize> ReedsSheppSegments for [PathSegmentType; N] {
	fn timeflip(mut self) -> Self {
		self.iter_mut().for_each(|v| *v = v.timeflip());
		self
	}
	fn reflect(mut self) -> Self {
		self.iter_mut().for_each(|v| *v = v.reflect());
		self
	}
	fn distance(&self) -> f64 {
		self.iter().map(PathSegmentType::distance).sum()
	}
}

impl ReedsSheppSegments for PathSegmentType {
	fn timeflip(self) -> Self {
		match self {
			Self::Right(v) => Self::Right(-v),
			Self::Left(v) => Self::Left(-v),
			Self::Straight(v) => Self::Straight(-v),
			Self::Nill => self,
		}
	}
	fn reflect(self) -> Self {
		match self {
			Self::Right(v) => Self::Left(v),
			Self::Left(v) => Self::Right(v),
			_ => self,
		}
	}
	fn distance(&self) -> f64 {
		match self {
			Self::Right(a) | Self::Left(a) | Self::Straight(a) => (*a).abs(),
			Self::Nill => 0.0,
		}
	}
}

#[derive(PartialEq)]
#[allow(clippy::module_name_repetitions)]
pub struct ReedsSheppPath {
	distance: f64,
	segments: [PathSegmentType; 5],
}

impl ReedsSheppPath {
	pub(crate) fn new(distance: f64, segments: [PathSegmentType; 5]) -> Self {
		Self { distance, segments }
	}
}

impl PartialOrd for ReedsSheppPath {
	fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
		self.distance.partial_cmp(&other.distance)
	}
}

pub struct ReedsShepp {
	pub start: Ray,
	pub end: Ray,
	pub path: ReedsSheppPath,
	pub max_curve: f64,
}

impl ReedsShepp {
	pub fn new(start: Ray, end: Ray, max_curve: f64) -> Result<Self, Error> {
		let mut local_end = start.ray_to_local(end);
		local_end.scale(max_curve);

		let x = local_end.pos.x;
		let y = local_end.pos.y;
		let phi = local_end.angle;
		let sp = phi.sin();
		let cp = phi.cos();
		let xb = x * cp + y * sp;
		let yb = x * sp - y * cp;

		// CSC (8.1) - L+S+L+
		let w1 = gen_paths(x, y, phi, word_one, 0);
		// CSC (8.2) - L+S+R+
		let w2 = gen_paths(x, y, phi, word_two, 0);
		// C|C|C (8.3) - L+R-L+
		let w3 = gen_paths(x, y, phi, word_three, 0);
		// C|CC (8.4) - L+R-L-
		let w4 = gen_paths(x, y, phi, word_four, 0);
		// CC|C (8.4) - L-R-L+ (backwards)
		let w4b = gen_paths(xb, yb, phi, word_four, 3);
		// CC|CC (8.7) - L+R+L-R-
		let w5 = gen_paths(x, y, phi, word_five, 0);
		// C|CC|C (8.8) - L+R-L-R+
		let w6 = gen_paths(x, y, phi, word_six, 0);
		// C|CSC (8.9) - L+R-S-L-
		let w7 = gen_paths(x, y, phi, word_seven, 0);
		// CSC|C (8.9) - L-S-R-L+ (backwards)
		let w7b = gen_paths(xb, yb, phi, word_seven, 4);
		// C|CSC (8.10) - L+R-S-R-
		let w8 = gen_paths(x, y, phi, word_eight, 0);
		// CSC|C (8.10) - R-S-R-L+ (backwards)
		let w8b = gen_paths(xb, yb, phi, word_eight, 4);
		// C|CSC|C (8.11) - L+R-S-L-R+
		let w9 = gen_paths(x, y, phi, word_nine, 0);

		let option_path = [w1, w2, w3, w4, w4b, w5, w6, w7, w7b, w8, w8b, w9]
			.into_iter()
			.flatten()
			.min_by(|x, y| float_cmp(x.distance, y.distance));

		let Some(path) = option_path
		 else {
			return Err(Error::PathNotFound);
		};

		Ok(Self {
			start,
			end,
			path,
			max_curve,
		})
	}
	#[must_use]
	pub fn get_points(&self, step_size: f64) -> Vec<(Ray, PathSegmentType)> {
		curved_paths::get_points(
			self.start,
			&self.path.segments,
			1.0 / self.max_curve,
			step_size,
		)
	}
}

fn word_one(x: f64, y: f64, phi: f64) -> Option<[PathSegmentType; 5]> {
	let (s2, s1) = to_polar(x - phi.sin(), y - 1.0 + phi.cos());
	if s1 < 0.0 {
		return None;
	}
	let s3 = map_angle(phi - s1);
	if s3 < 0.0 {
		return None;
	}
	Some([Left(s1), Straight(s2), Left(s3), Nill, Nill])
}

fn word_two(x: f64, y: f64, phi: f64) -> Option<[PathSegmentType; 5]> {
	let (u1, t_1) = to_polar(x + phi.sin(), y - 1.0 - phi.cos());
	let u1_sq_minus_4 = u1 * u1 - 4.0;
	if u1_sq_minus_4 < 0.0 {
		return None;
	}
	let s2 = u1_sq_minus_4.sqrt();
	let theta = 2.0f64.atan2(s2);
	let s1 = map_angle(t_1 + theta);
	let s3 = map_angle(s1 - phi);
	// from OMPL (unoptimal paths?)
	/*if s1 < 0.0 || s3 < 0.0 {
		return None;
	}*/
	Some([Left(s1), Straight(s2), Right(s3), Nill, Nill])
}

fn word_three(x: f64, y: f64, phi: f64) -> Option<[PathSegmentType; 5]> {
	let (xi, eta) = (x - phi.sin(), y - 1.0 + phi.cos());
	let (u1, theta) = to_polar(xi, eta);
	// typo in paper u1 not u1^2
	if u1 > 4.0 {
		return None;
	}

	let s2 = -2.0 * (0.25 * u1).asin();
	let s1 = map_angle(theta + 0.5 * s2 + PI);
	let s3 = map_angle(phi - s1 + s2);
	Some([Left(s1), Right(s2), Left(s3), Nill, Nill])
}

fn word_four(x: f64, y: f64, phi: f64) -> Option<[PathSegmentType; 5]> {
	word_three(x, y, phi)
}

fn word_five(x: f64, y: f64, phi: f64) -> Option<[PathSegmentType; 5]> {
	let (xi, eta) = (x + phi.sin(), y - 1.0 - phi.cos());
	let rho = 0.25 * (2.0 + (xi * xi + eta * eta).sqrt());
	if rho > 1.0 {
		return None;
	}
	let s2 = rho.acos();
	let s3 = -s2;
	let s1 = tau(s2, s3, xi, eta);
	let s4 = omega(s1, s2, s3, phi);
	if s1 < 0.0 || s4 > -0.0 {
		return None;
	}
	Some([Left(s1), Right(s2), Left(s3), Right(s4), Nill])
}

fn word_six(x: f64, y: f64, phi: f64) -> Option<[PathSegmentType; 5]> {
	let (xi, eta) = (x + phi.sin(), y - 1.0 - phi.cos());
	let rho = 0.0625 * (20.0 - xi * xi - eta * eta);
	if !(0.0..1.0).contains(&rho) {
		return None;
	}
	let s2 = -rho.acos();
	// since rho is in [0..1] s2 cannot be less than 0
	if s2 > FRAC_PI_2 {
		return None;
	}
	let s3 = s2;
	let s1 = tau(s2, s3, xi, eta);
	let s4 = omega(s1, s2, s3, phi);
	Some([Left(s1), Right(s2), Left(s3), Right(s4), Nill])
}

fn word_seven(x: f64, y: f64, phi: f64) -> Option<[PathSegmentType; 5]> {
	let (xi, eta) = (x - phi.sin(), y - 1.0 + phi.cos()); // changed from paper?
	let (rho, theta) = to_polar(xi, eta);
	if rho < 2.0 {
		return None;
	}
	let r = (rho * rho - 4.0).sqrt();
	let s3 = 2.0 - r;
	let s1 = map_angle(theta + r.atan2(-2.0));
	let s2 = -FRAC_PI_2;
	let s4 = map_angle(phi + s2 - s1);
	if s1 < 0.0 || s3 > -0.0 || s4 > -0.0 {
		return None;
	}
	Some([Left(s1), Right(s2), Straight(s3), Left(s4), Nill])
}

fn word_eight(x: f64, y: f64, phi: f64) -> Option<[PathSegmentType; 5]> {
	let (xi, eta) = (x + phi.sin(), y - 1.0 - phi.cos());
	let (rho, theta) = to_polar(-eta, xi);
	if rho < 2.0 {
		return None;
	}
	let s1 = theta;
	let s2 = -FRAC_PI_2;
	let s3 = 2.0 - rho;
	let s4 = map_angle(s1 + FRAC_PI_2 - phi);
	Some([Left(s1), Right(s2), Straight(s3), Right(s4), Nill])
}

fn word_nine(x: f64, y: f64, phi: f64) -> Option<[PathSegmentType; 5]> {
	let (xi, eta) = (x + phi.sin(), y - 1.0 - phi.cos());
	let (rho, _) = to_polar(xi, eta);
	if rho < 2.0 {
		return None;
	}
	// typo in paper u > 0 rather than t <= 0
	let s3 = 4.0 - (rho * rho - 4.0).sqrt();
	if s3 > 0.0 {
		return None;
	}
	let s1 = map_angle(((4.0 - s3) * xi - 2.0 * eta).atan2((-2.0) * xi + (s3 - 4.0) * eta));

	let s5 = map_angle(s1 - phi);
	Some([
		Left(s1),
		Right(-FRAC_PI_2),
		Straight(s3),
		Left(-FRAC_PI_2),
		Right(s5),
	])
}

fn to_polar(x: f64, y: f64) -> (f64, f64) {
	let r_sq = x * x + y * y;
	(r_sq.sqrt(), y.atan2(x))
}

// map angle to [-pi, pi]
fn map_angle(angle: f64) -> f64 {
	let mut angle = angle % TAU;
	if angle > PI {
		angle -= TAU;
	} else if angle < -PI {
		angle += TAU;
	}
	angle
}

fn tau(u: f64, v: f64, xi: f64, eta: f64) -> f64 {
	let delta = map_angle(u - v);
	let a = u.sin() - delta.sin();
	let b = u.cos() - delta.cos() - 1.0;
	let t1 = (eta * a - xi * b).atan2(xi * a + eta * b);
	let t2 = 2.0 * (delta.cos() - v.cos() - u.cos()) + 3.0;

	if t2 < 0.0 {
		map_angle(t1 + PI)
	} else {
		map_angle(t1)
	}
}

fn omega(tau: f64, u: f64, v: f64, phi: f64) -> f64 {
	map_angle(tau - u + v - phi)
}

fn gen_paths(
	x: f64,
	y: f64,
	phi: f64,
	path: fn(f64, f64, f64) -> Option<[PathSegmentType; 5]>,
	reversed_index: usize,
) -> Option<ReedsSheppPath> {
	[
		path(x, y, phi).map(|mut v| {
			v[0..reversed_index].reverse();
			v
		}),
		path(-x, y, -phi)
			.map(|mut v| {
				v[0..reversed_index].reverse();
				v
			})
			.map(ReedsSheppSegments::timeflip),
		path(x, -y, -phi)
			.map(|mut v| {
				v[0..reversed_index].reverse();
				v
			})
			.map(ReedsSheppSegments::reflect),
		path(-x, -y, phi)
			.map(|mut v| {
				v[0..reversed_index].reverse();
				v
			})
			.map(|v| v.timeflip().reflect()),
	]
	.into_iter()
	.flatten()
	.map(|v| (v.distance(), v))
	.min_by(|x, y| float_cmp(x.0, y.0))
	.map(|v| ReedsSheppPath::new(v.0, v.1))
}

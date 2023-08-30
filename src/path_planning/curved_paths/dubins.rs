use super::{
	reeds_shepp::ReedsSheppSegments,
	Error,
	PathSegmentType::{self, Left, Right, Straight},
	Ray,
};
use crate::{path_planning::curved_paths, prelude::*};
use core::f64::consts::TAU;

fn rsr(
	(alpha, beta, dist, sa, sb, ca, cb, cab): (f64, f64, f64, f64, f64, f64, f64, f64),
) -> Option<[PathSegmentType; 3]> {
	let p_sq = 2.0 + dist * dist - (2.0 * cab) + (2.0 * dist * (sb - sa));
	if p_sq < 0.0 {
		return None;
	}
	let tmp = (ca - cb).atan2(dist - sa + sb);
	let s1 = map_to_2pi(alpha - tmp);
	let s2 = p_sq.sqrt();
	let s3 = map_to_2pi(tmp - beta);
	Some([Right(s1), Straight(s2), Right(s3)])
}

fn rsl(
	(alpha, beta, dist, sa, sb, ca, cb, cab): (f64, f64, f64, f64, f64, f64, f64, f64),
) -> Option<[PathSegmentType; 3]> {
	let p_sq = dist * dist - 2.0 + (2.0 * cab) - (2.0 * dist * (sa + sb));
	if p_sq < 0.0 {
		return None;
	}
	let s2 = p_sq.sqrt();
	let tmp = (ca + cb).atan2(dist - sa - sb) - 2.0f64.atan2(s2);
	let s1 = map_to_2pi(alpha - tmp);
	let s3 = map_to_2pi(beta - tmp);
	Some([Right(s1), Straight(s2), Left(s3)])
}

fn lsr(
	(alpha, beta, dist, sa, sb, ca, cb, cab): (f64, f64, f64, f64, f64, f64, f64, f64),
) -> Option<[PathSegmentType; 3]> {
	let p_sq = -2.0 + dist * dist + (2.0 * cab) + (2.0 * dist * (sa + sb));
	if p_sq < 0.0 {
		return None;
	}
	let s2 = p_sq.sqrt();
	let tmp = (-ca - cb).atan2(dist + sa + sb) - (-2.0f64).atan2(s2);
	let s1 = map_to_2pi(tmp - alpha);
	let s3 = map_to_2pi(tmp - map_to_2pi(beta));
	Some([Left(s1), Straight(s2), Right(s3)])
}

fn lsl(
	(alpha, beta, dist, sa, sb, ca, cb, cab): (f64, f64, f64, f64, f64, f64, f64, f64),
) -> Option<[PathSegmentType; 3]> {
	let p_sq = 2.0 + dist * dist - (2.0 * cab) + (2.0 * dist * (sa - sb));
	if p_sq < 0.0 {
		return None;
	}
	let tmp = (cb - ca).atan2(dist + sa - sb);
	let s1 = map_to_2pi(tmp - alpha);
	let s2 = p_sq.sqrt();
	let s3 = map_to_2pi(beta - tmp);
	Some([Left(s1), Straight(s2), Left(s3)])
}

fn rlr(
	(alpha, beta, dist, sa, sb, ca, cb, cab): (f64, f64, f64, f64, f64, f64, f64, f64),
) -> Option<[PathSegmentType; 3]> {
	let tmp = (6.0 - dist * dist + 2.0 * cab + 2.0 * dist * (sa - sb)) * 0.125;
	if tmp.abs() > 1.0 {
		return None;
	}
	let s2 = map_to_2pi(TAU - tmp.acos());
	let s1 = map_to_2pi(alpha - (ca - cb).atan2(dist - sa + sb) + s2 / 2.0);
	let s3 = map_to_2pi(alpha - beta - s1 + s2);
	Some([Right(s1), Left(s2), Right(s3)])
}

fn lrl(
	(alpha, beta, dist, sa, sb, ca, cb, cab): (f64, f64, f64, f64, f64, f64, f64, f64),
) -> Option<[PathSegmentType; 3]> {
	let tmp = (6.0 - dist * dist + 2.0 * cab + 2.0 * dist * (sb - sa)) * 0.125;
	if tmp.abs() > 1.0 {
		return None;
	}
	let s2 = map_to_2pi(TAU - tmp.acos());
	let s1 = map_to_2pi(-alpha - (ca - cb).atan2(dist + sa - sb) + s2 / 2.0);
	let s3 = map_to_2pi(map_to_2pi(beta) - alpha - s1 + map_to_2pi(s2));
	Some([Left(s1), Right(s2), Left(s3)])
}

#[derive(PartialEq, Debug)]
#[allow(clippy::module_name_repetitions)]
pub struct DubinsPath {
	distance: f64,
	segments: [PathSegmentType; 3],
}

impl DubinsPath {
	pub(crate) fn new(distance: f64, segments: [PathSegmentType; 3]) -> Self {
		Self { distance, segments }
	}
}

impl PartialOrd for DubinsPath {
	fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
		self.distance.partial_cmp(&other.distance)
	}
}

#[derive(Debug)]
pub struct Dubins {
	pub start: Ray,
	pub end: Ray,
	pub path: DubinsPath,
	pub max_curve: f64,
}

impl Dubins {
	pub fn new(start: Ray, end: Ray, max_curve: f64) -> Result<Self, Error> {
		let local_end = start.ray_to_local(end);

		let d = max_curve * local_end.pos.coords.magnitude();
		let theta = local_end.pos.y.atan2(local_end.pos.x) % TAU;
		let alpha = -theta % TAU;
		let beta = (local_end.angle - theta) % TAU;
		let (sa, ca) = (alpha.sin(), alpha.cos());
		let (sb, cb) = (beta.sin(), beta.cos());
		let cab = (alpha - beta).cos();

		let data = (alpha, beta, d, sa, sb, ca, cb, cab);

		let option_path = [
			rsr(data),
			rsl(data),
			lsr(data),
			lsl(data),
			rlr(data),
			lrl(data),
		]
		.into_iter()
		.flatten()
		.map(|v| (v.distance(), v))
		.min_by(|x, y| float_cmp(x.0, y.0))
		.map(|v| DubinsPath::new(v.0, v.1));

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

fn map_to_2pi(angle: f64) -> f64 {
	let val = angle % TAU;
	if val < 0.0 {
		TAU + val
	} else {
		val
	}
}

pub(crate) mod dubins;
pub(crate) mod reeds_shepp;

use crate::prelude::*;

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum PathSegmentType {
	Right(f64),
	Straight(f64),
	Left(f64),
	Nill,
}

fn get_point_value(
	st: PathSegmentType,
	r0: Ray,
	cl: f64,
	min_radius: f64,
) -> (Ray, PathSegmentType) {
	use PathSegmentType::{Left, Nill, Right, Straight};

	match st {
		Straight(_) => (
			r0.translated(Vec2::new(r0.angle.cos(), r0.angle.sin()).scale(cl * min_radius)),
			st,
		),
		// cl is normalised arc length for left and right i.e. radians
		Left(_) => {
			let l = Vec2::new(cl.sin(), 1.0 - cl.cos()) * min_radius;
			(r0.ray_from_local(Ray::new(l.into(), cl)), st)
		}
		Right(_) => {
			let l = Vec2::new(cl.sin(), cl.cos() - 1.0) * min_radius;
			(r0.ray_from_local(Ray::new(l.into(), -cl)), st)
		}
		Nill => unreachable!(),
	}
}

fn get_points_local(
	segments: &[PathSegmentType],
	min_radius: f64,
	step_size: f64,
) -> Vec<(Ray, PathSegmentType)> {
	let mut points = Vec::new();

	debug_assert_ne!(segments[0], PathSegmentType::Nill);

	points.push((Ray::ZERO, segments[0]));

	for &current_segment in segments {
		let (PathSegmentType::Right(segment_length) 
			| PathSegmentType::Left(segment_length)
			| PathSegmentType::Straight(segment_length)) = current_segment else { break; };

		if segment_length == 0.0 {
			continue;
		}

		let origin = points[points.len() - 1];
		let step_size = step_size * segment_length.signum();

		let mut current_length = step_size;

		while (current_length + step_size).abs() <= segment_length.abs() {
			let np = get_point_value(current_segment, origin.0, current_length, min_radius);
			points.push(np);

			current_length += step_size;
		}
		points.push(get_point_value(
			current_segment,
			origin.0,
			segment_length,
			min_radius,
		));
	}

	points
}
#[must_use]
pub fn get_points(
	start: Ray,
	segments: &[PathSegmentType],
	min_radius: f64,
	step_size: f64,
) -> Vec<(Ray, PathSegmentType)> {
	get_points_local(segments, min_radius, step_size)
		.into_iter()
		.map(|v| (start.ray_from_local(v.0), v.1))
		.collect()
}

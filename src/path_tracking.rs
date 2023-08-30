pub mod pure_pursuit {
	pub use crate::prelude::*;

    #[derive(Debug)]
    pub enum TrackingError {
        RobotTooFar,
        WrongOrientation,
        InvalidPath,
        InvalidCodePath,
        InvalidInput,
    }

    // find furthest point along path within the lookahead distance of the robot
	fn get_path_point(points: &[Vec2], pos: &Vec2, l_sq: f64) -> Result<usize, TrackingError> {
        if points.len() < 2 {
            return Err(TrackingError::InvalidPath);
        }

		// find point furthest along the path within LOOKAHEAD DISTANCE
		let closest = points
			.iter()
			.enumerate()
			.filter_map(|(i, p)| {
				let d_sq = (p - pos).magnitude_squared();

				(d_sq < l_sq).then_some((i, d_sq))
			})
			.reduce(|a, b| if a.0 > b.0 { a } else { b })
			.map(|v| v.0);

		if let Some(v) = closest {
			// closest point is valid use that one
			Ok(v)
		} else {
            // robot is too far from path
            // path is considered invalid
            // and should be regenerated
			Err(TrackingError::RobotTooFar)
		}
	}

    // get curvature as 1 / radius, negative means left turn
	pub fn get_curvature(points: &[Vec2], pos: &Ray, lookahead_sq: f64) -> Result<f64, TrackingError> {
        // validate lookahead
        if lookahead_sq <= 0.0 {
            return Err(TrackingError::InvalidInput);
        }

        // find point on path to follow
		let c_i = get_path_point(points, &pos.pos.coords, lookahead_sq)?;

        // get direction along the path till next path point
        let dir = if c_i + 1 == points.len() {
            // use direction to last point from the second last point
            // in the case that the closest point is the last point
            points[c_i] - points[c_i - 1]
        } else {
            points[c_i + 1] - points[c_i]
        };

		// get intersection with direction from closest path point
        // and the lookahead circle
        let goal = find_path_intersection(points[c_i], dir, pos.pos.coords, lookahead_sq)?;

		// translate intersection into local space (unit x)
        let local_goal = Rotation2::new(-pos.angle) * (goal.1 - pos.pos.coords);
        if local_goal.x < 0.0 {
            log::warn!("pure pursuit, facing wrong way!");
            return Err(TrackingError::WrongOrientation);
        }

        Ok(2.0 * -local_goal.y / lookahead_sq)
	}

    // standard ray circle intersection
    // note that there should always be exactly one intersection
    // since "seg_start" is within the lookahead circle
    fn find_path_intersection(seg_start: Vec2, dir: Vec2, pos: Vec2, l_sq: f64) -> Result<(f64, Vec2), TrackingError> {
		let oc = seg_start - pos;

		let a = dir.dot(&dir);
		let b = 2.0 * oc.dot(&dir);
		let c = oc.dot(&oc) - l_sq;

		let disc = b * b - 4.0 * a * c;
		if disc < 0.0 {
            log::error!("no solutions found in pure pursuit, this is a bug.");
            Err(TrackingError::InvalidCodePath)
		} else {
			let disc = disc.sqrt();
			let denom = 2.0 * a;
			let mut t0 = (-b - disc) / denom;
			let mut t1 = (-b + disc) / denom;
			if t1 < t0 {
				core::mem::swap(&mut t0, &mut t1);
			};

			let t = if t0 > 0.0 {
                log::error!("more than one intersection found in pure pursuit, this is a bug.");
				return Err(TrackingError::InvalidCodePath);
			} else {
				if t1 <= 0.0 {
                    log::error!("no positive intersections found in pure pursuit, this is a bug.");
					return Err(TrackingError::InvalidCodePath);
				}
				t1
			};

			let point = seg_start + dir * t;
			Ok((t, point))
		}
	}

	#[cfg(test)]
	mod tests {
		use super::*;
		#[test]
		fn circle_intersection() {
			let int = find_path_intersection(
				Vec2::new(0.0, 0.0),
				Vec2::new(0.0, 0.6),
				Vec2::new(0.0, 0.5),
                0.6,
			)
			.unwrap()
			.1;
            println!("{:?}", int);
			assert!(int.x.abs() < 1e-10 && (int.y - 0.5).abs() < 1e-10);
		}
	}
}

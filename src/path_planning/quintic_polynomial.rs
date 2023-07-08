use crate::prelude::Vec2;

#[derive(Debug, Copy, Clone)]
pub enum QuinticError {
	OutOfRange,
	InvalidStartTime,
	InvalidTimeStep,
	ValidPolynomialNotFound,
}

#[derive(Debug, Copy, Clone)]
pub struct QuinticPolynomial {
	cx: [f64; 6],
	cy: [f64; 6],
	max_t: f64,
}

impl QuinticPolynomial {
	pub fn new(
		// position, velocity, acceleration
		start: (Vec2, Vec2, Vec2),
		end: (Vec2, Vec2, Vec2),
		t1: f64,
	) -> Result<Self, QuinticError> {
		if t1 < 0.0 {
			return Err(QuinticError::InvalidStartTime);
		}
		Ok(Self {
			cx: get_coefficients(
				(start.0.x, start.1.x, start.2.x),
				(end.0.x, end.1.x, end.2.x),
				t1,
			),
			cy: get_coefficients(
				(start.0.y, start.1.y, start.2.y),
				(end.0.y, end.1.y, end.2.y),
				t1,
			),
			max_t: t1,
		})
	}

	pub fn iterative_find_optimal_new<F>(
		(x0, v0, a0): (Vec2, Vec2, Vec2),
		(x1, v1, a1): (Vec2, Vec2, Vec2),
		validator: &F,
		min_time: f64,
		max_time: f64,
		time_step: f64,
	) -> Result<Self, QuinticError>
	where
		F: Fn(&Self) -> bool,
	{
		if time_step <= 0.0 || max_time < min_time {
			return Err(QuinticError::InvalidTimeStep);
		}

		let mut t = if min_time == 0.0 { time_step } else { min_time };
		while t <= max_time {
			let p = Self::new((x0, v0, a0), (x1, v1, a1), t)?;
			if validator(&p) {
				return Ok(p);
			}
			t += time_step;
		}

		Err(QuinticError::ValidPolynomialNotFound)
	}
	#[must_use]
	pub fn velocity(&self, t: f64) -> Vec2 {
		let t_2 = t * t;
		let t_3 = t_2 * t;
		let t_4 = t_3 * t;

		Vec2::new(
			self.cx[1]
				+ 2.0 * self.cx[2] * t
				+ 3.0 * self.cx[3] * t_2
				+ 4.0 * self.cx[4] * t_3
				+ 5.0 * self.cx[5] * t_4,
			self.cy[1]
				+ 2.0 * self.cy[2] * t
				+ 3.0 * self.cy[3] * t_2
				+ 4.0 * self.cy[4] * t_3
				+ 5.0 * self.cy[5] * t_4,
		)
	}
	#[must_use]
	pub fn acceleration(&self, t: f64) -> Vec2 {
		let t_2 = t * t;
		let t_3 = t_2 * t;
		Vec2::new(
			2.0 * self.cx[2]
				+ 6.0 * self.cx[3] * t
				+ 12.0 * self.cx[4] * t_2
				+ 20.0 * self.cx[5] * t_3,
			2.0 * self.cy[2]
				+ 6.0 * self.cy[3] * t
				+ 12.0 * self.cy[4] * t_2
				+ 20.0 * self.cy[5] * t_3,
		)
	}
	#[must_use]
	pub fn jerk(&self, t: f64) -> Vec2 {
		let t_2 = t * t;
		Vec2::new(
			6.0 * self.cx[3] + 24.0 * self.cx[4] * t + 60.0 * self.cx[5] * t_2,
			6.0 * self.cy[3] + 24.0 * self.cy[4] * t + 60.0 * self.cy[5] * t_2,
		)
	}
	pub fn evaluate(&self, t: f64) -> Result<Vec2, QuinticError> {
		if t < 0.0 || t > self.max_t {
			return Err(QuinticError::OutOfRange);
		}

		Ok(self.evaluate_unchecked(t))
	}
	#[must_use]
	pub fn evaluate_unchecked(&self, t: f64) -> Vec2 {
		let t_2 = t * t;
		let t_3 = t_2 * t;
		let t_4 = t_3 * t;
		let t_5 = t_4 * t;

		Vec2::new(
			self.cx[0]
				+ self.cx[1] * t + self.cx[2] * t_2
				+ self.cx[3] * t_3
				+ self.cx[4] * t_4
				+ self.cx[5] * t_5,
			self.cy[0]
				+ self.cy[1] * t + self.cy[2] * t_2
				+ self.cy[3] * t_3
				+ self.cy[4] * t_4
				+ self.cy[5] * t_5,
		)
	}
	#[must_use]
	pub fn max_t(&self) -> f64 {
		self.max_t
	}
}

fn get_coefficients(
	(x0, v0, a0): (f64, f64, f64),
	(x1, v1, a1): (f64, f64, f64),
	t1: f64,
) -> [f64; 6] {
	let inverse_t1 = 1.0 / t1;
	let t1_sq = t1 * t1;
	let inverse_t1_squared = 1.0 / t1_sq;
	let half_a0 = 0.5 * a0;

	let j0 = (x1 - x0 - v0 * t1 - half_a0 * t1_sq) * inverse_t1_squared * inverse_t1;
	let j1 = (v1 - v0 - a0 * t1) * inverse_t1_squared;
	let j2 = 0.5 * inverse_t1 * (a1 - a0);

	[
		x0,
		v0,
		half_a0,
		10.0 * j0 - 4.0 * j1 + j2,
		(7.0 * j1 - 15.0 * j0 - 2.0 * j2) * inverse_t1,
		(j2 - 3.0 * j1 + 6.0 * j0) * inverse_t1_squared,
	]
}

#[cfg(test)]
mod tests {
	use super::*;
	use core::f64::consts::PI;

	fn deg_to_rad(v: f64) -> f64 {
		v * PI / 180.0
	}
	fn polar(r: f64, theta: f64) -> Vec2 {
		Vec2::new(r * theta.cos(), r * theta.sin())
	}

	#[test]
	fn get_coefficients_test() {
		assert_eq!(
			get_coefficients((3.0, 1.0, 0.5), (7.0, 2.0, 0.0), 2.0),
			[3.0, 1.0, 0.25, 1.125, -0.8125, 0.15625]
		);
	}

	#[test]
	fn quintic_polynomial() {
		assert!(QuinticPolynomial::new(
			(
				Vec2::new(1.0, 2.0),
				Vec2::new(3.0, -1.0),
				Vec2::new(0.0, -0.5)
			),
			(
				Vec2::new(-3.0, -1.0),
				Vec2::new(2.0, 1.0),
				Vec2::new(0.0, 0.0)
			),
			5.0
		)
		.is_ok());
		assert!(QuinticPolynomial::new(
			(
				Vec2::new(1.0, 2.0),
				Vec2::new(3.0, -1.0),
				Vec2::new(0.0, -0.5)
			),
			(
				Vec2::new(-3.0, -1.0),
				Vec2::new(2.0, 1.0),
				Vec2::new(0.0, 0.0)
			),
			-0.1
		)
		.is_err());
	}

	#[test]
	fn quintic_polynomial_evaluate() {
		let p = QuinticPolynomial::new(
			(
				Vec2::new(3.0, 2.0),
				Vec2::new(1.0, -1.0),
				Vec2::new(0.5, -0.5),
			),
			(
				Vec2::new(7.0, -1.0),
				Vec2::new(2.0, 1.0),
				Vec2::new(0.0, 0.0),
			),
			2.0,
		)
		.unwrap();

		let value = p.evaluate(1.5).unwrap();

		assert!((value.x - 5.9326171875).abs() < 1e-10);
		assert!((value.y - (-1.1435546875)).abs() < 1e-10);
		assert!(p.evaluate(2.1).is_err());
		assert!(p.evaluate(-0.1).is_err());
	}

	#[test]
	fn solved_quintic() {
		let sx = Vec2::new(15.0, 20.0);
		let ex = Vec2::new(30.0, -10.0);

		let sv = polar(0.5, deg_to_rad(60.0));
		let ev = polar(1.3, deg_to_rad(-20.0));

		let sa = polar(0.1, deg_to_rad(60.0));
		let ea = polar(0.0, deg_to_rad(-20.0));

		let t_max = 12.0;

		let graph = QuinticPolynomial::new((sx, sv, sa), (ex, ev, ea), t_max).unwrap();

		let vec_correct = |o: Vec2, e: Vec2| -> bool {
			(o.x - e.x).abs() < 0.00001 && (o.y - e.y).abs() < 0.00001
		};

		let check_correct = |expected_x, expected_v, expected_a, t| -> bool {
			vec_correct(graph.evaluate(t).unwrap(), expected_x)
				&& vec_correct(graph.velocity(t), expected_v)
				&& vec_correct(graph.acceleration(t), expected_a)
		};

		assert!(check_correct(sx, sv, sa, 0.0) && check_correct(ex, ev, ea, t_max));
	}
}

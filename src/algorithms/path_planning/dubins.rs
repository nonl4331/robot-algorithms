use crate::prelude::*;
use core::f64::consts::TAU;
use smallvec::SmallVec;

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum PathSegmentType {
	Right,
	Straight,
	Left,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum PathType {
	RSR,
	RSL,
	LSR,
	LSL,
	RLR,
	LRL,
}

impl PathType {
	pub fn get_distances(&self, alpha: f64, beta: f64, dist: f64) -> Option<(f64, f64, f64, Self)> {
		let sa = alpha.sin();
		let sb = beta.sin();
		let ca = alpha.cos();
		let cb = beta.cos();
		let cab = (alpha - beta).cos();
		let (d1, d2, d3);

		match *self {
			PathType::RSR => {
				let p_sq = 2.0 + dist * dist - (2.0 * cab) + (2.0 * dist * (sa - sb));
				if p_sq < 0.0 {
					return None;
				}
				let tmp = (ca - cb).atan2(dist - sa + sb);
				d1 = map_to_2pi(alpha - tmp);
				d2 = p_sq.sqrt();
				d3 = map_to_2pi(tmp - beta);
			}
			PathType::RSL => {
				let p_sq = dist * dist - 2.0 + (2.0 * cab) - (2.0 * dist * (sa + sb));
				if p_sq < 0.0 {
					return None;
				}
				d1 = p_sq.sqrt();
				let tmp = (ca + cb).atan2(dist - sa - sb) - 2.0.atan2(d1);
				d2 = map_to_2pi(alpha - tmp);
				d3 = map_to_2pi(beta - tmp);
			}
			PathType::LSR => {
				let p_sq = -2.0 + dist * dist + (2.0 * cab) + (2.0 * dist * (sa + sb));
				if p_sq < 0.0 {
					return None;
				}
				d1 = p_sq.sqrt();
				let tmp = (-ca - cb).atan2(dist + sa + sb) - (-2.0).atan2(d1);
				d2 = map_to_2pi(tmp - alpha);
				d3 = map_to_2pi(tmp - map_to_2pi(beta));
			}
			PathType::LSL => {
				let p_sq = 2.0 + dist * dist - (2.0 * cab) + (2.0 * dist * (sa - sb));
				if p_sq < 0.0 {
					return None;
				}
				let tmp = (cb - ca).atan2(dist + sa - sb);
				d1 = map_to_2pi(tmp - alpha);
				d2 = p_sq.sqrt();
				d3 = map_to_2pi(beta - tmp);
			}
			PathType::RLR => {
				let tmp = (6.0 - dist * dist + 2.0 * cab + 2.0 * dist * (sa - sb)) / 8.0;
				if tmp.abs() > 1.0 {
					return None;
				}
				d2 = map_to_2pi(TAU - tmp.acos());
				d1 = map_to_2pi(alpha - (ca - cb).atan2(dist - sa + sb) + d2 / 2.0);
				d3 = map_to_2pi(alpha - beta - d1 + d2);
			}
			PathType::LRL => {
				let tmp = (6.0 - dist * dist + 2.0 * cab + 2.0 * dist * (sb - sa)) / 8.0;
				if tmp.abs() > 1.0 {
					return None;
				}
				d2 = map_to_2pi(TAU - tmp.acos());
				d1 = map_to_2pi(-alpha - (ca - cb).atan2(dist + sa - sb) + d2 / 2.0);
				d3 = map_to_2pi(map_to_2pi(beta) - alpha - d1 + map_to_2pi(d2));
			}
		}
		Some((d1, d2, d3, *self))
	}
	pub fn get_path_segment_types(&self) -> [PathSegmentType; 3] {
		match self {
			PathType::RSR => [
				PathSegmentType::Right,
				PathSegmentType::Straight,
				PathSegmentType::Right,
			],
			PathType::RSL => [
				PathSegmentType::Right,
				PathSegmentType::Straight,
				PathSegmentType::Left,
			],
			PathType::LSR => [
				PathSegmentType::Left,
				PathSegmentType::Straight,
				PathSegmentType::Right,
			],
			PathType::LSL => [
				PathSegmentType::Left,
				PathSegmentType::Straight,
				PathSegmentType::Left,
			],
			PathType::RLR => [
				PathSegmentType::Right,
				PathSegmentType::Left,
				PathSegmentType::Right,
			],
			PathType::LRL => [
				PathSegmentType::Left,
				PathSegmentType::Right,
				PathSegmentType::Left,
			],
		}
	}
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Error {
	PathNotFound,
	NaNInCalculation,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct DubinsPath {
	pub path_type: PathType,
	pub start: (Vec2, f64),
	pub end: (Vec2, f64),
	pub relative_end: (Vec2, f64),
	pub distances: [f64; 3],
	pub max_curve: f64,
}

impl DubinsPath {
	pub fn new(x0: Vec2, y0: f64, x1: Vec2, y1: f64, max_curve: f64) -> Result<Self, Error> {
		let old_x1 = x1;
		let old_y1 = y1;
		let x1 = Vec2::new(
			(x1.x - x0.x) * (-y0).cos() - (x1.y - x0.y) * (-y0).sin(),
			(x1.x - x0.x) * (-y0).sin() + (x1.y - x0.y) * (-y0).cos(),
		);
		let y1 = y1 - y0;

		let d = max_curve * x1.mag();
		let theta = x1.y.atan2(x1.x) % TAU;
		let alpha = -theta % TAU;
		let beta = (y1 - theta) % TAU;

		let v = if let Some(v) = [
			PathType::RSR,
			PathType::RSL,
			PathType::LSR,
			PathType::LSL,
			PathType::RLR,
			PathType::LRL,
		]
		.into_iter()
		.filter_map(|v| v.get_distances(alpha, beta, d))
		.min_by(
			|&a, &b| match (a.0 + a.1 + a.2).partial_cmp(&(b.0 + b.1 + b.2)) {
				Some(v) => v,
				None => core::cmp::Ordering::Less,
			},
		) {
			v
		} else {
			return Err(Error::PathNotFound);
		};

		Ok(Self {
			path_type: v.3,
			start: (x0, y0),
			end: (old_x1, old_y1),
			relative_end: (x1, y1),
			distances: [v.0, v.1, v.2],
			max_curve,
		})
	}

	fn get_point_value(
		&self,
		segment_type: PathSegmentType,
		origin: (Vec2, f64),
		current_length: f64,
	) -> (Vec2, f64, PathSegmentType) {
		match segment_type {
			PathSegmentType::Straight => (
				origin.0
					+ Vec2::new(origin.1.cos(), origin.1.sin()) * current_length / self.max_curve,
				origin.1,
				segment_type,
			),
			_ => {
				let local_x = current_length.sin() / self.max_curve;
				let mut local_y = (1.0 - current_length.cos()) / self.max_curve;
				if PathSegmentType::Right == segment_type {
					local_y *= -1.0;
				}
				let diff_x = (origin.1).cos() * local_x + (-origin.1).sin() * local_y;
				let diff_y = -(-origin.1).sin() * local_x + (origin.1).cos() * local_y;
				(
					origin.0 + Vec2::new(diff_x, diff_y),
					if PathSegmentType::Left == segment_type {
						origin.1 + current_length
					} else {
						origin.1 - current_length
					},
					segment_type,
				)
			}
		}
	}

	pub fn get_points_local(&self, step_size: f64) -> SmallVec<[(Vec2, f64, PathSegmentType); 64]> {
		let mut points = SmallVec::<[(Vec2, f64, PathSegmentType); 64]>::new();
		let segments = self.path_type.get_path_segment_types();
		points.push((Vec2::zero(), 0.0, segments[0]));

		for (i, &current_segment) in segments.iter().enumerate() {
			let segment_length = self.distances[i];
			if segment_length == 0.0 {
				continue;
			}

			let origin = points[points.len() - 1];

			let mut current_length = step_size;

			while (current_length + step_size).abs() <= segment_length.abs() {
				points.push(self.get_point_value(
					current_segment,
					(origin.0, origin.1),
					current_length,
				));

				current_length += step_size;
			}
			points.push(self.get_point_value(
				current_segment,
				(origin.0, origin.1),
				segment_length,
			));
		}

		points
	}

	pub fn get_points(&self, step_size: f64) -> SmallVec<[(Vec2, f64, PathSegmentType); 64]> {
		let mut points = self.get_points_local(step_size);

		let transform = |v: (Vec2, f64, PathSegmentType)| {
			(
				Vec2::new(
					(v.0.x) * (self.start.1).cos() - (v.0.y) * (self.start.1).sin(),
					(v.0.x) * (self.start.1).sin() + (v.0.y) * (self.start.1).cos(),
				) + self.start.0,
				v.1 + self.start.1,
				v.2,
			)
		};

		for point in points.iter_mut() {
			*point = transform(*point);
		}
		points
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

#[cfg(test)]
mod tests {
	use super::*;
	use core::f64::consts::PI;

	#[test]
	fn dubin_path() {
		DubinsPath::new(
			Vec2::new(1.0, 1.0),
			PI / 4.0,
			Vec2::new(-3.0, -3.0),
			-PI / 4.0,
			1.0,
		)
		.unwrap();
	}

	#[test]
	fn get_points() {
		let path = DubinsPath::new(
			Vec2::new(1.0, 1.0),
			PI / 4.0,
			Vec2::new(-3.0, -3.0),
			-PI / 4.0,
			1.0,
		)
		.unwrap();

		let points: SmallVec<[(Vec2, f64); 64]> = path
			.get_points(0.1)
			.into_iter()
			.map(|v| (v.0, v.1))
			.collect();

		let ref_points: SmallVec<[(Vec2, f64); 64]> = SmallVec::from_slice(&[
			(Vec2::new(1.0, 1.0), core::f64::consts::FRAC_PI_4),
			(
				Vec2::new(1.067060297290399, 1.0741254745095894),
				0.8853981633974479,
			),
			(
				Vec2::new(1.126385373038269, 1.1545754889996935),
				0.9853981633974485,
			),
			(
				Vec2::new(1.177382470697, 1.2405462135187664),
				1.085398163397448,
			),
			(
				Vec2::new(1.2195420441241855, 1.3311786570055566),
				1.1853981633974477,
			),
			(
				Vec2::new(1.2524428487982429, 1.4255672500438468),
				1.2853981633974483,
			),
			(
				Vec2::new(1.2757561507544293, 1.5227688930127192),
				1.3853981633974488,
			),
			(
				Vec2::new(1.2892490111859514, 1.62181237922622),
				1.4853981633974485,
			),
			(
				Vec2::new(1.292786613891524, 1.7217080989095277),
				1.585398163397448,
			),
			(
				Vec2::new(1.2863336123142892, 1.8214579270527014),
				1.6853981633974477,
			),
			(
				Vec2::new(1.269954482712928, 1.9200651963458437),
				1.7853981633974483,
			),
			(
				Vec2::new(1.243812879936197, 2.0165446555494073),
				1.885398163397448,
			),
			(
				Vec2::new(1.2081700022377755, 2.1099323137988986),
				1.9853981633974485,
			),
			(
				Vec2::new(1.1633819814696582, 2.1992950724829456),
				2.085398163397448,
			),
			(
				Vec2::new(1.1098963247304081, 2.2837400484561767),
				2.1853981633974486,
			),
			(
				Vec2::new(1.0482474430221567, 2.362423495432466),
				2.2853981633974483,
			),
			(
				Vec2::new(0.9790513115925215, 2.4345592344189155),
				2.385398163397449,
			),
			(
				Vec2::new(0.9029993153135479, 2.4994265089564776),
				2.4853981633974485,
			),
			(
				Vec2::new(0.8208513405926077, 2.556377186680243),
				2.585398163397449,
			),
			(
				Vec2::new(0.733428182838602, 2.604842235243788),
				2.6853981633974486,
			),
			(
				Vec2::new(0.6416033453455557, 2.6443374079022797),
				2.785398163397449,
			),
			(
				Vec2::new(0.5462943115364431, 2.6744680819458835),
				2.885398163397449,
			),
			(
				Vec2::new(0.4484533777721068, 2.6949332016394316),
				2.9853981633974485,
			),
			(
				Vec2::new(0.34905813832080734, 2.7055282862718157),
				3.085398163397449,
			),
			(
				Vec2::new(0.249101717559445, 2.706147473259658),
				-3.0977871437821367,
			),
			(
				Vec2::new(0.14958284700306645, 2.696784575891253),
				-2.997787143782137,
			),
			(
				Vec2::new(0.05149588630970692, 2.6775331451421422),
				-2.8977871437821374,
			),
			(
				Vec2::new(-0.04417911203262448, 2.648585534944694),
				-2.797787143782137,
			),
			(
				Vec2::new(-0.13648619506644044, 2.6102309802511945),
				-2.6977871437821364,
			),
			(
				Vec2::new(-0.22450306093073213, 2.5628527070938096),
				-2.5977871437821367,
			),
			(
				Vec2::new(-0.30735027419629124, 2.506924103516754),
				-2.497787143782137,
			),
			(
				Vec2::new(-0.3842000528938163, 2.443003989639431),
				-2.3977871437821365,
			),
			(
				Vec2::new(-0.4542845394377204, 2.3717310341105224),
				-2.297787143782136,
			),
			(
				Vec2::new(-0.5469113582225731, 2.249995602575736),
				-2.144669500168911,
			),
			(
				Vec2::new(-0.601200240361492, 2.1660151448721336),
				-2.144669500168911,
			),
			(
				Vec2::new(-0.6554891225004107, 2.082034687168531),
				-2.144669500168911,
			),
			(
				Vec2::new(-0.7097780046393298, 1.9980542294649284),
				-2.144669500168911,
			),
			(
				Vec2::new(-0.7640668867782485, 1.914073771761326),
				-2.144669500168911,
			),
			(
				Vec2::new(-0.8183557689171674, 1.8300933140577236),
				-2.144669500168911,
			),
			(
				Vec2::new(-0.8726446510560864, 1.746112856354121),
				-2.144669500168911,
			),
			(
				Vec2::new(-0.926933533195005, 1.6621323986505185),
				-2.144669500168911,
			),
			(
				Vec2::new(-0.981222415333924, 1.578151940946916),
				-2.144669500168911,
			),
			(
				Vec2::new(-1.0355112974728433, 1.4941714832433135),
				-2.144669500168911,
			),
			(
				Vec2::new(-1.0898001796117618, 1.410191025539711),
				-2.144669500168911,
			),
			(
				Vec2::new(-1.1440890617506807, 1.3262105678361085),
				-2.144669500168911,
			),
			(
				Vec2::new(-1.1983779438895996, 1.2422301101325057),
				-2.144669500168911,
			),
			(
				Vec2::new(-1.2526668260285185, 1.1582496524289032),
				-2.144669500168911,
			),
			(
				Vec2::new(-1.3069557081674374, 1.0742691947253005),
				-2.144669500168911,
			),
			(
				Vec2::new(-1.3612445903063564, 0.990288737021698),
				-2.144669500168911,
			),
			(
				Vec2::new(-1.4155334724452748, 0.9063082793180954),
				-2.144669500168911,
			),
			(
				Vec2::new(-1.4698223545841942, 0.8223278216144925),
				-2.144669500168911,
			),
			(
				Vec2::new(-1.524111236723113, 0.73834736391089),
				-2.144669500168911,
			),
			(
				Vec2::new(-1.578400118862032, 0.6543669062072874),
				-2.144669500168911,
			),
			(
				Vec2::new(-1.6326890010009505, 0.5703864485036849),
				-2.144669500168911,
			),
			(
				Vec2::new(-1.6869778831398694, 0.4864059908000824),
				-2.144669500168911,
			),
			(
				Vec2::new(-1.7412667652787888, 0.40242553309647944),
				-2.144669500168911,
			),
			(
				Vec2::new(-1.7955556474177077, 0.3184450753928769),
				-2.144669500168911,
			),
			(
				Vec2::new(-1.8498445295566266, 0.23446461768927418),
				-2.144669500168911,
			),
			(
				Vec2::new(-1.9041334116955455, 0.15048415998567188),
				-2.144669500168911,
			),
			(
				Vec2::new(-1.9584222938344644, 0.06650370228206914),
				-2.144669500168911,
			),
			(
				Vec2::new(-2.0127111759733833, -0.017476755421533152),
				-2.144669500168911,
			),
			(
				Vec2::new(-2.0670000581123023, -0.10145721312513611),
				-2.144669500168911,
			),
			(
				Vec2::new(-2.121288940251221, -0.18543767082873885),
				-2.144669500168911,
			),
			(
				Vec2::new(-2.17557782239014, -0.26941812853234137),
				-2.144669500168911,
			),
			(
				Vec2::new(-2.229866704529059, -0.3533985862359439),
				-2.144669500168911,
			),
			(
				Vec2::new(-2.284155586667978, -0.43737904393954663),
				-2.144669500168911,
			),
			(
				Vec2::new(-2.3384444688068973, -0.5213595016431491),
				-2.144669500168911,
			),
			(
				Vec2::new(-2.3927333509458153, -0.6053399593467517),
				-2.144669500168911,
			),
			(
				Vec2::new(-2.4470222330847347, -0.6893204170503546),
				-2.144669500168911,
			),
			(
				Vec2::new(-2.5013111152236536, -0.7733008747539571),
				-2.144669500168911,
			),
			(
				Vec2::new(-2.5555999973625725, -0.8572813324575597),
				-2.144669500168911,
			),
			(
				Vec2::new(-2.6098888795014914, -0.9412617901611622),
				-2.144669500168911,
			),
			(
				Vec2::new(-2.66417776164041, -1.0252422478647643),
				-2.144669500168911,
			),
			(
				Vec2::new(-2.7184666437793292, -1.1092227055683672),
				-2.144669500168911,
			),
			(
				Vec2::new(-2.7727555259182477, -1.1932031632719697),
				-2.144669500168911,
			),
			(
				Vec2::new(-2.827044408057166, -1.2771836209755714),
				-2.144669500168911,
			),
			(
				Vec2::new(-2.8813332901960846, -1.3611640786791734),
				-2.144669500168911,
			),
			(
				Vec2::new(-2.9356221723350036, -1.445144536382776),
				-2.144669500168911,
			),
			(
				Vec2::new(-2.989911054473922, -1.529124994086378),
				-2.144669500168911,
			),
			(
				Vec2::new(-3.0441999366128405, -1.61310545178998),
				-2.144669500168911,
			),
			(
				Vec2::new(-3.132697795849478, -1.7500043974242625),
				-2.144669500168911,
			),
			(
				Vec2::new(-3.1827007168825014, -1.8365571404894814),
				-2.0446695001689106,
			),
			(
				Vec2::new(-3.223812975526071, -1.9276694428046603),
				-1.94466950016891,
			),
			(
				Vec2::new(-3.255623791681728, -2.0224309403627907),
				-1.8446695001689104,
			),
			(
				Vec2::new(-3.277815322189701, -2.119894807604256),
				-1.7446695001689108,
			),
			(
				Vec2::new(-3.290165836612702, -2.2190872177848537),
				-1.6446695001689111,
			),
			(
				Vec2::new(-3.29255193269315, -2.319017073130708),
				-1.5446695001689106,
			),
			(
				Vec2::new(-3.28494976934775, -2.418685907559622),
				-1.444669500168911,
			),
			(
				Vec2::new(-3.267435304879708, -2.5170978630241185),
				-1.3446695001689104,
			),
			(
				Vec2::new(-3.240183538028476, -2.613269639795865),
				-1.2446695001689108,
			),
			(
				Vec2::new(-3.203466759440194, -2.706240321271519),
				-1.1446695001689111,
			),
			(
				Vec2::new(-3.157651831029564, -2.795080975133798),
				-1.0446695001689115,
			),
			(
				Vec2::new(-3.103196520416864, -2.878903934936127),
				-0.94466950016891,
			),
			(
				Vec2::new(-3.000000000000001, -2.9999999999999987),
				-0.7853981633974492,
			),
		]);
		for (refp, p) in ref_points.into_iter().zip(points.into_iter().map(|v| {
			if v.1 > PI {
				(v.0, v.1 - TAU)
			} else {
				v
			}
		})) {
			assert!((refp.0.x - p.0.x).abs() <= 1e-10);
			assert!((refp.0.y - p.0.y).abs() <= 1e-10);
			assert!((refp.1 - p.1).abs() <= 1e-10);
		}
	}
}

#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct Quat {
	pub a: f32,
	pub b: f32,
	pub c: f32,
	pub d: f32,
}

#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct DQuat {
	pub a: f64,
	pub b: f64,
	pub c: f64,
	pub d: f64,
}

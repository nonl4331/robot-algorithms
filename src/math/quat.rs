#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct Quat {
    a: f32,
    b: f32,
    c: f32,
    d: f32,
}

#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct DQuat {
    a: f64,
    b: f64,
    c: f64,
    d: f64,
}

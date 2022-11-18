use crate::common::*;
use crate::graphing::*;
use robot_algorithms::algorithms::path_planning::QuinticPolynomial;

mod common;

fn get_test_graph() -> QuinticPolynomial {
    QuinticPolynomial::iterative_find_optimal_new(
        (10.0, 10.0),
        polar(1.0, deg_to_rad(10.0)),
        polar(0.1, deg_to_rad(10.0)),
        (30.0, -10.0),
        polar(1.0, deg_to_rad(20.0)),
        polar(0.1, deg_to_rad(20.0)),
        &valid,
        5.0,
        100.0,
        5.0,
    )
    .unwrap()
}

fn valid(p: &QuinticPolynomial) -> bool {
    let dt = 0.1;

    let mut t = dt;
    while t < p.max_t() {
        if mag(p.acceleration(t)) > 1.0 || mag(p.jerk(t)) > 0.5 {
            return false;
        }

        t += dt;
    }

    true
}

#[test]
fn graph_animated() {
    let graph = get_test_graph();
    animate_graph(0.0, graph.max_t(), &graph);
}

#[test]
fn graph_static() {
    let graph = get_test_graph();
    draw_graph(0.0, graph.max_t(), &graph);
}
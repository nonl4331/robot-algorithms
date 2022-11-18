use image::codecs::gif::*;
use image::*;
use plotters::backend::BitMapBackend;
use robot_algorithms::algorithms::path_planning::QuinticPolynomial;

use plotters::prelude::*;
pub fn draw_graph(min: f64, max: f64, polynomial: &QuinticPolynomial) {
    const DIM: (u32, u32) = (7680, 4320);
    let scaling_factor = (DIM.0 as f64 / 1920.0, DIM.1 as f64 / 1080.0);
    let mean_scaling_factor = (scaling_factor.0 + scaling_factor.1) * 0.5;
    let mut buffer = vec![0u8; (DIM.0 * DIM.1) as usize * 3];
    {
        let root = BitMapBackend::with_buffer(&mut buffer, DIM).into_drawing_area();
        root.fill(&WHITE).unwrap();

        let values: Vec<(f64, f64)> = (min..max)
            .step((max - min) / 100.0)
            .values()
            .map(|t| polynomial.evaluate_unchecked(t))
            .collect();

        let max_val = (
            values.iter().max_by(|a, b| a.0.total_cmp(&b.0)).unwrap().0,
            values.iter().max_by(|a, b| a.1.total_cmp(&b.1)).unwrap().1,
        );
        let min_val = (
            values.iter().min_by(|a, b| a.0.total_cmp(&b.0)).unwrap().0,
            values.iter().min_by(|a, b| a.1.total_cmp(&b.1)).unwrap().1,
        );

        let range = (max_val.0 - min_val.0, max_val.1 - min_val.1);

        let mut chart = ChartBuilder::on(&root)
            .x_label_area_size(60.0 * scaling_factor.0)
            .y_label_area_size(50.0 * scaling_factor.1)
            .right_y_label_area_size(50.0 * scaling_factor.1)
            .margin(10.0 * mean_scaling_factor)
            .caption(
                "Robot Path",
                ("sans-serif", 40.0 * mean_scaling_factor).into_font(),
            )
            .build_cartesian_2d(
                (min_val.0 - 0.05 * range.0)..(max_val.0 + 0.05 * range.0),
                (min_val.1 - 0.05 * range.1)..(max_val.1 + 0.05 * range.1),
            )
            .unwrap();

        chart
            .configure_mesh()
            .disable_x_mesh()
            .disable_y_mesh()
            .label_style(("sans-serif", 20.0 * mean_scaling_factor, &BLACK).into_text_style(&root))
            .axis_style(ShapeStyle {
                color: BLACK.into(),
                filled: true,
                stroke_width: (1.0 * mean_scaling_factor).max(2.0) as u32,
            })
            .draw()
            .unwrap();

        chart
            .draw_series(PointSeries::of_element(
                vec![values[0], *values.last().unwrap()],
                5.0 * mean_scaling_factor,
                RED,
                &|c, s, st| {
                    EmptyElement::at(c)
                        + Circle::new((0, 0), s, st.filled())
                        + Text::new(
                            format!("({:.2}, {:.2})", c.0, c.1),
                            (10, 0),
                            ("sans-serif", 15.0 * mean_scaling_factor).into_font(),
                        )
                },
            ))
            .unwrap();

        chart
            .draw_series(LineSeries::new(
                values,
                RED.stroke_width((2.0 * mean_scaling_factor).max(2.0) as u32),
            ))
            .unwrap();
    }

    let img: image::DynamicImage = RgbImage::from_raw(DIM.0, DIM.1, buffer).unwrap().into();

    let img = img.thumbnail(3840, 2160);

    img.save("out.png").unwrap();
}

pub fn animate_graph(min: f64, max: f64, polynomial: &QuinticPolynomial) {
    const DIM: (u32, u32) = (3840, 2160);
    let scaling_factor = (DIM.0 as f64 / 1920.0, DIM.1 as f64 / 1080.0);
    let mean_scaling_factor = (scaling_factor.0 + scaling_factor.1) * 0.5;

    let gif_buffer = std::fs::File::create("small.gif").unwrap();
    let mut gif_encoder = GifEncoder::new_with_speed(gif_buffer, 5);
    gif_encoder.set_repeat(Repeat::Infinite).unwrap();

    let range = max - min;

    let num_values = range as usize * 30;

    let values: Vec<(f64, f64)> = (0..=num_values)
        .into_iter()
        .map(|i| polynomial.evaluate_unchecked(min + range * i as f64 / num_values as f64))
        .collect();

    let max_val = (
        values.iter().max_by(|a, b| a.0.total_cmp(&b.0)).unwrap().0,
        values.iter().max_by(|a, b| a.1.total_cmp(&b.1)).unwrap().1,
    );
    let min_val = (
        values.iter().min_by(|a, b| a.0.total_cmp(&b.0)).unwrap().0,
        values.iter().min_by(|a, b| a.1.total_cmp(&b.1)).unwrap().1,
    );

    for t_i in 0..num_values {
        let mut buffer = vec![0u8; (DIM.0 * DIM.1) as usize * 3];
        let max_t = min + range * t_i as f64 / num_values as f64;

        let values = &values[..];

        {
            let backend = BitMapBackend::with_buffer(&mut buffer, DIM);
            let root = backend.into_drawing_area();
            root.fill(&WHITE).unwrap();

            let range = (max_val.0 - min_val.0, max_val.1 - min_val.1);

            let mut chart = ChartBuilder::on(&root)
                    .x_label_area_size(60.0 * scaling_factor.0)
                    .y_label_area_size(50.0 * scaling_factor.1)
                    .right_y_label_area_size(50.0 * scaling_factor.1)
                    .margin(10.0 * mean_scaling_factor)
                    .caption(
                        format!(
                            "Robot Path | t = {max_t:.2}s, v = ({:.2}, {:.2})m/s, a = ({:.2}, {:.2})m/s^2, j = ({:.2}, {:.2})m/s^3",
                            polynomial.velocity(max_t).0,
                            polynomial.velocity(max_t).1,
                            polynomial.acceleration(max_t).0,
                            polynomial.acceleration(max_t).1,
                            polynomial.jerk(max_t).0,
                            polynomial.jerk(max_t).1
                        ),
                        ("sans-serif", 40.0 * mean_scaling_factor).into_font(),
                    )
                    .build_cartesian_2d(
                        (min_val.0 - 0.05 * range.0)..(max_val.0 + 0.05 * range.0),
                        (min_val.1 - 0.05 * range.1)..(max_val.1 + 0.05 * range.1),
                    )
                    .unwrap();

            chart
                .configure_mesh()
                .disable_x_mesh()
                .disable_y_mesh()
                .label_style(
                    ("sans-serif", 20.0 * mean_scaling_factor, &BLACK).into_text_style(&root),
                )
                .axis_style(ShapeStyle {
                    color: BLACK.into(),
                    filled: true,
                    stroke_width: (1.0 * mean_scaling_factor).max(2.0) as u32,
                })
                .draw()
                .unwrap();

            chart
                .draw_series(PointSeries::of_element(
                    if t_i == 0 {
                        vec![values[0]]
                    } else {
                        vec![values[0], values[t_i]]
                    },
                    5.0 * mean_scaling_factor,
                    RED,
                    &|c, s, st| {
                        EmptyElement::at(c)
                            + Circle::new((0, 0), s, st.filled())
                            + Text::new(
                                format!("({:.2}, {:.2})", c.0, c.1),
                                (10, 0),
                                ("sans-serif", 15.0 * mean_scaling_factor).into_font(),
                            )
                    },
                ))
                .unwrap();

            let line_series = LineSeries::new(
                values[0..=t_i].to_vec(),
                RED.stroke_width((2.0 * mean_scaling_factor).max(2.0) as u32),
            );

            chart.draw_series(line_series).unwrap();
        }
        let img = image::imageops::thumbnail(
            &RgbImage::from_raw(DIM.0, DIM.1, buffer).unwrap(),
            1280,
            720,
        );

        let img = DynamicImage::ImageRgb8(img);
        gif_encoder
            .encode_frame(image::Frame::from_parts(
                img.into_rgba8(),
                0,
                0,
                Delay::from_numer_denom_ms(100, 3),
            ))
            .unwrap();
    }
}

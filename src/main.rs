
use std::{
    f32::consts::PI,
    ops::{Add, AddAssign, Index, Mul, Range, Sub},
};

/// datastructure for Segments on the image circle
#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
struct Segment {
    /// bisector for the Segment, radians
    /// ranges from 0..2Pi
    bisector: f32,
    /// width of the Segment, radians
    width: f32,
    /// color of the Segment
    color: bool,
}

/// datastructure to hold the Segments
/// this will be used for the snapshot and the image that is cast onto the retina
#[derive(Clone, PartialEq, PartialOrd, Debug)]
struct Image {
    /// the segments that make up the image circle
    segments: Vec<Segment>,
}

/// datastructure for 2d vectors
#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
struct Vec2<T> {
    /// data for the 2d vector
    data: [T; 2],
}

/// bee struct to hold information about the snapshot and its position
#[derive(Clone, PartialEq, PartialOrd, Debug)]
struct Bee {
    /// snapshot of all obstacles
    snapshot: Image,
    /// position of the bee
    position: Vec2<i32>,
}

/// trait for obstacles
/// all obstacles will have to implement this trait
trait Obstacle {
    /// maps the obstacle from a position to a Segment
    fn map(&self, position: Vec2<i32>) -> Option<Segment>;
}

/// obstacle struct for circular objects
#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
struct Circle {
    /// center of the circle
    position: Vec2<f32>,
    /// radius of the circle
    radius: f32,
}

/// Grid struct for all your grid needs
/// origin of the Grid is always (0,0)
#[derive(Clone, PartialEq, Eq, Debug)]
struct Grid {
    /// width of the grid
    width: Range<i32>,
    /// height of the grid
    height: Range<i32>,
}

/// VectorField struct for storing all generated vectors
#[derive(Clone, PartialEq, Debug)]
struct VectorField {
    grid: Grid,
    vectors: Vec<Vec<Vec2<f32>>>,
    avg_angular_error: f32,
}

/// World that holds obstacles and the grid the bee is allowed to be on
struct World {
    /// list of obstacles in the world
    /// NOTE: the obstacles do not have to be on the grid
    obstacles: Vec<Box<dyn Obstacle>>,
    /// the grid that allows the bee to move
    grid: Grid,
}

// ------------------------------ Boilerplate Implementations -------------------------------//

impl<T> Vec2<T>
where
    Vec2<T>: Into<Vec2<f32>>,
    Vec2<T>: Clone,
{
    fn len(&self) -> f32 {
        let vec: Vec2<f32> = (*self).clone().into();
        (vec[0] * vec[0] + vec[1] * vec[1]).sqrt()
    }
    fn normalized(&self) -> Vec2<f32> {
        let vec: Vec2<f32> = (*self).clone().into();
        let len = vec.len();
        Vec2::<f32>::new(vec[0] / len, vec[1] / len)
    }
}
impl<T> Vec2<T> {
    fn new(x: T, y: T) -> Vec2<T> {
        Vec2 { data: [x, y] }
    }
}

impl<T> Index<usize> for Vec2<T> {
    type Output = T;

    fn index(&self, index: usize) -> &Self::Output {
        &self.data[index]
    }
}

impl Into<Vec2<f32>> for Vec2<i32> {
    fn into(self) -> Vec2<f32> {
        Vec2::<f32> {
            data: [self.data[0] as f32, self.data[1] as f32],
        }
    }
}

impl<T> Sub for Vec2<T>
where
    T: Sub<Output = T>,
    T: Copy,
{
    type Output = Vec2<T>;

    fn sub(self, rhs: Vec2<T>) -> Self::Output {
        Vec2::<T> {
            data: [self[0] - rhs[0], self[1] - rhs[1]],
        }
    }
}

impl<T> Add for Vec2<T>
where
    T: Add<Output = T>,
    T: Copy,
{
    type Output = Vec2<T>;

    fn add(self, rhs: Vec2<T>) -> Self::Output {
        Vec2::<T> {
            data: [self[0] + rhs[0], self[1] + rhs[1]],
        }
    }
}

impl<T> AddAssign for Vec2<T>
where
    T: Add<Output = T>,
    T: Copy,
{
    fn add_assign(&mut self, rhs: Self) {
        self.data[0] = self[0] + rhs[0];
        self.data[1] = self[1] + rhs[1];
    }
}

impl Mul<Vec2<f32>> for f32 {
    type Output = Vec2<f32>;

    fn mul(self, rhs: Vec2<f32>) -> Self::Output {
        Vec2::<f32>::new(self * rhs[0], self * rhs[1])
    }
}

trait Distance {
    fn dist(&self, other: Self) -> f32;
}

impl Distance for f32 {
    fn dist(&self, other: Self) -> f32 {
        (other - self)
            .sin()
            .atan2((other - self).cos())
    }
}

impl Distance for Segment {
    fn dist(&self, other: Self) -> f32 {
        self.bisector.dist(other.bisector)
    }
}

// -------------------------- Algorithm Implementations ---------------------------- //

impl Bee {
    fn new(world: &World, home_position: Vec2<i32>) -> Bee {
        let snapshot = Image::new(home_position, &world.obstacles);
        Bee {
            snapshot,
            position: home_position,
        }
    }
    fn home(&self, world: &World) -> Vec2<f32> {
        // take retina image
        let retinal_image = Image::new(self.position, &world.obstacles);
        // generate matched segments
        // loop over every segment on the snapshot:
        let matched = self
            .snapshot
            .segments
            .iter()
            .map(|snapshot_segment| {
                // find first element with the same sign
                let mut best_match_so_far = *retinal_image
                    .segments
                    .iter()
                    .find_map(|s| {
                        if s.color == snapshot_segment.color {
                            Some(s)
                        } else {
                            None
                        }
                    })
                    .unwrap();
                // check all other segments with the same sign:
                for retinal_segment in retinal_image.segments.clone() {
                    if snapshot_segment.dist(retinal_segment).abs()
                        < snapshot_segment.dist(best_match_so_far).abs()
                        && retinal_segment.color == snapshot_segment.color
                    {
                        best_match_so_far = retinal_segment;
                    }
                }
                // save the tuple of matched segments
                (*snapshot_segment, best_match_so_far)
            })
            .collect::<Vec<_>>();
        // generate turning vector
        let mut turning_vec = Vec2::<f32>::new(0.0, 0.0);
        matched.iter().for_each(|(snap_segment, ret_segment)| {
            // get angular difference
            let mut diff = if ret_segment.dist(*snap_segment) < 0.0 {
                -1.0 // point clockwise
            } else {
                1.0 // point counter clockwise
            };

            if ret_segment.width > PI {
                diff = -diff;
            }

            // generate the vector
            let vec = Vec2::<f32>::new(
                (ret_segment.bisector - PI / 2.0).cos() * diff,
                (ret_segment.bisector - PI / 2.0).sin() * diff,
            );
            // return the vector but normalized
            turning_vec += vec.normalized();
        });
        // generate positioning vector
        let mut positioning_vec = Vec2::<f32>::new(0.0, 0.0);
        matched.iter().for_each(|(snap_segment, ret_segment)| {
            // get size difference
            let diff = if snap_segment.width > ret_segment.width {
                1.0 // point away from the retinal bisector
            } else {
                -1.0 // point towards the center of the retina from the bisector
            };
            // generate the vector
            let vec = Vec2::<f32>::new(
                ret_segment.bisector.cos() * diff,
                ret_segment.bisector.sin() * diff,
            );
            // return the vector but normalized
            positioning_vec += vec.normalized()
        });
        // generate homing vector
        let final_vec = turning_vec + 3.0 * positioning_vec;
        final_vec.normalized()
    }
}

impl Obstacle for Circle {
    fn map(&self, position: Vec2<i32>) -> Option<Segment> {
        // turn the position vector from i32 to f32
        let position: Vec2<f32> = position.into();
        // get a vector from the origin
        let vec = self.position - position;

        // check whether the position is inside the obstacle
        if vec.len() >= self.radius {
            // get the angle of the vector to the x-axis
            // this gives the bisector of the segment
            let mut bisector = vec[1].atan2(vec[0]);
            if bisector < 0.0 {
                bisector = 2.0 * PI + bisector;
            }
            // calculate the width of the segment
            let width = (self.radius / vec.len()).asin() * 2.0;

            Some(Segment {
                bisector,
                width,
                // color black:
                color: true,
            })
        } else {
            // if it is in the obstacle return nothing
            None
        }
    }
}

impl Image {
    fn new(position: Vec2<i32>, obstacles: &Vec<Box<dyn Obstacle>>) -> Image {
        // create a new list of segments
        let mut segments: Vec<Segment> = Vec::new();
        // iterate over the obstacles
        for obstacle in obstacles {
            // map the obstacle onto a segment
            match obstacle.map(position) {
                // push the segment if it exists
                Some(segment) => segments.push(segment),
                // if the segment doesnt exist: do nothing
                None => {}
            }
        }

        // check for overlapping segments and merge them
        let mut merged_segments: Vec<Segment> = Vec::new();
        while {
            // merge the first segment with every other segment that collides
            // also return all separated segments
            let (merged, separated) = Segment::merge(segments[0], segments[1..].to_vec());
            // put the merged segment into the list with all the merged segments
            merged_segments.push(merged);
            // make the separated list the new segment list
            // since the Segment::merge function will return the check_with Segment as
            // the merged segment when there is no colliding segment,
            // it wont be lost
            segments = separated;

            // do while part:
            !segments.is_empty()
        } {}

        // sort the segments
        merged_segments.sort_unstable_by(|a, b| a.bisector.partial_cmp(&b.bisector).unwrap());

        let mut final_segments = merged_segments.clone();
        // fill the spaces between the segments
        for (i, segment) in merged_segments.iter().enumerate() {
            let self_edge = segment.bisector - (segment.width / 2.0);

            let edge_prev = if i == 0 {
                // if it is the first segment put a white segment before it
                let last = merged_segments.iter().last().unwrap();
                let mut out = last.bisector + (last.width / 2.0);
                out -= 2.0 * PI;
                out
            } else {
                // end edge of the previous segment
                merged_segments[i - 1].bisector + (merged_segments[i - 1].width / 2.0)
            };

            // calculate the bisector and width for the new segment:
            let mut new_bisector = (edge_prev + self_edge) / 2.0;
            let new_width = self_edge - edge_prev;

            // if the bisector is negative add 2pi onto it
            if new_bisector < 0.0 {
                new_bisector += 2.0 * PI;
            }

            // construct the segment
            let new_segment = Segment {
                bisector: new_bisector,
                width: new_width,
                color: false,
            };

            // add the struct to the list of segments
            final_segments.push(new_segment);
        }

        // sort the segments for easier use later on
        // this isn't necessary but makes life easier
        final_segments.sort_unstable_by(|a, b| a.bisector.partial_cmp(&b.bisector).unwrap());

        Image {
            segments: final_segments,
        }
    }
}

impl Segment {
    /// merges the check_with segment with the other given segments
    /// returns the merged segment and a list of segments that do not collide with the check_with segment
    fn merge(check_with: Segment, segments: Vec<Segment>) -> (Segment, Vec<Segment>) {
        // put check_with into the merged variable since it will be lost otherwise
        let mut merged = check_with;
        // create new list for all segments that dont collide with the check_with segment
        let mut separated: Vec<Segment> = Vec::new();
        // loop over every segment in the provided segments
        for segment in segments {
            // check if the segment collides with the check_with segment
            if merged.collides(segment) {
                // if it collides merge both together
                let mut merged_edges = (
                    merged.bisector - merged.width / 2.0, // start segment
                    merged.bisector + merged.width / 2.0, // end segment
                );
                let other_edges = (
                    segment.bisector - segment.width / 2.0, // start segment
                    segment.bisector + segment.width / 2.0, // end segment
                );

                // merge the edges together
                if other_edges.0 > merged_edges.1
                    && other_edges.0 > merged_edges.0
                    && other_edges.1 > merged_edges.1
                {
                    merged_edges.1 = other_edges.1;
                } else if other_edges.0 < merged_edges.1
                    && other_edges.0 < merged_edges.0
                    && other_edges.1 < merged_edges.1
                {
                    merged_edges.0 = other_edges.0;
                } else if other_edges.0 < merged_edges.0 && other_edges.1 > merged_edges.1 {
                    merged_edges = other_edges;
                }

                // turn edges into Segment
                let bisector = (merged_edges.0 + merged_edges.1) / 2.0;
                let width = merged_edges.1 - merged_edges.0;

                merged.bisector = bisector;
                merged.width = width;
            } else {
                // if it doesnt collide put it into the list of seperated segments
                separated.push(segment);
            }
        }
        (merged, separated)
    }
    /// checks if two segments collide/overlap
    /// returns true if the two segments collide/overlap
    fn collides(&self, other: Segment) -> bool {
        // calculate the distance between both bisectors
        let arc_dist = self.dist(other).abs();
        // if the arc_dist is greater than the sum of the halves of either width there is no intersection/overlap
        if arc_dist > self.width / 2.0 + other.width / 2.0 {
            false
        } else {
            true
        }
    }
}

impl VectorField {
    fn generate(mut bee: Bee, world: &World) -> VectorField {
        // clone the world grid
        let grid = world.grid.clone();
        // generate the data storage for the vectors
        let mut field =
            vec![
                vec![Vec2::<f32>::new(0.0, 0.0); (grid.width.end - grid.width.start) as usize];
                (grid.height.end - grid.height.start) as usize
            ];

        let mut out = VectorField {
            grid: grid,
            vectors: vec![],
            avg_angular_error: 0.0,
        };

        let num_vecs = (out.grid.width.end - out.grid.width.start) * (out.grid.height.end - out.grid.height.start);

        for y in (out.grid.clone()).height {
            for x in (out.grid.clone()).width {
                // calculate indices for storing
                let index = out.index(Vec2::<i32>::new(x, y));
                // position the bee correctly
                bee.position = Vec2::<i32>::new(x, y);
                // generate the homing vector
                let homing_vector = bee.home(world);

                // calculate the angular error of the generated vector
                let correct = Vec2::<f32>::new(0.0,0.0) - bee.position.clone().into();
                let dot = correct[0] * homing_vector[0] + correct[1] * homing_vector[1];
                let angle = (dot / (correct.len() * homing_vector.len())).acos();

                // save the average angular error
                if !angle.is_nan() {
                    out.avg_angular_error += angle / num_vecs as f32;
                }

                // store the homing vector
                field[index[0]][index[1]] = homing_vector;

                
            }
        }

        out.vectors = field;
        out
    }
    fn draw(&self, path: &str) -> Result<(), Box<dyn std::error::Error>> {
        use plotters::coord::types::RangedCoordf32;
        use plotters::prelude::*;
        extern crate plotters;

        let root = BitMapBackend::new(path, (640, 740)).into_drawing_area();

        root.fill(&RGBColor(240, 240, 240))?;

        let root = root.apply_coord_spec(Cartesian2d::<RangedCoordf32, RangedCoordf32>::new(
            -7f32..7f32,
            7f32..-7f32,
            (20..620, 20..620),
        ));

        let vector = |x: f32, y: f32, vec: Vec2<f32>| {
            let angle = vec[1].atan2(vec[0]) as f32;
            let arrow = vec![
                (-17, -1),
                (6, -1),
                (5, -3),
                (17, 0),
                (5, 3),
                (6, 1),
                (-17, 1),
            ];
            let rotated = arrow
                .iter()
                .map(|(x, y)| {
                    let x = *x as f32;
                    let y = *y as f32;
                    let new_x = (x * angle.cos()) - (y * angle.sin());
                    let new_y = -((y * angle.cos()) + (x * angle.sin()));
                    (new_x as i32, new_y as i32)
                })
                .collect::<Vec<_>>();
            return EmptyElement::at((x, y))
                + Polygon::new(rotated, ShapeStyle::from(&BLACK).filled());
        };

        root.draw(&Circle::new(
            (3.5, 2.0),
            20,
            ShapeStyle::from(&BLACK).filled(),
        ))?;
        root.draw(&Circle::new(
            (3.5, -2.0),
            20,
            ShapeStyle::from(&BLACK).filled(),
        ))?;
        root.draw(&Circle::new(
            (0.0, -4.0),
            20,
            ShapeStyle::from(&BLACK).filled(),
        ))?;

        root.draw(&Text::new(
            format!("average angular error: {}°",self.avg_angular_error * 180.0 / PI),
            (-3.0,-8.0), 
            ("sans-serif", 22.0).into_font()
        ))?;

        for y in (self.grid.clone()).height {
            for x in (self.grid.clone()).width {
                let index = self.index(Vec2::<i32>::new(x, y));
                if x == 0 && y == 0 {
                    root.draw(&Cross::new(
                        (0.0, 0.0),
                        10,
                        ShapeStyle::from(&BLACK).stroke_width(3),
                    ))
                    .unwrap();
                } else {
                    root.draw(&vector(
                        x as f32,
                        y as f32,
                        self.vectors[index[0]][index[1]],
                    ))
                    .unwrap();
                }
            }
        }
        root.present()?;
        Ok(())
    }
    fn index(&self, position: Vec2<i32>) -> Vec2<usize> {
        let height_total = self.grid.height.end - self.grid.height.start;
        let x = position[0] - self.grid.width.start;
        let y = height_total - (position[1] - self.grid.height.start) - 1; // has to be reversed

        Vec2::<usize>::new(x as usize, y as usize)
    }
}

fn main() {
    // constructing the world
    let obstacle1 = Circle {
        position: Vec2::<f32>::new(3.5, 2.0),
        radius: 0.5,
    };
    let obstacle2 = Circle {
        position: Vec2::<f32>::new(3.5, -2.0),
        radius: 0.5,
    };
    let obstacle3 = Circle {
        position: Vec2::<f32>::new(0.0, -4.0),
        radius: 0.5,
    };
    let grid = Grid {
        width: -7..8,
        height: -7..8,
    };
    let world = World {
        obstacles: vec![
            Box::new(obstacle1),
            Box::new(obstacle2),
            Box::new(obstacle3),
        ],
        grid: grid,
    };

    let bee = Bee::new(&world, Vec2::<i32>::new(0, 0));

    // generating every vector
    let vec_field = VectorField::generate(bee, &world);
    vec_field.draw("homing.png").unwrap();
}

#[test]
fn segment_map_test() {
    let circle = Circle {
        position: Vec2::<f32>::new(-1.0, 1.0),
        radius: 0.5,
    };

    let segment = circle.map(Vec2::<i32>::new(0, 0)).unwrap();

    println!("{:?}", segment);

    assert!((segment.bisector - (PI / 4.0) * 3.0).abs() < 0.01);
}

#[test]
fn image_new_test() {
    let circle1 = Circle {
        position: Vec2::<f32>::new(3.5, 2.0),
        radius: 0.5,
    };
    let circle2 = Circle {
        position: Vec2::<f32>::new(3.5, -2.0),
        radius: 0.5,
    };
    let circle3 = Circle {
        position: Vec2::<f32>::new(0.0, -4.0),
        radius: 0.5,
    };

    let obstacles: Vec<Box<dyn Obstacle>> =
        vec![Box::new(circle1), Box::new(circle2), Box::new(circle3)];

    let image = Image::new(Vec2::<i32>::new(0, 0), &obstacles);

    println!("{:?}", image);

    let mut sum = 0.0;
    for segment in image.segments {
        sum += segment.width;
    }

    println!("total size: {}", sum);

    assert!((sum - (PI * 2.0)).abs() < 0.01);
}

#[test]
fn segment_collide_test() {
    let s1 = Segment {
        bisector: PI / 4.0,
        width: PI / 2.0,
        color: true,
    };
    let s2 = Segment {
        bisector: 5.0 * PI / 4.0,
        width: PI / 2.0,
        color: true,
    };
    let s3 = Segment {
        bisector: 7.0 * PI / 4.0,
        width: PI / 2.0,
        color: true,
    };

    assert!(!s1.collides(s2));
    assert!(s1.collides(s3));
}

/// this test will always pass if the program doesnt crash
#[test]
fn help() {
    // constructing the world
    let obstacle1 = Circle {
        position: Vec2::<f32>::new(3.5, 2.0),
        radius: 0.5,
    };
    let obstacle2 = Circle {
        position: Vec2::<f32>::new(3.5, -2.0),
        radius: 0.5,
    };
    let obstacle3 = Circle {
        position: Vec2::<f32>::new(0.0, -4.0),
        radius: 0.5,
    };
    let grid = Grid {
        width: -7..8,
        height: -7..8,
    };
    let world = World {
        obstacles: vec![
            Box::new(obstacle1),
            Box::new(obstacle2),
            Box::new(obstacle3),
        ],
        grid: grid,
    };

    let mut bee = Bee::new(&world, Vec2::<i32>::new(0, 0));

    bee.position = Vec2::<i32>::new(5, -5);

    let out = bee.home(&world);
    println!("{:?}", out);
}

#[test]
fn i_dont_know_what_im_doing() {
    let s1 = Segment {
        bisector: 0.5,
        width: 1.0,
        color: true,
    };
    let s2 = Segment {
        bisector: 0.3,
        width: 1.0,
        color: true,
    };

    let test = s1.dist(s2);

    assert!(test > 0.0)
}

/// this test will always pass if the program doesnt crash
#[test]
fn vec_field_test() {
    let vec_q1 = Vec2::<f32>::new(-1.0, -1.0).normalized();
    let vec_q2 = Vec2::<f32>::new(1.0, -1.0).normalized();
    let vec_q3 = Vec2::<f32>::new(1.0, 1.0).normalized();
    let vec_q4 = Vec2::<f32>::new(-1.0, 1.0).normalized();

    let pos_q1 = Vec2::<i32>::new(7, 7);
    let pos_q2 = Vec2::<i32>::new(-6, 6);
    let pos_q3 = Vec2::<i32>::new(-5, -5);
    let pos_q4 = Vec2::<i32>::new(4, -4);

    let grid = Grid {
        width: -7..8,
        height: -7..8,
    };

    let field = vec![
        vec![Vec2::<f32>::new(0.0, 0.0); (grid.width.end - grid.width.start) as usize];
        (grid.height.end - grid.height.start) as usize
    ];

    let mut vector_field = VectorField {
        grid,
        vectors: field,
        avg_angular_error: 0.0
    };

    let index_1 = vector_field.index(pos_q1);
    let index_2 = vector_field.index(pos_q2);
    let index_3 = vector_field.index(pos_q3);
    let index_4 = vector_field.index(pos_q4);

    vector_field.vectors[index_1[0]][index_1[1]] = vec_q1;
    vector_field.vectors[index_2[0]][index_2[1]] = vec_q2;
    vector_field.vectors[index_3[0]][index_3[1]] = vec_q3;
    vector_field.vectors[index_4[0]][index_4[1]] = vec_q4;

    vector_field.draw("test.png").unwrap();
}

use std::{ops::{Range, Sub, Index, Add, AddAssign, Mul}, f32::consts::PI, iter::Sum, process::Output};


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
    data: [T; 2]
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

impl Segment {
    fn dist(&self, other: Segment) -> f32 {
        (self.bisector-other.bisector).sin().atan2((self.bisector-other.bisector).cos())
    }
}

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
        Vec2::<f32>::new(
            vec[0] / len,
            vec[1] / len
        )
    }
}
impl<T> Vec2<T> {
fn new(x: T, y: T) -> Vec2<T> {
    Vec2 { 
        data: [
            x,
            y
            ]
        }
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
            data: [self.data[0] as f32, self.data[1] as f32]
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
            data: [
                self[0] - rhs[0],
                self[1] - rhs[1]
            ]
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
            data: [
                self[0] + rhs[0],
                self[1] + rhs[1]
            ]
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
    type Output = Vec2::<f32>;

    fn mul(self, rhs: Vec2<f32>) -> Self::Output {
        Vec2::<f32>::new(
            self * rhs[0],
            self * rhs[1]
        )
    }
}

// -------------------------- Algorithm Implementations ---------------------------- //

impl Bee {
    fn new(world: &World, home_position: Vec2<i32>) -> Bee {
        let snapshot = Image::new(home_position, &world.obstacles);
        Bee { snapshot, position: home_position }
    }
    fn home(&self, world: &World) -> Vec2<f32> {
        // take retina image
        let retinal_image = Image::new(self.position, &world.obstacles);
        // generate matched segments
        // loop over every segment on the snapshot:
        let matched = self.snapshot.segments.iter().map(|snapshot_segment| { 
            // find first element with the same sign
            let mut best_match_so_far = *retinal_image.segments.iter().find_map(|s| {
                if s.color == snapshot_segment.color {
                    Some(s)
                } else {
                    None
                }
            }).unwrap();
            // check all other segments with the same sign:
            for retinal_segment in retinal_image.segments.clone() {
                if snapshot_segment.dist(retinal_segment).abs() < snapshot_segment.dist(best_match_so_far).abs() && retinal_segment.color == snapshot_segment.color {
                    best_match_so_far = retinal_segment;
                }
            }
            // save the tuple of matched segments
            (*snapshot_segment, best_match_so_far)
        }).collect::<Vec<_>>();
        // generate turning vector
        let mut turning_vec = Vec2::<f32>::new(0.0, 0.0);
        matched.iter().for_each(|(snap_segment, ret_segment)| {
            // get angular difference
            let diff = if snap_segment.dist(*ret_segment) > 0.0 || (ret_segment.width > PI) {
                -1.0
            } else {
                1.0
            };
            // generate the vector
            let vec = Vec2::<f32>::new(
                (ret_segment.bisector + PI/2.0).cos() * diff, 
                (ret_segment.bisector + PI/2.0).sin() * diff,
            );
            // return the vector but normalized
            turning_vec += vec.normalized();
        });
        // generate positioning vector
        let mut positioning_vec = Vec2::<f32>::new(0.0, 0.0);
        matched.iter().for_each(|(snap_segment, ret_segment)| {
            // get size difference
            let diff = if snap_segment.width < ret_segment.width  {
                1.0
            } else {
                -1.0
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
        let final_vec = turning_vec;
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
                bisector = 2.0*PI + bisector;
            }
            // calculate the width of the segment
            let width = (self.radius/vec.len()).asin() * 2.0;
            
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
    fn new(position: Vec2<i32>, obstacles: &Vec<Box<dyn Obstacle>> ) -> Image {
        // create a new list of segments
        let mut segments: Vec<Segment> = Vec::new();
        // iterate over the obstacles
        for obstacle in obstacles {
            // map the obstacle onto a segment
            match obstacle.map(position) {
                // push the segment if it exists
                Some(segment) => segments.push(segment),
                // if the segment doesnt exist: do nothing
                None => {},
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
        merged_segments.sort_unstable_by(|a,b| a.bisector.partial_cmp(&b.bisector).unwrap());

        let mut final_segments = merged_segments.clone();
        // fill the spaces between the segments
        for (i, segment) in merged_segments.iter().enumerate() {
            let self_edge = segment.bisector - (segment.width/2.0);

            let edge_prev = if i == 0 {
                // if it is the first segment put a white segment before it
                let last = merged_segments.iter().last().unwrap();
                let mut out = last.bisector + (last.width/2.0);
                out -= 2.0*PI;
                out
            } else {
                // end edge of the previous segment
                merged_segments[i-1].bisector + (merged_segments[i-1].width/2.0)
            };

            // calculate the bisector and width for the new segment:
            let mut new_bisector = (edge_prev + self_edge) / 2.0;
            let new_width = self_edge - edge_prev;

            // if the bisector is negative add 2pi onto it
            if new_bisector < 0.0 {
                new_bisector += 2.0*PI;
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
        final_segments.sort_unstable_by(|a,b| a.bisector.partial_cmp(&b.bisector).unwrap());

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
                    merged.bisector - merged.width/2.0, // start segment
                    merged.bisector + merged.width/2.0, // end segment
                );
                let other_edges = (
                    segment.bisector - segment.width/2.0, // start segment
                    segment.bisector + segment.width/2.0, // end segment
                );

                // merge the edges together
                if other_edges.0 > merged_edges.1 && other_edges.0 > merged_edges.0 && other_edges.1 > merged_edges.1 {
                    merged_edges.1 = other_edges.1;
                } else if other_edges.0 < merged_edges.1 && other_edges.0 < merged_edges.0 && other_edges.1 < merged_edges.1 {
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
        let arc_dist = self.dist(other);
        // if the arc_dist is greater than the sum of the halves of either width there is no intersection/overlap
        if arc_dist > self.width/2.0 + other.width/2.0 {
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
        let mut field = vec![vec![Vec2::<f32>::new(0.0, 0.0);(grid.width.end - grid.width.start) as usize]; (grid.height.end - grid.height.start) as usize];

        for y in (grid.clone()).height {
            for x in (grid.clone()).width {
                // calculate indices for storing
                let index_x = (x - grid.width.start) as usize;
                let index_y = (y - grid.height.start) as usize;
                // position the bee correctly
                bee.position = Vec2::<i32>::new(x, y);
                // generate the homing vector
                let homing_vector = bee.home(world);
                // store the homing vector
                field[index_x][index_y] = homing_vector;
            }
        }

        VectorField {
            grid: grid,
            vectors: field,
        }
    }
    fn draw(&self, path: &str) -> Result<(), Box<dyn std::error::Error>> {
        use plotters::prelude::*;
        use plotters::coord::types::RangedCoordf32;

        let root = BitMapBackend::new(path, (640, 640)).into_drawing_area();

        root.fill(&RGBColor(240, 240, 240))?;
    
        let root = root.apply_coord_spec(Cartesian2d::<RangedCoordf32, RangedCoordf32>::new(
            -7f32..7f32,
            7f32..-7f32,
            (20..620, 20..620),
        ));
    
        let vector = |x: f32, y: f32, vec: Vec2::<f32>| {
            let angle = vec[1].atan2(vec[0]) as f32; 
            let arrow = vec![(-17,-1), (6,-1), (5, -3), (17, 0), (5,3), (6,1), (-17, 1)];
            let rotated = arrow.iter().map(|(x,y)| {
                let x = *x as f32;
                let y = *y as f32;
                let new_x = (x * angle.cos()) - (y * angle.sin());
                let new_y = - ((y * angle.cos()) + (x * angle.sin()));
                (new_x as i32, new_y as i32)
            }).collect::<Vec<_>>();
            return EmptyElement::at((x, y))
                + Polygon::new(rotated, ShapeStyle::from(&BLACK).filled());
        };
    
        for (y,e) in self.vectors.iter().enumerate() {
            for (x,vec) in e.iter().enumerate() {
                root.draw(&vector(x as f32 + (self.grid.width.start as f32) , y as f32 + (self.grid.width.start as f32) , *vec)).unwrap();
            }
        }
        root.present()?;
        Ok(())
    }
}


fn main() {
    // constructing the world
    let obstacle1 = Circle {
        position: Vec2::<f32>::new(3.5,2.0),
        radius: 0.5,
    };
    let obstacle2 = Circle {
        position: Vec2::<f32>::new(3.5,-2.0),
        radius: 0.5,
    };
    let obstacle3 = Circle {
        position: Vec2::<f32>::new(0.0,-4.0),
        radius: 0.5,
    };
    let grid = Grid {
        width: -7..8,
        height: -7..8,
    };
    let world = World {
        obstacles: vec![Box::new(obstacle1), Box::new(obstacle2), Box::new(obstacle3)],
        grid: grid,
    };

    let bee = Bee::new(&world, Vec2::<i32>::new(0,0));

    // generating every vector
    let vec_field = VectorField::generate(bee, &world);
    vec_field.draw("1.png").unwrap();

}

#[test]
fn segment_map_test() {
    let circle = Circle {
        position: Vec2::<f32>::new(-1.0,1.0),
        radius: 0.5,
    };

    let segment = circle.map(Vec2::<i32>::new(0,0)).unwrap();

    println!("{:?}", segment);

    assert!((segment.bisector - (PI/4.0)*3.0).abs() < 0.01);

}

#[test]
fn image_new_test() {
    let circle1 = Circle {
        position: Vec2::<f32>::new(3.5,2.0),
        radius: 0.5,
    };
    let circle2 = Circle {
        position: Vec2::<f32>::new(3.5,-2.0),
        radius: 0.5,
    };
    let circle3 = Circle {
        position: Vec2::<f32>::new(0.0,-4.0),
        radius: 0.5,
    };

    let obstacles: Vec<Box<dyn Obstacle>> = vec![Box::new(circle1), Box::new(circle2), Box::new(circle3)];

    let image = Image::new(Vec2::<i32>::new(0,0), &obstacles);

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
        bisector: PI/4.0,
        width: PI/2.0,
        color: true,
    };
    let s2 = Segment {
        bisector: 5.0*PI/4.0,
        width: PI/2.0,
        color: true,
    };
    let s3 = Segment {
        bisector: 7.0*PI/4.0,
        width: PI/2.0,
        color: true,
    };

    assert!(!s1.collides(s2));
    assert!(s1.collides(s3));

}

// TODO remove this test when everything works as intended
#[test] 
fn help() {
    // constructing the world
    let obstacle1 = Circle {
        position: Vec2::<f32>::new(3.5,2.0),
        radius: 0.5,
    };
    let obstacle2 = Circle {
        position: Vec2::<f32>::new(3.5,-2.0),
        radius: 0.5,
    };
    let obstacle3 = Circle {
        position: Vec2::<f32>::new(0.0,-4.0),
        radius: 0.5,
    };
    let grid = Grid {
        width: -7..8,
        height: -7..8,
    };
    let world = World {
        obstacles: vec![Box::new(obstacle1), Box::new(obstacle2), Box::new(obstacle3)],
        grid: grid,
    };

    let mut bee = Bee::new(&world, Vec2::<i32>::new(0,0));

    bee.position = Vec2::<i32>::new( 0, 1);

    let out = bee.home(&world);

    println!("{:?}", out);

}
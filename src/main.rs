use std::{
    fs::File,
    io::{BufRead, BufReader, Lines},
};

use nalgebra::*;
#[derive(Debug, Clone)]
struct Map {
    walls: Vec<Wall>,
}

#[derive(Debug, Clone, Copy)]
struct Wall {
    p1: Vector2<f64>,
    p2: Vector2<f64>,
    forward: Vector2<f64>,
}

impl Wall {
    fn new(p1: Vector2<f64>, p2: Vector2<f64>) -> Self {
        let vec3 = (
            Vector3::<f64>::new(p1.x, p1.y, 0.0),
            Vector3::<f64>::new(p2.x, p2.y, 0.0),
        );
        let up = Vector3::<f64>::new(0.0, 0.0, 1.0);
        let forward = up.cross(&(vec3.1 - vec3.0));
        let forward = Vector2::<f64>::new(forward.x, forward.y);
        Self { p1, p2, forward }
    }
    fn intersection(&self, plane: &Wall) -> Option<Vector2<f64>> {
        let x1 = self.p1.x;
        let y1 = self.p1.y;
        let x2 = self.p2.x;
        let y2 = self.p2.y;

        let x3 = plane.p1.x;
        let y3 = plane.p1.y;
        let x4 = plane.p2.x;
        let y4 = plane.p2.y;

        let denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

        if denominator < 0.001 && denominator > -0.001 {
            return None;
        }

        let t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denominator;
        let u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denominator;

        if t > 0.0 && t < 1.0 {
            Some(Vector2::<f64>::new(x1 + t * (x2 - x1), y1 + t * (y2 - y1)))
        } else {
            None
        }
    }

    fn splice(&self, point: Vector2<f64>) -> (Wall, Wall) {
        (
            Wall {
                p1: self.p1,
                p2: point,
                forward: self.forward,
            },
            Wall {
                p1: point,
                p2: self.p2,
                forward: self.forward,
            },
        )
    }

    fn in_front(&self, wall: &Wall) -> bool {
        let center = (wall.p1 + wall.p2) / 2.0;
        let diff = center - self.p1;
        return diff.dot(&self.forward) > 0.0;
    }
    fn in_front_point(&self, point: &Vector2<f64>) -> bool {
        let diff = point - self.p1;
        return diff.dot(&self.forward) > 0.0;
    }
}

impl Map {
    fn from_file(path: &str) -> Map {
        let mut out = Map { walls: vec![] };
        let file = File::open(path).unwrap();
        let reader = BufReader::new(file);
        let mut verticies = vec![];
        let mut walls = false;
        for line in reader.lines() {
            let line = line.unwrap();
            if !walls {
                if line == "walls" {
                    walls = true;
                    continue;
                }
                let coordinates = line
                    .split(' ')
                    .map(|x| x.parse::<f64>().unwrap())
                    .collect::<Vec<f64>>();
                verticies.push(Vector2::new(coordinates[0], coordinates[1]));
            } else {
                let indexs = line
                    .split(' ')
                    .map(|x| x.parse::<usize>().unwrap())
                    .collect::<Vec<usize>>();
                out.walls.push(Wall::new(
                    verticies[indexs[0] - 1],
                    verticies[indexs[1] - 1],
                ));
            }
        }

        out
    }
    fn generate_tree(&self) -> Option<BSPTree> {
        Self::tree_create(&self.walls)
    }

    fn tree_create(walls: &Vec<Wall>) -> Option<BSPTree> {
        if walls.len() == 0 {
            return None;
        }
        if walls.len() == 1 {
            return Some(BSPTree {
                behind: Box::new(None),
                front: Box::new(None),
                segment: walls[0],
            });
        }
        let slice_plane = walls[0];

        // splice all walls that need splicing
        let mut new_walls = vec![];
        for wall in &walls[1..] {
            if let Some(intersection) = wall.intersection(&slice_plane) {
                let spliced = wall.splice(intersection);
                new_walls.push(spliced.0);
                new_walls.push(spliced.1);
            } else {
                new_walls.push(*wall);
            }
        }
        // calculate front and back walls
        let mut front = vec![];
        let mut back = vec![];

        for wall in &new_walls {
            if slice_plane.in_front(wall) {
                front.push(*wall);
            } else {
                back.push(*wall);
            }
        }

        Some(BSPTree {
            behind: Box::new(Self::tree_create(&back)),
            front: Box::new(Self::tree_create(&front)),
            segment: slice_plane,
        })
    }
}

#[derive(Debug, Clone)]
struct BSPTree {
    behind: Box<Option<BSPTree>>,
    front: Box<Option<BSPTree>>,
    segment: Wall,
}

impl BSPTree {
    fn get_render_order(&self, camera_pos: Vector2<f64>) -> Vec<Wall> {
        let mut out = vec![];

        Self::get_render_walls(&Some(self.clone()), &mut out, camera_pos);

        out
    }

    fn get_render_walls(node: &Option<BSPTree>, out: &mut Vec<Wall>, camera_pos: Vector2<f64>) {
        if node.is_none() {
            return;
        }
        let node = node.as_ref().unwrap();
        if !node.segment.in_front_point(&camera_pos) {
            Self::get_render_walls(&node.front, out, camera_pos);
            out.push(node.segment);
            Self::get_render_walls(&node.behind, out, camera_pos);
        } else {
            Self::get_render_walls(&node.behind, out, camera_pos);
            out.push(node.segment);
            Self::get_render_walls(&node.front, out, camera_pos);
        }
    }
}
fn main() {
    let map = Map::from_file("./map.txt");
    // println!("{:?}", map.walls[1].intersection(&map.walls[0]));
    println!("{:?}", map);
    let tree = map.generate_tree().unwrap();
    println!("{:#?}", tree.clone().front.unwrap());
    let render_order = tree.get_render_order(Vector2::<f64>::new(-0.6, -0.5));
    println!("{:?}", render_order);
}

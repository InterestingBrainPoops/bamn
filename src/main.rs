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

    fn splice(&mut self, point: Vector2<f64>) -> Wall {
        let p2 = self.p2;
        self.p2 = point;
        Wall::new(point, p2)
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
}
fn main() {
    let map = Map::from_file("./map.txt");
    println!("{:?}", map.walls[1].intersection(&map.walls[0]));
    println!("{:?}", map);
}

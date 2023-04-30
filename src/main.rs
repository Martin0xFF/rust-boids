use rand::prelude::*;
use std::collections::VecDeque;

use bevy::{
    prelude::*,
    sprite::collide_aabb::{collide, Collision},
    sprite::MaterialMesh2dBundle,
};

const NUM_BOIDS: usize = 100;
const BACKGROUND_COLOR: Color = Color::rgb(0.0, 0.0, 0.0);
const BOID_COLOR: Color = Color::rgb(0.7, 0.0, 0.7);
const BOID_SIZE: Vec3 = Vec3::new(10.0, 10.0, 0.0);
const BOID_STARTING_POSITION: Vec3 = Vec3::new(0., 0., 1.0);
const BOID_SPEED: f32 = 400.0;
const TIME_STEP: f32 = 1.0 / 60.0;

const INITIAL_BALL_DIRECTION: Vec2 = Vec2::new(0.5, -0.5);

const WALL_THICKNESS: f32 = 10.0;
// x coordinates
const LEFT_WALL: f32 = -600.;
const RIGHT_WALL: f32 = 600.;
// y coordinates
const BOTTOM_WALL: f32 = -300.;
const TOP_WALL: f32 = 300.;
const WALL_COLOR: Color = Color::rgb(0.8, 0.8, 0.8);

#[derive(Component)]
struct Boid;
#[derive(Component, Deref, DerefMut)]
struct Velocity(Vec2);

#[derive(Default)]
struct CollisionEvent;
#[derive(Component)]
struct Collider;
// This bundle is a collection of the components that define a "wall" in our game
#[derive(Bundle)]
struct WallBundle {
    // You can nest bundles inside of other bundles like this
    // Allowing you to compose their functionality
    sprite_bundle: SpriteBundle,
    collider: Collider,
}

fn norm(vel: &Velocity) -> f32 {
    f32::sqrt(vel.x * vel.x + vel.y * vel.y)
}
fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(ClearColor(BACKGROUND_COLOR))
        .add_startup_system(setup)
        .add_event::<CollisionEvent>()
        // Add our gameplay simulation systems to the fixed timestep schedule
        .add_systems(
            (
                boid_logic,
                check_for_collisions,
                apply_velocity.after(check_for_collisions),
            )
                .in_schedule(CoreSchedule::FixedUpdate),
        )
        // Configure how frequently our gameplay systems are run
        .insert_resource(FixedTime::new_from_secs(TIME_STEP))
        .add_system(bevy::window::close_on_esc)
        .run();
    println!("Hello, world!");
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
    asset_server: Res<AssetServer>,
) {
    // Camera
    commands.spawn(Camera2dBundle::default());
    let mut rng = rand::thread_rng();
    // Boid
    for i in 0..NUM_BOIDS {
        let mut boid_start_pos = BOID_STARTING_POSITION;
        let rand_x: f32 = rng.gen();
        let rand_y: f32 = rng.gen();
        boid_start_pos[0] += rand_x * 599. - 599.;
        boid_start_pos[1] += rand_y * 299. - 299.;
        commands.spawn((
            MaterialMesh2dBundle {
                mesh: meshes.add(shape::Circle::default().into()).into(),
                material: materials.add(ColorMaterial::from(BOID_COLOR)),
                transform: Transform::from_translation(boid_start_pos).with_scale(BOID_SIZE),
                ..default()
            },
            Boid,
            Velocity(INITIAL_BALL_DIRECTION.normalize() * BOID_SPEED),
        ));
    }
    // Walls
    commands.spawn(WallBundle::new(WallLocation::Left));
    commands.spawn(WallBundle::new(WallLocation::Right));
    commands.spawn(WallBundle::new(WallLocation::Bottom));
    commands.spawn(WallBundle::new(WallLocation::Top));
}

fn apply_velocity(mut query: Query<(&mut Transform, &Velocity)>) {
    for (mut transform, velocity) in &mut query {
        transform.translation.x += velocity.x * TIME_STEP;
        transform.translation.y += velocity.y * TIME_STEP;
    }
}

fn get_polar(vel: &Velocity) -> Vec<f32> {
    let theta = f32::atan2(vel.y, vel.x);
    let r = norm(vel);
    return vec![r, theta];
}
fn get_carte(polar: &Vec<f32>) -> Velocity {
    let x = polar[0] * f32::cos(polar[1]);
    let y = polar[0] * f32::sin(polar[1]);
    return Velocity(Vec2 { x: x, y: y });
}
fn within_range(boid_t: &Transform, other_boid_t: &Transform, r: &f32, theta: &f32) -> bool {
    let dist = boid_t.translation.distance(other_boid_t.translation);
    if (dist == 0.0) {
        return false;
    }
    if (&dist < r) {
        return true;
    }
    return false;
}

fn boid_logic(
    mut boid_query: Query<(&mut Velocity, &Transform), With<Boid>>,
    mut collision_events: EventWriter<CollisionEvent>,
) {
    let kalign = 0.1;
    let kspace = 1.0;
    let vision_distance = 50.0;
    let vision_fov = 90.0;
    let mut new_boid_velocities: VecDeque<Velocity> = VecDeque::new();
    // TODO(0xff): include void logic within this function
    for (boid_velocity, boid_transform) in &boid_query {
        let polar_boid = get_polar(boid_velocity);
        let mut accumative_alignment_residual = 0.0;
        let mut seperation_update = vec![0.0, 0.0];
        for (other_boid_velocity, other_boid_transform) in &boid_query {
            if within_range(
                boid_transform,
                other_boid_transform,
                &vision_distance,
                &vision_fov,
            ) {
                let other_polar_boid = get_polar(other_boid_velocity);

                // Alignment
                let residual_theta = other_polar_boid[1] - polar_boid[1];
                accumative_alignment_residual += kalign * residual_theta;

                // Seperation
                let repulsion_angle = -boid_transform
                    .translation
                    .angle_between(other_boid_transform.translation);
                let repulsion_strength = 1.0
                    / boid_transform
                        .translation
                        .distance_squared(other_boid_transform.translation);
                let temp = vec![kspace * repulsion_strength, repulsion_angle];
                let sep_vector = get_carte(&temp);
                seperation_update[0] += sep_vector[0];
                seperation_update[1] += sep_vector[1];
            }
        }

        // Alignment
        let new_polar = vec![polar_boid[0], polar_boid[1] + accumative_alignment_residual];
        let mut new_boid_velocity = get_carte(&new_polar);

        // Seperation
        new_boid_velocity[0] += seperation_update[0];
        new_boid_velocity[1] += seperation_update[1];

        let new_speed = new_boid_velocity
            .distance(Vec2 { x: 0.0, y: 0.0 })
            .clamp(0.0, BOID_SPEED);
        new_boid_velocities.push_back(Velocity(new_boid_velocity.normalize() * new_speed));
    }

    for (mut boid_velocity, boid_transform) in &mut boid_query {
        let maybe_velocity = new_boid_velocities.pop_front();
        match maybe_velocity {
            None => {}
            Some(vel) => *boid_velocity = vel,
        }
    }
}
fn check_for_collisions(
    mut boid_query: Query<(&mut Velocity, &Transform), With<Boid>>,
    collider_query: Query<(Entity, &Transform), With<Collider>>,
    mut collision_events: EventWriter<CollisionEvent>,
) {
    // TODO(0xff): include void logic within this function
    for (mut boid_velocity, boid_transform) in &mut boid_query {
        let boid_size = boid_transform.scale.truncate();

        // check collision with walls
        for (collider_entity, transform) in &collider_query {
            let collision = collide(
                boid_transform.translation,
                boid_size,
                transform.translation,
                transform.scale.truncate(),
            );
            if let Some(collision) = collision {
                // Sends a collision event so that other systems can react to the collision
                collision_events.send_default();

                // reflect the boid when it collides
                let mut reflect_x = false;
                let mut reflect_y = false;

                // only reflect if the boid's velocity is going in the opposite direction of the
                // collision
                match collision {
                    Collision::Left => reflect_x = boid_velocity.x > 0.0,
                    Collision::Right => reflect_x = boid_velocity.x < 0.0,
                    Collision::Top => reflect_y = boid_velocity.y < 0.0,
                    Collision::Bottom => reflect_y = boid_velocity.y > 0.0,
                    Collision::Inside => { /* do nothing */ }
                }

                // reflect velocity on the x-axis if we hit something on the x-axis
                if reflect_x {
                    boid_velocity.x = -boid_velocity.x;
                }

                // reflect velocity on the y-axis if we hit something on the y-axis
                if reflect_y {
                    boid_velocity.y = -boid_velocity.y;
                }
            }
        }
    }
}
/// Which side of the arena is this wall located on?
enum WallLocation {
    Left,
    Right,
    Bottom,
    Top,
}

impl WallLocation {
    fn position(&self) -> Vec2 {
        match self {
            WallLocation::Left => Vec2::new(LEFT_WALL, 0.),
            WallLocation::Right => Vec2::new(RIGHT_WALL, 0.),
            WallLocation::Bottom => Vec2::new(0., BOTTOM_WALL),
            WallLocation::Top => Vec2::new(0., TOP_WALL),
        }
    }

    fn size(&self) -> Vec2 {
        let arena_height = TOP_WALL - BOTTOM_WALL;
        let arena_width = RIGHT_WALL - LEFT_WALL;
        // Make sure we haven't messed up our constants
        assert!(arena_height > 0.0);
        assert!(arena_width > 0.0);

        match self {
            WallLocation::Left | WallLocation::Right => {
                Vec2::new(WALL_THICKNESS, arena_height + WALL_THICKNESS)
            }
            WallLocation::Bottom | WallLocation::Top => {
                Vec2::new(arena_width + WALL_THICKNESS, WALL_THICKNESS)
            }
        }
    }
}

impl WallBundle {
    // This "builder method" allows us to reuse logic across our wall entities,
    // making our code easier to read and less prone to bugs when we change the logic
    fn new(location: WallLocation) -> WallBundle {
        WallBundle {
            sprite_bundle: SpriteBundle {
                transform: Transform {
                    // We need to convert our Vec2 into a Vec3, by giving it a z-coordinate
                    // This is used to determine the order of our sprites
                    translation: location.position().extend(0.0),
                    // The z-scale of 2D objects must always be 1.0,
                    // or their ordering will be affected in surprising ways.
                    // See https://github.com/bevyengine/bevy/issues/4149
                    scale: location.size().extend(1.0),
                    ..default()
                },
                sprite: Sprite {
                    color: WALL_COLOR,
                    ..default()
                },
                ..default()
            },
            collider: Collider,
        }
    }
}

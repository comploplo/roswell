//! Drive turtlesim from a terminal UI.
//!
//! Publishes `geometry_msgs/Twist` to `/turtle1/cmd_vel` and subscribes
//! `turtlesim/Pose` on `/turtle1/pose` via the [`Transport`] abstraction.
//!
//!   cargo run --bin turtle_teleop          # interactive TUI
//!   cargo run --bin turtle_teleop auto     # scripted drive (headless verify)
//!
//! Controls: W/S forward/back · A/D turn left/right · Space stop · Q quit.

use std::io;
use std::time::{Duration, Instant};

use roscmp_dds::msgs::{geometry_msgs__Twist, geometry_msgs__Vector3, turtlesim__Pose};
use roscmp_dds::transport::{Dds, MsgPublisher, MsgSubscriber, Qos, Transport};

/// The pose fields we display (the generated message isn't `Copy`).
#[derive(Clone, Copy)]
struct Pose2 {
    x: f32,
    y: f32,
    theta: f32,
}

fn twist(lin_x: f64, ang_z: f64) -> geometry_msgs__Twist {
    geometry_msgs__Twist {
        linear: geometry_msgs__Vector3 {
            x: lin_x,
            y: 0.0,
            z: 0.0,
        },
        angular: geometry_msgs__Vector3 {
            x: 0.0,
            y: 0.0,
            z: ang_z,
        },
    }
}

/// Drain all pending pose samples, keeping the latest.
fn latest_pose(sub: &mut impl MsgSubscriber<turtlesim__Pose>, current: &mut Option<Pose2>) {
    while let Some(p) = sub.take() {
        *current = Some(Pose2 {
            x: p.x,
            y: p.y,
            theta: p.theta,
        });
    }
}

fn main() {
    let auto = std::env::args().nth(1).as_deref() == Some("auto");
    let dds = Dds::new(0);
    let cmd = dds.publisher::<geometry_msgs__Twist>("/turtle1/cmd_vel", Qos::Default);
    let mut pose = dds.subscriber::<turtlesim__Pose>("/turtle1/pose", Qos::Default);
    if auto {
        run_auto(&cmd, &mut pose);
    } else {
        run_tui(&cmd, &mut pose).expect("tui");
    }
}

/// Scripted drive for headless verification: forward, turn, stop; report pose.
fn run_auto(
    cmd: &impl MsgPublisher<geometry_msgs__Twist>,
    sub: &mut impl MsgSubscriber<turtlesim__Pose>,
) {
    let mut pose: Option<Pose2> = None;
    let start = Instant::now();
    let mut init: Option<Pose2> = None;

    while start.elapsed() < Duration::from_secs(2) {
        latest_pose(sub, &mut pose);
        if init.is_none() {
            init = pose;
        }
        std::thread::sleep(Duration::from_millis(50));
    }

    let drive_start = Instant::now();
    loop {
        let t = drive_start.elapsed().as_secs_f64();
        let c = if t < 2.0 {
            twist(2.0, 0.0)
        } else if t < 4.0 {
            twist(0.0, 1.8)
        } else {
            twist(0.0, 0.0)
        };
        cmd.publish(c);
        latest_pose(sub, &mut pose);
        if t >= 4.5 {
            break;
        }
        std::thread::sleep(Duration::from_millis(50));
    }

    let (Some(i), Some(f)) = (init, pose) else {
        eprintln!("no pose received from turtlesim");
        std::process::exit(1);
    };
    println!("INIT  x={:.3} y={:.3} theta={:.3}", i.x, i.y, i.theta);
    println!("FINAL x={:.3} y={:.3} theta={:.3}", f.x, f.y, f.theta);
    let moved = (f.x - i.x).abs() > 0.1 || (f.theta - i.theta).abs() > 0.1;
    println!("MOVED={moved}");
    std::process::exit(i32::from(!moved));
}

// ---- interactive TUI ----------------------------------------------------

use crossterm::{
    event::{self, Event, KeyCode},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use ratatui::{
    prelude::*,
    widgets::{Block, Borders, Paragraph},
};

fn run_tui(
    cmd: &impl MsgPublisher<geometry_msgs__Twist>,
    sub: &mut impl MsgSubscriber<turtlesim__Pose>,
) -> io::Result<()> {
    enable_raw_mode()?;
    let mut out = io::stdout();
    execute!(out, EnterAlternateScreen)?;
    let mut term = Terminal::new(CrosstermBackend::new(out))?;

    let (mut lin, mut ang) = (0.0f64, 0.0f64);
    let mut pose: Option<Pose2> = None;
    let result = loop {
        if event::poll(Duration::from_millis(50))? {
            if let Event::Key(k) = event::read()? {
                match k.code {
                    KeyCode::Char('w') | KeyCode::Up => lin = (lin + 0.5).min(4.0),
                    KeyCode::Char('s') | KeyCode::Down => lin = (lin - 0.5).max(-4.0),
                    KeyCode::Char('a') | KeyCode::Left => ang = (ang + 0.5).min(4.0),
                    KeyCode::Char('d') | KeyCode::Right => ang = (ang - 0.5).max(-4.0),
                    KeyCode::Char(' ') => {
                        lin = 0.0;
                        ang = 0.0;
                    }
                    KeyCode::Char('q') | KeyCode::Esc => break Ok(()),
                    _ => {}
                }
            }
        }

        cmd.publish(twist(lin, ang));
        latest_pose(sub, &mut pose);

        let pose_line = match pose {
            Some(p) => format!("x={:.2}  y={:.2}  theta={:.2}", p.x, p.y, p.theta),
            None => "(waiting for turtlesim /turtle1/pose...)".to_string(),
        };
        let text = vec![
            Line::from("Drive: W/S forward·back   A/D turn   Space stop   Q quit"),
            Line::from(""),
            Line::from(format!(
                "command   linear.x={lin:+.2}   angular.z={ang:+.2}"
            )),
            Line::from(format!("pose      {pose_line}")),
        ];
        term.draw(|f| {
            let block = Block::default()
                .borders(Borders::ALL)
                .title(" roscmp turtle teleop (RTPS + our CDR) ");
            f.render_widget(Paragraph::new(text).block(block), f.area());
        })?;
    };

    disable_raw_mode()?;
    execute!(term.backend_mut(), LeaveAlternateScreen)?;
    term.show_cursor()?;
    result
}

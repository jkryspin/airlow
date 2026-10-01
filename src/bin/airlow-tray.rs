//! airlow-tray: background tray app. Select "Speakers (Steam Streaming Speakers)" as the Windows output; when the
//! paired AirPods connect (case opened or put in), their audio is streamed over airlow's low-latency stack.

#![cfg_attr(windows, windows_subsystem = "windows")]

use airlow::aacp::{self, NoiseMode};
use airlow::daemon::{self, Cmd, Status, icon_rgba};
use airlow::live;
use std::path::PathBuf;
use std::sync::mpsc;
use std::time::Duration;
use tao::event::Event;
use tao::event_loop::{ControlFlow, EventLoopBuilder};
use tray_icon::menu::{CheckMenuItem, Menu, MenuEvent, MenuItem, PredefinedMenuItem, Submenu};
use tray_icon::{Icon, TrayIconBuilder};

enum Ev {
    Status(Status),
    Noise(Option<NoiseMode>),
    Pods(aacp::Pods),
    Menu(tray_icon::menu::MenuId),
}

const RUN_KEY: &str = r"HKCU\Software\Microsoft\Windows\CurrentVersion\Run";
const RUN_NAME: &str = "airlow";

fn data_dir() -> PathBuf {
    PathBuf::from(std::env::var("APPDATA").unwrap_or_else(|_| ".".into())).join("airlow")
}

/// Run a helper program without flashing a console window.
fn quiet(program: &str, args: &[&str]) -> Option<std::process::Output> {
    use std::os::windows::process::CommandExt;
    std::process::Command::new(program).args(args).creation_flags(0x0800_0000).output().ok()
}

fn autostart_enabled() -> bool {
    quiet("reg", &["query", RUN_KEY, "/v", RUN_NAME]).map(|o| o.status.success()).unwrap_or(false)
}

fn set_autostart(on: bool) {
    if on {
        let exe = std::env::current_exe().map(|p| format!("\"{}\"", p.display())).unwrap_or_default();
        quiet("reg", &["add", RUN_KEY, "/v", RUN_NAME, "/t", "REG_SZ", "/d", &exe, "/f"]);
    } else {
        quiet("reg", &["delete", RUN_KEY, "/v", RUN_NAME, "/f"]);
    }
}

/// With no console, stdout/stderr go nowhere: point them at a log file (truncated when it grows large).
fn redirect_logs() {
    use std::os::windows::io::AsRawHandle;
    use windows::Win32::Foundation::HANDLE;
    use windows::Win32::System::Console::{STD_ERROR_HANDLE, STD_OUTPUT_HANDLE, SetStdHandle};
    let dir = data_dir();
    let _ = std::fs::create_dir_all(&dir);
    let path = dir.join("airlow.log");
    if std::fs::metadata(&path).map(|m| m.len() > 2_000_000).unwrap_or(false) {
        let _ = std::fs::rename(&path, dir.join("airlow.old.log"));
    }
    if let Ok(f) = std::fs::OpenOptions::new().create(true).append(true).open(&path) {
        let h = HANDLE(f.as_raw_handle());
        unsafe {
            let _ = SetStdHandle(STD_OUTPUT_HANDLE, h);
            let _ = SetStdHandle(STD_ERROR_HANDLE, h);
        }
        std::mem::forget(f);
    }
}

/// True if another airlow-tray already runs in this session.
fn already_running() -> bool {
    use windows::Win32::Foundation::{ERROR_ALREADY_EXISTS, GetLastError};
    use windows::Win32::System::Threading::CreateMutexW;
    use windows::core::w;
    unsafe {
        let m = CreateMutexW(None, false, w!("Local\\airlow-tray-single-instance"));
        let exists = m.is_ok() && GetLastError() == ERROR_ALREADY_EXISTS;
        std::mem::forget(m);
        exists
    }
}

fn icon_for(s: &Status) -> Icon {
    Icon::from_rgba(icon_rgba(32, s.rgb()), 32, 32).expect("icon")
}

fn main() {
    if already_running() {
        return;
    }
    redirect_logs();
    airlow::realtime_tuning();
    println!("airlow-tray starting");

    let event_loop = EventLoopBuilder::<Ev>::with_user_event().build();
    let proxy = event_loop.create_proxy();
    {
        let p = proxy.clone();
        MenuEvent::set_event_handler(Some(move |e: MenuEvent| {
            let _ = p.send_event(Ev::Menu(e.id));
        }));
    }

    let status_item = MenuItem::new("Starting...", false, None);
    let battery_item = MenuItem::new(aacp::Pods::default().battery_text(), false, None);
    let ears_item = MenuItem::new(aacp::Pods::default().ears_text(), false, None);
    let pair_item = MenuItem::new("Pair AirPods...", true, None);
    let reconnect_item = MenuItem::new("Reconnect now", true, None);
    let noise_menu = Submenu::new("Noise control", false);
    let noise_items: Vec<(NoiseMode, CheckMenuItem)> =
        NoiseMode::ALL.iter().map(|m| (*m, CheckMenuItem::new(m.label(), true, false, None))).collect();
    for (_, item) in &noise_items {
        let _ = noise_menu.append(item);
    }
    let autostart_item = CheckMenuItem::new("Start with Windows", true, autostart_enabled(), None);
    let settings_item = MenuItem::new("Open settings file", true, None);
    let logs_item = MenuItem::new("Open log folder", true, None);
    let quit_item = MenuItem::new("Quit", true, None);
    let menu = Menu::new();
    let _ = menu.append_items(&[
        &status_item,
        &battery_item,
        &ears_item,
        &PredefinedMenuItem::separator(),
        &pair_item,
        &reconnect_item,
        &noise_menu,
        &PredefinedMenuItem::separator(),
        &autostart_item,
        &settings_item,
        &logs_item,
        &PredefinedMenuItem::separator(),
        &quit_item,
    ]);

    let tray = TrayIconBuilder::new()
        .with_menu(Box::new(menu))
        .with_tooltip("airlow")
        .with_icon(icon_for(&Status::Waiting))
        .build()
        .expect("tray icon");

    // The AirPods report their noise control mode on the control channel; mirror it into the menu.
    {
        let p = proxy.clone();
        std::thread::spawn(move || {
            let (mut last, mut last_pods) = (None, aacp::Pods::default());
            loop {
                let now = aacp::current();
                if now != last {
                    last = now;
                    if p.send_event(Ev::Noise(now)).is_err() {
                        return;
                    }
                }
                let pods = aacp::pods();
                if pods != last_pods {
                    last_pods = pods;
                    if p.send_event(Ev::Pods(pods)).is_err() {
                        return;
                    }
                }
                std::thread::sleep(Duration::from_millis(300));
            }
        });
    }

    let (tx, rx) = mpsc::channel::<Cmd>();
    let worker = {
        let p = proxy.clone();
        std::thread::spawn(move || {
            daemon::run(rx, &move |s| {
                println!("status: {}", s.text());
                let _ = p.send_event(Ev::Status(s));
            })
        })
    };
    let mut worker = Some(worker);

    event_loop.run(move |event, _, flow| {
        *flow = ControlFlow::Wait;
        let Event::UserEvent(ev) = event else { return };
        match ev {
            Ev::Status(s) => {
                let text = s.text();
                status_item.set_text(&text);
                let _ = tray.set_icon(Some(icon_for(&s)));
                let _ = tray.set_tooltip(Some(format!("airlow: {text}")));
            }
            Ev::Pods(p) => {
                battery_item.set_text(p.battery_text());
                ears_item.set_text(p.ears_text());
            }
            Ev::Noise(mode) => {
                noise_menu.set_enabled(mode.is_some());
                for (m, item) in &noise_items {
                    item.set_checked(Some(*m) == mode);
                }
            }
            Ev::Menu(id) => {
                if let Some((m, item)) = noise_items.iter().find(|(_, i)| id == *i.id()) {
                    // The check mark follows the AirPods' own confirmation, not the click.
                    item.set_checked(aacp::current() == Some(*m));
                    aacp::request(*m);
                    return;
                }
                if id == *pair_item.id() {
                    let _ = tx.send(Cmd::Pair);
                } else if id == *reconnect_item.id() {
                    let _ = tx.send(Cmd::Reconnect);
                } else if id == *autostart_item.id() {
                    set_autostart(autostart_item.is_checked());
                } else if id == *settings_item.id() {
                    let p = daemon::Config::path();
                    if !p.exists() {
                        let _ = daemon::Config::load();
                    }
                    let _ = std::process::Command::new("notepad").arg(p).spawn();
                } else if id == *logs_item.id() {
                    let _ = std::process::Command::new("explorer").arg(data_dir()).spawn();
                } else if id == *quit_item.id() {
                    live::STOP.store(true, std::sync::atomic::Ordering::Relaxed);
                    let _ = tx.send(Cmd::Quit);
                    if let Some(w) = worker.take() {
                        // The worker finishes within a moment (Suspend, then return); do not hang on a stuck radio.
                        let (done_tx, done_rx) = mpsc::channel();
                        std::thread::spawn(move || {
                            let _ = w.join();
                            let _ = done_tx.send(());
                        });
                        let _ = done_rx.recv_timeout(Duration::from_secs(3));
                    }
                    *flow = ControlFlow::Exit;
                }
            }
        }
    });
}

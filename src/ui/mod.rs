// ZerOS — Desktop Shell & App Window Manager (`M9.5`)
//
// Hosts the 4 modular applications (`src/apps/`):
//   - `AppId::Music`   (`1.听歌`): `MusicApp`
//   - `AppId::Reader`  (`2.看书`): `ReaderApp`
//   - `AppId::MediaBt` (`3.蓝牙`): `MediaBtApp`
//   - `AppId::System`  (`4.系统`): `SystemApp`
//
// Responsibilities of `DesktopShell`:
//   - Renders the unified 21px top Chinese navigation & status bar (`A2DP` + `MM:SS` uptime)
//   - Dispatches hardware bezel buttons (Scheme B: short-press `B1..B4` -> in-app slot `0..3`,
//     long-press `B1` -> switch app tab, long-press `B4` -> safe reboot to Raspberry Pi OS)
//   - Dispatches NEC IR remote codes and ADS7846 touch events (`y <= 22` top bar vs `y > 22` app area)
//   - Routes cross-application `AppIntent` messages (`SwitchApp`, `PlayTrack`, `OpenBook`)

use alloc::string::String;
use core::fmt::Write;

use crate::apps::media_bt::MediaBtApp;
use crate::apps::music::MusicApp;
use crate::apps::reader::ReaderApp;
use crate::apps::system::SystemApp;
use crate::apps::{App, AppId, AppIntent};
use crate::arch::timer;
use crate::drivers::bluetooth;
use crate::drivers::display::{colors, Color};
use crate::drivers::input::{self, touch, InputEvent};
use crate::gfx::{Canvas, SCREEN_H, SCREEN_W};
use crate::kernel::{task, tryboot};

pub struct DesktopShell {
    canvas: Canvas,
    active_app: AppId,
    needs_full_redraw: bool,
    needs_dynamic_redraw: bool,

    // Hosted applications
    music_app: MusicApp,
    reader_app: ReaderApp,
    media_bt_app: MediaBtApp,
    system_app: SystemApp,
}

impl DesktopShell {
    pub fn new() -> Self {
        Self {
            canvas: Canvas::new(),
            active_app: AppId::Music,
            needs_full_redraw: true,
            needs_dynamic_redraw: false,
            music_app: MusicApp::new(),
            reader_app: ReaderApp::new(),
            media_bt_app: MediaBtApp::new(),
            system_app: SystemApp::new(),
        }
    }

    /// Return a mutable trait-object reference to the currently active application.
    fn current_app_mut(&mut self) -> &mut dyn App {
        match self.active_app {
            AppId::Music => &mut self.music_app,
            AppId::Reader => &mut self.reader_app,
            AppId::MediaBt => &mut self.media_bt_app,
            AppId::System => &mut self.system_app,
        }
    }

    /// Route an `AppIntent` returned by an application across the desktop shell.
    fn dispatch_intent(&mut self, intent: AppIntent) {
        match intent {
            AppIntent::None => {}
            AppIntent::SwitchApp(target) => {
                self.active_app = target;
            }
            AppIntent::PlayTrack { title } => {
                self.music_app.play_track(title);
                self.active_app = AppId::Music;
            }
            AppIntent::OpenBook { title, content } => {
                self.reader_app.open_book(title, &content);
                self.active_app = AppId::Reader;
            }
        }
    }

    /// Process a hardware input event (Bezel Buttons, Long-Press, IR Remote, or Resistive Touch).
    pub fn handle_event(&mut self, ev: InputEvent) {
        match ev {
            InputEvent::ButtonPress(btn) => {
                // Scheme B: Short-press B1..B4 executes the 4 in-page actions (0..=3) of the active app
                self.system_app.set_last_button(Some(btn));
                let slot = (btn as usize) & 3;
                let intent = self.current_app_mut().on_slot_action(slot);
                self.dispatch_intent(intent);
                self.needs_full_redraw = true;
            }
            InputEvent::ButtonLongPress(btn) => {
                self.system_app.set_last_button(Some(btn));
                match btn {
                    0 => {
                        // Long-press B1 (>= 0.75s): Switch top application tab
                        crate::println!("[UI] B1 long-press -> switching app tab");
                        self.active_app = self.active_app.next();
                        self.needs_full_redraw = true;
                    }
                    3 => {
                        // Long-press B4 (>= 1.2s): Safe reboot back to Raspberry Pi OS
                        crate::println!("[UI] B4 long-press -> rebooting to RPi OS...");
                        tryboot::reboot_to_partition(0);
                    }
                    _ => {}
                }
            }
            InputEvent::ButtonRelease(_) => {
                self.system_app.set_last_button(None);
                if self.active_app == AppId::System {
                    self.needs_full_redraw = true;
                }
            }
            InputEvent::IrCode(code) => {
                let cmd = (code >> 16) & 0xFF;
                let next_app = match cmd {
                    0x45 => Some(AppId::Music),
                    0x46 => Some(AppId::Reader),
                    0x47 => Some(AppId::MediaBt),
                    0x44 => Some(AppId::System),
                    _ => None,
                };
                if let Some(app_id) = next_app {
                    self.active_app = app_id;
                    self.needs_full_redraw = true;
                } else if self.active_app == AppId::System {
                    self.needs_full_redraw = true;
                }
            }
            InputEvent::Touch(x, y) => {
                self.system_app.set_last_touch((x, y));
                let tx = x as i32;
                let ty = y as i32;

                // 1. Unified Top Bar Tabs (y = 0..22, x = 4..220)
                if ty <= 22 && tx >= 4 && tx < 220 {
                    let idx = ((tx - 4) / 54).clamp(0, 3) as usize;
                    let new_app = AppId::from_index(idx);
                    if new_app != self.active_app {
                        self.active_app = new_app;
                        self.needs_full_redraw = true;
                    }
                    return;
                }

                // 2. Delegate client-area touch to active application
                let intent = self.current_app_mut().on_touch(tx, ty);
                self.dispatch_intent(intent);
                self.needs_full_redraw = true;
            }
        }
    }

    /// Render the unified 21px top Chinese navigation & status bar into `self.canvas`.
    fn draw_top_bar(&mut self) {
        self.canvas
            .fill_rect(0, 0, SCREEN_W, 21, colors::rgb(16, 24, 42));
        self.canvas
            .draw_line(0, 21, SCREEN_W - 1, 21, colors::rgb(45, 72, 115));

        let tabs: [(AppId, &str, Color); 4] = [
            (
                AppId::Music,
                self.music_app.tab_label(),
                self.music_app.accent_color(),
            ),
            (
                AppId::Reader,
                self.reader_app.tab_label(),
                self.reader_app.accent_color(),
            ),
            (
                AppId::MediaBt,
                self.media_bt_app.tab_label(),
                self.media_bt_app.accent_color(),
            ),
            (
                AppId::System,
                self.system_app.tab_label(),
                self.system_app.accent_color(),
            ),
        ];

        for (i, &(id, label, accent)) in tabs.iter().enumerate() {
            let tx = 4 + (i as i32) * 54;
            if self.active_app == id {
                self.canvas
                    .fill_rrect(tx, 2, 51, 17, 4, colors::rgb(28, 52, 90));
                self.canvas.stroke_rrect(tx, 2, 51, 17, 4, accent);
                self.canvas
                    .draw_mixed_text(tx + 5, 4, label, colors::WHITE, 12);
            } else {
                self.canvas
                    .fill_rrect(tx, 2, 51, 17, 4, colors::rgb(20, 30, 50));
                self.canvas
                    .draw_mixed_text(tx + 5, 4, label, colors::rgb(145, 168, 204), 12);
            }
        }

        // Right status badge: Bluetooth A2DP + Uptime MM:SS
        let bt = bluetooth::status();
        self.canvas
            .fill_rrect(222, 4, 48, 13, 3, colors::rgb(22, 52, 42));
        let dot_col = if bt.ble_advertising {
            colors::rgb(50, 240, 140)
        } else {
            colors::rgb(255, 190, 60)
        };
        self.canvas.fill_circle(229, 10, 2, dot_col);
        self.canvas
            .draw_str_ascii(235, 6, "A2DP", colors::rgb(90, 245, 165));

        let up_sec = timer::ticks() / 100;
        let mm = (up_sec / 60) % 100;
        let ss = up_sec % 60;
        let mut clk = String::with_capacity(6);
        let _ = write!(clk, "{:02}:{:02}", mm, ss);
        self.canvas
            .draw_str_ascii(275, 6, &clk, colors::rgb(220, 235, 255));
    }

    /// Render the active application into the off-screen canvas and flush dirty regions over SPI.
    pub fn render(&mut self) {
        if self.needs_full_redraw || self.needs_dynamic_redraw {
            if self.needs_full_redraw {
                self.canvas.fill_gradient_v(
                    0,
                    22,
                    SCREEN_W,
                    SCREEN_H - 22,
                    (12, 18, 32),
                    (8, 12, 22),
                );
            }
            self.draw_top_bar();
            match self.active_app {
                AppId::Music => self.music_app.render(&mut self.canvas),
                AppId::Reader => self.reader_app.render(&mut self.canvas),
                AppId::MediaBt => self.media_bt_app.render(&mut self.canvas),
                AppId::System => self.system_app.render(&mut self.canvas),
            }
            self.canvas.flush();
            self.needs_full_redraw = false;
            self.needs_dynamic_redraw = false;
        }
    }

    /// Periodic tick called every ~200ms (`sub_tick` increments at 5Hz; `sub_tick % 5 == 0` is 1Hz).
    pub fn tick_200ms(&mut self, sub_tick: u32) {
        let active = self.active_app;
        let dirty_music = self
            .music_app
            .on_tick_200ms(sub_tick, active == AppId::Music);
        let dirty_reader = self
            .reader_app
            .on_tick_200ms(sub_tick, active == AppId::Reader);
        let dirty_bt = self
            .media_bt_app
            .on_tick_200ms(sub_tick, active == AppId::MediaBt);
        let dirty_sys = self
            .system_app
            .on_tick_200ms(sub_tick, active == AppId::System);

        if (sub_tick % 5 == 0) || dirty_music || dirty_reader || dirty_bt || dirty_sys {
            self.needs_dynamic_redraw = true;
        }
    }
}

/// Interactive GUI task entry point (`ui_desktop`, spawned by `kernel_main`).
pub fn ui_task() {
    crate::println!(
        "[OK] GUI: M9.5 Modular App Desktop started (MusicApp + ReaderApp + MediaBtApp + SystemApp)"
    );
    let mut shell = DesktopShell::new();
    shell.render();

    let mut sub_tick = 0u32;

    loop {
        if let Some((tx, ty)) = touch::poll_touch() {
            input::push_event(InputEvent::Touch(tx, ty));
        }

        while let Some(ev) = input::poll_event() {
            shell.handle_event(ev);
        }

        shell.render();

        task::sleep_ms(40);
        sub_tick = sub_tick.wrapping_add(1);
        if sub_tick % 5 == 0 {
            shell.tick_200ms(sub_tick / 5);
        }
    }
}

// ZerOS — Application Framework & Intent Bus (`M9.5` + `M10.1`)
//
// Decouples individual applications (`MusicApp`, `ReaderApp`, `MediaBtApp`, `SystemApp`)
// from the `ui` Desktop Shell. Each application implements the `App` trait and communicates
// cross-app navigation, media-open, or Bluetooth scan/connect requests via `AppIntent`.

pub mod media_bt;
pub mod music;
pub mod reader;
pub mod system;

use alloc::string::String;
use crate::drivers::display::Color;
use crate::gfx::Canvas;

/// Identifier for each built-in desktop application tab (`0..=3`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum AppId {
    Music = 0,
    Reader = 1,
    MediaBt = 2,
    System = 3,
}

impl AppId {
    pub const ALL: [AppId; 4] = [
        AppId::Music,
        AppId::Reader,
        AppId::MediaBt,
        AppId::System,
    ];

    pub fn next(self) -> Self {
        match self {
            AppId::Music => AppId::Reader,
            AppId::Reader => AppId::MediaBt,
            AppId::MediaBt => AppId::System,
            AppId::System => AppId::Music,
        }
    }

    pub fn from_index(i: usize) -> Self {
        match i & 3 {
            0 => AppId::Music,
            1 => AppId::Reader,
            2 => AppId::MediaBt,
            _ => AppId::System,
        }
    }
}

/// Cross-application message returned by an app to request shell-level actions
/// (switching tabs, playing a track, opening an e-book, or scanning/connecting Bluetooth).
pub enum AppIntent {
    None,
    SwitchApp(AppId),
    PlayTrack { title: String },
    OpenBook { title: String, content: String },
    ScanBluetooth,
    ConnectBluetooth(usize),
}

/// Standard interface implemented by all ZerOS graphical applications.
pub trait App {
    /// Unique application identifier (`AppId`).
    fn id(&self) -> AppId;

    /// Short Chinese tab label shown in the 21px top bar (e.g. `"1.听歌"`).
    fn tab_label(&self) -> &'static str;

    /// Accent color for the active top-bar tab border.
    fn accent_color(&self) -> Color;

    /// Render the application's client area (`y = 22..240`) into the off-screen `Canvas`.
    fn render(&mut self, canvas: &mut Canvas);

    /// Handle a short-press on one of the 4 bezel buttons (`slot = 0..=3` for `B1..B4`).
    fn on_slot_action(&mut self, slot: usize) -> AppIntent;

    /// Handle a touch event inside the application's client area (`y > 22`).
    fn on_touch(&mut self, x: i32, y: i32) -> AppIntent;

    /// Periodic 200ms timer tick (`sub_tick` increments at 5Hz; `sub_tick % 5 == 0` is 1Hz).
    /// Returns `true` if the application is currently visible and needs a dynamic redraw.
    fn on_tick_200ms(&mut self, sub_tick: u32, is_active: bool) -> bool;
}

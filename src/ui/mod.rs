// ZerOS — Interactive GUI Launcher & System Apps (M7.1 + M7.2 + M7.3)
//
// Provides a 4-tab graphical interface tailored for the Argon POD's
// 320×240 RGB565 TFT display, 4 hardware buttons, NEC IR receiver, and
// resistive touch screen:
//
//   Tab 0 [1:SYS]   — Live System Monitor + Preemptive Task Spawner (M7.2)
//   Tab 1 [2:FILES] — VFS File Browser with Subdirectory Navigation & Streaming Preview (M7.3)
//   Tab 2 [3:INPUT] — Hardware Button, NEC IR & Touch Screen Visualizer
//   Tab 3 [4:LOGS]  — Live Kernel Log Viewer
//
// Controls (4 Argon POD Bezel Buttons):
//   - Button 1 (GPIO 16): Next Tab (or Close File Preview)
//   - Button 2 (GPIO 20): Move Cursor Down (in FILES) / Prev Tab
//   - Button 3 (GPIO 21):
//       * In [1:SYS]   -> Spawn CPU-bound preemptive worker task (`cpu_work`)
//       * In [2:FILES] -> Enter `<DIR>` subdirectory OR stream-preview file via VFS FD
//   - Button 4 (GPIO 26): Reboot back to Raspberry Pi OS immediately

#![allow(dead_code)]

use alloc::string::String;
use alloc::vec::Vec;
use core::fmt::Write;

use crate::arch::timer;
use crate::drivers::display::{self, colors, Color};
use crate::drivers::input::{self, touch, InputEvent};
use crate::drivers::{mailbox, pwm};
use crate::fs::fat32::{self, DirEntry};
use crate::fs::vfs::{self, OpenMode};
use crate::kernel::{console, task, tryboot};
use crate::mm::page_alloc;

// ─── Color Palette (Retro Dark Card UI) ───
const COL_BG: Color = colors::rgb(12, 16, 24);
const COL_CARD_BG: Color = colors::rgb(22, 30, 46);
const COL_CARD_BORDER: Color = colors::rgb(56, 78, 112);
const COL_TAB_INACTIVE: Color = colors::rgb(28, 38, 56);
const COL_TAB_ACTIVE: Color = colors::rgb(0, 120, 215);
const COL_ACCENT: Color = colors::rgb(0, 210, 255);
const COL_TEXT_DIM: Color = colors::rgb(150, 165, 190);
const COL_FOOTER_BG: Color = colors::rgb(18, 24, 36);
const COL_SEL_BG: Color = colors::rgb(0, 90, 160);

#[derive(Clone, Copy, PartialEq, Eq)]
enum Tab {
    SysMon = 0,
    Files = 1,
    InputTest = 2,
    KernelLog = 3,
}

impl Tab {
    fn next(self) -> Self {
        match self {
            Tab::SysMon => Tab::Files,
            Tab::Files => Tab::InputTest,
            Tab::InputTest => Tab::KernelLog,
            Tab::KernelLog => Tab::SysMon,
        }
    }

    fn prev(self) -> Self {
        match self {
            Tab::SysMon => Tab::KernelLog,
            Tab::Files => Tab::SysMon,
            Tab::InputTest => Tab::Files,
            Tab::KernelLog => Tab::InputTest,
        }
    }
}

/// Pure CPU-bound worker task spawned by pressing Button 3 in `1:SYS`.
/// Intentionally does NOT call `yield_now()` or `sleep_ms()` inside its prime
/// computation loop so that hardware time-slice preemption (`preempt_from_irq`)
/// preempts it every 50ms! Once finished, its 16KB stack is automatically reaped.
fn preemptive_cpu_worker() {
    crate::println!("[WORK] cpu_work started (pure CPU loop, testing preemption)...");
    let start_tick = timer::ticks();
    let mut primes_found = 0u32;
    let mut candidate = 2u32;

    // Run pure CPU prime search for ~250 ticks (2.5 seconds) without yielding
    while timer::ticks().saturating_sub(start_tick) < 250 {
        let mut is_prime = true;
        let mut d = 2u32;
        while d * d <= candidate {
            if candidate % d == 0 {
                is_prime = false;
                break;
            }
            d += 1;
        }
        if is_prime {
            primes_found += 1;
        }
        candidate += 1;
        core::hint::black_box(primes_found);
    }

    crate::println!(
        "[WORK] cpu_work done! Found {} primes (auto-reaping 16KB stack)",
        primes_found
    );
}

pub struct UiState {
    tab: Tab,
    needs_full_redraw: bool,
    needs_content_update: bool,
    // Files tab state (M7.3 VFS + Subdirectory support)
    dir_stack: Vec<(u32, String)>, // Stack of (dir_cluster, dir_name)
    files: Vec<DirEntry>,
    file_cursor: usize,
    viewing_file: bool,
    file_preview_name: String,
    file_preview_lines: Vec<String>,
    // Input tab state
    last_btn_pressed: Option<u8>,
    last_ir_code: Option<u32>,
    last_touch: Option<(u16, u16)>,
    event_count: u32,
}

impl UiState {
    pub fn new() -> Self {
        let files = if fat32::is_mounted() {
            fat32::list_dir(0)
        } else {
            Vec::new()
        };

        Self {
            tab: Tab::SysMon,
            needs_full_redraw: true,
            needs_content_update: true,
            dir_stack: Vec::new(),
            files,
            file_cursor: 0,
            viewing_file: false,
            file_preview_name: String::new(),
            file_preview_lines: Vec::new(),
            last_btn_pressed: None,
            last_ir_code: None,
            last_touch: None,
            event_count: 0,
        }
    }

    fn current_dir_cluster(&self) -> u32 {
        self.dir_stack.last().map(|(c, _)| *c).unwrap_or(0)
    }

    fn current_path_display(&self) -> String {
        if self.dir_stack.is_empty() {
            String::from("/")
        } else {
            let mut s = String::from("/");
            for (_, name) in &self.dir_stack {
                s.push_str(name);
                s.push('/');
            }
            s
        }
    }

    /// Handle an input event (button press/release, IR code, or touch).
    pub fn handle_event(&mut self, ev: InputEvent) {
        self.event_count += 1;
        match ev {
            InputEvent::ButtonPress(btn) => {
                self.last_btn_pressed = Some(btn);
                match btn {
                    0 => {
                        // Button 1 (GPIO 16): Next Tab (or close file preview)
                        if self.viewing_file {
                            self.viewing_file = false;
                        } else {
                            self.tab = self.tab.next();
                        }
                        self.needs_full_redraw = true;
                    }
                    1 => {
                        // Button 2 (GPIO 20): In FILES tab -> move cursor down; otherwise prev tab
                        if self.tab == Tab::Files && !self.viewing_file && !self.files.is_empty() {
                            self.file_cursor = (self.file_cursor + 1) % self.files.len();
                            self.needs_content_update = true;
                        } else if self.viewing_file {
                            self.viewing_file = false;
                            self.needs_full_redraw = true;
                        } else {
                            self.tab = self.tab.prev();
                            self.needs_full_redraw = true;
                        }
                    }
                    2 => {
                        // Button 3 (GPIO 21): Context Action!
                        match self.tab {
                            Tab::SysMon => {
                                // Spawn a CPU-bound worker task to demonstrate M7.2 preemption & stack reaping
                                let _ = task::spawn("cpu_work", preemptive_cpu_worker);
                                self.needs_content_update = true;
                            }
                            Tab::Files => {
                                if self.viewing_file {
                                    self.viewing_file = false;
                                } else {
                                    self.activate_selected_entry();
                                }
                                self.needs_full_redraw = true;
                            }
                            Tab::InputTest => {
                                // Cycle BCM2710 Hardware PWM0 Backlight Brightness (100% -> 75% -> 50% -> 25% -> 10%)
                                pwm::cycle_brightness();
                                self.needs_content_update = true;
                            }
                            _ => {
                                self.needs_content_update = true;
                            }
                        }
                    }
                    3 => {
                        // Button 4 (GPIO 26): Reboot back to Raspberry Pi OS (Slot A)
                        crate::println!("[UI] Button 4 pressed -> Rebooting to RPi OS...");
                        display::fill_rect(20, 90, 280, 60, colors::BLUE);
                        display::draw_rect_border(20, 90, 280, 60, 2, colors::WHITE);
                        display::draw_str(
                            44,
                            116,
                            "Rebooting to Raspberry Pi OS...",
                            colors::WHITE,
                            colors::BLUE,
                        );
                        tryboot::reboot_to_partition(0);
                    }
                    _ => {}
                }
            }
            InputEvent::ButtonRelease(_) => {
                if self.tab == Tab::InputTest {
                    self.needs_content_update = true;
                }
            }
            InputEvent::IrCode(code) => {
                self.last_ir_code = Some(code);
                if self.tab == Tab::InputTest {
                    self.needs_content_update = true;
                }
            }
            InputEvent::Touch(x, y) => {
                self.last_touch = Some((x, y));
                if (8..28).contains(&y) {
                    let new_tab = match x / 80 {
                        0 => Tab::SysMon,
                        1 => Tab::Files,
                        2 => Tab::InputTest,
                        _ => Tab::KernelLog,
                    };
                    if new_tab != self.tab {
                        self.viewing_file = false;
                        self.tab = new_tab;
                        self.needs_full_redraw = true;
                        return;
                    }
                }
                if self.tab == Tab::InputTest {
                    self.needs_content_update = true;
                }
            }
        }
    }

    /// In `2:FILES`, either enter a subdirectory (`<DIR>`) or stream-preview a file via VFS FD.
    fn activate_selected_entry(&mut self) {
        if self.files.is_empty() {
            return;
        }
        let entry = self.files[self.file_cursor].clone();

        if entry.is_dir {
            if entry.name == ".." {
                self.dir_stack.pop();
            } else {
                self.dir_stack.push((entry.first_cluster, entry.name));
            }
            let cluster = self.current_dir_cluster();
            self.files = fat32::list_dir(cluster);
            self.file_cursor = 0;
            return;
        }

        // Open file via M7.3 VFS File Descriptor Table and stream up to 384 bytes
        self.file_preview_name = entry.name.clone();
        self.file_preview_lines.clear();

        if let Some(fd) = vfs::open_entry(entry.clone(), OpenMode::ReadOnly) {
            let mut buf = [0u8; 384];
            let n = vfs::read(fd, &mut buf).unwrap_or(0);
            vfs::close(fd);

            let slice = &buf[..n];
            // Check if content is mostly printable ASCII text or binary
            let non_text = slice
                .iter()
                .filter(|&&b| b != b'\n' && b != b'\r' && b != b'\t' && !(0x20..=0x7E).contains(&b))
                .count();

            if non_text * 5 > slice.len().max(1) {
                // Binary file: render Hex + ASCII dump lines
                for chunk in slice.chunks(8).take(12) {
                    let mut line = String::with_capacity(36);
                    for &b in chunk {
                        let _ = write!(line, "{:02X} ", b);
                    }
                    while line.len() < 25 {
                        line.push(' ');
                    }
                    line.push('|');
                    for &b in chunk {
                        let ch = if (0x20..=0x7E).contains(&b) {
                            b as char
                        } else {
                            '.'
                        };
                        line.push(ch);
                    }
                    self.file_preview_lines.push(line);
                }
            } else {
                // Text file: render lines
                for line_bytes in slice.split(|&b| b == b'\n').take(12) {
                    let mut s = String::new();
                    for &b in line_bytes.iter().take(36) {
                        if b == b'\r' {
                            continue;
                        }
                        let ch = if (0x20..=0x7E).contains(&b) {
                            b as char
                        } else {
                            ' '
                        };
                        s.push(ch);
                    }
                    self.file_preview_lines.push(s);
                }
            }
        } else {
            self.file_preview_lines
                .push(String::from("(VFS failed to open file)"));
        }

        self.viewing_file = true;
    }

    /// Periodic 1Hz tick — refreshes live data on the active tab without full-screen flicker.
    pub fn on_periodic_tick(&mut self) {
        if self.tab == Tab::SysMon || self.tab == Tab::KernelLog {
            self.needs_content_update = true;
        }
    }

    /// Render any pending UI updates.
    pub fn render(&mut self) {
        if self.needs_full_redraw {
            self.draw_chrome();
            self.needs_full_redraw = false;
            self.needs_content_update = true;
        }

        if self.needs_content_update {
            self.needs_content_update = false;
            match self.tab {
                Tab::SysMon => self.draw_sysmon_content(),
                Tab::Files => self.draw_files_content(),
                Tab::InputTest => self.draw_input_content(),
                Tab::KernelLog => self.draw_log_content(),
            }
        }
    }

    /// Draw the tab bar (`y = 8..28`), main card frame (`y = 32..220`), and footer (`y = 224..240`).
    fn draw_chrome(&self) {
        display::fill_rect(0, 8, 320, 232, COL_BG);

        let tabs = [
            (Tab::SysMon, " 1:SYS    "),
            (Tab::Files, " 2:FILES  "),
            (Tab::InputTest, " 3:INPUT  "),
            (Tab::KernelLog, " 4:LOGS   "),
        ];
        for (i, (t, label)) in tabs.iter().enumerate() {
            let x = (i as u32) * 80;
            let active = self.tab == *t;
            let bg = if active { COL_TAB_ACTIVE } else { COL_TAB_INACTIVE };
            let fg = if active { colors::WHITE } else { COL_TEXT_DIM };
            display::fill_rect(x, 8, 79, 20, bg);
            display::draw_str(x, 14, label, fg, bg);
            if active {
                display::fill_rect(x, 26, 79, 2, COL_ACCENT);
            }
        }

        display::fill_rect(6, 32, 308, 188, COL_CARD_BG);
        display::draw_rect_border(6, 32, 308, 188, 1, COL_CARD_BORDER);

        display::fill_rect(0, 224, 320, 16, COL_FOOTER_BG);
        let footer = match self.tab {
            Tab::SysMon => " B1:Tab B2:Prev B3:+Task  B4:RPiOS    ",
            Tab::Files if !self.viewing_file => " B1:Tab B2:Down B3:Open/CD B4:RPiOS   ",
            Tab::Files => " B1/B2/B3:Close Preview   B4:RPiOS    ",
            Tab::InputTest => " B1:Tab B2:Prev B3:DimPWM B4:RPiOS    ",
            _ => " B1:NextTab B2:Prev B3:Ref B4:RPiOS   ",
        };
        display::draw_str(4, 228, footer, COL_ACCENT, COL_FOOTER_BG);
    }

    fn draw_sysmon_content(&self) {
        let mut hdr = String::with_capacity(38);
        let _ = write!(
            hdr,
            "PREEMPTIVE OS | Pre:{:<3} Reap:{:<2} ",
            task::preemptions_count(),
            task::reaped_count()
        );
        display::draw_str(14, 40, &hdr, COL_ACCENT, COL_CARD_BG);
        display::fill_rect(14, 50, 292, 1, COL_CARD_BORDER);

        // 1. SoC Temperature + ARM Clock
        let temp_mc = mailbox::get_soc_temp_mc().unwrap_or(42_500);
        let temp_c = temp_mc / 1000;
        let temp_frac = (temp_mc % 1000) / 100;
        let arm_mhz = mailbox::get_arm_clock_hz().unwrap_or(1_000_000_000) / 1_000_000;

        let mut line = String::with_capacity(40);
        let _ = write!(
            line,
            "SoC Temp : {}.{} C   ARM: {:>4} MHz  ",
            temp_c, temp_frac, arm_mhz
        );
        display::draw_str(14, 58, &line, colors::WHITE, COL_CARD_BG);

        let temp_pct = temp_c.saturating_sub(25).min(60) * 100 / 60;
        let temp_col = if temp_c < 55 {
            colors::GREEN
        } else if temp_c < 70 {
            colors::YELLOW
        } else {
            colors::RED
        };
        display::draw_progress_bar(14, 70, 292, 10, temp_pct, temp_col, COL_BG, COL_CARD_BORDER);

        // 2. Physical Memory Usage
        let free_pages = page_alloc::free_count();
        let free_mb = (free_pages * page_alloc::PAGE_SIZE) / (1024 * 1024);
        let total_mb = 256usize;
        let used_mb = total_mb.saturating_sub(free_mb);
        let mem_pct = ((used_mb * 100) / total_mb).max(4) as u32;

        line.clear();
        let _ = write!(
            line,
            "RAM Pool : {} MB free / {} MB ({}p)",
            free_mb, total_mb, free_pages
        );
        display::draw_str(14, 88, &line, colors::WHITE, COL_CARD_BG);
        display::draw_progress_bar(
            14,
            100,
            292,
            10,
            mem_pct,
            COL_ACCENT,
            COL_BG,
            COL_CARD_BORDER,
        );

        // 3. Preemptive Multitasking Scheduler Table
        display::draw_str(
            14,
            118,
            "PID NAME       STATE    TICKS   PRE ",
            COL_ACCENT,
            COL_CARD_BG,
        );
        display::fill_rect(14, 128, 292, 1, COL_CARD_BORDER);

        let mut row_y = 134u32;
        task::with_tasks(|t| {
            if row_y + 8 > 214 {
                return;
            }
            let st = match t.state {
                task::TaskState::Running => "RUN  ",
                task::TaskState::Ready => "READY",
                task::TaskState::Sleeping(_) => "SLEEP",
                task::TaskState::Finished => "ZOMBI",
                task::TaskState::Unused => "FREE ",
            };
            let mut tline = String::with_capacity(40);
            let _ = write!(
                tline,
                " #{:<1} {:<10} {:<6} {:>7} {:>4}  ",
                t.id, t.name, st, t.ticks_run, t.preemptions
            );
            display::draw_str(14, row_y, &tline, colors::WHITE, COL_CARD_BG);
            row_y += 12;
        });

        // Clear any leftover task rows when a spawned worker task gets reaped
        while row_y + 8 <= 214 {
            display::draw_str(
                14,
                row_y,
                "                                    ",
                colors::WHITE,
                COL_CARD_BG,
            );
            row_y += 12;
        }
    }

    fn draw_files_content(&self) {
        if self.viewing_file {
            let mut title = String::with_capacity(38);
            let _ = write!(
                title,
                "VFS FD: {:<18} (Ops:{})",
                self.file_preview_name,
                vfs::total_ops()
            );
            display::draw_str(14, 40, &title, colors::YELLOW, COL_CARD_BG);
            display::fill_rect(14, 50, 292, 1, COL_CARD_BORDER);

            for i in 0..12 {
                let y = 56 + (i as u32) * 13;
                let text = self
                    .file_preview_lines
                    .get(i)
                    .map(|s| s.as_str())
                    .unwrap_or("");
                let mut padded = String::with_capacity(36);
                let _ = write!(padded, "{:<36}", text);
                display::draw_str(14, y, &padded, colors::WHITE, COL_CARD_BG);
            }
            return;
        }

        let path_str = self.current_path_display();
        let mut hdr = String::with_capacity(38);
        let _ = write!(
            hdr,
            "VFS {:<18} ({:>2} items) ",
            path_str,
            self.files.len()
        );
        display::draw_str(14, 40, &hdr, COL_ACCENT, COL_CARD_BG);
        display::fill_rect(14, 50, 292, 1, COL_CARD_BORDER);

        if self.files.is_empty() {
            display::draw_str(
                14,
                70,
                "Empty directory or unmounted.",
                COL_TEXT_DIM,
                COL_CARD_BG,
            );
            return;
        }

        let visible = 10usize;
        let start = if self.file_cursor >= visible {
            self.file_cursor + 1 - visible
        } else {
            0
        };

        for row in 0..visible {
            let idx = start + row;
            let y = 56 + (row as u32) * 15;
            if idx < self.files.len() {
                let entry = &self.files[idx];
                let selected = idx == self.file_cursor;
                let bg = if selected { COL_SEL_BG } else { COL_CARD_BG };
                let fg = if selected {
                    colors::YELLOW
                } else if entry.is_dir {
                    colors::CYAN
                } else {
                    colors::WHITE
                };
                let prefix = if selected { ">" } else { " " };
                let mut line = String::with_capacity(38);
                if entry.is_dir {
                    let _ = write!(
                        line,
                        "{} [{:<13}]     <DIR>   c:{:<4}",
                        prefix, entry.name, entry.first_cluster
                    );
                } else {
                    let _ = write!(
                        line,
                        "{} {:<15} {:>9} B  c:{:<4}",
                        prefix, entry.name, entry.size, entry.first_cluster
                    );
                }
                display::draw_str(12, y, &line, fg, bg);
            } else {
                display::draw_str(
                    12,
                    y,
                    "                                    ",
                    COL_TEXT_DIM,
                    COL_CARD_BG,
                );
            }
        }
    }

    fn draw_input_content(&self) {
        display::draw_str(
            14,
            40,
            "ARGON POD INPUT & SENSOR TESTER",
            COL_ACCENT,
            COL_CARD_BG,
        );
        display::fill_rect(14, 50, 292, 1, COL_CARD_BORDER);

        let btn_mask = input::button_state();
        let pins = [16u32, 20, 21, 26];
        for i in 0..4usize {
            let x = 16 + (i as u32) * 73;
            let y = 60;
            let pressed = (btn_mask & (1 << i)) != 0;
            let box_bg = if pressed { colors::GREEN } else { COL_BG };
            let box_fg = if pressed { colors::BLACK } else { colors::WHITE };
            display::fill_rect(x, y, 64, 38, box_bg);
            display::draw_rect_border(x, y, 64, 38, 1, COL_CARD_BORDER);

            let mut lbl = String::with_capacity(10);
            let _ = write!(lbl, " BTN {} ", i + 1);
            display::draw_str(x + 4, y + 8, &lbl, box_fg, box_bg);

            lbl.clear();
            let _ = write!(lbl, "GPIO {:>2}", pins[i]);
            display::draw_str(x + 4, y + 22, &lbl, box_fg, box_bg);
        }

        let mut line = String::with_capacity(38);
        match self.last_btn_pressed {
            Some(b) => {
                let _ = write!(
                    line,
                    "Last Button : BTN {} (Events: {:>4})  ",
                    b + 1,
                    self.event_count
                );
            }
            None => {
                let _ = write!(
                    line,
                    "Last Button : None  (Events: {:>4})  ",
                    self.event_count
                );
            }
        }
        display::draw_str(14, 112, &line, colors::WHITE, COL_CARD_BG);

        line.clear();
        match self.last_ir_code {
            Some(code) => {
                let addr = (code >> 8) & 0xFF;
                let cmd = code & 0xFF;
                let _ = write!(
                    line,
                    "IR (GPIO23) : 0x{:04X} (A:{:02X} C:{:02X})   ",
                    code, addr, cmd
                );
            }
            None => {
                let _ = write!(line, "IR (GPIO23) : Waiting for remote... ");
            }
        }
        display::draw_str(14, 134, &line, colors::CYAN, COL_CARD_BG);

        line.clear();
        match self.last_touch {
            Some((tx, ty)) => {
                let _ = write!(
                    line,
                    "Touch (CE1) : X={:>3}, Y={:>3} (Active) ",
                    tx, ty
                );
            }
            None => {
                let _ = write!(line, "Touch (CE1) : Touch screen to test  ");
            }
        }
        display::draw_str(14, 156, &line, colors::YELLOW, COL_CARD_BG);

        line.clear();
        let pct = pwm::brightness();
        let _ = write!(
            line,
            "PWM0 Backlight : {:>3}% (10.1kHz GPIO18)",
            pct
        );
        display::draw_str(14, 178, &line, COL_ACCENT, COL_CARD_BG);
        display::draw_progress_bar(
            14,
            192,
            292,
            12,
            pct,
            colors::YELLOW,
            COL_BG,
            COL_CARD_BORDER,
        );
    }

    fn draw_log_content(&self) {
        display::draw_str(
            14,
            40,
            "KERNEL CONSOLE RING BUFFER",
            COL_ACCENT,
            COL_CARD_BG,
        );
        display::fill_rect(14, 50, 292, 1, COL_CARD_BORDER);

        console::with_recent_logs(15, |i, line_str| {
            let y = 56 + (i as u32) * 10;
            let mut padded = String::with_capacity(36);
            let clipped = if line_str.len() > 36 {
                &line_str[..36]
            } else {
                line_str
            };
            let _ = write!(padded, "{:<36}", clipped);
            display::draw_str(14, y, &padded, colors::GREEN, COL_CARD_BG);
        });
    }
}

/// Interactive UI & Input Task — runs at 25Hz (every 40ms), polling buttons, IR, and Touch.
pub fn ui_task() {
    task::sleep_ms(1500);

    console::set_gui_mode(true);
    let mut ui = UiState::new();
    ui.render();

    let mut tick_div = 0u32;
    loop {
        let mut had_event = false;
        while let Some(ev) = input::poll_event() {
            ui.handle_event(ev);
            had_event = true;
        }

        if let Some((tx, ty)) = touch::poll_touch() {
            ui.handle_event(InputEvent::Touch(tx, ty));
            had_event = true;
        }

        tick_div += 1;
        if tick_div >= 25 {
            tick_div = 0;
            ui.on_periodic_tick();
        }

        if had_event || tick_div == 0 {
            ui.render();
        }

        task::sleep_ms(40);
    }
}

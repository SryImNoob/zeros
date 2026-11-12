// ZerOS — Bluetooth A2DP Manager & SD Card Chinese Media Library Application (`3.蓝牙`)
//
// Features:
//   - Bluetooth 5.0 HCI / BLE & A2DP status card with one-tap re-scan (`4.搜蓝牙`)
//   - Live Synopsys DWC2 USB 2.0 Hub & Broadcom `BCM43430B0` Wi-Fi SDIO1 hardware summary
//   - FAT32 Long File Name (LFN UTF-8) Chinese Media Library browser (`1.上移`, `2.下移`, `3.打开`)
//   - Dispatches `AppIntent::PlayTrack` for `.MP3`/`.WAV`/`.LRC` and `AppIntent::OpenBook` for `.TXT`

use alloc::string::String;
use alloc::vec::Vec;
use core::fmt::Write;

use crate::apps::{App, AppId, AppIntent};
use crate::drivers::display::{colors, Color};
use crate::drivers::{bluetooth, usb, wifi};
use crate::fs::fat32::{self, DirEntry};
use crate::gfx::Canvas;

pub struct MediaBtApp {
    files: Vec<DirEntry>,
    file_cursor: usize,
    file_scroll: usize,
}

impl MediaBtApp {
    pub fn new() -> Self {
        let files = Self::load_sorted_dir(0);
        Self {
            files,
            file_cursor: 0,
            file_scroll: 0,
        }
    }

    fn load_sorted_dir(cluster: u32) -> Vec<DirEntry> {
        if !fat32::is_mounted() {
            return Vec::new();
        }
        let mut list = fat32::list_dir(cluster);
        list.sort_by_key(|e| {
            let up = e.display_name.to_ascii_uppercase();
            if up.ends_with(".MP3") || up.ends_with(".WAV") {
                0
            } else if up.ends_with(".TXT") || up.ends_with(".LRC") {
                1
            } else if e.is_dir {
                2
            } else {
                3
            }
        });
        list
    }

    fn open_selected_file(&mut self) -> AppIntent {
        if self.files.is_empty() {
            return AppIntent::None;
        }
        let entry = self.files[self.file_cursor].clone();
        if entry.is_dir {
            self.files = Self::load_sorted_dir(entry.first_cluster);
            self.file_cursor = 0;
            self.file_scroll = 0;
            return AppIntent::None;
        }

        let up = entry.display_name.to_ascii_uppercase();
        if up.ends_with(".MP3") || up.ends_with(".WAV") || up.ends_with(".LRC") {
            let clean_title = entry
                .display_name
                .trim_end_matches(".mp3")
                .trim_end_matches(".MP3")
                .trim_end_matches(".lrc")
                .trim_end_matches(".LRC");
            AppIntent::PlayTrack {
                title: String::from(clean_title),
            }
        } else {
            let max_len = (entry.size as usize).min(32 * 1024);
            let mut buf = alloc::vec![0u8; max_len];
            let n = fat32::read_entry_range(&entry, 0, &mut buf);
            buf.truncate(n);
            let content = core::str::from_utf8(&buf).unwrap_or("(二进制文件，无法作为文本阅读)");
            AppIntent::OpenBook {
                title: entry.display_name,
                content: String::from(content),
            }
        }
    }
}

impl App for MediaBtApp {
    fn id(&self) -> AppId {
        AppId::MediaBt
    }

    fn tab_label(&self) -> &'static str {
        "3.蓝牙"
    }

    fn accent_color(&self) -> Color {
        colors::rgb(175, 115, 255)
    }

    fn render(&mut self, canvas: &mut Canvas) {
        // Top Card: Bluetooth Audio & Wireless/USB Status
        canvas.draw_card(
            6,
            26,
            308,
            78,
            6,
            colors::rgb(19, 28, 46),
            colors::rgb(50, 74, 114),
        );
        canvas.draw_mixed_text(
            14,
            32,
            "蓝牙音频输出 (A2DP Source)",
            colors::rgb(0, 215, 255),
            14,
        );
        canvas.fill_rrect(236, 31, 70, 17, 4, colors::rgb(0, 130, 215));
        canvas.draw_mixed_text(242, 33, "4.搜蓝牙", colors::WHITE, 12);

        // Row 1: Onboard BCM43430B0 Bluetooth + Wi-Fi status
        let bt = bluetooth::status();
        canvas.fill_rrect(14, 52, 292, 22, 4, colors::rgb(18, 46, 48));
        canvas.stroke_rrect(14, 52, 292, 22, 4, colors::rgb(50, 210, 140));
        canvas.fill_circle(24, 63, 3, colors::rgb(50, 240, 140));
        canvas.draw_str_ascii(34, 59, "ZerOS-Pi", colors::WHITE);
        let mut bt_desc = String::with_capacity(40);
        if bt.ble_advertising {
            let _ = write!(bt_desc, "b{:04x} · 蓝牙5.0广播中", bt.hci_revision & 0x0FFF);
        } else {
            let _ = write!(bt_desc, "b{:04x} · 点击右上角搜索", bt.hci_revision & 0x0FFF);
        }
        canvas.draw_mixed_text(108, 56, &bt_desc, colors::rgb(90, 245, 165), 12);

        // Row 2: USB 2.0 Hub + Wi-Fi SDIO1 summary
        canvas.fill_rrect(14, 77, 292, 20, 4, colors::rgb(15, 22, 36));
        let usb_st = usb::refresh_status();
        let wf_st = wifi::status();
        let mut hw_line = String::with_capacity(40);
        if usb_st.downstream_count > 0 {
            let (vid, pid) = usb_st.downstream_devs[0]
                .and_then(|p| p.device)
                .map(|d| (d.vid, d.pid))
                .unwrap_or((0, 0));
            let _ = write!(
                hw_line,
                "USB:{}Dev ({:04x}:{:04x}) | WiFi:{:X}",
                usb_st.downstream_count, vid, pid, wf_st.chip_id
            );
        } else {
            let _ = write!(hw_line, "USB: Hub OK | WiFi: BCM{:X}", wf_st.chip_id);
        }
        canvas.draw_str_ascii(20, 83, &hw_line, colors::rgb(185, 205, 235));

        // Bottom Card: SD Card Chinese Media Library (MP3 Songs + TXT E-Books + Files)
        canvas.draw_card(
            6,
            110,
            308,
            124,
            6,
            colors::rgb(19, 28, 46),
            colors::rgb(50, 74, 114),
        );
        canvas.draw_mixed_text(
            14,
            116,
            "媒体库 (1.上移 2.下移 3.打开)",
            colors::rgb(255, 205, 75),
            12,
        );

        let visible = 4usize;
        for row in 0..visible {
            let idx = self.file_scroll + row;
            let ry = 136 + (row as i32) * 23;
            if let Some(entry) = self.files.get(idx) {
                let sel = idx == self.file_cursor;
                if sel {
                    canvas.fill_rrect(14, ry, 292, 21, 4, colors::rgb(0, 90, 165));
                    canvas.stroke_rrect(14, ry, 292, 21, 4, colors::rgb(0, 210, 255));
                } else {
                    canvas.fill_rrect(14, ry, 292, 21, 4, colors::rgb(15, 22, 36));
                }

                let up = entry.display_name.to_ascii_uppercase();
                let (prefix, col) = if entry.is_dir {
                    ("[目录] ", colors::rgb(255, 205, 85))
                } else if up.ends_with(".MP3") || up.ends_with(".WAV") || up.ends_with(".LRC") {
                    ("♪ ", colors::rgb(190, 140, 255))
                } else if up.ends_with(".TXT") {
                    ("书 ", colors::rgb(255, 205, 85))
                } else {
                    ("· ", colors::rgb(195, 215, 240))
                };

                let mut label = String::from(prefix);
                label.push_str(&entry.display_name);
                canvas.draw_mixed_text(
                    20,
                    ry + 3,
                    &label,
                    if sel { colors::WHITE } else { col },
                    14,
                );

                let mut sz_str = String::with_capacity(12);
                if entry.is_dir {
                    sz_str.push_str("<DIR>");
                } else if entry.size >= 1024 * 1024 {
                    let _ = write!(
                        sz_str,
                        "{}.{}M",
                        entry.size / (1024 * 1024),
                        (entry.size % (1024 * 1024)) / 102_400
                    );
                } else if entry.size >= 1024 {
                    let _ = write!(sz_str, "{}K", entry.size / 1024);
                } else {
                    let _ = write!(sz_str, "{}B", entry.size);
                }
                let sx = 298 - (sz_str.len() as i32) * 8;
                canvas.draw_str_ascii(sx, ry + 6, &sz_str, colors::rgb(220, 235, 255));
            }
        }
    }

    fn on_slot_action(&mut self, slot: usize) -> AppIntent {
        match slot {
            0 => {
                // 1.上移 ▲
                if !self.files.is_empty() {
                    if self.file_cursor > 0 {
                        self.file_cursor -= 1;
                    } else {
                        self.file_cursor = self.files.len() - 1;
                    }
                    if self.file_cursor < self.file_scroll {
                        self.file_scroll = self.file_cursor;
                    } else if self.file_cursor >= self.file_scroll + 4 {
                        self.file_scroll = self.file_cursor - 3;
                    }
                }
                AppIntent::None
            }
            1 => {
                // 2.下移 ▼
                if !self.files.is_empty() {
                    self.file_cursor = (self.file_cursor + 1) % self.files.len();
                    if self.file_cursor < self.file_scroll {
                        self.file_scroll = self.file_cursor;
                    } else if self.file_cursor >= self.file_scroll + 4 {
                        self.file_scroll = self.file_cursor - 3;
                    }
                }
                AppIntent::None
            }
            2 => {
                // 3.打开 ⏎
                self.open_selected_file()
            }
            _ => {
                // 4.搜蓝牙
                bluetooth::init();
                AppIntent::None
            }
        }
    }

    fn on_touch(&mut self, tx: i32, ty: i32) -> AppIntent {
        // Top card [4.搜蓝牙] button (x = 230..310, y = 28..52)
        if ty >= 28 && ty <= 52 && tx >= 230 {
            bluetooth::init();
            return AppIntent::None;
        }
        // Bottom media library rows (y = 136..228, 4 visible rows × 23px)
        if ty >= 136 && ty < 228 {
            let row = ((ty - 136) / 23) as usize;
            let target_idx = self.file_scroll + row;
            if target_idx < self.files.len() {
                if self.file_cursor == target_idx {
                    return self.open_selected_file();
                } else {
                    self.file_cursor = target_idx;
                }
            }
        }
        AppIntent::None
    }

    fn on_tick_200ms(&mut self, _sub_tick: u32, _is_active: bool) -> bool {
        false
    }
}

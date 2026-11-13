// ZerOS — Dedicated Bluetooth Device Scanner & A2DP/ACL Connection Manager (`3.蓝牙`) (`M10.1`)
//
// Features:
//   - Full-screen scrollable Bluetooth Device List (6 visible rows × 24px, up to 16 devices)
//   - Displays device index, category tag (`[音箱]`, `[手机]`, `[BLE]`, `[经典]`), UTF-8 name,
//     signal strength (`RSSI dBm`), and connection state (`已连接`)
//   - Bottom 4 action buttons mapped 1:1 to `B1..B4` short-press:
//       * `B1` (`1.上移`) : Move selection cursor up in the Bluetooth list
//       * `B2` (`2.下移`) : Move selection cursor down in the Bluetooth list
//       * `B3` (`3.连接`) : Initiate ACL + SSP pairing/connection to the selected device
//       * `B4` (`4.搜索`) : Scan nearby Classic Bluetooth + BLE devices

use alloc::string::String;
use core::fmt::Write;

use crate::apps::{App, AppId, AppIntent};
use crate::drivers::display::{colors, Color};
use crate::drivers::{bluetooth, usb, wifi};
use crate::gfx::Canvas;

const VISIBLE_ROWS: usize = 6;

pub struct MediaBtApp {
    cursor: usize,
    scroll: usize,
    status_banner: String,
    is_busy: bool,
}

impl MediaBtApp {
    pub fn new() -> Self {
        Self {
            cursor: 0,
            scroll: 0,
            status_banner: String::from("按 B4 [4.搜索] 扫描周围蓝牙设备"),
            is_busy: false,
        }
    }

    pub fn set_scanning_banner(&mut self) {
        self.is_busy = true;
        self.status_banner = String::from("正在搜索周围经典蓝牙与BLE设备 (约6秒)...");
    }

    pub fn on_scan_finished(&mut self, found: usize) {
        self.is_busy = false;
        if found > 0 {
            self.cursor = 0;
            self.scroll = 0;
            self.status_banner.clear();
            let _ = write!(
                self.status_banner,
                "共搜到 {} 台设备 · 上下选择后按 3.连接",
                found
            );
        } else {
            self.status_banner = String::from("未搜到设备 · 请确认音箱已开机并可配对");
        }
    }

    pub fn set_connecting_banner(&mut self, idx: usize) {
        self.is_busy = true;
        let st = bluetooth::status();
        self.status_banner.clear();
        if idx < st.device_count {
            let _ = write!(
                self.status_banner,
                "正在配对连接: {} ...",
                st.devices[idx].name_str()
            );
        } else {
            self.status_banner.push_str("正在连接所选蓝牙设备...");
        }
    }

    pub fn on_connect_finished(&mut self, idx: usize, ok: bool) {
        self.is_busy = false;
        let st = bluetooth::status();
        self.status_banner.clear();
        if ok && idx < st.device_count {
            if st.devices[idx].is_ble && st.devices[idx].is_audio && !st.a2dp_ready {
                let _ = write!(
                    self.status_banner,
                    "仅连BLE(无声) · 请按音箱蓝牙键快闪再点4.搜索"
                );
            } else {
                let mode = if st.a2dp_ready && !st.devices[idx].is_ble {
                    "A2DP音频"
                } else if st.devices[idx].is_ble {
                    "BLE/GATT"
                } else {
                    "经典蓝牙"
                };
                let _ = write!(
                    self.status_banner,
                    "已连接 {}: {} (0x{:02X})",
                    mode,
                    st.devices[idx].name_str(),
                    st.acl_handle & 0xFF
                );
            }
        } else {
            let reason = match st.last_conn_err {
                0x04 => "请短按音箱蓝牙键至蓝灯快闪两下",
                0x05 | 0x06 | 0x18 => "音箱拒绝配对(请短按蓝牙键快闪)",
                0x08 => "连接超时",
                0x0D => "请勿开STPAIR白灯·短按蓝牙键快闪",
                0x13 => "音箱主动断开连接",
                0xE1 => "ACL已连·等待SSP配对(请短按快闪)",
                0xE2 => "SSP已过·等待链路加密",
                0xE3 => "已加密·等待L2CAP通道",
                0xE4 => "L2CAP已开·等待AVDTP协商",
                0xE5 => "AVDTP已开·等待音频流通道",
                _ => "配对未完成",
            };
            let _ = write!(
                self.status_banner,
                "连接失败(0x{:02X}): {}",
                st.last_conn_err, reason
            );
        }
    }

    fn sync_scroll(&mut self, count: usize) {
        if count == 0 {
            self.cursor = 0;
            self.scroll = 0;
            return;
        }
        if self.cursor >= count {
            self.cursor = count - 1;
        }
        if self.cursor < self.scroll {
            self.scroll = self.cursor;
        } else if self.cursor >= self.scroll + VISIBLE_ROWS {
            self.scroll = self.cursor + 1 - VISIBLE_ROWS;
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
        let bt = bluetooth::status();
        let count = bt.device_count;
        self.sync_scroll(count);

        // ─── Main Card: Full-Screen Scrollable Bluetooth Device List (y = 25..200) ───
        canvas.draw_card(
            6,
            25,
            308,
            175,
            6,
            colors::rgb(19, 28, 46),
            colors::rgb(50, 74, 114),
        );

        // Header line (y = 29..45)
        let hdr_col = if self.is_busy {
            colors::rgb(0, 235, 255)
        } else {
            colors::rgb(255, 205, 75)
        };
        canvas.draw_mixed_text(14, 30, &self.status_banner, hdr_col, 12);

        if count > 0 && !self.is_busy {
            let mut idx_badge = String::with_capacity(12);
            let _ = write!(idx_badge, "{}/{}", self.cursor + 1, count);
            let bw = (idx_badge.len() as i32) * 8;
            canvas.draw_str_ascii(304 - bw, 32, &idx_badge, colors::rgb(160, 190, 230));
        }
        canvas.draw_line(14, 46, 306, 46, colors::rgb(36, 54, 84));

        if count == 0 {
            // Empty-state prompt card before first scan
            canvas.fill_rrect(16, 56, 288, 66, 6, colors::rgb(14, 21, 35));
            canvas.stroke_rrect(16, 56, 288, 66, 6, colors::rgb(45, 72, 115));

            if self.is_busy {
                canvas.draw_mixed_text(
                    42,
                    70,
                    "正在主动扫描周围 2.4GHz 蓝牙设备...",
                    colors::rgb(0, 230, 255),
                    15,
                );
                canvas.draw_mixed_text(
                    48,
                    95,
                    "包含经典蓝牙音箱/耳机与 BLE 广播外设",
                    colors::rgb(160, 190, 230),
                    12,
                );
            } else {
                canvas.draw_mixed_text(
                    52,
                    68,
                    "点击下方 [4.搜索] 扫描周围蓝牙",
                    colors::WHITE,
                    15,
                );
                canvas.draw_mixed_text(
                    42,
                    94,
                    "支持搜索蓝牙音箱、耳机、手机与 BLE 外设",
                    colors::rgb(150, 180, 220),
                    12,
                );
            }

            // Local controller & wireless hardware status box (y = 132..190)
            canvas.fill_rrect(16, 132, 288, 58, 5, colors::rgb(15, 36, 40));
            canvas.stroke_rrect(16, 132, 288, 58, 5, colors::rgb(45, 185, 130));
            canvas.fill_circle(26, 146, 3, colors::rgb(50, 240, 140));
            let mut local_line = String::with_capacity(48);
            let _ = write!(
                local_line,
                "本机: ZerOS-Pi (固件 b{:04x} · 广播中)",
                bt.hci_revision & 0x0FFF
            );
            canvas.draw_mixed_text(35, 139, &local_line, colors::rgb(110, 250, 175), 12);

            let usb_st = usb::refresh_status();
            let wf_st = wifi::status();
            let mut hw_line = String::with_capacity(48);
            let _ = write!(
                hw_line,
                "MAC {:02X}:{:02X}:{:02X} | WiFi {:X} | USB {}口",
                bt.bd_addr[3],
                bt.bd_addr[4],
                bt.bd_addr[5],
                wf_st.chip_id,
                usb_st.downstream_count
            );
            canvas.draw_mixed_text(35, 164, &hw_line, colors::rgb(185, 210, 240), 12);
        } else {
            // Render 6 visible rows of discovered Bluetooth devices (y = 50..194)
            for row in 0..VISIBLE_ROWS {
                let idx = self.scroll + row;
                let ry = 50 + (row as i32) * 24;
                if idx < count {
                    let dev = &bt.devices[idx];
                    let sel = idx == self.cursor;

                    if sel {
                        canvas.fill_rrect(14, ry, 286, 22, 4, colors::rgb(0, 95, 175));
                        canvas.stroke_rrect(14, ry, 286, 22, 4, colors::rgb(80, 220, 255));
                    } else if dev.connected {
                        canvas.fill_rrect(14, ry, 286, 22, 4, colors::rgb(18, 56, 48));
                        canvas.stroke_rrect(14, ry, 286, 22, 4, colors::rgb(50, 225, 145));
                    } else {
                        let row_bg = if (row & 1) == 0 {
                            colors::rgb(15, 22, 36)
                        } else {
                            colors::rgb(17, 25, 41)
                        };
                        canvas.fill_rrect(14, ry, 286, 22, 4, row_bg);
                    }

                    // Left: "1.[音箱] 设备名称"
                    let mut label = String::with_capacity(48);
                    let _ = write!(label, "{}.{}", idx + 1, dev.category_tag());
                    label.push_str(dev.name_str());

                    let text_col = if sel {
                        colors::WHITE
                    } else if dev.connected {
                        colors::rgb(100, 255, 175)
                    } else if dev.is_audio {
                        colors::rgb(0, 225, 255)
                    } else if dev.has_real_name {
                        colors::rgb(230, 238, 250)
                    } else {
                        colors::rgb(170, 190, 220)
                    };
                    canvas.draw_mixed_text(18, ry + 4, &label, text_col, 14);

                    // Right: Connected pill or RSSI dBm badge
                    if dev.connected {
                        if dev.is_ble && dev.is_audio && !bt.a2dp_ready {
                            canvas.fill_rrect(246, ry + 4, 48, 14, 3, colors::rgb(185, 115, 20));
                            canvas.draw_mixed_text(250, ry + 5, "仅BLE", colors::WHITE, 12);
                        } else {
                            canvas.fill_rrect(246, ry + 4, 48, 14, 3, colors::rgb(20, 150, 95));
                            canvas.draw_mixed_text(250, ry + 5, "已连接", colors::WHITE, 12);
                        }
                    } else {
                        let mut rssi_s = String::with_capacity(8);
                        let _ = write!(rssi_s, "{}dB", dev.rssi);
                        let rx = 294 - (rssi_s.len() as i32) * 8;
                        let rssi_col = if sel {
                            colors::rgb(220, 245, 255)
                        } else if dev.rssi >= -60 {
                            colors::rgb(90, 240, 165)
                        } else {
                            colors::rgb(150, 175, 210)
                        };
                        canvas.draw_str_ascii(rx, ry + 7, &rssi_s, rssi_col);
                    }
                }
            }

            // Right scrollbar track (x = 303..307, y = 50..192)
            let track_y = 50i32;
            let track_h = 142i32;
            canvas.fill_rrect(303, track_y, 4, track_h, 2, colors::rgb(13, 19, 32));
            let thumb_h = if count <= VISIBLE_ROWS {
                track_h
            } else {
                ((track_h * (VISIBLE_ROWS as i32)) / (count as i32)).max(18)
            };
            let thumb_y = if count <= VISIBLE_ROWS {
                track_y
            } else {
                track_y
                    + ((track_h - thumb_h) * (self.scroll as i32))
                        / ((count - VISIBLE_ROWS) as i32)
            };
            canvas.fill_rrect(303, thumb_y, 4, thumb_h, 2, colors::rgb(0, 200, 255));
        }

        // ─── Bottom 4 Action Buttons (y = 204..234) — 1:1 mapped to B1..B4 short-press ───
        let btns: [(i32, i32, i32, &str, bool); 4] = [
            (8, 204, 72, "1.上移", false),
            (86, 204, 72, "2.下移", false),
            (164, 204, 72, "3.连接", true),
            (242, 204, 70, "4.搜索", false),
        ];
        for &(bx, by, bw, label, primary) in &btns {
            let bg = if primary {
                colors::rgb(0, 135, 225)
            } else {
                colors::rgb(22, 34, 56)
            };
            let bd = if primary {
                colors::rgb(100, 220, 255)
            } else {
                colors::rgb(58, 86, 130)
            };
            canvas.fill_rrect(bx, by, bw, 30, 6, bg);
            canvas.stroke_rrect(bx, by, bw, 30, 6, bd);
            let lw = Canvas::measure_mixed_text(label, 14);
            let tx = bx + (bw - lw) / 2;
            canvas.draw_mixed_text(tx, by + 8, label, colors::WHITE, 14);
        }
    }

    fn on_slot_action(&mut self, slot: usize) -> AppIntent {
        let count = bluetooth::status().device_count;
        match slot {
            0 => {
                // 1.上移 ▲
                if count > 0 {
                    if self.cursor > 0 {
                        self.cursor -= 1;
                    } else {
                        self.cursor = count - 1;
                    }
                    self.sync_scroll(count);
                }
                AppIntent::None
            }
            1 => {
                // 2.下移 ▼
                if count > 0 {
                    self.cursor = (self.cursor + 1) % count;
                    self.sync_scroll(count);
                }
                AppIntent::None
            }
            2 => {
                // 3.连接 ⏎
                if count > 0 && self.cursor < count {
                    AppIntent::ConnectBluetooth(self.cursor)
                } else {
                    AppIntent::ScanBluetooth
                }
            }
            _ => {
                // 4.搜索
                AppIntent::ScanBluetooth
            }
        }
    }

    fn on_touch(&mut self, tx: i32, ty: i32) -> AppIntent {
        let count = bluetooth::status().device_count;

        // Bottom 4 buttons (y >= 200)
        if ty >= 200 {
            let slot = ((tx - 8).max(0) / 78).clamp(0, 3) as usize;
            return self.on_slot_action(slot);
        }

        // Empty-state prompt card tap -> start scan
        if count == 0 && ty >= 50 && ty <= 130 {
            return AppIntent::ScanBluetooth;
        }

        // Tap a row in the 6-row Bluetooth device list (y = 50..194)
        if count > 0 && ty >= 50 && ty < 194 {
            let row = ((ty - 50) / 24) as usize;
            let idx = self.scroll + row;
            if idx < count {
                if self.cursor == idx {
                    return AppIntent::ConnectBluetooth(idx);
                } else {
                    self.cursor = idx;
                }
            }
        }
        AppIntent::None
    }

    fn on_tick_200ms(&mut self, _sub_tick: u32, _is_active: bool) -> bool {
        false
    }
}

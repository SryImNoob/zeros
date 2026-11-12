// ZerOS — System Control Center & Hardware Monitor Application (`4.系统`)
//
// Features:
//   - Hardware PWM0 Backlight Touch Slider (`10%..100%`) + `[退回Pi]` safe reboot button
//   - CPU & SoC Temperature circular ring gauge + 64-sample live waveform graph
//   - Preemptive worker task spawner (`3.+任务`, spawns `preemptive_cpu_worker`)
//   - 4 Bezel Button live indicators (`B1..B4`) & ADS7846 interactive touch drawing pad (`4.清画板`)

use alloc::string::String;
use alloc::vec::Vec;
use core::fmt::Write;

use crate::apps::{App, AppId, AppIntent};
use crate::arch::timer;
use crate::drivers::display::{colors, Color};
use crate::drivers::input::{self, touch};
use crate::drivers::{mailbox, pwm};
use crate::gfx::Canvas;
use crate::kernel::{task, tryboot};
use crate::mm::{heap, page_alloc};

/// Pure CPU-bound worker task spawned in `4.系统` to demonstrate 50ms hardware preemption.
fn preemptive_cpu_worker() {
    crate::println!("[WORK] cpu_work started (pure CPU loop, testing preemption)...");
    let start_tick = timer::ticks();
    let mut primes_found = 0u32;
    let mut candidate = 2u32;

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

pub struct SystemApp {
    waveform_history: [u8; 64],
    waveform_head: usize,
    anim_tick: u32,
    last_btn_pressed: Option<u8>,
    touch_trail: Vec<(i32, i32)>,
    last_touch: Option<(u16, u16)>,
}

impl SystemApp {
    pub fn new() -> Self {
        let mut wf = [35u8; 64];
        for i in 0..64 {
            wf[i] = (30 + ((i * 17) % 35)) as u8;
        }
        Self {
            waveform_history: wf,
            waveform_head: 0,
            anim_tick: 0,
            last_btn_pressed: None,
            touch_trail: Vec::with_capacity(64),
            last_touch: None,
        }
    }

    pub fn set_last_button(&mut self, btn: Option<u8>) {
        self.last_btn_pressed = btn;
    }

    pub fn set_last_touch(&mut self, pos: (u16, u16)) {
        self.last_touch = Some(pos);
    }
}

impl App for SystemApp {
    fn id(&self) -> AppId {
        AppId::System
    }

    fn tab_label(&self) -> &'static str {
        "4.系统"
    }

    fn accent_color(&self) -> Color {
        colors::rgb(65, 230, 150)
    }

    fn render(&mut self, canvas: &mut Canvas) {
        // Top Card: Hardware PWM0 Backlight Touch Slider + Reboot Button
        canvas.draw_card(
            6,
            25,
            308,
            48,
            6,
            colors::rgb(19, 28, 46),
            colors::rgb(50, 74, 114),
        );
        let bl_pct = pwm::brightness();
        let mut bl_hdr = String::with_capacity(32);
        let _ = write!(bl_hdr, "背光亮度调节 (PWM0) {}%", bl_pct);
        canvas.draw_mixed_text(14, 29, &bl_hdr, colors::rgb(255, 210, 65), 12);

        // Slider Track (x = 14..242, y = 48..60)
        canvas.fill_rrect(14, 48, 228, 12, 6, colors::rgb(13, 19, 32));
        canvas.stroke_rrect(14, 48, 228, 12, 6, colors::rgb(48, 70, 108));
        let fill_w = ((bl_pct as i32) * 224) / 100;
        canvas.fill_rrect(16, 50, fill_w.max(6), 8, 4, colors::rgb(255, 195, 45));
        canvas.fill_circle(16 + fill_w, 54, 6, colors::WHITE);
        canvas.fill_circle(16 + fill_w, 54, 3, colors::rgb(255, 175, 0));

        // [退回Pi] Button (x = 250..306, y = 44..64)
        canvas.fill_rrect(250, 44, 56, 20, 4, colors::rgb(150, 40, 50));
        canvas.stroke_rrect(250, 44, 56, 20, 4, colors::rgb(255, 110, 120));
        canvas.draw_mixed_text(255, 48, "退回Pi", colors::WHITE, 12);

        // Bottom-Left Card: CPU Ring Gauge + Live Waveform Chart + [+任务]
        canvas.draw_card(
            6,
            79,
            150,
            155,
            6,
            colors::rgb(19, 28, 46),
            colors::rgb(50, 74, 114),
        );
        canvas.draw_mixed_text(14, 84, "处理器与温度", colors::rgb(0, 215, 255), 12);
        canvas.fill_rrect(94, 83, 56, 16, 4, colors::rgb(0, 130, 215));
        canvas.draw_mixed_text(98, 85, "3.+任务", colors::WHITE, 12);

        let temp_mc = mailbox::get_soc_temp_mc().unwrap_or(42_000);
        let temp_c = temp_mc / 1000;
        let active_tasks = task::active_count();
        let load_pct = ((active_tasks as u32) * 20).min(98);

        canvas.draw_ring(
            34,
            124,
            18,
            13,
            load_pct,
            colors::rgb(0, 220, 255),
            colors::rgb(34, 48, 74),
        );
        let mut pct_s = String::with_capacity(6);
        let _ = write!(pct_s, "{}%", load_pct);
        canvas.draw_str_ascii(23, 120, &pct_s, colors::WHITE);

        let mut t_s = String::with_capacity(16);
        let _ = write!(t_s, "{}.{}C", temp_c, (temp_mc % 1000) / 100);
        canvas.draw_mixed_text(60, 106, "温度:", colors::rgb(255, 200, 70), 12);
        canvas.draw_str_ascii(96, 108, &t_s, colors::rgb(255, 200, 70));

        let mut p_s = String::with_capacity(16);
        let _ = write!(p_s, "抢占:{}", task::preemptions_count());
        canvas.draw_mixed_text(60, 124, &p_s, colors::rgb(95, 240, 160), 12);

        let free_mb = page_alloc::free_count() * page_alloc::PAGE_SIZE / (1024 * 1024);
        let (heap_used, _) = heap::stats();
        let mut m_s = String::with_capacity(20);
        let _ = write!(m_s, "页:{}M 堆:{}K", free_mb, heap_used / 1024);
        canvas.draw_mixed_text(60, 140, &m_s, colors::rgb(185, 205, 235), 12);

        // Mini Live Waveform Plot (x = 14..148, y = 160..226)
        let gx = 14i32;
        let gy = 160i32;
        let gw = 134i32;
        let gh = 66i32;
        canvas.fill_rrect(gx, gy, gw, gh, 4, colors::rgb(12, 18, 30));
        for i in 0..63 {
            let idx0 = (self.waveform_head + i) % 64;
            let idx1 = (self.waveform_head + i + 1) % 64;
            let v0 = (self.waveform_history[idx0] as i32).clamp(5, 95);
            let v1 = (self.waveform_history[idx1] as i32).clamp(5, 95);
            let x0 = gx + 3 + (i as i32) * 2;
            let x1 = x0 + 2;
            let y0 = gy + gh - 4 - (v0 * (gh - 10)) / 100;
            let y1 = gy + gh - 4 - (v1 * (gh - 10)) / 100;
            for fy in y0..(gy + gh - 2) {
                canvas.set_px_alpha(x0, fy, colors::rgb(0, 195, 255), 65);
            }
            canvas.draw_line(x0, y0, x1, y1, colors::rgb(0, 235, 255));
        }

        // Bottom-Right Card: 4 Bezel Buttons + Touch Drawing Pad
        canvas.draw_card(
            162,
            79,
            152,
            155,
            6,
            colors::rgb(19, 28, 46),
            colors::rgb(50, 74, 114),
        );
        canvas.draw_mixed_text(170, 84, "按键与触摸画板", colors::rgb(65, 235, 155), 12);

        let btn_mask = input::button_state();
        for i in 0..4 {
            let bx = 170 + (i as i32) * 35;
            let held = (btn_mask & (1 << i)) != 0 || self.last_btn_pressed == Some(i as u8);
            let bg = if held {
                colors::rgb(0, 180, 120)
            } else {
                colors::rgb(28, 40, 64)
            };
            canvas.fill_rrect(bx, 102, 31, 22, 4, bg);
            let mut bs = String::with_capacity(4);
            let _ = write!(bs, "B{}", i + 1);
            canvas.draw_str_ascii(bx + 8, 109, &bs, colors::WHITE);
        }

        // Touch Pad Area (x = 170..306, y = 130..210)
        canvas.fill_rrect(170, 130, 136, 80, 4, colors::rgb(10, 15, 26));
        canvas.stroke_rrect(170, 130, 136, 80, 4, colors::rgb(45, 70, 110));

        if self.touch_trail.is_empty() {
            let pts: [(i32, i32); 16] = [
                (210, 170),
                (216, 156),
                (228, 156),
                (238, 170),
                (248, 184),
                (260, 184),
                (266, 170),
                (260, 156),
                (248, 156),
                (238, 170),
                (228, 184),
                (216, 184),
                (210, 170),
                (216, 156),
                (228, 156),
                (238, 170),
            ];
            for i in 0..15 {
                canvas.draw_line(
                    pts[i].0,
                    pts[i].1,
                    pts[i + 1].0,
                    pts[i + 1].1,
                    colors::rgb(0, 210, 255),
                );
            }
        } else {
            for &(px, py) in &self.touch_trail {
                canvas.fill_circle(px, py, 2, colors::rgb(0, 230, 255));
            }
        }

        let mut tc_str = String::with_capacity(20);
        if let Some((tx, ty)) = self.last_touch {
            let _ = write!(
                tc_str,
                "X:{:03} Y:{:03} Pen:{}",
                tx,
                ty,
                touch::is_touched() as u8
            );
        } else {
            let _ = write!(tc_str, "Touch Pad Ready");
        }
        canvas.draw_str_ascii(172, 215, &tc_str, colors::rgb(200, 220, 250));
    }

    fn on_slot_action(&mut self, slot: usize) -> AppIntent {
        match slot {
            0 => {
                // 1.亮度 -15%
                let next = pwm::brightness().saturating_sub(15).max(10);
                pwm::set_brightness(next);
                AppIntent::None
            }
            1 => {
                // 2.亮度 +15%
                let next = (pwm::brightness() + 15).min(100);
                pwm::set_brightness(next);
                AppIntent::None
            }
            2 => {
                // 3.+任务
                task::spawn("cpu_work", preemptive_cpu_worker);
                AppIntent::None
            }
            _ => {
                // 4.清画板
                self.touch_trail.clear();
                AppIntent::None
            }
        }
    }

    fn on_touch(&mut self, tx: i32, ty: i32) -> AppIntent {
        // Top card: PWM Backlight Slider (x = 14..246, y = 38..70)
        if ty >= 38 && ty <= 70 && tx >= 14 && tx <= 246 {
            let pct = (((tx - 16).clamp(0, 224) * 90) / 224 + 10) as u32;
            pwm::set_brightness(pct);
            return AppIntent::None;
        }
        // Top card: [退回Pi] button (x = 250..312, y = 38..70)
        if ty >= 38 && ty <= 70 && tx >= 250 {
            tryboot::reboot_to_partition(0);
        }
        // Bottom-left card: [3.+任务] button (x = 92..156, y = 80..106)
        if ty >= 80 && ty <= 106 && tx >= 92 && tx <= 156 {
            task::spawn("cpu_work", preemptive_cpu_worker);
            return AppIntent::None;
        }
        // Bottom-right card: Touch drawing pad (x = 172..306, y = 132..210)
        if tx >= 172 && tx <= 306 && ty >= 132 && ty <= 210 {
            if self.touch_trail.len() >= 64 {
                self.touch_trail.remove(0);
            }
            self.touch_trail.push((tx, ty));
        }
        AppIntent::None
    }

    fn on_tick_200ms(&mut self, sub_tick: u32, is_active: bool) -> bool {
        self.anim_tick = self.anim_tick.wrapping_add(1);
        if sub_tick % 5 == 0 {
            let temp_c = (mailbox::get_soc_temp_mc().unwrap_or(42_000) / 1000) as u8;
            let sample = ((temp_c.saturating_sub(25)) * 2 + ((self.anim_tick as u8 * 13) % 25))
                .clamp(15, 92);
            self.waveform_history[self.waveform_head] = sample;
            self.waveform_head = (self.waveform_head + 1) % 64;
            return is_active;
        }
        is_active
    }
}

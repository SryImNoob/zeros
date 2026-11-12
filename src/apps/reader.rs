// ZerOS — Immersive UTF-8 Chinese E-Book Reader Application (`2.看书`)
//
// Features:
//   - Crisp 15x15 4-level anti-aliased Chinese typography (`18` CJK chars/row × `8` rows = `144` chars/page)
//   - Automatic paragraph indentation & mixed UTF-8 CJK + ASCII word-wrap engine
//   - Loads `/SANTI.TXT` (`《三体 I：地球往事》`) from FAT32 SD card at boot, or opens any `.TXT` from `3.蓝牙`
//   - Touch left/right half of page to turn pages, plus 1:1 `B1..B4` bezel button controls

use alloc::string::String;
use alloc::vec::Vec;
use core::fmt::Write;

use crate::apps::{App, AppId, AppIntent};
use crate::drivers::display::{colors, Color};
use crate::drivers::pwm;
use crate::fs::fat32;
use crate::gfx::Canvas;

const DEFAULT_SANTI_BOOK: &str = "\
汪淼觉得，来找他的这两个人是一个奇怪的组合：一名警察和一名军人。那名警察叫史强，长得五大三粗，满脸横肉，穿着件脏兮兮的皮夹克，浑身烟味，说话粗声大嗓，怎么看都不像一名人民警察。而那名少将常伟思则完全相反，身材挺拔，举止儒雅，戴着一副金丝边眼镜，透出学者般的沉稳与冷静。\n\
“汪教授，请坐。”常伟思微笑着示意，“这次请你来，是想向你了解一些关于‘科学边界’学会的情况。最近几个月，全球各地接连发生了多起顶尖物理学家自杀的事件，包括杨冬教授。我们注意到，他们在生前都曾与‘科学边界’有过接触。”\n\
汪淼点点头：“我确实认识丁仪和杨冬，但我并没有加入那个学会。在纳米材料研究领域，我更习惯依靠实验室里看得见的数据，而不是空谈哲学假说。”\n\
史强掐灭了烟头，咧嘴一笑：“汪教授，有时候眼睛看见的东西，未必就是真的。要是有一天，连你们奉若神明的物理定律都开始闪烁不定，你还能坐得住吗？”\n\
窗外的夜色渐渐深沉，远处的城市灯火在寒风中微微摇曳。汪淼并不知道，就在这一晚之后，他的视网膜上将会浮现出一串令整个科学界陷入恐惧的倒计时……";

pub struct ReaderApp {
    book_title: String,
    book_lines: Vec<String>,
    book_page: usize,
}

impl ReaderApp {
    pub fn new() -> Self {
        let (book_title, book_raw) = if fat32::is_mounted() {
            if let Some(bytes) = fat32::read_file("/SANTI.TXT") {
                if let Ok(s) = core::str::from_utf8(&bytes) {
                    (
                        String::from("《三体 I：地球往事》 刘慈欣"),
                        String::from(s),
                    )
                } else {
                    (
                        String::from("《三体 I：地球往事》 第一章 科学边界"),
                        String::from(DEFAULT_SANTI_BOOK),
                    )
                }
            } else {
                (
                    String::from("《三体 I：地球往事》 第一章 科学边界"),
                    String::from(DEFAULT_SANTI_BOOK),
                )
            }
        } else {
            (
                String::from("《三体 I：地球往事》 第一章 科学边界"),
                String::from(DEFAULT_SANTI_BOOK),
            )
        };

        let book_lines = Self::layout_ebook_lines(&book_raw, 36);
        Self {
            book_title,
            book_lines,
            book_page: 0,
        }
    }

    pub fn open_book(&mut self, title: String, content: &str) {
        self.book_title = title;
        self.book_lines = Self::layout_ebook_lines(content, 36);
        self.book_page = 0;
    }

    fn total_pages(&self) -> usize {
        ((self.book_lines.len() + 7) / 8).max(1)
    }

    /// Word-wrap UTF-8 Chinese + ASCII text into lines of at most `max_half_widths`
    /// (`36` half-widths = `18` full-width `15x15` Chinese characters per row).
    fn layout_ebook_lines(text: &str, max_half_widths: usize) -> Vec<String> {
        let mut lines = Vec::new();
        for raw_para in text.lines() {
            let para = raw_para.trim_end_matches('\r');
            if para.trim().is_empty() {
                continue;
            }
            let mut cur_line = String::new();
            let mut cur_w = 0usize;

            let trimmed = para.trim_start();
            let is_header = trimmed.starts_with('《')
                || trimmed.starts_with("第")
                || trimmed.starts_with("作者");
            if !is_header && !para.starts_with(' ') && !para.starts_with('　') {
                cur_line.push_str("  ");
                cur_w = 2;
            }

            for ch in para.chars() {
                let cw = if (ch as u32) < 128 { 1 } else { 2 };
                if cur_w + cw > max_half_widths && !cur_line.is_empty() {
                    lines.push(cur_line);
                    cur_line = String::new();
                    cur_w = 0;
                }
                cur_line.push(ch);
                cur_w += cw;
            }
            if !cur_line.is_empty() {
                lines.push(cur_line);
            }
        }
        if lines.is_empty() {
            lines.push(String::from("  (空文档)"));
        }
        lines
    }
}

impl App for ReaderApp {
    fn id(&self) -> AppId {
        AppId::Reader
    }

    fn tab_label(&self) -> &'static str {
        "2.看书"
    }

    fn accent_color(&self) -> Color {
        colors::rgb(255, 195, 55)
    }

    fn render(&mut self, canvas: &mut Canvas) {
        canvas.draw_card(
            6,
            25,
            308,
            188,
            6,
            colors::rgb(16, 23, 38),
            colors::rgb(48, 70, 108),
        );

        // Book Title & Right-Aligned Page Indicator in Header
        canvas.draw_mixed_text(14, 30, &self.book_title, colors::rgb(255, 205, 85), 12);
        let total_pages = self.total_pages();
        let cur_page = (self.book_page + 1).min(total_pages);
        let mut pg_str = String::with_capacity(16);
        let _ = write!(pg_str, "{}/{}页", cur_page, total_pages);
        let pw = Canvas::measure_mixed_text(&pg_str, 12);
        canvas.draw_mixed_text(304 - pw, 30, &pg_str, colors::rgb(150, 180, 220), 12);
        canvas.draw_line(14, 46, 304, 46, colors::rgb(36, 54, 84));

        // 8 Lines of Crisp 15x15 Anti-Aliased Chinese Paragraph Text
        let start_line = self.book_page * 8;
        for row in 0..8 {
            if let Some(line) = self.book_lines.get(start_line + row) {
                let ly = 52 + (row as i32) * 20;
                canvas.draw_mixed_text(15, ly, line, colors::rgb(228, 236, 248), 15);
            }
        }

        // Bottom 4 Reader Action Buttons (y = 216..236) — 1:1 mapped to B1..B4 short-press
        let reader_btns: [(i32, i32, i32, &str); 4] = [
            (8, 216, 72, "1.上页"),
            (86, 216, 72, "2.下页"),
            (164, 216, 72, "3.书架"),
            (242, 216, 70, "4.亮度"),
        ];
        for &(bx, by, bw, label) in &reader_btns {
            canvas.fill_rrect(bx, by, bw, 20, 4, colors::rgb(26, 40, 66));
            canvas.stroke_rrect(bx, by, bw, 20, 4, colors::rgb(65, 95, 145));
            let lw = Canvas::measure_mixed_text(label, 12);
            let tx = bx + (bw - lw) / 2;
            canvas.draw_mixed_text(tx, by + 4, label, colors::rgb(235, 245, 255), 12);
        }
    }

    fn on_slot_action(&mut self, slot: usize) -> AppIntent {
        match slot {
            0 => {
                // 1.上页
                if self.book_page > 0 {
                    self.book_page -= 1;
                }
                AppIntent::None
            }
            1 => {
                // 2.下页
                if self.book_page + 1 < self.total_pages() {
                    self.book_page += 1;
                } else {
                    self.book_page = 0;
                }
                AppIntent::None
            }
            2 => {
                // 3.书架
                AppIntent::SwitchApp(AppId::MediaBt)
            }
            _ => {
                // 4.亮度
                pwm::cycle_brightness();
                AppIntent::None
            }
        }
    }

    fn on_touch(&mut self, tx: i32, ty: i32) -> AppIntent {
        if ty >= 212 {
            let slot = ((tx - 8).max(0) / 78).clamp(0, 3) as usize;
            self.on_slot_action(slot)
        } else if ty >= 26 {
            if tx < 160 {
                self.on_slot_action(0)
            } else {
                self.on_slot_action(1)
            }
        } else {
            AppIntent::None
        }
    }

    fn on_tick_200ms(&mut self, _sub_tick: u32, _is_active: bool) -> bool {
        false
    }
}

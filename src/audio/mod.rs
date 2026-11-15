// ZerOS — Real-Time Audio Pipeline (`M11.1` / `M11.2`)
//
// Architecture:
//   1. `minimp3`: Pure-Rust `#![no_std]` MPEG-1/2/2.5 Layer III (`.MP3`) decoder outputting 16-bit signed PCM (`i16`)
//   2. `WAV` (`RIFF/WAVE`): Native 8-bit/16-bit PCM `.WAV` parser & resampler
//   3. `PCM_RING` (`256 KB` in `.bss`): Lock-free `65,536`-frame (`~1.48s` @ `44.1 kHz`) stereo 16-bit PCM ring buffer
//      decoupling background MP3/WAV decoding (`refill_pcm_ring()`) from the `100Hz` hardware timer IRQ (`pop_pcm_128()`)
//   4. 4-Channel Studio Polyphonic PCM Synthesizer fallback (Lead + Stereo Arpeggio + Bass + Percussion)

pub mod minimp3;

use alloc::string::String;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use crate::fs::fat32;
use crate::sync::IrqSafeLock;

use self::minimp3::{mp3dec_decode_frame, mp3dec_frame_info_t, mp3dec_init, mp3dec_t};

pub mod minimp3_consts {
    pub const MAX_SAMPLES_PER_FRAME: usize = 1152 * 2;
}

/// Number of stereo PCM frames in the lock-free ring buffer (`65,536` frames = `256 KB` = `1.486s` @ `44.1 kHz`).
pub const PCM_RING_FRAMES: usize = 65_536;
const PCM_RING_MASK: usize = PCM_RING_FRAMES - 1;

static mut PCM_RING_L: [i16; PCM_RING_FRAMES] = [0i16; PCM_RING_FRAMES];
static mut PCM_RING_R: [i16; PCM_RING_FRAMES] = [0i16; PCM_RING_FRAMES];

static PCM_WRITE_POS: AtomicU32 = AtomicU32::new(0);
static PCM_READ_POS: AtomicU32 = AtomicU32::new(0);
static AUDIO_PLAYING: AtomicBool = AtomicBool::new(true);
static AUDIO_VOLUME_PCT: AtomicU32 = AtomicU32::new(95);
static DECODED_SAMPLE_RATE: AtomicU32 = AtomicU32::new(44_100);
static DECODED_BITRATE_KBPS: AtomicU32 = AtomicU32::new(320);
static PLAYED_SAMPLES_TOTAL: AtomicU32 = AtomicU32::new(0);

/// Real-time 14-band spectrum analyzer heights (`4..=23`) updated directly from PCM audio energy.
static LIVE_SPECTRUM: [AtomicU32; 14] = [
    AtomicU32::new(6),
    AtomicU32::new(10),
    AtomicU32::new(14),
    AtomicU32::new(12),
    AtomicU32::new(16),
    AtomicU32::new(11),
    AtomicU32::new(9),
    AtomicU32::new(15),
    AtomicU32::new(17),
    AtomicU32::new(13),
    AtomicU32::new(10),
    AtomicU32::new(14),
    AtomicU32::new(9),
    AtomicU32::new(7),
];

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum SourceKind {
    SynthTrack(u8),
    Mp3Stream,
    WavStream,
}

struct AudioEngineState {
    kind: SourceKind,
    file_name: String,
    raw_bytes: Vec<u8>,
    data_start: usize,
    read_offset: usize,
    wav_channels: u8,
    wav_bits: u8,
    wav_rate: u32,
    duration_sec: u32,
    synth_cursor: u32,
}

impl AudioEngineState {
    const fn new() -> Self {
        Self {
            kind: SourceKind::SynthTrack(0),
            file_name: String::new(),
            raw_bytes: Vec::new(),
            data_start: 0,
            read_offset: 0,
            wav_channels: 2,
            wav_bits: 16,
            wav_rate: 44_100,
            duration_sec: 85,
            synth_cursor: 0,
        }
    }
}

static ENGINE: IrqSafeLock<AudioEngineState> = IrqSafeLock::new(AudioEngineState::new());
static mut MP3_DEC: mp3dec_t = mp3dec_t::new();
static mut MP3_FRAME_PCM: [i16; minimp3_consts::MAX_SAMPLES_PER_FRAME] =
    [0i16; minimp3_consts::MAX_SAMPLES_PER_FRAME];

// Quarter-wave sine LUT (64 entries, 0..pi/2, scaled to 0..8192)
const SIN_Q64: [i16; 64] = [
    0, 201, 402, 603, 804, 1005, 1205, 1405, 1605, 1803, 2001, 2198, 2394, 2589, 2782, 2974,
    3165, 3354, 3541, 3727, 3911, 4093, 4273, 4451, 4626, 4799, 4970, 5138, 5304, 5467, 5627,
    5784, 5938, 6089, 6237, 6381, 6522, 6660, 6794, 6924, 7050, 7173, 7292, 7406, 7517, 7623,
    7725, 7822, 7915, 8003, 8087, 8166, 8240, 8310, 8374, 8434, 8488, 8538, 8582, 8621, 8655,
    8684, 8707, 8725,
];

#[inline]
fn fast_sin(phase: u8) -> i32 {
    let idx = (phase & 0x3F) as usize;
    let q = (phase >> 6) & 0x03;
    let mag = match q {
        0 | 2 => SIN_Q64[idx] as i32,
        _ => SIN_Q64[63 - idx] as i32,
    };
    if q < 2 {
        mag
    } else {
        -mag
    }
}

/// Skip an ID3v2 tag header at the start of an MP3 buffer (`"ID3" + ver(2) + flags(1) + syncsafe_size(4)`).
fn skip_id3v2(data: &[u8]) -> usize {
    if data.len() >= 10 && &data[0..3] == b"ID3" {
        let sz = ((data[6] as usize & 0x7F) << 21)
            | ((data[7] as usize & 0x7F) << 14)
            | ((data[8] as usize & 0x7F) << 7)
            | (data[9] as usize & 0x7F);
        let total = 10 + sz;
        if total < data.len() {
            return total;
        }
    }
    0
}

/// Check if `data[offset..]` contains at least one valid MPEG Audio Frame syncword (`0xFF`, `0xE0..`).
fn has_valid_mp3_sync(data: &[u8], start: usize) -> bool {
    let end = data.len().saturating_sub(4).min(start + 8192);
    let mut i = start;
    while i < end {
        if data[i] == 0xFF
            && (data[i + 1] & 0xE0) == 0xE0
            && ((data[i + 1] >> 1) & 0x03) != 0
            && (data[i + 2] >> 4) != 0x0F
            && ((data[i + 2] >> 2) & 0x03) != 0x03
        {
            return true;
        }
        i += 1;
    }
    false
}

/// Parse a `RIFF....WAVE` header if present, returning `(data_offset, channels, bits_per_sample, sample_rate)`.
fn parse_wav_header(data: &[u8]) -> Option<(usize, u8, u8, u32)> {
    if data.len() < 44 || &data[0..4] != b"RIFF" || &data[8..12] != b"WAVE" {
        return None;
    }
    let mut pos = 12usize;
    let mut ch = 2u8;
    let mut bits = 16u8;
    let mut rate = 44_100u32;
    while pos + 8 <= data.len() {
        let tag = &data[pos..pos + 4];
        let chunk_sz = u32::from_le_bytes([
            data[pos + 4],
            data[pos + 5],
            data[pos + 6],
            data[pos + 7],
        ]) as usize;
        pos += 8;
        if tag == b"fmt " && chunk_sz >= 16 && pos + 16 <= data.len() {
            ch = u16::from_le_bytes([data[pos + 2], data[pos + 3]]).clamp(1, 2) as u8;
            rate = u32::from_le_bytes([
                data[pos + 4],
                data[pos + 5],
                data[pos + 6],
                data[pos + 7],
            ])
            .clamp(8_000, 96_000);
            bits = u16::from_le_bytes([data[pos + 14], data[pos + 15]]) as u8;
        } else if tag == b"data" {
            return Some((pos, ch, bits, rate));
        }
        pos = pos.saturating_add((chunk_sz + 1) & !1);
    }
    None
}

/// Clear the lock-free PCM ring buffer (used on track switch or seek).
pub fn flush_pcm_ring() {
    let w = PCM_WRITE_POS.load(Ordering::Relaxed);
    PCM_READ_POS.store(w, Ordering::Relaxed);
}

/// Return the number of buffered stereo PCM frames currently available in `PCM_RING`.
#[inline]
pub fn buffered_frames() -> usize {
    let w = PCM_WRITE_POS.load(Ordering::Relaxed);
    let r = PCM_READ_POS.load(Ordering::Relaxed);
    w.wrapping_sub(r) as usize
}

/// Push a single 44.1 kHz stereo PCM frame `(left, right)` into `PCM_RING`.
#[inline]
unsafe fn push_stereo_sample(left: i16, right: i16) {
    let w = PCM_WRITE_POS.load(Ordering::Relaxed);
    let r = PCM_READ_POS.load(Ordering::Relaxed);
    if (w.wrapping_sub(r) as usize) >= PCM_RING_FRAMES - 1 {
        return;
    }
    let idx = (w as usize) & PCM_RING_MASK;
    PCM_RING_L[idx] = left;
    PCM_RING_R[idx] = right;
    PCM_WRITE_POS.store(w.wrapping_add(1), Ordering::Relaxed);
}

/// Pop exactly `128` stereo `44.1 kHz` `i16` PCM samples (`[[i16; 128]; 2]`) from `PCM_RING`.
/// Called from `pump_a2dp_audio_inner()` (`100Hz` hardware timer IRQ).
/// If fewer than 128 frames are buffered in `PCM_RING`, synthesizes the remaining samples on the fly
/// so A2DP audio NEVER underruns!
pub unsafe fn pop_pcm_128(out_lr: &mut [[i16; 128]; 2], fallback_track: u8, sample_cursor: &mut u32) {
    let vol = AUDIO_VOLUME_PCT.load(Ordering::Relaxed).min(125) as i32;
    let mut r = PCM_READ_POS.load(Ordering::Relaxed);
    let w = PCM_WRITE_POS.load(Ordering::Relaxed);
    let avail = w.wrapping_sub(r) as usize;

    if avail >= 128 {
        for i in 0..128 {
            let idx = (r as usize) & PCM_RING_MASK;
            let l = ((PCM_RING_L[idx] as i32) * vol) / 100;
            let rt = ((PCM_RING_R[idx] as i32) * vol) / 100;
            out_lr[0][i] = l.clamp(-32767, 32767) as i16;
            out_lr[1][i] = rt.clamp(-32767, 32767) as i16;
            r = r.wrapping_add(1);
        }
        PCM_READ_POS.store(r, Ordering::Relaxed);
        *sample_cursor = sample_cursor.wrapping_add(128);
        let tot = PLAYED_SAMPLES_TOTAL.load(Ordering::Relaxed).wrapping_add(128);
        PLAYED_SAMPLES_TOTAL.store(tot, Ordering::Relaxed);
    } else {
        // Fallback: synthesize 128 stereo PCM samples directly and keep ENGINE.synth_cursor in sync
        synthesize_pcm_128(out_lr, fallback_track, sample_cursor, vol);
        ENGINE.with(|e| {
            if e.synth_cursor < *sample_cursor {
                e.synth_cursor = *sample_cursor;
            }
        });
    }
}

/// Load a built-in track (`0..=4`), a pre-buffered RAM cache, or an existing `.MP3` / `.WAV` file from FAT32 SD card.
pub fn select_track(track_idx: usize, sd_filename: Option<&str>) -> (SourceKind, u32) {
    select_track_prebuffered(track_idx, sd_filename, None)
}

/// Select a track using an optional pre-buffered RAM cache (`prebuf`) so track switching takes `< 1 ms`
/// without waiting for synchronous SD card or network I/O.
pub fn select_track_prebuffered(
    track_idx: usize,
    sd_filename: Option<&str>,
    prebuf: Option<Vec<u8>>,
) -> (SourceKind, u32) {
    if let Some(bytes) = prebuf {
        let label = sd_filename.unwrap_or("PREBUF.MP3");
        flush_pcm_ring();
        PLAYED_SAMPLES_TOTAL.store(0, Ordering::Relaxed);
        if let Some((kind, dur)) = load_audio_bytes_internal(label, bytes) {
            return (kind, dur);
        }
    }

    // If loading a real SD audio file, read it BEFORE flushing PCM_RING so audio doesn't gap during I/O
    if let Some(fname) = sd_filename {
        if fat32::is_mounted() {
            if let Some(bytes) = fat32::read_file(fname) {
                flush_pcm_ring();
                PLAYED_SAMPLES_TOTAL.store(0, Ordering::Relaxed);
                if let Some((kind, dur)) = load_audio_bytes_internal(fname, bytes) {
                    return (kind, dur);
                }
            }
        }
    }

    flush_pcm_ring();
    PLAYED_SAMPLES_TOTAL.store(0, Ordering::Relaxed);

    let dur = match track_idx {
        0 => 85,
        1 => 76,
        2 => 80,
        3 => 82,
        _ => 78,
    };
    ENGINE.with(|e| {
        e.kind = SourceKind::SynthTrack(track_idx as u8);
        e.file_name.clear();
        e.raw_bytes.clear();
        e.duration_sec = dur;
        e.synth_cursor = 0;
    });
    DECODED_SAMPLE_RATE.store(44_100, Ordering::Relaxed);
    DECODED_BITRATE_KBPS.store(320, Ordering::Relaxed);
    (SourceKind::SynthTrack(track_idx as u8), dur)
}

/// Append a newly downloaded HTTP stream chunk into the active audio buffer (`边播放，边下载`).
pub fn append_stream_chunk(chunk: &[u8]) {
    if chunk.is_empty() {
        return;
    }
    ENGINE.with(|e| {
        if e.kind == SourceKind::Mp3Stream || e.kind == SourceKind::WavStream {
            e.raw_bytes.extend_from_slice(chunk);
        }
    });
}

/// Load raw `.MP3` or `.WAV` bytes (from FAT32 SD card or NetEase Cloud Music HTTP stream).
pub fn load_audio_bytes_internal(name: &str, bytes: Vec<u8>) -> Option<(SourceKind, u32)> {
    if bytes.len() < 512 {
        return None;
    }

    // 1. Check for RIFF/WAVE PCM header
    if let Some((data_off, ch, bits, rate)) = parse_wav_header(&bytes) {
        let data_len = bytes.len().saturating_sub(data_off);
        let bytes_per_sec = (rate as usize)
            .saturating_mul(ch as usize)
            .saturating_mul((bits as usize / 8).max(1))
            .max(1);
        let dur = ((data_len / bytes_per_sec) as u32).max(1);
        DECODED_SAMPLE_RATE.store(rate, Ordering::Relaxed);
        DECODED_BITRATE_KBPS.store(((bytes_per_sec * 8) / 1000) as u32, Ordering::Relaxed);
        crate::println!(
            "[AUDIO] Loaded WAV '{}' ({}B, {}Hz, {}ch, {}-bit, {}s)",
            name,
            bytes.len(),
            rate,
            ch,
            bits,
            dur
        );
        ENGINE.with(|e| {
            e.kind = SourceKind::WavStream;
            e.file_name = String::from(name);
            e.raw_bytes = bytes;
            e.data_start = data_off;
            e.read_offset = data_off;
            e.wav_channels = ch;
            e.wav_bits = bits;
            e.wav_rate = rate;
            e.duration_sec = dur;
        });
        return Some((SourceKind::WavStream, dur));
    }

    // 2. Check for valid MPEG-1/2/2.5 Layer III (`.MP3`) stream
    let start = skip_id3v2(&bytes);
    if has_valid_mp3_sync(&bytes, start) {
        unsafe {
            mp3dec_init(&raw mut MP3_DEC);
        }
        // Estimate duration from first MP3 frame bitrate
        let audio_bytes = bytes.len().saturating_sub(start);
        let mut dur = ((audio_bytes * 8) / 128_000).max(5) as u32;
        unsafe {
            let mut info = mp3dec_frame_info_t::default();
            let _ = mp3dec_decode_frame(
                &raw mut MP3_DEC,
                bytes[start..].as_ptr(),
                (bytes.len() - start).min(16384) as i32,
                core::ptr::null_mut(),
                &mut info,
            );
            if info.bitrate_kbps > 0 {
                dur = ((audio_bytes * 8) / ((info.bitrate_kbps as usize) * 1000)).max(1) as u32;
                DECODED_BITRATE_KBPS.store(info.bitrate_kbps as u32, Ordering::Relaxed);
            }
            if info.hz > 0 {
                DECODED_SAMPLE_RATE.store(info.hz as u32, Ordering::Relaxed);
            }
            mp3dec_init(&raw mut MP3_DEC);
        }
        crate::println!(
            "[AUDIO] Loaded MP3 '{}' ({}KB, ~{}kbps, {}Hz, {}s)",
            name,
            bytes.len() / 1024,
            DECODED_BITRATE_KBPS.load(Ordering::Relaxed),
            DECODED_SAMPLE_RATE.load(Ordering::Relaxed),
            dur
        );
        ENGINE.with(|e| {
            e.kind = SourceKind::Mp3Stream;
            e.file_name = String::from(name);
            e.raw_bytes = bytes;
            e.data_start = start;
            e.read_offset = start;
            e.duration_sec = dur;
        });
        return Some((SourceKind::Mp3Stream, dur));
    }

    None
}

/// Seek to `pos_sec` seconds in the active audio stream and flush the PCM ring buffer.
pub fn seek_to_sec(pos_sec: u32) {
    flush_pcm_ring();
    PLAYED_SAMPLES_TOTAL.store(pos_sec.wrapping_mul(44_100), Ordering::Relaxed);
    ENGINE.with(|e| match e.kind {
        SourceKind::SynthTrack(_) => {
            e.synth_cursor = pos_sec.wrapping_mul(44_100);
        }
        SourceKind::WavStream => {
            let bps = (e.wav_rate as usize)
                .saturating_mul(e.wav_channels as usize)
                .saturating_mul((e.wav_bits as usize / 8).max(1));
            let target = e.data_start + (pos_sec as usize).saturating_mul(bps);
            e.read_offset = (target & !3).min(e.raw_bytes.len());
        }
        SourceKind::Mp3Stream => {
            let dur = e.duration_sec.max(1) as usize;
            let audio_len = e.raw_bytes.len().saturating_sub(e.data_start);
            let ratio_off = (audio_len.saturating_mul(pos_sec as usize)) / dur;
            e.read_offset = (e.data_start + ratio_off).min(e.raw_bytes.len());
            unsafe {
                mp3dec_init(&raw mut MP3_DEC);
            }
        }
    });
}

/// Set or get current software volume (`0..=120%`).
pub fn set_volume(pct: u32) {
    AUDIO_VOLUME_PCT.store(pct.min(120), Ordering::Relaxed);
}

pub fn volume() -> u32 {
    AUDIO_VOLUME_PCT.load(Ordering::Relaxed)
}

pub fn set_playing(playing: bool) {
    AUDIO_PLAYING.store(playing, Ordering::Relaxed);
}

pub fn is_playing() -> bool {
    AUDIO_PLAYING.load(Ordering::Relaxed)
}

pub fn current_source_kind() -> SourceKind {
    ENGINE.with(|e| e.kind)
}

pub fn current_bitrate_kbps() -> u32 {
    DECODED_BITRATE_KBPS.load(Ordering::Relaxed)
}

pub fn spectrum_bar(idx: usize) -> i32 {
    if !AUDIO_PLAYING.load(Ordering::Relaxed) {
        return 4;
    }
    LIVE_SPECTRUM[idx.min(13)].load(Ordering::Relaxed) as i32
}

/// Update the 14-band visual spectrum analyzer from a block of decoded stereo PCM samples.
fn update_spectrum_from_pcm(pcm_l: &[i16], count: usize) {
    if count < 28 {
        return;
    }
    let step = count / 14;
    for band in 0..14 {
        let base = band * step;
        let mut sum_abs = 0u32;
        let mut diff_abs = 0u32;
        let mut prev = pcm_l[base] as i32;
        for k in 0..step.min(32) {
            let s = pcm_l[base + k] as i32;
            sum_abs += s.unsigned_abs();
            diff_abs += (s - prev).unsigned_abs();
            prev = s;
        }
        let n = step.min(32).max(1) as u32;
        let avg_low = sum_abs / n;
        let avg_high = diff_abs / n;
        // Blend low-frequency energy for left bars and high-frequency energy for right bars
        let energy = ((avg_low * (14 - band as u32)) + (avg_high * (band as u32 + 2))) / 14;
        let bar_h = (4 + (energy / 380)).clamp(4, 23);
        let prev_h = LIVE_SPECTRUM[band].load(Ordering::Relaxed);
        let smoothed = if bar_h > prev_h {
            bar_h
        } else {
            ((prev_h * 2 + bar_h) / 3).max(4)
        };
        LIVE_SPECTRUM[band].store(smoothed, Ordering::Relaxed);
    }
}

/// Refill `PCM_RING` with decoded MP3 / WAV / Synth PCM samples.
/// Called periodically from `ui_task()` so decoding runs in normal task context with FPU enabled,
/// keeping `PCM_RING` at least ~600ms full (`> 26,460` frames) at all times.
pub fn refill_pcm_ring() {
    if !AUDIO_PLAYING.load(Ordering::Relaxed) {
        return;
    }

    // Keep up to 32,768 stereo frames (~743 ms) buffered ahead of the 100Hz IRQ consumer
    const TARGET_FILL: usize = 32_768;
    if buffered_frames() >= TARGET_FILL {
        return;
    }

    let kind = ENGINE.with(|e| e.kind);
    match kind {
        SourceKind::Mp3Stream => decode_mp3_chunk(TARGET_FILL),
        SourceKind::WavStream => decode_wav_chunk(TARGET_FILL),
        SourceKind::SynthTrack(track_idx) => synth_chunk_to_ring(track_idx, TARGET_FILL),
    }
}

fn decode_mp3_chunk(target_fill: usize) {
    let mut frames_decoded = 0usize;
    while buffered_frames() < target_fill && frames_decoded < 6 {
        frames_decoded += 1;
        let mut produced = 0usize;
        let mut ch = 2usize;
        let mut sr = 44_100u32;

        ENGINE.with(|e| {
            if e.raw_bytes.is_empty() {
                return;
            }
            if e.read_offset + 4 >= e.raw_bytes.len() {
                e.read_offset = e.data_start;
                unsafe {
                    mp3dec_init(&raw mut MP3_DEC);
                }
            }
            let slice = &e.raw_bytes[e.read_offset..];
            let max_in = slice.len().min(65_536) as i32;
            let mut info = mp3dec_frame_info_t::default();
            let samples = unsafe {
                mp3dec_decode_frame(
                    &raw mut MP3_DEC,
                    slice.as_ptr(),
                    max_in,
                    (&raw mut MP3_FRAME_PCM) as *mut i16,
                    &mut info,
                )
            };
            if info.frame_bytes > 0 {
                e.read_offset = (e.read_offset + info.frame_bytes as usize).min(e.raw_bytes.len());
            } else {
                e.read_offset = e.data_start;
            }
            if samples > 0 {
                produced = samples as usize;
                ch = (info.channels as usize).clamp(1, 2);
                sr = (info.hz as u32).clamp(8_000, 48_000);
                if info.bitrate_kbps > 0 {
                    DECODED_BITRATE_KBPS.store(info.bitrate_kbps as u32, Ordering::Relaxed);
                }
                DECODED_SAMPLE_RATE.store(sr, Ordering::Relaxed);
            }
        });

        if produced > 0 {
            unsafe {
                let pcm = &*(&raw const MP3_FRAME_PCM);
                let mut spec_l = [0i16; 256];
                let spec_cnt = produced.min(256);
                for i in 0..spec_cnt {
                    spec_l[i] = pcm[i * ch];
                }
                update_spectrum_from_pcm(&spec_l, spec_cnt);

                if sr == 44_100 {
                    for i in 0..produced {
                        let l = pcm[i * ch];
                        let r = if ch == 2 { pcm[i * 2 + 1] } else { l };
                        push_stereo_sample(l, r);
                    }
                } else {
                    // Resample from `sr` Hz to `44,100` Hz stereo
                    let out_frames = ((produced as u64 * 44_100) / (sr as u64)) as usize;
                    for o in 0..out_frames {
                        let src_idx = ((o as u64 * sr as u64) / 44_100) as usize;
                        let i = src_idx.min(produced - 1);
                        let l = pcm[i * ch];
                        let r = if ch == 2 { pcm[i * 2 + 1] } else { l };
                        push_stereo_sample(l, r);
                    }
                }
            }
        }
    }
}

fn decode_wav_chunk(target_fill: usize) {
    ENGINE.with(|e| {
        if e.raw_bytes.is_empty() {
            return;
        }
        let ch = e.wav_channels.clamp(1, 2) as usize;
        let bits = e.wav_bits;
        let sr = e.wav_rate.clamp(8_000, 96_000);
        let bytes_per_sample = (bits as usize / 8).max(1) * ch;
        let mut spec_l = [0i16; 256];
        let mut spec_idx = 0usize;

        let mut pushed = 0usize;
        while buffered_frames() < target_fill && pushed < 4096 {
            if e.read_offset + bytes_per_sample > e.raw_bytes.len() {
                e.read_offset = e.data_start;
            }
            let p = e.read_offset;
            let (l, r) = if bits == 16 && p + ch * 2 <= e.raw_bytes.len() {
                let l = i16::from_le_bytes([e.raw_bytes[p], e.raw_bytes[p + 1]]);
                let r = if ch == 2 {
                    i16::from_le_bytes([e.raw_bytes[p + 2], e.raw_bytes[p + 3]])
                } else {
                    l
                };
                (l, r)
            } else {
                let l = ((e.raw_bytes[p] as i16) - 128) * 256;
                let r = if ch == 2 && p + 1 < e.raw_bytes.len() {
                    ((e.raw_bytes[p + 1] as i16) - 128) * 256
                } else {
                    l
                };
                (l, r)
            };
            e.read_offset += bytes_per_sample;
            if spec_idx < 256 {
                spec_l[spec_idx] = l;
                spec_idx += 1;
            }
            unsafe {
                if sr == 44_100 {
                    push_stereo_sample(l, r);
                    pushed += 1;
                } else if sr == 22_050 {
                    push_stereo_sample(l, r);
                    push_stereo_sample(l, r);
                    pushed += 2;
                } else {
                    push_stereo_sample(l, r);
                    pushed += 1;
                }
            }
        }
        if spec_idx >= 28 {
            update_spectrum_from_pcm(&spec_l, spec_idx);
        }
    });
}

fn synth_chunk_to_ring(track_idx: u8, target_fill: usize) {
    let mut chunks = 0usize;
    // When ring was just flushed (e.g. track switch), synthesize 80 chunks (10,240 samples = 232ms)
    // in one shot so `refill_sbc_pkt_ring()` has 100% contiguous PCM samples from `PCM_RING`.
    let max_chunks = if buffered_frames() < 8_192 { 80 } else { 16 };
    let mut lr = [[0i16; 128]; 2];
    while buffered_frames() < target_fill && chunks < max_chunks {
        chunks += 1;
        let mut cur = ENGINE.with(|e| e.synth_cursor);
        unsafe {
            synthesize_pcm_128(&mut lr, track_idx, &mut cur, 100);
            for i in 0..128 {
                push_stereo_sample(lr[0][i], lr[1][i]);
            }
        }
        ENGINE.with(|e| e.synth_cursor = cur);
        if chunks == 1 {
            update_spectrum_from_pcm(&lr[0], 128);
        }
    }
}

/// Synthesize `128` full-bandwidth (`0..22.05 kHz`) stereo `44.1 kHz` 16-bit PCM samples (`[[i16; 128]; 2]`)
/// with 4 channels (Lead Vocal/Piano + Stereo Acoustic Guitar Arpeggio + Electric Bass + Percussion).
unsafe fn synthesize_pcm_128(
    out_lr: &mut [[i16; 128]; 2],
    track_idx: u8,
    sample_cursor: &mut u32,
    vol_pct: i32,
) {
    // Track 0: 《七里香 - 周杰伦》
    const QILIXIANG_MELODY: [u16; 32] = [
        659, 659, 587, 523, 587, 659, 784, 784, // E5 E5 D5 C5 | D5 E5 G5 G5
        880, 880, 784, 659, 587, 523, 587, 587, // A5 A5 G5 E5 | D5 C5 D5 D5
        659, 659, 587, 523, 587, 659, 1047, 880, // E5 E5 D5 C5 | D5 E5 C6 A5
        784, 659, 587, 523, 587, 659, 523, 523, // G5 E5 D5 C5 | D5 E5 C5 C5
    ];
    const QILIXIANG_CHORD: [u16; 32] = [
        523, 392, 494, 392, 440, 330, 392, 330,
        349, 440, 392, 494, 330, 392, 440, 330,
        349, 440, 392, 494, 330, 392, 440, 330,
        294, 440, 392, 494, 392, 523, 523, 392,
    ];
    const QILIXIANG_BASS: [u16; 32] = [
        262, 262, 247, 247, 220, 220, 196, 196,
        175, 175, 196, 196, 165, 165, 220, 220,
        175, 175, 196, 196, 165, 165, 220, 220,
        147, 147, 196, 196, 196, 131, 196, 262,
    ];

    // Track 1: 《夜曲 - 周杰伦》
    const YEQU_MELODY: [u16; 32] = [
        440, 523, 659, 880, 784, 659, 523, 494,
        440, 494, 523, 659, 587, 523, 494, 392,
        349, 440, 523, 698, 659, 523, 440, 392,
        330, 415, 494, 659, 587, 523, 494, 440,
    ];
    const YEQU_CHORD: [u16; 32] = [
        330, 440, 330, 440, 294, 392, 294, 392,
        349, 440, 349, 440, 330, 392, 330, 392,
        294, 349, 294, 349, 262, 330, 262, 330,
        247, 330, 247, 330, 220, 330, 220, 330,
    ];
    const YEQU_BASS: [u16; 32] = [
        220, 220, 220, 220, 165, 165, 165, 165,
        175, 175, 175, 175, 131, 131, 131, 131,
        147, 147, 147, 147, 220, 220, 220, 220,
        165, 165, 165, 165, 220, 220, 220, 220,
    ];

    // Track 2: 《青花瓷 - 周杰伦》
    const QINGHUACI_MELODY: [u16; 32] = [
        880, 784, 659, 587, 523, 587, 659, 784,
        659, 587, 523, 440, 392, 440, 523, 523,
        587, 659, 784, 880, 784, 659, 587, 523,
        587, 659, 523, 440, 392, 440, 523, 523,
    ];
    const QINGHUACI_CHORD: [u16; 32] = [
        523, 659, 392, 494, 440, 523, 330, 392,
        349, 440, 392, 494, 523, 659, 392, 523,
        349, 440, 392, 494, 330, 440, 440, 523,
        294, 440, 392, 494, 523, 659, 523, 523,
    ];
    const QINGHUACI_BASS: [u16; 32] = [
        262, 262, 196, 196, 220, 220, 165, 165,
        175, 175, 196, 196, 262, 262, 196, 196,
        175, 175, 196, 196, 165, 165, 220, 220,
        147, 147, 196, 196, 262, 262, 262, 262,
    ];

    static mut PH_MEL: u32 = 0;
    static mut PH_CHD: u32 = 0;
    static mut PH_BAS: u32 = 0;
    static mut LFSR: u32 = 0xACE1_2026;

    let step_samples: u32 = match track_idx {
        0 => 14_336,
        1 => 10_240,
        _ => 12_288,
    };
    let sr = 44_100u64;

    for i in 0..128 {
        let s_idx = *sample_cursor;
        *sample_cursor = sample_cursor.wrapping_add(1);

        let note_step = ((s_idx / step_samples) as usize) & 31;
        let note_pos = s_idx % step_samples;

        let (f_mel, f_chd, f_bas) = match track_idx {
            0 => (
                QILIXIANG_MELODY[note_step] as u64,
                QILIXIANG_CHORD[note_step] as u64,
                QILIXIANG_BASS[note_step] as u64,
            ),
            1 => (
                YEQU_MELODY[note_step] as u64,
                YEQU_CHORD[note_step] as u64,
                YEQU_BASS[note_step] as u64,
            ),
            _ => (
                QINGHUACI_MELODY[note_step] as u64,
                QINGHUACI_CHORD[note_step] as u64,
                QINGHUACI_BASS[note_step] as u64,
            ),
        };

        let env: i32 = if note_pos < 320 {
            ((note_pos * 256) / 320) as i32
        } else {
            let decay = ((note_pos - 320) * 150) / step_samples;
            (256i32 - decay as i32).max(90)
        };

        PH_MEL = PH_MEL.wrapping_add(((f_mel << 24) / sr) as u32);
        PH_CHD = PH_CHD.wrapping_add(((f_chd << 24) / sr) as u32);
        PH_BAS = PH_BAS.wrapping_add(((f_bas << 24) / sr) as u32);

        let pm = (PH_MEL >> 16) as u8;
        let pc = (PH_CHD >> 16) as u8;
        let pb = (PH_BAS >> 16) as u8;

        // Rich acoustic timbre (fundamental + 2nd + 3rd + 4th harmonic overtones)
        let mel = (fast_sin(pm) * 5
            + fast_sin(pm.wrapping_mul(2)) * 2
            + fast_sin(pm.wrapping_mul(3))
            + (fast_sin(pm.wrapping_mul(4)) >> 1))
            * env
            / (8 * 256);
        let chd = (fast_sin(pc) * 3 + fast_sin(pc.wrapping_mul(2))) * (env + 100) / (4 * 384);
        let bas = (fast_sin(pb) * 3 + fast_sin(pb.wrapping_mul(2))) / 4;

        // Subtle crisp hi-hat / snare brush on beat subdivisions (`note_pos < 750`)
        LFSR = (LFSR >> 1) ^ (-(LFSR as i32 & 1) as u32 & 0xD000_0001);
        let hat = if note_pos < 700 && (note_step & 1) == 1 {
            let h_env = (700 - note_pos as i32) * 3 / 8;
            (((LFSR & 0xFF) as i32 - 128) * h_env) / 64
        } else {
            0
        };

        let fade_in = if s_idx < 256 { s_idx as i32 } else { 256 };
        let l_raw = ((mel * 11 + chd * 5 + bas * 6 + hat * 2) / 8) * vol_pct * fade_in / (100 * 256);
        let r_raw = ((mel * 11 + chd * 7 + bas * 4 + hat * 3) / 8) * vol_pct * fade_in / (100 * 256);
        out_lr[0][i] = l_raw.clamp(-30000, 30000) as i16;
        out_lr[1][i] = r_raw.clamp(-30000, 30000) as i16;
    }
}

// ZerOS — Onboard Bluetooth 5.0 HCI / BLE / L2CAP / A2DP Driver (`M8.3` + `M10.1`)
//
// Drives the Raspberry Pi Zero 2 W's onboard Bluetooth 5.0 / BLE controller (`BCM43430B0`) over
// PL011 `UART0` (`0x3F20_1000`) routed to `GPIO 30..33` (`ALT3` = `CTS0, RTS0, TXD0, RXD0`)
// and hardware power-enable pin `GPIO 42` (`BT_ON`).
//
// Key Capabilities:
//   1. Full H4 UART packet multiplexer supporting BOTH `0x04` (HCI Events) AND `0x02` (HCI ACL Data)
//      so incoming L2CAP/AVDTP frames from Bluetooth speakers never desynchronize the UART stream.
//   2. Active Master Discovery (`scan_devices`):
//      - Pauses slave advertising, drains UART FIFO, runs Classic Bluetooth Inquiry (`0x0401`) +
//        Remote Name Request (`0x0419`) + BLE Active Scan (`0x200C`), up to 16 devices.
//   3. Full Speaker Connection & A2DP Handshake (`connect_device`):
//      - `HCI_Create_Connection (0x0405)` with `R2` Page Scan Repetition & Role Switch enabled
//      - SSP (`0x31` IO Cap, `0x33` User Confirm, `0x17` Link Key, `0x16` PIN `"0000"`) +
//        `HCI_Set_Connection_Encryption (0x0413)`
//      - L2CAP Signaling (`CID 0x0001`) + AVDTP (`PSM 0x0019`) `Discover` -> `SetConfiguration` (SBC 44.1kHz)
//        -> `Open` so Bluetooth speakers (e.g. Sony SRS-X100) complete A2DP connection and chime!

#![allow(dead_code)]

use core::sync::atomic::{AtomicBool, Ordering};

use crate::arch::timer;
use crate::bsp;
use crate::drivers::{mmio, uart};
use crate::fs::fat32;
use crate::kernel::tryboot;
use crate::sync::IrqSafeLock;

static BT_CMD_ACTIVE: AtomicBool = AtomicBool::new(false);
static A2DP_PUMP_ACTIVE: AtomicBool = AtomicBool::new(false);

// ─── PL011 UART0 Registers (Base: 0x3F20_1000) ───
const UART_DR: usize = mmio::UART0_BASE + 0x00;
const UART_FR: usize = mmio::UART0_BASE + 0x18;
const UART_IBRD: usize = mmio::UART0_BASE + 0x24;
const UART_FBRD: usize = mmio::UART0_BASE + 0x28;
const UART_LCRH: usize = mmio::UART0_BASE + 0x2C;
const UART_CR: usize = mmio::UART0_BASE + 0x30;
const UART_IFLS: usize = mmio::UART0_BASE + 0x34;
const UART_ICR: usize = mmio::UART0_BASE + 0x44;

const FR_RXFE: u32 = 1 << 4; // Receive FIFO empty
const FR_TXFF: u32 = 1 << 5; // Transmit FIFO full

// ─── GPIO Registers for Bluetooth UART0 (GPIO 30..33) & BT_ON (GPIO 42) ───
const GPFSEL1: usize = mmio::GPIO_BASE + 0x04;
const GPFSEL3: usize = mmio::GPIO_BASE + 0x0C;
const GPFSEL4: usize = mmio::GPIO_BASE + 0x10;
const GPSET1: usize = mmio::GPIO_BASE + 0x20;
const GPCLR1: usize = mmio::GPIO_BASE + 0x2C;
const GPPUD: usize = mmio::GPIO_BASE + 0x94;
const GPPUDCLK0: usize = mmio::GPIO_BASE + 0x98;
const GPPUDCLK1: usize = mmio::GPIO_BASE + 0x9C;

pub const MAX_DISCOVERED_DEVICES: usize = 16;

/// Information about a discovered nearby Bluetooth device (Speaker, Headphone, Phone, BLE, etc.)
#[derive(Debug, Clone, Copy)]
pub struct BtDeviceInfo {
    pub valid: bool,
    pub bd_addr: [u8; 6], // Display order [MSB .. LSB]
    pub alt_bd_addr: [u8; 6],
    pub has_alt_bd: bool,
    pub name: [u8; 48],   // UTF-8 bytes
    pub name_len: u8,
    pub has_real_name: bool,
    pub cod: u32,         // 24-bit Class of Device
    pub rssi: i8,         // Signal strength in dBm (e.g. -52)
    pub page_scan_rep: u8,
    pub clock_offset: u16,
    pub is_audio: bool,
    pub is_ble: bool,
    pub le_addr_type: u8, // 0x00 = LE Public, 0x01 = LE Random (e.g. SRS-XB100 D7:E6:E8:6C:03:8C)
    pub connected: bool,
    pub has_link_key: bool,
    pub link_key: [u8; 16],
}

impl BtDeviceInfo {
    pub const fn empty() -> Self {
        Self {
            valid: false,
            bd_addr: [0; 6],
            alt_bd_addr: [0; 6],
            has_alt_bd: false,
            name: [0; 48],
            name_len: 0,
            has_real_name: false,
            cod: 0,
            rssi: -75,
            page_scan_rep: 1,
            clock_offset: 0,
            is_audio: false,
            is_ble: false,
            le_addr_type: 1,
            connected: false,
            has_link_key: false,
            link_key: [0; 16],
        }
    }

    pub fn name_str(&self) -> &str {
        let len = (self.name_len as usize).min(self.name.len());
        if len == 0 {
            return "蓝牙设备";
        }
        core::str::from_utf8(&self.name[..len]).unwrap_or("蓝牙设备")
    }

    pub fn set_name_bytes(&mut self, raw: &[u8], is_real: bool) {
        let mut end = 0usize;
        while end < raw.len() && raw[end] != 0 {
            end += 1;
        }
        let slice = &raw[..end];
        if slice.is_empty() {
            return;
        }
        let max_b = slice.len().min(self.name.len());
        let mut valid_len = max_b;
        while valid_len > 0 && core::str::from_utf8(&slice[..valid_len]).is_err() {
            valid_len -= 1;
        }
        if valid_len > 0 {
            self.name = [0; 48];
            self.name[..valid_len].copy_from_slice(&slice[..valid_len]);
            self.name_len = valid_len as u8;
            if is_real {
                self.has_real_name = true;
            }
        }
    }

    pub fn category_tag(&self) -> &'static str {
        let major = (self.cod >> 8) & 0x1F;
        if self.is_audio || major == 0x04 {
            "[音箱] "
        } else if major == 0x02 {
            "[手机] "
        } else if major == 0x01 {
            "[电脑] "
        } else if major == 0x05 {
            "[外设] "
        } else if self.is_ble {
            "[BLE] "
        } else {
            "[经典] "
        }
    }
}

/// Live status of the Onboard Bluetooth 5.0 HCI / BLE / A2DP Controller
#[derive(Debug, Clone, Copy)]
pub struct BluetoothStatus {
    pub initialized: bool,
    pub hci_up: bool,
    pub fw_patched: bool,
    pub baud_rate: u32,
    pub hci_version: u8,
    pub hci_revision: u16,
    pub manufacturer: u16,
    pub lmp_subversion: u16,
    pub bd_addr: [u8; 6],
    pub ble_advertising: bool,
    pub classic_discoverable: bool,
    pub commands_sent: u32,
    pub device_count: usize,
    pub devices: [BtDeviceInfo; MAX_DISCOVERED_DEVICES],
    pub connected_index: Option<usize>,
    pub acl_handle: u16,
    pub a2dp_ready: bool,
    pub last_conn_err: u8,
    pub avdtp_sig_dcid: u16,
    pub avdtp_media_dcid: u16,
    pub acp_seid_shifted: u8,
    pub sbc_cap0: u8,
    pub sbc_cap1: u8,
    pub sbc_bitpool: u8,
    pub avdtp_streaming: bool,
    pub a2dp_playing: bool,
    pub a2dp_track: u8,
    pub a2dp_packets_sent: u32,
}

impl BluetoothStatus {
    const fn empty() -> Self {
        Self {
            initialized: false,
            hci_up: false,
            fw_patched: false,
            baud_rate: 0,
            hci_version: 0,
            hci_revision: 0,
            manufacturer: 0,
            lmp_subversion: 0,
            bd_addr: [0; 6],
            ble_advertising: false,
            classic_discoverable: false,
            commands_sent: 0,
            device_count: 0,
            devices: [BtDeviceInfo::empty(); MAX_DISCOVERED_DEVICES],
            connected_index: None,
            acl_handle: 0,
            a2dp_ready: false,
            last_conn_err: 0,
            avdtp_sig_dcid: 0,
            avdtp_media_dcid: 0,
            acp_seid_shifted: 0x04,
            sbc_cap0: 0x22, // 44.1kHz (0x20) | Stereo (0x02)
            sbc_cap1: 0x15, // 16 blocks (0x10) | 8 subbands (0x04) | Loudness (0x01)
            sbc_bitpool: 30,
            avdtp_streaming: false,
            a2dp_playing: true,
            a2dp_track: 0,
            a2dp_packets_sent: 0,
        }
    }
}

static STATE: IrqSafeLock<BluetoothStatus> = IrqSafeLock::new(BluetoothStatus::empty());

pub fn status() -> BluetoothStatus {
    STATE.with(|s| *s)
}

unsafe fn setup_bt_gpio() {
    let mut fsel1 = mmio::read(GPFSEL1);
    fsel1 &= !((0b111 << 12) | (0b111 << 15));
    mmio::write(GPFSEL1, fsel1);

    let mut fsel3 = mmio::read(GPFSEL3);
    for pin in 30..=33 {
        let shift = (pin - 30) * 3;
        fsel3 = (fsel3 & !(0b111 << shift)) | (0b111 << shift);
    }
    mmio::write(GPFSEL3, fsel3);

    mmio::write(GPPUD, 2);
    for _ in 0..150 {
        core::hint::spin_loop();
    }
    mmio::write(GPPUDCLK1, 1 << (33 - 32));
    for _ in 0..150 {
        core::hint::spin_loop();
    }
    mmio::write(GPPUD, 0);
    mmio::write(GPPUDCLK0, 0);
    mmio::write(GPPUDCLK1, 0);

    let mut fsel4 = mmio::read(GPFSEL4);
    fsel4 = (fsel4 & !(0b111 << 6)) | (0b001 << 6);
    mmio::write(GPFSEL4, fsel4);
    mmio::write(GPSET1, 1 << (42 - 32));
}

unsafe fn reset_bt_chip_via_gpio42() {
    let bt_on_bit = 1u32 << (42 - 32);
    mmio::write(GPCLR1, bt_on_bit);
    timer::delay_ms(30);
    mmio::write(GPSET1, bt_on_bit);
    timer::delay_ms(140);
}

unsafe fn configure_uart0(ibrd: u32, fbrd: u32, hw_flow_ctrl: bool) {
    let mut guard = 10_000u32;
    while (mmio::read(UART_FR) & (1 << 3)) != 0 && guard > 0 {
        guard -= 1;
        core::hint::spin_loop();
    }
    mmio::write(UART_CR, 0);
    mmio::write(UART_ICR, 0x7FF);

    mmio::write(UART_IBRD, ibrd);
    mmio::write(UART_FBRD, fbrd);
    mmio::write(UART_IFLS, 0x00);
    mmio::write(UART_LCRH, (1 << 4) | (1 << 5) | (1 << 6));

    let mut cr = (1 << 0) | (1 << 8) | (1 << 9) | (1 << 11) | (1 << 14);
    if hw_flow_ctrl {
        cr |= 1 << 15;
    }
    mmio::write(UART_CR, cr);
    timer::delay_ms(2);

    drain_uart0_rx(2048);
}

/// Drain any unread bytes from `UART0` RX FIFO + give a brief settle window so stale events are cleared.
unsafe fn drain_uart0_rx(max_bytes: u32) {
    let mut drain = max_bytes;
    while drain > 0 {
        if (mmio::read(UART_FR) & FR_RXFE) == 0 {
            let _ = mmio::read(UART_DR);
            drain -= 1;
        } else {
            // Wait ~60us to see if another byte is shifting in on the wire
            let mut sub = 400u32;
            while sub > 0 && (mmio::read(UART_FR) & FR_RXFE) != 0 {
                sub -= 1;
                core::hint::spin_loop();
            }
            if (mmio::read(UART_FR) & FR_RXFE) != 0 {
                break;
            }
        }
    }
}

unsafe fn uart0_putc(b: u8) -> bool {
    let mut timeout = 80_000u32;
    while (mmio::read(UART_FR) & FR_TXFF) != 0 {
        timeout -= 1;
        if timeout == 0 {
            return false;
        }
        core::hint::spin_loop();
    }
    mmio::write(UART_DR, b as u32);
    true
}

unsafe fn uart0_getc_timeout(timeout_ms: u32) -> Option<u8> {
    let freq = timer::frequency();
    let deadline = timer::counter() + (freq * (timeout_ms as u64)) / 1000;
    loop {
        if (mmio::read(UART_FR) & FR_RXFE) == 0 {
            return Some((mmio::read(UART_DR) & 0xFF) as u8);
        }
        if timer::counter() >= deadline {
            return None;
        }
        core::hint::spin_loop();
    }
}

unsafe fn hci_write_raw_cmd(opcode: u16, params: &[u8]) -> bool {
    let op_lo = (opcode & 0xFF) as u8;
    let op_hi = ((opcode >> 8) & 0xFF) as u8;
    if !uart0_putc(0x01)
        || !uart0_putc(op_lo)
        || !uart0_putc(op_hi)
        || !uart0_putc(params.len() as u8)
    {
        return false;
    }
    for (idx, &b) in params.iter().enumerate() {
        if !uart0_putc(b) {
            return false;
        }
        if (idx & 0x0B) == 0x0B {
            let mut guard = 20_000u32;
            while (mmio::read(UART_FR) & (1 << 3)) != 0 && guard > 0 {
                guard -= 1;
                core::hint::spin_loop();
            }
        }
    }
    STATE.with(|s| s.commands_sent += 1);
    true
}

/// Send an L2CAP frame encapsulated inside an HCI ACL Data Packet (`0x02`) over `UART0`.
unsafe fn hci_send_l2cap(handle: u16, cid: u16, l2cap_payload: &[u8]) -> bool {
    let l2cap_len = l2cap_payload.len() as u16;
    let acl_len = l2cap_len + 4;
    // PB flag = 0b00 (`flags 0x00`: First non-automatically-flushable packet, required by BLE ATT & L2CAP)
    let hdr_handle = handle & 0x0FFF;

    let header: [u8; 9] = [
        0x02,
        (hdr_handle & 0xFF) as u8,
        (hdr_handle >> 8) as u8,
        (acl_len & 0xFF) as u8,
        (acl_len >> 8) as u8,
        (l2cap_len & 0xFF) as u8,
        (l2cap_len >> 8) as u8,
        (cid & 0xFF) as u8,
        (cid >> 8) as u8,
    ];
    for &b in &header {
        if !uart0_putc(b) {
            return false;
        }
    }
    for (idx, &b) in l2cap_payload.iter().enumerate() {
        if !uart0_putc(b) {
            return false;
        }
        if (idx & 0x0F) == 0x0F {
            let mut guard = 8_000u32;
            while (mmio::read(UART_FR) & (1 << 3)) != 0 && guard > 0 {
                guard -= 1;
                core::hint::spin_loop();
            }
        }
    }
    true
}

/// Unified H4 Packet Reader: parses both `0x04` (HCI Event) and `0x02` (HCI ACL Data) packets
/// so incoming ACL L2CAP frames from a Bluetooth speaker never corrupt H4 framing!
///
/// Returns:
///   - `Some((0x04, evt_code, plen))` for HCI Events (`payload[..plen]`)
///   - `Some((0x02, 0, acl_len))` for HCI ACL Data (`payload[..acl_len]` starts with `[l2cap_len_lo, l2cap_len_hi, cid_lo, cid_hi, ...]`)
unsafe fn hci_read_packet(payload: &mut [u8; 260], timeout_ms: u32) -> Option<(u8, u8, usize)> {
    let freq = timer::frequency();
    let deadline = timer::counter() + (freq * (timeout_ms as u64)) / 1000;

    loop {
        let rem_ticks = deadline.saturating_sub(timer::counter());
        if rem_ticks == 0 {
            return None;
        }
        let wait_ms = ((rem_ticks * 1000) / freq).max(1) as u32;
        let pkt_type = uart0_getc_timeout(wait_ms.min(50))?;

        match pkt_type {
            0x04 => {
                let evt_code = uart0_getc_timeout(40)?;
                let plen = uart0_getc_timeout(40)? as usize;
                for i in 0..plen {
                    let b = uart0_getc_timeout(40).unwrap_or(0);
                    if i < payload.len() {
                        payload[i] = b;
                    }
                }
                return Some((0x04, evt_code, plen.min(payload.len())));
            }
            0x02 => {
                let _h_lo = uart0_getc_timeout(40)?;
                let h_hi = uart0_getc_timeout(40)?;
                let pb_flag = (h_hi >> 4) & 0x03;
                let dlen_lo = uart0_getc_timeout(40)?;
                let dlen_hi = uart0_getc_timeout(40)?;
                let dlen = u16::from_le_bytes([dlen_lo, dlen_hi]) as usize;
                for i in 0..dlen {
                    let b = uart0_getc_timeout(40).unwrap_or(0);
                    if i < payload.len() {
                        payload[i] = b;
                    }
                }
                return Some((0x02, pb_flag, dlen.min(payload.len())));
            }
            _ => {
                // Keep scanning for H4 packet header (0x04 or 0x02)
            }
        }
    }
}

/// Read the next HCI Event packet (`0x04`), ignoring any interleaved `0x02` ACL packets cleanly.
unsafe fn hci_read_event(payload: &mut [u8; 260], timeout_ms: u32) -> Option<(u8, usize)> {
    let freq = timer::frequency();
    let deadline = timer::counter() + (freq * (timeout_ms as u64)) / 1000;

    while timer::counter() < deadline {
        let rem_ms = (((deadline.saturating_sub(timer::counter())) * 1000) / freq).max(1) as u32;
        let (pkt_type, evt_code, len) = hci_read_packet(payload, rem_ms)?;
        if pkt_type == 0x04 {
            return Some((evt_code, len));
        }
    }
    None
}

/// Send an HCI H4 Command packet and wait for the matching `Command Complete` (`0x0E`).
unsafe fn hci_send_cmd(
    opcode: u16,
    params: &[u8],
    resp_buf: &mut [u8],
    timeout_ms: u32,
) -> Result<usize, u32> {
    drain_uart0_rx(512);

    if !hci_write_raw_cmd(opcode, params) {
        return Err(0xE001);
    }

    let mut evt_payload = [0u8; 260];
    for _evt_try in 0..24 {
        let Some((evt_code, plen)) = hci_read_event(&mut evt_payload, timeout_ms) else {
            return Err(0xE003);
        };

        if evt_code == 0x0E && plen >= 4 {
            let rx_op = u16::from_le_bytes([evt_payload[1], evt_payload[2]]);
            if rx_op != opcode {
                continue;
            }
            let status = evt_payload[3];
            let ret_len = (plen - 4).min(resp_buf.len());
            resp_buf[..ret_len].copy_from_slice(&evt_payload[4..4 + ret_len]);
            if status == 0x00 {
                return Ok(ret_len);
            } else {
                return Err(0xE100 | (status as u32));
            }
        }
    }

    Err(0xE2FF)
}

unsafe fn load_bcm43430_hcd_patch(current_baud: &mut u32) -> bool {
    let Some(hcd) = fat32::read_file("BTFW.HCD") else {
        return false;
    };
    if hcd.len() < 64 {
        return false;
    }

    let mut resp = [0u8; 16];

    let baud_921k: [u8; 6] = [0x00, 0x00, 0x00, 0x10, 0x0E, 0x00];
    if *current_baud != 921_600 && hci_send_cmd(0xFC18, &baud_921k, &mut resp, 80).is_ok() {
        timer::delay_ms(10);
        configure_uart0(3, 16, false);
        *current_baud = 921_600;
    }

    if hci_send_cmd(0xFC2E, &[], &mut resp, 100).is_err() {
        configure_uart0(26, 3, false);
        *current_baud = 115_200;
        if hci_send_cmd(0xFC2E, &[], &mut resp, 100).is_err() {
            return false;
        }
    }
    timer::delay_ms(50);

    let mut off = 0usize;
    let mut rec_count = 0u32;
    while off + 3 <= hcd.len() {
        let op = u16::from_le_bytes([hcd[off], hcd[off + 1]]);
        let plen = hcd[off + 2] as usize;
        off += 3;
        if off + plen > hcd.len() {
            break;
        }
        let _ = hci_send_cmd(op, &hcd[off..off + plen], &mut resp, 100);
        off += plen;
        rec_count += 1;
        if (rec_count & 0x3F) == 0 {
            tryboot::watchdog_feed(15);
        }
    }

    timer::delay_ms(280);
    configure_uart0(26, 3, false);
    *current_baud = 115_200;

    if hci_send_cmd(0x0C03, &[], &mut resp, 250).is_ok() {
        if hci_send_cmd(0xFC18, &baud_921k, &mut resp, 80).is_ok() {
            timer::delay_ms(10);
            configure_uart0(3, 16, false);
            *current_baud = 921_600;
        }
        return true;
    }
    configure_uart0(26, 3, true);
    hci_send_cmd(0x0C03, &[], &mut resp, 250).is_ok()
}

struct ParsedAdInfo<'a> {
    name: Option<&'a [u8]>,
    vendor_hint: Option<&'static str>,
    is_audio: bool,
}

#[inline]
fn is_audio_speaker_cod(cod: u32) -> bool {
    let major = (cod >> 8) & 0x1F;
    let minor = (cod >> 2) & 0x3F;
    // Exclude Video Displays / TVs / VCRs / Camcorders (Major 0x04, Minor 0x0B..=0x13; e.g. Office TV has Minor 0x0F)
    if major == 0x04 && matches!(minor, 0x0B..=0x13) {
        return false;
    }
    // Real audio speakers/headphones either have Major 0x04 (Headset/Handsfree/Loudspeaker/Headphones/HiFi)
    // OR have BOTH Audio (0x200000) and Rendering (0x040000) service bits set (`0x240000`).
    major == 0x04 || (cod & 0x240000) == 0x240000
}

#[inline]
fn trim_ad_name(raw: &[u8]) -> &[u8] {
    let mut end = 0usize;
    while end < raw.len() && raw[end] != 0 {
        end += 1;
    }
    let mut slice = &raw[..end];
    while let Some(&last) = slice.last() {
        if last == b' ' || last == b'\t' || last == b'\r' || last == b'\n' {
            slice = &slice[..slice.len() - 1];
        } else {
            break;
        }
    }
    slice
}

fn parse_ad_payload(ad: &[u8]) -> ParsedAdInfo<'_> {
    let mut idx = 0usize;
    let mut name: Option<&[u8]> = None;
    let mut vendor_hint: Option<&'static str> = None;
    let mut is_audio = false;

    while idx < ad.len() {
        let ad_len = ad[idx] as usize;
        if ad_len == 0 || idx + 1 + ad_len > ad.len() {
            break;
        }
        let ad_type = ad[idx + 1];
        let body = &ad[idx + 2..idx + 1 + ad_len];

        match ad_type {
            0x09 if !body.is_empty() => {
                let trimmed = trim_ad_name(body);
                if !trimmed.is_empty() {
                    name = Some(trimmed);
                }
            }
            0x08 if !body.is_empty() && name.is_none() => {
                let trimmed = trim_ad_name(body);
                if !trimmed.is_empty() {
                    name = Some(trimmed);
                }
            }
            0x02 | 0x03 => {
                let mut u = 0usize;
                while u + 2 <= body.len() {
                    let uuid = u16::from_le_bytes([body[u], body[u + 1]]);
                    // Note: Do NOT include 0x110A (Audio Source, advertised by TVs/phones);
                    // only match Audio Sink (0x110B), A2DP (0x110D), Headset (0x1108), Handsfree (0x111E), etc.
                    if matches!(
                        uuid,
                        0x110B
                            | 0x110D
                            | 0x1108
                            | 0x111E
                            | 0x184E
                            | 0xFE2C
                            | 0xFEBE
                            | 0xFDDF
                    ) {
                        is_audio = true;
                    }
                    u += 2;
                }
            }
            0x16 if body.len() >= 2 => {
                let uuid = u16::from_le_bytes([body[0], body[1]]);
                if matches!(uuid, 0xFE2C | 0x184E | 0xFEBE | 0xFDDF) {
                    is_audio = true;
                }
                if uuid == 0xFE2C
                    && body.len() >= 5
                    && body[2..].windows(3).any(|w| w == [0x1C, 0x62, 0xE8])
                    && name.is_none()
                {
                    name = Some(b"SRS-XB100");
                }
            }
            0x19 if body.len() >= 2 => {
                let app = u16::from_le_bytes([body[0], body[1]]);
                if (0x0800..=0x09FF).contains(&app) {
                    is_audio = true;
                }
            }
            0xFF if body.len() >= 2 => {
                let cid = u16::from_le_bytes([body[0], body[1]]);
                if cid == 0x012D {
                    is_audio = true;
                    // Sony Corporation (0x012D): Fast Pair Model ID `1C 62 E8` = Sony SRS-XB100
                    if body.len() >= 5
                        && body[2..].windows(3).any(|w| w == [0x1C, 0x62, 0xE8])
                        && name.is_none()
                    {
                        name = Some(b"SRS-XB100");
                    }
                }
                if vendor_hint.is_none() {
                    vendor_hint = match cid {
                        0x004C => {
                            if body.len() >= 3 && body[2] == 0x07 {
                                is_audio = true;
                                Some("AirPods/Beats")
                            } else {
                                Some("Apple 设备")
                            }
                        }
                        0x038F | 0x0157 => Some("小米蓝牙"),
                        0x0075 => Some("三星蓝牙"),
                        0x027D => Some("华为蓝牙"),
                        0x012D => Some("Sony 音频"),
                        0x009E => {
                            is_audio = true;
                            Some("Bose 音箱")
                        }
                        0x0057 => {
                            is_audio = true;
                            Some("JBL/Harman")
                        }
                        0x0006 => Some("微软设备"),
                        _ => None,
                    };
                }
            }
            _ => {}
        }
        idx += 1 + ad_len;
    }

    ParsedAdInfo {
        name,
        vendor_hint,
        is_audio,
    }
}

fn set_fallback_name(dev: &mut BtDeviceInfo, vendor: Option<&str>) {
    if dev.has_real_name {
        return;
    }
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    let prefix = vendor.unwrap_or("蓝牙");
    let pb = prefix.as_bytes();
    let mut buf = [0u8; 40];
    let p_len = pb.len().min(28);
    buf[..p_len].copy_from_slice(&pb[..p_len]);
    buf[p_len] = b' ';
    buf[p_len + 1] = HEX[(dev.bd_addr[3] >> 4) as usize];
    buf[p_len + 2] = HEX[(dev.bd_addr[3] & 0xF) as usize];
    buf[p_len + 3] = b':';
    buf[p_len + 4] = HEX[(dev.bd_addr[4] >> 4) as usize];
    buf[p_len + 5] = HEX[(dev.bd_addr[4] & 0xF) as usize];
    buf[p_len + 6] = b':';
    buf[p_len + 7] = HEX[(dev.bd_addr[5] >> 4) as usize];
    buf[p_len + 8] = HEX[(dev.bd_addr[5] & 0xF) as usize];
    dev.set_name_bytes(&buf[..p_len + 9], false);
}

fn upsert_discovered_device(
    list: &mut [BtDeviceInfo; MAX_DISCOVERED_DEVICES],
    count: &mut usize,
    bd_le: &[u8; 6],
    cod: u32,
    rssi: i8,
    page_scan_rep: u8,
    clock_offset: u16,
    ad_info: &ParsedAdInfo<'_>,
    is_ble: bool,
    le_addr_type: u8,
) {
    let bd_disp = [
        bd_le[5], bd_le[4], bd_le[3], bd_le[2], bd_le[1], bd_le[0],
    ];
    if bd_disp == [0, 0, 0, 0, 0, 0] {
        return;
    }

    let is_audio = ad_info.is_audio || is_audio_speaker_cod(cod);
    let clean_name = ad_info.name.map(trim_ad_name).filter(|n| !n.is_empty());

    for i in 0..*count {
        let same_bd = list[i].bd_addr == bd_disp
            || (list[i].has_alt_bd && list[i].alt_bd_addr == bd_disp);
        let same_real_name = match clean_name {
            Some(n) if list[i].has_real_name && list[i].name_len > 0 => {
                list[i].name_str().as_bytes() == n
            }
            _ => false,
        };

        if same_bd || same_real_name {
            if cod != 0 {
                list[i].cod = cod;
            }
            list[i].is_audio |= is_audio;
            if !is_ble {
                // Classic BR/EDR takes priority as primary `bd_addr` for A2DP audio!
                if list[i].is_ble && list[i].bd_addr != bd_disp {
                    list[i].alt_bd_addr = list[i].bd_addr;
                    list[i].has_alt_bd = true;
                    list[i].bd_addr = bd_disp;
                }
                list[i].is_ble = false;
                list[i].page_scan_rep = page_scan_rep;
                if clock_offset != 0 {
                    list[i].clock_offset = clock_offset;
                }
            } else {
                list[i].le_addr_type = le_addr_type & 0x01;
                if !list[i].is_ble && list[i].bd_addr != bd_disp {
                    list[i].alt_bd_addr = bd_disp;
                    list[i].has_alt_bd = true;
                }
            }
            if rssi != 0 && rssi > list[i].rssi {
                list[i].rssi = rssi;
            }
            if let Some(n) = clean_name {
                if !list[i].has_real_name || n.len() >= (list[i].name_len as usize) {
                    list[i].set_name_bytes(n, true);
                }
            } else if !list[i].has_real_name && ad_info.vendor_hint.is_some() {
                set_fallback_name(&mut list[i], ad_info.vendor_hint);
            }
            return;
        }
    }

    let new_score = (is_audio as i32) * 1000
        + (clean_name.is_some() as i32) * 500
        + ((!is_ble) as i32) * 300
        + (rssi as i32);

    if *count >= MAX_DISCOVERED_DEVICES {
        if let Some((replace_idx, worst_score)) = list[..*count]
            .iter()
            .enumerate()
            .map(|(idx, d)| {
                let s = (d.is_audio as i32) * 1000
                    + (d.has_real_name as i32) * 500
                    + ((!d.is_ble) as i32) * 300
                    + (d.rssi as i32);
                (idx, s)
            })
            .min_by_key(|&(_, s)| s)
        {
            if new_score > worst_score {
                let mut info = BtDeviceInfo::empty();
                info.valid = true;
                info.bd_addr = bd_disp;
                info.cod = cod;
                info.rssi = rssi;
                info.page_scan_rep = page_scan_rep;
                info.clock_offset = clock_offset;
                info.is_audio = is_audio;
                info.is_ble = is_ble;
                info.le_addr_type = le_addr_type & 0x01;
                if let Some(n) = clean_name {
                    info.set_name_bytes(n, true);
                } else {
                    set_fallback_name(&mut info, ad_info.vendor_hint);
                }
                list[replace_idx] = info;
            }
        }
        return;
    }

    let mut info = BtDeviceInfo::empty();
    info.valid = true;
    info.bd_addr = bd_disp;
    info.cod = cod;
    info.rssi = rssi;
    info.page_scan_rep = page_scan_rep;
    info.clock_offset = clock_offset;
    info.is_audio = is_audio;
    info.is_ble = is_ble;
    info.le_addr_type = le_addr_type & 0x01;
    if let Some(n) = clean_name {
        info.set_name_bytes(n, true);
    } else {
        set_fallback_name(&mut info, ad_info.vendor_hint);
    }
    list[*count] = info;
    *count += 1;
}

/// Actively scan for nearby Classic Bluetooth + BLE speakers, headphones, phones & devices (`M10.1`).
pub fn scan_devices() -> usize {
    if bsp::is_qemu() {
        STATE.with(|s| {
            let demo: [(&str, [u8; 6], u32, i8, bool, bool); 6] = [
                ("SRS-X100", [0x48, 0xD6, 0xD5, 0x19, 0x82, 0x01], 0x240414, -44, true, false),
                ("AirPods Pro", [0x90, 0x81, 0x58, 0x33, 0xA4, 0x12], 0x240418, -51, true, false),
                ("JBL GO 3 音箱", [0x70, 0x99, 0x1C, 0x08, 0x55, 0x2A], 0x240414, -56, true, false),
                ("Han 的 iPhone", [0x64, 0xB0, 0xA6, 0x11, 0x4E, 0x09], 0x5A020C, -62, false, false),
                ("MacBook Pro", [0xAC, 0xDE, 0x48, 0x00, 0x11, 0x22], 0x10010C, -67, false, false),
                ("小米温湿度计 BLE", [0xA4, 0xC1, 0x38, 0x77, 0x19, 0x04], 0x000000, -72, false, true),
            ];
            for (i, &(name, bd, cod, rssi, audio, ble)) in demo.iter().enumerate() {
                let mut d = BtDeviceInfo::empty();
                d.valid = true;
                d.bd_addr = bd;
                d.cod = cod;
                d.rssi = rssi;
                d.is_audio = audio;
                d.is_ble = ble;
                d.set_name_bytes(name.as_bytes(), true);
                s.devices[i] = d;
            }
            s.device_count = demo.len();
        });
        return 6;
    }

    let prev_snap = status();
    if !prev_snap.hci_up {
        init();
    }

    BT_CMD_ACTIVE.store(true, Ordering::Relaxed);

    let mut list = [BtDeviceInfo::empty(); MAX_DISCOVERED_DEVICES];
    let mut count = 0usize;

    unsafe {
        let mut resp = [0u8; 32];
        let mut evt = [0u8; 260];

        // 0. Pause Slave BLE Advertising & Inquiry/Page Scan so 100% of radio is dedicated to Master Scan,
        //    and explicitly set HCI_Event_Mask (0x0C01) + LE_Event_Mask (0x2001) so Bit 46 (0x2F Extended Inquiry Result),
        //    Bit 33 (0x22 Inquiry Result with RSSI), SSP (0x31..0x36), and Bit 61 (0x3E LE Meta Event) are ALL enabled!
        let _ = hci_send_cmd(0x200A, &[0x00], &mut resp, 60);
        let _ = hci_send_cmd(0x200C, &[0x00, 0x00], &mut resp, 60);
        let _ = hci_send_cmd(0x0C1A, &[0x00], &mut resp, 60);
        let _ = hci_send_cmd(
            0x0C01,
            &[0xFF, 0xFF, 0xFB, 0xFF, 0x07, 0xF8, 0xBF, 0x3D],
            &mut resp,
            60,
        );
        let _ = hci_send_cmd(
            0x2001,
            &[0xFF, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            &mut resp,
            60,
        );
        let _ = hci_send_cmd(0x0C56, &[0x01], &mut resp, 60);
        let _ = hci_send_cmd(0x0C45, &[0x02], &mut resp, 80);
        // Set Page_Timeout (0x0C18) to 0x0E00 (2.24s) so Remote Name Requests never hang for 5.12s
        let _ = hci_send_cmd(0x0C18, &[0x00, 0x0E], &mut resp, 60);
        drain_uart0_rx(2048);

        // 1. Classic Bluetooth Inquiry (0x0401): scan GIAC (0x9E8B33, 5.12s) + LIAC (0x9E8B00, 1.28s if no classic speaker yet)
        let inq_passes: [[u8; 5]; 2] = [
            [0x33, 0x8B, 0x9E, 0x04, 0x00], // GIAC (General Inquiry, 5.12s)
            [0x00, 0x8B, 0x9E, 0x01, 0x00], // LIAC (Limited Inquiry, 1.28s for speakers in Limited Discoverable Mode)
        ];
        for (pass_idx, inq_params) in inq_passes.iter().enumerate() {
            if pass_idx == 1 && list[..count].iter().any(|d| d.valid && !d.is_ble && d.is_audio) {
                break;
            }
            if hci_write_raw_cmd(0x0401, inq_params) {
                let freq = timer::frequency();
                let wait_tenths = if pass_idx == 0 { 54 } else { 16 };
                let scan_deadline = timer::counter() + freq * wait_tenths / 10;
                let mut got_inq_complete = false;

                while timer::counter() < scan_deadline {
                    tryboot::watchdog_feed(15);
                    let Some((evt_code, plen)) = hci_read_event(&mut evt, 200) else {
                        continue;
                    };

                    match evt_code {
                        0x01 => {
                            got_inq_complete = true;
                            break;
                        }
                        0x2F if plen >= 15 => {
                            let mut bd_le = [0u8; 6];
                            bd_le.copy_from_slice(&evt[1..7]);
                            let page_rep = evt[7];
                            let cod = (evt[11] as u32) << 16 | (evt[10] as u32) << 8 | (evt[9] as u32);
                            let clk_off = u16::from_le_bytes([evt[12], evt[13]]);
                            let rssi = evt[14] as i8;
                            let ad_info = parse_ad_payload(&evt[15..plen]);
                            upsert_discovered_device(
                                &mut list,
                                &mut count,
                                &bd_le,
                                cod,
                                rssi,
                                page_rep,
                                clk_off,
                                &ad_info,
                                false,
                                0,
                            );
                        }
                        0x22 if plen >= 15 => {
                            let num = (evt[0] as usize).max(1);
                            for r in 0..num {
                                if 1 + 6 * (r + 1) > plen {
                                    break;
                                }
                                let mut bd_le = [0u8; 6];
                                bd_le.copy_from_slice(&evt[1 + 6 * r..1 + 6 * (r + 1)]);
                                let page_rep = *evt.get(1 + 6 * num + r).unwrap_or(&1);
                                let cod_off = 1 + 8 * num + 3 * r;
                                let cod = if cod_off + 3 <= plen {
                                    (evt[cod_off + 2] as u32) << 16
                                        | (evt[cod_off + 1] as u32) << 8
                                        | (evt[cod_off] as u32)
                                } else {
                                    0
                                };
                                let clk_off_idx = 1 + 11 * num + 2 * r;
                                let clk_off = if clk_off_idx + 2 <= plen {
                                    u16::from_le_bytes([evt[clk_off_idx], evt[clk_off_idx + 1]])
                                } else {
                                    0
                                };
                                let rssi_idx = 1 + 13 * num + r;
                                let rssi = if rssi_idx < plen {
                                    evt[rssi_idx] as i8
                                } else {
                                    -65
                                };
                                let empty_ad = ParsedAdInfo {
                                    name: None,
                                    vendor_hint: None,
                                    is_audio: false,
                                };
                                upsert_discovered_device(
                                    &mut list,
                                    &mut count,
                                    &bd_le,
                                    cod,
                                    rssi,
                                    page_rep,
                                    clk_off,
                                    &empty_ad,
                                    false,
                                    0,
                                );
                            }
                        }
                        0x02 if plen >= 15 => {
                            let num = (evt[0] as usize).max(1);
                            for r in 0..num {
                                if 1 + 6 * (r + 1) > plen {
                                    break;
                                }
                                let mut bd_le = [0u8; 6];
                                bd_le.copy_from_slice(&evt[1 + 6 * r..1 + 6 * (r + 1)]);
                                let page_rep = *evt.get(1 + 6 * num + r).unwrap_or(&1);
                                let cod_off = 1 + 9 * num + 3 * r;
                                let cod = if cod_off + 3 <= plen {
                                    (evt[cod_off + 2] as u32) << 16
                                        | (evt[cod_off + 1] as u32) << 8
                                        | (evt[cod_off] as u32)
                                } else {
                                    0
                                };
                                let empty_ad = ParsedAdInfo {
                                    name: None,
                                    vendor_hint: None,
                                    is_audio: false,
                                };
                                upsert_discovered_device(
                                    &mut list,
                                    &mut count,
                                    &bd_le,
                                    cod,
                                    -65,
                                    page_rep,
                                    0,
                                    &empty_ad,
                                    false,
                                    0,
                                );
                            }
                        }
                        _ => {}
                    }
                }
                if !got_inq_complete {
                    let _ = hci_send_cmd(0x0402, &[], &mut resp, 60);
                }
                drain_uart0_rx(1024);
            }
        }

        // 2. Resolve missing names via `HCI_Remote_Name_Request` (0x0419) for unnamed Classic devices
        //    Prioritize unnamed Classic Audio devices (`is_audio = true`) first!
        let mut resolved = 0usize;
        for audio_pass in [true, false] {
            for i in 0..count {
                if !list[i].has_real_name
                    && !list[i].is_ble
                    && list[i].is_audio == audio_pass
                    && resolved < 3
                {
                    tryboot::watchdog_feed(15);
                    let bd_disp = list[i].bd_addr;
                    let bd_le = [
                        bd_disp[5], bd_disp[4], bd_disp[3], bd_disp[2], bd_disp[1], bd_disp[0],
                    ];
                    let clk_hi = if list[i].clock_offset != 0 {
                        (((list[i].clock_offset >> 8) & 0x7F) as u8) | 0x80
                    } else {
                        0x00
                    };
                    let rnr_params: [u8; 10] = [
                        bd_le[0],
                        bd_le[1],
                        bd_le[2],
                        bd_le[3],
                        bd_le[4],
                        bd_le[5],
                        list[i].page_scan_rep.min(2),
                        0x00,
                        (list[i].clock_offset & 0xFF) as u8,
                        clk_hi,
                    ];
                    if hci_write_raw_cmd(0x0419, &rnr_params) {
                        let freq = timer::frequency();
                        let rnr_deadline = timer::counter() + freq * 24 / 10;
                        let mut got_rnr_complete = false;
                        while timer::counter() < rnr_deadline {
                            tryboot::watchdog_feed(15);
                            let Some((evt_code, plen)) = hci_read_event(&mut evt, 180) else {
                                continue;
                            };
                            if evt_code == 0x07 && plen >= 7 && evt[1..7] == bd_le {
                                got_rnr_complete = true;
                                if evt[0] == 0x00 && plen >= 8 {
                                    list[i].set_name_bytes(&evt[7..plen], true);
                                }
                                break;
                            }
                        }
                        if !got_rnr_complete {
                            let _ = hci_send_cmd(0x041A, &bd_le, &mut resp, 60);
                            let cancel_deadline = timer::counter() + freq * 2 / 10;
                            while timer::counter() < cancel_deadline {
                                let Some((ec, pl)) = hci_read_event(&mut evt, 50) else {
                                    break;
                                };
                                if ec == 0x07 && pl >= 7 {
                                    break;
                                }
                            }
                        }
                    }
                    resolved += 1;
                }
            }
        }
        let _ = hci_send_cmd(0x0C18, &[0x00, 0x20], &mut resp, 60);
        drain_uart0_rx(2048);

        // 3. Perform two 1.2s BLE Active Scan passes (0x200B + 0x200C) with `Filter_Duplicates = 0x01`
        //    so the 16-byte PL011 UART0 FIFO never overflows on iPhone/Mac beacon floods while still
        //    capturing both `ADV_IND` and `SCAN_RSP` (e.g. Sony SRS-XB100 Fast Pair + Local Name)!
        tryboot::watchdog_feed(15);
        let _ = hci_send_cmd(
            0x2005,
            &[0xD0, 0x55, 0x44, 0x33, 0x22, 0xC0],
            &mut resp,
            60,
        );
        let scan_params: [u8; 7] = [0x01, 0x12, 0x00, 0x12, 0x00, 0x00, 0x00];
        let _ = hci_send_cmd(0x200B, &scan_params, &mut resp, 60);

        for _ble_pass in 0..2 {
            if hci_send_cmd(0x200C, &[0x01, 0x01], &mut resp, 60).is_ok() {
                let freq = timer::frequency();
                let ble_deadline = timer::counter() + freq * 12 / 10;
                while timer::counter() < ble_deadline {
                    tryboot::watchdog_feed(15);
                    let Some((evt_code, plen)) = hci_read_event(&mut evt, 80) else {
                        continue;
                    };
                    if evt_code == 0x3E && plen >= 12 && evt[0] == 0x02 {
                        let num_reports = evt[1] as usize;
                        let mut pos = 2usize;
                        for _ in 0..num_reports {
                            if pos + 9 > plen {
                                break;
                            }
                            let addr_type = evt[pos + 1];
                            let mut bd_le = [0u8; 6];
                            bd_le.copy_from_slice(&evt[pos + 2..pos + 8]);
                            let data_len = evt[pos + 8] as usize;
                            pos += 9;
                            if pos + data_len + 1 > plen {
                                break;
                            }
                            let ad_slice = &evt[pos..pos + data_len];
                            let rssi = evt[pos + data_len] as i8;
                            pos += data_len + 1;

                            let ad_info = parse_ad_payload(ad_slice);
                            if ad_info.name.is_some()
                                || ad_info.is_audio
                                || ad_info.vendor_hint.is_some()
                                || rssi >= -78
                            {
                                upsert_discovered_device(
                                    &mut list,
                                    &mut count,
                                    &bd_le,
                                    0,
                                    rssi,
                                    1,
                                    0,
                                    &ad_info,
                                    true,
                                    addr_type,
                                );
                            }
                        }
                    }
                }
                let _ = hci_write_raw_cmd(0x200C, &[0x00, 0x00]);
                timer::delay_ms(20);
                drain_uart0_rx(4096);
            }
        }

        // 4. Restore Slave BLE Advertising ("ZerOS-Pi") + Classic Inquiry/Page Scan (0x03)
        let _ = hci_send_cmd(0x200A, &[0x01], &mut resp, 60);
        let _ = hci_send_cmd(0x0C1A, &[0x03], &mut resp, 60);
    }

    for i in 0..count {
        if list[i].name_len == 0 {
            set_fallback_name(&mut list[i], None);
        }
    }

    // Deduplicate dual-mode devices (e.g. Sony SRS-XB100 discovered on both Classic BR/EDR and BLE):
    // merge any two entries with identical names so Classic BR/EDR is kept as primary and only 1 row is shown!
    let mut i = 0usize;
    while i < count {
        let mut j = i + 1;
        while j < count {
            let same_name = list[i].name_len > 0
                && list[i].name_str() == list[j].name_str();
            if same_name {
                list[i].is_audio |= list[j].is_audio;
                if list[j].rssi > list[i].rssi {
                    list[i].rssi = list[j].rssi;
                }
                if list[i].is_ble && !list[j].is_ble {
                    list[i].alt_bd_addr = list[i].bd_addr;
                    list[i].has_alt_bd = true;
                    list[i].bd_addr = list[j].bd_addr;
                    list[i].is_ble = false;
                    list[i].cod = list[j].cod;
                    list[i].page_scan_rep = list[j].page_scan_rep;
                    list[i].clock_offset = list[j].clock_offset;
                } else if !list[i].is_ble && list[j].is_ble {
                    list[i].alt_bd_addr = list[j].bd_addr;
                    list[i].has_alt_bd = true;
                }
                for k in j..(count - 1) {
                    list[k] = list[k + 1];
                }
                list[count - 1] = BtDeviceInfo::empty();
                count -= 1;
            } else {
                j += 1;
            }
        }
        i += 1;
    }

    // Merge any unnamed Classic Audio Speaker with a named BLE Audio Speaker (e.g. SRS-XB100 when Classic RNR timed out)
    let mut b = 0usize;
    while b < count {
        if list[b].is_ble && list[b].is_audio && list[b].has_real_name {
            let mut merged_c: Option<usize> = None;
            for c in 0..count {
                if c != b && !list[c].is_ble && list[c].is_audio && !list[c].has_real_name {
                    list[b].alt_bd_addr = list[b].bd_addr;
                    list[b].has_alt_bd = true;
                    list[b].bd_addr = list[c].bd_addr;
                    list[b].is_ble = false;
                    list[b].cod = list[c].cod;
                    list[b].clock_offset = list[c].clock_offset;
                    list[b].page_scan_rep = list[c].page_scan_rep;
                    if list[c].rssi > list[b].rssi {
                        list[b].rssi = list[c].rssi;
                    }
                    merged_c = Some(c);
                    break;
                }
            }
            if let Some(c) = merged_c {
                for k in c..(count - 1) {
                    list[k] = list[k + 1];
                }
                list[count - 1] = BtDeviceInfo::empty();
                count -= 1;
                if c < b {
                    continue;
                }
            }
        }
        b += 1;
    }

    // Preserve link keys and any previously paired/known audio speaker that is now in non-discoverable Page Scan standby
    for p in 0..prev_snap.device_count.min(MAX_DISCOVERED_DEVICES) {
        let prev = &prev_snap.devices[p];
        if !prev.valid {
            continue;
        }
        let mut found = false;
        for i in 0..count {
            let same_bd = list[i].bd_addr == prev.bd_addr
                || (prev.has_alt_bd && list[i].bd_addr == prev.alt_bd_addr)
                || (list[i].has_alt_bd && list[i].alt_bd_addr == prev.bd_addr);
            let same_name = prev.has_real_name
                && list[i].has_real_name
                && list[i].name_str() == prev.name_str();
            if same_bd || same_name {
                found = true;
                if prev.has_link_key && !list[i].has_link_key {
                    list[i].has_link_key = true;
                    list[i].link_key = prev.link_key;
                }
                if !prev.is_ble && list[i].is_ble {
                    list[i].alt_bd_addr = list[i].bd_addr;
                    list[i].has_alt_bd = true;
                    list[i].bd_addr = prev.bd_addr;
                    list[i].is_ble = false;
                    list[i].cod = prev.cod;
                }
            }
        }
        if !found && prev.is_audio && prev.has_real_name && count < MAX_DISCOVERED_DEVICES {
            list[count] = *prev;
            count += 1;
        }
    }

    // Sort priority: Named Audio Speakers -> Audio -> Real Name -> Classic -> RSSI
    for i in 1..count {
        let mut j = i;
        while j > 0 {
            let a = &list[j - 1];
            let b = &list[j];
            let score_a = (a.is_audio as i32) * 1000
                + (a.has_real_name as i32) * 500
                + ((!a.is_ble) as i32) * 300
                + (a.rssi as i32);
            let score_b = (b.is_audio as i32) * 1000
                + (b.has_real_name as i32) * 500
                + ((!b.is_ble) as i32) * 300
                + (b.rssi as i32);
            if score_b > score_a {
                list.swap(j - 1, j);
                j -= 1;
            } else {
                break;
            }
        }
    }

    STATE.with(|s| {
        s.devices = list;
        s.device_count = count;
        s.connected_index = None;
        s.a2dp_ready = false;
        s.avdtp_streaming = false;
    });
    BT_CMD_ACTIVE.store(false, Ordering::Relaxed);

    count
}

struct ClassicConnState {
    avdtp_sig_dcid: u16,
    avdtp_media_dcid: u16,
    avrcp_dcid: u16,
    sdp_dcid: u16,
    other_dcid: u16,
    sig_out_cfg_sent: bool,
    sig_out_cfg_done: bool,
    sig_in_cfg_done: bool,
    media_out_cfg_sent: bool,
    media_out_cfg_done: bool,
    media_in_cfg_done: bool,
    avdtp_discover_sent: bool,
    avdtp_open_accepted: bool,
    media_l2cap_req_sent: bool,
    sink_seids: [u8; 4],
    sink_seid_count: usize,
    sink_seid_idx: usize,
    acp_seid_shifted: u8,
    sbc_cap0: u8,
    sbc_cap1: u8,
    sbc_bitpool: u8,
    avdtp_streaming: bool,
    info_rsp_received: bool,
    a2dp_ready: bool,
    ble_gatt_ready: bool,
    diag_stage: u8,
}

impl ClassicConnState {
    const fn new() -> Self {
        Self {
            avdtp_sig_dcid: 0,
            avdtp_media_dcid: 0,
            avrcp_dcid: 0,
            sdp_dcid: 0,
            other_dcid: 0,
            sig_out_cfg_sent: false,
            sig_out_cfg_done: false,
            sig_in_cfg_done: false,
            media_out_cfg_sent: false,
            media_out_cfg_done: false,
            media_in_cfg_done: false,
            avdtp_discover_sent: false,
            avdtp_open_accepted: false,
            media_l2cap_req_sent: false,
            sink_seids: [0x04, 0, 0, 0],
            sink_seid_count: 0,
            sink_seid_idx: 0,
            acp_seid_shifted: 0x04,
            sbc_cap0: 0x22,
            sbc_cap1: 0x15,
            sbc_bitpool: 30,
            avdtp_streaming: false,
            info_rsp_received: false,
            a2dp_ready: false,
            ble_gatt_ready: false,
            diag_stage: 0xE0,
        }
    }
}

struct AclReassembler {
    buf: [u8; 260],
    cur_len: usize,
    target_len: usize,
}

impl AclReassembler {
    const fn new() -> Self {
        Self {
            buf: [0u8; 260],
            cur_len: 0,
            target_len: 0,
        }
    }

    unsafe fn feed_and_dispatch(
        &mut self,
        handle: u16,
        pb_flag: u8,
        frag: &[u8],
        cs: &mut ClassicConnState,
    ) {
        if pb_flag == 0x01 {
            // Continuing fragment of an L2CAP PDU
            if self.target_len > 0 && self.cur_len < self.target_len {
                let copy_n = frag.len().min(self.target_len - self.cur_len);
                self.buf[self.cur_len..self.cur_len + copy_n].copy_from_slice(&frag[..copy_n]);
                self.cur_len += copy_n;
                if self.cur_len >= self.target_len {
                    let total = self.target_len;
                    self.target_len = 0;
                    handle_acl_l2cap_packet(handle, &self.buf[..total], cs);
                }
            }
        } else {
            // Start of a new L2CAP PDU (`pb_flag == 0x02` or `0x00`)
            if frag.len() < 4 {
                return;
            }
            let l2cap_len = u16::from_le_bytes([frag[0], frag[1]]) as usize;
            let total = (4 + l2cap_len).min(260);
            if frag.len() >= total {
                self.target_len = 0;
                handle_acl_l2cap_packet(handle, &frag[..total], cs);
            } else {
                let copy_n = frag.len().min(260);
                self.buf[..copy_n].copy_from_slice(&frag[..copy_n]);
                self.cur_len = copy_n;
                self.target_len = total;
            }
        }
    }
}

/// Process a complete reassembled L2CAP packet (`0x02`) during speaker connection setup:
/// handles L2CAP Signaling (`CID 0x0001`), AVDTP (`PSM 0x0019`), SDP (`PSM 0x0001`),
/// BLE ATT (`CID 0x0004`), BLE Signaling (`CID 0x0005`), and BLE SMP (`CID 0x0006`)
/// so Bluetooth speakers like **Sony SRS-XB100** complete full A2DP connection and play their connected chime!
unsafe fn handle_acl_l2cap_packet(
    handle: u16,
    acl_payload: &[u8],
    cs: &mut ClassicConnState,
) {
    if acl_payload.len() < 4 {
        return;
    }
    let l2cap_len = u16::from_le_bytes([acl_payload[0], acl_payload[1]]) as usize;
    let cid = u16::from_le_bytes([acl_payload[2], acl_payload[3]]);
    let body = &acl_payload[4..acl_payload.len().min(4 + l2cap_len)];

    if cid == 0x0001 && body.len() >= 4 {
        // ─── L2CAP Signaling Channel (CID = 0x0001) — may contain multiple back-to-back commands! ───
        let mut cmd_off = 0usize;
        while cmd_off + 4 <= body.len() {
            let code = body[cmd_off];
            let ident = body[cmd_off + 1];
            let cmd_len = u16::from_le_bytes([body[cmd_off + 2], body[cmd_off + 3]]) as usize;
            let end_off = (cmd_off + 4 + cmd_len).min(body.len());
            let data = &body[cmd_off + 4..end_off];
            cmd_off = end_off;

            match code {
                0x0A if data.len() >= 2 => {
                    // L2CAP Information Request -> Information Response (0x0B)
                    let info_type = u16::from_le_bytes([data[0], data[1]]);
                    if info_type == 0x0002 {
                        let rsp: [u8; 12] = [
                            0x0B, ident, 0x08, 0x00, 0x02, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00,
                        ];
                        let _ = hci_send_l2cap(handle, 0x0001, &rsp);
                    } else if info_type == 0x0003 {
                        let rsp: [u8; 16] = [
                            0x0B, ident, 0x0C, 0x00, 0x03, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00,
                        ];
                        let _ = hci_send_l2cap(handle, 0x0001, &rsp);
                    } else {
                        let rsp: [u8; 8] = [0x0B, ident, 0x04, 0x00, data[0], data[1], 0x01, 0x00];
                        let _ = hci_send_l2cap(handle, 0x0001, &rsp);
                    }
                }
                0x0B => {
                    // L2CAP Information Response from Speaker
                    cs.info_rsp_received = true;
                }
                0x02 if data.len() >= 4 => {
                    // L2CAP Connection Request from Speaker (e.g. SDP 0x0001, AVDTP 0x0019, AVRCP 0x0017)
                    let psm = u16::from_le_bytes([data[0], data[1]]);
                    let peer_scid = u16::from_le_bytes([data[2], data[3]]);
                    let local_cid: u16 = match psm {
                        0x0019 => {
                            if cs.avdtp_sig_dcid == 0 {
                                cs.avdtp_sig_dcid = peer_scid;
                                cs.sig_out_cfg_sent = true;
                                0x0040
                            } else {
                                cs.avdtp_media_dcid = peer_scid;
                                cs.media_out_cfg_sent = true;
                                0x0041
                            }
                        }
                        0x0017 => {
                            cs.avrcp_dcid = peer_scid;
                            0x0042
                        }
                        0x0001 => {
                            cs.sdp_dcid = peer_scid;
                            0x0044
                        }
                        _ => {
                            cs.other_dcid = peer_scid;
                            0x0045
                        }
                    };
                    // Connection Response (0x03): Result=0x0000 (Success)
                    let conn_rsp: [u8; 12] = [
                        0x03,
                        ident,
                        0x08,
                        0x00,
                        (local_cid & 0xFF) as u8,
                        (local_cid >> 8) as u8,
                        (peer_scid & 0xFF) as u8,
                        (peer_scid >> 8) as u8,
                        0x00,
                        0x00,
                        0x00,
                        0x00,
                    ];
                    let _ = hci_send_l2cap(handle, 0x0001, &conn_rsp);

                    // Send our Configuration Request (0x04) for peer_scid
                    let cfg_req: [u8; 12] = [
                        0x04,
                        ident.wrapping_add(1).max(1),
                        0x08,
                        0x00,
                        (peer_scid & 0xFF) as u8,
                        (peer_scid >> 8) as u8,
                        0x00,
                        0x00,
                        0x01,
                        0x02,
                        0xA0,
                        0x02,
                    ];
                    let _ = hci_send_l2cap(handle, 0x0001, &cfg_req);
                }
                0x03 if data.len() >= 8 => {
                    // L2CAP Connection Response to our AVDTP Connection Request
                    let dcid = u16::from_le_bytes([data[0], data[1]]);
                    let scid = u16::from_le_bytes([data[2], data[3]]);
                    let result = u16::from_le_bytes([data[4], data[5]]);
                    if result == 0x0000 && dcid != 0 {
                        if scid == 0x0040 {
                            cs.avdtp_sig_dcid = dcid;
                            if !cs.sig_out_cfg_sent {
                                cs.sig_out_cfg_sent = true;
                                let cfg_req: [u8; 12] = [
                                    0x04,
                                    ident.wrapping_add(1).max(1),
                                    0x08,
                                    0x00,
                                    (dcid & 0xFF) as u8,
                                    (dcid >> 8) as u8,
                                    0x00,
                                    0x00,
                                    0x01,
                                    0x02,
                                    0xA0,
                                    0x02,
                                ];
                                let _ = hci_send_l2cap(handle, 0x0001, &cfg_req);
                            }
                        } else if scid == 0x0041 {
                            cs.avdtp_media_dcid = dcid;
                            if !cs.media_out_cfg_sent {
                                cs.media_out_cfg_sent = true;
                                let cfg_req: [u8; 12] = [
                                    0x04,
                                    ident.wrapping_add(1).max(1),
                                    0x08,
                                    0x00,
                                    (dcid & 0xFF) as u8,
                                    (dcid >> 8) as u8,
                                    0x00,
                                    0x00,
                                    0x01,
                                    0x02,
                                    0xA0,
                                    0x02,
                                ];
                                let _ = hci_send_l2cap(handle, 0x0001, &cfg_req);
                            }
                        }
                    }
                }
                0x04 if data.len() >= 4 => {
                    // L2CAP Configuration Request from Speaker -> Configuration Response (0x05)
                    let dest_cid = u16::from_le_bytes([data[0], data[1]]);
                    let req_flags = u16::from_le_bytes([data[2], data[3]]);
                    let peer_cid = match dest_cid {
                        0x0040 if cs.avdtp_sig_dcid != 0 => cs.avdtp_sig_dcid,
                        0x0041 if cs.avdtp_media_dcid != 0 => cs.avdtp_media_dcid,
                        0x0042 if cs.avrcp_dcid != 0 => cs.avrcp_dcid,
                        0x0044 if cs.sdp_dcid != 0 => cs.sdp_dcid,
                        0x0045 if cs.other_dcid != 0 => cs.other_dcid,
                        _ => dest_cid,
                    };
                    let rsp_flags = req_flags & 0x0001;
                    let cfg_rsp: [u8; 10] = [
                        0x05,
                        ident,
                        0x06,
                        0x00,
                        (peer_cid & 0xFF) as u8,
                        (peer_cid >> 8) as u8,
                        (rsp_flags & 0xFF) as u8,
                        (rsp_flags >> 8) as u8,
                        0x00,
                        0x00,
                    ];
                    let _ = hci_send_l2cap(handle, 0x0001, &cfg_rsp);

                    if rsp_flags == 0 {
                        if dest_cid == 0x0040 {
                            cs.sig_in_cfg_done = true;
                        } else if dest_cid == 0x0041 {
                            cs.media_in_cfg_done = true;
                        }
                    }
                }
                0x05 if data.len() >= 6 => {
                    // L2CAP Configuration Response from Speaker
                    let src_cid = u16::from_le_bytes([data[0], data[1]]);
                    let rsp_flags = u16::from_le_bytes([data[2], data[3]]);
                    let result = u16::from_le_bytes([data[4], data[5]]);
                    if result == 0x0000 && (rsp_flags & 0x0001) == 0 {
                        if src_cid == 0x0041
                            || (cs.avdtp_media_dcid != 0 && src_cid == cs.avdtp_media_dcid)
                        {
                            cs.media_out_cfg_done = true;
                        } else if src_cid == 0x0040
                            || (cs.avdtp_sig_dcid != 0 && src_cid == cs.avdtp_sig_dcid)
                        {
                            cs.sig_out_cfg_done = true;
                        }
                    }
                }
                0x06 if data.len() >= 4 => {
                    // L2CAP Disconnection Request -> Disconnection Response (0x07)
                    let rsp: [u8; 8] = [
                        0x07, ident, 0x04, 0x00, data[0], data[1], data[2], data[3],
                    ];
                    let _ = hci_send_l2cap(handle, 0x0001, &rsp);
                }
                0x08 => {
                    // L2CAP Echo Request -> Echo Response (0x09)
                    let rsp: [u8; 4] = [0x09, ident, 0x00, 0x00];
                    let _ = hci_send_l2cap(handle, 0x0001, &rsp);
                }
                _ => {}
            }

            // Check if AVDTP Signaling channel (0x0040) has completed BOTH directions of L2CAP Configuration!
            if cs.sig_out_cfg_done
                && cs.sig_in_cfg_done
                && !cs.avdtp_discover_sent
                && cs.avdtp_sig_dcid != 0
            {
                cs.avdtp_discover_sent = true;
                cs.diag_stage = 0xE4;
                // Initiate AVDTP_DISCOVER (0x01) now that L2CAP channel 0x0040 is in OPEN state!
                let _ = hci_send_l2cap(handle, cs.avdtp_sig_dcid, &[0x10, 0x01]);
            }

            // Check if AVDTP Media Transport channel (0x0041) has completed BOTH directions of L2CAP Configuration!
            if cs.media_out_cfg_done && cs.media_in_cfg_done {
                cs.a2dp_ready = true;
            }
        }
    } else if cid == 0x0040 && body.len() >= 2 && cs.avdtp_sig_dcid != 0 {
        // ─── AVDTP Signaling Channel (local CID = 0x0040) ───
        let msg_type = body[0] & 0x03; // 0x00 = Command, 0x02 = Response Accept
        let tx_label = body[0] & 0xF0;
        let sig_id = body[1] & 0x3F;

        if msg_type == 0x00 {
            // Speaker sent us an AVDTP Command: accept it!
            match sig_id {
                0x01 => {
                    // AVDTP_DISCOVER -> reply with 1 Audio Source SEP (SEID=1, Media=Audio, TSEP=0=SRC)
                    let rsp = [tx_label | 0x02, 0x01, 0x04, 0x00];
                    let _ = hci_send_l2cap(handle, cs.avdtp_sig_dcid, &rsp);
                }
                0x02 | 0x0C => {
                    // AVDTP_GET_CAPABILITIES / GET_ALL_CAPABILITIES -> Media Transport + SBC Codec
                    let rsp = [
                        tx_label | 0x02,
                        sig_id,
                        0x01,
                        0x00,
                        0x07,
                        0x06,
                        0x00,
                        0x00,
                        0xFF,
                        0xFF,
                        0x02,
                        0x35,
                    ];
                    let _ = hci_send_l2cap(handle, cs.avdtp_sig_dcid, &rsp);
                }
                0x03 => {
                    // AVDTP_SET_CONFIGURATION -> Response Accept
                    let rsp = [tx_label | 0x02, 0x03];
                    let _ = hci_send_l2cap(handle, cs.avdtp_sig_dcid, &rsp);
                }
                0x06 => {
                    // AVDTP_OPEN from Speaker -> Response Accept
                    let rsp = [tx_label | 0x02, 0x06];
                    let _ = hci_send_l2cap(handle, cs.avdtp_sig_dcid, &rsp);
                    cs.avdtp_open_accepted = true;
                    cs.diag_stage = 0xE5;
                }
                0x07 => {
                    // AVDTP_START -> Response Accept
                    let rsp = [tx_label | 0x02, 0x07];
                    let _ = hci_send_l2cap(handle, cs.avdtp_sig_dcid, &rsp);
                    cs.a2dp_ready = true;
                    cs.avdtp_streaming = true;
                }
                0x0D => {
                    // AVDTP_DELAYREPORT -> Response Accept
                    let rsp = [tx_label | 0x02, 0x0D];
                    let _ = hci_send_l2cap(handle, cs.avdtp_sig_dcid, &rsp);
                }
                _ => {
                    let rsp = [tx_label | 0x02, sig_id];
                    let _ = hci_send_l2cap(handle, cs.avdtp_sig_dcid, &rsp);
                }
            }
        } else if msg_type == 0x02 {
            // Speaker replied with AVDTP Response Accept to our command!
            match sig_id {
                0x01 if body.len() >= 4 => {
                    // AVDTP_DISCOVER Response: collect all non-in-use Audio Sink SEPs (`(b0 & 0x02) == 0 && (b1 & 0xF8) == 0x08`)
                    cs.sink_seid_count = 0;
                    cs.sink_seid_idx = 0;
                    let mut pos = 2usize;
                    while pos + 2 <= body.len() {
                        let b0 = body[pos];
                        let b1 = body[pos + 1];
                        if (b0 & 0xFC) != 0 && (b0 & 0x02) == 0 && (b1 & 0xF8) == 0x08 {
                            if cs.sink_seid_count < cs.sink_seids.len() {
                                cs.sink_seids[cs.sink_seid_count] = b0 & 0xFC;
                                cs.sink_seid_count += 1;
                            }
                        }
                        pos += 2;
                    }
                    if cs.sink_seid_count > 0 {
                        cs.acp_seid_shifted = cs.sink_seids[0];
                    } else {
                        cs.acp_seid_shifted = body[2] & 0xFC;
                    }
                    // Step 2 of A2DP setup: Send AVDTP_GET_CAPABILITIES (0x02) for `acp_seid_shifted`
                    let get_cap: [u8; 3] = [0x20, 0x02, cs.acp_seid_shifted];
                    let _ = hci_send_l2cap(handle, cs.avdtp_sig_dcid, &get_cap);
                }
                0x02 | 0x0C => {
                    // Parse speaker's GET_CAPABILITIES response to find SBC (`Codec Type == 0x00`) and negotiate valid parameters
                    let mut is_sbc = false;
                    let mut sbc_cap0 = 0x22u8;
                    let mut sbc_cap1 = 0x15u8;
                    let mut sbc_min_bp = 0x02u8;
                    let mut sbc_max_bp = 0x35u8;

                    let mut pos = 2usize;
                    while pos + 2 <= body.len() {
                        let cat = body[pos];
                        let losc = body[pos + 1] as usize;
                        if pos + 2 + losc > body.len() {
                            break;
                        }
                        if cat == 0x07 && losc >= 6 {
                            let media_type = body[pos + 2];
                            let codec_type = body[pos + 3];
                            if media_type == 0x00 && codec_type == 0x00 {
                                is_sbc = true;
                                let c0 = body[pos + 4];
                                let c1 = body[pos + 5];
                                sbc_min_bp = body[pos + 6].max(2);
                                sbc_max_bp = body[pos + 7].min(53).max(sbc_min_bp);

                                let freq_bit = if (c0 & 0x20) != 0 {
                                    0x20
                                } else if (c0 & 0x10) != 0 {
                                    0x10
                                } else if (c0 & 0x40) != 0 {
                                    0x40
                                } else {
                                    0x80
                                };
                                // Prefer pure STEREO (0x02) when supported by Sink, or Joint Stereo (0x01)
                                let ch_bit = if (c0 & 0x02) != 0 {
                                    0x02
                                } else if (c0 & 0x01) != 0 {
                                    0x01
                                } else if (c0 & 0x04) != 0 {
                                    0x04
                                } else {
                                    0x08
                                };
                                sbc_cap0 = freq_bit | ch_bit;

                                let blk_bit = if (c1 & 0x10) != 0 {
                                    0x10
                                } else if (c1 & 0x20) != 0 {
                                    0x20
                                } else if (c1 & 0x40) != 0 {
                                    0x40
                                } else {
                                    0x80
                                };
                                let sub_bit = if (c1 & 0x04) != 0 { 0x04 } else { 0x08 };
                                let alloc_bit = if (c1 & 0x01) != 0 { 0x01 } else { 0x02 };
                                sbc_cap1 = blk_bit | sub_bit | alloc_bit;
                            }
                        }
                        pos += 2 + losc;
                    }

                    if !is_sbc && cs.sink_seid_idx + 1 < cs.sink_seid_count {
                        // This SEP was AAC/LDAC; query the next Audio Sink SEP for SBC!
                        cs.sink_seid_idx += 1;
                        cs.acp_seid_shifted = cs.sink_seids[cs.sink_seid_idx];
                        let get_cap: [u8; 3] = [0x20, 0x02, cs.acp_seid_shifted];
                        let _ = hci_send_l2cap(handle, cs.avdtp_sig_dcid, &get_cap);
                    } else {
                        cs.sbc_cap0 = sbc_cap0;
                        cs.sbc_cap1 = sbc_cap1;
                        cs.sbc_bitpool = 30u8.clamp(sbc_min_bp, sbc_max_bp);
                        let set_cfg: [u8; 14] = [
                            0x30,
                            0x03,
                            cs.acp_seid_shifted,
                            0x04, // INT SEID = 1
                            0x01,
                            0x00, // Service Category: Media Transport
                            0x07,
                            0x06,
                            0x00,
                            0x00,
                            sbc_cap0,
                            sbc_cap1,
                            sbc_min_bp,
                            sbc_max_bp,
                        ];
                        let _ = hci_send_l2cap(handle, cs.avdtp_sig_dcid, &set_cfg);
                    }
                }
                0x03 => {
                    // AVDTP_SET_CONFIGURATION accepted -> Send AVDTP_OPEN (0x06) with `cs.acp_seid_shifted`!
                    let open_cmd: [u8; 3] = [0x40, 0x06, cs.acp_seid_shifted];
                    let _ = hci_send_l2cap(handle, cs.avdtp_sig_dcid, &open_cmd);
                }
                0x06 => {
                    // AVDTP_OPEN accepted -> Open L2CAP Media Transport channel (PSM 0x0019, SCID 0x0041)
                    // This second L2CAP channel transitions the speaker's A2DP Sink to OPEN and triggers its chime!
                    cs.avdtp_open_accepted = true;
                    cs.diag_stage = 0xE5;
                    if !cs.media_l2cap_req_sent {
                        cs.media_l2cap_req_sent = true;
                        let conn_req: [u8; 8] = [0x02, 0x22, 0x04, 0x00, 0x19, 0x00, 0x41, 0x00];
                        let _ = hci_send_l2cap(handle, 0x0001, &conn_req);
                    }
                }
                0x07 => {
                    // AVDTP_START accepted -> Speaker is now in STREAMING state!
                    cs.avdtp_streaming = true;
                }
                _ => {}
            }
        }
    } else if cid == 0x0042 && body.len() >= 6 && cs.avrcp_dcid != 0 {
        // ─── AVRCP Remote Control Channel (local CID = 0x0042, PSM 0x0017) ───
        // Handles hardware Play/Pause, Next, Previous, and Volume buttons pressed on the Bluetooth speaker!
        let avctp_hdr = body[0];
        let msg_type = (avctp_hdr >> 1) & 0x01; // 0 = Command, 1 = Response
        if msg_type == 0 && body.len() >= 8 && body[1] == 0x11 && body[2] == 0x0E {
            let ctype = body[3] & 0x0F;
            let opcode = body[5];
            if ctype == 0x00 && opcode == 0x7C {
                // AV/C PASS THROUGH command (0x7C)
                let state_op = body[6];
                let pressed = (state_op & 0x80) == 0;
                let op_id = state_op & 0x7F;
                let mut rsp = [0u8; 8];
                rsp.copy_from_slice(&body[0..8]);
                rsp[0] = (avctp_hdr & 0xF0) | 0x02; // AVCTP Response bit
                rsp[3] = 0x09; // AV/C ACCEPTED (0x09)
                let _ = hci_send_l2cap(handle, cs.avrcp_dcid, &rsp);

                if pressed {
                    let act = match op_id {
                        0x44 | 0x46 | 0x45 => 1u32, // PLAY / PAUSE / STOP -> Toggle Play/Pause
                        0x4B => 2u32,               // FORWARD -> Next Track
                        0x4C => 3u32,               // BACKWARD -> Prev Track
                        0x41 => 4u32,               // VOLUME UP
                        0x42 => 5u32,               // VOLUME DOWN
                        _ => 0u32,
                    };
                    if act != 0 {
                        PENDING_AVRCP_ACTION.store(act, Ordering::Relaxed);
                    }
                }
            }
        }
    } else if cid == 0x0044 && body.len() >= 5 && cs.sdp_dcid != 0 {
        // ─── SDP Server Stub (local CID = 0x0044): reply cleanly so speaker's SDP query finishes immediately ───
        let pdu_id = body[0];
        let tid_hi = body[1];
        let tid_lo = body[2];
        match pdu_id {
            0x02 => {
                // SDP_ServiceSearchRequest -> SDP_ServiceSearchResponse (0x03, Total=0, Current=0, Cont=0)
                let rsp: [u8; 10] = [0x03, tid_hi, tid_lo, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00];
                let _ = hci_send_l2cap(handle, cs.sdp_dcid, &rsp);
            }
            0x04 | 0x06 => {
                // SDP_ServiceAttributeRequest / ServiceSearchAttributeRequest -> Empty AttributeList `0x35 0x00`
                let rsp_id = pdu_id + 1;
                let rsp: [u8; 10] = [rsp_id, tid_hi, tid_lo, 0x00, 0x05, 0x00, 0x02, 0x35, 0x00, 0x00];
                let _ = hci_send_l2cap(handle, cs.sdp_dcid, &rsp);
            }
            _ => {}
        }
    } else if cid == 0x0004 && !body.is_empty() {
        // ─── BLE ATT Fixed Channel (CID = 0x0004, e.g. Sony SRS-XB100 GATT) ───
        let att_op = body[0];
        match att_op {
            0x02 => {
                let _ = hci_send_l2cap(handle, 0x0004, &[0x03, 0xF4, 0x01]);
                cs.ble_gatt_ready = true;
            }
            0x03 => {
                let _ = hci_send_l2cap(handle, 0x0004, &[0x0A, 0x03, 0x00]);
                cs.ble_gatt_ready = true;
            }
            0x08 | 0x10 if body.len() >= 3 => {
                let err_rsp = [0x01, att_op, body[1], body[2], 0x0A];
                let _ = hci_send_l2cap(handle, 0x0004, &err_rsp);
                cs.ble_gatt_ready = true;
            }
            _ => {
                cs.ble_gatt_ready = true;
            }
        }
    } else if cid == 0x0005 && body.len() >= 2 {
        // ─── BLE L2CAP Signaling Channel (CID = 0x0005) ───
        if body[0] == 0x12 && body.len() >= 4 {
            let rsp = [0x13, body[1], 0x02, 0x00, 0x00, 0x00];
            let _ = hci_send_l2cap(handle, 0x0005, &rsp);
            cs.ble_gatt_ready = true;
        }
    } else if cid == 0x0006 && !body.is_empty() {
        // ─── BLE Security Manager Protocol (SMP Fixed Channel CID = 0x0006) ───
        if body[0] == 0x0B {
            let smp_req: [u8; 7] = [0x01, 0x03, 0x00, 0x2D, 0x10, 0x0F, 0x0F];
            let _ = hci_send_l2cap(handle, 0x0006, &smp_req);
        }
        cs.ble_gatt_ready = true;
    }
}

/// Initiate an active Classic (`0x0405` ACL + SSP + A2DP `AVDTP PSM 0x0019`) or BLE (`0x200D` GATT)
/// connection to discovered device `idx`.
///
/// For dual-mode audio speakers like **Sony SRS-XB100** (which broadcast on BLE `D7:E6:E8:6C:03:8C`
/// continuously, and open Classic `BR/EDR` Inquiry/Page Scan when their Pairing LED is double-flashing),
/// we **always** resolve and connect their **Classic `BR/EDR` A2DP transport first**, because the speaker
/// only exits Pairing Mode and plays its connected chime when Classic `BR/EDR` A2DP (`PSM 0x0019`) opens!
pub fn connect_device(idx: usize) -> bool {
    let st = status();
    if idx >= st.device_count || !st.devices[idx].valid {
        return false;
    }

    if bsp::is_qemu() {
        STATE.with(|s| {
            for d in s.devices.iter_mut() {
                d.connected = false;
            }
            s.devices[idx].connected = true;
            s.connected_index = Some(idx);
            s.acl_handle = 0x0040;
            s.a2dp_ready = true;
            s.last_conn_err = 0;
        });
        return true;
    }

    let mut dev = st.devices[idx];

    BT_CMD_ACTIVE.store(true, Ordering::Relaxed);

    let mut connected_handle: Option<u16> = None;
    let mut a2dp_ready = false;
    let mut ble_gatt_ready = false;
    let mut connected_classic_bd: Option<[u8; 6]> = None;
    let mut saved_sig_dcid = 0u16;
    let mut saved_media_dcid = 0u16;
    let mut saved_acp_seid = 0x04u8;
    let mut saved_sbc_cap0 = 0x22u8;
    let mut saved_sbc_cap1 = 0x15u8;
    let mut saved_sbc_bp = 30u8;
    let mut saved_streaming = false;
    let mut last_err = 0xFFu8;

    unsafe {
        let mut resp = [0u8; 32];
        let mut pkt = [0u8; 260];

        // 0. Synchronously stop BLE Scan (0x200C), BLE Advertising (0x200A), AND Classic Inquiry/Page Scan (0x0C1A = 0x00 `noscan`)!
        let _ = hci_send_cmd(0x200C, &[0x00, 0x00], &mut resp, 60);
        let _ = hci_send_cmd(0x200A, &[0x00], &mut resp, 60);
        let _ = hci_send_cmd(0x0C1A, &[0x00], &mut resp, 60);
        let _ = hci_send_cmd(
            0x0C01,
            &[0xFF, 0xFF, 0xFB, 0xFF, 0x07, 0xF8, 0xBF, 0x3D],
            &mut resp,
            60,
        );
        let _ = hci_send_cmd(
            0x2001,
            &[0xFF, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            &mut resp,
            60,
        );
        drain_uart0_rx(4096);

        // ─── Phase A: Classic BR/EDR + A2DP (`0x0405` -> `SSP` -> `AVDTP PSM 0x0019`) ───
        // Always attempt Classic BR/EDR first for any Classic device (`!dev.is_ble`) OR any Audio Speaker (`dev.is_audio`,
        // e.g. Sony SRS-XB100), so the speaker exits Pairing Mode and plays its connected chime!
        if !dev.is_ble || dev.is_audio {
            // Collect up to 4 candidate Classic BR/EDR targets `(bd_disp, page_scan_rep, clock_offset)`
            let mut candidates: [([u8; 6], u8, u16); 4] = [([0; 6], 1, 0); 4];
            let mut cand_count = 0usize;

            fn push_cand(
                candidates: &mut [([u8; 6], u8, u16); 4],
                cand_count: &mut usize,
                bd: [u8; 6],
                rep: u8,
                clk: u16,
            ) {
                if bd == [0; 6] || *cand_count >= 4 {
                    return;
                }
                for k in 0..*cand_count {
                    if candidates[k].0 == bd {
                        return;
                    }
                }
                candidates[*cand_count] = (bd, rep.min(2), clk);
                *cand_count += 1;
            }

            if !dev.is_ble {
                push_cand(
                    &mut candidates,
                    &mut cand_count,
                    dev.bd_addr,
                    dev.page_scan_rep,
                    dev.clock_offset,
                );
            } else {
                // 1) If this dual-mode speaker already has a linked Classic BD_ADDR (`alt_bd_addr`), try it first
                if dev.has_alt_bd {
                    push_cand(
                        &mut candidates,
                        &mut cand_count,
                        dev.alt_bd_addr,
                        dev.page_scan_rep,
                        dev.clock_offset,
                    );
                }

                // 2) Check if `st.devices` has a Classic BR/EDR entry with the EXACT same real name (never match Office TV!)
                for i in 0..st.device_count {
                    let d = &st.devices[i];
                    if d.valid
                        && !d.is_ble
                        && d.has_real_name
                        && dev.has_real_name
                        && d.name_str() == dev.name_str()
                    {
                        push_cand(
                            &mut candidates,
                            &mut cand_count,
                            d.bd_addr,
                            d.page_scan_rep,
                            d.clock_offset,
                        );
                    }
                }

                // 3) If the user put `SRS-XB100` into Pairing Mode (double-flashing blue LED) and pressed `3.连接`
                //    on its BLE entry without re-scanning, run a fast GIAC (3.84s) + LIAC (1.28s) Classic Inquiry (`0x0401`)
                //    right now to catch the speaker's live Classic BR/EDR `BD_ADDR` and `Clock_Offset`!
                if cand_count == 0 {
                    let _ = hci_send_cmd(0x0C45, &[0x02], &mut resp, 60);
                    let inq_passes: [[u8; 5]; 2] = [
                        [0x33, 0x8B, 0x9E, 0x03, 0x00], // GIAC (3.84s)
                        [0x00, 0x8B, 0x9E, 0x01, 0x00], // LIAC (1.28s)
                    ];
                    for (pass_idx, inq_params) in inq_passes.iter().enumerate() {
                        if cand_count > 0 {
                            break;
                        }
                        if hci_write_raw_cmd(0x0401, inq_params) {
                            let freq = timer::frequency();
                            let wait_tenths = if pass_idx == 0 { 40 } else { 15 };
                            let inq_deadline = timer::counter() + freq * wait_tenths / 10;
                            while timer::counter() < inq_deadline {
                                tryboot::watchdog_feed(15);
                                let Some((evt_code, plen)) = hci_read_event(&mut pkt, 150) else {
                                    continue;
                                };
                                match evt_code {
                                    0x01 => break,
                                    0x2F if plen >= 15 => {
                                        let bd_disp = [
                                            pkt[6], pkt[5], pkt[4], pkt[3], pkt[2], pkt[1],
                                        ];
                                        let page_rep = pkt[7];
                                        let cod = (pkt[11] as u32) << 16
                                            | (pkt[10] as u32) << 8
                                            | (pkt[9] as u32);
                                        let clk_off = u16::from_le_bytes([pkt[12], pkt[13]]);
                                        let ad = parse_ad_payload(&pkt[15..plen]);
                                        let name_matches = match ad.name {
                                            Some(n) if !n.is_empty() && dev.has_real_name => {
                                                n == dev.name_str().as_bytes()
                                            }
                                            None => ad.is_audio || is_audio_speaker_cod(cod),
                                            _ => false,
                                        };
                                        if name_matches {
                                            push_cand(
                                                &mut candidates,
                                                &mut cand_count,
                                                bd_disp,
                                                page_rep,
                                                clk_off,
                                            );
                                            break;
                                        }
                                    }
                                    0x22 | 0x02 if plen >= 15 => {
                                        let num = (pkt[0] as usize).max(1);
                                        for r in 0..num {
                                            if 1 + 6 * (r + 1) > plen {
                                                break;
                                            }
                                            let base = 1 + 6 * r;
                                            let bd_disp = [
                                                pkt[base + 5],
                                                pkt[base + 4],
                                                pkt[base + 3],
                                                pkt[base + 2],
                                                pkt[base + 1],
                                                pkt[base],
                                            ];
                                            let page_rep = *pkt.get(1 + 6 * num + r).unwrap_or(&1);
                                            let cod_off = if evt_code == 0x22 {
                                                1 + 8 * num + 3 * r
                                            } else {
                                                1 + 9 * num + 3 * r
                                            };
                                            let cod = if cod_off + 3 <= plen {
                                                (pkt[cod_off + 2] as u32) << 16
                                                    | (pkt[cod_off + 1] as u32) << 8
                                                    | (pkt[cod_off] as u32)
                                            } else {
                                                0
                                            };
                                            let clk_idx = if evt_code == 0x22 {
                                                1 + 11 * num + 2 * r
                                            } else {
                                                1 + 12 * num + 2 * r
                                            };
                                            let clk_off = if clk_idx + 2 <= plen {
                                                u16::from_le_bytes([pkt[clk_idx], pkt[clk_idx + 1]])
                                            } else {
                                                0
                                            };
                                            let is_other_named = st.devices[..st.device_count]
                                                .iter()
                                                .any(|d| {
                                                    d.valid
                                                        && d.bd_addr == bd_disp
                                                        && d.has_real_name
                                                        && d.name_str() != dev.name_str()
                                                });
                                            if !is_other_named && is_audio_speaker_cod(cod) {
                                                push_cand(
                                                    &mut candidates,
                                                    &mut cand_count,
                                                    bd_disp,
                                                    page_rep,
                                                    clk_off,
                                                );
                                            }
                                        }
                                    }
                                    _ => {}
                                }
                            }
                            let _ = hci_send_cmd(0x0402, &[], &mut resp, 60);
                            drain_uart0_rx(2048);
                        }
                    }
                }
            }

            if cand_count == 0 && dev.is_audio {
                // Speaker did not respond to Classic BR/EDR Inquiry (GIAC or LIAC):
                // its 5-minute pairing mode timed out or it is already connected to a phone.
                last_err = 0x04;
            } else if cand_count > 0 {
                if dev.is_ble {
                    let (first_bd, first_rep, first_clk) = candidates[0];
                    STATE.with(|s| {
                        if idx < s.device_count && s.devices[idx].bd_addr != first_bd {
                            s.devices[idx].alt_bd_addr = first_bd;
                            s.devices[idx].has_alt_bd = true;
                            s.devices[idx].page_scan_rep = first_rep;
                            s.devices[idx].clock_offset = first_clk;
                        }
                    });
                }
                // Enable Simple Pairing Mode (0x0C56 = 1), Smartphone + Audio Source CoD (`0x7A020C`),
                // `noscan` (`0x0C1A = 0x00`), Page Timeout = 5.12s (`0x2000`), Role Switch allowed (`0x080F = 0x000F`)
                let _ = hci_send_cmd(0x0C56, &[0x01], &mut resp, 60);
                let _ = hci_send_cmd(0x0C24, &[0x0C, 0x02, 0x7A], &mut resp, 60);
                let _ = hci_send_cmd(0x0C1A, &[0x00], &mut resp, 60);
                let _ = hci_send_cmd(0x0C20, &[0x00], &mut resp, 60);
                let _ = hci_send_cmd(0x0C18, &[0x00, 0x20], &mut resp, 60);
                let _ = hci_send_cmd(0x080F, &[0x0F, 0x00], &mut resp, 60);
                drain_uart0_rx(1024);

                for c_idx in 0..cand_count {
                    if connected_handle.is_some() && (a2dp_ready || !dev.is_audio) {
                        break;
                    }
                    let (cand_bd, cand_rep, cand_clk) = candidates[c_idx];
                    let cand_le = [
                        cand_bd[5], cand_bd[4], cand_bd[3], cand_bd[2], cand_bd[1], cand_bd[0],
                    ];
                    let clk_hi = if cand_clk != 0 {
                        (((cand_clk >> 8) & 0x7F) as u8) | 0x80
                    } else {
                        0x00
                    };
                    let conn_params: [u8; 13] = [
                        cand_le[0],
                        cand_le[1],
                        cand_le[2],
                        cand_le[3],
                        cand_le[4],
                        cand_le[5],
                        0x18,
                        0xCC,
                        cand_rep.max(1).min(2),
                        0x00,
                        (cand_clk & 0xFF) as u8,
                        clk_hi,
                        0x01,
                    ];

                    let mut cs = ClassicConnState::new();
                    let mut reasm = AclReassembler::new();
                    let mut auth_requested = false;
                    let mut retried_fresh_auth = false;
                    let mut info_req_sent = false;
                    let mut l2cap_requested = false;
                    let mut acl_conn_tick = 0u64;
                    let mut enc_ready_tick = 0u64;
                    let mut discover_tick = 0u64;
                    let mut discover_retried = false;

                    if hci_write_raw_cmd(0x0405, &conn_params) {
                        let freq = timer::frequency();
                        let conn_deadline = timer::counter() + freq * 95 / 10;

                        while timer::counter() < conn_deadline {
                            tryboot::watchdog_feed(15);
                            if cs.a2dp_ready {
                                a2dp_ready = true;
                                // Pump L2CAP for an extra 300ms after `a2dp_ready` so final ConfigRsp / AVRCP / SDP
                                // frames complete and the speaker plays its connected chime!
                                let settle_end = timer::counter() + (freq * 3) / 10;
                                while timer::counter() < settle_end {
                                    let Some((pt, ev, pl)) = hci_read_packet(&mut pkt, 50) else {
                                        continue;
                                    };
                                    if pt == 0x02 {
                                        if let Some(h) = connected_handle {
                                            reasm.feed_and_dispatch(h, ev, &pkt[..pl], &mut cs);
                                        }
                                    }
                                }

                                // Transition A2DP stream from OPEN to STREAMING (`AVDTP_START = 0x07`)
                                if let Some(h) = connected_handle {
                                    if !cs.avdtp_streaming && cs.avdtp_sig_dcid != 0 {
                                        let start_cmd: [u8; 3] = [0x50, 0x07, cs.acp_seid_shifted];
                                        let _ = hci_send_l2cap(h, cs.avdtp_sig_dcid, &start_cmd);
                                        let start_end = timer::counter() + (freq * 15) / 100;
                                        while timer::counter() < start_end {
                                            let Some((pt, ev, pl)) = hci_read_packet(&mut pkt, 35)
                                            else {
                                                continue;
                                            };
                                            if pt == 0x02 {
                                                reasm.feed_and_dispatch(
                                                    h,
                                                    ev,
                                                    &pkt[..pl],
                                                    &mut cs,
                                                );
                                                if cs.avdtp_streaming {
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                }
                                saved_sig_dcid = cs.avdtp_sig_dcid;
                                saved_media_dcid = cs.avdtp_media_dcid;
                                saved_acp_seid = cs.acp_seid_shifted;
                                saved_sbc_cap0 = cs.sbc_cap0;
                                saved_sbc_cap1 = cs.sbc_cap1;
                                saved_sbc_bp = cs.sbc_bitpool;
                                saved_streaming = cs.avdtp_streaming;
                                break;
                            }

                            if let Some(h) = connected_handle {
                                let now = timer::counter();
                                // 1) If 800ms elapsed after ACL Connection Complete without `0x0B`/`0x23` triggering `0x0411`, request auth now!
                                if !auth_requested && acl_conn_tick != 0 && now > acl_conn_tick + (freq * 8) / 10 {
                                    auth_requested = true;
                                    let h_bytes = h.to_le_bytes();
                                    let _ = hci_write_raw_cmd(0x0411, &h_bytes);
                                }
                                // 2) Once Encryption (or L2CAP Info Response) is ready, open AVDTP Signaling (PSM 0x0019, SCID 0x0040)!
                                if !l2cap_requested
                                    && cs.avdtp_sig_dcid == 0
                                    && ((cs.info_rsp_received && enc_ready_tick != 0)
                                        || (enc_ready_tick != 0 && now > enc_ready_tick + freq / 4)
                                        || (acl_conn_tick != 0 && now > acl_conn_tick + freq * 38 / 10))
                                {
                                    l2cap_requested = true;
                                    let conn_req: [u8; 8] =
                                        [0x02, 0x02, 0x04, 0x00, 0x19, 0x00, 0x40, 0x00];
                                    let _ = hci_send_l2cap(h, 0x0001, &conn_req);
                                }
                                // 3) If AVDTP_DISCOVER was sent and 900ms elapsed without response, retry AVDTP_DISCOVER once!
                                if cs.avdtp_discover_sent
                                    && !discover_retried
                                    && cs.sink_seid_count == 0
                                    && !cs.avdtp_open_accepted
                                    && discover_tick != 0
                                    && now > discover_tick + (freq * 9) / 10
                                {
                                    discover_retried = true;
                                    let _ = hci_send_l2cap(h, cs.avdtp_sig_dcid, &[0x10, 0x01]);
                                }
                            }

                            let Some((pkt_type, evt_code, plen)) = hci_read_packet(&mut pkt, 120)
                            else {
                                continue;
                            };

                            if pkt_type == 0x02 {
                                if let Some(h) = connected_handle {
                                    let prev_disc = cs.avdtp_discover_sent;
                                    reasm.feed_and_dispatch(h, evt_code, &pkt[..plen], &mut cs);
                                    if !prev_disc && cs.avdtp_discover_sent {
                                        discover_tick = timer::counter();
                                    }
                                    if cs.a2dp_ready {
                                        a2dp_ready = true;
                                    }
                                }
                                continue;
                            }

                            match evt_code {
                                0x04 if plen >= 10 => {
                                    let mut acc = [0u8; 7];
                                    acc[0..6].copy_from_slice(&pkt[0..6]);
                                    acc[6] = 0x01;
                                    let _ = hci_write_raw_cmd(0x0409, &acc);
                                }
                                0x03 if plen >= 11 => {
                                    let status_byte = pkt[0];
                                    let handle = u16::from_le_bytes([pkt[1], pkt[2]]) & 0x0FFF;
                                    last_err = status_byte;
                                    if status_byte == 0x00 {
                                        connected_handle = Some(handle);
                                        connected_classic_bd = Some(cand_bd);
                                        acl_conn_tick = timer::counter();
                                        cs.diag_stage = 0xE1;
                                        // Step 1 after ACL Connect: Read Remote Supported Features (0x041B) ONLY!
                                        // (Wait for `0x0B` + `0x041C` Extended Features Page 1 before sending `0x0411`!)
                                        let h_bytes = handle.to_le_bytes();
                                        let _ = hci_write_raw_cmd(0x041B, &h_bytes);
                                    } else {
                                        break;
                                    }
                                }
                                0x0B if plen >= 11 => {
                                    // Read Remote Supported Features Complete
                                    if let Some(h) = connected_handle {
                                        let ext_supported = (pkt[10] & 0x80) != 0;
                                        if ext_supported {
                                            // Read Remote Extended Features Page 1 (0x041C) so controller sees peer's SSP Host bit!
                                            let cmd: [u8; 3] = [(h & 0xFF) as u8, (h >> 8) as u8, 0x01];
                                            let _ = hci_write_raw_cmd(0x041C, &cmd);
                                        } else if !auth_requested {
                                            auth_requested = true;
                                            let h_bytes = h.to_le_bytes();
                                            let _ = hci_write_raw_cmd(0x0411, &h_bytes);
                                        }
                                    }
                                }
                                0x23 if plen >= 3 => {
                                    // Read Remote Extended Features Complete (Page 1) -> Now initiate SSP Authentication (0x0411)!
                                    if let Some(h) = connected_handle {
                                        if !auth_requested {
                                            auth_requested = true;
                                            let h_bytes = h.to_le_bytes();
                                            let _ = hci_write_raw_cmd(0x0411, &h_bytes);
                                        }
                                    }
                                }
                                0x0F if plen >= 4 => {
                                    let st_code = pkt[0];
                                    let op = u16::from_le_bytes([pkt[2], pkt[3]]);
                                    if op == 0x0405 && st_code == 0x0B {
                                        let prev_h = if st.acl_handle != 0 {
                                            st.acl_handle
                                        } else {
                                            0x000B
                                        };
                                        connected_handle = Some(prev_h);
                                        connected_classic_bd = Some(cand_bd);
                                        acl_conn_tick = timer::counter();
                                        enc_ready_tick = timer::counter();
                                        cs.diag_stage = 0xE3;
                                        last_err = 0x00;
                                        if !info_req_sent {
                                            info_req_sent = true;
                                            let info_req: [u8; 6] = [0x0A, 0x01, 0x02, 0x00, 0x02, 0x00];
                                            let _ = hci_send_l2cap(prev_h, 0x0001, &info_req);
                                        }
                                    } else if op == 0x0405 && st_code != 0x00 {
                                        last_err = st_code;
                                        break;
                                    }
                                }
                                0x16 if plen >= 6 => {
                                    let mut pin_rep = [0u8; 23];
                                    pin_rep[0..6].copy_from_slice(&pkt[0..6]);
                                    pin_rep[6] = 4;
                                    pin_rep[7..11].copy_from_slice(b"0000");
                                    let _ = hci_write_raw_cmd(0x040D, &pin_rep);
                                }
                                0x17 if plen >= 6 => {
                                    if dev.has_link_key {
                                        let mut lk_rep = [0u8; 22];
                                        lk_rep[0..6].copy_from_slice(&pkt[0..6]);
                                        lk_rep[6..22].copy_from_slice(&dev.link_key);
                                        let _ = hci_write_raw_cmd(0x040B, &lk_rep);
                                    } else {
                                        let _ = hci_write_raw_cmd(0x040C, &pkt[0..6]);
                                    }
                                }
                                0x18 if plen >= 22 => {
                                    let mut lk = [0u8; 16];
                                    lk.copy_from_slice(&pkt[6..22]);
                                    dev.has_link_key = true;
                                    dev.link_key = lk;
                                    STATE.with(|s| {
                                        if idx < s.device_count {
                                            s.devices[idx].has_link_key = true;
                                            s.devices[idx].link_key = lk;
                                        }
                                    });
                                }
                                0x31 if plen >= 6 => {
                                    let mut io_rep = [0u8; 9];
                                    io_rep[0..6].copy_from_slice(&pkt[0..6]);
                                    io_rep[6] = 0x03; // NoInputNoOutput (Just Works)
                                    io_rep[7] = 0x00;
                                    io_rep[8] = 0x02; // MITM Protection Not Required - General Bonding
                                    let _ = hci_write_raw_cmd(0x042B, &io_rep);
                                }
                                0x33 if plen >= 6 => {
                                    let _ = hci_write_raw_cmd(0x042C, &pkt[0..6]);
                                }
                                0x06 if plen >= 3 => {
                                    if let Some(h) = connected_handle {
                                        if pkt[0] == 0x00 {
                                            cs.diag_stage = 0xE2;
                                            let enc_cmd: [u8; 3] =
                                                [(h & 0xFF) as u8, (h >> 8) as u8, 0x01];
                                            let _ = hci_write_raw_cmd(0x0413, &enc_cmd);
                                        } else if dev.has_link_key && !retried_fresh_auth {
                                            // Stale Link Key rejected by speaker: clear it and re-run fresh SSP pairing!
                                            retried_fresh_auth = true;
                                            dev.has_link_key = false;
                                            STATE.with(|s| {
                                                if idx < s.device_count {
                                                    s.devices[idx].has_link_key = false;
                                                }
                                            });
                                            let h_bytes = h.to_le_bytes();
                                            let _ = hci_write_raw_cmd(0x0411, &h_bytes);
                                        } else {
                                            last_err = pkt[0];
                                            break;
                                        }
                                    }
                                }
                                0x08 if plen >= 4 => {
                                    if let Some(h) = connected_handle {
                                        if pkt[0] == 0x00 {
                                            cs.diag_stage = 0xE3;
                                            enc_ready_tick = timer::counter();
                                            if !info_req_sent {
                                                info_req_sent = true;
                                                let info_req: [u8; 6] =
                                                    [0x0A, 0x01, 0x02, 0x00, 0x02, 0x00];
                                                let _ = hci_send_l2cap(h, 0x0001, &info_req);
                                            }
                                        } else {
                                            last_err = pkt[0];
                                            break;
                                        }
                                    }
                                }
                                0x05 if plen >= 4 => {
                                    connected_handle = None;
                                    last_err = pkt[3];
                                    break;
                                }
                                _ => {}
                            }
                        }

                        // For Audio Speakers (`dev.is_audio`), require `a2dp_ready == true` so we NEVER falsely
                        // display "已连接" when the speaker hasn't completed A2DP setup and played its connected chime!
                        if dev.is_audio && !a2dp_ready {
                            if let Some(h) = connected_handle.take() {
                                let disc: [u8; 3] = [(h & 0xFF) as u8, (h >> 8) as u8, 0x13];
                                let _ = hci_write_raw_cmd(0x0406, &disc);
                                timer::delay_ms(40);
                                drain_uart0_rx(1024);
                            }
                            if last_err == 0x00 || last_err == 0xFF {
                                last_err = cs.diag_stage;
                            }
                        }

                        if connected_handle.is_none() {
                            // Cancel pending Classic Create Connection (0x0408) so BCM43430B0 never returns 0x0D!
                            let _ = hci_write_raw_cmd(0x0408, &cand_le);
                            timer::delay_ms(25);
                            drain_uart0_rx(1024);
                        }
                    }
                }
            }
        }

        // ─── Phase B: BLE Connection (`0x200D` GATT + SMP Pairing `CID 0x0006`) for non-audio BLE peripherals ───
        // Note: Audio speakers (`dev.is_audio`) require Classic BR/EDR A2DP for audio playback and pairing chime;
        // never fall back to a silent BLE GATT link on a speaker!
        if connected_handle.is_none() && dev.is_ble && !dev.is_audio {
            let bd_disp = dev.bd_addr;
            let bd_le = [
                bd_disp[5], bd_disp[4], bd_disp[3], bd_disp[2], bd_disp[1], bd_disp[0],
            ];
            let mut cs = ClassicConnState::new();
            let mut reasm = AclReassembler::new();

            let addr_types = [dev.le_addr_type & 0x01, (dev.le_addr_type & 0x01) ^ 0x01];
            for &peer_addr_type in &addr_types {
                if connected_handle.is_some() {
                    break;
                }
                let le_conn_params: [u8; 25] = [
                    0x04,
                    0x00,
                    0x04,
                    0x00,
                    0x00,
                    peer_addr_type,
                    bd_le[0],
                    bd_le[1],
                    bd_le[2],
                    bd_le[3],
                    bd_le[4],
                    bd_le[5],
                    0x00,
                    0x0F,
                    0x00,
                    0x0F,
                    0x00,
                    0x00,
                    0x00,
                    0x80,
                    0x0C,
                    0x01,
                    0x00,
                    0x01,
                    0x00,
                ];

                if hci_write_raw_cmd(0x200D, &le_conn_params) {
                    let freq = timer::frequency();
                    let le_deadline = timer::counter() + freq * 36 / 10;
                    let mut att_sent = false;

                    while timer::counter() < le_deadline {
                        tryboot::watchdog_feed(15);
                        if connected_handle.is_some() && ble_gatt_ready {
                            break;
                        }
                        let Some((pkt_type, evt_code, plen)) = hci_read_packet(&mut pkt, 150)
                        else {
                            if let Some(h) = connected_handle {
                                if !att_sent {
                                    att_sent = true;
                                    let _ = hci_send_l2cap(h, 0x0004, &[0x02, 0x05, 0x02]);
                                    let _ = hci_send_l2cap(
                                        h,
                                        0x0006,
                                        &[0x01, 0x03, 0x00, 0x2D, 0x10, 0x0F, 0x0F],
                                    );
                                } else {
                                    ble_gatt_ready = true;
                                    break;
                                }
                            }
                            continue;
                        };

                        if pkt_type == 0x02 {
                            if let Some(h) = connected_handle {
                                reasm.feed_and_dispatch(h, evt_code, &pkt[..plen], &mut cs);
                                if cs.ble_gatt_ready {
                                    ble_gatt_ready = true;
                                    break;
                                }
                            }
                            continue;
                        }

                        match evt_code {
                            0x3E if plen >= 6 => {
                                let subevt = pkt[0];
                                if subevt == 0x01 || subevt == 0x0A {
                                    let st_byte = pkt[1];
                                    let handle = u16::from_le_bytes([pkt[2], pkt[3]]) & 0x0FFF;
                                    last_err = st_byte;
                                    if st_byte == 0x00 {
                                        connected_handle = Some(handle);
                                        let h_bytes = handle.to_le_bytes();
                                        let _ = hci_write_raw_cmd(0x2016, &h_bytes);
                                    } else {
                                        break;
                                    }
                                } else if subevt == 0x04 {
                                    if pkt[1] == 0x00 {
                                        if let Some(h) = connected_handle {
                                            if !att_sent {
                                                att_sent = true;
                                                let _ =
                                                    hci_send_l2cap(h, 0x0004, &[0x02, 0x05, 0x02]);
                                                let _ = hci_send_l2cap(
                                                    h,
                                                    0x0006,
                                                    &[0x01, 0x03, 0x00, 0x2D, 0x10, 0x0F, 0x0F],
                                                );
                                            }
                                        }
                                    } else {
                                        connected_handle = None;
                                        last_err = pkt[1];
                                        break;
                                    }
                                }
                            }
                            0x0F if plen >= 4 => {
                                let st_code = pkt[0];
                                let op = u16::from_le_bytes([pkt[2], pkt[3]]);
                                if op == 0x200D && st_code == 0x0B {
                                    let prev_h = if st.acl_handle != 0 {
                                        st.acl_handle
                                    } else {
                                        0x0040
                                    };
                                    connected_handle = Some(prev_h);
                                    ble_gatt_ready = true;
                                    last_err = 0x00;
                                    break;
                                } else if op == 0x200D && st_code != 0x00 {
                                    last_err = st_code;
                                    break;
                                }
                            }
                            0x05 if plen >= 4 => {
                                connected_handle = None;
                                last_err = pkt[3];
                                break;
                            }
                            _ => {}
                        }
                    }

                    if connected_handle.is_none() {
                        let _ = hci_send_cmd(0x200E, &[], &mut resp, 60);
                        drain_uart0_rx(512);
                    }
                }
            }
        }

        // Only restore Slave Inquiry+Page Scan and BLE Advertising if no active connection is open
        if connected_handle.is_none() {
            let _ = hci_send_cmd(0x0C1A, &[0x03], &mut resp, 60);
            let _ = hci_send_cmd(0x200A, &[0x01], &mut resp, 60);
            drain_uart0_rx(512);
        }
    }

    if let Some(handle) = connected_handle {
        STATE.with(|s| {
            for d in s.devices.iter_mut() {
                d.connected = false;
            }
            if idx < s.device_count {
                if let Some(cbd) = connected_classic_bd {
                    if s.devices[idx].bd_addr != cbd {
                        s.devices[idx].alt_bd_addr = s.devices[idx].bd_addr;
                        s.devices[idx].has_alt_bd = true;
                        s.devices[idx].bd_addr = cbd;
                    }
                    s.devices[idx].is_ble = false;
                }
                s.devices[idx].connected = true;
                s.connected_index = Some(idx);
                s.acl_handle = handle;
                s.a2dp_ready = a2dp_ready;
                s.avdtp_sig_dcid = saved_sig_dcid;
                s.avdtp_media_dcid = saved_media_dcid;
                s.acp_seid_shifted = saved_acp_seid;
                s.sbc_cap0 = saved_sbc_cap0;
                s.sbc_cap1 = saved_sbc_cap1;
                s.sbc_bitpool = saved_sbc_bp;
                s.avdtp_streaming = saved_streaming;
                s.last_conn_err = 0;
            }
        });
        unsafe {
            A2DP_NEXT_TICK = 0;
            RUNTIME_CS = ClassicConnState::new();
            RUNTIME_CS.avdtp_sig_dcid = saved_sig_dcid;
            RUNTIME_CS.avdtp_media_dcid = saved_media_dcid;
            RUNTIME_CS.acp_seid_shifted = saved_acp_seid;
            RUNTIME_CS.sbc_cap0 = saved_sbc_cap0;
            RUNTIME_CS.sbc_cap1 = saved_sbc_cap1;
            RUNTIME_CS.sbc_bitpool = saved_sbc_bp;
            RUNTIME_CS.avdtp_streaming = saved_streaming;
            RUNTIME_CS.a2dp_ready = a2dp_ready;
            RUNTIME_REASM = AclReassembler::new();
        }
        BT_CMD_ACTIVE.store(false, Ordering::Relaxed);
        true
    } else {
        STATE.with(|s| {
            s.last_conn_err = last_err;
        });
        BT_CMD_ACTIVE.store(false, Ordering::Relaxed);
        false
    }
}

// ─── M10.2 / M11.1: Real-Time Bluetooth A2DP 8-Subband SBC Audio Encoder & Media Streamer ───

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum AvrcpAction {
    PlayPause,
    NextTrack,
    PrevTrack,
    VolumeUp,
    VolumeDown,
}

static PENDING_AVRCP_ACTION: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);

/// Poll any pending hardware button press sent by the Bluetooth speaker over `AVRCP (PSM 0x0017)`.
pub fn take_avrcp_action() -> Option<AvrcpAction> {
    let raw = PENDING_AVRCP_ACTION.load(Ordering::Relaxed);
    if raw == 0 {
        return None;
    }
    PENDING_AVRCP_ACTION.store(0, Ordering::Relaxed);
    match raw {
        1 => Some(AvrcpAction::PlayPause),
        2 => Some(AvrcpAction::NextTrack),
        3 => Some(AvrcpAction::PrevTrack),
        4 => Some(AvrcpAction::VolumeUp),
        5 => Some(AvrcpAction::VolumeDown),
        _ => None,
    }
}

static mut A2DP_SEQ: u16 = 1;
static mut A2DP_TS: u32 = 0;
static mut A2DP_SAMPLE_CURSOR: u32 = 0;
static mut A2DP_NEXT_TICK: u64 = 0;
static mut RUNTIME_CS: ClassicConnState = ClassicConnState::new();
static mut RUNTIME_REASM: AclReassembler = AclReassembler::new();

// ─── Pre-Encoded A2DP Media Packet Ring Buffer (`16` packets = `232 ms` audio) ───
const SBC_PKT_RING_CAP: usize = 16;
const SBC_PKT_RING_MASK: usize = SBC_PKT_RING_CAP - 1;
static mut SBC_PKT_DATA: [[u8; 440]; SBC_PKT_RING_CAP] = [[0u8; 440]; SBC_PKT_RING_CAP];
static mut SBC_PKT_LEN: [u16; SBC_PKT_RING_CAP] = [0u16; SBC_PKT_RING_CAP];
static SBC_PKT_READ: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);
static SBC_PKT_WRITE: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);

#[inline]
fn clear_sbc_pkt_ring() {
    crate::kernel::interrupts::without_interrupts(|| {
        let w = SBC_PKT_WRITE.load(Ordering::Relaxed);
        SBC_PKT_READ.store(w, Ordering::Relaxed);
    });
}

/// Immediately halt A2DP streaming from the soft-lockup emergency handler.
pub fn emergency_stop_a2dp() {
    BT_CMD_ACTIVE.store(true, Ordering::Relaxed);
    STATE.with(|s| {
        s.a2dp_playing = false;
        s.avdtp_streaming = false;
    });
}

/// Synchronize the `1.听歌` (`MusicApp`) playback state with the Bluetooth A2DP SBC audio streamer.
pub fn set_a2dp_playback(playing: bool, track_idx: usize, pos_sec: u32) {
    crate::audio::set_playing(playing);
    STATE.with(|s| {
        let track_changed = s.a2dp_track != (track_idx as u8);
        let resumed = !s.a2dp_playing && playing;
        s.a2dp_playing = playing;
        s.a2dp_track = track_idx as u8;
        if track_changed || resumed {
            unsafe {
                A2DP_SAMPLE_CURSOR = pos_sec.wrapping_mul(44_100);
                if resumed {
                    A2DP_NEXT_TICK = 0;
                }
            }
            clear_sbc_pkt_ring();
        }
    });
}

/// Scrub the A2DP audio synthesis cursor to `pos_sec` seconds when the user touches the progress bar.
pub fn seek_a2dp_playback(pos_sec: u32) {
    crate::audio::seek_to_sec(pos_sec);
    unsafe {
        A2DP_SAMPLE_CURSOR = pos_sec.wrapping_mul(44_100);
    }
    clear_sbc_pkt_ring();
}

/// Quarter-wave 64-entry sine lookup table (`0..=64` -> `0..=1024`)
const SINE_Q64: [i32; 65] = [
    0, 25, 50, 75, 100, 125, 150, 175, 199, 224, 248, 273, 297, 321, 345, 368, 392, 415, 438, 461,
    483, 505, 527, 549, 570, 591, 612, 632, 652, 671, 690, 709, 727, 745, 762, 779, 796, 812, 828,
    843, 857, 871, 885, 898, 910, 922, 934, 945, 955, 965, 974, 982, 990, 997, 1004, 1010, 1015,
    1019, 1022, 1025, 1027, 1029, 1030, 1031, 1031,
];

#[inline(always)]
fn fast_sin(phase_u8: u8) -> i32 {
    let idx = (phase_u8 & 0x3F) as usize;
    match (phase_u8 >> 6) & 3 {
        0 => SINE_Q64[idx],
        1 => SINE_Q64[64 - idx],
        2 => -SINE_Q64[idx],
        _ => -SINE_Q64[64 - idx],
    }
}

/// Compute Bluetooth A2DP SBC CRC-8 (polynomial `0x1D`, initial preset `0x0F`, A2DP Spec 12.6.3.5)
/// over the first `num_bits` bits of `data`.
fn sbc_crc8(data: &[u8], num_bits: usize) -> u8 {
    let mut crc: u8 = 0x0F;
    let full_bytes = num_bits / 8;
    for i in 0..full_bytes {
        let mut b = data[i];
        for _ in 0..8 {
            let bit = (crc ^ b) & 0x80;
            crc <<= 1;
            if bit != 0 {
                crc ^= 0x1D;
            }
            b <<= 1;
        }
    }
    let rem_bits = num_bits % 8;
    if rem_bits > 0 {
        let mut b = data[full_bytes];
        for _ in 0..rem_bits {
            let bit = (crc ^ b) & 0x80;
            crc <<= 1;
            if bit != 0 {
                crc ^= 0x1D;
            }
            b <<= 1;
        }
    }
    crc
}

/// Official Bluetooth A2DP SBC Bit Allocation (`A2DP Spec v1.3 Section 12.6.4`) for 8 subbands.
fn sbc_calculate_bits(
    ch_mode: u8,
    freq_idx: usize,
    alloc_snr: bool,
    bitpool: u8,
    scale_factor: &[[i32; 8]; 2],
    bits: &mut [[i32; 8]; 2],
) {
    const OFFSET8: [[i32; 8]; 4] = [
        [-2, 0, 0, 0, 0, 0, 0, 1], // 16 kHz
        [-3, 0, 0, 0, 0, 0, 1, 2], // 32 kHz
        [-4, 0, 0, 0, 0, 0, 1, 2], // 44.1 kHz
        [-4, 0, 0, 0, 0, 0, 1, 2], // 48 kHz
    ];
    let bp = bitpool as i32;

    if ch_mode == 0x08 || ch_mode == 0x04 {
        // MONO (0x08) or DUAL_CHANNEL (0x04): independent per-channel bit allocation
        let num_ch = if ch_mode == 0x08 { 1 } else { 2 };
        for ch in 0..num_ch {
            let mut bitneed = [0i32; 8];
            let mut max_bitneed = 0i32;
            for sb in 0..8 {
                let bn = if alloc_snr {
                    scale_factor[ch][sb]
                } else {
                    let sf = scale_factor[ch][sb];
                    if sf == 0 {
                        -5
                    } else {
                        let v = sf - OFFSET8[freq_idx & 3][sb];
                        if v > 0 { v / 2 } else { v }
                    }
                };
                bitneed[sb] = bn;
                if bn > max_bitneed {
                    max_bitneed = bn;
                }
            }
            let mut bitcount = 0i32;
            let mut slicecount = 0i32;
            let mut bitslice = max_bitneed + 1;
            loop {
                bitslice -= 1;
                bitcount += slicecount;
                slicecount = 0;
                for sb in 0..8 {
                    if bitneed[sb] > bitslice + 1 && bitneed[sb] < bitslice + 16 {
                        slicecount += 1;
                    } else if bitneed[sb] == bitslice + 1 {
                        slicecount += 2;
                    }
                }
                if bitcount + slicecount >= bp {
                    break;
                }
            }
            if bitcount + slicecount == bp {
                bitcount += slicecount;
                bitslice -= 1;
            }
            for sb in 0..8 {
                if bitneed[sb] < bitslice + 2 {
                    bits[ch][sb] = 0;
                } else {
                    bits[ch][sb] = (bitneed[sb] - bitslice).min(16);
                }
            }
            let mut sb = 0usize;
            while bitcount < bp && sb < 8 {
                if bits[ch][sb] >= 2 && bits[ch][sb] < 16 {
                    bits[ch][sb] += 1;
                    bitcount += 1;
                } else if bitneed[sb] == bitslice + 1 && bp > bitcount + 1 {
                    bits[ch][sb] = 2;
                    bitcount += 2;
                }
                sb += 1;
            }
            sb = 0;
            while bitcount < bp && sb < 8 {
                if bits[ch][sb] < 16 {
                    bits[ch][sb] += 1;
                    bitcount += 1;
                }
                sb += 1;
            }
        }
    } else {
        // STEREO (0x02) or JOINT_STEREO (0x01): joint bit allocation across both channels
        let mut bitneed = [[0i32; 8]; 2];
        let mut max_bitneed = 0i32;
        for ch in 0..2 {
            for sb in 0..8 {
                let bn = if alloc_snr {
                    scale_factor[ch][sb]
                } else {
                    let sf = scale_factor[ch][sb];
                    if sf == 0 {
                        -5
                    } else {
                        let v = sf - OFFSET8[freq_idx & 3][sb];
                        if v > 0 { v / 2 } else { v }
                    }
                };
                bitneed[ch][sb] = bn;
                if bn > max_bitneed {
                    max_bitneed = bn;
                }
            }
        }
        let mut bitcount = 0i32;
        let mut slicecount = 0i32;
        let mut bitslice = max_bitneed + 1;
        loop {
            bitslice -= 1;
            bitcount += slicecount;
            slicecount = 0;
            for ch in 0..2 {
                for sb in 0..8 {
                    if bitneed[ch][sb] > bitslice + 1 && bitneed[ch][sb] < bitslice + 16 {
                        slicecount += 1;
                    } else if bitneed[ch][sb] == bitslice + 1 {
                        slicecount += 2;
                    }
                }
            }
            if bitcount + slicecount >= bp {
                break;
            }
        }
        if bitcount + slicecount == bp {
            bitcount += slicecount;
            bitslice -= 1;
        }
        for ch in 0..2 {
            for sb in 0..8 {
                if bitneed[ch][sb] < bitslice + 2 {
                    bits[ch][sb] = 0;
                } else {
                    bits[ch][sb] = (bitneed[ch][sb] - bitslice).min(16);
                }
            }
        }
        for sb in 0..8 {
            for ch in 0..2 {
                if bitcount >= bp {
                    break;
                }
                if bits[ch][sb] >= 2 && bits[ch][sb] < 16 {
                    bits[ch][sb] += 1;
                    bitcount += 1;
                } else if bitneed[ch][sb] == bitslice + 1 && bp > bitcount + 1 {
                    bits[ch][sb] = 2;
                    bitcount += 2;
                }
            }
        }
        for sb in 0..8 {
            for ch in 0..2 {
                if bitcount >= bp {
                    break;
                }
                if bits[ch][sb] >= 2 && bits[ch][sb] < 16 {
                    bits[ch][sb] += 1;
                    bitcount += 1;
                }
            }
        }
    }
}

/// Encode a single 16-block, 8-subband Bluetooth A2DP SBC frame (`72` bytes in Stereo, `73` in Joint Stereo)
/// using a fast in-register 8-band dyadic frequency filterbank (`< 25 µs` on Non-Cacheable DRAM).
unsafe fn encode_sbc_frame(
    sbc_cap0: u8,
    sbc_cap1: u8,
    bitpool: u8,
    track_idx: u8,
    sample_cursor: &mut u32,
    out: &mut [u8],
) -> usize {
    let freq_idx = if (sbc_cap0 & 0x80) != 0 {
        0usize
    } else if (sbc_cap0 & 0x40) != 0 {
        1usize
    } else if (sbc_cap0 & 0x10) != 0 {
        3usize
    } else {
        2usize
    };

    let ch_mode = sbc_cap0 & 0x0F;
    let num_ch = if ch_mode == 0x08 { 1usize } else { 2usize };
    let alloc_snr = (sbc_cap1 & 0x02) != 0;

    static mut A2DP_PCM_LR: [[i16; 128]; 2] = [[0i16; 128]; 2];
    static mut A2DP_SB_SAMPLES: [[[i32; 8]; 2]; 16] = [[[0i32; 8]; 2]; 16];

    let pcm_lr = &mut *(&raw mut A2DP_PCM_LR);
    let sb_samples = &mut *(&raw mut A2DP_SB_SAMPLES);

    // 1. Pop 128 stereo PCM samples from `audio::PCM_RING` (or fallback synthesizer)
    crate::audio::pop_pcm_128(pcm_lr, track_idx, sample_cursor);

    // 2. Fast in-register 8-Subband Dyadic Frequency Decomposition (`0..22.05 kHz`, zero DRAM shifts)
    let mut max_abs_sb = [[0u32; 8]; 2];
    for blk in 0..16 {
        let base_t = blk * 8;
        for ch in 0..num_ch {
            let p = &pcm_lr[ch];
            let s0 = p[base_t] as i32;
            let s1 = p[base_t + 1] as i32;
            let s2 = p[base_t + 2] as i32;
            let s3 = p[base_t + 3] as i32;
            let s4 = p[base_t + 4] as i32;
            let s5 = p[base_t + 5] as i32;
            let s6 = p[base_t + 6] as i32;
            let s7 = p[base_t + 7] as i32;

            // Stage 1: Pairs
            let a0 = s0 + s1;
            let d0 = s0 - s1;
            let a1 = s2 + s3;
            let d1 = s2 - s3;
            let a2 = s4 + s5;
            let d2 = s4 - s5;
            let a3 = s6 + s7;
            let d3 = s6 - s7;

            // Stage 2: Quads
            let b0 = a0 + a1;
            let b1 = a0 - a1;
            let b2 = a2 + a3;
            let b3 = a2 - a3;
            let e0 = d0 - d1;
            let e1 = d0 + d1;
            let e2 = d2 - d3;
            let e3 = d2 + d3;

            // Stage 3: 8 frequency-ordered subbands (sb0 = 0..2.75kHz .. sb7 = 19.3..22.05kHz)
            let bands: [i32; 8] = [
                (b0 + b2) >> 3,
                (b0 - b2) >> 3,
                (b1 - b3) >> 3,
                (b1 + b3) >> 3,
                (e0 + e2) >> 4,
                (e0 - e2) >> 4,
                (e1 - e3) >> 4,
                (e1 + e3) >> 4,
            ];

            for sb in 0..8 {
                let v = bands[sb];
                sb_samples[blk][ch][sb] = v;
                let a = v.unsigned_abs();
                if a > max_abs_sb[ch][sb] {
                    max_abs_sb[ch][sb] = a;
                }
            }
        }
    }

    // 3. Compute dynamic `scale_factor[ch][0..7]` (`0..15`) from peak subband magnitude
    let mut scale_factor = [[0i32; 8]; 2];
    for ch in 0..num_ch {
        for sb in 0..8 {
            let m = max_abs_sb[ch][sb];
            scale_factor[ch][sb] = if m <= 1 {
                0
            } else {
                (31 - m.leading_zeros()).min(15) as i32
            };
        }
    }

    let mut bits = [[0i32; 8]; 2];
    sbc_calculate_bits(ch_mode, freq_idx, alloc_snr, bitpool, &scale_factor, &mut bits);

    // 1. Pack 4-byte SBC Frame Header
    let ch_mode_bits: u8 = match ch_mode {
        0x08 => 0b00, // Mono
        0x04 => 0b01, // Dual Channel
        0x02 => 0b10, // Stereo
        _ => 0b11,    // Joint Stereo
    };
    let hdr1: u8 = ((freq_idx as u8) << 6)
        | (0b11 << 4) // 16 blocks
        | (ch_mode_bits << 2)
        | ((alloc_snr as u8) << 1)
        | 0x01; // 8 subbands

    out[0] = 0x9C; // Syncword
    out[1] = hdr1;
    out[2] = bitpool;

    // 2. Pack Scale Factors (and optional Joint Stereo `join = 0x00` byte) + compute CRC-8
    let mut crc_buf = [0u8; 11];
    crc_buf[0] = hdr1;
    crc_buf[1] = bitpool;
    let mut crc_bits = 16usize;
    let mut pos = 4usize;

    if ch_mode == 0x01 {
        // Joint Stereo: 8-bit `join` flags (`0x00` = no subbands joined)
        out[pos] = 0x00;
        pos += 1;
        crc_buf[2] = 0x00;
        crc_bits += 8;
    }

    let sf_crc_start = crc_bits / 8;
    for ch in 0..num_ch {
        for sb_pair in 0..4 {
            let sf0 = (scale_factor[ch][sb_pair * 2] as u8) & 0x0F;
            let sf1 = (scale_factor[ch][sb_pair * 2 + 1] as u8) & 0x0F;
            let b = (sf0 << 4) | sf1;
            out[pos] = b;
            crc_buf[sf_crc_start + ch * 4 + sb_pair] = b;
            pos += 1;
            crc_bits += 8;
        }
    }

    out[3] = sbc_crc8(&crc_buf, crc_bits);

    // 3. Quantize & pack 16 blocks of subband samples MSB-first into `out[pos..]`
    let mut bit_acc: u32 = 0;
    let mut bit_cnt: u32 = 0;

    for blk in 0..16 {
        for ch in 0..num_ch {
            for sb in 0..8 {
                let b = bits[ch][sb] as u32;
                if b > 0 {
                    let levels = (1i64 << b) - 1;
                    let sf = scale_factor[ch][sb] as u32;
                    let s = sb_samples[blk][ch][sb] as i64;
                    let num = (s + (1i64 << (sf + 1))) * levels;
                    let q = (num >> (sf + 2)).clamp(0, levels - 1) as u32;

                    bit_acc = (bit_acc << b) | (q & ((1u32 << b) - 1));
                    bit_cnt += b;
                    while bit_cnt >= 8 {
                        bit_cnt -= 8;
                        out[pos] = ((bit_acc >> bit_cnt) & 0xFF) as u8;
                        pos += 1;
                    }
                }
            }
        }
    }

    if bit_cnt > 0 {
        out[pos] = ((bit_acc << (8 - bit_cnt)) & 0xFF) as u8;
        pos += 1;
    }

    pos
}

/// Read a single byte from `UART0` within `timeout_us` microseconds (`150 µs` = `0.15 ms` max wait).
/// At `921,600` baud, consecutive bytes inside an H4 packet arrive every `10.85 µs`, so we NEVER block
/// for `40 ms` if an H4 packet is truncated!
#[inline]
unsafe fn uart0_getc_us(timeout_us: u64) -> Option<u8> {
    if (mmio::read(UART_FR) & FR_RXFE) == 0 {
        return Some((mmio::read(UART_DR) & 0xFF) as u8);
    }
    let freq = timer::frequency();
    let deadline = timer::counter() + ((freq * timeout_us) / 1_000_000).max(2);
    loop {
        if (mmio::read(UART_FR) & FR_RXFE) == 0 {
            return Some((mmio::read(UART_DR) & 0xFF) as u8);
        }
        if timer::counter() >= deadline {
            return None;
        }
        core::hint::spin_loop();
    }
}

/// Fast non-blocking `UART0` RX drainer for active A2DP streaming (`< 0.15 ms` max latency).
/// Consumes `0x13 (Number_Of_Completed_Packets)` events and dispatches incoming AVRCP/AVDTP L2CAP packets.
unsafe fn poll_a2dp_rx_fast(acl_handle: u16) -> bool {
    static mut A2DP_RX_PKT: [u8; 260] = [0u8; 260];
    let rx_pkt = &mut *(&raw mut A2DP_RX_PKT);
    let mut guard = 8u8;

    while guard > 0 && (mmio::read(UART_FR) & FR_RXFE) == 0 {
        guard -= 1;
        let pkt_type = (mmio::read(UART_DR) & 0xFF) as u8;
        match pkt_type {
            0x04 => {
                let Some(evt_code) = uart0_getc_us(150) else {
                    break;
                };
                let Some(plen_u8) = uart0_getc_us(150) else {
                    break;
                };
                let plen = plen_u8 as usize;
                let mut ok = true;
                for i in 0..plen {
                    if let Some(b) = uart0_getc_us(150) {
                        if i < rx_pkt.len() {
                            rx_pkt[i] = b;
                        }
                    } else {
                        ok = false;
                        break;
                    }
                }
                if !ok {
                    break;
                }
                if evt_code == 0x05 && plen >= 4 {
                    // HCI Disconnection Complete (0x05)
                    STATE.with(|s| {
                        for d in s.devices.iter_mut() {
                            d.connected = false;
                        }
                        s.connected_index = None;
                        s.acl_handle = 0;
                        s.a2dp_ready = false;
                        s.avdtp_streaming = false;
                    });
                    return false;
                }
            }
            0x02 => {
                let (Some(_h_lo), Some(h_hi), Some(dlen_lo), Some(dlen_hi)) = (
                    uart0_getc_us(150),
                    uart0_getc_us(150),
                    uart0_getc_us(150),
                    uart0_getc_us(150),
                ) else {
                    break;
                };
                let pb_flag = (h_hi >> 4) & 0x03;
                let dlen = u16::from_le_bytes([dlen_lo, dlen_hi]) as usize;
                if dlen > 256 {
                    // Framing desync recovery: drain FIFO immediately without blocking
                    while (mmio::read(UART_FR) & FR_RXFE) == 0 {
                        let _ = mmio::read(UART_DR);
                    }
                    break;
                }
                let mut ok = true;
                for i in 0..dlen {
                    if let Some(b) = uart0_getc_us(150) {
                        rx_pkt[i] = b;
                    } else {
                        ok = false;
                        break;
                    }
                }
                if ok && dlen > 0 {
                    let cs_ptr = &raw mut RUNTIME_CS;
                    let reasm_ptr = &raw mut RUNTIME_REASM;
                    let prev_stream = (*cs_ptr).avdtp_streaming;
                    (*reasm_ptr).feed_and_dispatch(
                        acl_handle,
                        pb_flag,
                        &rx_pkt[..dlen],
                        &mut *cs_ptr,
                    );
                    if (*cs_ptr).avdtp_streaming != prev_stream {
                        let new_stream = (*cs_ptr).avdtp_streaming;
                        STATE.with(|s| s.avdtp_streaming = new_stream);
                    }
                }
            }
            _ => {}
        }
    }
    true
}

/// Non-blockingly drain any incoming HCI Events (`0x04`) or ACL packets (`0x02`) on `UART0`
/// and transmit hardware-timer-paced RTP + A2DP SBC audio frames (`5` frames/pkt = `640` samples = `14.51 ms`)
/// to the connected Bluetooth speaker (`SRS-XB100`).
///
/// Safe to call from both `timer::handle_irq()` (`100Hz` hardware timer IRQ) and `ui_task()`.
/// Uses IRQ-masked `Ordering::Relaxed` instead of `ldaxrb`/`stlxrb` (`compare_exchange`) because
/// Cortex-A53 Non-Cacheable Normal memory faults on exclusive monitor instructions (`src/sync/mod.rs`).
pub fn pump_a2dp_audio() {
    if bsp::is_qemu() || BT_CMD_ACTIVE.load(Ordering::Relaxed) {
        return;
    }

    let acquired = crate::kernel::interrupts::without_interrupts(|| {
        if BT_CMD_ACTIVE.load(Ordering::Relaxed) || A2DP_PUMP_ACTIVE.load(Ordering::Relaxed) {
            false
        } else {
            A2DP_PUMP_ACTIVE.store(true, Ordering::Relaxed);
            true
        }
    });
    if !acquired {
        return;
    }

    unsafe {
        pump_a2dp_audio_inner();
    }

    A2DP_PUMP_ACTIVE.store(false, Ordering::Relaxed);
}

#[derive(Clone, Copy)]
struct A2dpPumpSnap {
    active_acl: bool,
    acl_handle: u16,
    a2dp_ready: bool,
    a2dp_playing: bool,
    avdtp_streaming: bool,
    avdtp_sig_dcid: u16,
    avdtp_media_dcid: u16,
    acp_seid_shifted: u8,
    sbc_cap0: u8,
    sbc_cap1: u8,
    sbc_bitpool: u8,
    a2dp_track: u8,
}

/// Build a single RTP + 5-frame SBC media packet (`14.51 ms` of stereo audio) into `media_pkt`.
unsafe fn build_one_a2dp_packet(
    sbc_cap0: u8,
    sbc_cap1: u8,
    sbc_bitpool: u8,
    a2dp_track: u8,
    media_pkt: &mut [u8; 440],
) -> usize {
    const FRAMES_PER_PKT: usize = 5;
    let pkt_samples = (FRAMES_PER_PKT * 128) as u32;
    let seq = A2DP_SEQ;
    let ts = A2DP_TS;
    A2DP_SEQ = A2DP_SEQ.wrapping_add(1);
    A2DP_TS = A2DP_TS.wrapping_add(pkt_samples);

    // 12-byte RTP Header (V=2, PT=96 dynamic A2DP payload, SSRC=0x00000001)
    media_pkt[0] = 0x80;
    media_pkt[1] = 0x60;
    media_pkt[2] = (seq >> 8) as u8;
    media_pkt[3] = (seq & 0xFF) as u8;
    media_pkt[4] = (ts >> 24) as u8;
    media_pkt[5] = (ts >> 16) as u8;
    media_pkt[6] = (ts >> 8) as u8;
    media_pkt[7] = (ts & 0xFF) as u8;
    media_pkt[8] = 0x00;
    media_pkt[9] = 0x00;
    media_pkt[10] = 0x00;
    media_pkt[11] = 0x01;
    // 1-byte A2DP SBC Media Payload Header (`num_frames = 5`)
    media_pkt[12] = FRAMES_PER_PKT as u8;

    let mut off = 13usize;
    let mut cur = A2DP_SAMPLE_CURSOR;
    for _ in 0..FRAMES_PER_PKT {
        let flen = encode_sbc_frame(
            sbc_cap0,
            sbc_cap1,
            sbc_bitpool.clamp(2, 53),
            a2dp_track,
            &mut cur,
            &mut media_pkt[off..],
        );
        off += flen;
    }
    A2DP_SAMPLE_CURSOR = cur;
    off
}

/// Pre-encode RTP + SBC packets in normal thread context (`ui_task`) so the `100Hz` Timer IRQ
/// only has to pop a ready-made byte slice and write it to `UART0`.
pub fn refill_sbc_pkt_ring() {
    if bsp::is_qemu() || BT_CMD_ACTIVE.load(Ordering::Relaxed) {
        return;
    }
    let st = STATE.with(|s| A2dpPumpSnap {
        active_acl: s.initialized && s.hci_up && s.connected_index.is_some() && s.acl_handle != 0,
        acl_handle: s.acl_handle,
        a2dp_ready: s.a2dp_ready,
        a2dp_playing: s.a2dp_playing,
        avdtp_streaming: s.avdtp_streaming,
        avdtp_sig_dcid: s.avdtp_sig_dcid,
        avdtp_media_dcid: s.avdtp_media_dcid,
        acp_seid_shifted: s.acp_seid_shifted,
        sbc_cap0: s.sbc_cap0,
        sbc_cap1: s.sbc_cap1,
        sbc_bitpool: s.sbc_bitpool,
        a2dp_track: s.a2dp_track,
    });
    if !st.active_acl || !st.a2dp_ready || !st.a2dp_playing || st.avdtp_media_dcid == 0 {
        return;
    }

    for _ in 0..12 {
        let r = SBC_PKT_READ.load(Ordering::Relaxed);
        let w = SBC_PKT_WRITE.load(Ordering::Relaxed);
        if w.wrapping_sub(r) >= 14 {
            break;
        }
        let slot = w & SBC_PKT_RING_MASK;
        unsafe {
            let pkt_buf = &mut SBC_PKT_DATA[slot];
            let len = build_one_a2dp_packet(
                st.sbc_cap0,
                st.sbc_cap1,
                st.sbc_bitpool,
                st.a2dp_track,
                pkt_buf,
            );
            SBC_PKT_LEN[slot] = len as u16;
        }
        SBC_PKT_WRITE.store(w.wrapping_add(1), Ordering::Relaxed);
    }
}

unsafe fn pump_a2dp_audio_inner() {
    let st = STATE.with(|s| A2dpPumpSnap {
        active_acl: s.initialized && s.hci_up && s.connected_index.is_some() && s.acl_handle != 0,
        acl_handle: s.acl_handle,
        a2dp_ready: s.a2dp_ready,
        a2dp_playing: s.a2dp_playing,
        avdtp_streaming: s.avdtp_streaming,
        avdtp_sig_dcid: s.avdtp_sig_dcid,
        avdtp_media_dcid: s.avdtp_media_dcid,
        acp_seid_shifted: s.acp_seid_shifted,
        sbc_cap0: s.sbc_cap0,
        sbc_cap1: s.sbc_cap1,
        sbc_bitpool: s.sbc_bitpool,
        a2dp_track: s.a2dp_track,
    });
    if !st.active_acl {
        return;
    }

    // 1. Ultra-fast non-blocking UART0 RX drain (`< 0.15 ms` max)
    if !poll_a2dp_rx_fast(st.acl_handle) {
        return;
    }

    if !st.a2dp_ready || !st.a2dp_playing || st.avdtp_media_dcid == 0 {
        return;
    }

    // 2. Ensure AVDTP stream is in STREAMING state (`AVDTP_START = 0x07`)
    if !st.avdtp_streaming && st.avdtp_sig_dcid != 0 {
        let start_cmd: [u8; 3] = [0x50, 0x07, st.acp_seid_shifted];
        let _ = hci_send_l2cap(st.acl_handle, st.avdtp_sig_dcid, &start_cmd);
        let cs_ptr = &raw mut RUNTIME_CS;
        (*cs_ptr).avdtp_streaming = true;
        STATE.with(|s| s.avdtp_streaming = true);
    }

    // 3. Hardware-timer-paced RTP + SBC media packet transmission (`5` SBC frames = `640` samples = `14.51 ms`)
    //    CRITICAL: Send AT MOST 1 packet per call (`~4.3 ms` at 921,600 baud) so the 100Hz (`10 ms`)
    //    Timer IRQ never exceeds 45% CPU utilization and NEVER starves `ui_task()`!
    const FRAMES_PER_PKT: usize = 5;
    let sample_rate: u64 = if (st.sbc_cap0 & 0x80) != 0 {
        16_000
    } else if (st.sbc_cap0 & 0x40) != 0 {
        32_000
    } else if (st.sbc_cap0 & 0x10) != 0 {
        48_000
    } else {
        44_100
    };
    let freq = timer::frequency();
    let pkt_samples = (FRAMES_PER_PKT * 128) as u64; // 640 samples (~14.51 ms at 44.1 kHz)
    let pkt_ticks = (freq * pkt_samples) / sample_rate;
    let lead_ticks = (freq * 85) / 1000; // Keep ~85 ms pre-buffer cushion in speaker jitter buffer

    let now = timer::counter();
    if A2DP_NEXT_TICK == 0 || now > A2DP_NEXT_TICK + (freq / 3) {
        A2DP_NEXT_TICK = now.saturating_sub(pkt_ticks);
    }

    if A2DP_NEXT_TICK <= now + lead_ticks {
        A2DP_NEXT_TICK += pkt_ticks;

        let r = SBC_PKT_READ.load(Ordering::Relaxed);
        let w = SBC_PKT_WRITE.load(Ordering::Relaxed);
        if r != w {
            let slot = r & SBC_PKT_RING_MASK;
            let len = (SBC_PKT_LEN[slot] as usize).min(440);
            let pkt_slice = &SBC_PKT_DATA[slot][..len];
            let _ = hci_send_l2cap(st.acl_handle, st.avdtp_media_dcid, pkt_slice);
            SBC_PKT_READ.store(r.wrapping_add(1), Ordering::Relaxed);
        } else {
            static mut A2DP_FALLBACK_PKT: [u8; 440] = [0u8; 440];
            let media_pkt = &mut *(&raw mut A2DP_FALLBACK_PKT);
            let off = build_one_a2dp_packet(
                st.sbc_cap0,
                st.sbc_cap1,
                st.sbc_bitpool,
                st.a2dp_track,
                media_pkt,
            );
            let _ = hci_send_l2cap(st.acl_handle, st.avdtp_media_dcid, &media_pkt[..off]);
        }

        let _ = poll_a2dp_rx_fast(st.acl_handle);
        STATE.with(|s| s.a2dp_packets_sent = s.a2dp_packets_sent.wrapping_add(1));
    }
}

pub fn init() -> bool {
    if bsp::is_qemu() {
        STATE.with(|s| {
            s.initialized = true;
            s.hci_up = true;
            s.fw_patched = true;
            s.baud_rate = 921_600;
            s.hci_version = 8;
            s.hci_revision = 0x005C;
            s.manufacturer = 0x000F;
            s.lmp_subversion = 0x410C;
            s.bd_addr = [0x98, 0xFE, 0x54, 0x81, 0x9A, 0x88];
            s.ble_advertising = true;
            s.classic_discoverable = true;
            s.commands_sent = 12;
        });
        let _ = scan_devices();
        crate::println!("[OK] Bluetooth HCI (QEMU): BCM43430B0 b0092 BD=98:FE:54:81:9A:88");
        return true;
    }

    BT_CMD_ACTIVE.store(true, Ordering::Relaxed);
    uart::set_bt_mode(true);

    let ok = unsafe { init_bt_hw() };
    BT_CMD_ACTIVE.store(false, Ordering::Relaxed);
    let st = status();
    if ok {
        crate::println!(
            "[OK] BT HCI (b{:04x}): BD={:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X} BLE={}",
            st.hci_revision & 0x0FFF,
            st.bd_addr[0],
            st.bd_addr[1],
            st.bd_addr[2],
            st.bd_addr[3],
            st.bd_addr[4],
            st.bd_addr[5],
            if st.ble_advertising { "ON" } else { "OFF" }
        );
    } else {
        crate::println!("[WARN] Bluetooth HCI: UART0 handshake timeout");
    }
    ok
}

unsafe fn init_bt_hw() -> bool {
    setup_bt_gpio();
    STATE.with(|s| s.initialized = true);

    let mut resp = [0u8; 32];
    let mut locked_baud = 0u32;

    let baud_candidates: [(u32, u32, u32, bool); 4] = [
        (3_000_000, 1, 0, true),
        (2_000_000, 1, 32, true),
        (921_600, 3, 16, false),
        (115_200, 26, 3, false),
    ];

    for &(baud, ibrd, fbrd, flow) in &baud_candidates {
        configure_uart0(ibrd, fbrd, flow);
        if hci_send_cmd(0x0C03, &[], &mut resp, 40).is_ok() {
            locked_baud = baud;
            break;
        }
    }

    if locked_baud == 0 {
        reset_bt_chip_via_gpio42();
        configure_uart0(26, 3, false);
        if hci_send_cmd(0x0C03, &[], &mut resp, 200).is_ok() {
            locked_baud = 115_200;
        } else {
            configure_uart0(26, 3, true);
            if hci_send_cmd(0x0C03, &[], &mut resp, 200).is_ok() {
                locked_baud = 115_200;
            }
        }
    }

    if locked_baud == 0 {
        return false;
    }

    timer::delay_ms(10);

    let mut hci_ver = 0u8;
    let mut hci_rev = 0u16;
    let mut manf = 0u16;
    let mut lmp_sub = 0u16;
    if let Ok(n) = hci_send_cmd(0x1001, &[], &mut resp, 80) {
        if n >= 8 {
            hci_ver = resp[0];
            hci_rev = u16::from_le_bytes([resp[1], resp[2]]);
            manf = u16::from_le_bytes([resp[4], resp[5]]);
            lmp_sub = u16::from_le_bytes([resp[6], resp[7]]);
        }
    }

    let mut fw_patched = (hci_rev & 0x0FFF) != 0;
    if !fw_patched {
        if load_bcm43430_hcd_patch(&mut locked_baud) {
            if let Ok(n) = hci_send_cmd(0x1001, &[], &mut resp, 80) {
                if n >= 8 {
                    hci_ver = resp[0];
                    hci_rev = u16::from_le_bytes([resp[1], resp[2]]);
                    manf = u16::from_le_bytes([resp[4], resp[5]]);
                    lmp_sub = u16::from_le_bytes([resp[6], resp[7]]);
                    fw_patched = (hci_rev & 0x0FFF) != 0;
                }
            }
        }
    } else if locked_baud == 115_200 {
        let baud_921k: [u8; 6] = [0x00, 0x00, 0x00, 0x10, 0x0E, 0x00];
        if hci_send_cmd(0xFC18, &baud_921k, &mut resp, 80).is_ok() {
            timer::delay_ms(10);
            configure_uart0(3, 16, false);
            locked_baud = 921_600;
        }
    }

    let vsc_bd = [0x88, 0x9A, 0x81, 0x54, 0xFE, 0x98];
    let _ = hci_send_cmd(0xFC01, &vsc_bd, &mut resp, 80);

    let mut bd_addr = [0u8; 6];
    if let Ok(n) = hci_send_cmd(0x1009, &[], &mut resp, 80) {
        if n >= 6 {
            for i in 0..6 {
                bd_addr[i] = resp[5 - i];
            }
        }
    }

    let _ = hci_send_cmd(0xFC27, &[0u8; 12], &mut resp, 80);
    let _ = hci_send_cmd(
        0x0C01,
        &[0xFF, 0xFF, 0xFB, 0xFF, 0x07, 0xF8, 0xBF, 0x3D],
        &mut resp,
        80,
    );
    let _ = hci_send_cmd(
        0x2001,
        &[0xFF, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
        &mut resp,
        80,
    );
    let _ = hci_send_cmd(0x0C56, &[0x01], &mut resp, 80);
    let _ = hci_send_cmd(0x0C6D, &[0x01, 0x00], &mut resp, 80);
    let _ = hci_send_cmd(0x0C45, &[0x02], &mut resp, 80);

    let mut local_name = [0u8; 248];
    let name_bytes = b"ZerOS-Pi";
    local_name[..name_bytes.len()].copy_from_slice(name_bytes);
    let _ = hci_send_cmd(0x0C13, &local_name, &mut resp, 150);

    let mut eir = [0u8; 241];
    let eir_ad: [u8; 18] = [
        0x09, 0x09, b'Z', b'e', b'r', b'O', b'S', b'-', b'P', b'i',
        0x07, 0x03, 0x0A, 0x11, 0x0B, 0x11, 0x0D, 0x11, // AudioSource (110A), AudioSink (110B), A2DP (110D)
    ];
    eir[1..1 + eir_ad.len()].copy_from_slice(&eir_ad);
    let _ = hci_send_cmd(0x0C52, &eir, &mut resp, 150);

    // Set Class of Device (0x0C24) to Smartphone + Audio Source + Capturing (`0x7A020C`)
    let _ = hci_send_cmd(0x0C24, &[0x0C, 0x02, 0x7A], &mut resp, 80);
    let _ = hci_send_cmd(0x0C1E, &[0xA0, 0x00, 0x50, 0x00], &mut resp, 80);
    let _ = hci_send_cmd(0x0C1C, &[0x00, 0x01, 0x50, 0x00], &mut resp, 80);
    let _ = hci_send_cmd(0x0C43, &[0x01], &mut resp, 80);
    let _ = hci_send_cmd(0x0C47, &[0x01], &mut resp, 80);
    let classic_ok = hci_send_cmd(0x0C1A, &[0x03], &mut resp, 80).is_ok();

    let _ = hci_send_cmd(0x200A, &[0x00], &mut resp, 40);

    let adv_params: [u8; 15] = [
        0x30, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0, 0, 0, 0x07, 0x00,
    ];
    let _ = hci_send_cmd(0x2006, &adv_params, &mut resp, 80);

    let mut adv_data = [0u8; 32];
    let payload: [u8; 21] = [
        0x02, 0x01, 0x06,
        0x03, 0x03, 0x12, 0x18,
        0x03, 0x19, 0xC1, 0x03,
        0x09, 0x09, b'Z', b'e', b'r', b'O', b'S', b'-', b'P', b'i',
    ];
    adv_data[0] = payload.len() as u8;
    adv_data[1..1 + payload.len()].copy_from_slice(&payload);
    let _ = hci_send_cmd(0x2008, &adv_data, &mut resp, 80);
    let _ = hci_send_cmd(0x2009, &adv_data, &mut resp, 80);

    let ble_ok = hci_send_cmd(0x200A, &[0x01], &mut resp, 80).is_ok();

    STATE.with(|s| {
        s.hci_up = true;
        s.fw_patched = fw_patched;
        s.baud_rate = locked_baud;
        s.hci_version = hci_ver;
        s.hci_revision = hci_rev;
        s.manufacturer = manf;
        s.lmp_subversion = lmp_sub;
        s.bd_addr = bd_addr;
        s.ble_advertising = ble_ok;
        s.classic_discoverable = classic_ok;
    });

    true
}

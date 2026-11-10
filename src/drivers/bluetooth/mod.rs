// ZerOS — Onboard Bluetooth 5.0 HCI / BLE Driver (`BCM43430B0` / `SYN43436`) (M8.3)
//
// Drives the Raspberry Pi Zero 2 W's onboard Bluetooth 5.0 / BLE controller over
// PL011 `UART0` (`0x3F20_1000`) routed to `GPIO 30..33` (`ALT3` = `CTS0, RTS0, TXD0, RXD0`)
// and hardware power-enable pin `GPIO 42` (`BT_ON`).
//
// Capabilities:
//   1. Claims PL011 `UART0` from debug print output on real hardware and routes `GPIO 30..33` (`ALT3`)
//   2. Auto-detects active Bluetooth HCI UART baud rate (`3Mbps` / `921.6kbps` / `115.2kbps`) across
//      warm reboots, or performs a clean hardware reset via `GPIO 42` (`BT_ON`) to boot the ROM at `115,200` baud
//   3. Automatically loads `/boot/firmware/BTFW.HCD` (`SYN43430B0.hcd`, `37.4MHz wlbga_iLNA_iTR`
//      RF synthesizer & antenna switch patch, `build 0092`) from the FAT32 SD card at `921,600` baud
//      whenever the controller is in unpatched ROM state (`build 0000`)!
//   4. Configures full Classic Bluetooth (BR/EDR) + BLE Dual-Mode Discovery:
//      - `HCI_VSC_Write_BD_ADDR (0xFC01)` -> `98:FE:54:81:9A:88`
//      - `HCI_Write_Simple_Pairing_Mode (0x0C56)` & `HCI_Write_LE_Host_Support (0x0C6D)`
//      - `HCI_Write_Local_Name (0x0C13)` -> `"ZerOS-Pi"` (248 bytes)
//      - `HCI_Write_Extended_Inquiry_Response (0x0C52)` -> `"ZerOS-Pi"` EIR (241 bytes)
//      - `HCI_Write_Class_of_Device (0x0C24)` -> `0x100104` (Computer / Workstation)
//      - `HCI_Write_Scan_Enable (0x0C1A)` -> `0x03` (Inquiry + Page Scan enabled)
//      - `HCI_LE_Set_Advertising_Parameters/Data/ScanRsp/Enable (0x2006..0x200A)` -> `"ZerOS-Pi"`

#![allow(dead_code)]

use crate::arch::timer;
use crate::bsp;
use crate::drivers::{mmio, uart};
use crate::fs::fat32;
use crate::kernel::tryboot;
use crate::sync::IrqSafeLock;

// ─── PL011 UART0 Registers (Base: 0x3F20_1000) ───
const UART_DR: usize = mmio::UART0_BASE + 0x00;
const UART_FR: usize = mmio::UART0_BASE + 0x18;
const UART_IBRD: usize = mmio::UART0_BASE + 0x24;
const UART_FBRD: usize = mmio::UART0_BASE + 0x28;
const UART_LCRH: usize = mmio::UART0_BASE + 0x2C;
const UART_CR: usize = mmio::UART0_BASE + 0x30;
const UART_ICR: usize = mmio::UART0_BASE + 0x44;

const FR_RXFE: u32 = 1 << 4; // Receive FIFO empty
const FR_TXFF: u32 = 1 << 5; // Transmit FIFO full
const FR_BUSY: u32 = 1 << 3; // UART busy

// ─── GPIO Registers for Bluetooth UART0 (GPIO 30..33) & BT_ON (GPIO 42) ───
const GPFSEL1: usize = mmio::GPIO_BASE + 0x04; // GPIO 10..19
const GPFSEL3: usize = mmio::GPIO_BASE + 0x0C; // GPIO 30..39
const GPFSEL4: usize = mmio::GPIO_BASE + 0x10; // GPIO 40..49
const GPSET1: usize = mmio::GPIO_BASE + 0x20; // Set GPIO 32..53
const GPCLR1: usize = mmio::GPIO_BASE + 0x2C; // Clear GPIO 32..53
const GPPUD: usize = mmio::GPIO_BASE + 0x94;
const GPPUDCLK0: usize = mmio::GPIO_BASE + 0x98;
const GPPUDCLK1: usize = mmio::GPIO_BASE + 0x9C;

/// Live status of the Onboard Bluetooth 5.0 HCI / BLE Controller
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
    pub bd_addr: [u8; 6], // Stored in display order: [MSB .. LSB]
    pub ble_advertising: bool,
    pub classic_discoverable: bool,
    pub commands_sent: u32,
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
        }
    }
}

static STATE: IrqSafeLock<BluetoothStatus> = IrqSafeLock::new(BluetoothStatus::empty());

/// Return the cached Bluetooth HCI / BLE status.
pub fn status() -> BluetoothStatus {
    STATE.with(|s| *s)
}

/// Route `GPIO 30..33` to `ALT3` (`CTS0, RTS0, TXD0, RXD0`) and configure `GPIO 42` (`BT_ON`).
unsafe fn setup_bt_gpio() {
    // 1. Disconnect GPIO 14..15 from UART0 so header pins don't interfere with Bluetooth UART0
    let mut fsel1 = mmio::read(GPFSEL1);
    fsel1 &= !((0b111 << 12) | (0b111 << 15));
    mmio::write(GPFSEL1, fsel1);

    // 2. Route GPIO 30 (CTS0), 31 (RTS0), 32 (TXD0), 33 (RXD0) to ALT3 (0b111)
    let mut fsel3 = mmio::read(GPFSEL3);
    for pin in 30..=33 {
        let shift = (pin - 30) * 3;
        fsel3 = (fsel3 & !(0b111 << shift)) | (0b111 << shift);
    }
    mmio::write(GPFSEL3, fsel3);

    // Enable pull-up on RXD0 (GPIO 33)
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

    // 3. Configure GPIO 42 (BT_ON) as OUTPUT (0b001 in GPFSEL4 bits [8:6]) and ensure HIGH
    let mut fsel4 = mmio::read(GPFSEL4);
    fsel4 = (fsel4 & !(0b111 << 6)) | (0b001 << 6);
    mmio::write(GPFSEL4, fsel4);
    mmio::write(GPSET1, 1 << (42 - 32));
}

/// Perform a clean hardware reset pulse on `GPIO 42` (`BT_ON`) so the `BCM43430B0` Bluetooth
/// controller resets to its ROM default `115,200` baud H4 UART state.
unsafe fn reset_bt_chip_via_gpio42() {
    let bt_on_bit = 1u32 << (42 - 32);
    mmio::write(GPCLR1, bt_on_bit);
    timer::delay_ms(30);
    mmio::write(GPSET1, bt_on_bit);
    timer::delay_ms(140);
}

/// Configure PL011 `UART0` (`0x3F20_1000`) for the given divisor (`ibrd`, `fbrd`) @ 48MHz.
unsafe fn configure_uart0(ibrd: u32, fbrd: u32, hw_flow_ctrl: bool) {
    let mut guard = 10_000u32;
    while (mmio::read(UART_FR) & FR_BUSY) != 0 && guard > 0 {
        guard -= 1;
        core::hint::spin_loop();
    }
    mmio::write(UART_CR, 0);
    mmio::write(UART_ICR, 0x7FF);

    mmio::write(UART_IBRD, ibrd);
    mmio::write(UART_FBRD, fbrd);
    // 8N1, FIFOs enabled
    mmio::write(UART_LCRH, (1 << 4) | (1 << 5) | (1 << 6));

    // UARTEN (0) | TXE (8) | RXE (9) | RTS (11, drive RTS0 active-low)
    let mut cr = (1 << 0) | (1 << 8) | (1 << 9) | (1 << 11);
    if hw_flow_ctrl {
        cr |= (1 << 14) | (1 << 15); // RTSEN | CTSEN
    }
    mmio::write(UART_CR, cr);
    timer::delay_ms(2);

    // Drain any stale bytes from RX FIFO
    let mut drain = 256u32;
    while (mmio::read(UART_FR) & FR_RXFE) == 0 && drain > 0 {
        let _ = mmio::read(UART_DR);
        drain -= 1;
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
    let spins_per_ms = 8_000u32;
    let mut total_spins = timeout_ms.saturating_mul(spins_per_ms).max(1_000);
    while total_spins > 0 {
        if (mmio::read(UART_FR) & FR_RXFE) == 0 {
            return Some((mmio::read(UART_DR) & 0xFF) as u8);
        }
        total_spins -= 1;
        core::hint::spin_loop();
    }
    None
}

/// Send an HCI H4 Command packet (`0x01`, `opcode_lo`, `opcode_hi`, `param_len`, `params...`)
/// and wait for the matching HCI Command Complete Event (`0x04`, `0x0E`, `len`, `num_pkts`, `op_lo`, `op_hi`, `status`, ...).
unsafe fn hci_send_cmd(
    opcode: u16,
    params: &[u8],
    resp_buf: &mut [u8],
    timeout_ms: u32,
) -> Result<usize, u32> {
    let mut drain = 128u32;
    while (mmio::read(UART_FR) & FR_RXFE) == 0 && drain > 0 {
        let _ = mmio::read(UART_DR);
        drain -= 1;
    }

    let op_lo = (opcode & 0xFF) as u8;
    let op_hi = ((opcode >> 8) & 0xFF) as u8;

    if !uart0_putc(0x01)
        || !uart0_putc(op_lo)
        || !uart0_putc(op_hi)
        || !uart0_putc(params.len() as u8)
    {
        return Err(0xE001);
    }
    for (idx, &b) in params.iter().enumerate() {
        if !uart0_putc(b) {
            return Err(0xE002);
        }
        // Pace large commands (e.g. 248-byte Write_Local_Name / 241-byte Write_EIR)
        // every 12 bytes so the BCM43430B0 UART FIFO never overflows
        if (idx & 0x0B) == 0x0B {
            let mut guard = 20_000u32;
            while (mmio::read(UART_FR) & FR_BUSY) != 0 && guard > 0 {
                guard -= 1;
                core::hint::spin_loop();
            }
        }
    }

    STATE.with(|s| s.commands_sent += 1);

    // Wait for matching Command Complete Event (0x04, 0x0E, plen, num_pkts, op_lo, op_hi, status, ...)
    for _evt_try in 0..8 {
        let mut sync_tries = 32u32;
        loop {
            let Some(pkt_type) = uart0_getc_timeout(timeout_ms) else {
                return Err(0xE003);
            };
            if pkt_type == 0x04 {
                break;
            }
            sync_tries -= 1;
            if sync_tries == 0 {
                return Err(0xE004);
            }
        }

        let Some(evt_code) = uart0_getc_timeout(30) else {
            return Err(0xE005);
        };
        let Some(plen) = uart0_getc_timeout(30) else {
            return Err(0xE006);
        };

        let mut evt_payload = [0u8; 64];
        let read_len = (plen as usize).min(evt_payload.len());
        for i in 0..(plen as usize) {
            let b = uart0_getc_timeout(30).unwrap_or(0);
            if i < read_len {
                evt_payload[i] = b;
            }
        }

        if evt_code == 0x0E && read_len >= 4 {
            let rx_op = u16::from_le_bytes([evt_payload[1], evt_payload[2]]);
            if rx_op != opcode {
                continue;
            }
            let status = evt_payload[3];
            let ret_len = (read_len - 4).min(resp_buf.len());
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

/// Download `/boot/firmware/BTFW.HCD` (`SYN43430B0.hcd`, `37.4MHz wlbga_iLNA_iTR` RF patch)
/// into the `BCM43430B0` Bluetooth SRAM at high speed (`921,600` baud).
unsafe fn load_bcm43430_hcd_patch(current_baud: &mut u32) -> bool {
    let Some(hcd) = fat32::read_file("BTFW.HCD") else {
        return false;
    };
    if hcd.len() < 64 {
        return false;
    }

    let mut resp = [0u8; 16];

    // Switch UART0 to 921,600 baud (0x000E1000) via Broadcom VSC Update_UART_Baud_Rate (0xFC18)
    // so downloading the 44KB HCD image takes ~450ms instead of 4 seconds!
    let baud_921k: [u8; 6] = [0x00, 0x00, 0x00, 0x10, 0x0E, 0x00];
    if *current_baud != 921_600 && hci_send_cmd(0xFC18, &baud_921k, &mut resp, 80).is_ok() {
        timer::delay_ms(10);
        configure_uart0(3, 16, false);
        *current_baud = 921_600;
    }

    // Enter Minidriver download mode: HCI_VSC_Download_Minidriver (0xFC2E)
    if hci_send_cmd(0xFC2E, &[], &mut resp, 100).is_err() {
        // Fallback to 115,200 baud if 921,600 wasn't accepted
        configure_uart0(26, 3, false);
        *current_baud = 115_200;
        if hci_send_cmd(0xFC2E, &[], &mut resp, 100).is_err() {
            return false;
        }
    }
    timer::delay_ms(50);

    // Stream all HCD records: [opcode_lo, opcode_hi, plen, params...]
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

    // Wait 280ms for HCI_VSC_Launch_RAM (0xFC4E) to reboot the Bluetooth CPU into the patched RAM firmware!
    // Note: Launch_RAM resets the chip's UART back to 115,200 baud!
    timer::delay_ms(280);
    configure_uart0(26, 3, false);
    *current_baud = 115_200;

    if hci_send_cmd(0x0C03, &[], &mut resp, 250).is_ok() {
        return true;
    }
    configure_uart0(26, 3, true);
    hci_send_cmd(0x0C03, &[], &mut resp, 250).is_ok()
}

/// Initialize the onboard `BCM43430B0` Bluetooth 5.0 controller, load its `37.4MHz` RF HCD patch,
/// and start both Classic Bluetooth Discovery + BLE Advertising (`"ZerOS-Pi"`).
pub fn init() -> bool {
    if bsp::is_qemu() {
        STATE.with(|s| {
            s.initialized = true;
            s.hci_up = true;
            s.fw_patched = true;
            s.baud_rate = 115_200;
            s.hci_version = 8;
            s.hci_revision = 0x005C; // build 0092
            s.manufacturer = 0x000F;
            s.lmp_subversion = 0x410C;
            s.bd_addr = [0x98, 0xFE, 0x54, 0x81, 0x9A, 0x88];
            s.ble_advertising = true;
            s.classic_discoverable = true;
            s.commands_sent = 12;
        });
        crate::println!("[OK] Bluetooth HCI (QEMU): BCM43430B0 b0092 BD=98:FE:54:81:9A:88");
        return true;
    }

    uart::set_bt_mode(true);

    let ok = unsafe { init_bt_hw() };
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

    // Step 1: Probe existing warm-boot baud rates (3Mbps, 2Mbps, 921.6kbps, 115.2kbps)
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

    // Step 2: If warm-boot probe didn't answer, hardware-reset BCM43430B0 via GPIO 42 (BT_ON)
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

    // Step 3: Read Local Version Information (0x1001) to check if RF HCD patch (build 0092) is loaded
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

    // Step 4: If running unpatched ROM (build 0000), load `/boot/firmware/BTFW.HCD` (`SYN43430B0.hcd`)
    // so the 37.4MHz RF synthesizer and iLNA_iTR antenna switch are calibrated!
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
    }

    // Step 5: Program & Read BD_ADDR (0xFC01 + 0x1009)
    let vsc_bd = [0x88, 0x9A, 0x81, 0x54, 0xFE, 0x98]; // 98:FE:54:81:9A:88 in LE
    let _ = hci_send_cmd(0xFC01, &vsc_bd, &mut resp, 80);

    let mut bd_addr = [0u8; 6];
    if let Ok(n) = hci_send_cmd(0x1009, &[], &mut resp, 80) {
        if n >= 6 {
            for i in 0..6 {
                bd_addr[i] = resp[5 - i];
            }
        }
    }

    // Step 6: Configure Classic Bluetooth (BR/EDR) for iPhone / Android System Bluetooth Settings!
    // 6a. Disable Broadcom Sleep Mode (0xFC27 = 12 zero bytes) so RF stays active 100% of the time
    let _ = hci_send_cmd(0xFC27, &[0u8; 12], &mut resp, 80);

    // 6b. Enable Secure Simple Pairing (0x0C56 = 0x01) & Simultaneous LE+BR/EDR Host Support (0x0C6D = [1, 0])
    let _ = hci_send_cmd(0x0C56, &[0x01], &mut resp, 80);
    let _ = hci_send_cmd(0x0C6D, &[0x01, 0x00], &mut resp, 80);

    // 6c. Write Local Name (0x0C13, 248 bytes): "ZerOS-Pi"
    let mut local_name = [0u8; 248];
    let name_bytes = b"ZerOS-Pi";
    local_name[..name_bytes.len()].copy_from_slice(name_bytes);
    let _ = hci_send_cmd(0x0C13, &local_name, &mut resp, 150);

    // 6d. Write Extended Inquiry Response (0x0C52, 241 bytes):
    //     Byte 0 = 0x00 (FEC not required), followed by AD structures:
    //     - Complete Local Name "ZerOS-Pi"
    //     - Complete 16-bit Service Class UUIDs: 0x1124 (HID), 0x110B (Audio Sink), 0x1812 (BLE HID)
    let mut eir = [0u8; 241];
    let eir_ad: [u8; 18] = [
        0x09, 0x09, b'Z', b'e', b'r', b'O', b'S', b'-', b'P', b'i', // Complete Local Name
        0x07, 0x03, 0x24, 0x11, 0x0B, 0x11, 0x12, 0x18, // UUIDs: HID (1124), AudioSink (110B), BLE HID (1812)
    ];
    eir[1..1 + eir_ad.len()].copy_from_slice(&eir_ad);
    let _ = hci_send_cmd(0x0C52, &eir, &mut resp, 150);

    // 6e. Write Class of Device (0x0C24, 3 bytes):
    //     iOS Settings -> Bluetooth hides Major Class "Computer (0x01)", so we advertise
    //     Peripheral/Keyboard + Audio Service ([0x40, 0x25, 0x24] = 0x242540) which iOS always lists!
    let _ = hci_send_cmd(0x0C24, &[0x40, 0x25, 0x24], &mut resp, 80);

    // 6f. Set High-Duty-Cycle Inquiry Scan Activity (0x0C1E: Interval=0x00A0 100ms, Window=0x0050 50ms = 50% duty cycle!)
    //     and Page Scan Activity (0x0C1C: Interval=0x0100 160ms, Window=0x0050 50ms)
    let _ = hci_send_cmd(0x0C1E, &[0xA0, 0x00, 0x50, 0x00], &mut resp, 80);
    let _ = hci_send_cmd(0x0C1C, &[0x00, 0x01, 0x50, 0x00], &mut resp, 80);
    let _ = hci_send_cmd(0x0C43, &[0x01], &mut resp, 80); // Interlaced Inquiry Scan
    let _ = hci_send_cmd(0x0C47, &[0x01], &mut resp, 80); // Interlaced Page Scan
    let classic_ok = hci_send_cmd(0x0C1A, &[0x03], &mut resp, 80).is_ok();

    // Step 7: Configure & Enable Bluetooth Low Energy (BLE) Advertising ("ZerOS-Pi")
    //         Include BLE HID Service UUID (0x1812) + Keyboard Appearance (0x03C1) so iOS Settings -> Bluetooth
    //         displays the BLE advertisement in "Other Devices" as well!
    let _ = hci_send_cmd(0x200A, &[0x00], &mut resp, 40);

    let adv_params: [u8; 15] = [
        0x30, 0x00, // Min interval (30ms for fast iPhone discovery)
        0x60, 0x00, // Max interval (60ms)
        0x00,       // ADV_IND (connectable & scannable undirected)
        0x00,       // Public Device Address
        0x00,       // Peer Address Type
        0, 0, 0, 0, 0, 0, // Peer Address
        0x07,       // Channels 37, 38, 39
        0x00,       // Filter policy
    ];
    let _ = hci_send_cmd(0x2006, &adv_params, &mut resp, 80);

    let mut adv_data = [0u8; 32];
    let payload: [u8; 21] = [
        0x02, 0x01, 0x06, // Flags: LE General Discoverable
        0x03, 0x03, 0x12, 0x18, // Complete 16-bit UUID: 0x1812 (Human Interface Device)
        0x03, 0x19, 0xC1, 0x03, // Appearance: 0x03C1 (HID Keyboard)
        0x09, 0x09, b'Z', b'e', b'r', b'O', b'S', b'-', b'P', b'i', // Complete Local Name: "ZerOS-Pi"
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

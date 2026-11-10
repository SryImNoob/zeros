// ZerOS — Synopsys DesignWare Hi-Speed USB 2.0 OTG (`DWC2`) Driver (M8.2)
//
// Drives the BCM2710 USB 2.0 OTG controller at `0x3F98_0000` (compatible with
// both real Raspberry Pi Zero 2 W hardware and QEMU `-M raspi3b`).
//
// Capabilities:
//   1. Powers on the USB HCD hardware domain via VideoCore Mailbox (Device ID = 3)
//   2. Verifies the Synopsys OTG Core ID (`GSNPSID = 0x4F54280A`, "OT" v2.80a)
//   3. Performs DWC2 Core Soft Reset, FIFO allocation, and Host Mode setup
//   4. Enables VBUS Host Port Power (`HPRT0.PrtPwr`) and monitors live port state
//   5. Executes DWC2 DMA Control Transfers with automatic `NAK` retry
//   6. Full Multi-Port USB 2.0 Hub Enumeration (`Class = 0x09`, e.g. Argon POD Base
//      `1a86:8091` 4-port Hub): configures the Hub at Address 1, powers on all
//      downstream Type-A ports (`Port 1..4`), scans ALL connected downstream ports,
//      assigns each downstream device a unique USB address (`2, 3, ...`) so multiple
//      devices don't collide on Address 0, and enumerates their `VID:PID` via
//      USB 2.0 Split Transactions (`HCSPLT0` `SSPLIT` / `CSPLIT`).

#![allow(dead_code)]

use crate::arch::timer;
use crate::drivers::{mailbox, mmio};
use crate::sync::IrqSafeLock;

/// Base physical address of the Synopsys DWC2 USB controller on BCM2710
pub const DWC2_BASE: usize = mmio::PERIPHERAL_BASE + 0x0098_0000;

// ─── Core Global Registers ───
const GOTGCTL: usize = DWC2_BASE + 0x000; // OTG Control and Status
const GAHBCFG: usize = DWC2_BASE + 0x008; // AHB Configuration
const GUSBCFG: usize = DWC2_BASE + 0x00C; // USB Configuration
const GRSTCTL: usize = DWC2_BASE + 0x010; // Reset Register
const GINTSTS: usize = DWC2_BASE + 0x014; // Interrupt Status
const GRXFSIZ: usize = DWC2_BASE + 0x024; // Receive FIFO Size
const GNPTXFSIZ: usize = DWC2_BASE + 0x028; // Non-Periodic Transmit FIFO Size
const GSNPSID: usize = DWC2_BASE + 0x040; // Synopsys ID Register (0x4F54280A)
const HPTXFSIZ: usize = DWC2_BASE + 0x100; // Host Periodic Transmit FIFO Size

// ─── Host Mode Registers ───
const HCFG: usize = DWC2_BASE + 0x400; // Host Configuration
const HFNUM: usize = DWC2_BASE + 0x408; // Host Frame Number / Frame Time Remaining
const HPRT0: usize = DWC2_BASE + 0x440; // Host Port Control and Status

// ─── Host Channel 0 Registers (for Control Transfers) ───
const HCCHAR0: usize = DWC2_BASE + 0x500;
const HCSPLT0: usize = DWC2_BASE + 0x504;
const HCINT0: usize = DWC2_BASE + 0x508;
const HCINTMSK0: usize = DWC2_BASE + 0x50C;
const HCTSIZ0: usize = DWC2_BASE + 0x510;
const HCDMA0: usize = DWC2_BASE + 0x514;

// ─── Bit Definitions ───
const GAHBCFG_GLBL_INTR_EN: u32 = 1 << 0;
const GAHBCFG_HBST_INCR4: u32 = 0b011 << 1;
const GAHBCFG_DMA_EN: u32 = 1 << 5;

const GUSBCFG_FORCE_HOST: u32 = 1 << 29;

const GRSTCTL_CSFT_RST: u32 = 1 << 0;
const GRSTCTL_RXF_FLSH: u32 = 1 << 4;
const GRSTCTL_TXF_FLSH: u32 = 1 << 5;
const GRSTCTL_AHB_IDLE: u32 = 1 << 31;

const HPRT0_CONN_STS: u32 = 1 << 0;
const HPRT0_CONN_DET: u32 = 1 << 1;
const HPRT0_ENA: u32 = 1 << 2;
const HPRT0_EN_CHNG: u32 = 1 << 3;
const HPRT0_OVR_CURR_CHNG: u32 = 1 << 5;
const HPRT0_RST: u32 = 1 << 8;
const HPRT0_PWR: u32 = 1 << 12;
const HPRT0_SPD_SHIFT: u32 = 17;
const HPRT0_SPD_MASK: u32 = 0b11 << 17;

/// Mask of Write-1-to-Clear (W1C) bits in `HPRT0` that must be masked to 0
/// during read-modify-write so we don't accidentally disable the port!
const HPRT0_W1C_MASK: u32 = HPRT0_CONN_DET | HPRT0_ENA | HPRT0_EN_CHNG | HPRT0_OVR_CURR_CHNG;

// ─── HCINT0 Interrupt Bits ───
const HCINT_XFER_COMPL: u32 = 1 << 0;
const HCINT_CH_HLTD: u32 = 1 << 1;
const HCINT_AHB_ERR: u32 = 1 << 2;
const HCINT_STALL: u32 = 1 << 3;
const HCINT_NAK: u32 = 1 << 4;
const HCINT_ACK: u32 = 1 << 5;
const HCINT_NYET: u32 = 1 << 6;
const HCINT_XACT_ERR: u32 = 1 << 7;

// ─── PID Constants for HCTSIZ0[30:29] ───
const PID_DATA0: u32 = 0;
const PID_DATA1: u32 = 2;
const PID_SETUP: u32 = 3;

/// Negotiated USB port speed
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UsbSpeed {
    Disconnected,
    HighSpeed, // 480 Mbps
    FullSpeed, // 12 Mbps
    LowSpeed,  // 1.5 Mbps
}

impl UsbSpeed {
    pub fn as_str(self) -> &'static str {
        match self {
            UsbSpeed::Disconnected => "No Device",
            UsbSpeed::HighSpeed => "High (480M)",
            UsbSpeed::FullSpeed => "Full (12M)",
            UsbSpeed::LowSpeed => "Low (1.5M)",
        }
    }
}

/// Enumerated USB Device Descriptor summary
#[derive(Debug, Clone, Copy)]
pub struct UsbDeviceInfo {
    pub vid: u16,
    pub pid: u16,
    pub usb_bcd: u16,
    pub dev_class: u8,
    pub max_packet_size: u8,
}

/// Information about a single downstream port on an attached USB Hub
#[derive(Debug, Clone, Copy)]
pub struct DownstreamPortInfo {
    /// 1-based Hub port index (`1..=4`)
    pub port: u8,
    /// Assigned USB bus address (`2..=5`)
    pub usb_addr: u8,
    /// Raw `wPortStatus` from `GET_PORT_STATUS`
    pub status: u16,
    /// Negotiated speed on this Hub port
    pub speed: UsbSpeed,
    /// Enumerated Device Descriptor (`VID:PID`, Class)
    pub device: Option<UsbDeviceInfo>,
}

/// Live status of the DWC2 USB 2.0 OTG Controller & attached Hub / Downstream devices
#[derive(Debug, Clone, Copy)]
pub struct UsbStatus {
    pub initialized: bool,
    pub powered: bool,
    pub snps_id: u32,
    pub host_mode: bool,
    pub port_powered: bool,
    pub port_connected: bool,
    pub port_enabled: bool,
    pub speed: UsbSpeed,
    pub hprt0_raw: u32,
    /// Root Port device (on Argon POD Base: `1a86:8091` 4-Port USB 2.0 Hub)
    pub device: Option<UsbDeviceInfo>,
    /// Number of downstream ports if Root Port device is a USB Hub (`Class = 0x09`)
    pub hub_ports: u8,
    /// Number of connected downstream devices found across all Hub ports (`0..=4`)
    pub downstream_count: u8,
    /// Enumerated downstream devices plugged into the Hub's ports
    pub downstream_devs: [Option<DownstreamPortInfo>; 4],
    pub scans_run: u32,
}

impl UsbStatus {
    const fn empty() -> Self {
        Self {
            initialized: false,
            powered: false,
            snps_id: 0,
            host_mode: false,
            port_powered: false,
            port_connected: false,
            port_enabled: false,
            speed: UsbSpeed::Disconnected,
            hprt0_raw: 0,
            device: None,
            hub_ports: 0,
            downstream_count: 0,
            downstream_devs: [None; 4],
            scans_run: 0,
        }
    }
}

static STATE: IrqSafeLock<UsbStatus> = IrqSafeLock::new(UsbStatus::empty());

/// 64-byte aligned DMA buffer for USB Control Transfers (mapped Non-Cacheable by MMU).
#[repr(C, align(64))]
struct UsbDmaBuffer {
    bytes: [u8; 64],
}

static mut DMA_BUF: UsbDmaBuffer = UsbDmaBuffer { bytes: [0; 64] };

/// Convert a physical RAM address into a BCM2710 VideoCore uncached bus address (`0xC000_0000`).
#[inline(always)]
fn phys_to_bus(addr: usize) -> u32 {
    0xC000_0000 | ((addr as u32) & 0x3FFF_FFFF)
}

/// Initialize the BCM2710 Synopsys DWC2 USB 2.0 OTG Controller in Host Mode.
pub fn init() -> bool {
    let powered = mailbox::set_usb_power(true);

    unsafe {
        let snps_id = mmio::read(GSNPSID);
        if (snps_id & 0xFFFF_0000) != 0x4F54_0000 {
            crate::println!(
                "[WARN] USB DWC2: unexpected GSNPSID=0x{:08x} (powered={})",
                snps_id,
                powered
            );
            return false;
        }

        // Soft reset core
        let mut timeout = 100_000u32;
        while (mmio::read(GRSTCTL) & GRSTCTL_AHB_IDLE) == 0 && timeout > 0 {
            timeout -= 1;
            core::hint::spin_loop();
        }
        mmio::write(GRSTCTL, GRSTCTL_CSFT_RST);
        timeout = 100_000;
        while (mmio::read(GRSTCTL) & GRSTCTL_CSFT_RST) != 0 && timeout > 0 {
            timeout -= 1;
            core::hint::spin_loop();
        }
        timer::delay_ms(5);

        // Force Host Mode
        let usbcfg = mmio::read(GUSBCFG);
        mmio::write(GUSBCFG, usbcfg | GUSBCFG_FORCE_HOST);
        timer::delay_ms(25);

        // Configure FIFOs & flush
        mmio::write(GRXFSIZ, 1024);
        mmio::write(GNPTXFSIZ, (1024 << 16) | 1024);
        mmio::write(HPTXFSIZ, (512 << 16) | 2048);
        mmio::write(GRSTCTL, GRSTCTL_TXF_FLSH | GRSTCTL_RXF_FLSH | (0x10 << 6));
        timeout = 10_000;
        while (mmio::read(GRSTCTL) & (GRSTCTL_TXF_FLSH | GRSTCTL_RXF_FLSH)) != 0 && timeout > 0 {
            timeout -= 1;
            core::hint::spin_loop();
        }

        // Enable Internal DMA engine + Global Interrupts
        mmio::write(
            GAHBCFG,
            GAHBCFG_DMA_EN | GAHBCFG_HBST_INCR4 | GAHBCFG_GLBL_INTR_EN,
        );

        // Power on Host Port VBUS (PrtPwr = 1)
        let hprt = mmio::read(HPRT0) & !HPRT0_W1C_MASK;
        mmio::write(HPRT0, hprt | HPRT0_PWR);
        timer::delay_ms(10);

        STATE.with(|s| {
            s.initialized = true;
            s.powered = powered;
            s.snps_id = snps_id;
        });
    }

    let st = refresh_status();
    crate::println!(
        "[OK] USB DWC2: Core ID=0x{:08x} (v{:x}.{:02x}a), HostMode={}, HPRT0=0x{:08x}",
        st.snps_id,
        (st.snps_id >> 12) & 0xF,
        (st.snps_id >> 4) & 0xFF,
        st.host_mode,
        st.hprt0_raw
    );
    true
}

/// Read live hardware registers (`GINTSTS`, `HPRT0`) and update cached `UsbStatus`.
pub fn refresh_status() -> UsbStatus {
    STATE.with(|s| {
        if !s.initialized {
            return *s;
        }
        unsafe {
            let gintsts = mmio::read(GINTSTS);
            let hprt = mmio::read(HPRT0);

            s.host_mode = (gintsts & 1) != 0;
            s.hprt0_raw = hprt;
            s.port_powered = (hprt & HPRT0_PWR) != 0;
            s.port_connected = (hprt & HPRT0_CONN_STS) != 0;
            s.port_enabled = (hprt & HPRT0_ENA) != 0;

            s.speed = if !s.port_connected {
                s.device = None;
                s.hub_ports = 0;
                s.downstream_count = 0;
                s.downstream_devs = [None; 4];
                UsbSpeed::Disconnected
            } else {
                match (hprt & HPRT0_SPD_MASK) >> HPRT0_SPD_SHIFT {
                    0 => UsbSpeed::HighSpeed,
                    1 => UsbSpeed::FullSpeed,
                    2 => UsbSpeed::LowSpeed,
                    _ => UsbSpeed::FullSpeed,
                }
            };
        }
        *s
    })
}

/// Halt Channel 0 cleanly if it is still enabled.
unsafe fn halt_channel0(hcchar_base: u32) {
    if (mmio::read(HCCHAR0) & (1 << 31)) != 0 {
        mmio::write(HCCHAR0, hcchar_base | (1 << 31) | (1 << 30));
        let mut guard = 10_000u32;
        while (mmio::read(HCCHAR0) & (1 << 31)) != 0 && guard > 0 {
            guard -= 1;
            core::hint::spin_loop();
        }
    }
    mmio::write(HCSPLT0, 0);
}

/// Execute a single USB packet transaction (`SETUP`, `DATA IN/OUT`, or `STATUS IN/OUT`)
/// on Channel 0 in DMA mode, with automatic `NAK` retries and optional Split Transaction
/// (`split = Some((hub_addr, hub_port))`) for Full/Low-Speed devices behind a High-Speed Hub.
unsafe fn dwc2_xfer_packet(
    dev_addr: u8,
    is_in: bool,
    pid: u32,
    low_speed: bool,
    mps: u32,
    buf: &mut [u8],
    len: usize,
    split: Option<(u8, u8)>,
) -> Result<usize, u32> {
    let low_spd_bit = if low_speed { 1 << 17 } else { 0 };
    let ep_dir_bit = if is_in { 1 << 15 } else { 0 };
    let mc_1 = 1 << 20;
    let hcchar_base =
        ((dev_addr as u32 & 0x7F) << 22) | mc_1 | ep_dir_bit | low_spd_bit | (mps & 0x7FF);
    let dma_addr = phys_to_bus(core::ptr::addr_of!(DMA_BUF) as usize);

    for _retry in 0..35 {
        if !is_in && len > 0 {
            DMA_BUF.bytes[..len].copy_from_slice(&buf[..len]);
        }
        core::arch::asm!("dsb sy", options(nostack, preserves_flags));

        if let Some((hub_addr, hub_port)) = split {
            // ─── Phase 1: Start-Split (SSPLIT) ───
            let ssplit = (1 << 31) | ((hub_addr as u32 & 0x7F) << 7) | (hub_port as u32 & 0x7F);
            mmio::write(HCSPLT0, ssplit);
            mmio::write(HCINT0, 0x3FFF);
            mmio::write(HCINTMSK0, 0x3FFF);
            let ssplit_size = if is_in { 0 } else { len as u32 };
            mmio::write(HCTSIZ0, ((pid & 3) << 29) | (1 << 19) | ssplit_size);
            mmio::write(HCDMA0, dma_addr);

            let odd_frm = ((mmio::read(HFNUM) + 1) & 1) << 29;
            mmio::write(HCCHAR0, (1 << 31) | odd_frm | hcchar_base);

            let mut timeout = 80_000u32;
            while (mmio::read(HCINT0) & HCINT_CH_HLTD) == 0 && timeout > 0 {
                timeout -= 1;
                core::hint::spin_loop();
            }
            let ss_int = mmio::read(HCINT0);
            if timeout == 0 {
                halt_channel0(hcchar_base);
            }
            if (ss_int & (HCINT_STALL | HCINT_AHB_ERR | HCINT_XACT_ERR)) != 0 {
                mmio::write(HCSPLT0, 0);
                return Err(ss_int);
            }
            if (ss_int & HCINT_NAK) != 0 {
                timer::delay_ms(1);
                continue;
            }

            // Give the Hub Transaction Translator time to start FS/LS packet on downstream port
            timer::delay_ms(if low_speed { 2 } else { 1 });

            // ─── Phase 2: Complete-Split (CSPLIT) ───
            let csplit = ssplit | (1 << 16);
            let mut got_nak = false;
            for _cs_try in 0..18 {
                mmio::write(HCSPLT0, csplit);
                mmio::write(HCINT0, 0x3FFF);
                mmio::write(HCINTMSK0, 0x3FFF);
                let csplit_size = if is_in { len as u32 } else { 0 };
                mmio::write(HCTSIZ0, ((pid & 3) << 29) | (1 << 19) | csplit_size);
                mmio::write(HCDMA0, dma_addr);

                let odd_frm = ((mmio::read(HFNUM) + 1) & 1) << 29;
                mmio::write(HCCHAR0, (1 << 31) | odd_frm | hcchar_base);

                timeout = 80_000;
                while (mmio::read(HCINT0) & HCINT_CH_HLTD) == 0 && timeout > 0 {
                    timeout -= 1;
                    core::hint::spin_loop();
                }
                let cs_int = mmio::read(HCINT0);
                let hctsiz = mmio::read(HCTSIZ0);
                if timeout == 0 {
                    halt_channel0(hcchar_base);
                }
                core::arch::asm!("dsb sy", options(nostack, preserves_flags));

                if (cs_int & (HCINT_XFER_COMPL | HCINT_ACK)) != 0
                    && (cs_int & (HCINT_NYET | HCINT_NAK | HCINT_STALL | HCINT_XACT_ERR)) == 0
                {
                    mmio::write(HCSPLT0, 0);
                    if is_in && len > 0 {
                        let rem = (hctsiz & 0x7FFFF) as usize;
                        let rx = len.saturating_sub(rem.min(len));
                        let actual = if rx > 0 { rx } else { len };
                        buf[..actual].copy_from_slice(&DMA_BUF.bytes[..actual]);
                        return Ok(actual);
                    }
                    return Ok(len);
                }
                if (cs_int & HCINT_NYET) != 0 {
                    timer::delay_ms(1);
                    continue;
                }
                if (cs_int & HCINT_NAK) != 0 {
                    got_nak = true;
                    break;
                }
                if (cs_int & (HCINT_STALL | HCINT_AHB_ERR | HCINT_XACT_ERR)) != 0 {
                    mmio::write(HCSPLT0, 0);
                    return Err(cs_int);
                }
                timer::delay_ms(1);
            }
            mmio::write(HCSPLT0, 0);
            if got_nak {
                timer::delay_ms(1);
                continue;
            }
            return Err(0xDEAD_C500);
        } else {
            // ─── Non-Split Direct Transaction ───
            mmio::write(HCSPLT0, 0);
            mmio::write(HCINT0, 0x3FFF);
            mmio::write(HCINTMSK0, 0x3FFF);
            mmio::write(HCTSIZ0, ((pid & 3) << 29) | (1 << 19) | (len as u32));
            mmio::write(HCDMA0, dma_addr);
            mmio::write(HCCHAR0, (1 << 31) | hcchar_base);

            let mut timeout = 100_000u32;
            while (mmio::read(HCINT0) & HCINT_CH_HLTD) == 0 && timeout > 0 {
                timeout -= 1;
                core::hint::spin_loop();
            }
            let hcint = mmio::read(HCINT0);
            let hctsiz = mmio::read(HCTSIZ0);
            if timeout == 0 {
                halt_channel0(hcchar_base);
            }
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));

            if (hcint & HCINT_XFER_COMPL) != 0 {
                if is_in && len > 0 {
                    let rem = (hctsiz & 0x7FFFF) as usize;
                    let rx = len.saturating_sub(rem.min(len));
                    buf[..rx].copy_from_slice(&DMA_BUF.bytes[..rx]);
                    return Ok(rx);
                }
                return Ok(len);
            }
            if (hcint & (HCINT_STALL | HCINT_AHB_ERR | HCINT_XACT_ERR)) != 0 {
                return Err(hcint);
            }
            if (hcint & (HCINT_NAK | HCINT_NYET)) != 0 {
                timer::delay_ms(1);
                continue;
            }
            timer::delay_ms(1);
        }
    }

    Err(mmio::read(HCINT0))
}

/// Execute a complete 3-stage USB Control Transfer (`SETUP` -> optional `DATA IN` -> `STATUS`).
unsafe fn usb_control_transfer(
    dev_addr: u8,
    low_speed: bool,
    mps: u32,
    setup: [u8; 8],
    data_in_buf: &mut [u8],
    split: Option<(u8, u8)>,
) -> Result<usize, u32> {
    // 1. SETUP Stage (8 bytes OUT, PID_SETUP)
    let mut setup_copy = setup;
    dwc2_xfer_packet(
        dev_addr,
        false,
        PID_SETUP,
        low_speed,
        mps,
        &mut setup_copy,
        8,
        split,
    )?;

    let req_len = u16::from_le_bytes([setup[6], setup[7]]) as usize;
    let is_dev_to_host = (setup[0] & 0x80) != 0;

    // 2. DATA IN Stage (if requested)
    let mut total_rx = 0usize;
    if req_len > 0 && is_dev_to_host {
        let target_len = req_len.min(data_in_buf.len());
        let mut pid = PID_DATA1;
        while total_rx < target_len {
            let chunk_req = (target_len - total_rx).min(mps as usize);
            let mut pkt_buf = [0u8; 64];
            let got = dwc2_xfer_packet(
                dev_addr,
                true,
                pid,
                low_speed,
                mps,
                &mut pkt_buf,
                chunk_req,
                split,
            )?;
            data_in_buf[total_rx..total_rx + got].copy_from_slice(&pkt_buf[..got]);
            total_rx += got;
            pid = if pid == PID_DATA1 {
                PID_DATA0
            } else {
                PID_DATA1
            };
            if got < (mps as usize) {
                break;
            }
        }
    }

    // 3. STATUS Stage (0-byte ZLP with DATA1 in opposite direction)
    let status_in = !(req_len > 0 && is_dev_to_host);
    let mut zlp = [0u8; 0];
    let _ = dwc2_xfer_packet(
        dev_addr,
        status_in,
        PID_DATA1,
        low_speed,
        mps,
        &mut zlp,
        0,
        split,
    );

    Ok(total_rx)
}

/// Read and parse an 18-byte USB Device Descriptor from `dev_addr`.
unsafe fn read_device_descriptor(
    dev_addr: u8,
    speed: UsbSpeed,
    split: Option<(u8, u8)>,
) -> Result<UsbDeviceInfo, u32> {
    let low_speed = speed == UsbSpeed::LowSpeed;
    let mps = if low_speed { 8 } else { 64 };
    // GET_DESCRIPTOR (Device = 0x0100, wLength = 18)
    let setup = [0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00];
    let mut desc = [0u8; 18];
    let rx_len = usb_control_transfer(dev_addr, low_speed, mps, setup, &mut desc, split)?;

    if rx_len >= 12 && desc[1] == 0x01 {
        Ok(UsbDeviceInfo {
            usb_bcd: u16::from_le_bytes([desc[2], desc[3]]),
            dev_class: desc[4],
            max_packet_size: desc[7],
            vid: u16::from_le_bytes([desc[8], desc[9]]),
            pid: u16::from_le_bytes([desc[10], desc[11]]),
        })
    } else {
        Err(0xBAD0_0000 | (rx_len as u32))
    }
}

/// Perform a USB Port Reset on `HPRT0`, enumerate the Root Port device, and if the Root Port
/// device is a USB 2.0 Hub (`Class = 0x09`, such as the Argon POD's `1a86:8091` 4-port Hub),
/// automatically configure the Hub, power on its downstream Type-A ports, scan ALL attached
/// downstream devices (`Port 1..4`), assign each a unique USB address (`2, 3, ...`), and
/// enumerate their `VID:PID`!
pub fn scan_port() -> UsbStatus {
    let mut st = refresh_status();
    if !st.initialized {
        return st;
    }

    let scan_id = STATE.with(|s| {
        s.scans_run += 1;
        s.hub_ports = 0;
        s.downstream_count = 0;
        s.downstream_devs = [None; 4];
        s.scans_run
    });

    if !st.port_connected {
        crate::println!(
            "[USB] Scan #{}: no device on root port (HPRT0=0x{:08x})",
            scan_id,
            st.hprt0_raw
        );
        return refresh_status();
    }

    unsafe {
        // 1. Assert Root Port Reset (PrtRst = 1) for 50ms
        let hprt = mmio::read(HPRT0) & !HPRT0_W1C_MASK;
        mmio::write(HPRT0, hprt | HPRT0_PWR | HPRT0_RST);
        timer::delay_ms(50);

        // 2. Deassert Root Port Reset (PrtRst = 0) and wait 35ms for recovery
        let hprt = mmio::read(HPRT0) & !(HPRT0_W1C_MASK | HPRT0_RST);
        mmio::write(HPRT0, hprt | HPRT0_PWR);
        timer::delay_ms(35);
    }

    st = refresh_status();
    if !st.port_enabled {
        crate::println!(
            "[USB] Scan #{}: root reset done, Spd={} HPRT0=0x{:08x}",
            scan_id,
            st.speed.as_str(),
            st.hprt0_raw
        );
        return st;
    }

    unsafe {
        // Ensure DMA is active
        mmio::write(
            GAHBCFG,
            GAHBCFG_DMA_EN | GAHBCFG_HBST_INCR4 | GAHBCFG_GLBL_INTR_EN,
        );

        if st.speed == UsbSpeed::LowSpeed {
            mmio::write(HCFG, 2);
        } else if st.speed == UsbSpeed::FullSpeed {
            mmio::write(HCFG, 1);
        }

        // 3. Read Root Port Device Descriptor at Address 0
        let root_dev = match read_device_descriptor(0, st.speed, None) {
            Ok(info) => {
                STATE.with(|s| s.device = Some(info));
                info
            }
            Err(err) => {
                crate::println!(
                    "[USB] Scan #{}: Root {} Ena=1 Err=0x{:x}",
                    scan_id,
                    st.speed.as_str(),
                    err
                );
                return refresh_status();
            }
        };

        // If Root Port device is NOT a USB Hub (Class != 0x09), we're done!
        if root_dev.dev_class != 0x09 {
            crate::println!(
                "[OK] USB #{}: Root VID:PID={:04x}:{:04x} Cls={:02x} ({})",
                scan_id,
                root_dev.vid,
                root_dev.pid,
                root_dev.dev_class,
                st.speed.as_str()
            );
            return refresh_status();
        }

        // ─── 4. Root Device IS a USB 2.0 Hub (e.g., Argon POD 1a86:8091) ───
        let hub_mps = (root_dev.max_packet_size as u32).max(8);

        // 4a. Assign Address 1 to the Hub: SET_ADDRESS(1)
        let set_addr_1 = [0x00, 0x05, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00];
        let mut empty = [0u8; 0];
        if usb_control_transfer(0, false, hub_mps, set_addr_1, &mut empty, None).is_err() {
            crate::println!(
                "[USB] #{}: Hub {:04x}:{:04x} SET_ADDRESS(1) failed",
                scan_id,
                root_dev.vid,
                root_dev.pid
            );
            return refresh_status();
        }
        timer::delay_ms(5);

        // 4b. Configure the Hub using Single-TT or Multi-TT default config: SET_CONFIGURATION(1)
        let set_cfg = [0x00, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00];
        let _ = usb_control_transfer(1, false, hub_mps, set_cfg, &mut empty, None);
        timer::delay_ms(5);

        // If Hub supports TT-per-port (bDeviceProtocol == 2, like 1a86:8091), select Interface 0 AltSetting 1
        // so each downstream port has its own independent Transaction Translator!
        let set_iface_multi_tt = [0x01, 0x0B, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00];
        let _ = usb_control_transfer(1, false, hub_mps, set_iface_multi_tt, &mut empty, None);
        timer::delay_ms(2);

        // 4c. Read Hub Descriptor (`0x2900`) to get number of downstream ports (default 4)
        let get_hub_desc = [0xA0, 0x06, 0x00, 0x29, 0x00, 0x00, 0x09, 0x00];
        let mut hub_desc = [0u8; 9];
        let num_ports = if let Ok(n) =
            usb_control_transfer(1, false, hub_mps, get_hub_desc, &mut hub_desc, None)
        {
            if n >= 3 && (1..=8).contains(&hub_desc[2]) {
                hub_desc[2]
            } else {
                4
            }
        } else {
            4
        };
        STATE.with(|s| s.hub_ports = num_ports);

        // 4d. Power on all downstream ports: SET_PORT_FEATURE(PORT_POWER = 8, port = 1..=num_ports)
        for port in 1..=num_ports {
            let set_pwr = [0x23, 0x03, 0x08, 0x00, port, 0x00, 0x00, 0x00];
            let _ = usb_control_transfer(1, false, hub_mps, set_pwr, &mut empty, None);
        }
        timer::delay_ms(80);

        // 4e. Scan ALL downstream ports 1..=num_ports (do NOT break after the first!)
        let mut dev_count = 0usize;
        for port in 1..=num_ports {
            if dev_count >= 4 {
                break;
            }
            let get_port_sts = [0xA3, 0x00, 0x00, 0x00, port, 0x00, 0x04, 0x00];
            let mut sts_buf = [0u8; 4];
            let Ok(n) = usb_control_transfer(1, false, hub_mps, get_port_sts, &mut sts_buf, None)
            else {
                continue;
            };
            if n < 2 {
                continue;
            }
            let mut w_status = u16::from_le_bytes([sts_buf[0], sts_buf[1]]);
            if (w_status & 0x0001) == 0 {
                // Port has no device connected
                continue;
            }

            // ─── Downstream Device Detected on Hub Port `port`! ───
            // Reset this Hub downstream port: SET_PORT_FEATURE(PORT_RESET = 4, port)
            let set_rst = [0x23, 0x03, 0x04, 0x00, port, 0x00, 0x00, 0x00];
            let _ = usb_control_transfer(1, false, hub_mps, set_rst, &mut empty, None);
            timer::delay_ms(60);

            // Clear C_PORT_RESET (20 = 0x14) and read updated wPortStatus
            let clr_rst = [0x23, 0x01, 0x14, 0x00, port, 0x00, 0x00, 0x00];
            let _ = usb_control_transfer(1, false, hub_mps, clr_rst, &mut empty, None);
            timer::delay_ms(25);

            if let Ok(n2) =
                usb_control_transfer(1, false, hub_mps, get_port_sts, &mut sts_buf, None)
            {
                if n2 >= 2 {
                    w_status = u16::from_le_bytes([sts_buf[0], sts_buf[1]]);
                }
            }

            let ds_speed = if (w_status & (1 << 9)) != 0 {
                UsbSpeed::LowSpeed
            } else if (w_status & (1 << 10)) != 0 {
                UsbSpeed::HighSpeed
            } else {
                UsbSpeed::FullSpeed
            };

            let split = if st.speed == UsbSpeed::HighSpeed && ds_speed != UsbSpeed::HighSpeed {
                Some((1u8, port))
            } else {
                None
            };

            let new_addr = (2 + dev_count) as u8;
            let ds_dev = read_device_descriptor(0, ds_speed, split).ok();

            // Crucial for multi-port enumeration:
            // Assign unique USB address (`new_addr = 2, 3, ...`) to this device while it's at Address 0
            // so when we reset the NEXT Hub port, the next device at Address 0 won't collide with this one!
            let ds_low_spd = ds_speed == UsbSpeed::LowSpeed;
            let ds_mps = ds_dev
                .map(|d| (d.max_packet_size as u32).max(8))
                .unwrap_or(if ds_low_spd { 8 } else { 64 });
            let set_ds_addr = [0x00, 0x05, new_addr, 0x00, 0x00, 0x00, 0x00, 0x00];
            let _ = usb_control_transfer(0, ds_low_spd, ds_mps, set_ds_addr, &mut empty, split);
            timer::delay_ms(5);

            let port_info = DownstreamPortInfo {
                port,
                usb_addr: new_addr,
                status: w_status,
                speed: ds_speed,
                device: ds_dev,
            };

            STATE.with(|s| {
                s.downstream_devs[dev_count] = Some(port_info);
                s.downstream_count = (dev_count + 1) as u8;
            });
            dev_count += 1;

            if let Some(d) = ds_dev {
                crate::println!(
                    "[OK] USB #{}: P{}[A{}]->{:04x}:{:04x} Cls={:02x} ({})",
                    scan_id,
                    port,
                    new_addr,
                    d.vid,
                    d.pid,
                    d.dev_class,
                    ds_speed.as_str()
                );
            } else {
                crate::println!(
                    "[OK] USB #{}: P{}[A{}]->Connected ({}, St=0x{:04x})",
                    scan_id,
                    port,
                    new_addr,
                    ds_speed.as_str(),
                    w_status
                );
            }
        }

        if dev_count == 0 {
            crate::println!(
                "[USB] #{}: Hub {:04x}:{:04x} ({}p) -> Type-A Empty",
                scan_id,
                root_dev.vid,
                root_dev.pid,
                num_ports
            );
        }
    }

    refresh_status()
}

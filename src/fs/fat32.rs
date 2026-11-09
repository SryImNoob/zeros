// ZerOS — MBR & FAT32 Filesystem Driver (M6.2)
//
// Supports:
//   - MBR partition table parsing (4 primary partitions) + superfloppy fallback
//   - FAT32 BIOS Parameter Block (BPB) parsing
//   - Root directory listing (8.3 short filenames)
//   - Reading files across cluster chains into Vec<u8>
//   - In-place updating of small configuration files (e.g. autoboot.txt for tryboot)

#![allow(dead_code)]

use alloc::string::String;
use alloc::vec::Vec;
use crate::drivers::block::sd::{self, SECTOR_SIZE};

/// MBR Partition entry
#[derive(Debug, Clone, Copy)]
pub struct PartitionInfo {
    pub index: u8,
    pub part_type: u8,
    pub start_lba: u32,
    pub sector_count: u32,
}

/// Directory entry metadata
#[derive(Debug, Clone)]
pub struct DirEntry {
    pub name: String,
    pub is_dir: bool,
    pub size: u32,
    pub first_cluster: u32,
    pub dir_sector: u32,
    pub dir_offset: usize,
}

/// Mounted FAT32 volume state
pub struct Fat32Volume {
    pub part_lba: u32,
    pub bytes_per_sector: u16,
    pub sectors_per_cluster: u8,
    pub reserved_sectors: u16,
    pub num_fats: u8,
    pub sectors_per_fat: u32,
    pub root_cluster: u32,
    pub fat_start_lba: u32,
    pub data_start_lba: u32,
    pub label: [u8; 11],
}

static mut VOLUME: Option<Fat32Volume> = None;
static mut PARTITIONS: [Option<PartitionInfo>; 4] = [None; 4];

/// Initialize and mount the first FAT32 partition on the SD card.
pub fn init() -> bool {
    if !sd::is_initialized() {
        return false;
    }

    let mut sector0 = [0u8; SECTOR_SIZE];
    if !sd::read_sector(0, &mut sector0) {
        crate::println!("[ERR] FAT32: failed to read sector 0");
        return false;
    }

    // Check boot signature 0x55AA
    if sector0[510] != 0x55 || sector0[511] != 0xAA {
        crate::println!("[WARN] FAT32: invalid boot signature on sector 0");
        return false;
    }

    // Check if sector 0 is already a FAT32 VBR (superfloppy)
    // In a FAT32 VBR, offset 0x52..0x5A is "FAT32   "
    if &sector0[0x52..0x57] == b"FAT32" {
        return mount_partition(0);
    }

    // Parse MBR partition table at 0x1BE (4 × 16-byte entries)
    let mut first_fat_lba = None;
    for i in 0..4 {
        let offset = 0x1BE + i * 16;
        let ptype = sector0[offset + 4];
        let start_lba = u32::from_le_bytes([
            sector0[offset + 8],
            sector0[offset + 9],
            sector0[offset + 10],
            sector0[offset + 11],
        ]);
        let count = u32::from_le_bytes([
            sector0[offset + 12],
            sector0[offset + 13],
            sector0[offset + 14],
            sector0[offset + 15],
        ]);

        if ptype != 0 && count > 0 {
            let info = PartitionInfo {
                index: (i + 1) as u8,
                part_type: ptype,
                start_lba,
                sector_count: count,
            };
            unsafe { PARTITIONS[i] = Some(info); }
            crate::println!(
                "[OK] MBR Partition #{}: type=0x{:02X}, start_lba={}, size={}MB",
                i + 1,
                ptype,
                start_lba,
                count / 2048
            );

            // FAT32 (0x0B / 0x0C) or FAT16/E (0x06 / 0x0E)
            if first_fat_lba.is_none()
                && (ptype == 0x0B || ptype == 0x0C || ptype == 0x06 || ptype == 0x0E)
            {
                first_fat_lba = Some(start_lba);
            }
        }
    }

    if let Some(lba) = first_fat_lba {
        mount_partition(lba)
    } else {
        crate::println!("[WARN] FAT32: no FAT partition found in MBR");
        false
    }
}

/// Mount a FAT32 volume starting at `part_lba`.
pub fn mount_partition(part_lba: u32) -> bool {
    let mut vbr = [0u8; SECTOR_SIZE];
    if !sd::read_sector(part_lba, &mut vbr) {
        crate::println!("[ERR] FAT32: failed to read VBR at LBA {}", part_lba);
        return false;
    }

    let bytes_per_sector = u16::from_le_bytes([vbr[0x0B], vbr[0x0C]]);
    let sectors_per_cluster = vbr[0x0D];
    let reserved_sectors = u16::from_le_bytes([vbr[0x0E], vbr[0x0F]]);
    let num_fats = vbr[0x10];
    let sectors_per_fat_16 = u16::from_le_bytes([vbr[0x16], vbr[0x17]]);
    let sectors_per_fat_32 = u32::from_le_bytes([vbr[0x24], vbr[0x25], vbr[0x26], vbr[0x27]]);
    let sectors_per_fat = if sectors_per_fat_32 != 0 {
        sectors_per_fat_32
    } else {
        sectors_per_fat_16 as u32
    };
    let root_cluster = u32::from_le_bytes([vbr[0x2C], vbr[0x2D], vbr[0x2E], vbr[0x2F]]);

    if bytes_per_sector != 512 || sectors_per_cluster == 0 || num_fats == 0 {
        crate::println!("[ERR] FAT32: unsupported BPB parameters");
        return false;
    }

    let fat_start_lba = part_lba + (reserved_sectors as u32);
    let data_start_lba = fat_start_lba + (num_fats as u32) * sectors_per_fat;

    let mut label = [b' '; 11];
    label.copy_from_slice(&vbr[0x47..0x52]);

    let label_str = core::str::from_utf8(&label).unwrap_or("UNKNOWN").trim();

    crate::println!(
        "[OK] FAT32 mounted: label='{}', LBA={}, cluster={}B, data_lba={}",
        label_str,
        part_lba,
        (sectors_per_cluster as u32) * 512,
        data_start_lba
    );

    unsafe {
        VOLUME = Some(Fat32Volume {
            part_lba,
            bytes_per_sector,
            sectors_per_cluster,
            reserved_sectors,
            num_fats,
            sectors_per_fat,
            root_cluster,
            fat_start_lba,
            data_start_lba,
            label,
        });
    }

    true
}

/// Check if a FAT32 volume is currently mounted.
pub fn is_mounted() -> bool {
    unsafe { (*core::ptr::addr_of!(VOLUME)).is_some() }
}

/// Get discovered MBR partitions.
pub fn partitions() -> [Option<PartitionInfo>; 4] {
    unsafe { *core::ptr::addr_of!(PARTITIONS) }
}

impl Fat32Volume {
    /// Convert a cluster number (>= 2) to its first LBA sector.
    fn cluster_to_lba(&self, cluster: u32) -> u32 {
        self.data_start_lba + (cluster - 2) * (self.sectors_per_cluster as u32)
    }

    /// Read the FAT table entry for `cluster` to find the next cluster in the chain.
    fn next_cluster(&self, cluster: u32) -> Option<u32> {
        let fat_offset = cluster * 4;
        let fat_sector = self.fat_start_lba + (fat_offset / 512);
        let ent_offset = (fat_offset % 512) as usize;

        let mut buf = [0u8; SECTOR_SIZE];
        if !sd::read_sector(fat_sector, &mut buf) {
            return None;
        }

        let val = u32::from_le_bytes([
            buf[ent_offset],
            buf[ent_offset + 1],
            buf[ent_offset + 2],
            buf[ent_offset + 3],
        ]) & 0x0FFF_FFFF;

        if val >= 0x0FFF_FFF8 || val < 2 {
            None // End of chain
        } else {
            Some(val)
        }
    }
}

/// Get the root directory cluster number (usually 2).
pub fn root_cluster() -> u32 {
    match unsafe { &*core::ptr::addr_of!(VOLUME) } {
        Some(v) => v.root_cluster,
        None => 2,
    }
}

/// List all files and subdirectories in the root directory.
pub fn list_root() -> Vec<DirEntry> {
    list_dir(0)
}

/// List all files and subdirectories in the directory starting at `dir_cluster`
/// (`0` means the root directory).
pub fn list_dir(dir_cluster: u32) -> Vec<DirEntry> {
    let mut entries = Vec::new();
    let vol = match unsafe { &*core::ptr::addr_of!(VOLUME) } {
        Some(v) => v,
        None => return entries,
    };

    let mut cluster = if dir_cluster < 2 {
        vol.root_cluster
    } else {
        dir_cluster
    };
    let mut sector_buf = [0u8; SECTOR_SIZE];

    loop {
        let base_lba = vol.cluster_to_lba(cluster);
        for s in 0..(vol.sectors_per_cluster as u32) {
            let lba = base_lba + s;
            if !sd::read_sector(lba, &mut sector_buf) {
                return entries;
            }

            // 16 directory entries per 512-byte sector (32 bytes each)
            for idx in 0..16 {
                let offset = idx * 32;
                let entry = &sector_buf[offset..offset + 32];

                let first_byte = entry[0];
                if first_byte == 0x00 {
                    // End of directory
                    return entries;
                }
                if first_byte == 0xE5 {
                    // Deleted entry
                    continue;
                }

                let attr = entry[11];
                if attr == 0x0F || (attr & 0x08) != 0 {
                    // Long File Name (LFN) entry or Volume Label — skip
                    continue;
                }

                let is_dir = (attr & 0x10) != 0;
                let name = format_short_name(&entry[0..11]);
                if name == "." {
                    continue; // Skip self-reference entry
                }
                let cluster_hi = u16::from_le_bytes([entry[20], entry[21]]) as u32;
                let cluster_lo = u16::from_le_bytes([entry[26], entry[27]]) as u32;
                let first_cluster = (cluster_hi << 16) | cluster_lo;
                let size = u32::from_le_bytes([entry[28], entry[29], entry[30], entry[31]]);

                entries.push(DirEntry {
                    name,
                    is_dir,
                    size,
                    first_cluster,
                    dir_sector: lba,
                    dir_offset: offset,
                });
            }
        }

        match vol.next_cluster(cluster) {
            Some(next) => cluster = next,
            None => break,
        }
    }

    entries
}

/// Resolve a path like `"/CONFIG.TXT"`, `"VERSION.TXT"`, or `"/OVERLAYS/TFT9341.DTBO"`
/// into its `DirEntry`.
pub fn resolve_path(path: &str) -> Option<DirEntry> {
    let trimmed = path.trim_matches('/');
    if trimmed.is_empty() {
        return None;
    }

    let mut cur_cluster = 0u32;
    let mut parts = trimmed.split('/').peekable();

    while let Some(part) = parts.next() {
        if part.is_empty() {
            continue;
        }
        let entries = list_dir(cur_cluster);
        let found = entries
            .into_iter()
            .find(|e| eq_ignore_ascii_case(&e.name, part))?;

        if parts.peek().is_some() {
            if !found.is_dir {
                return None;
            }
            cur_cluster = found.first_cluster;
        } else {
            return Some(found);
        }
    }

    None
}

/// Read up to `buf.len()` bytes from `entry` starting at `offset` (streamed without loading whole file).
pub fn read_entry_range(entry: &DirEntry, offset: usize, buf: &mut [u8]) -> usize {
    if entry.is_dir || offset >= entry.size as usize || buf.is_empty() || entry.first_cluster < 2 {
        return 0;
    }

    let vol = match unsafe { &*core::ptr::addr_of!(VOLUME) } {
        Some(v) => v,
        None => return 0,
    };

    let spc = vol.sectors_per_cluster as usize;
    let cluster_bytes = spc * SECTOR_SIZE;
    let max_avail = (entry.size as usize) - offset;
    let to_read = buf.len().min(max_avail);

    // Advance to the starting cluster
    let skip_clusters = offset / cluster_bytes;
    let mut cluster = entry.first_cluster;
    for _ in 0..skip_clusters {
        match vol.next_cluster(cluster) {
            Some(next) => cluster = next,
            None => return 0,
        }
    }

    let mut cluster_off = offset % cluster_bytes;
    let mut copied = 0usize;
    let mut sector_buf = [0u8; SECTOR_SIZE];

    while copied < to_read {
        let base_lba = vol.cluster_to_lba(cluster);
        let start_s = cluster_off / SECTOR_SIZE;
        let mut sec_off = cluster_off % SECTOR_SIZE;

        for s in start_s..spc {
            if copied >= to_read {
                break;
            }
            if !sd::read_sector(base_lba + (s as u32), &mut sector_buf) {
                return copied;
            }
            let chunk = (SECTOR_SIZE - sec_off).min(to_read - copied);
            buf[copied..copied + chunk].copy_from_slice(&sector_buf[sec_off..sec_off + chunk]);
            copied += chunk;
            sec_off = 0;
        }

        cluster_off = 0;
        if copied >= to_read {
            break;
        }

        match vol.next_cluster(cluster) {
            Some(next) => cluster = next,
            None => break,
        }
    }

    copied
}

/// Read the entire contents of a file by name or path (e.g. `"VERSION.TXT"` or `"/OVERLAYS/README"`).
pub fn read_file(path: &str) -> Option<Vec<u8>> {
    let target = resolve_path(path)?;
    if target.is_dir {
        return None;
    }

    let mut data = alloc::vec![0u8; target.size as usize];
    let n = read_entry_range(&target, 0, &mut data);
    data.truncate(n);
    Some(data)
}

/// Update an existing small file (<= 512 bytes) given its `DirEntry`.
pub fn write_entry_small(target: &mut DirEntry, new_data: &[u8]) -> bool {
    if new_data.len() > SECTOR_SIZE || target.is_dir || target.first_cluster < 2 {
        return false;
    }

    let vol = match unsafe { &*core::ptr::addr_of!(VOLUME) } {
        Some(v) => v,
        None => return false,
    };

    let data_lba = vol.cluster_to_lba(target.first_cluster);
    let mut sector_buf = [0u8; SECTOR_SIZE];
    sector_buf[..new_data.len()].copy_from_slice(new_data);
    if !sd::write_sector(data_lba, &sector_buf) {
        return false;
    }

    let mut dir_buf = [0u8; SECTOR_SIZE];
    if !sd::read_sector(target.dir_sector, &mut dir_buf) {
        return false;
    }

    let new_size = new_data.len() as u32;
    let size_bytes = new_size.to_le_bytes();
    let off = target.dir_offset + 28;
    dir_buf[off..off + 4].copy_from_slice(&size_bytes);

    if sd::write_sector(target.dir_sector, &dir_buf) {
        target.size = new_size;
        true
    } else {
        false
    }
}

/// Update an existing small file (<= 1 sector / 512 bytes) by path or filename.
/// Used by Tryboot A/B to update `AUTOBOOT.TXT` on the boot partition.
pub fn write_small_file(filename: &str, new_data: &[u8]) -> bool {
    if let Some(mut entry) = resolve_path(filename) {
        write_entry_small(&mut entry, new_data)
    } else {
        false
    }
}

// ─── String formatting helpers ───

fn format_short_name(raw: &[u8]) -> String {
    let base = core::str::from_utf8(&raw[0..8]).unwrap_or("").trim_end();
    let ext = core::str::from_utf8(&raw[8..11]).unwrap_or("").trim_end();
    let mut s = String::with_capacity(12);
    s.push_str(base);
    if !ext.is_empty() {
        s.push('.');
        s.push_str(ext);
    }
    s
}

fn eq_ignore_ascii_case(a: &str, b: &str) -> bool {
    if a.len() != b.len() {
        return false;
    }
    a.bytes()
        .zip(b.bytes())
        .all(|(x, y)| x.to_ascii_uppercase() == y.to_ascii_uppercase())
}

#!/usr/bin/env python3
"""
ZerOS — Dual-Partition FAT32 SD Card Image Builder (M6.2 / M6.3)

Creates `sdcard.img` (16 MB) with an MBR partition table:
  - Partition 1 (Slot A, LBA 2048):  FAT32 volume "ZEROS_SLOTA"
  - Partition 2 (Slot B, LBA 16384): FAT32 volume "ZEROS_SLOTB"

Populates Partition 1 & 2 with:
  - AUTOBOOT.TXT  (Raspberry Pi A/B tryboot configuration)
  - CONFIG.TXT    (Raspberry Pi AArch64 bare-metal boot config)
  - VERSION.TXT   (ZerOS slot & build metadata)
  - KERNEL8.IMG   (Compiled ZerOS kernel binary, if present)
"""

import os
import struct
import sys

SECTOR_SIZE = 512
TOTAL_SECTORS = 32768  # 16 MB image

PART1_LBA = 2048
PART1_SECTORS = 14336  # 7 MB

PART2_LBA = 16384
PART2_SECTORS = 14336  # 7 MB


def make_mbr() -> bytes:
    mbr = bytearray(SECTOR_SIZE)
    # Partition 1: Slot A (Bootable 0x80, Type 0x0C = FAT32 LBA)
    struct.pack_into(
        "<BBBBBBBBII",
        mbr,
        0x1BE,
        0x80, 0, 1, 0, 0x0C, 0xFE, 0xFF, 0xFF,
        PART1_LBA,
        PART1_SECTORS,
    )
    # Partition 2: Slot B (Type 0x0C = FAT32 LBA)
    struct.pack_into(
        "<BBBBBBBBII",
        mbr,
        0x1CE,
        0x00, 0, 1, 0, 0x0C, 0xFE, 0xFF, 0xFF,
        PART2_LBA,
        PART2_SECTORS,
    )
    mbr[510] = 0x55
    mbr[511] = 0xAA
    return bytes(mbr)


def build_fat32_partition(total_sectors: int, label: bytes, files: list[tuple[str, bytes]]) -> bytearray:
    part = bytearray(total_sectors * SECTOR_SIZE)

    bytes_per_sector = 512
    sectors_per_cluster = 8  # 4 KB clusters
    reserved_sectors = 32
    num_fats = 2
    sectors_per_fat = 16
    root_cluster = 2

    # ── 1. Build VBR (Boot Sector) at sector 0 ──
    vbr = bytearray(SECTOR_SIZE)
    vbr[0:3] = b"\xEB\x58\x90"
    vbr[3:11] = b"ZEROSFAT"
    struct.pack_into("<H", vbr, 0x0B, bytes_per_sector)
    vbr[0x0D] = sectors_per_cluster
    struct.pack_into("<H", vbr, 0x0E, reserved_sectors)
    vbr[0x10] = num_fats
    struct.pack_into("<H", vbr, 0x11, 0)       # Root entry count (0 for FAT32)
    struct.pack_into("<H", vbr, 0x13, 0)       # Total sectors 16
    vbr[0x15] = 0xF8                           # Media descriptor (fixed disk)
    struct.pack_into("<H", vbr, 0x16, 0)       # Sectors per FAT 16 (0 for FAT32)
    struct.pack_into("<H", vbr, 0x18, 63)      # Sectors per track
    struct.pack_into("<H", vbr, 0x1A, 255)     # Heads
    struct.pack_into("<I", vbr, 0x20, total_sectors)
    struct.pack_into("<I", vbr, 0x24, sectors_per_fat)
    struct.pack_into("<H", vbr, 0x28, 0)       # Ext flags
    struct.pack_into("<H", vbr, 0x2A, 0)       # FS version
    struct.pack_into("<I", vbr, 0x2C, root_cluster)
    struct.pack_into("<H", vbr, 0x30, 1)       # FSInfo sector
    struct.pack_into("<H", vbr, 0x32, 6)       # Backup boot sector
    vbr[0x42] = 0x29                           # Extended boot signature
    struct.pack_into("<I", vbr, 0x43, 0x20261109)
    vbr[0x47:0x52] = label.ljust(11, b" ")[:11]
    vbr[0x52:0x5A] = b"FAT32   "
    vbr[510] = 0x55
    vbr[511] = 0xAA
    part[0:SECTOR_SIZE] = vbr

    # ── 2. Initialize FAT tables ──
    fat = bytearray(sectors_per_fat * SECTOR_SIZE)
    struct.pack_into("<I", fat, 0, 0x0FFFFFF8)  # Cluster 0
    struct.pack_into("<I", fat, 4, 0x0FFFFFFF)  # Cluster 1
    struct.pack_into("<I", fat, 8, 0x0FFFFFFF)  # Cluster 2 (Root directory)

    data_start_sector = reserved_sectors + num_fats * sectors_per_fat
    cluster_size = sectors_per_cluster * SECTOR_SIZE

    def cluster_offset(c: int) -> int:
        return (data_start_sector + (c - 2) * sectors_per_cluster) * SECTOR_SIZE

    next_free_cluster = 3
    root_dir = bytearray(cluster_size)
    dir_idx = 0

    for filename, content in files:
        # Format 8.3 short name
        if "." in filename:
            base, ext = filename.split(".", 1)
        else:
            base, ext = filename, ""
        short_name = base.upper().ljust(8, " ")[:8].encode("ascii") + ext.upper().ljust(3, " ")[:3].encode("ascii")

        size = len(content)
        if size == 0:
            first_cluster = 0
        else:
            first_cluster = next_free_cluster
            num_clusters = (size + cluster_size - 1) // cluster_size
            for i in range(num_clusters):
                c = first_cluster + i
                chunk = content[i * cluster_size : (i + 1) * cluster_size]
                off = cluster_offset(c)
                part[off : off + len(chunk)] = chunk
                # Link in FAT
                if i + 1 < num_clusters:
                    struct.pack_into("<I", fat, c * 4, c + 1)
                else:
                    struct.pack_into("<I", fat, c * 4, 0x0FFFFFFF)
            next_free_cluster += num_clusters

        # Write 32-byte directory entry in root_dir
        ent_off = dir_idx * 32
        root_dir[ent_off : ent_off + 11] = short_name
        root_dir[ent_off + 11] = 0x20  # Archive attribute
        cluster_hi = (first_cluster >> 16) & 0xFFFF
        cluster_lo = first_cluster & 0xFFFF
        struct.pack_into("<H", root_dir, ent_off + 20, cluster_hi)
        struct.pack_into("<H", root_dir, ent_off + 26, cluster_lo)
        struct.pack_into("<I", root_dir, ent_off + 28, size)
        dir_idx += 1

    # Write root directory into Cluster 2
    r_off = cluster_offset(root_cluster)
    part[r_off : r_off + cluster_size] = root_dir

    # Write FAT1 and FAT2
    fat1_off = reserved_sectors * SECTOR_SIZE
    fat2_off = (reserved_sectors + sectors_per_fat) * SECTOR_SIZE
    part[fat1_off : fat1_off + len(fat)] = fat
    part[fat2_off : fat2_off + len(fat)] = fat

    return part


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else "sdcard.img"
    kernel_path = "kernel8.img"

    autoboot_txt = (
        "[all]\n"
        "tryboot_a_b=1\n"
        "boot_partition=1\n"
        "[tryboot]\n"
        "boot_partition=2\n"
    ).encode("utf-8")

    config_txt = (
        "# ZerOS Raspberry Pi Zero 2 W Boot Config\n"
        "arm_64bit=1\n"
        "kernel=kernel8.img\n"
        "enable_uart=1\n"
        "dtparam=spi=on\n"
    ).encode("utf-8")

    version_a = b"ZerOS v0.1.0 | Slot A (Primary Recovery Partition)\n"
    version_b = b"ZerOS v0.1.0 | Slot B (Tryboot Experimental Partition)\n"

    kernel_data = open(kernel_path, "rb").read() if os.path.exists(kernel_path) else b""

    files_a = [
        ("AUTOBOOT.TXT", autoboot_txt),
        ("CONFIG.TXT", config_txt),
        ("VERSION.TXT", version_a),
    ]
    files_b = [
        ("CONFIG.TXT", config_txt),
        ("VERSION.TXT", version_b),
    ]
    if kernel_data:
        files_a.append(("KERNEL8.IMG", kernel_data))
        files_b.append(("KERNEL8.IMG", kernel_data))

    img = bytearray(TOTAL_SECTORS * SECTOR_SIZE)
    img[0:SECTOR_SIZE] = make_mbr()

    part1 = build_fat32_partition(PART1_SECTORS, b"ZEROS_SLOTA", files_a)
    part2 = build_fat32_partition(PART2_SECTORS, b"ZEROS_SLOTB", files_b)

    p1_off = PART1_LBA * SECTOR_SIZE
    p2_off = PART2_LBA * SECTOR_SIZE
    img[p1_off : p1_off + len(part1)] = part1
    img[p2_off : p2_off + len(part2)] = part2

    with open(out_path, "wb") as f:
        f.write(img)

    print(f"✅ Created dual-partition FAT32 SD image: {out_path} ({len(img) // (1024*1024)} MB)")


if __name__ == "__main__":
    main()

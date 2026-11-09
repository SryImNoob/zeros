// ZerOS — Virtual Filesystem (VFS) & File Descriptor Table (M7.3)
//
// Provides a UNIX-style File Descriptor (FD) abstraction over mounted
// filesystems (currently FAT32 on Arasan EMMC SD card):
//   - `vfs::open(path, mode)` / `vfs::open_entry(entry, mode)` -> `Some(fd)`
//   - `vfs::read(fd, &mut buf)` (streams bytes at current cursor without full-file allocation)
//   - `vfs::seek(fd, new_offset)`
//   - `vfs::write(fd, data)`
//   - `vfs::close(fd)`

#![allow(dead_code)]

use crate::fs::fat32::{self, DirEntry};
use crate::sync::IrqSafeLock;

/// Maximum number of simultaneously open file descriptors across the kernel
pub const MAX_OPEN_FILES: usize = 16;

/// File open mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OpenMode {
    ReadOnly,
    ReadWrite,
}

#[derive(Clone)]
struct FileDescSlot {
    in_use: bool,
    entry: Option<DirEntry>,
    offset: usize,
    mode: OpenMode,
}

impl FileDescSlot {
    const fn empty() -> Self {
        Self {
            in_use: false,
            entry: None,
            offset: 0,
            mode: OpenMode::ReadOnly,
        }
    }
}

struct VfsTable {
    slots: [FileDescSlot; MAX_OPEN_FILES],
    total_ops: u64,
}

impl VfsTable {
    const fn new() -> Self {
        Self {
            slots: [const { FileDescSlot::empty() }; MAX_OPEN_FILES],
            total_ops: 0,
        }
    }
}

static VFS: IrqSafeLock<VfsTable> = IrqSafeLock::new(VfsTable::new());

/// Open a file by path (e.g. `"/CONFIG.TXT"` or `"/OVERLAYS/TFT9341.DTBO"`) and return its FD (`0..15`).
pub fn open(path: &str, mode: OpenMode) -> Option<usize> {
    let entry = fat32::resolve_path(path)?;
    open_entry(entry, mode)
}

/// Open an already-resolved `DirEntry` as a File Descriptor (`0..15`).
pub fn open_entry(entry: DirEntry, mode: OpenMode) -> Option<usize> {
    if entry.is_dir {
        return None;
    }

    VFS.with(|vfs| {
        vfs.total_ops += 1;
        for (fd, slot) in vfs.slots.iter_mut().enumerate() {
            if !slot.in_use {
                slot.in_use = true;
                slot.entry = Some(entry);
                slot.offset = 0;
                slot.mode = mode;
                return Some(fd);
            }
        }
        None
    })
}

/// Read up to `buf.len()` bytes from open file descriptor `fd` at its current offset,
/// advancing the file offset by the number of bytes read.
pub fn read(fd: usize, buf: &mut [u8]) -> Option<usize> {
    if fd >= MAX_OPEN_FILES {
        return None;
    }

    let (entry, offset) = VFS.with(|vfs| {
        vfs.total_ops += 1;
        let slot = &vfs.slots[fd];
        if !slot.in_use {
            return None;
        }
        Some((slot.entry.clone()?, slot.offset))
    })?;

    let n = fat32::read_entry_range(&entry, offset, buf);

    VFS.with(|vfs| {
        if vfs.slots[fd].in_use {
            vfs.slots[fd].offset = offset + n;
        }
    });

    Some(n)
}

/// Seek to byte `new_offset` in file descriptor `fd`.
pub fn seek(fd: usize, new_offset: usize) -> Option<usize> {
    if fd >= MAX_OPEN_FILES {
        return None;
    }

    VFS.with(|vfs| {
        vfs.total_ops += 1;
        let slot = &mut vfs.slots[fd];
        if !slot.in_use {
            return None;
        }
        let size = slot.entry.as_ref()?.size as usize;
        slot.offset = new_offset.min(size);
        Some(slot.offset)
    })
}

/// Write `data` to open file descriptor `fd` (requires `OpenMode::ReadWrite`).
pub fn write(fd: usize, data: &[u8]) -> Option<usize> {
    if fd >= MAX_OPEN_FILES {
        return None;
    }

    let mut entry = VFS.with(|vfs| {
        vfs.total_ops += 1;
        let slot = &vfs.slots[fd];
        if !slot.in_use || slot.mode != OpenMode::ReadWrite {
            return None;
        }
        slot.entry.clone()
    })?;

    if !fat32::write_entry_small(&mut entry, data) {
        return None;
    }

    VFS.with(|vfs| {
        if vfs.slots[fd].in_use {
            vfs.slots[fd].offset = data.len();
            vfs.slots[fd].entry = Some(entry);
        }
    });

    Some(data.len())
}

/// Close an open file descriptor `fd`.
pub fn close(fd: usize) -> bool {
    if fd >= MAX_OPEN_FILES {
        return false;
    }

    VFS.with(|vfs| {
        vfs.total_ops += 1;
        if vfs.slots[fd].in_use {
            vfs.slots[fd] = FileDescSlot::empty();
            true
        } else {
            false
        }
    })
}

/// Return the number of currently open file descriptors.
pub fn open_count() -> usize {
    VFS.with(|vfs| vfs.slots.iter().filter(|s| s.in_use).count())
}

/// Return total VFS syscall-equivalent operations performed since boot.
pub fn total_ops() -> u64 {
    VFS.with(|vfs| vfs.total_ops)
}

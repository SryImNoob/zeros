// ZerOS — Pure `#![no_std]` ISO/IEC 18004 QR Code Model 2 Generator (`M12.1`)
//
// Zero-heap, stack-only QR Code encoder supporting Versions 1..5 (21x21 up to 37x37,
// up to 106 UTF-8/ASCII bytes in Byte Mode, Error Correction Level L).
// Used by `MusicApp` to render the NetEase Cloud Music (`music.163.com`) QR login code
// directly onto the 320x240 ILI9341 display.

/// Maximum QR matrix dimension (Version 5 = 37x37)
pub const MAX_QR_SIZE: usize = 37;

pub struct QrCode {
    pub size: usize,
    modules: [[bool; MAX_QR_SIZE]; MAX_QR_SIZE],
    is_func: [[bool; MAX_QR_SIZE]; MAX_QR_SIZE],
}

impl QrCode {
    /// Encode a byte slice (up to 106 bytes, e.g. `https://music.163.com/login?codekey=...`)
    /// into a standard ISO/IEC 18004 QR Code matrix.
    pub fn encode_bytes(data: &[u8]) -> Option<Self> {
        // Select smallest version (1..=5) in ECC-L Byte Mode that fits `data.len()`
        let (version, data_cw, ec_cw): (usize, usize, usize) = if data.len() <= 17 {
            (1, 19, 7)
        } else if data.len() <= 32 {
            (2, 34, 10)
        } else if data.len() <= 53 {
            (3, 55, 15)
        } else if data.len() <= 78 {
            (4, 80, 20)
        } else if data.len() <= 106 {
            (5, 108, 26)
        } else {
            return None;
        };

        let size = 17 + version * 4;
        let mut qr = Self {
            size,
            modules: [[false; MAX_QR_SIZE]; MAX_QR_SIZE],
            is_func: [[false; MAX_QR_SIZE]; MAX_QR_SIZE],
        };

        // 1. Build Byte-Mode bitstream:
        //    - 4-bit mode indicator: 0b0100 (Byte mode)
        //    - 8-bit character count indicator (for Versions 1..9)
        //    - Payload bytes
        //    - Up to 4-bit terminator 0b0000
        //    - Pad to byte boundary, then alternate 0xEC / 0x11 up to `data_cw`
        let mut codewords = [0u8; 134];
        let mut bit_len = 0usize;

        Self::append_bits(&mut codewords, &mut bit_len, 0b0100, 4);
        Self::append_bits(&mut codewords, &mut bit_len, data.len() as u32, 8);
        for &b in data {
            Self::append_bits(&mut codewords, &mut bit_len, b as u32, 8);
        }

        let capacity_bits = data_cw * 8;
        let term = (capacity_bits - bit_len).min(4);
        Self::append_bits(&mut codewords, &mut bit_len, 0, term);
        while bit_len % 8 != 0 {
            Self::append_bits(&mut codewords, &mut bit_len, 0, 1);
        }

        let mut pad_toggle = false;
        while bit_len < capacity_bits {
            let pad = if pad_toggle { 0x11 } else { 0xEC };
            Self::append_bits(&mut codewords, &mut bit_len, pad, 8);
            pad_toggle = !pad_toggle;
        }

        // 2. Compute Reed-Solomon ECC codewords over GF(2^8) (modulo 0x11D)
        let mut ecc = [0u8; 32];
        Self::reed_solomon_compute(&codewords[..data_cw], ec_cw, &mut ecc[..ec_cw]);
        for i in 0..ec_cw {
            codewords[data_cw + i] = ecc[i];
        }
        let total_cw = data_cw + ec_cw;

        // 3. Draw function patterns (Finders, Separators, Timing, Alignment, Dark Module, Format)
        qr.draw_function_patterns(version);

        // 4. Draw data + ECC codewords in zigzag order
        qr.draw_codewords(&codewords[..total_cw]);

        // 5. Apply Mask Pattern 0 ((x + y) % 2 == 0) and write final Format Info (0x77C4)
        qr.apply_mask_0();
        qr.draw_format_bits(0x77C4);

        Some(qr)
    }

    #[inline]
    pub fn get_module(&self, x: usize, y: usize) -> bool {
        if x < self.size && y < self.size {
            self.modules[y][x]
        } else {
            false
        }
    }

    fn append_bits(buf: &mut [u8], bit_len: &mut usize, val: u32, num_bits: usize) {
        for i in (0..num_bits).rev() {
            let bit = ((val >> i) & 1) as u8;
            let byte_idx = *bit_len >> 3;
            let bit_idx = 7 - (*bit_len & 7);
            buf[byte_idx] |= bit << bit_idx;
            *bit_len += 1;
        }
    }

    fn gf_mul(mut x: u8, mut y: u8) -> u8 {
        let mut z = 0u8;
        for _ in 0..8 {
            if (y & 1) != 0 {
                z ^= x;
            }
            let hi = x & 0x80;
            x <<= 1;
            if hi != 0 {
                x ^= 0x1D;
            }
            y >>= 1;
        }
        z
    }

    fn reed_solomon_compute(data: &[u8], degree: usize, result: &mut [u8]) {
        // Compute generator polynomial coefficients g[0..degree]
        let mut gen = [0u8; 32];
        gen[degree - 1] = 1;
        let mut root = 1u8;
        for _ in 0..degree {
            for j in 0..degree {
                gen[j] = Self::gf_mul(gen[j], root);
                if j + 1 < degree {
                    gen[j] ^= gen[j + 1];
                }
            }
            root = Self::gf_mul(root, 0x02);
        }

        for r in result.iter_mut() {
            *r = 0;
        }
        for &b in data {
            let factor = b ^ result[0];
            for j in 0..(degree - 1) {
                result[j] = result[j + 1] ^ Self::gf_mul(gen[j], factor);
            }
            result[degree - 1] = Self::gf_mul(gen[degree - 1], factor);
        }
    }

    fn set_func_module(&mut self, x: usize, y: usize, is_dark: bool) {
        if x < self.size && y < self.size {
            self.modules[y][x] = is_dark;
            self.is_func[y][x] = true;
        }
    }

    fn draw_finder_pattern(&mut self, cx: i32, cy: i32) {
        for dy in -4..=4 {
            for dx in -4..=4 {
                let xx = cx + dx;
                let yy = cy + dy;
                if xx >= 0 && (xx as usize) < self.size && yy >= 0 && (yy as usize) < self.size {
                    let dist = dx.abs().max(dy.abs());
                    let dark = dist != 2 && dist != 4;
                    self.set_func_module(xx as usize, yy as usize, dark);
                }
            }
        }
    }

    fn draw_alignment_pattern(&mut self, cx: usize, cy: usize) {
        for dy in -2i32..=2 {
            for dx in -2i32..=2 {
                let dist = dx.abs().max(dy.abs());
                self.set_func_module(
                    (cx as i32 + dx) as usize,
                    (cy as i32 + dy) as usize,
                    dist != 1,
                );
            }
        }
    }

    fn draw_function_patterns(&mut self, version: usize) {
        let sz = self.size;
        // Timing patterns
        for i in 0..sz {
            self.set_func_module(6, i, i % 2 == 0);
            self.set_func_module(i, 6, i % 2 == 0);
        }

        // 3 Finder patterns
        self.draw_finder_pattern(3, 3);
        self.draw_finder_pattern((sz - 4) as i32, 3);
        self.draw_finder_pattern(3, (sz - 4) as i32);

        // Alignment pattern for Versions 2..=5 at (sz - 7, sz - 7)
        if version >= 2 {
            self.draw_alignment_pattern(sz - 7, sz - 7);
        }

        // Reserve Format Information modules
        for i in 0..=8 {
            if i != 6 {
                self.set_func_module(8, i, false);
                self.set_func_module(i, 8, false);
            }
        }
        for i in 0..8 {
            self.set_func_module(sz - 1 - i, 8, false);
            self.set_func_module(8, sz - 1 - i, false);
        }
        // Dark module
        self.set_func_module(8, sz - 8, true);
    }

    fn draw_codewords(&mut self, data: &[u8]) {
        let sz = self.size as i32;
        let mut bit_idx = 0usize;
        let total_bits = data.len() * 8;

        let mut right = sz - 1;
        while right >= 1 {
            if right == 6 {
                right = 5;
            }
            for vert in 0..sz {
                for j in 0..2 {
                    let x = (right - j) as usize;
                    let upward = ((right + 1) & 2) == 0;
                    let y = (if upward { sz - 1 - vert } else { vert }) as usize;
                    if !self.is_func[y][x] && bit_idx < total_bits {
                        let dark = ((data[bit_idx >> 3] >> (7 - (bit_idx & 7))) & 1) != 0;
                        self.modules[y][x] = dark;
                        bit_idx += 1;
                    }
                }
            }
            right -= 2;
        }
    }

    fn apply_mask_0(&mut self) {
        for y in 0..self.size {
            for x in 0..self.size {
                if !self.is_func[y][x] && ((x + y) % 2 == 0) {
                    self.modules[y][x] = !self.modules[y][x];
                }
            }
        }
    }

    fn draw_format_bits(&mut self, bits: u16) {
        let sz = self.size;
        for i in 0..=5 {
            self.set_func_module(8, i, ((bits >> i) & 1) != 0);
        }
        self.set_func_module(8, 7, ((bits >> 6) & 1) != 0);
        self.set_func_module(8, 8, ((bits >> 7) & 1) != 0);
        self.set_func_module(7, 8, ((bits >> 8) & 1) != 0);
        for i in 9..15 {
            self.set_func_module(14 - i, 8, ((bits >> i) & 1) != 0);
        }

        for i in 0..8 {
            self.set_func_module(sz - 1 - i, 8, ((bits >> i) & 1) != 0);
        }
        for i in 8..15 {
            self.set_func_module(8, sz - 15 + i, ((bits >> i) & 1) != 0);
        }
        self.set_func_module(8, sz - 8, true);
    }
}

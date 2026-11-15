// ZerOS — NetEase Cloud Music `eapi` Cryptography (`AES-128-ECB` + `MD5`) & Protocol (`M13.4` / `M14.1`)
//
// Why `eapi` (`http://interface.music.163.com/eapi/...`)?
//   - Unlike `/weapi/` (which requires HTTPS TLS + 1024-bit RSA + AES-128-CBC),
//     NetEase Cloud Music's desktop/mobile client `/eapi/` protocol works directly over
//     plain HTTP port 80 (`interface.music.163.com:80`) using deterministic `MD5` + `AES-128-ECB`.
//
// `eapi` signing & encryption specification:
//   1. `digest = MD5_hex("nobody" + api_url + "use" + json_payload + "md5forencrypt")`
//   2. `plaintext = api_url + "-36cd479b6b5-" + json_payload + "-36cd479b6b5-" + digest`
//   3. `params = HEX_UPPER(AES_128_ECB_PKCS7(plaintext, key = b"e82ckenh8dichen8"))`

use alloc::string::String;
use alloc::vec::Vec;
use core::fmt::Write;

const EAPI_KEY: [u8; 16] = *b"e82ckenh8dichen8";

// ─── RFC 1321 MD5 Implementation (`#![no_std]`) ───

const MD5_S: [u32; 64] = [
    7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 5, 9, 14, 20, 5, 9, 14, 20, 5, 9,
    14, 20, 5, 9, 14, 20, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 6, 10, 15,
    21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21,
];

const MD5_K: [u32; 64] = [
    0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee, 0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501,
    0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be, 0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821,
    0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa, 0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8,
    0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed, 0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a,
    0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c, 0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70,
    0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05, 0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665,
    0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039, 0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
    0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1, 0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391,
];

pub fn md5_digest(msg: &[u8]) -> [u8; 16] {
    let mut a0: u32 = 0x6745_2301;
    let mut b0: u32 = 0xefcd_ab89;
    let mut c0: u32 = 0x98ba_dcfe;
    let mut d0: u32 = 0x1032_5476;

    let bit_len = (msg.len() as u64).wrapping_mul(8);
    let mut padded = Vec::with_capacity((msg.len() + 72) & !63);
    padded.extend_from_slice(msg);
    padded.push(0x80);
    while (padded.len() % 64) != 56 {
        padded.push(0x00);
    }
    padded.extend_from_slice(&bit_len.to_le_bytes());

    for chunk in padded.chunks_exact(64) {
        let mut m = [0u32; 16];
        for (i, word) in m.iter_mut().enumerate() {
            let off = i * 4;
            *word = u32::from_le_bytes([
                chunk[off],
                chunk[off + 1],
                chunk[off + 2],
                chunk[off + 3],
            ]);
        }

        let mut a = a0;
        let mut b = b0;
        let mut c = c0;
        let mut d = d0;

        for i in 0..64 {
            let (f, g) = if i < 16 {
                ((b & c) | (!b & d), i)
            } else if i < 32 {
                ((d & b) | (!d & c), (5 * i + 1) % 16)
            } else if i < 48 {
                (b ^ c ^ d, (3 * i + 5) % 16)
            } else {
                (c ^ (b | !d), (7 * i) % 16)
            };

            let temp = d;
            d = c;
            c = b;
            let sum = a.wrapping_add(f).wrapping_add(MD5_K[i]).wrapping_add(m[g]);
            b = b.wrapping_add(sum.rotate_left(MD5_S[i]));
            a = temp;
        }

        a0 = a0.wrapping_add(a);
        b0 = b0.wrapping_add(b);
        c0 = c0.wrapping_add(c);
        d0 = d0.wrapping_add(d);
    }

    let mut out = [0u8; 16];
    out[0..4].copy_from_slice(&a0.to_le_bytes());
    out[4..8].copy_from_slice(&b0.to_le_bytes());
    out[8..12].copy_from_slice(&c0.to_le_bytes());
    out[12..16].copy_from_slice(&d0.to_le_bytes());
    out
}

pub fn md5_hex_lower(msg: &[u8]) -> String {
    let raw = md5_digest(msg);
    let mut s = String::with_capacity(32);
    for b in raw {
        let _ = write!(s, "{:02x}", b);
    }
    s
}

// ─── FIPS-197 AES-128-ECB Implementation (`#![no_std]`) ───

const AES_SBOX: [u8; 256] = [
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16,
];

const AES_RCON: [u8; 10] = [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36];

#[inline(always)]
fn xtime(x: u8) -> u8 {
    (x << 1) ^ if (x & 0x80) != 0 { 0x1b } else { 0x00 }
}

fn aes128_expand_key(key: &[u8; 16]) -> [[u8; 16]; 11] {
    let mut rk = [[0u8; 16]; 11];
    rk[0] = *key;
    for round in 1..=10 {
        let prev = rk[round - 1];
        let mut temp = [
            AES_SBOX[prev[13] as usize] ^ AES_RCON[round - 1],
            AES_SBOX[prev[14] as usize],
            AES_SBOX[prev[15] as usize],
            AES_SBOX[prev[12] as usize],
        ];
        let mut cur = [0u8; 16];
        for col in 0..4 {
            let off = col * 4;
            for i in 0..4 {
                cur[off + i] = prev[off + i] ^ temp[i];
                temp[i] = cur[off + i];
            }
        }
        rk[round] = cur;
    }
    rk
}

fn aes128_encrypt_block(block: &mut [u8; 16], rk: &[[u8; 16]; 11]) {
    for i in 0..16 {
        block[i] ^= rk[0][i];
    }
    for round in 1..=10 {
        // SubBytes
        for b in block.iter_mut() {
            *b = AES_SBOX[*b as usize];
        }
        // ShiftRows
        let s = *block;
        block[1] = s[5];
        block[5] = s[9];
        block[9] = s[13];
        block[13] = s[1];

        block[2] = s[10];
        block[6] = s[14];
        block[10] = s[2];
        block[14] = s[6];

        block[3] = s[15];
        block[7] = s[3];
        block[11] = s[7];
        block[15] = s[11];

        // MixColumns (rounds 1..9)
        if round < 10 {
            for col in 0..4 {
                let c = col * 4;
                let a0 = block[c];
                let a1 = block[c + 1];
                let a2 = block[c + 2];
                let a3 = block[c + 3];
                let t = a0 ^ a1 ^ a2 ^ a3;
                block[c] ^= t ^ xtime(a0 ^ a1);
                block[c + 1] ^= t ^ xtime(a1 ^ a2);
                block[c + 2] ^= t ^ xtime(a2 ^ a3);
                block[c + 3] ^= t ^ xtime(a3 ^ a0);
            }
        }
        // AddRoundKey
        for i in 0..16 {
            block[i] ^= rk[round][i];
        }
    }
}

/// Build the `params=...` form body for a NetEase Cloud Music `eapi` request.
///
/// `api_url` is the canonical `/api/...` path (e.g. `"/api/login/qrcode/unikey"`), while
/// the HTTP request is sent to `POST /eapi/login/qrcode/unikey`.
pub fn build_eapi_params_body(api_url: &str, json_payload: &str) -> String {
    let mut hash_input = String::with_capacity(64 + api_url.len() + json_payload.len());
    hash_input.push_str("nobody");
    hash_input.push_str(api_url);
    hash_input.push_str("use");
    hash_input.push_str(json_payload);
    hash_input.push_str("md5forencrypt");
    let digest = md5_hex_lower(hash_input.as_bytes());

    let mut plain = Vec::with_capacity(api_url.len() + json_payload.len() + 64);
    plain.extend_from_slice(api_url.as_bytes());
    plain.extend_from_slice(b"-36cd479b6b5-");
    plain.extend_from_slice(json_payload.as_bytes());
    plain.extend_from_slice(b"-36cd479b6b5-");
    plain.extend_from_slice(digest.as_bytes());

    // PKCS#7 padding to 16-byte block boundary
    let pad_len = 16 - (plain.len() % 16);
    for _ in 0..pad_len {
        plain.push(pad_len as u8);
    }

    let rk = aes128_expand_key(&EAPI_KEY);
    let mut out = String::with_capacity(7 + plain.len() * 2);
    out.push_str("params=");
    for chunk in plain.chunks_exact_mut(16) {
        let mut blk = [0u8; 16];
        blk.copy_from_slice(chunk);
        aes128_encrypt_block(&mut blk, &rk);
        for b in blk {
            let _ = write!(out, "{:02X}", b);
        }
    }
    out
}

/// Extract a string value `"key":"value"` from a JSON response body.
pub fn extract_json_str(json: &str, key: &str) -> Option<String> {
    let mut pat = String::with_capacity(key.len() + 4);
    pat.push('"');
    pat.push_str(key);
    pat.push_str("\":\"");
    let start = json.find(&pat)? + pat.len();
    let rest = &json[start..];
    let end = rest.find('"')?;
    Some(String::from(&rest[..end]))
}

/// Extract an integer value `"key":800` from a JSON response body.
pub fn extract_json_int(json: &str, key: &str) -> Option<i32> {
    let mut pat = String::with_capacity(key.len() + 3);
    pat.push('"');
    pat.push_str(key);
    pat.push_str("\":");
    let start = json.find(&pat)? + pat.len();
    let rest = json[start..].trim_start();
    let mut end = 0usize;
    for b in rest.bytes() {
        if b.is_ascii_digit() || b == b'-' {
            end += 1;
        } else {
            break;
        }
    }
    if end == 0 {
        return None;
    }
    rest[..end].parse::<i32>().ok()
}

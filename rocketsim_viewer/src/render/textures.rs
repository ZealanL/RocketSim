//! PNG decoding + GL texture upload.
//!
//! Bundled PNGs (all 128x128 RGBA8) decode to raw rows; upload uses
//! `UNPACK_ALIGNMENT 1`, trilinear min + linear mag, repeat wrap, and
//! mipmaps. Unknown names panic (a typo must crash, never render black).

use std::io::Cursor;

use glow::HasContext;

/// Decoded RGBA8 image, row-major from the top.
pub struct PngImage {
    /// Width in pixels (all bundled assets are 128).
    pub width: u32,
    /// Height in pixels (all bundled assets are 128).
    pub height: u32,
    /// `width*height*4` RGBA bytes.
    pub bytes: Vec<u8>,
}

impl PngImage {
    /// Decodes PNG bytes; panics unless 8-bit RGBA (our asset contract).
    pub fn decode(png_bytes: &[u8]) -> Self {
        let decoder = png::Decoder::new(Cursor::new(png_bytes));
        let mut reader = decoder.read_info().expect("texture: PNG info unreadable");
        let mut buf = vec![
            0;
            reader
                .output_buffer_size()
                .expect("texture: no buffer size")
        ];
        let info = reader
            .next_frame(&mut buf)
            .expect("texture: PNG decode failed");
        assert!(
            matches!(info.color_type, png::ColorType::Rgba),
            "texture: expected RGBA8, got {:?}",
            info.color_type
        );
        assert_eq!(
            info.bit_depth,
            png::BitDepth::Eight,
            "texture: expected 8-bit"
        );
        let bytes = buf[..info.buffer_size()].to_vec();
        assert_eq!(bytes.len() as u32, info.width * info.height * 4);
        Self {
            width: info.width,
            height: info.height,
            bytes,
        }
    }

    /// 1x1 magenta: loud-not-black default for future procedural paths.
    pub fn fallback_magenta() -> Self {
        Self {
            width: 1,
            height: 1,
            bytes: vec![255, 0, 255, 255],
        }
    }
}

/// Owned GL texture. Create/upload/destroy on the GL thread.
pub struct TextureGl {
    id: glow::Texture,
}

impl TextureGl {
    /// Uploads `img` with trilinear filtering + mipmaps.
    pub fn new(gl: &glow::Context, img: &PngImage) -> Self {
        unsafe {
            gl.pixel_store_i32(glow::UNPACK_ALIGNMENT, 1);
            let id = gl.create_texture().expect("GL create texture");
            gl.bind_texture(glow::TEXTURE_2D, Some(id));
            gl.tex_parameter_i32(
                glow::TEXTURE_2D,
                glow::TEXTURE_MIN_FILTER,
                glow::LINEAR_MIPMAP_LINEAR as i32,
            );
            gl.tex_parameter_i32(
                glow::TEXTURE_2D,
                glow::TEXTURE_MAG_FILTER,
                glow::LINEAR as i32,
            );
            gl.tex_parameter_i32(glow::TEXTURE_2D, glow::TEXTURE_WRAP_S, glow::REPEAT as i32);
            gl.tex_parameter_i32(glow::TEXTURE_2D, glow::TEXTURE_WRAP_T, glow::REPEAT as i32);
            gl.tex_image_2d(
                glow::TEXTURE_2D,
                0,
                glow::RGBA8 as i32,
                img.width as i32,
                img.height as i32,
                0,
                glow::RGBA,
                glow::UNSIGNED_BYTE,
                glow::PixelUnpackData::Slice(Some(img.bytes.as_slice())),
            );
            gl.generate_mipmap(glow::TEXTURE_2D);
            gl.bind_texture(glow::TEXTURE_2D, None);
            Self { id }
        }
    }

    /// Binds to unit 0 (the only unit the v2 shaders sample).
    pub fn bind(&self, gl: &glow::Context) {
        unsafe {
            gl.active_texture(glow::TEXTURE0);
            gl.bind_texture(glow::TEXTURE_2D, Some(self.id));
        }
    }

    /// Deletes the texture. Call on the GL thread before dropping.
    pub fn destroy(&self, gl: &glow::Context) {
        unsafe {
            gl.delete_texture(self.id);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bundled_pngs_decode() {
        for (label, bytes) in [
            ("ball", include_bytes!("../textures/ball.png").as_slice()),
            (
                "car_blue",
                include_bytes!("../textures/car_blue.png").as_slice(),
            ),
            (
                "car_orange",
                include_bytes!("../textures/car_orange.png").as_slice(),
            ),
            ("white", include_bytes!("../textures/white.png").as_slice()),
        ] {
            let img = PngImage::decode(bytes);
            assert_eq!((img.width, img.height), (128, 128), "{label}: size");
            assert_eq!(img.bytes.len(), 128 * 128 * 4, "{label}: bytes");
        }
    }

    #[test]
    fn special_images() {
        // The arena shader multiplies team colors by white: any tint here
        // would tint the whole stadium.
        let img = PngImage::decode(include_bytes!("../textures/white.png"));
        assert!(img.bytes.chunks(4).all(|px| px == [255, 255, 255, 255]));
        // Loud-not-black 1x1 default for future procedural paths.
        let img = PngImage::fallback_magenta();
        assert_eq!((img.width, img.height), (1, 1));
        assert_eq!(img.bytes, vec![255, 0, 255, 255]);
    }

    #[test]
    #[should_panic]
    fn garbage_rejected() {
        PngImage::decode(b"definitely not a png");
    }
}

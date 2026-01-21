use std::io::Cursor;

#[derive(Debug, Clone)]
pub struct Texture {
    pub(crate) width: u32,
    pub(crate) height: u32,
    pub(crate) image_bytes: Vec<u8>,
}

impl Texture {
    pub fn load_png(png_bytes: &[u8]) -> Self {
        let decoder = png::Decoder::new(Cursor::new(png_bytes));
        let mut reader = decoder.read_info().expect("Failed to decode PNG info");
        let mut buf = vec![0; reader.output_buffer_size().unwrap()];
        let info = reader.next_frame(&mut buf).expect("Failed to decode PNG");
        let image_bytes = buf[..info.buffer_size()].to_vec();

        Self {
            width: info.width,
            height: info.height,
            image_bytes,
        }
    }
}

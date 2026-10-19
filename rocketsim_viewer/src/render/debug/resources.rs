use glow::HasContext;
use rocketsim_vis::debug::{MeshResource, TextureResource};

use crate::render::gl_util::bytes_of;

use super::{GpuMesh, GpuTexture};

pub(super) unsafe fn upload_mesh(gl: &glow::Context, mesh: &MeshResource) -> GpuMesh {
    let vao = unsafe { gl.create_vertex_array().expect("debug mesh VAO") };
    let mut buffers = Vec::with_capacity(3);
    unsafe {
        gl.bind_vertex_array(Some(vao));
        let pos = gl.create_buffer().expect("debug positions");
        buffers.push(pos);
        gl.bind_buffer(glow::ARRAY_BUFFER, Some(pos));
        gl.buffer_data_u8_slice(
            glow::ARRAY_BUFFER,
            bytes_of(&mesh.positions),
            glow::STATIC_DRAW,
        );
        gl.enable_vertex_attrib_array(0);
        gl.vertex_attrib_pointer_f32(0, 3, glow::FLOAT, false, 12, 0);
        let generated_normals = mesh.normals.is_none().then(|| recompute_normals(mesh));
        let normals = mesh
            .normals
            .as_deref()
            .or(generated_normals.as_deref())
            .expect("generated normals");
        let normal = gl.create_buffer().expect("debug normals");
        buffers.push(normal);
        gl.bind_buffer(glow::ARRAY_BUFFER, Some(normal));
        gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, bytes_of(normals), glow::STATIC_DRAW);
        gl.enable_vertex_attrib_array(1);
        gl.vertex_attrib_pointer_f32(1, 3, glow::FLOAT, false, 12, 0);
        let colors: Vec<[f32; 4]> = mesh.colors.iter().map(|color| color.to_array()).collect();
        let color = gl.create_buffer().expect("debug colors");
        buffers.push(color);
        gl.bind_buffer(glow::ARRAY_BUFFER, Some(color));
        gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, bytes_of(&colors), glow::STATIC_DRAW);
        gl.enable_vertex_attrib_array(2);
        gl.vertex_attrib_pointer_f32(2, 4, glow::FLOAT, false, 16, 0);
        let generated_uvs = mesh
            .uvs
            .is_none()
            .then(|| vec![[0.0; 2]; mesh.positions.len()]);
        let uvs = mesh
            .uvs
            .as_deref()
            .or(generated_uvs.as_deref())
            .expect("generated UVs");
        let uv_buffer = gl.create_buffer().expect("debug UVs");
        gl.bind_buffer(glow::ARRAY_BUFFER, Some(uv_buffer));
        gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, bytes_of(uvs), glow::STATIC_DRAW);
        gl.enable_vertex_attrib_array(3);
        gl.vertex_attrib_pointer_f32(3, 2, glow::FLOAT, false, 8, 0);
        let index = gl.create_buffer().expect("debug indices");
        gl.bind_buffer(glow::ELEMENT_ARRAY_BUFFER, Some(index));
        gl.buffer_data_u8_slice(
            glow::ELEMENT_ARRAY_BUFFER,
            bytes_of(&mesh.indices),
            glow::STATIC_DRAW,
        );
        gl.bind_vertex_array(None);
        GpuMesh {
            vao,
            buffers: [buffers[0], buffers[1], buffers[2], uv_buffer],
            index,
            count: mesh.indices.len() as i32,
            revision: mesh.revision,
            builtin: false,
            transparent: mesh.colors.iter().any(|color| color.a < 1.0),
            center: mesh_center(mesh),
        }
    }
}

pub(super) fn recompute_normals(mesh: &MeshResource) -> Vec<[f32; 3]> {
    let mut normals = vec![glam::Vec3A::ZERO; mesh.positions.len()];
    for tri in mesh.indices.as_chunks::<3>().0 {
        let a = glam::Vec3A::from_array(mesh.positions[tri[0] as usize]);
        let b = glam::Vec3A::from_array(mesh.positions[tri[1] as usize]);
        let c = glam::Vec3A::from_array(mesh.positions[tri[2] as usize]);
        let normal = (b - a).cross(c - a);
        for &index in tri {
            normals[index as usize] += normal;
        }
    }
    normals
        .into_iter()
        .map(|normal| normal.normalize_or(glam::Vec3A::Z).to_array())
        .collect()
}

pub(super) unsafe fn upload_texture(gl: &glow::Context, texture: &TextureResource) -> GpuTexture {
    unsafe {
        let handle = gl.create_texture().expect("debug texture");
        gl.bind_texture(glow::TEXTURE_2D, Some(handle));
        gl.tex_parameter_i32(
            glow::TEXTURE_2D,
            glow::TEXTURE_MIN_FILTER,
            glow::LINEAR as i32,
        );
        gl.tex_parameter_i32(
            glow::TEXTURE_2D,
            glow::TEXTURE_MAG_FILTER,
            glow::LINEAR as i32,
        );
        gl.pixel_store_i32(glow::UNPACK_ALIGNMENT, 1);
        gl.tex_image_2d(
            glow::TEXTURE_2D,
            0,
            glow::RGBA8 as i32,
            texture.width as i32,
            texture.height as i32,
            0,
            glow::RGBA,
            glow::UNSIGNED_BYTE,
            glow::PixelUnpackData::Slice(Some(&texture.rgba)),
        );
        GpuTexture {
            handle,
            revision: texture.revision,
            builtin: false,
            transparent: texture
                .rgba
                .as_chunks::<4>()
                .0
                .iter()
                .any(|pixel| pixel[3] < 255),
        }
    }
}

pub(super) unsafe fn delete_mesh(gl: &glow::Context, mesh: GpuMesh) {
    unsafe {
        gl.delete_vertex_array(mesh.vao);
        for buffer in mesh.buffers {
            gl.delete_buffer(buffer);
        }
        gl.delete_buffer(mesh.index);
    }
}

fn mesh_center(mesh: &MeshResource) -> glam::Vec3A {
    let mut min = glam::Vec3A::splat(f32::INFINITY);
    let mut max = glam::Vec3A::splat(f32::NEG_INFINITY);
    for position in &mesh.positions {
        let position = glam::Vec3A::from_array(*position);
        min = min.min(position);
        max = max.max(position);
    }
    (min + max) * 0.5
}

pub(super) unsafe fn upload_ordered_instances(
    gl: &glow::Context,
    buffer: glow::Buffer,
    capacity: &mut usize,
    data: &[f32],
) -> usize {
    let bytes = bytes_of(data);
    unsafe {
        gl.bind_buffer(glow::ARRAY_BUFFER, Some(buffer));
        if bytes.len() > *capacity {
            *capacity = bytes.len().next_power_of_two();
            gl.buffer_data_size(glow::ARRAY_BUFFER, *capacity as i32, glow::DYNAMIC_DRAW);
        }
        if !bytes.is_empty() {
            gl.buffer_sub_data_u8_slice(glow::ARRAY_BUFFER, 0, bytes);
        }
        gl.bind_buffer(glow::ARRAY_BUFFER, None);
    }
    bytes.len()
}

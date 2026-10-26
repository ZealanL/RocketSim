#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("debug-render-bench requires Linux and a surfaceless EGL driver");
    std::process::exit(1);
}

#[cfg(target_os = "linux")]
fn main() {
    native::main();
}

#[cfg(target_os = "linux")]
mod native {
    use std::{
        ffi::{CString, c_char, c_int, c_uint, c_void},
        fs::File,
        io::BufWriter,
        ptr,
        sync::Arc,
    };

    use clap::Parser;
    use glam::{Mat4, Vec3, Vec3A};
    use glow::HasContext;
    use rocketsim_viewer::{
        geometry::lines::{Color as LineColor, Segment},
        render::{debug::DebugGl, lines::LinesGl, view::view_proj},
    };
    use rocketsim_vis::debug::{
        BuiltinMesh, BuiltinTexture, Color, DebugItem, DebugRenderer, DebugScene, DepthMode,
        DrawList, GroupOptions, MeshInstanceItem, MeshResource, PointItem, PolylineItem, Width,
    };

    const W: u32 = 1280;
    const H: u32 = 720;
    const EGL_NONE: c_int = 0x3038;
    const EGL_SURFACE_TYPE: c_int = 0x3033;
    const EGL_PBUFFER_BIT: c_int = 1;
    const EGL_RENDERABLE_TYPE: c_int = 0x3040;
    const EGL_OPENGL_BIT: c_int = 8;
    const EGL_RED_SIZE: c_int = 0x3024;
    const EGL_GREEN_SIZE: c_int = 0x3023;
    const EGL_BLUE_SIZE: c_int = 0x3022;
    const EGL_ALPHA_SIZE: c_int = 0x3021;
    const EGL_DEPTH_SIZE: c_int = 0x3025;
    const EGL_WIDTH: c_int = 0x3057;
    const EGL_HEIGHT: c_int = 0x3056;
    const EGL_OPENGL_API: c_uint = 0x30A2;
    const EGL_PLATFORM_SURFACELESS_MESA: c_uint = 0x31DD;
    type Display = *mut c_void;
    type Config = *mut c_void;
    type Surface = *mut c_void;
    type Context = *mut c_void;
    #[link(name = "EGL")]
    unsafe extern "C" {
        fn eglGetProcAddress(name: *const c_char) -> *const c_void;
        fn eglInitialize(d: Display, major: *mut c_int, minor: *mut c_int) -> u32;
        fn eglBindAPI(api: c_uint) -> u32;
        fn eglChooseConfig(
            d: Display,
            attrs: *const c_int,
            configs: *mut Config,
            size: c_int,
            count: *mut c_int,
        ) -> u32;
        fn eglCreatePbufferSurface(d: Display, c: Config, attrs: *const c_int) -> Surface;
        fn eglCreateContext(d: Display, c: Config, share: Context, attrs: *const c_int) -> Context;
        fn eglMakeCurrent(d: Display, draw: Surface, read: Surface, ctx: Context) -> u32;
        fn eglDestroyContext(d: Display, c: Context) -> u32;
        fn eglDestroySurface(d: Display, s: Surface) -> u32;
        fn eglTerminate(d: Display) -> u32;
    }

    struct Egl {
        d: Display,
        s: Surface,
        c: Context,
    }
    impl Egl {
        fn new() -> Result<Self, String> {
            unsafe {
                let get = eglGetProcAddress(c"eglGetPlatformDisplayEXT".as_ptr());
                if get.is_null() {
                    return Err("EGL surfaceless platform entry point is unavailable".into());
                }
                let get: unsafe extern "C" fn(c_uint, *mut c_void, *const c_int) -> Display =
                    std::mem::transmute(get);
                let d = get(EGL_PLATFORM_SURFACELESS_MESA, ptr::null_mut(), ptr::null());
                if d.is_null() {
                    return Err(
                        "eglGetPlatformDisplay(EGL_PLATFORM_SURFACELESS_MESA) failed".into(),
                    );
                }
                let (mut maj, mut min) = (0, 0);
                if eglInitialize(d, &mut maj, &mut min) == 0 {
                    return Err(
                        "eglInitialize failed; check headless EGL driver availability".into(),
                    );
                }
                if eglBindAPI(EGL_OPENGL_API) == 0 {
                    eglTerminate(d);
                    return Err("eglBindAPI(EGL_OPENGL_API) failed".into());
                }
                let attrs = [
                    EGL_SURFACE_TYPE,
                    EGL_PBUFFER_BIT,
                    EGL_RENDERABLE_TYPE,
                    EGL_OPENGL_BIT,
                    EGL_RED_SIZE,
                    8,
                    EGL_GREEN_SIZE,
                    8,
                    EGL_BLUE_SIZE,
                    8,
                    EGL_ALPHA_SIZE,
                    8,
                    EGL_DEPTH_SIZE,
                    24,
                    EGL_NONE,
                ];
                let (mut config, mut count) = (ptr::null_mut(), 0);
                if eglChooseConfig(d, attrs.as_ptr(), &mut config, 1, &mut count) == 0 || count == 0
                {
                    eglTerminate(d);
                    return Err("EGL has no RGBA8/depth24 OpenGL pbuffer config".into());
                }
                let p = [EGL_WIDTH, W as c_int, EGL_HEIGHT, H as c_int, EGL_NONE];
                let s = eglCreatePbufferSurface(d, config, p.as_ptr());
                if s.is_null() {
                    eglTerminate(d);
                    return Err("eglCreatePbufferSurface failed".into());
                }
                let ca = [0x3098, 3, 0x30FB, 3, 0x30FD, 1, EGL_NONE];
                let c = eglCreateContext(d, config, ptr::null_mut(), ca.as_ptr());
                if c.is_null() {
                    eglDestroySurface(d, s);
                    eglTerminate(d);
                    return Err("EGL could not create an OpenGL 3.3 core context".into());
                }
                if eglMakeCurrent(d, s, s, c) == 0 {
                    eglDestroyContext(d, c);
                    eglDestroySurface(d, s);
                    eglTerminate(d);
                    return Err("eglMakeCurrent failed".into());
                }
                Ok(Self { d, s, c })
            }
        }
    }
    impl Drop for Egl {
        fn drop(&mut self) {
            unsafe {
                eglMakeCurrent(self.d, ptr::null_mut(), ptr::null_mut(), ptr::null_mut());
                eglDestroyContext(self.d, self.c);
                eglDestroySurface(self.d, self.s);
                eglTerminate(self.d);
            }
        }
    }

    #[derive(Parser)]
    #[command(about = "Headless EGL benchmark for the retained DebugGl renderer")]
    struct Args {
        #[arg(long, default_value_t = 50_000)]
        lines: usize,
        #[arg(long, default_value_t = 10_000)]
        points: usize,
        #[arg(long, default_value_t = 5_000)]
        instances: usize,
        #[arg(long, default_value_t = 180)]
        frames: usize,
        #[arg(long, default_value_t = 30)]
        warmup: usize,
        #[arg(long, default_value = "target/input-review/debug-render/native.png")]
        output: String,
        #[arg(long)]
        json: Option<String>,
        #[arg(long, help = "Run small GPU ordering and depth regression fixtures")]
        check_ordering: bool,
        #[arg(
            long,
            help = "Rebuild group geometry buffers every frame for a same-scene baseline"
        )]
        rebuild: bool,
    }
    fn ensure(cond: bool, msg: &str) -> Result<(), String> {
        if cond { Ok(()) } else { Err(msg.into()) }
    }
    pub fn main() {
        if let Err(e) = run() {
            eprintln!("debug-render-bench: {e}");
            std::process::exit(1);
        }
    }
    fn run() -> Result<(), String> {
        let a = Args::parse();

        ensure(
            (1..=10_000).contains(&a.frames) && (1..=10_000).contains(&a.warmup),
            "frames and warmup must each be 1..10000",
        )?;
        ensure(
            a.lines <= 1_000_000 && a.points <= 60_000 && a.instances <= 60_000,
            "lines must be <=1000000; points and instances must be <=60000",
        )?;
        let line_groups = a.lines.div_ceil(4095);
        ensure(
            line_groups
                .saturating_add(a.points)
                .saturating_add(a.instances)
                <= 65_536,
            "group item count exceeds DebugScene limit 65536",
        )?;
        let est = a
            .lines
            .saturating_mul(60)
            .saturating_add(a.points.saturating_mul(48))
            .saturating_add(a.instances.saturating_mul(112));
        ensure(
            est <= 8 * 1024 * 1024,
            "estimated workload exceeds DebugScene 8 MiB limit",
        )?;
        let egl = Egl::new()?;
        let gl = unsafe {
            glow::Context::from_loader_function(|name| {
                CString::new(name)
                    .ok()
                    .map_or(ptr::null(), |s| eglGetProcAddress(s.as_ptr()))
            })
        };
        let version = unsafe { gl.get_parameter_string(glow::VERSION) };
        let vendor = unsafe { gl.get_parameter_string(glow::RENDERER) };
        eprintln!("OpenGL renderer: {vendor}; version: {version}");
        if a.check_ordering {
            return check_opaque_occlusion(&gl, &a.output);
        }
        let (renderer, scene) = make_scene(&a)?;
        let mut gpu = DebugGl::new(&gl);
        gpu.set_profiling(true);
        let eye = Vec3::new(-5000.0, -6000.0, 4500.0);
        let target = Vec3::new(0.0, 0.0, 500.0);
        let forward = (target - eye).normalize();
        let right = forward.cross(Vec3::Z).normalize();
        let up = right.cross(forward);
        let view = Mat4::from_cols(
            glam::Vec4::new(right.x, up.x, -forward.x, 0.0),
            glam::Vec4::new(right.y, up.y, -forward.y, 0.0),
            glam::Vec4::new(right.z, up.z, -forward.z, 0.0),
            glam::Vec4::new(-right.dot(eye), -up.dot(eye), forward.dot(eye), 1.0),
        );
        let f = 1.0 / (60_f32.to_radians() * 0.5).tan();
        let aspect = W as f32 / H as f32;
        let near = 10.0;
        let far = 30_000.0;
        let proj = Mat4::from_cols(
            glam::Vec4::new(f / aspect, 0.0, 0.0, 0.0),
            glam::Vec4::new(0.0, f, 0.0, 0.0),
            glam::Vec4::new(0.0, 0.0, far / (near - far), -1.0),
            glam::Vec4::new(0.0, 0.0, near * far / (near - far), 0.0),
        );
        let mut cpu = Vec::new();
        let mut gpu_ms = Vec::new();
        let mut warmup_uploads = None;
        let mut wall_ms = Vec::new();
        for frame in 0..a.warmup + a.frames {
            unsafe {
                gl.viewport(0, 0, W as i32, H as i32);
                gl.clear_color(0.025, 0.035, 0.055, 1.0);
                gl.clear(glow::COLOR_BUFFER_BIT | glow::DEPTH_BUFFER_BIT);
            }
            let frame_scene = if a.rebuild {
                with_geometry_revision(&scene, frame as u64 + 1)
            } else {
                Arc::clone(&scene)
            };
            let wall_started = std::time::Instant::now();
            gpu.sync(&gl, &frame_scene);
            gpu.draw(&gl, &frame_scene, None, &view, &proj, [W as f32, H as f32]);
            unsafe {
                gl.finish();
            }
            let s = gpu.stats();
            if a.warmup > 0 && frame + 1 == a.warmup {
                warmup_uploads = Some((s.mesh_uploads, s.line_uploads, s.upload_bytes));
            }
            if frame >= a.warmup {
                wall_ms.push(wall_started.elapsed().as_secs_f64() * 1000.0);
                cpu.push(s.cpu_ms);
                if let Some(v) = s.gpu_ms {
                    gpu_ms.push(v);
                }
            }
            let err = unsafe { gl.get_error() };
            ensure(
                err == glow::NO_ERROR,
                &format!("OpenGL error 0x{err:04x} on frame {frame}"),
            )?;
        }
        let mut rgba = vec![0_u8; (W * H * 4) as usize];
        unsafe {
            gl.pixel_store_i32(glow::PACK_ALIGNMENT, 1);
            gl.read_pixels(
                0,
                0,
                W as i32,
                H as i32,
                glow::RGBA,
                glow::UNSIGNED_BYTE,
                glow::PixelPackData::Slice(Some(&mut rgba[..])),
            );
        }
        ensure(
            rgba.as_chunks::<4>().0.iter().all(|p| p[3] == 255),
            "readback has non-opaque alpha pixels",
        )?;
        let background = [rgba[0], rgba[1], rgba[2]];
        let nonbg = rgba
            .as_chunks::<4>()
            .0
            .iter()
            .filter(|p| p[0..3] != background)
            .count();
        ensure(nonbg > 0, "render readback is entirely background")?;
        std::fs::create_dir_all(
            std::path::Path::new(&a.output)
                .parent()
                .unwrap_or(std::path::Path::new(".")),
        )
        .map_err(|e| e.to_string())?;
        let file = BufWriter::new(File::create(&a.output).map_err(|e| e.to_string())?);
        let mut enc = png::Encoder::new(file, W, H);
        enc.set_color(png::ColorType::Rgba);
        enc.set_depth(png::BitDepth::Eight);
        let mut writer = enc.write_header().map_err(|e| e.to_string())?;
        let stride = W as usize * 4;
        let mut flipped = Vec::with_capacity(rgba.len());
        for row in (0..H as usize).rev() {
            flipped.extend_from_slice(&rgba[row * stride..(row + 1) * stride]);
        }
        writer
            .write_image_data(&flipped)
            .map_err(|e| e.to_string())?;
        let s = gpu.stats();
        let (warm_mesh, warm_lines, warm_bytes) = warmup_uploads.ok_or_else(|| {
            "warmup must be at least one frame to measure upload deltas".to_string()
        })?;
        let mesh_uploads_delta = s.mesh_uploads - warm_mesh;
        let line_uploads_delta = s.line_uploads - warm_lines;
        let upload_bytes_delta = s.upload_bytes - warm_bytes;
        if !a.rebuild {
            ensure(
                mesh_uploads_delta == 0 && line_uploads_delta == 0 && upload_bytes_delta == 0,
                "cached workload uploaded resources after warmup",
            )?;
        }
        println!(
            "mode={} frames={} warmup={} line_segments={} points={} instances={} draw_calls={} uploads={} upload_bytes={} mesh_uploads_delta={} line_uploads_delta={} upload_bytes_delta={} cpu_ms_mean={:.3} cpu_ms_p50={:.3} cpu_ms_p95={:.3} gpu_ms_mean={} wall_total_ms_mean={:.3} non_background_pixels={} image={}",
            if a.rebuild { "rebuild" } else { "cached" },
            a.frames,
            a.warmup,
            a.lines,
            a.points,
            a.instances,
            s.draw_calls,
            s.mesh_uploads + s.line_uploads,
            s.upload_bytes,
            mesh_uploads_delta,
            line_uploads_delta,
            upload_bytes_delta,
            mean(&cpu),
            percentile(&mut cpu.clone(), 0.50),
            percentile(&mut cpu.clone(), 0.95),
            if gpu_ms.is_empty() {
                "n/a".to_string()
            } else {
                format!("{:.3}", mean(&gpu_ms))
            },
            mean(&wall_ms),
            nonbg,
            a.output
        );
        if let Some(path) = a.json {
            let data = format!(
                "{{\"renderer\":\"{}\",\"version\":\"{}\",\"mode\":\"{}\",\"frames\":{},\"warmup\":{},\"lines\":{},\"points\":{},\"instances\":{},\"draw_calls\":{},\"mesh_uploads\":{},\"line_uploads\":{},\"upload_bytes\":{},\"mesh_uploads_delta\":{},\"line_uploads_delta\":{},\"upload_bytes_delta\":{},\"wall_total_ms_mean\":{:.6},\"cpu_ms_mean\":{:.6},\"cpu_ms_p50\":{:.6},\"cpu_ms_p95\":{:.6},\"gpu_ms_mean\":{},\"non_background_pixels\":{},\"image\":\"{}\"}}",
                escape(&vendor),
                escape(&version),
                if a.rebuild { "rebuild" } else { "cached" },
                a.frames,
                a.warmup,
                a.lines,
                a.points,
                a.instances,
                s.draw_calls,
                s.mesh_uploads,
                s.line_uploads,
                s.upload_bytes,
                mesh_uploads_delta,
                line_uploads_delta,
                upload_bytes_delta,
                mean(&wall_ms),
                mean(&cpu),
                percentile(&mut cpu.clone(), 0.50),
                percentile(&mut cpu.clone(), 0.95),
                if gpu_ms.is_empty() {
                    "null".to_string()
                } else {
                    format!("{:.6}", mean(&gpu_ms))
                },
                nonbg,
                escape(&a.output)
            );
            std::fs::write(path, data).map_err(|e| e.to_string())?;
        }
        gpu.destroy(&gl);
        drop(egl);
        let _ = renderer;
        Ok(())
    }
    fn check_opaque_occlusion(gl: &glow::Context, output: &str) -> Result<(), String> {
        use rocketsim_vis::debug::{Anchor, Lifetime};
        let mesh = MeshResource::new(
            "ordering-quad",
            vec![
                [-100.0, 0.0, 200.0],
                [100.0, 0.0, 200.0],
                [100.0, 0.0, 400.0],
                [-100.0, 0.0, 400.0],
            ],
            vec![Color::WHITE; 4],
            Some(vec![[0.0, -1.0, 0.0]; 4]),
            None,
            vec![0, 2, 1, 0, 3, 2],
        )
        .map_err(|e| e.to_string())?;
        let renderer = DebugRenderer::new_detached();
        renderer.insert_mesh(mesh).map_err(|e| e.to_string())?;
        let instance = |position, tint| {
            DebugItem::Instance(MeshInstanceItem {
                mesh: "ordering-quad".into(),
                position,
                rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                scale: [1.0; 3],
                tint,
                texture: None,
                depth: DepthMode::Test,
            })
        };
        let eye = Vec3::new(0.0, -1000.0, 300.0);
        let forward = Vec3::Y;
        let right = forward.cross(Vec3::Z).normalize();
        let up = right.cross(forward);
        let view = Mat4::from_cols(
            glam::Vec4::new(right.x, up.x, -forward.x, 0.0),
            glam::Vec4::new(right.y, up.y, -forward.y, 0.0),
            glam::Vec4::new(right.z, up.z, -forward.z, 0.0),
            glam::Vec4::new(-right.dot(eye), -up.dot(eye), forward.dot(eye), 1.0),
        );
        let f = 1.0 / (60_f32.to_radians() * 0.5).tan();
        let near = 1.0;
        let far = 5000.0;
        let proj = Mat4::from_cols(
            glam::Vec4::new(f / (W as f32 / H as f32), 0.0, 0.0, 0.0),
            glam::Vec4::new(0.0, f, 0.0, 0.0),
            glam::Vec4::new(0.0, 0.0, far / (near - far), -1.0),
            glam::Vec4::new(0.0, 0.0, near * far / (near - far), 0.0),
        );
        let opaque = instance([0.0, 0.0, 0.0], Color::rgb(1.0, 0.0, 0.0));
        let transparent = instance([0.0, 100.0, 0.0], Color::rgba(0.0, 0.0, 1.0, 0.5));
        let render = |items: Vec<DebugItem>| -> Result<(Vec<u8>, [u8; 4]), String> {
            renderer
                .replace_group(
                    "ordering",
                    DrawList { items },
                    GroupOptions {
                        anchor: Anchor::World,
                        lifetime: Lifetime::Persistent,
                        ..Default::default()
                    },
                )
                .map_err(|e| e.to_string())?;
            let scene = renderer.snapshot();
            let mut gpu = DebugGl::new(gl);
            gpu.sync(gl, &scene);
            unsafe {
                gl.viewport(0, 0, W as i32, H as i32);
                gl.depth_mask(true);
                gl.clear_color(0.025, 0.035, 0.055, 1.0);
                gl.clear(glow::COLOR_BUFFER_BIT | glow::DEPTH_BUFFER_BIT);
            }
            gpu.draw(gl, &scene, None, &view, &proj, [W as f32, H as f32]);
            unsafe {
                gl.finish();
            }
            let mut rgba = vec![0_u8; (W * H * 4) as usize];
            unsafe {
                gl.pixel_store_i32(glow::PACK_ALIGNMENT, 1);
                gl.read_pixels(
                    0,
                    0,
                    W as i32,
                    H as i32,
                    glow::RGBA,
                    glow::UNSIGNED_BYTE,
                    glow::PixelPackData::Slice(Some(&mut rgba)),
                );
            }
            let err = unsafe { gl.get_error() };
            ensure(
                err == glow::NO_ERROR,
                &format!("OpenGL error 0x{err:04x} in opaque occlusion fixture"),
            )?;
            ensure(
                rgba.as_chunks::<4>().0.iter().all(|p| p[3] == 255),
                "opaque occlusion readback contains non-opaque alpha",
            )?;
            let offset = ((H as usize / 2) * W as usize + W as usize / 2) * 4;
            let center = rgba[offset..offset + 4].try_into().unwrap();
            gpu.destroy(gl);
            Ok((rgba, center))
        };
        let (_, expected) = render(vec![opaque.clone()])?;
        for (name, items) in [
            ("opaque-first", vec![opaque.clone(), transparent.clone()]),
            ("transparent-first", vec![transparent, opaque.clone()]),
        ] {
            let (rgba, actual) = render(items)?;
            ensure(
                actual == expected,
                &format!(
                    "{name}: center pixel {actual:?} differs from opaque-only reference {expected:?}"
                ),
            )?;
            println!("PASS {name}: center RGBA {actual:?} matches opaque-only reference");
            if name == "transparent-first" {
                save_rgba_png(output, &rgba)?;
            }
        }
        check_line_depth(gl, output, &renderer, &opaque, expected)?;
        check_ribbon_rendering(gl, &renderer, &opaque, expected)?;
        check_transparent_ordering(gl, output)?;
        check_builtin_assets(gl)?;
        Ok(())
    }
    fn check_line_depth(
        gl: &glow::Context,
        _output: &str,
        renderer: &DebugRenderer,
        opaque: &DebugItem,
        reference: [u8; 4],
    ) -> Result<(), String> {
        use rocketsim_vis::debug::{Anchor, Lifetime};
        let eye = Vec3::new(0.0, -1000.0, 300.0);
        let forward = Vec3::Y;
        let right = forward.cross(Vec3::Z).normalize();
        let up = right.cross(forward);
        let view = Mat4::from_cols(
            glam::Vec4::new(right.x, up.x, -forward.x, 0.0),
            glam::Vec4::new(right.y, up.y, -forward.y, 0.0),
            glam::Vec4::new(right.z, up.z, -forward.z, 0.0),
            glam::Vec4::new(-right.dot(eye), -up.dot(eye), forward.dot(eye), 1.0),
        );
        let f = 1.0 / (60_f32.to_radians() * 0.5).tan();
        let (near, far) = (1.0, 5000.0);
        let proj = Mat4::from_cols(
            glam::Vec4::new(f / (W as f32 / H as f32), 0.0, 0.0, 0.0),
            glam::Vec4::new(0.0, f, 0.0, 0.0),
            glam::Vec4::new(0.0, 0.0, far / (near - far), -1.0),
            glam::Vec4::new(0.0, 0.0, near * far / (near - far), 0.0),
        );
        let mut gpu = DebugGl::new(gl);
        for (mode, name) in [(DepthMode::Test, "Test"), (DepthMode::Always, "Always")] {
            for line_first in [false, true] {
                let line = DebugItem::Polyline(PolylineItem {
                    points: vec![[-100.0, 100.0, 300.0], [100.0, 100.0, 300.0]],
                    color: Color::rgb(0.0, 1.0, 0.0),
                    width: Width::Pixels(16.0),
                    depth: mode,
                    closed: false,
                });
                let items = if line_first {
                    vec![line, opaque.clone()]
                } else {
                    vec![opaque.clone(), line]
                };
                renderer
                    .replace_group(
                        "ordering",
                        DrawList { items },
                        GroupOptions {
                            anchor: Anchor::World,
                            lifetime: Lifetime::Persistent,
                            ..Default::default()
                        },
                    )
                    .map_err(|e| e.to_string())?;
                let scene = renderer.snapshot();
                gpu.sync(gl, &scene);
                unsafe {
                    gl.viewport(0, 0, W as i32, H as i32);
                    gl.depth_mask(true);
                    gl.clear_color(0.025, 0.035, 0.055, 1.0);
                    gl.clear(glow::COLOR_BUFFER_BIT | glow::DEPTH_BUFFER_BIT);
                }
                gpu.draw(gl, &scene, None, &view, &proj, [W as f32, H as f32]);
                unsafe {
                    gl.finish();
                }
                let mut rgba = vec![0_u8; (W * H * 4) as usize];
                unsafe {
                    gl.pixel_store_i32(glow::PACK_ALIGNMENT, 1);
                    gl.read_pixels(
                        0,
                        0,
                        W as i32,
                        H as i32,
                        glow::RGBA,
                        glow::UNSIGNED_BYTE,
                        glow::PixelPackData::Slice(Some(&mut rgba)),
                    );
                }
                let err = unsafe { gl.get_error() };
                ensure(
                    err == glow::NO_ERROR,
                    &format!("OpenGL error 0x{err:04x} in line depth fixture"),
                )?;
                ensure(
                    rgba.as_chunks::<4>().0.iter().all(|p| p[3] == 255),
                    "line depth readback contains non-opaque alpha",
                )?;
                let offset = ((H as usize / 2) * W as usize + W as usize / 2) * 4;
                let center: [u8; 4] = rgba[offset..offset + 4].try_into().unwrap();
                if mode == DepthMode::Test {
                    ensure(
                        center == reference,
                        &format!(
                            "DepthMode::Test line should be hidden: center {center:?}, opaque-only reference {reference:?}"
                        ),
                    )?;
                } else {
                    ensure(
                        center[1] > 128 && center[0] < 32,
                        &format!("DepthMode::Always line should be visible: center {center:?}"),
                    )?;
                }
                println!("PASS line-depth-{name}-line_first={line_first}: center RGBA {center:?}");
            }
        }
        gpu.destroy(gl);
        Ok(())
    }

    fn render_ribbon_frame(
        gl: &glow::Context,
        lines: &LinesGl,
        debug: Option<(&mut DebugGl, &DebugScene)>,
        view: &Mat4,
        proj: &Mat4,
    ) -> Result<Vec<u8>, String> {
        unsafe {
            gl.viewport(0, 0, W as i32, H as i32);
            gl.depth_mask(true);
            gl.clear_color(0.025, 0.035, 0.055, 1.0);
            gl.clear(glow::COLOR_BUFFER_BIT | glow::DEPTH_BUFFER_BIT);
        }
        if let Some((debug, scene)) = debug {
            debug.draw(gl, scene, None, view, proj, [W as f32, H as f32]);
        }
        lines.draw(gl, view, proj);
        unsafe {
            gl.finish();
        }
        let error = unsafe { gl.get_error() };
        ensure(
            error == glow::NO_ERROR,
            &format!("OpenGL error 0x{error:04x} in ribbon fixture"),
        )?;
        let mut rgba = vec![0_u8; (W * H * 4) as usize];
        unsafe {
            gl.pixel_store_i32(glow::PACK_ALIGNMENT, 1);
            gl.read_pixels(
                0,
                0,
                W as i32,
                H as i32,
                glow::RGBA,
                glow::UNSIGNED_BYTE,
                glow::PixelPackData::Slice(Some(&mut rgba)),
            );
        }
        ensure(
            rgba.as_chunks::<4>().0.iter().all(|pixel| pixel[3] == 255),
            "ribbon readback contains non-opaque alpha",
        )?;
        Ok(rgba)
    }

    fn check_ribbon_rendering(
        gl: &glow::Context,
        renderer: &DebugRenderer,
        opaque: &DebugItem,
        opaque_reference: [u8; 4],
    ) -> Result<(), String> {
        let eye = Vec3::new(0.0, -1000.0, 300.0);
        let (view, proj) = view_proj(eye, Vec3::new(0.0, 0.0, 300.0), 90.0, W as f32 / H as f32);
        let xy = |point: Vec3| {
            let clip = proj * view * point.extend(1.0);
            let ndc = clip.truncate() / clip.w;
            (
                (((ndc.x * 0.5 + 0.5) * W as f32).floor() as i32).clamp(0, W as i32 - 1) as usize,
                (((ndc.y * 0.5 + 0.5) * H as f32).floor() as i32).clamp(0, H as i32 - 1) as usize,
            )
        };
        let pixel = |rgba: &[u8], point: Vec3| {
            let (x, y) = xy(point);
            rgba.as_chunks::<4>().0[y * W as usize + x]
        };
        let upload = |lines: &mut LinesGl, segments: &[Segment]| {
            lines.upload(
                gl,
                segments,
                Vec3A::from(eye),
                &view,
                proj.y_axis.y,
                H as f32,
            );
        };
        let color = LineColor::new(1.0, 1.0, 1.0, 0.5);
        let width = 60.0;
        let segment = |a: Vec3, b: Vec3| Segment {
            a: Vec3A::from(a),
            b: Vec3A::from(b),
            color_a: color,
            color_b: color,
            width_a: width,
            width_b: width,
            ribbon: true,
        };
        let segments = |points: &[Vec3]| {
            points
                .windows(2)
                .map(|pair| segment(pair[0], pair[1]))
                .collect::<Vec<_>>()
        };
        let straight_points = (0..=6)
            .map(|index| Vec3::new(-300.0 + index as f32 * 100.0, 100.0, 300.0))
            .collect::<Vec<_>>();
        let bent_points = [
            Vec3::new(-300.0, 100.0, 300.0),
            Vec3::new(-100.0, 100.0, 300.0),
            Vec3::new(100.0, 100.0, 380.0),
            Vec3::new(300.0, 100.0, 380.0),
        ];

        let mut lines = LinesGl::new(gl);
        let mut straight_reference = [0_u8; 4];
        for (name, points) in [
            ("straight", straight_points.as_slice()),
            ("bent", bent_points.as_slice()),
        ] {
            let ribbon = segments(points);
            upload(&mut lines, &ribbon);
            let rgba = render_ribbon_frame(gl, &lines, None, &view, &proj)?;
            let bg = [rgba[0], rgba[1], rgba[2]];
            let reference = pixel(&rgba, points[0].lerp(points[1], 0.5));
            ensure(
                reference[0..3] != bg,
                &format!("{name} ribbon did not render"),
            )?;
            if name == "straight" {
                straight_reference = reference;
            }
            let joins = points.len() - 2;
            for &point in &points[1..points.len() - 1] {
                let (x, y) = xy(point);
                for dy in -2_i32..=2 {
                    for dx in -2_i32..=2 {
                        let px = (x as i32 + dx).clamp(0, W as i32 - 1) as usize;
                        let py = (y as i32 + dy).clamp(0, H as i32 - 1) as usize;
                        let actual = rgba.as_chunks::<4>().0[py * W as usize + px];
                        ensure(
                            actual[3] == reference[3]
                                && actual[0..3]
                                    .iter()
                                    .zip(&reference[0..3])
                                    .all(|(a, b)| a.abs_diff(*b) <= 2),
                            &format!(
                                "{name} join at {point:?} has a seam at pixel ({px},{py}): {actual:?}, interior {reference:?}"
                            ),
                        )?;
                    }
                }
            }
            println!("PASS ribbon-{name}: {joins} joins match interior pixel {reference:?}");
        }

        let capped = Segment::flat(
            Vec3A::new(-100.0, 100.0, 300.0),
            Vec3A::new(100.0, 100.0, 300.0),
            color,
            width,
        );
        upload(&mut lines, &[capped]);
        let capped_rgba = render_ribbon_frame(gl, &lines, None, &view, &proj)?;
        let capped_bg = [capped_rgba[0], capped_rgba[1], capped_rgba[2]];
        let cap_inside = pixel(&capped_rgba, Vec3::new(115.0, 100.0, 315.0));
        let cap_outside = pixel(&capped_rgba, Vec3::new(128.0, 100.0, 328.0));
        ensure(
            cap_inside[0..3] != capped_bg,
            "round cap point did not render",
        )?;
        ensure(
            cap_outside[0..3] == capped_bg,
            "round cap discard did not trim the padded quad",
        )?;
        println!("PASS line-round-cap: inside {cap_inside:?}, outside {cap_outside:?}");

        renderer
            .replace_group(
                "ordering",
                DrawList {
                    items: vec![opaque.clone()],
                },
                GroupOptions::default(),
            )
            .map_err(|error| error.to_string())?;
        let scene = renderer.snapshot();
        let mut debug = DebugGl::new(gl);
        debug.sync(gl, &scene);
        upload(&mut lines, &segments(&straight_points));
        let occluded = render_ribbon_frame(gl, &lines, Some((&mut debug, &scene)), &view, &proj)?;
        let center_with_ribbon = pixel(&occluded, Vec3::new(0.0, 100.0, 300.0));
        ensure(
            center_with_ribbon[3] == opaque_reference[3]
                && center_with_ribbon[0..3]
                    .iter()
                    .zip(&opaque_reference[0..3])
                    .all(|(a, b)| a.abs_diff(*b) <= 2),
            &format!(
                "translucent ribbon changed the opaque occluder: {center_with_ribbon:?} vs {opaque_reference:?}"
            ),
        )?;
        let exposed = pixel(&occluded, Vec3::new(-250.0, 100.0, 300.0));
        ensure(
            exposed == straight_reference,
            &format!(
                "exposed ribbon pixel {exposed:?} differs from ribbon-only reference {straight_reference:?}"
            ),
        )?;
        println!("PASS ribbon-depth: occluded center {center_with_ribbon:?}, exposed {exposed:?}");

        debug.destroy(gl);
        lines.destroy(gl);
        Ok(())
    }

    fn check_transparent_ordering(gl: &glow::Context, output: &str) -> Result<(), String> {
        use rocketsim_vis::debug::{Anchor, Lifetime};
        let renderer = DebugRenderer::new_detached();
        let verts = vec![
            [-100.0, 0.0, 200.0],
            [100.0, 0.0, 200.0],
            [100.0, 0.0, 400.0],
            [-100.0, 0.0, 400.0],
        ];
        for id in ["order-a", "order-b"] {
            renderer
                .insert_mesh(
                    MeshResource::new(
                        id,
                        verts.clone(),
                        vec![Color::WHITE; 4],
                        Some(vec![[0.0, -1.0, 0.0]; 4]),
                        None,
                        vec![0, 2, 1, 0, 3, 2],
                    )
                    .map_err(|e| e.to_string())?,
                )
                .map_err(|e| e.to_string())?;
        }
        let make = |mesh: &str, y: f32, tint: Color| {
            DebugItem::Instance(MeshInstanceItem {
                mesh: mesh.into(),
                position: [0.0, y, 0.0],
                rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                scale: [1.0; 3],
                tint,
                texture: None,
                depth: DepthMode::Test,
            })
        };
        let items = [
            make("order-a", 200.0, Color::rgba(1.0, 0.0, 0.0, 0.5)),
            make("order-b", 100.0, Color::rgba(0.0, 1.0, 0.0, 0.5)),
            make("order-a", 0.0, Color::rgba(0.0, 0.0, 1.0, 0.5)),
        ];
        let permutations = [
            [0, 1, 2],
            [0, 2, 1],
            [1, 0, 2],
            [1, 2, 0],
            [2, 0, 1],
            [2, 1, 0],
        ];
        let mut gpu = DebugGl::new(gl);
        let projection = |aspect| {
            let f = 1.0 / (60_f32.to_radians() * 0.5).tan();
            let (near, far) = (1.0, 5000.0);
            Mat4::from_cols(
                glam::Vec4::new(f / aspect, 0.0, 0.0, 0.0),
                glam::Vec4::new(0.0, f, 0.0, 0.0),
                glam::Vec4::new(0.0, 0.0, far / (near - far), -1.0),
                glam::Vec4::new(0.0, 0.0, near * far / (near - far), 0.0),
            )
        };
        let render = |items: Vec<DebugItem>,
                      camera_y: f32,
                      gpu: &mut DebugGl,
                      clear: bool|
         -> Result<(Vec<u8>, [u8; 4]), String> {
            renderer
                .replace_group(
                    "order-scene",
                    DrawList { items },
                    GroupOptions {
                        anchor: Anchor::World,
                        lifetime: Lifetime::Persistent,
                        ..Default::default()
                    },
                )
                .map_err(|e| e.to_string())?;
            let scene = renderer.snapshot();
            let eye = Vec3::new(0.0, camera_y, 300.0);
            let forward = Vec3::new(0.0, -camera_y.signum(), 0.0);
            let right = forward.cross(Vec3::Z).normalize();
            let up = right.cross(forward);
            let view = Mat4::from_cols(
                glam::Vec4::new(right.x, up.x, -forward.x, 0.0),
                glam::Vec4::new(right.y, up.y, -forward.y, 0.0),
                glam::Vec4::new(right.z, up.z, -forward.z, 0.0),
                glam::Vec4::new(-right.dot(eye), -up.dot(eye), forward.dot(eye), 1.0),
            );
            gpu.sync(gl, &scene);
            unsafe {
                gl.viewport(0, 0, W as i32, H as i32);
                if clear {
                    gl.depth_mask(true);
                    gl.clear_color(0.025, 0.035, 0.055, 1.0);
                    gl.clear(glow::COLOR_BUFFER_BIT | glow::DEPTH_BUFFER_BIT);
                }
            }
            gpu.draw(
                gl,
                &scene,
                None,
                &view,
                &projection(W as f32 / H as f32),
                [W as f32, H as f32],
            );
            unsafe {
                gl.finish();
            }
            let mut rgba = vec![0_u8; (W * H * 4) as usize];
            unsafe {
                gl.pixel_store_i32(glow::PACK_ALIGNMENT, 1);
                gl.read_pixels(
                    0,
                    0,
                    W as i32,
                    H as i32,
                    glow::RGBA,
                    glow::UNSIGNED_BYTE,
                    glow::PixelPackData::Slice(Some(&mut rgba)),
                );
            }
            let err = unsafe { gl.get_error() };
            ensure(
                err == glow::NO_ERROR,
                &format!("OpenGL error 0x{err:04x} in transparent ordering fixture"),
            )?;
            ensure(
                rgba.as_chunks::<4>().0.iter().all(|p| p[3] == 255),
                "transparent ordering readback contains non-opaque alpha",
            )?;
            let offset = ((H as usize / 2) * W as usize + W as usize / 2) * 4;
            Ok((rgba.clone(), rgba[offset..offset + 4].try_into().unwrap()))
        };
        for camera_y in [-1000.0, 1000.0] {
            let order = if camera_y < 0.0 { [0, 1, 2] } else { [2, 1, 0] };
            let mut reference = Vec::new();
            for idx in order {
                let (_, pixel) = render(
                    vec![items[idx].clone()],
                    camera_y,
                    &mut gpu,
                    reference.is_empty(),
                )?;
                reference.push(pixel);
            }
            let expected = reference[2];
            for (n, permutation) in permutations.iter().enumerate() {
                let list = permutation.iter().map(|&i| items[i].clone()).collect();
                let (rgba, actual) = render(list, camera_y, &mut gpu, true)?;
                ensure(
                    actual == expected,
                    &format!(
                        "transparent permutation {n} from camera y={camera_y}: center {actual:?}, expected far-to-near {expected:?}"
                    ),
                )?;
                if n == permutations.len() - 1 {
                    save_rgba_png(output, &rgba)?;
                }
            }
            println!(
                "PASS transparent-order camera_y={camera_y}: 6 permutations match far-to-near reference {expected:?}"
            );
        }
        gpu.destroy(gl);
        Ok(())
    }
    fn check_builtin_assets(gl: &glow::Context) -> Result<(), String> {
        let make_items = |textured: bool| {
            let mut items = DrawList::new();
            let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
            for (mesh, position, texture) in [
                (
                    BuiltinMesh::Ball,
                    [-180.0, 0.0, 300.0],
                    BuiltinTexture::Ball,
                ),
                (
                    BuiltinMesh::CarOctane,
                    [180.0, 0.0, 300.0],
                    BuiltinTexture::CarBlue,
                ),
            ] {
                items.instance(
                    mesh.id(),
                    position,
                    identity,
                    [1.0; 3],
                    Color::WHITE,
                    textured.then(|| texture.id().to_owned()),
                    DepthMode::Test,
                );
            }
            items
        };

        let renderer = DebugRenderer::new_detached();
        renderer
            .replace_group("builtin-assets", make_items(true), GroupOptions::default())
            .map_err(|error| error.to_string())?;
        let scene = renderer.snapshot();
        ensure(
            scene.meshes.is_empty() && scene.textures.is_empty(),
            "built-in assets must not need uploaded resource definitions",
        )?;

        let mut gpu = DebugGl::new(gl);
        gpu.sync(gl, &scene);
        let uploaded = gpu.stats();
        ensure(
            uploaded.mesh_uploads == 2 && uploaded.texture_uploads == 2,
            &format!(
                "expected two lazy mesh and texture uploads, got {} meshes and {} textures",
                uploaded.mesh_uploads, uploaded.texture_uploads
            ),
        )?;

        let eye = Vec3::new(0.0, -1200.0, 300.0);
        let forward = Vec3::Y;
        let right = forward.cross(Vec3::Z).normalize();
        let up = right.cross(forward);
        let view = Mat4::from_cols(
            glam::Vec4::new(right.x, up.x, -forward.x, 0.0),
            glam::Vec4::new(right.y, up.y, -forward.y, 0.0),
            glam::Vec4::new(right.z, up.z, -forward.z, 0.0),
            glam::Vec4::new(-right.dot(eye), -up.dot(eye), forward.dot(eye), 1.0),
        );
        let f = 1.0 / (60_f32.to_radians() * 0.5).tan();
        let near = 1.0;
        let far = 5000.0;
        let proj = Mat4::from_cols(
            glam::Vec4::new(f / (W as f32 / H as f32), 0.0, 0.0, 0.0),
            glam::Vec4::new(0.0, f, 0.0, 0.0),
            glam::Vec4::new(0.0, 0.0, far / (near - far), -1.0),
            glam::Vec4::new(0.0, 0.0, near * far / (near - far), 0.0),
        );
        let render = |gpu: &mut DebugGl, scene: &Arc<DebugScene>| -> Result<Vec<u8>, String> {
            unsafe {
                gl.viewport(0, 0, W as i32, H as i32);
                gl.depth_mask(true);
                gl.clear_color(0.025, 0.035, 0.055, 1.0);
                gl.clear(glow::COLOR_BUFFER_BIT | glow::DEPTH_BUFFER_BIT);
            }
            gpu.draw(gl, scene, None, &view, &proj, [W as f32, H as f32]);
            unsafe {
                gl.finish();
            }
            let mut rgba = vec![0_u8; (W * H * 4) as usize];
            unsafe {
                gl.pixel_store_i32(glow::PACK_ALIGNMENT, 1);
                gl.read_pixels(
                    0,
                    0,
                    W as i32,
                    H as i32,
                    glow::RGBA,
                    glow::UNSIGNED_BYTE,
                    glow::PixelPackData::Slice(Some(&mut rgba)),
                );
            }
            let error = unsafe { gl.get_error() };
            ensure(
                error == glow::NO_ERROR,
                &format!("OpenGL error 0x{error:04x} in built-in asset fixture"),
            )?;
            ensure(
                rgba.as_chunks::<4>().0.iter().all(|pixel| pixel[3] == 255),
                "built-in asset readback contains non-opaque alpha",
            )?;
            Ok(rgba)
        };

        let textured = render(&mut gpu, &scene)?;
        let background = [textured[0], textured[1], textured[2]];
        let mut visible_halves = [0_usize; 2];
        for (index, pixel) in textured.as_chunks::<4>().0.iter().enumerate() {
            if pixel[0..3] != background {
                let half = usize::from(index % W as usize >= W as usize / 2);
                visible_halves[half] += 1;
            }
        }
        ensure(
            visible_halves.iter().all(|&count| count > 100),
            &format!("both built-in models must render visible pixels: {visible_halves:?}"),
        )?;

        gpu.sync(gl, &scene);
        let reused = gpu.stats();
        ensure(
            reused.mesh_uploads == uploaded.mesh_uploads
                && reused.texture_uploads == uploaded.texture_uploads,
            "reusing one scene uploaded built-in assets again",
        )?;
        ensure(
            render(&mut gpu, &scene)? == textured,
            "built-in rendering changed when the same scene cache was reused",
        )?;

        renderer
            .replace_group("builtin-assets", make_items(false), GroupOptions::default())
            .map_err(|error| error.to_string())?;
        let untextured_scene = renderer.snapshot();
        gpu.sync(gl, &untextured_scene);
        let untextured_stats = gpu.stats();
        ensure(
            untextured_stats.mesh_uploads == uploaded.mesh_uploads
                && untextured_stats.texture_uploads == uploaded.texture_uploads,
            "changing only built-in texture references uploaded resources again",
        )?;
        let untextured = render(&mut gpu, &untextured_scene)?;
        let mut texture_differences = [0_usize; 2];
        for (index, (with_texture, without_texture)) in textured
            .as_chunks::<4>()
            .0
            .iter()
            .zip(untextured.as_chunks::<4>().0)
            .enumerate()
        {
            if with_texture[0..3] != without_texture[0..3] {
                let half = usize::from(index % W as usize >= W as usize / 2);
                texture_differences[half] += 1;
            }
        }
        ensure(
            texture_differences.iter().all(|&count| count > 0),
            &format!("both built-in textures must affect visible pixels: {texture_differences:?}"),
        )?;
        println!(
            "PASS builtin-assets meshes={} textures={} visible_pixels={visible_halves:?} texture_pixels={texture_differences:?} cache_reused=true",
            uploaded.mesh_uploads, uploaded.texture_uploads
        );
        gpu.destroy(gl);
        Ok(())
    }

    fn save_rgba_png(output: &str, rgba: &[u8]) -> Result<(), String> {
        std::fs::create_dir_all(
            std::path::Path::new(output)
                .parent()
                .unwrap_or(std::path::Path::new(".")),
        )
        .map_err(|e| e.to_string())?;
        let file = BufWriter::new(File::create(output).map_err(|e| e.to_string())?);
        let mut enc = png::Encoder::new(file, W, H);
        enc.set_color(png::ColorType::Rgba);
        enc.set_depth(png::BitDepth::Eight);
        let mut writer = enc.write_header().map_err(|e| e.to_string())?;
        let mut flipped = Vec::with_capacity(rgba.len());
        for row in (0..H as usize).rev() {
            flipped.extend_from_slice(&rgba[row * W as usize * 4..(row + 1) * W as usize * 4]);
        }
        writer.write_image_data(&flipped).map_err(|e| e.to_string())
    }
    fn with_geometry_revision(
        scene: &Arc<rocketsim_vis::debug::DebugScene>,
        revision: u64,
    ) -> Arc<rocketsim_vis::debug::DebugScene> {
        let groups = scene
            .groups
            .iter()
            .map(|(name, group)| {
                let mut group = (**group).clone();
                group.geometry_revision = group.geometry_revision.wrapping_add(revision);
                (name.clone(), Arc::new(group))
            })
            .collect();
        Arc::new(rocketsim_vis::debug::DebugScene {
            groups,
            meshes: scene.meshes.clone(),
            textures: scene.textures.clone(),
            revision: scene.revision,
        })
    }

    fn mean(v: &[f64]) -> f64 {
        v.iter().sum::<f64>() / v.len() as f64
    }
    fn percentile(v: &mut [f64], p: f64) -> f64 {
        v.sort_by(f64::total_cmp);
        v[((v.len() - 1) as f64 * p).round() as usize]
    }
    fn escape(s: &str) -> String {
        s.replace('\\', "\\\\").replace('"', "\\\"")
    }
    fn make_scene(
        a: &Args,
    ) -> Result<(DebugRenderer, Arc<rocketsim_vis::debug::DebugScene>), String> {
        use rocketsim_vis::debug::{Anchor, Lifetime};
        let r = DebugRenderer::new_detached();
        let mut items = Vec::new();
        let mut rem = a.lines;
        let mut group = 0;
        while rem > 0 {
            let seg = rem.min(4095);
            let points = (0..=seg)
                .map(|i| {
                    let t = i as f32 / seg.max(1) as f32;
                    let angle = t * std::f32::consts::TAU * 12.0;
                    [
                        angle.cos() * 2200.0,
                        angle.sin() * 2200.0,
                        100.0 + group as f32 * 7.0,
                    ]
                })
                .collect();
            let name = format!("lines{group}");
            r.replace_group(
                &name,
                DrawList {
                    items: vec![DebugItem::Polyline(PolylineItem {
                        points,
                        color: Color::CYAN,
                        width: Width::World(12.0),
                        depth: DepthMode::Test,
                        closed: false,
                    })],
                },
                GroupOptions {
                    anchor: Anchor::World,
                    lifetime: Lifetime::Persistent,
                    ..Default::default()
                },
            )
            .map_err(|e| e.to_string())?;
            rem -= seg;
            group += 1;
        }
        for i in 0..a.points {
            let f = i as f32;
            items.push(DebugItem::Point(PointItem {
                pos: [
                    ((f * 17.0).sin()) * 2600.0,
                    ((f * 29.0).cos()) * 2600.0,
                    200.0 + (f % 13.0) * 80.0,
                ],
                color: Color::YELLOW,
                size: Width::World(45.0),
                depth: DepthMode::Test,
            }));
        }
        let mut tris = vec![];
        let mut normals = vec![];
        let mut colors = vec![];
        for p in [
            [0.0, 120.0, 0.0],
            [-100.0, -80.0, 0.0],
            [100.0, -80.0, 0.0],
            [0.0, 0.0, 150.0],
        ] {
            tris.push(p);
            normals.push([0.0, 0.0, 1.0]);
            colors.push(Color::rgb(0.2, 0.8, 0.9));
        }
        let mesh = MeshResource::new(
            "bench-tetra",
            tris,
            colors,
            Some(normals),
            None,
            vec![0, 1, 2, 0, 3, 1, 1, 3, 2, 2, 3, 0],
        )
        .map_err(|e| e.to_string())?;
        r.insert_mesh(mesh).map_err(|e| e.to_string())?;
        for i in 0..a.instances {
            let f = i as f32;
            items.push(DebugItem::Instance(MeshInstanceItem {
                mesh: "bench-tetra".into(),
                position: [
                    (f % 100.0 - 50.0) * 42.0,
                    (f / 100.0 % 50.0 - 25.0) * 75.0,
                    1000.0,
                ],
                rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                scale: [1.0, 1.0, 1.0],
                tint: Color::rgb(0.9, 0.6, 0.25),
                texture: None,
                depth: DepthMode::Test,
            }));
        }
        for chunk in items.chunks(4096).enumerate() {
            let (idx, slice) = chunk;
            r.replace_group(
                &format!("items{idx}"),
                DrawList {
                    items: slice.to_vec(),
                },
                GroupOptions::default(),
            )
            .map_err(|e| e.to_string())?;
        }
        Ok((r.clone(), r.snapshot()))
    }
}

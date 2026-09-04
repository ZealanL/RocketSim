use std::{fs, path::Path};

fn generate_comparison_tests() {
    println!("cargo:rerun-if-changed=tests/rl_comparison_test/test_recordings");

    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let test_data_dir = Path::new(&manifest_dir)
        .join("tests")
        .join("rl_comparison_test")
        .join("test_recordings");
    let out_dir = std::env::var("OUT_DIR").unwrap();
    let destination = Path::new(&out_dir).join("gen_comparison_tests.rs");

    let mut test_code = String::new();
    if let Ok(entries) = fs::read_dir(test_data_dir) {
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_file() {
                if !path.to_str().unwrap().to_lowercase().ends_with(".rlpr") {
                    continue;
                }

                let file_name = path.file_stem().unwrap().to_str().unwrap();
                let test_name = file_name.trim().replace('-', "_");

                test_code.push_str(&format!(
                    "#[test] fn case_{}() {{ run_comparison_test(\"{}\", include_bytes!(r\"{}\")); }}",
                    test_name, test_name, path.display()
                ));
            }
        }
    } else {
        panic!("Tests dir not found");
    }

    fs::write(destination, test_code).unwrap();
}

fn main() {
    generate_comparison_tests();
}

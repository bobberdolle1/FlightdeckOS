fn main() {
    println!(
        "cargo:rustc-link-search=native={}/xplm-sdk",
        env!("CARGO_MANIFEST_DIR")
            .replace("\\", "/")
            .split("/crates")
            .next()
            .unwrap_or(".")
    );
    println!("cargo:rustc-link-lib=dylib=XPLM_64");
}

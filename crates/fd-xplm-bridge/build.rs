fn main() {
    // XPLM_64.lib is Laminar's import library for XPLM.dll (X-Plane
    // provides the DLL at runtime). Vendored at repo root per the SDK
    // license (see xplm-sdk/LICENSE.txt).
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

fn main() {
    cc::Build::new()
        .include("vendor/sbc")
        .flag_if_supported("/FIvendor/sbc/msvc_compat.h")
        .file("vendor/sbc/sbc.c")
        .file("vendor/sbc/sbc_primitives.c")
        .warnings(false)
        .opt_level(3)
        .compile("sbc");
    println!("cargo:rerun-if-changed=vendor/sbc");
}

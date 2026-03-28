from pathlib import Path

Import("env")


def disable_lvgl_asm_sources() -> None:
    libdeps_dir = Path(env.subst("$PROJECT_LIBDEPS_DIR"))
    lvgl_dir = libdeps_dir / env.subst("$PIOENV") / "lvgl"

    for relative_path in (
        "src/draw/sw/blend/neon/lv_blend_neon.S",
        "src/draw/sw/blend/helium/lv_blend_helium.S",
    ):
        source_path = lvgl_dir / relative_path
        disabled_path = source_path.with_suffix(source_path.suffix + ".disabled")

        if source_path.exists() and not disabled_path.exists():
            source_path.rename(disabled_path)
            print(f"Disabled incompatible LVGL asm source: {relative_path}")


disable_lvgl_asm_sources()
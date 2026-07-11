Import("env")
import os, sys, subprocess

sys.path.insert(0, os.path.join(env["PROJECT_DIR"], "tools"))
import hex2sysex

def get_git_rev():
    git_rev = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"]).decode().strip()
    git_status = subprocess.check_output(["git", "status", "-s", "--untracked-files=no"]).decode()
    suffix = "dirty" if git_status else ""

    return git_rev + suffix

def make_syx(target, source, env):
    hex_path = str(source[0])
    out_path = str(target[0])
    print(f"makesyx: {hex_path} -> {out_path}")
    with open(out_path, "wb") as f:
        hex2sysex.process(hex_path, f)
    print("makesyx: wrote %d bytes" % os.path.getsize(out_path))
    make_full_hex(hex_path, out_path[:-4] + "-full.hex", env)

def make_full_hex(app_hex, out_path, env):
    # Merge the complete ISP image: this app + the MIDI bootloader + the SPM
    # flash service. ISP flashing chip-erases the whole part, so a bare app
    # hex silently removes the bootloader and the service; the -full hex is
    # the one to flash with avrdude. Runs on every app build so it can never
    # go stale; build the other two envs first:
    #   pio run -e bootloader -e flash-service -e combined
    from intelhex import IntelHex
    boot = os.path.join(env["PROJECT_DIR"], ".pio", "build", "bootloader", "firmware.hex")
    service = os.path.join(env["PROJECT_DIR"], ".pio", "build", "flash-service", "firmware.hex")
    if not os.path.exists(boot):
        print("makesyx: no bootloader hex, skipping -full.hex (pio run -e bootloader first)")
        return
    def load(path):
        h = IntelHex(path)
        h.start_addr = None   # drop ELF entry records; they differ per image
        return h
    ih = load(app_hex)
    ih.merge(load(boot))
    if os.path.exists(service):
        ih.merge(load(service))
    else:
        print("makesyx: WARNING: flash-service hex missing, -full.hex has no SPM service")
    ih.write_hex_file(out_path)
    print(f"makesyx: wrote {os.path.basename(out_path)} (app + bootloader"
          + (" + flash service)" if os.path.exists(service) else ")"))

git_rev = get_git_rev()
pioenv = env["PIOENV"]
progname = f"SuperOS-606_v%s_{pioenv}_{git_rev}" % env.GetProjectOption("custom_project_version")
syx_path = os.path.join(env["PROJECT_DIR"], f"{progname}.syx")

# Command node: always rebuild the .syx regardless of whether the .hex changed.
# AlwaysBuild marks it unconditionally out-of-date every run.
# Default adds it to the targets built by plain `pio run`.
syx = env.Command(syx_path, "$BUILD_DIR/${PROGNAME}.hex", make_syx)
env.AlwaysBuild(syx)
env.Default(syx)

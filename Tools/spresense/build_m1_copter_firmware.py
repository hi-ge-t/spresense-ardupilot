#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Build and verify the output-disabled Spresense M1 Copter image."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys


SDK_COMMIT = "7fd61b2c03f06a4ff0302b84c755e58c338788b2"
UPSTREAM_COMMIT = "1511f27194f1dcc3728270883047bdf022b3fd53"
PROFILE = "spresense-m1-copter-link"
ARTIFACT_DIRECTORY = "build/spresense-m1-copter-link-artifacts"
CONFIG = "copter_app/output_disabled"
REQUIRED_CONFIG = {
    "CONFIG_SPRESENSE_M1_COPTER_LINK=y",
    "# CONFIG_SPRESENSE_M1_GCS is not set",
    "# CONFIG_CXD56_GNSS is not set",
    "CONFIG_CXD56_GNSS_ADDON=y",
    "CONFIG_SENSORS_CXD5610_GNSS=y",
    "CONFIG_SENSORS_CXD5610_GNSS_NSIGNALRECEIVERS=4",
    "CONFIG_SENSORS_CXD5610_GNSS_RX_THREAD_PRIORITY=120",
    "CONFIG_SENSORS_CXD5610_GNSS_RX_THREAD_STACKSIZE=2048",
    "CONFIG_CXD56_GNSS_RAM=y",
    "CONFIG_CXD56_GNSS_HEAP=y",
    "CONFIG_CXD56_SDIO=y",
    "# CONFIG_FS_AUTOMOUNTER is not set",
    "CONFIG_SENSORS_CXD5602PWBIMU=y",
    "CONFIG_CXD56_CXD5602PWBIMU_SPI5_DMAC=y",
    "CONFIG_CXD56_SPI5=y",
    "CONFIG_CXD56_SPI5_PINMAP_EMMC=y",
    "# CONFIG_CXD56_EMMC is not set",
    "# CONFIG_CXD56_PWM is not set",
    "# CONFIG_PWM is not set",
    "# CONFIG_SYSTEM_NSH is not set",
    "# CONFIG_SYSTEM_CDCACM is not set",
    "# CONFIG_SYSTEM_USBMSC is not set",
    "CONFIG_PTHREAD_MUTEX_TYPES=y",
    "CONFIG_PRIORITY_INHERITANCE=y",
    "CONFIG_LIBM_NEWLIB=y",
    'CONFIG_INIT_ENTRYPOINT="arducopter_spresense_main"',
    "CONFIG_INIT_PRIORITY=180",
    "CONFIG_INIT_STACKSIZE=65536",
    "CONFIG_UART1_BAUD=115200",
    "CONFIG_UART1_SERIAL_CONSOLE=y",
    "# CONFIG_UART2_SERIAL_CONSOLE is not set",
}
FORBIDDEN_CONFIG = {
    "CONFIG_CXD56_SDCARD_AUTOMOUNT=y",
}
REQUIRED_SYMBOLS = {
    "arducopter_spresense_main",
    "g_builtin_count",
    "g_builtins",
    "pthread_create",
    "pthread_mutexattr_settype",
    "board_cxd5602pwbimu_initialize",
    "cxd5602pwbimu_register",
    "cxd5610_gnss_register",
    "_ZN16AP_Arming_Copter3armEN9AP_Arming6MethodEb",
    "_ZN9Spresense8RCOutput5writeEht",
    "_ZN9Spresense9Scheduler13thread_createE7FunctorIvJEEPKcmN6AP_HAL9Scheduler13priority_baseEa",
    "_ZN9Spresense9Semaphore4takeEm",
    "_ZN9Spresense10gnss_startEv",
    "_ZN9Spresense9gnss_readERNS_10GnssSampleE",
    "_ZN9Spresense12pwbimu_startEt",
    "_ZN9Spresense11pwbimu_readERNS_9ImuSampleE",
    "_ZN16AP_GPS_Spresense4readEv",
    "_ZN27AP_InertialSensor_Spresense10accumulateEv",
}
FORBIDDEN_SYMBOLS = {
    "spresense_main",
    "board_pwm_setup",
    "cxd56_pwm_initialize",
    "cxd56_pwm_register",
    "cxd56_emmc_initialize",
}


def run(command, *, cwd: Path, env: dict[str, str], capture=False):
    return subprocess.run(
        [str(value) for value in command],
        cwd=cwd,
        env=env,
        check=True,
        capture_output=capture,
        text=capture,
    )


def git_value(path: Path, *arguments: str) -> str:
    return subprocess.run(
        ["git", "-C", str(path), *arguments],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify_config(path: Path) -> None:
    lines = set(path.read_text(encoding="utf-8").splitlines())
    missing = sorted(REQUIRED_CONFIG - lines)
    if missing:
        raise RuntimeError("configuration contract: " + " | ".join(missing))
    forbidden = sorted(FORBIDDEN_CONFIG & lines)
    if forbidden:
        raise RuntimeError(
            "forbidden configuration: " + " | ".join(forbidden)
        )


def generate_kconfig(root: Path, sdk_root: Path, env: dict[str, str]) -> None:
    sdk = sdk_root / "sdk"
    mkkconfig = sdk / "tools/mkkconfig.py"
    examples = sdk_root / "examples"
    externals = sdk_root / "externals"
    spresense_home = root / "Tools/spresense"
    run(
        [sys.executable, mkkconfig, "-m", "Examples", "-o",
         examples / "Kconfig", *sorted(examples.iterdir())],
        cwd=sdk,
        env=env,
    )
    run(
        [sys.executable, mkkconfig, "-m", "Externals", "-o",
         externals / "Kconfig", *sorted(externals.iterdir())],
        cwd=sdk,
        env=env,
    )
    run(
        [sys.executable, mkkconfig, "-m", "ArduPilot Spresense bring-up",
         "-o", spresense_home / "Kconfig",
         spresense_home / "copter_app", spresense_home / "m1_gcs_app"],
        cwd=sdk,
        env=env,
    )
    run(
        [sys.executable, mkkconfig, "-m", "Spresense SDK", "-o",
         sdk / "Kconfig", sdk / "modules", sdk / "system", examples,
         externals, spresense_home],
        cwd=sdk,
        env=env,
    )


def compiler_library(
    compiler: Path, option: str, root: Path, env: dict[str, str]
) -> Path:
    flags = [
        "-mlittle-endian",
        "-march=armv7e-m",
        "-mtune=cortex-m4",
        "-mfpu=fpv4-sp-d16",
        "-mfloat-abi=hard",
        "-mthumb",
    ]
    value = run(
        [compiler, *flags, option], cwd=root, env=env, capture=True
    ).stdout.strip()
    path = Path(value)
    if not path.is_file():
        raise RuntimeError(f"compiler library is missing: {value}")
    return path


def parse_nm(lines: list[str]) -> tuple[set[str], list[str]]:
    symbols = set()
    entries = []
    for line in lines:
        fields = line.split()
        if len(fields) < 3:
            continue
        name = fields[-1]
        symbols.add(name)
        if name == "arducopter_spresense_main":
            entries.append(line)
    return symbols, entries


def remove_generated_newlib_link(nuttx: Path) -> None:
    link = nuttx / "include/newlib"
    if link.is_symlink():
        link.unlink()
        return
    if link.exists():
        raise RuntimeError(f"unexpected generated path is not a symlink: {link}")


def prepare_generated_newlib_link(nuttx: Path) -> None:
    include = nuttx / "libs/libm/newlib/include"
    if not include.is_dir():
        raise RuntimeError(f"NuttX newlib headers are missing: {include}")
    remove_generated_newlib_link(nuttx)
    (nuttx / "include/newlib").symlink_to(include, target_is_directory=True)


def parser() -> argparse.ArgumentParser:
    value = argparse.ArgumentParser(description=__doc__)
    value.add_argument(
        "--allow-dirty",
        action="store_true",
        help="development build only; dirty artifacts remain non-flashable",
    )
    value.add_argument(
        "--reuse-build",
        action="store_true",
        help="reuse current Waf archives and Sony configuration for iteration",
    )
    value.add_argument("--jobs", type=int, default=2)
    return value


def main() -> int:
    arguments = parser().parse_args()
    root = Path(__file__).resolve().parents[2]
    sdk_root = root / "modules/Spresense"
    sdk = sdk_root / "sdk"
    nuttx = sdk_root / "nuttx"
    toolchain = Path.home() / "spresenseenv/usr/bin"
    compiler = toolchain / "arm-none-eabi-gcc"
    cxx = toolchain / "arm-none-eabi-g++"
    nm = toolchain / "arm-none-eabi-nm"
    python = shutil.which("python3.11") or sys.executable
    archive = root / "build/spresense/lib/bin/libarducopter.a"
    libraries = root / "build/spresense/lib/libArduCopter_libs.a"
    hal_sdk_archive = (
        root / "build/spresense-m1-copter-link-sdk"
        / "libAP_HAL_Spresense_sdk.a"
    )
    artifact_dir = root / ARTIFACT_DIRECTORY
    env = os.environ.copy()
    env["PATH"] = f"{toolchain}:{env.get('PATH', '')}"
    env["SPRESENSE_HOME"] = str(root / "Tools/spresense")
    env["GCCVER"] = "10"

    try:
        if arguments.jobs < 1 or arguments.jobs > 8:
            raise RuntimeError("--jobs must be in the range 1..8")
        for required in (compiler, cxx, nm):
            if not required.is_file():
                raise RuntimeError(f"Sony toolchain is missing: {required}")
        gcc_version = run(
            [compiler, "-dumpfullversion", "-dumpversion"],
            cwd=root,
            env=env,
            capture=True,
        ).stdout.strip()
        if gcc_version != "10.3.1":
            raise RuntimeError(f"Sony GCC version mismatch: {gcc_version}")
        sdk_commit = git_value(sdk_root, "rev-parse", "HEAD")
        if sdk_commit != SDK_COMMIT:
            raise RuntimeError(f"Sony SDK commit mismatch: {sdk_commit}")
        project_commit = git_value(root, "rev-parse", "HEAD")
        upstream_commit = git_value(root, "rev-parse", "Copter-4.7.0^{}")
        if upstream_commit != UPSTREAM_COMMIT:
            raise RuntimeError(
                f"upstream Copter baseline mismatch: {upstream_commit}"
            )
        mavlink_commit = git_value(root / "modules/mavlink", "rev-parse", "HEAD")
        expected_mavlink = git_value(
            root, "ls-tree", "HEAD", "modules/mavlink"
        ).split()[2]
        if mavlink_commit != expected_mavlink:
            raise RuntimeError(
                f"MAVLink submodule commit mismatch: {mavlink_commit}"
            )
        dirty = bool(git_value(
            root, "status", "--porcelain", "--ignore-submodules=untracked"
        ))
        if dirty and not arguments.allow_dirty:
            raise RuntimeError(
                "worktree is dirty; commit first or use --allow-dirty for a "
                "non-flashable development build"
            )

        run(
            [sys.executable, root / "Tools/spresense/verify_m1_copter_contract.py"],
            cwd=root,
            env=env,
        )
        if arguments.reuse_build:
            for required in (archive, libraries, nuttx / ".config"):
                if not required.is_file():
                    raise RuntimeError(f"reused build input is missing: {required}")
        else:
            run(
                [python, root / "waf", "configure", "--board", "spresense",
                 "--disable-scripting", "--disable-networking",
                 "--no-submodule-update"],
                cwd=root,
                env=env,
            )
            run(
                [python, root / "waf", "copter", "--targets",
                 "bin/arducopter", f"-j{arguments.jobs}"],
                cwd=root,
                env=env,
            )
            generate_kconfig(root, sdk_root, env)
            run(
                [sys.executable, "tools/config.py", "default", CONFIG],
                cwd=sdk,
                env=env,
            )
        verify_config(nuttx / ".config")

        libgcc = compiler_library(
            compiler, "--print-libgcc-file-name", root, env
        )
        libsupcxx = compiler_library(
            cxx, "--print-file-name=libsupc++.a", root, env
        )
        # Application.mk uses a .built marker and does not know if another SDK
        # subtree recreated libapps.a.  The HAL has a dedicated archive; force
        # its archive step on every firmware link so --reuse-build is equally
        # deterministic.
        for generated in (
            root / "Tools/spresense/copter_app/.built",
            root / "Tools/spresense/.built",
            hal_sdk_archive,
            Path(f"{hal_sdk_archive}.lock"),
        ):
            generated.unlink(missing_ok=True)

        extra_libraries = (
            archive,
            libraries,
            hal_sdk_archive,
            libsupcxx,
            libgcc,
        )
        env["EXTRA_LIBS"] = " ".join(str(path) for path in extra_libraries)
        make_args = [f"COMPILER_RT_LIB={libgcc}"]
        prepare_generated_newlib_link(nuttx)
        run(
            ["make", "-C", nuttx, "-j1", *make_args, "context"],
            cwd=root,
            env=env,
        )
        run(
            ["make", "-C", nuttx, "-j1", *make_args, "depend"],
            cwd=root,
            env=env,
        )
        run(
            ["make", "-C", sdk, f"-j{arguments.jobs}", *make_args],
            cwd=root,
            env=env,
        )
        if not hal_sdk_archive.is_file():
            raise RuntimeError(
                f"AP_HAL_Spresense SDK archive is missing: {hal_sdk_archive}"
            )

        sources = {
            "nuttx.spk": sdk / "nuttx.spk",
            "nuttx": sdk / "nuttx",
            "nuttx.map": nuttx / "nuttx.map",
            "nuttx.config": nuttx / ".config",
        }
        for source in sources.values():
            if not source.is_file():
                raise RuntimeError(f"build output is missing: {source}")
        artifact_dir.mkdir(parents=True, exist_ok=True)
        for name, source in sources.items():
            shutil.copy2(source, artifact_dir / name)

        map_report = artifact_dir / "memory-layout.json"
        run(
            [sys.executable, root / "Tools/spresense/verify_m1_copter_map.py",
             "--map", artifact_dir / "nuttx.map", "--output", map_report],
            cwd=root,
            env=env,
        )
        nm_lines = run(
            [nm, artifact_dir / "nuttx"], cwd=root, env=env, capture=True
        ).stdout.splitlines()
        symbols, entries = parse_nm(nm_lines)
        if len(entries) != 1 or " T " not in f" {entries[0]} ":
            raise RuntimeError(
                "arducopter_spresense_main is not unique/strong: "
                f"{entries}"
            )
        missing = sorted(REQUIRED_SYMBOLS - symbols)
        if missing:
            raise RuntimeError("required symbol missing: " + " | ".join(missing))
        forbidden = sorted(FORBIDDEN_SYMBOLS & symbols)
        if forbidden:
            raise RuntimeError(
                "forbidden output/fallback symbol linked: "
                + " | ".join(forbidden)
            )

        memory = json.loads(map_report.read_text(encoding="utf-8"))
        manifest_values = {
            "profile": PROFILE,
            "project_commit": project_commit,
            "project_tree": "dirty" if dirty else "clean",
            "upstream_commit": UPSTREAM_COMMIT,
            "sdk_commit": SDK_COMMIT,
            "mavlink_commit": mavlink_commit,
            "gcc_version": gcc_version,
            "m1.copter.full": "true",
            "m1.copter.entry": "arducopter_spresense_main",
            "m1.copter.scheduler": "nuttx-pthread",
            "m1.copter.loop_rate_default_hz": "100",
            "m1.gcs.transport": "/dev/ttyS0@115200",
            "m1.gnss.builtin": "disabled",
            "m1.gnss.addon": "required",
            "m1.gnss.device": "/dev/gps2",
            "m1.gnss.ram": "required-complete-heap",
            "m1.gnss.hal_integration": "AP_GPS_Spresense",
            "m1.pwbimu.addon": "required",
            "m1.pwbimu.device": "/dev/imu0",
            "m1.pwbimu.bus": "SPI5",
            "m1.pwbimu.pinshare": "eMMC",
            "m1.pwbimu.hal_integration": "AP_InertialSensor_Spresense",
            "m1.pwbimu.sample_rate_hz": "60",
            "m1.pwbimu.sample_rate_status": "bringup-only-hardware-HOLD",
            "m1.pwbimu.orientation": "ROTATION_NONE-hardware-HOLD",
            "m1.sensor.hal_integration": "GNSS+INS",
            "m1.sensor.runtime": "hardware-HOLD",
            "m1.sensor_fallback": "disabled",
            "m1.outputs": "disabled",
            "m1.arming": "always-denied",
            "m1.physical_write_expected": "0",
            "m1.storage.development": "microSD",
            "m1.storage.mount_policy": "explicit-fixed-device",
            "m1.storage.final_candidate": "eMMC",
            "m1.storage.automatic_fallback": "disabled",
            "m1.storage.pwbimu_emmc_coexistence": "hardware-design-HOLD",
            "m1.runtime": "hardware-HOLD",
            "m1.flight_ready": "false",
            "memory.application_heap_bytes": str(
                memory["application_sram"]["heap_envelope"]["bytes"]
            ),
            "memory.gnss_heap_bytes": str(
                memory["gnss_ram"]["heap"]["bytes"]
            ),
        }
        for name in (*sources, "memory-layout.json"):
            manifest_values[f"artifact.{name}.sha256"] = sha256(
                artifact_dir / name
            )
        (artifact_dir / "ARTIFACTS.manifest").write_text(
            "".join(
                f"{key}={value}\n" for key, value in manifest_values.items()
            ),
            encoding="utf-8",
        )
        remove_generated_newlib_link(nuttx)
    except (
        RuntimeError,
        OSError,
        UnicodeError,
        json.JSONDecodeError,
        subprocess.CalledProcessError,
    ) as error:
        if (nuttx / "include/newlib").is_symlink():
            (nuttx / "include/newlib").unlink()
        print(f"spresense_m1_copter_build=FAIL reason={error}", file=sys.stderr)
        return 1

    print(
        "spresense_m1_copter_build=PASS "
        f"project_commit={project_commit} "
        f"tree={'dirty' if dirty else 'clean'} gcc={gcc_version} "
        f"artifact_dir={artifact_dir} sensors=GNSS+INS"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

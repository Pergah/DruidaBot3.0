"""
DruidaBot3.0 — Generador y pusher de frontend.bin
==================================================
Uso:
    python make_frontend.py           # genera + pushea
    python make_frontend.py --only-build  # solo genera el .bin local
"""

import subprocess
import struct
import tempfile
import sys
import os

# ─── CONFIGURACIÓN ───────────────────────────────────────────────────────────
MKFATFS        = r"C:\Tools\mkfatfs\mkfatfs.exe"
DATA_DIR       = os.path.join(os.path.dirname(__file__), "data")
REPO_URL       = "https://github.com/Pergah/DruidaBot3.0.git"
REPO_DIR       = r"C:\Users\BRYAN\Documents\Arduino\DruidaBot_release"
OUTPUT_BIN     = os.path.join(REPO_DIR, "frontend.bin")
PARTITION_SIZE = 0x9E0000   # ffat en partitions.csv
SECTOR_SIZE    = 4096
GIT_NAME       = "Pergah"
GIT_EMAIL      = "pergah@users.noreply.github.com"
# ─────────────────────────────────────────────────────────────────────────────


def run(cmd, cwd=None, capture=False):
    r = subprocess.run(cmd, cwd=cwd,
                       capture_output=capture, text=True)
    if r.returncode != 0:
        raise RuntimeError((r.stderr or r.stdout or "").strip())
    return r


def wl_crc32(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xEDB88320 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFFFFFF


def calculate_wl_layout(partition_size, sector_size=4096):
    cfg_size    = ((36 + sector_size - 1) // sector_size) * sector_size
    num_sectors = partition_size // sector_size
    state_raw   = 64 + num_sectors * 4
    state_size  = ((state_raw + sector_size - 1) // sector_size) * sector_size
    addr_cfg    = partition_size - cfg_size
    addr_state1 = partition_size - cfg_size - state_size * 2
    addr_state2 = partition_size - cfg_size - state_size
    flash_size  = (addr_state1 // sector_size - 1) * sector_size
    max_pos     = flash_size // sector_size + 1
    return dict(flash_size=flash_size, max_pos=max_pos, cfg_size=cfg_size,
                state_size=state_size, addr_cfg=addr_cfg,
                addr_state1=addr_state1, addr_state2=addr_state2)


def wrap_with_wear_leveling(fat_image: bytes, partition_size: int,
                             sector_size=4096) -> bytes:
    l = calculate_wl_layout(partition_size, sector_size)
    if len(fat_image) != l['flash_size']:
        raise ValueError(f"FAT size mismatch: {len(fat_image)} != {l['flash_size']}")

    image = bytearray(b'\xFF' * partition_size)
    image[sector_size:sector_size + len(fat_image)] = fat_image

    cfg = bytearray(36)
    struct.pack_into('<IIIIIIIII', cfg, 0,
        0, partition_size, sector_size, sector_size,
        16, 16, 2, 32, 0)
    cfg_crc = wl_crc32(bytes(cfg[:32]))
    struct.pack_into('<I', cfg, 32, cfg_crc)
    image[l['addr_cfg']:l['addr_cfg'] + 36] = cfg

    state = bytearray(64)
    struct.pack_into('<IIIIIIII', state, 0,
        0, l['max_pos'], 0, 0, 16, sector_size, 2, cfg_crc)
    struct.pack_into('<I', state, 60, wl_crc32(bytes(state[:60])))
    image[l['addr_state1']:l['addr_state1'] + 64] = state
    image[l['addr_state2']:l['addr_state2'] + 64] = state

    return bytes(image)


def build():
    layout = calculate_wl_layout(PARTITION_SIZE)
    fat_size = layout['flash_size']
    print(f"  Partición : {PARTITION_SIZE} bytes")
    print(f"  FAT usable: {fat_size} bytes ({fat_size // 1024} KB)")

    raw = tempfile.mktemp(suffix='.fat.raw')
    try:
        print(f"\n[1/2] mkfatfs -s {fat_size} ...")
        r = subprocess.run(
            [MKFATFS, '-c', DATA_DIR, '-s', str(fat_size), raw],
            capture_output=True, text=True)
        print(r.stdout or r.stderr)
        if r.returncode != 0:
            raise RuntimeError(f"mkfatfs error: {r.stderr}")

        with open(raw, 'rb') as f:
            fat = f.read()

        print("[2/2] Aplicando Wear-Leveling ...")
        wl = wrap_with_wear_leveling(fat, PARTITION_SIZE)
        print(f"  Imagen final: {len(wl)} bytes")
        return wl
    finally:
        if os.path.exists(raw):
            os.remove(raw)


def ensure_repo():
    if os.path.isdir(os.path.join(REPO_DIR, '.git')):
        print("[git] Actualizando repo local ...")
        run(['git', 'pull', '--rebase', 'origin', 'main'], cwd=REPO_DIR)
    else:
        print("[git] Clonando repo ...")
        os.makedirs(REPO_DIR, exist_ok=True)
        run(['git', 'clone', REPO_URL, REPO_DIR])
        run(['git', 'config', 'user.name',  GIT_NAME],  cwd=REPO_DIR)
        run(['git', 'config', 'user.email', GIT_EMAIL], cwd=REPO_DIR)


def push(wl_image: bytes):
    ensure_repo()

    with open(OUTPUT_BIN, 'wb') as f:
        f.write(wl_image)
    print(f"[git] frontend.bin escrito ({len(wl_image)} bytes)")

    run(['git', 'add', 'frontend.bin'], cwd=REPO_DIR)

    status = subprocess.run(
        ['git', 'status', '--porcelain'],
        cwd=REPO_DIR, capture_output=True, text=True).stdout.strip()
    if not status:
        print("[git] Sin cambios — nada que pushear.")
        return

    run(['git', 'commit', '-m', 'Update frontend.bin'], cwd=REPO_DIR)
    run(['git', 'push', 'origin', 'main'], cwd=REPO_DIR)
    print("[git] Push exitoso:", REPO_URL)


def main():
    only_build = '--only-build' in sys.argv

    print("=" * 50)
    print("  DruidaBot3.0 — frontend.bin builder")
    print("=" * 50)

    wl_image = build()

    if only_build:
        local = os.path.join(os.path.dirname(__file__), 'frontend.bin')
        with open(local, 'wb') as f:
            f.write(wl_image)
        print(f"\nGuardado localmente: {local}")
    else:
        push(wl_image)

    print("\nListo.")


if __name__ == '__main__':
    main()

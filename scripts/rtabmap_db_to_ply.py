#!/usr/bin/env python3
"""Read XYZI laser scans from an RTAB-Map .db, transform to world frame, write a single PLY."""
import sqlite3, struct, zlib, sys, os
import numpy as np


def unpack_pose(blob: bytes) -> np.ndarray:
    floats = struct.unpack('<12f', blob)
    return np.array(floats, dtype=np.float64).reshape(3, 4)


def unpack_scan_info(blob: bytes):
    n = len(blob) // 4
    f = struct.unpack(f'<{n}f', blob[:n * 4])
    fmt = int(f[0])
    local = np.array(f[7:19], dtype=np.float64).reshape(3, 4)
    return fmt, local


def transform_3x4(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    return pts @ T[:3, :3].T + T[:3, 3]


def main(db_path: str, out_path: str, voxel: float = 0.05) -> None:
    db = sqlite3.connect(db_path)
    c = db.cursor()
    c.execute('''
        SELECT n.id, n.pose, d.scan, d.scan_info
        FROM Node n JOIN Data d ON n.id = d.id
        WHERE d.scan IS NOT NULL AND n.pose IS NOT NULL
        ORDER BY n.id
    ''')
    rows = c.fetchall()
    print(f"Found {len(rows)} nodes with scans")

    inv_v = 1.0 / voxel if voxel > 0 else 0.0
    out_pts = []
    total_raw = 0
    for nid, pose_b, scan_b, info_b in rows:
        pose = unpack_pose(pose_b)
        fmt, local = unpack_scan_info(info_b)
        if fmt != 6:
            print(f"  node {nid}: skipping, unexpected format {fmt}")
            continue
        raw = zlib.decompress(scan_b)
        n = len(raw) // 16
        if n == 0:
            continue
        arr = np.frombuffer(raw, dtype=np.float32, count=n * 4).reshape(n, 4)
        xyz = arr[:, :3].astype(np.float64)
        intens = arr[:, 3].astype(np.float32)
        xyz = transform_3x4(local, xyz)
        xyz = transform_3x4(pose, xyz)
        total_raw += n
        out_pts.append(np.column_stack([xyz.astype(np.float32), intens]))
        if nid % 50 == 0:
            print(f"  node {nid}: raw_total={total_raw:,}")

    if not out_pts:
        print("No points produced.")
        sys.exit(1)
    cloud = np.concatenate(out_pts, axis=0)
    print(f"Total raw points: {cloud.shape[0]:,}")
    if voxel > 0:
        keys = np.floor(cloud[:, :3].astype(np.float64) * inv_v).astype(np.int64)
        # encode 3-int key as a single int64
        kx, ky, kz = keys[:, 0], keys[:, 1], keys[:, 2]
        h = (kx & 0x1FFFFF) | ((ky & 0x1FFFFF) << 21) | ((kz & 0x1FFFFF) << 42)
        _, idx = np.unique(h, return_index=True)
        cloud = cloud[np.sort(idx)]
        print(f"After voxel ({voxel} m) downsample: {cloud.shape[0]:,}")
    write_ply_binary(out_path, cloud)
    print(f"Wrote {out_path} ({os.path.getsize(out_path) / 1e6:.1f} MB)")


def write_ply_binary(path: str, cloud: np.ndarray) -> None:
    n = cloud.shape[0]
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {n}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property float intensity\n"
        "end_header\n"
    ).encode("ascii")
    with open(path, "wb") as f:
        f.write(header)
        f.write(cloud.astype(np.float32).tobytes())


if __name__ == "__main__":
    db_path = sys.argv[1]
    out_path = sys.argv[2]
    voxel = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05
    main(db_path, out_path, voxel)

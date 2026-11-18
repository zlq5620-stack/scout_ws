"""Offline FAST-LIO PCD to conservative 2D occupancy grid (no ROS/hardware)."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from scipy.ndimage import binary_closing
import yaml


def read_xyz(path):
    """Read PCL binary/ascii PCD with explicit scalar field layout checks."""
    with open(path, 'rb') as stream:
        header = {}
        while True:
            raw = stream.readline()
            if not raw:
                raise ValueError('PCD header has no DATA line')
            line = raw.decode('ascii').strip()
            if not line or line.startswith('#'):
                continue
            key, *values = line.split()
            header[key] = values
            if key == 'DATA':
                break
        fields = header['FIELDS']
        sizes = list(map(int, header['SIZE']))
        types = header['TYPE']
        counts = list(map(int, header.get('COUNT', ['1'] * len(fields))))
        dtype_fields = []
        for name, size, kind, count in zip(fields, sizes, types, counts):
            dt = np.dtype('<' + {'F': 'f', 'I': 'i', 'U': 'u'}[kind] + str(size))
            dtype_fields.append((name, dt) if count == 1 else (name, dt, (count,)))
        count = int(header['POINTS'][0])
        if header['DATA'] == ['binary']:
            dtype = np.dtype(dtype_fields)
            data = stream.read()
            if len(data) != count * dtype.itemsize:
                raise ValueError('PCD binary payload length does not match POINTS')
            cloud = np.frombuffer(data, dtype=dtype)
            xyz = np.column_stack([cloud[k] for k in ('x', 'y', 'z')])
        elif header['DATA'] == ['ascii']:
            data = np.loadtxt(stream, ndmin=2)
            if len(data) != count:
                raise ValueError('PCD ASCII count does not match POINTS')
            offsets = np.cumsum([0] + counts[:-1]).tolist()
            xyz = data[:, [offsets[fields.index(k)] for k in ('x', 'y', 'z')]]
        else:
            raise ValueError('binary_compressed PCD is not supported; export binary/ascii first')
    return np.asarray(xyz[np.isfinite(xyz).all(axis=1)], dtype=np.float64), count


def mounting_matrix():
    """Match existing launch: T_map_camera_init = T_map_base * T_base_lidar * inv(T_imu_lidar)."""
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler('xyz', [0, .523599, 0]).as_matrix()
    transform[:3, 3] = np.array([.130, 0, .326]) - transform[:3, :3] @ np.array([-.011, -.02329, .04412]) + [0, 0, .079]
    return transform


def apply_transform(xyz, transform):
    return xyz @ transform[:3, :3].T + transform[:3, 3]


def estimate_ground(xyz, threshold=.04, max_tilt=15):
    """RANSAC near expected ground; reject high-tilt planes and refine by SVD."""
    candidates = xyz[(xyz[:, 2] > -.4) & (xyz[:, 2] < .4)]
    if len(candidates) < 100:
        raise ValueError('Insufficient points near expected ground; inspect mounting/scene')
    rng = np.random.default_rng(42)
    candidates = candidates[rng.choice(len(candidates), min(15000, len(candidates)), replace=False)]
    best = None
    best_count = 0
    for _ in range(200):
        tri = candidates[rng.choice(len(candidates), 3, replace=False)]
        normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        length = np.linalg.norm(normal)
        if length < 1e-8:
            continue
        normal /= length
        if normal[2] < 0:
            normal = -normal
        if normal[2] < np.cos(np.deg2rad(max_tilt)):
            continue
        d = -normal @ tri[0]
        mask = np.abs(candidates @ normal + d) < threshold
        if mask.sum() > best_count:
            best_count = int(mask.sum())
            best = mask
    if best is None or best_count < max(100, .3 * len(candidates)):
        raise ValueError('No sufficiently supported ground plane found')
    inliers = candidates[best]
    center = np.mean(inliers, axis=0)
    _, _, vh = np.linalg.svd(inliers - center, full_matrices=False)
    normal = vh[-1]
    if normal[2] < 0:
        normal = -normal
    tilt = float(np.rad2deg(np.arccos(np.clip(normal[2], -1, 1))))
    if tilt > max_tilt:
        raise ValueError('Refined ground tilt outside configured limit')
    d = float(-normal @ center)
    # Shortest rotation n -> +Z; single-vector align_vectors is underdetermined
    # on older SciPy versions and may introduce an arbitrary map yaw.
    v = np.cross(normal, [0., 0., 1.])
    skew = np.array([[0., -v[2], v[1]], [v[2], 0., -v[0]], [-v[1], v[0], 0.]])
    rotation = np.eye(3) + skew + skew @ skew / (1. + normal[2])
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[2, 3] = d
    return transform, {'normal': normal.tolist(), 'd': d, 'tilt_deg': tilt,
                       'candidate_points': len(candidates), 'inlier_fraction': best_count / len(candidates)}


def rasterize(ground, obstacles, resolution, padding=.5):
    """Free only where ground is observed; unknown elsewhere; obstacles always win."""
    points = np.concatenate([ground, obstacles])
    if not len(points):
        raise ValueError('No ground/obstacle points in selected height bands')
    origin = np.floor((points[:, :2].min(axis=0) - padding) / resolution) * resolution
    shape = np.ceil((points[:, :2].max(axis=0) + padding - origin) / resolution).astype(int) + 1
    if np.prod(shape) > 10000000:
        raise ValueError('Map too large; use a smaller --max-range or larger --resolution')
    occupied = np.zeros((shape[1], shape[0]), dtype=bool)
    free = np.zeros_like(occupied)
    for pts, array in ((ground, free), (obstacles, occupied)):
        indices = np.floor((pts[:, :2] - origin) / resolution + 1e-9).astype(int)
        array[indices[:, 1], indices[:, 0]] = True
    # Fill only one-cell sampling holes surrounded by observed floor, never whole rooms.
    free = binary_closing(free, structure=np.ones((3, 3))) | free
    grid = np.full_like(occupied, 205, dtype=np.uint8)
    grid[free] = 254
    grid[occupied] = 0
    return grid, origin


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pcd', type=Path)
    parser.add_argument('--output', type=Path, required=True, help='Output prefix, without extension')
    parser.add_argument('--resolution', type=float, default=.05)
    parser.add_argument('--min-height', type=float, default=.10)
    parser.add_argument('--max-height', type=float, default=.75,
                        help='Obstacle upper height in m; default: existing map band 0.75; current measured vehicle height 0.60')
    parser.add_argument('--max-range', type=float, default=15.)
    parser.add_argument('--voxel-size', type=float, default=.03)
    parser.add_argument('--neighbor-radius', type=float, default=.12)
    parser.add_argument('--min-neighbors', type=int, default=3)
    parser.add_argument('--ground-threshold', type=float, default=.04)
    parser.add_argument('--mount-transform', type=Path, help='YAML with T_map_camera_init (4x4); default: current measured mounting')
    parser.add_argument('--force', action='store_true', help='Replace existing outputs')
    opt = parser.parse_args(args)
    if not (0 < opt.ground_threshold < opt.min_height < opt.max_height and
            opt.resolution > 0 and opt.voxel_size > 0 and opt.max_range > 0 and
            opt.neighbor_radius > 0 and opt.min_neighbors >= 1):
        parser.error('Invalid resolution, filtering or height-band parameters')
    outputs = [Path(str(opt.output) + ext) for ext in ('.pgm', '.yaml', '.frames.yaml', '.report.json', '.preview.png')]
    if not opt.force and any(p.exists() for p in outputs):
        parser.error('Output already exists; select another name or pass --force')
    opt.output.parent.mkdir(parents=True, exist_ok=True)
    xyz, raw_count = read_xyz(opt.pcd)
    finite_count = len(xyz)
    in_range = np.linalg.norm(xyz, axis=1) <= opt.max_range
    xyz = xyz[in_range]
    keys = np.floor(xyz / opt.voxel_size).astype(np.int64)
    _, indices = np.unique(keys, axis=0, return_index=True)
    xyz = xyz[indices]
    voxel_count = len(xyz)
    neighbors = cKDTree(xyz).query_ball_point(xyz, opt.neighbor_radius, return_length=True)
    xyz = xyz[neighbors >= opt.min_neighbors]
    mounting = mounting_matrix()
    if opt.mount_transform:
        mounting = np.array(yaml.safe_load(opt.mount_transform.read_text())['T_map_camera_init'], dtype=float)
        if mounting.shape != (4, 4) or not np.allclose(mounting[3], [0, 0, 0, 1]):
            raise ValueError('Mount transform must be a homogeneous 4x4 matrix')
        r = mounting[:3, :3]
        if not np.allclose(r.T @ r, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(r), 1.):
            raise ValueError('Mount transform rotation is invalid')
    mapped = apply_transform(xyz, mounting)
    leveling, plane = estimate_ground(mapped, opt.ground_threshold)
    leveled = apply_transform(mapped, leveling)
    ground = leveled[np.abs(leveled[:, 2]) < opt.ground_threshold]
    obstacles = leveled[(leveled[:, 2] >= opt.min_height) & (leveled[:, 2] <= opt.max_height)]
    grid, origin = rasterize(ground, obstacles, opt.resolution)
    with outputs[0].open('wb') as stream:
        stream.write(f'P5\n{grid.shape[1]} {grid.shape[0]}\n255\n'.encode())
        stream.write(np.flipud(grid).tobytes())
    outputs[1].write_text(yaml.safe_dump({'image': outputs[0].name, 'mode': 'trinary',
        'resolution': opt.resolution, 'origin': [float(origin[0]), float(origin[1]), 0.],
        'negate': 0, 'occupied_thresh': .65, 'free_thresh': .196}, sort_keys=False))
    outputs[2].write_text(yaml.safe_dump({
        'source_pcd': str(opt.pcd.resolve()),
        'source_sha256': hashlib.sha256(opt.pcd.read_bytes()).hexdigest(),
        'grid_frame': 'navigation_map', 'pcd_frame': 'camera_init',
        'T_map_camera_init': mounting.tolist(),
        'T_navigation_map_map': leveling.tolist(),
        'T_navigation_map_camera_init': (leveling @ mounting).tolist(),
        'ground_plane_in_map': plane,
        'note': 'Offline grid axes differ from live FAST-LIO map after leveling. Localization must use this transform or estimate pose in the exported grid.'
    }, sort_keys=False))
    report = {'source_points': raw_count, 'finite_points': finite_count,
        'range_retained': int(in_range.sum()), 'voxel_points': voxel_count,
        'neighbor_filter_retained': len(xyz), 'ground_points': len(ground),
        'obstacle_points': len(obstacles), 'ground_fit': plane,
        'resolution': opt.resolution, 'height_band': [opt.min_height, opt.max_height],
        'size_cells': [grid.shape[1], grid.shape[0]],
        'size_m': [grid.shape[1] * opt.resolution, grid.shape[0] * opt.resolution],
        'occupied_cells': int((grid == 0).sum()), 'free_cells': int((grid == 254).sum()),
        'unknown_cells': int((grid == 205).sum()),
        'limitations': 'Floor evidence is not ray-traced free space or proof of clearance; height band must match actual vehicle envelope. Not yet navigation validated.'}
    outputs[3].write_text(json.dumps(report, indent=2, ensure_ascii=False))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(10, 10))
    extent = [origin[0], origin[0] + grid.shape[1] * opt.resolution,
              origin[1], origin[1] + grid.shape[0] * opt.resolution]
    ax.imshow(grid, cmap='gray', vmin=0, vmax=255, origin='lower', extent=extent, interpolation='nearest')
    ax.set(xlabel='X (m)', ylabel='Y (m)', title='PCD grid: black=occupied, white=observed floor, gray=unknown')
    ax.set_aspect('equal');fig.tight_layout();fig.savefig(outputs[4], dpi=160);plt.close(fig)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    print('Outputs:', *map(str, outputs), sep='\n')


if __name__ == '__main__':
    main()

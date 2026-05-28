# Multi-Lidar Calibration

Standalone extrinsic calibration tool for N lidars. Subscribes to DDS point cloud topics, captures frames, and computes refined mount transforms using GICP.

Works with any robot — just provide a config JSON with lidar names, DDS topics, and initial mount estimates.

## Features

- **Multi-frame capture** — captures K frames per lidar, picks the best GICP result
- **Pre-filtering** — range clipping + statistical outlier removal before alignment
- **Coarse-to-fine GICP** — two-pass alignment for robust convergence
- **Fitness threshold** — rejects poor alignments, refuses to write bad calibrations
- **Joint N-lidar refinement** — iteratively aligns each lidar against all others (N > 2)

## Build

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Dependencies

- [message_manager](https://github.com/botcrew) (FastDDS transport)
- PCL (Point Cloud Library)
- Eigen3
- nlohmann_json
- FastDDS + FastCDR

## Usage

```bash
# Basic calibration (lidars must be publishing on DDS)
./multi_lidar_calibrate --config calibration_config.json

# With options
./multi_lidar_calibrate \
    --config calibration_config.json \
    --ref front_lidar \
    --num-frames 10 \
    --max-fitness 0.2 \
    --write-config \
    --save-aligned merged.pcd
```

## Config Format

```json
{
  "dds": { "domain_id": 0 },
  "lidars": [
    {
      "name": "front_lidar",
      "topic": "/front_lidar/points",
      "mount_tf": {
        "translation": [2.0, 0.0, 0.0],
        "rotation_rpy_deg": [33, 0, 90]
      }
    }
  ]
}
```

See `config/example_config.json` for a full example.

## CLI Options

| Flag | Default | Description |
|------|---------|-------------|
| `--config` | `calibration_config.json` | Config JSON path |
| `--ref` | first lidar | Reference lidar (kept fixed) |
| `--timeout` | 30 | Capture timeout (seconds) |
| `--num-frames` | 5 | Frames per lidar for best-of-K |
| `--voxel` | 0.1 | VoxelGrid leaf size (m) |
| `--max-iter` | 100 | Max GICP iterations per pass |
| `--corr-dist` | 2.0 | Max correspondence distance (m) |
| `--min-range` | 0.5 | Remove points closer than this (m) |
| `--max-range` | 100.0 | Remove points farther than this (m) |
| `--max-fitness` | 0.3 | Reject results above this fitness |
| `--refine-iters` | 10 | Joint refinement iterations (N>2) |
| `--save-aligned` | — | Save merged cloud to PCD |
| `--write-config` | — | Write results back to config |

## Tests

```bash
cd build
make calibration_tests
./calibration_tests
```

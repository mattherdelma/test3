# RTPercept — Real-time Vision Perception & Multi-Object Tracking

A low-latency C++17/20 perception pipeline for drone/robot vision and on-screen
content analysis on Windows.

```
 ┌────────────────┐   SPSC lock-free   ┌──────────────────────┐   ┌──────────────┐
 │  DXGI Capture  │ ─── ring queue ──► │ YOLOv8 (TensorRT)    │ ► │  ByteTrack   │
 │  (Desktop Dup) │   GPU frames       │ CUDA pre/post + NMS  │   │  Kalman MOT  │
 └────────────────┘                    └──────────────────────┘   └──────────────┘
   capture thread                          inference thread
```

## Components

| Module | File(s) | Responsibility |
|--------|---------|----------------|
| Capture | `src/capture/DxgiCapturer.*` | DXGI Desktop Duplication, zero-copy D3D11↔CUDA interop |
| Queue | `src/common/SPSCQueue.h` | Lock-free single-producer/single-consumer ring buffer |
| Inference | `src/inference/TensorRTEngine.*` | Generic TensorRT engine RAII wrapper, CUDA stream + pinned memory |
| Detector | `src/inference/YoloV8.*` | YOLOv8 head: CUDA letterbox/normalize, GPU decode + NMS |
| CUDA kernels | `src/inference/kernels.cu` | Preprocess (BGRA→RGB, resize, normalize) + decode/NMS |
| Tracking | `src/tracking/*` | ByteTrack: Kalman filter, track lifecycle, IoU + lapjv association |

## Design goals

- **Zero-copy capture**: the DXGI-acquired `ID3D11Texture2D` is registered with
  CUDA (`cudaGraphicsD3D11RegisterResource`) and mapped straight into the
  preprocessing kernel — no staging through host memory.
- **Thread separation**: capture and inference run on dedicated threads,
  communicating through a wait-free SPSC queue. No mutex on the hot path.
- **Stream isolation**: all inference work runs on a dedicated `cudaStream_t`
  with pinned host buffers so it never serializes against the desktop compositor.
- **RAII everywhere**: every COM / CUDA / TensorRT handle is owned by a smart
  pointer or a custom deleter; no manual `Release()`/`free()` on the happy path.

## Build

### Windows 10 + Visual Studio 2019 (target platform)

See **[docs/VS2019.md](docs/VS2019.md)** for the full walkthrough. In short:

- **Tests only (no CUDA):** open `vs2019\RTPercept.sln` → Release|x64 → build.
  Builds the tracking library and the portable unit tests; only needs Eigen.
- **Full pipeline (CUDA + TensorRT + DXGI):**
  ```bat
  set TENSORRT_ROOT=C:\TensorRT-10.x
  set EIGEN3_INCLUDE_DIR=C:\libs\eigen-3.4.0
  scripts\generate_vs2019.bat pipeline
  cmake --build build\vs2019-pipeline --config Release
  ```
  Produces `rtpercept.exe`. Toolset v142, C++17, x64, CUDA arch sm_89.

### Cross-platform (CMake, for the tracking/queue tests)

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release      # add -DBUILD_PIPELINE=ON for CUDA
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## Tests

| Test | What it checks | Platform |
|------|----------------|----------|
| `test_tracking`   | lapjv assignment, Kalman convergence, stable track IDs, occlusion recovery | any |
| `test_spsc_queue` | FIFO + full/empty edges, 1M-item single-producer/consumer stress | any |
| `test_capture`    | DXGI duplication smoke test: frame validity, size, capture rate | Windows + GPU |

## Export a YOLOv8 engine

```bash
yolo export model=yolov8s.pt format=onnx opset=12 dynamic=False imgsz=640
trtexec --onnx=yolov8s.onnx --saveEngine=yolov8s.fp16.engine --fp16
# INT8 (needs a calibration cache):
trtexec --onnx=yolov8s.onnx --saveEngine=yolov8s.int8.engine --int8 \
        --calib=calib.cache
```

> The DXGI capture path is Windows-only. The TensorRT / ByteTrack code is
> portable and compiles on Linux for testing against a file/camera source.

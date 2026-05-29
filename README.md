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

Requires: CUDA 12.x, TensorRT 8.6/10.x, Eigen3, and the Windows SDK (D3D11/DXGI).

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

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

# Physics

## Overview
Moss uses Jolt Physics to provide fast, stable, and modern rigid-body dynamics for games and simulations.
It is designed to take full advantage of hardware acceleration and vectorized instructions across multiple CPU architectures.
[Jolt Physics 5.3.0](https://github.com/jrouwe/JoltPhysics)

Supported Architectures:
- ```X86/x64``` - SSE2, SSE4.1, SSE4.2, AVX, AVX2, or AVX512.
- ```ARM32/ARM64``` - NEON, FP16
- ```RISC-V64```
- ```LoongArch64```
- ```PowerPC64LE```

> [!NOTE]  
> Moss automatically selects the best available SIMD path at runtime, ensuring optimal performance without manual configuration.
> You can optionally force a specific instruction path for testing, benchmarking, or for low-end CPUs.

## Macros
```cpp
```
## Enums
```cpp
```
## Structs
```cpp
```
## Classes
```cpp
```
## Functions
```cpp
```

# Contributing to Moss

Thank you for contributing to **Moss Framework**, a modular, high-performance game-development framework for 2D and 3D applications.

Moss is an active-development project built around modular systems, explicit performance characteristics, portability, and stable public APIs. Contributions are welcome across the framework, documentation, examples, tests, tooling, and platform backends.

## Before You Start

For substantial architectural or public-API changes, open an issue first so the approach can be discussed before significant implementation work begins.

Small bug fixes, documentation improvements, tests, examples, and clearly scoped maintenance changes can normally go directly into a pull request.

Please check existing issues and pull requests before starting work.

## Development Requirements

Moss currently uses:

- CMake 3.20 or newer
- C++17
- Git
- The platform SDKs and graphics/audio/XR dependencies required by the subsystem you are changing

Moss supports multiple platforms and graphics APIs, so platform-specific work should be tested on every practical target available to the contributor.

## Building

```bash
git clone https://github.com/TxbiG/Moss.git
cd Moss

cmake -S . -B build
cmake --build build
```

Run the available tests when enabled by the build configuration:

```bash
ctest --test-dir build --output-on-failure
```

Moss contains optional platform and backend functionality. Do not assume every optional dependency is available on every machine; record unavailable targets in the pull request.

## Repository Structure

- `include/Moss/` — public framework headers
- `src/` — framework implementation
- `docs/` — documentation and developer guides
- `examples/` — example applications
- `external/` — third-party dependencies
- `performance/` — benchmarks and profiling tools
- `.github/workflows/` — CI

## Areas for Contribution

Useful contributions include:

- Rendering and GPU backends
- 2D/3D physics
- Audio and spatial audio
- Input and haptics
- Networking and multiplayer
- OpenXR and XR functionality
- GUI systems
- Platform backends
- Navigation
- CMake, installation, and package/export support
- Performance and profiling
- Tests and regression coverage
- Examples and documentation

The current project focus includes reliable CMake configuration/install/export behaviour, backend parity, smaller public APIs, and broader tests/examples. Keep changes aligned with those goals where practical.

## Design Guidelines

- Prefer clear, portable C++17.
- Keep public interfaces small and intentional.
- Keep platform/backend implementation details behind appropriate abstractions.
- Preserve existing ownership and lifetime conventions.
- Avoid unnecessary allocations or blocking work in real-time paths.
- Avoid unrelated formatting or refactoring changes in focused pull requests.
- Update documentation when public behaviour changes.

Moss deliberately uses C-like handles and descriptors for many public APIs. New APIs should follow the established style unless there is a concrete reason to introduce a different abstraction.

## Bindings

Moss has bindings for:

- C — CMoss
- C# — MossSharp
- Rust — MossRS
- Java — JavaMoss
- JavaScript — MossJS
- Lua — LuaMoss
- Python — PyMoss

Changes to public Moss APIs may therefore require corresponding binding updates. When changing a public header, check which bindings consume the affected API and document any follow-up work.

## API and ABI Compatibility

Prefer additive, backwards-compatible changes where practical.

When changing public headers or exported interfaces, consider:

- Function and type compatibility
- Struct layout
- Ownership and lifetime
- Nullability
- Calling conventions
- Platform-specific behaviour
- Binding compatibility

Breaking changes should be explicitly called out in the pull request.

## Testing

For bug fixes:

1. Reproduce the original problem.
2. Add a regression test when practical.
3. Implement the fix.
4. Run the relevant test suite.
5. Test affected backends/platforms where possible.

For rendering, GPU, audio, XR, and platform changes, state exactly which backend and platform were tested.

## Commit Messages

Conventional Commit prefixes are recommended:

```text
feat: add Vulkan texture upload path
fix: prevent invalid audio device access
docs: document renderer resource ownership
test: add physics collision regression
refactor: simplify platform backend
perf: reduce per-frame allocations
build: improve CMake installation
ci: add backend build coverage
```

Keep commits focused and avoid mixing unrelated changes.

## Pull Requests

A good pull request should:

- Explain what changed.
- Explain why the change is needed.
- Keep the scope focused.
- Include tests or explain why testing was not possible.
- Mention affected platforms/backends.
- Update documentation where appropriate.
- Identify public API or ABI changes.
- Include reproduction steps for bug fixes.

## Reporting Bugs

Include:

- Moss commit/version
- Operating system and architecture
- Compiler and version
- CMake version
- Graphics/audio/XR backend where relevant
- Reproduction steps or a minimal example
- Expected behaviour
- Actual behaviour
- Relevant logs, traces, or stack information

## Security

Do not publish sensitive security issues, credentials, private data, or exploitable details in a public issue. Use GitHub's private security-reporting mechanism when it is enabled for the repository.

## Licence

Moss is distributed under the **MIT License**. Contributions should be compatible with the repository's licence and existing third-party licence requirements.

Thank you for helping improve Moss.

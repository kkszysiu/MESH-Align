# CLAUDE.md

Project guidance lives in **[AGENTS.md](AGENTS.md)** — read it first. It covers
the build/test/run commands, the architecture, conventions, and the hard-won
gotchas. Design detail is in [`docs/`](docs/).

## Quick reference

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release   # first configure needs network (CPM)
cmake --build build --parallel
ctest --test-dir build --output-on-failure       # 5 headless suites must stay green
./build/app/meshalign                            # run
```

## Don't forget (full list in AGENTS.md)

- **`core` links no GL/ImGui** — strict layering `app → render → core`.
- Eigen: `cross` needs `<Eigen/Geometry>`, `inverse()` needs `<Eigen/LU>` (link
  fails, not compile).
- Include GL only via `ma_gl/GL.h` (macOS framework vs glad).
- PLY: write binary; ASCII goes through our own reader (tinyply ASCII is broken).
- pfd default path is a folder — pass empty, and dialogs are non-blocking.
- GL only on the main thread; analysis runs on a worker.
- After changing the analysis pipeline, update `docs/ALGORITHMS.md` and run `ctest`.

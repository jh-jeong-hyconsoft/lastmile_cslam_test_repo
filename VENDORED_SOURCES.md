# Vendored Sources

- `Multi-Robot-Graph-SLAM-client` and `Multi-Robot-Graph-SLAM-server` were
  vendored from https://github.com/aserbremen/Multi-Robot-Graph-SLAM (BSD
  2-Clause).
- Per-package licenses are preserved in `Multi-Robot-Graph-SLAM-*/LICENSE` and
  within their dependencies:
  - `src/fast_gicp/LICENSE`
  - `src/mrg_slam/LICENSE`
  - `src/small_gicp/LICENSE`
  - `src/ndt_omp/LICENSE`
  - Packages without a dedicated LICENSE inherit the parent repository license.
- All git metadata from the vendored sources was removed so this repository can
  track them directly rather than as submodules.

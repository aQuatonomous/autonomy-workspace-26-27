# asv_wave_sim: waves, hydrodynamics and the team simulator

**To run the simulator, see [SIMULATION.md](../../SIMULATION.md) at the repo root.** It covers
setup, running, courses, the judge and troubleshooting.

This folder holds two things:

- **The team simulator:** `sim/` (run scripts), `course/` (2027 and 2026 course descriptions),
  `tools/` (course and model generators) and `gz-waves-models/` (boat model, course element
  models, worlds). The judge and autolabel packages are `src/sim_judge` and `src/sim_autolabel`.
- **The wave plugins** (`gz-waves/`): Stephen Mainwaring's
  [asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim), Gazebo Harmonic plugins for FFT
  ocean waves and surface-vessel hydrodynamics (`gz-waves1`). The workspace build compiles them;
  `sim/setup_sim.sh` installs their dependencies (`libcgal-dev`, `libfftw3-dev`). See the upstream
  repository for the plugin documentation.

The plugins are licensed under GPL-3.0 (CGAL and FFTW dependencies).

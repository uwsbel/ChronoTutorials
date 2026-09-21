# Chrono FMU Tutorials

Two worked examples of exporting Project Chrono models as **FMI 3.0 co-simulation FMUs** and running them against MATLAB/Simscape models.

1. **`spring_system/`** — a simple two-mass spring. The starting point: minimal source, minimal build, all-static.
2. **`lunar_lander/`** — a lander touching down on Chrono CRM/SCM deformable terrain. A realistic model that needs an external-process workaround for its dynamic dependencies.

Author: Ahmed Ansari (and Claude)

---

## Before you start

You need a **static** Chrono build with FMU export enabled. The FMUs here link against it. When you configure Chrono, set:

- `BUILD_SHARED_LIBS = OFF`
- `CH_USE_MSVC_STATIC_RUNTIME = ON` (Windows)
- `CH_ENABLE_MODULE_FMI = ON`

Point the tutorials at that build with `Chrono_DIR` during configuration (see below). The lander's terrain solver additionally needs a **second, shared** Chrono build with the VSG/FSI modules — more on that in its own section.

> **Note:** compiled outputs are git-ignored — `.fmu`, `.dll`, `.exe`, `build/`, and the MATLAB `slprj/` cache are not in the repo. You build the FMUs yourself with the steps below.

---

## Repo layout

```
spring_system/
  monolithic/          # plain Chrono program (baseline, no FMU)
  fmi_implementation/
    src/               # the FMU source: two_mass_spring_fmu.{h,cpp}
    matlab/            # Simulink model + params.m
    scripts/           # python: compare monolithic vs co-sim results
    results/           # saved plots/CSV

lunar_lander/
  CAD/                 # SolidWorks + exported OBJ/STL geometry
  chrono_fmu/
    fmu/               # the lander FMU: crm_fmu.{h,cpp}  (static)
    crm_terrain/       # external terrain solver: crm_terrain.cpp  (shared)
    common/            # messages.h — shared FMU <-> solver message format
    resources/         # solver .exe + all DLLs get packaged here
  matlab/              # Simscape model + startup.m
```

---

## Tutorial 1 — Spring system

The Chrono body and the spring live inside the FMU; the second body and the driving force live in Simscape. Each step, Simscape sends the body position in, and the FMU returns the spring force.

**Build the FMU:**

```bash
cd spring_system/fmi_implementation
cmake -B build -S . -DChrono_DIR=/path/to/chrono_build_static/cmake
cmake --build build --config Release
```

This produces `build/FMU3cs_two_mass_spring/FMU3cs_two_mass_spring.fmu`.

**Run it:** open `matlab/`, run `params.m` (sets masses, stiffness, timestep, and adds the FMU to the path), then open and run the Simulink model. `scripts/compare_mono_cosim.py` plots the co-simulation result against the monolithic baseline in `spring_system/monolithic/`.

---

## Tutorial 2 — Lunar lander on CRM terrain

Same idea, bigger model: the lander body is in Simscape, the deformable terrain is in Chrono. But the terrain modules (VSG, FSI) **cannot** be built statically, so they can't go inside the FMU. The workaround splits the work into two processes:

- **`crm_terrain/`** — an external `.exe` built against the **shared** Chrono build (VSG + FSI + Vehicle). This runs the actual terrain simulation.
- **`fmu/`** — the static FMU. It holds no terrain modules; it talks to the terrain `.exe` over ZeroMQ (message format in `common/messages.h`).

When the terrain solver builds, its `.exe` and **all** its DLLs are copied into `chrono_fmu/resources/`, so the finished FMU carries its dependencies with it.

**Build the external terrain solver first** (against the *shared* Chrono build):

```bash
cd lunar_lander/chrono_fmu/crm_terrain
cmake -B build -S . -DChrono_DIR=/path/to/chrono_build_shared/cmake
cmake --build build --config Release
```

This builds `crm_terrain.exe` and stages it plus its DLLs into `../resources/`.

**Then build the FMU** (against the *static* Chrono build). It also needs ZeroMQ (`cppzmq` / `libzmq`, e.g. via vcpkg):

```bash
cd lunar_lander/chrono_fmu/fmu
cmake -B build -S . -DChrono_DIR=/path/to/chrono_build_static/cmake
cmake --build build --config Release
```

This produces `build/FMU3cs_crm/FMU3cs_crm.fmu`.

**Run it:** open `lunar_lander/matlab/`, run `startup.m` (adds the FMU to the path, sets the lander's initial height and pitch, opens the Simscape model), then run the model.

---

## How the FMUs are built in the source

Both FMUs follow the same pattern:

- The CMake calls `find_package(Chrono COMPONENTS FMI ...)` on the static build, then pulls in the `fmu-forge/fmi3` machinery via `FetchContent`, which creates the FMU target and handles packaging.
- Your `.cpp`/`.h` are attached to that target with `target_sources`.
- The `FmuComponent` class derives from `chrono::fmi3::FmuChronoComponentBase`. Its **simulation behavior** is defined by overriding the base hooks — `doStepIMPL`, `enterInitializationModeIMPL`, `exitInitializationModeIMPL` — while inputs, outputs, and parameters are declared with `AddFmuVariable`.
- The MSVC runtime is forced to `/MT` to match the static Chrono build. This must match or the FMU won't link/run correctly.

Start with `spring_system/fmi_implementation/src/two_mass_spring_fmu.cpp` — it's the shortest and clearest illustration of all of the above.

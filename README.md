# UKF Target Tracking Accelerator

This repository is a work-in-progress project for building an Unscented Kalman Filter (UKF) target-tracking pipeline in two stages:

1. Simulate and validate the UKF algorithm in Python.
2. Translate the UKF datapath into Verilog as a hardware accelerator.

The intended workflow is:

1. Start with the software model in [`simulation/`](/workspaces/UKF-target-tracking-accelarator/simulation).
2. Use the simulation results to understand state ranges, noise tuning, and fixed-point requirements.
3. Implement the same UKF building blocks in [`UKFaccelarator/verilog/`](/workspaces/UKF-target-tracking-accelarator/UKFaccelarator/verilog).

## Repository layout

- [`simulation/simul.py`](/workspaces/UKF-target-tracking-accelarator/simulation/simul.py): main Python UKF simulation for a synthetic 3D target trajectory.
- [`simulation/fixed_point_analysis.py`](/workspaces/UKF-target-tracking-accelarator/simulation/fixed_point_analysis.py): range analysis to help choose fixed-point formats for RTL.
- [`simulation/grid_search.py`](/workspaces/UKF-target-tracking-accelarator/simulation/grid_search.py): older parameter search script.
- [`UKFaccelarator/verilog/`](/workspaces/UKF-target-tracking-accelarator/UKFaccelarator/verilog): early Verilog implementation of UKF blocks and top-level accelerator structure.
- [`UKFaccelarator/scripts/run_sim.sh`](/workspaces/UKF-target-tracking-accelarator/UKFaccelarator/scripts/run_sim.sh): helper script to build and launch the Verilog simulation waveform.

## What the code does

### Python stage: simulate the UKF

The Python model tracks a synthetic 3D moving target with state:

- position: `x, y, z`
- velocity: `vx, vy, vz`

The simulation:

- generates a smooth ground-truth trajectory,
- adds noisy position measurements,
- runs UKF predict/update steps,
- computes tracking RMSE,
- optionally saves a 3D plot to `simulation/ukf_simulation.png`.

This stage is the algorithm reference model for the later hardware design.

### Verilog stage: build a UKF accelerator

The Verilog directory now contains an architectural SR-UKF accelerator model that follows the block diagram in `diagram.jpg`. It is organized as a control-driven datapath with reusable blocks for the prediction and update stages.

#### Architecture figure

![SR-UKF accelerator architecture](/workspaces/UKF-target-tracking-accelarator/diagram.jpg)

Figure: SR-UKF accelerator concept used in this project. The control FSM schedules sigma-point generation, selects the `f` or `h` path in the shared compute fabric, buffers intermediate sigma points, and reuses the same matrix-operation backend (`QR`, `cholupdate`, `trisolve`) across both the prediction and update phases.

#### Accelerator architecture

The top-level [`UKFaccelarator/verilog/ukf.v`](/workspaces/UKF-target-tracking-accelarator/UKFaccelarator/verilog/ukf.v) is split into the following blocks:

- `wc_unit`: generates the UKF weights used for the center sigma point and the off-center sigma points.
- `cordic_unit`: produces a sigma-point spread term from the current state and stands in for the square-root / scaling support in the diagram.
- `spg_unit`: generates sigma-point pairs around the current state.
- `shared_compute_fabric`: shared datapath for the motion model `f` during prediction and the measurement model `h` during update.
- `uacc_unit`: weighted accumulator used to build the predicted state mean and the predicted measurement mean.
- `sigma_buffer`: stores propagated sigma points so the same shared backend can be reused later.
- `pacc_unit`: accumulates a spread metric around the mean, representing the covariance-building pass.
- `qr_unit`: placeholder QR backend stage.
- `cholupdate_unit`: placeholder Cholesky-update backend stage.
- `trisolve_unit`: placeholder triangular-solve / state-correction stage.
- top-level `ukf` FSM: orchestrates the two passes and schedules the shared compute fabric and matrix backend.

#### Dataflow

Prediction stage:

1. `wc_unit` and `cordic_unit` prepare weights and sigma-point spread.
2. `spg_unit` emits sigma-point pairs.
3. `shared_compute_fabric` applies the motion model `f`.
4. `uacc_unit` accumulates the predicted state mean.
5. `sigma_buffer` stores propagated sigma points.
6. `pacc_unit` scans the stored sigma points to build the spread metric.
7. `qr_unit` represents the square-root covariance backend for the predicted state.

Update stage:

1. The control FSM reuses `spg_unit` and the shared compute fabric, this time selecting the measurement path `h`.
2. `uacc_unit` accumulates the predicted measurement mean.
3. `sigma_buffer` stores measurement-space sigma points.
4. `pacc_unit` builds the innovation spread metric.
5. `qr_unit`, `cholupdate_unit`, and `trisolve_unit` model the shared matrix backend used to update the state estimate.

#### Current implementation scope

- The RTL matches the intended accelerator structure and handshake flow.
- The `f` path is implemented as a simple constant-velocity propagation.
- The `h` path maps the state to position-only measurements.
- `QR`, `cholupdate`, and `trisolve` are currently behavioral placeholder blocks with explicit latency, not full fixed-point linear algebra implementations yet.
- The design is useful for validating control flow, buffering, scheduling, and testbench integration before replacing the placeholders with full math kernels.

## Prerequisites

### Python

- `python3`
- `numpy`
- optional: `matplotlib` for saving the trajectory plot

Install Python packages if needed:

```bash
python3 -m pip install numpy matplotlib
```

### Verilog tools

- `cmake`
- `iverilog`
- `vvp`
- optional: `gtkwave`

## How to run

### 1. Run the UKF software simulation

From the repository root:

```bash
python3 simulation/simul.py
```

Expected output is similar to:

```text
Finished UKF simulation. Position RMSE: 2.143
Saved plot to simulation/ukf_simulation.png
```

This produces:

- console output with the final RMSE
- an image at `simulation/ukf_simulation.png` if `matplotlib` is installed

### 2. Run fixed-point analysis for the hardware design

This script estimates numerical ranges for UKF states and intermediate values so you can choose fixed-point formats for Verilog:

```bash
python3 simulation/fixed_point_analysis.py
```

You can also run a smaller sweep during development:

```bash
python3 simulation/fixed_point_analysis.py --trials 2 --time-steps 20
```

This prints suggested Q formats for signals such as sigma points, covariance matrices, and Kalman gain.

### 3. Configure the Verilog build

```bash
cmake -S UKFaccelarator/verilog -B UKFaccelarator/verilog/build
```

### 4. Build and run the Verilog simulation

After the build directory is configured and `iverilog` is installed:

```bash
cmake --build UKFaccelarator/verilog/build --target run_sim
```

Or use the helper script:

```bash
bash UKFaccelarator/scripts/run_sim.sh
```

The helper script:

- configures CMake if needed,
- builds and runs the `iverilog` simulation,
- writes the waveform to `UKFaccelarator/verilog/build/ukf.vcd`.

If `gtkwave` is installed and a GUI display is available, the helper script opens the waveform automatically.

### 5. Run in GitHub Codespaces

GitHub Codespaces is usually headless, so the recommended flow is:

```bash
bash UKFaccelarator/scripts/run_sim.sh
```

This will still compile and run the Verilog simulation in Codespaces and generate:

```text
UKFaccelarator/verilog/build/ukf.vcd
```

Because Codespaces normally does not provide a desktop display, `gtkwave` may not open directly even if it is installed. In that case:

- keep the generated `.vcd` file as the simulation artifact,
- download it from the Codespace, or
- open it using a compatible waveform viewer extension/tool.

## Current status

- The Python UKF simulation is runnable and serves as the reference implementation.
- The fixed-point analysis script is usable for planning RTL datapath widths.
- The Verilog side now includes a simulated accelerator architecture with control FSM, shared compute fabric, sigma-point buffering, and placeholder matrix-backend stages.
- The Verilog math backend is still not a numerically complete SR-UKF implementation.
- The current Codespaces-friendly RTL flow now compiles `ukf.v`, runs `tb_ukf.v`, and generates a waveform file with `iverilog` and `vvp`.

## Goal of the project

The goal is to move from a floating-point software UKF model to a hardware-oriented implementation that can eventually accelerate UKF target tracking in Verilog using fixed-point arithmetic and modular datapath blocks.

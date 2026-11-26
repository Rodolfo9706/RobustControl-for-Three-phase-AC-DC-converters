# Robust Regulation of Three-Phase AC/DC Converters

![MATLAB](https://img.shields.io/badge/MATLAB-R2024a%2B-orange?logo=mathworks)
![C2000](https://img.shields.io/badge/TI-C2000%20F28379D-red)
![Status](https://img.shields.io/badge/Status-Under%20Revision-yellow)
![License](https://img.shields.io/badge/License-MIT-green)

Simulation models and hardware implementation (Simulink + C2000 and stand-alone C) for the manuscript:

> **"Robust Regulation of Three-Phase AC/DC Converters"**
> *Rodolfo Isaac Verdín Monzón and Gerardo Flores*
> *IEEE Transactions on Control Systems Technology*, manuscript 26-1303 (under revision).

All code follows the notation and equation numbers of the revised manuscript.

---

## 📌 Overview

The repository implements a **robust adaptive passivity-based controller** that regulates the dc-link voltage of a **three-phase PWM rectifier** under:

- an unknown load conductance $`g_L`$ and dc-bus disturbances $`\delta(t)`$;
- saturation of the modulation indices, $`p \in \mathcal{S} = [-a_d, a_d] \times [-a_q, a_q]`$.

The controller acts on the passive output of the converter. It adapts the active-current reference through an online estimate $`\hat{g}`$ of the equivalent bus conductance, with smooth projection and σ-modification, combined with a filtered and bounded proportional action.

---

## 📐 Averaged Model

In the $`dq`$ frame of the amplitude-invariant Park transformation, with the $`d`$ axis aligned with the source voltage, the averaged model (1) is

```math
L \dot{i}_d = -R i_d - \frac{1}{2} p_d v_o + L \omega i_q + E_m
```

```math
L \dot{i}_q = -R i_q - \frac{1}{2} p_q v_o - L \omega i_d
```

```math
C \dot{v}_o = \frac{3}{4} (p_d i_d + p_q i_q) - g_L v_o + C \delta(t)
```

where $`i_d, i_q`$ are the $`dq`$ line currents, $`v_o`$ is the dc-link voltage with reference $`v_o^d`$, $`p_d, p_q`$ are the averaged modulation indices, $`E_m`$ and $`\omega`$ are the peak phase voltage and angular frequency of the source, and $`g_L = 1/R_L`$.

---

## 🛠️ Control Law (16)

**Saturated port law**

```math
p_d = \mathrm{sat}_{a_d} \left( p_d^{eq}(g_r) - \frac{2 k_s}{3} \left[ I(g_r)\, e_3 - v_o^d\, e_1 \right] \right)
```

```math
p_q = \mathrm{sat}_{a_q} \left( p_q^{eq}(g_r) + \frac{2 k_q}{3 v_o^d}\, i_q \right)
```

**Reference conductance, filter and estimator**

```math
g_r = \hat{g} - \kappa \bar{\nu} \tanh(\nu / \bar{\nu})
```

```math
\dot{\nu} = \lambda_f (e_3 - \nu)
```

```math
\dot{\hat{g}} = \mathrm{Proj} \left( \hat{g},\; -2\gamma v_o^d e_3 - \gamma \sigma (\hat{g} - g_c) \right)
```

with $`e_1 = i_d - I(g_r)`$, $`e_3 = v_o - v_o^d`$, the equilibrium current

```math
I(g) = \frac{E_m - \sqrt{E_m^2 - \frac{8}{3} R (v_o^d)^2 g}}{2 R}
```

and the feedforward $`p_d^{eq}(g) = 2(E_m - R I(g))/v_o^d`$, $`p_q^{eq}(g) = -2 L \omega I(g)/v_o^d`$.

---

## 📁 Repository Structure

```text
.
├── Simulink_Simulation/       # Averaged model (1) in closed loop with (16), Section IV
│   └── <model>.slx
├── Hardware_Implementation/   # Implementation used in the experiments, Section V
│   ├── ThreephaseControl.slx  # LAUNCHXL-F28379D model, code generation + External Mode
│   ├── ctrl16.m               # Controller (16), code of the MATLAB Function block in the model
│   ├── params_tcst.m          # Experimental parameters (Tables IV–V, certified gains, PLL)
│   ├── spll3ph_srf.m          # SRF-PLL, code of the MATLAB Function block in the model
│   └── spll_gains.m           # PLL gains from settling time and damping
└── Embedded_Code/             # Reference C implementation of the control chain (CCS)
    ├── robust_controller.h    # Controller (16), forward Euler, Ts = 1e-4 s
    ├── transforms_and_pll.h   # Clarke/Park transforms and SRF-PLL
    └── main_launchpad.c       # 10 kHz ISR: ADC → Clarke → PLL → Park → (16) → inv. Park → SVPWM
```

---

## 🧰 Requirements

### Simulations (`Simulink_Simulation/`)

- MATLAB and Simulink R2024a or later (no additional toolboxes are needed for the averaged model)

### Simulink model and code generation (`Hardware_Implementation/`)

- MATLAB and Simulink R2024a or later
- MATLAB Coder, Simulink Coder and Embedded Coder (code generation for the target)
- C2000 Microcontroller Blockset (ADC, ePWM and target support for the LAUNCHXL-F28379D)
- Motor Control Blockset (PWM Reference Generator)
- Simscape Electrical (abc→dq0 and dq0→αβ0 transforms). Equivalent transform blocks can be used instead, e.g. the C2000 Clarke, Park and Inverse Park Transformation blocks of the C2000 Microcontroller Blockset (TI DMC library) or the Clarke, Park and Inverse Park Transform blocks of the Motor Control Blockset, provided they use the same convention as model (1): amplitude-invariant transform with the $`d`$ axis aligned at the PLL angle
- Texas Instruments Code Composer Studio (CCS) and C2000Ware, installed and linked through the C2000 Microcontroller Blockset **Hardware Setup**

After installing the blockset, run the Hardware Setup (*Home → Add-Ons → Manage Add-Ons → C2000 Microcontroller Blockset → Setup*), select the TI Delfino F2837xD / LAUNCHXL-F28379D and give the paths to CCS and C2000Ware. To list the products required by the model on your installation:

```matlab
dependencies.toolboxDependencyAnalysis({'Hardware_Implementation/ThreephaseControl.slx'})
```

### Stand-alone C code (`Embedded_Code/`)

- Texas Instruments Code Composer Studio (CCS) with the TI C2000 code generation tools (C28x compiler)
- C2000Ware (driverlib and F2837xD device support: `F28x_Project.h`, `driverlib.h`, `device.h`)
- LAUNCHXL-F28379D with its on-board XDS100v2 debug probe (drivers installed with CCS)

Create a CCS project for the F28379D from a C2000Ware driverlib example, add the three files of `Embedded_Code/`, and add the ADC/ePWM configuration where indicated in `main_launchpad.c`.

---

## 🔌 Experimental Setup

| Component | Description |
|---|---|
| Controller board | TI LAUNCHXL-F28379D (C2000 Delfino), 10 kHz control and PWM |
| Power stage | Three-phase IGBT bridge, 2.5 µs dead time |
| Source | BLDC generator driven by a motor (trapezoidal back-EMF, ≈ 446 Hz) |
| Line filter | $`L`$ = 2 mH, $`R`$ = 0.1 Ω per phase |
| DC link | $`C`$ = 2.5 mF, resistive load $`R_L`$ = 45 Ω (plus switched load for disturbance tests) |
| Current sensors | ACS712-05B (185 mV/A), with analog conditioning to the 0–3 V ADC range |
| Voltage sensor | Resistive divider on the dc link |

---

## 🚀 Quick Start

```bash
git clone https://github.com/Rodolfo9706/RobustControl-for-Three-phase-AC-DC-converters.git
cd RobustControl-for-Three-phase-AC-DC-converters
```

### Simulations (Section IV)

Open the model in `Simulink_Simulation/` and run it. It simulates the averaged model (1) with the parameters of Table I; the grid angle is known, so no PLL is used.

### Experiments (Section V)

The experiments run on a LAUNCHXL-F28379D. The Simulink model `Hardware_Implementation/ThreephaseControl.slx` is the implementation: Embedded Coder generates and deploys the code (ADC → Clarke/Park → PLL → controller (16) → inverse Park → SVPWM at 10 kHz), and **External Mode** is used to monitor the signals and change the voltage reference in real time. `Embedded_Code/` contains a reference hand-written C version of the same control chain for Code Composer Studio (peripheral configuration not included).

1. Run `Hardware_Implementation/params_tcst.m`. Choose the gains with `GAINS`:
   - `'tableV'`: gains of Table V (performance tuning, not covered by condition (18));
   - `'certified'`: gains satisfying conditions (17)–(18) over $`v_o^d \in [24, 29]`$ V.

   The script also creates the PLL parameters `PL` (settling time 20 ms, damping 0.7).
2. In the MATLAB Function blocks, declare `P` (`ctrl16`) and `PL` (`spll3ph_srf`) as non-tunable parameters.
3. Connect the PLL output `wt` to the angle input of the abc→dq0 and dq0→αβ0 transforms, and `freq` through a gain `2*pi` to the input `w` of `ctrl16`.
4. Use discrete integrators (forward Euler, Ts = 1e-4 s) for `ĝ` (initial condition `P.gc`) and `ν` (initial condition 0).
5. Calibrate the current-sensor offsets and the dc-voltage divider.
6. Run the model in External Mode (*Hardware → Monitor & Tune*). The dashboard knob sets $`v_o^d`$; scopes show $`v_o`$, $`i_d`$, $`i_q`$, the modulation indices and the controller states.

---

## ⚙️ Main Parameters

| Parameter | Simulation (Table I) | Experiment (Tables IV–V) |
|---|---|---|
| $`E_m`$ | 80 V | 12 V (fundamental, peak phase) |
| $`f`$ | 60 Hz | ≈ 446 Hz (BLDC generator) |
| $`L`$, $`R`$, $`C`$ | 10 mH, 0.3 Ω, 2.5 mF | 2 mH, 0.1 Ω, 2.5 mF |
| $`v_o^d`$ | 160 V | 24–29 V |
| $`k_s`$, $`k_q`$ | 0.02, 30 | 0.1, 100 |
| $`\gamma`$, $`\kappa`$, $`\lambda_f`$ | 1.44e-3, 4e-3, 1000 | 2.5e-3, 8e-3, 1500 |

The complete experimental set, including the certified gains, is in `Hardware_Implementation/params_tcst.m`.

---

## 📖 Citation

```bibtex
@article{verdin2026robust,
  title   = {Robust Regulation of Three-Phase {AC/DC} Converters},
  author  = {Verd{\'i}n Monz{\'o}n, Rodolfo Isaac and Flores, Gerardo},
  journal = {IEEE Transactions on Control Systems Technology},
  note    = {Under revision},
  year    = {2026}
}
```

---

## 📬 Contact

Rodolfo Isaac Verdín Monzón — r.verdinmonzon@gmail.com
Laboratorio de Percepción y Robótica (LAPyR), Centro de Investigaciones en Óptica (CIO), México.

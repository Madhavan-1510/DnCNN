# DnCNN Hardware Accelerator — Zynq FPGA

A real-time image denoising accelerator implementing DnCNN on a Zynq-7000 SoC with HDMI input and output.

---

## What's in this repo

```
.
├── bd/
│   └── design_1_bd.tcl           # Block design — source this to recreate everything
├── ip_repo/
│   ├── dncnn_top_v1_1/           # Custom DnCNN accelerator IP (packaged)
│   └── digilent-vivado-ip/       # Digilent HDMI IPs (git submodule)
├── src/
│   ├── tlast_gen.v                # RTL source — AXI-Stream TLAST generator
│   └── EEPROM_8b.v                # RTL source — I2C EEPROM interface
└── README.md
```

---

## Requirements

- **Vivado 2023.1** (the TCL script will error on other versions — upgrade instructions inside the TCL if needed)
- A **Zynq-7000** board (e.g. Zybo Z7, Arty Z7)
- **Digilent board files** installed → [guide here](https://digilent.com/reference/programmable-logic/guides/installing-vivado-and-vitis)

---

## Step 1 — Clone with submodules

```bash
git clone --recurse-submodules https://github.com/<your-username>/<your-repo>.git
cd <your-repo>
```

> Already cloned without submodules? Run:
> ```bash
> git submodule update --init --recursive
> ```

---

## Step 2 — Open Vivado and create a new project

1. Open **Vivado 2023.1**
2. Click **Create Project** → give it a name and location → choose **RTL Project**
3. On the *Add Sources* screen, add the RTL files:
   - `src/tlast_gen.v`
   - `src/EEPROM_8b.v`
4. Select your **Zynq board** on the board selection screen
5. Finish creating the project

---

## Step 3 — Add the IP repositories

In the **Vivado Tcl Console** at the bottom, run:

```tcl
set_property ip_repo_paths {
    ./ip_repo/dncnn_top_v1_1
    ./ip_repo/digilent-vivado-ip/ip_repo
} [current_project]

update_ip_catalog
```

> This tells Vivado where to find `dncnn_top` and the Digilent `dvi2rgb`/`rgb2dvi` IPs.
> You should see them appear in the IP Catalog after this.

---

## Step 4 — Source the block design

In the same Tcl Console:

```tcl
source bd/design_1_bd.tcl
```

Vivado will rebuild the entire block design automatically — all connections, parameters, and addresses included.

When it finishes, right-click the BD in the **Sources** panel → **Create HDL Wrapper** → select **Let Vivado manage wrapper and auto-update** → OK.

---

## Step 5 — Generate Bitstream

```
Flow Navigator (left panel) → Generate Bitstream
```

Or via Tcl:
```tcl
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1
```

---

## IP breakdown

| IP | Source | Notes |
|---|---|---|
| `dncnn_top_v1_1` | This repo (`ip_repo/`) | Custom HLS accelerator, packaged IP |
| `dvi2rgb_0` | Digilent submodule | HDMI input decoder |
| `rgb2dvi_0` | Digilent submodule | HDMI output encoder |
| `tlast_gen` | This repo (`src/`) | RTL module ref — add as source file |
| `EEPROM_8b` | This repo (`src/`) | RTL module ref — add as source file |
| `axi_vdma` × 3 | Xilinx built-in | Video DMA, included with Vivado |
| `v_tc_0` | Xilinx built-in | Video Timing Controller |
| `v_axi4s_vid_out_0` | Xilinx built-in | Stream to parallel video |
| Everything else | Xilinx built-in | Standard AXI infrastructure |

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `dncnn_top` not found | Make sure `ip_repo_paths` includes `./ip_repo/dncnn_top_v1_1`, then `update_ip_catalog` |
| `dvi2rgb`/`rgb2dvi` not found | Run `git submodule update --init --recursive`, check path includes `digilent-vivado-ip/ip_repo` |
| `tlast_gen` or `EEPROM_8b` module not found | These are RTL references — make sure the `.v` files are added as sources in your project before sourcing the TCL |
| Wrong Vivado version error | The TCL targets **2023.1** — either use that version or follow the upgrade steps printed in the error message |
| Wrapper not created | Right-click BD in Sources → *Create HDL Wrapper* → *Let Vivado manage* |

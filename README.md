# Tiny-DnCNN INT8 Hardware Accelerator

Real-time image denoising on a Zynq-7000 SoC. A custom RTL accelerator runs a 17-layer DnCNN inference pipeline entirely in programmable logic, with live HDMI input and output handled through Digilent HDMI IPs and three AXI VDMAs. Weights are stored in DDR3 and streamed into on-chip BRAM between layers by the ARM PS.

**Target:** XC7Z020-1CLG400C (PYNQ-Z2) | **Vivado:** 2023.1 | **Precision:** INT8 (QAT)

---

## How it works

DnCNN learns to predict the noise in an image, not the clean image itself. The final output is:

```
clean = noisy − DnCNN(noisy)
```

The hardware maps this to three physical pipelines that reuse a single MAC array across all 17 layers:

| Stage | Layers | Operation | Module |
|---|---|---|---|
| Ingestor | Layer 1 | Conv 3×3 + ReLU, 1→16ch | `dncnn_ingestor` |
| Workhorse | Layers 2–16 | Conv 3×3 + BN (folded) + ReLU, 16→16ch × 15 iterations | `dncnn_workhorse` |
| Finisher | Layer 17 | Conv 3×3 only, 16→1ch | `dncnn_finisher` |
| Residual | Post-network | `clean = clamp(raw − noise, 0, 255)` | `residual_sub` |

The Workhorse is one physical circuit that loops 15 times. Between each iteration the ARM loads the next layer's weights from DDR3 into the on-chip weight BRAM.

---

## Architecture

```
HDMI IN ──► dvi2rgb ──► async_fifo_ingest ──► dncnn_top ──► async_fifo_eject ──► rgb2dvi ──► HDMI OUT
                                                    │
                                          ┌─────────┴──────────┐
                                          │   dncnn_ingestor   │  Layer 1
                                          │   dncnn_workhorse  │  Layers 2–16 (loop)
                                          │   dncnn_finisher   │  Layer 17
                                          │   residual_sub     │  Final output
                                          └─────────┬──────────┘
                                                    │
                              PS ARM ◄──► AXI4-Lite (axilite_slave)
                              DDR3  ◄──► AXI VDMA × 3 (frame buffers + weight DMA)
```

**Clock domains:**
- `clk_pixel_in` — HDMI RX pixel clock (recovered from TMDS)
- `clk_pixel_out` — HDMI TX pixel clock
- `clk_core` — DnCNN processing clock (100 MHz from PS FCLK)

Async FIFOs with Gray-code pointers handle all clock domain crossings.

---

## Resource Utilization (Post-Implementation, XC7Z020)

Numbers from `report_utilization -hierarchical` on the fully routed design.

### Top-Level Summary

| Resource | Used | Available | Utilization |
|---|---|---|---|
| LUT (total) | 22,646 | 53,200 | 42.6% |
| Logic LUTs | 17,890 | 53,200 | 33.6% |
| LUTRAMs | 4,254 | 17,400 | 24.4% |
| SRLs | 502 | — | — |
| Flip-Flops | 21,865 | 106,400 | 20.5% |
| RAMB36 | 14 | 140 | 10.0% |
| RAMB18 | 167 | 280 | 59.6% |
| DSP48E1 | 160 | 220 | 72.7% |

### Per-Module Breakdown

| Module | LUTs | FFs | RAMB18 | DSPs | Notes |
|---|---|---|---|---|---|
| `dncnn_top_0` (total) | 12,671 | 6,502 | 164 | 160 | Entire accelerator |
| ↳ `dncnn_workhorse` | 7,212 | 2,067 | 64 | 128 | 15-layer loop, mac_array_16x16 |
| ↳ `dncnn_finisher` | 3,681 | 1,869 | 64 | 16 | Layer 17, mac_array_16x1 |
| ↳ `dncnn_ingestor` | 859 | 923 | 4 | 16 | Layer 1, mac_array_1x16 |
| ↳ `axilite_slave` | 328 | 152 | 0 | 0 | AXI4-Lite register bank |
| ↳ `async_fifo_ingest` | 291 | 66 | 0 | 0 | HDMI RX → core CDC |
| `axi_vdma_0` | 2,442 | 3,938 | 0 (6×RAMB36) | 0 | Primary VDMA |
| `axi_vdma_1` | 1,950 | 2,978 | 0 (4×RAMB36) | 0 | — |
| `axi_vdma_2` | 895 | 1,688 | 0 (1×RAMB36) | 0 | — |
| `axi_interconnect_0` | 2,133 | 2,231 | 0 | 0 | Includes AXI dwidth converter |
| `axi_interconnect_1` | 637 | 678 | 0 | 0 | — |
| `axi_interconnect_2` | 202 | 315 | 0 | 0 | — |
| `v_tc_0` | 941 | 2,593 | 0 | 0 | Video Timing Controller |
| `dvi2rgb_0` | 300 | 396 | 0 | 0 | HDMI RX decoder |
| `rgb2dvi_0` | 157 | 142 | 0 | 0 | HDMI TX encoder |
| `EEPROM_8b_0` | 54 | 47 | 1 | 0 | I2C EEPROM + dependencies |

> RAMB36 vs RAMB18: Vivado reports the workhorse and finisher line buffers as RAMB18 pairs (64 each) rather than RAMB36 tiles. 64 RAMB18 = 32 RAMB36 equivalent — consistent with the C=16 line buffer budget.

---

## Repo Structure

```
.
├── src/
│   ├── design/
│   │   ├── dncnn_top.v            # Top-level, instantiates all stages
│   │   ├── dncnn_ingestor.v       # Layer 1: 1→16ch Conv + ReLU
│   │   ├── dncnn_workhorse.v      # Layers 2–16: 16→16ch Conv + BN + ReLU
│   │   ├── dncnn_finisher.v       # Layer 17: 16→1ch Conv
│   │   ├── residual_sub.v         # clean = raw − noise
│   │   ├── mac_array_16x16.v      # 16 OC × 16 IC parallel MAC array
│   │   ├── mac_array_1x16.v       # 1 OC × 16 IC (ingestor)
│   │   ├── mac_array_16x1.v       # 16 OC × 1 IC (finisher)
│   │   ├── mac_unit.v             # Single DSP48E1 MAC cell
│   │   ├── line_buffer_16ch.v     # 2-row BRAM line buffer, 16ch
│   │   ├── line_buffer_1ch.v      # 3-row BRAM line buffer, 1ch
│   │   ├── weight_bram.v          # 16-lane byte-split weight BRAM
│   │   ├── sat_relu.v             # Saturation + optional ReLU
│   │   ├── async_fifo_ingest.v    # CDC: HDMI RX → core clock
│   │   ├── async_fifo_eject.v     # CDC: core clock → HDMI TX
│   │   ├── axilite_slave.v        # AXI4-Lite register interface
│   │   ├── tlast_gen.v            # AXI-Stream TLAST generator (BD module ref)
│   │   ├── EEPROM_8b.vhd          # I2C EEPROM top-level (BD module ref)
│   │   ├── TWI_SlaveCtl.vhd       # EEPROM dependency
│   │   ├── GlitchFilter.vhd       # EEPROM dependency
│   │   └── SyncAsync.vhd          # EEPROM dependency
│   └── tb/                        # Testbenches
├── bd/
│   └── design_1_bd.tcl            # Block design TCL — source to recreate
├── ip_repo/
│   ├── viip/                      # Packaged dncnn_top IP (dncnn_top_v1_1)
│   └── digilent-vivado-ip/        # Git submodule — dvi2rgb / rgb2dvi
├── constrs/
│   └── top.xdc                    # Pin assignments and clock constraints
├── docs/
│   └── design_1.pdf               # Block design diagram
└── README.md
```

---

## Block Design IPs

| IP | Source | Role |
|---|---|---|
| `dncnn_top_v1_1` | `ip_repo/viip/` | Custom accelerator (packaged IP) |
| `tlast_gen` | `src/design/` | RTL module ref — TLAST for AXI-Stream |
| `EEPROM_8b` | `src/design/` | RTL module ref — I2C EEPROM (needs 4 VHDL files) |
| `dvi2rgb_0` | Digilent submodule | HDMI/DVI input decoder |
| `rgb2dvi_0` | Digilent submodule | HDMI/DVI output encoder |
| `axi_vdma` × 3 | Xilinx built-in | Frame buffer DMA |
| `v_tc_0` | Xilinx built-in | Video Timing Controller |
| `v_axi4s_vid_out_0` | Xilinx built-in | AXI-Stream to parallel video |
| `axi_interconnect` × 2 | Xilinx built-in | AXI bus fabric |
| `proc_sys_reset` × 2 | Xilinx built-in | Per-domain synchronized resets |

---

## Replication

### Prerequisites
- Vivado 2023.1 (the BD TCL targets this version exactly — other versions will warn)
- PYNQ-Z2 board files installed → [Digilent guide](https://digilent.com/reference/programmable-logic/guides/installing-vivado-and-vitis)

### 1. Clone

```bash
git clone --recurse-submodules https://github.com/<your-username>/<your-repo>.git
cd <your-repo>
```

> Already cloned without submodules? Run: `git submodule update --init --recursive`

### 2. Create a new Vivado project

Open Vivado 2023.1 → **Create Project** → RTL Project → select **PYNQ-Z2**.

On the *Add Sources* screen, add **all files** from `src/design/` — both `.v` and all four `.vhd` files. The EEPROM will fail elaboration if any VHDL dependency is missing.

### 3. Set IP repo paths

In the Vivado Tcl Console:

```tcl
set_property ip_repo_paths {
    ./ip_repo/viip
    ./ip_repo/digilent-vivado-ip/ip
} [current_project]

update_ip_catalog
```

### 4. Source the block design

```tcl
source bd/design_1_bd.tcl
```

When complete: right-click the BD in Sources → **Create HDL Wrapper** → **Let Vivado manage**.

### 5. Generate bitstream

```
Flow Navigator → Generate Bitstream
```

Post-synthesis checks: DSP < 187, BRAM36 < 119, LUT < 45K, all timing WNS > 0.

---

## AXI4-Lite Register Map

The ARM controls the accelerator through `axilite_slave`. Base address set in the BD address editor.

| Offset | Register | Description |
|---|---|---|
| `0x00` | `CR_CTRL` | Bit 0 = START, Bit 1 = soft reset |
| `0x04` | `SR_STATUS` | Bit 0 = IDLE, Bit 1 = RUNNING, Bit 2 = DONE |
| `0x08` | `CR_LAYER` | Current layer index (0–16), written by ARM before each layer |
| `0x0C` | `SR_IRQ` | Interrupt status — cleared by writing 1 |

ARM FSM per frame: load weights into DDR3 → write `CR_LAYER` → pulse `CR_START` → wait for `done_irq` on IRQ_F2P → repeat for next layer → read final frame from VDMA.

---

## Timing & Power (Post-Implementation)

### Timing

| Clock | Period | Frequency |
|---|---|---|
| `clk_fpga_0` (core/AXI) | 10.000 ns | 100 MHz |
| `clk_fpga_1` | 5.000 ns | 200 MHz |
| `clk_pixel_in` (HDMI RX) | 13.468 ns | 74.25 MHz |

**WNS: −8.967 ns — timing is NOT met.** 4,563 failing endpoints, all in the `clk_fpga_0` domain.

The critical path runs from `dncnn_workhorse → mac_array_16x16 → DSP48E1` output through a 26-level logic chain (17× CARRY4 + LUTs) to the output register, with a data path delay of 15.8 ns against a 10 ns budget. The accumulator adder tree in the workhorse is the bottleneck.

Hold timing passes (WHS: +0.050 ns, 0 failing endpoints).

**To fix timing:**
- Pipeline the accumulator adder tree in `mac_array_16x16.v` — break the CARRY4 chain across 2–3 registered stages
- Or reduce `clk_fpga_0` to 50–75 MHz in the PS clock configuration and re-run implementation
- 2 cross-domain paths (`clk_fpga_0 → PixelClk_int`) are also failing with WNS −3.65 ns — review `set_false_path` constraints in XDC for those CDC crossings

### Methodology Warnings

| ID | Severity | Description | Count |
|---|---|---|---|
| TIMING-6/7/8 | Critical Warning | No common clock between related clock pairs | 2 each |
| TIMING-9 | Warning | Unknown CDC logic — no double-register synchronizer detected | 1 |
| TIMING-16 | Warning | Large setup violations | 1,000 |
| TIMING-18 | Warning | Missing input/output delay constraints | 3 |

TIMING-9 is the most important: Vivado cannot verify the async FIFO CDC paths because the synchronizer structure isn't recognized. Running `report_cdc` and switching to `XPM_CDC` primitives is recommended to clear this.

### Power

| | Power (W) |
|---|---|
| Total On-Chip | **1.899** |
| Dynamic | 1.736 |
| Device Static | 0.163 |
| Junction Temperature | 46.9 °C |
| Max Ambient | 63.1 °C |

> Confidence level is **Low** (no switching activity file provided). Actual dynamic power may differ once real video traffic is applied.

---

## Key Design Decisions

**C = 16 channels** — C=32 needs 256 DSPs (over budget). C=16 uses 128 DSPs (58%), fits in RAMB18 pairs for line buffers, and delivers visually clean denoising at INT8 with QAT.

**BN folded offline** — Batch normalization scale/shift is folded into the conv weights before export. No BN hardware at runtime. Saves DSPs and simplifies the datapath.

**Three VDMAs** — One captures raw HDMI frames (S2MM), one feeds them to the accelerator (MM2S), one drives the display (MM2S). Triple-buffering keeps HDMI output live while the accelerator processes the next frame.

**Async FIFOs for CDC** — The HDMI pixel clock is recovered from TMDS and is not phase-locked to PS FCLK. All domain crossings use Gray-code pointer async FIFOs with `set_false_path` in XDC.

**Expected throughput** — ~0.14 FPS at 640×480. The Workhorse loops 15× sequentially, each pass takes ~442 ms at 100 MHz. The display stays live showing the previous clean frame.

---

## Reference

Zhang, K., Zuo, W., Chen, Y., Meng, D., & Zhang, L. (2017). *Beyond a Gaussian Denoiser: Residual Learning of Deep CNN for Image Denoising.* IEEE Transactions on Image Processing, 26(7), 3142–3155.

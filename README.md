# Tiny-DnCNN INT8 Hardware Accelerator on PYNQ-Z2

A from-scratch RTL implementation of the DnCNN image denoising neural network as a Zynq-7000 hardware accelerator. 17 convolutional layers run entirely in programmable logic using INT8 arithmetic on DSP48E1 blocks. Live HDMI input is denoised and output over HDMI. The ARM Cortex-A9 PS orchestrates the layer-by-layer execution loop via interrupts while the display stays live throughout.

**Board:** PYNQ-Z2 (XC7Z020-1CLG400C) | **Tool:** Vivado 2023.1 | **Precision:** INT8 with QAT

---

## Table of Contents

1. [What DnCNN Does — and What That Means in Hardware](#1-what-dncnn-does)
2. [The Core Architectural Insight — Three Pipelines, Not Seventeen](#2-three-pipelines-not-seventeen)
3. [Why C=16 Channels — The Only Numbers That Work](#3-why-c16-channels)
4. [Fixed-Point Format — INT8 Everywhere and Why](#4-fixed-point-format)
5. [Batch Normalization — How It Disappears at Runtime](#5-batch-normalization-fold)
6. [System Architecture — Full Picture](#6-system-architecture)
7. [Clock Domains and Async FIFOs](#7-clock-domains-and-async-fifos)
8. [DDR3 Memory Map](#8-ddr3-memory-map)
9. [The Layer-Loop Problem — How the ARM and RTL Cooperate](#9-the-layer-loop-problem)
10. [Residual Subtraction — Solved](#10-residual-subtraction)
11. [HDMI Underflow — Solved with Triple Buffering](#11-hdmi-underflow)
12. [RTL Modules — Design Decisions for Each](#12-rtl-modules)
13. [Line Buffer Math — Why BRAM and LUTRAM Are Both Needed](#13-line-buffer-math)
14. [MAC Array — DSP48E1 Usage and Parallelism](#14-mac-array)
15. [Datapath Bit-Widths — Why 22-bit Accumulators](#15-datapath-bit-widths)
16. [AXI4-Lite Register Map](#16-axi4-lite-register-map)
17. [Block Design — IPs and Connections](#17-block-design)
18. [Resource Utilization — Real Numbers from Implementation](#18-resource-utilization)
19. [Timing — Status and What Needs Fixing](#19-timing)
20. [Power](#20-power)
21. [Replication Steps](#21-replication-steps)

---

## 1. What DnCNN Does

DnCNN (Zhang et al., 2017) is a residual denoising network. Unlike earlier denoisers that try to directly predict the clean image, DnCNN learns to predict **the noise itself**. The clean output is:

```
clean_image = noisy_input − DnCNN(noisy_input)
```

This residual formulation works because noise patterns are structurally simpler to learn than clean image content. In practice it outperforms classical methods like BM3D on Gaussian noise.

### Network Structure (DnCNN-S for σ=25)

| Layer Group | Count | Operations | Channels |
|---|---|---|---|
| Layer 1 | 1 | Conv 3×3 + ReLU | 1 → C |
| Layers 2–16 | 15 | Conv 3×3 + BatchNorm + ReLU | C → C |
| Layer 17 | 1 | Conv 3×3 only | C → 1 |

The original paper uses C=64. This implementation uses C=16 — see Section 3 for why.

---

## 2. Three Pipelines, Not Seventeen

The most important architectural decision in this project: the hardware does not have 17 separate pipelines. It has **three**, and the Workhorse runs 15 times in a loop.

```
Layer 1      → dncnn_ingestor   : 1→16ch, Conv + ReLU
Layers 2–16  → dncnn_workhorse  : 16→16ch, Conv + BN (folded) + ReLU   [×15 iterations]
Layer 17     → dncnn_finisher   : 16→1ch, Conv + residual subtract
```

Between each Workhorse iteration, the ARM PS:
1. Loads the next layer's weights from DDR3 into on-chip BRAM
2. Swaps the VDMA read/write addresses (ping-pong between feature space A and B)
3. Pulses the START register
4. Waits for the done interrupt

The RTL itself is stateless between layers — it does not know which layer it is running. The ARM sets `REG_LAYER_TYPE` before each START pulse to select which pipeline is active (0=ingestor, 1=workhorse, 2=finisher). This is the correct separation of concerns: the ARM handles sequencing and memory management, the RTL handles arithmetic at full throughput.

---

## 3. Why C=16 Channels

The resource budget on XC7Z020 is the hard ceiling. Every channel decision flows from it.

| Resource | Total | Safe Budget (85%) |
|---|---|---|
| LUT | 53,200 | 45,220 |
| DSP48E1 | 220 | 187 |
| BRAM36 | 140 | 119 |

The table below shows what each channel count costs in on-chip BRAM (two line-buffer rows, which is the dominant cost) and DSPs:

| Channels C | 2 BRAM rows (bits) | BRAM36 tiles | DSPs (8 IC ∥, 16 OC ∥) | Decision |
|---|---|---|---|---|
| 8 | 81,920 | 5 | 64 | Too weak — poor PSNR |
| **16** | **163,840** | **10** | **128** | **✅ chosen** |
| 32 | 327,680 | 18 | 256 | Over DSP budget |
| 64 | 655,360 | 36 | 1,024 | Impossible |

C=16 uses 128 DSPs (58% of 220) and 10 BRAM36 tiles for line buffers — well within budget. It leaves room for the three AXI VDMAs, AXI interconnects, HDMI IPs, and video timing controller that the block design needs. Denoising quality at C=16 with QAT-trained INT8 weights is visibly clean on AWGN σ=25, which is the target noise level.

C=32 was ruled out because the MAC array alone would need 256 DSPs before accounting for any other IP. C=8 was ruled out because quality degrades too far.

---

## 4. Fixed-Point Format

### Symmetric Signed INT8 Everywhere

All weights and activations use **signed INT8**, range −128 to +127, with a per-tensor scale factor:

```
real_value = INT8_value × scale_factor
scale_factor = max_abs_value / 127.0
```

Post-ReLU activations are always ≥ 0, so the sign bit is always 0 after ReLU — you could use UINT8 for one extra bit of range. This was deliberately avoided. Mixing signed weights with unsigned activations requires a mixed-signed DSP48E1 configuration (the B port is signed but A is unsigned in that mode), which changes the accumulator handling between layers and adds error-prone complexity. Signed INT8 everywhere keeps the hardware uniform across all 17 layers.

### Why NOT FP16 or BF16

The DSP48E1 is designed for integer multiply-accumulate. Floating-point on 7-series FPGAs requires either Xilinx FP IP (LUT-heavy, slow) or very careful manual packing. For a CNN with identical arithmetic across all layers, INT8 is the correct choice. It also reduces weight storage from 39KB (FP32) to under 10KB.

### QAT vs PTQ

Post-Training Quantization (PTQ) — train in FP32, quantize afterward — causes noticeable quality loss at C=16 because the reduced channel count already limits representational capacity. Quantization-Aware Training (QAT) is used instead: fake quantization nodes are inserted during training that simulate INT8 rounding, so the network learns to compensate. This recovers most of the quality lost from the channel reduction.

---

## 5. Batch Normalization Fold

### What BN Does at Training Time

Batch normalization normalizes each channel's activation across a mini-batch:

```
BN(x) = γ × (x − μ) / √(σ² + ε) + β
```

where γ, β are learned per-channel, and μ, σ² are running estimates of batch statistics.

### Why BN Disappears at Inference

At inference time, μ and σ² are fixed (from training). BN is then just a per-channel linear transform: scale by `γ/√(σ²+ε)` and shift by `β − γμ/√(σ²+ε)`. Since the convolution that precedes BN is also linear, these two linear operations compose into one:

```
y = M × conv(x) + B
  where M = γ / √(σ² + ε)
        B = β − γ × μ / √(σ² + ε)
```

M is folded directly into the convolution weights and B into the bias before export:

```python
def fold_bn_into_weights(conv_w, conv_b, bn_gamma, bn_beta, bn_mean, bn_var, eps=1e-5):
    M = bn_gamma / np.sqrt(bn_var + eps)
    B = bn_beta - bn_gamma * bn_mean / np.sqrt(bn_var + eps)
    folded_w = conv_w * M[:, np.newaxis, np.newaxis, np.newaxis]
    folded_b = conv_b * M + B
    return folded_w, folded_b
```

**Result: there is no BN module in the RTL.** The `bn_apply.v` file referenced in older design documents does not exist in this implementation. Any reference to it is an error from earlier iterations. The workhorse pipeline is strictly: window → MAC accumulate → bias add → sat_relu. The BN constants are baked into the folded weights stored in DDR3.

---

## 6. System Architecture

```
                     Zynq PS (ARM Cortex-A9)
                     Python / PYNQ overlay
                     ┌──────────┬──────────┬──────────┐
                     │ VDMA_0   │ VDMA_1   │ VDMA_2   │
                     │ display  │ feat A/B │ raw buf  │
                     └────┬─────┴────┬─────┴────┬─────┘
                          │          │           │
                   ═══════════════ DDR3 512MB ══════════════
                          │          │           │
                AXI4-Lite │          └───────────┘
                ctrl/status│              AXI-Stream (MM2S/S2MM)
                          │
                 ┌─────────▼───────────────────────────────┐
                 │         DnCNN Accelerator Core           │
                 │              clk_core 100 MHz            │
HDMI IN  ───────►  async_fifo_ingest                        │
74.25 MHz        │       │                                  │
                 │  ┌────▼──────────────────────────────┐   │
                 │  │  dncnn_ingestor / workhorse /      │   │
                 │  │  finisher (selected by ARM)        │   │
                 │  └───────────────────────────────┬───┘   │
                 │                                  │        │
                 │              async_fifo_eject ◄──┘        │
                 └───────────────────────────────────────────┘
                                    │
HDMI OUT ◄──────────────────────────┘
74.25 MHz
```

### Data flow — one frame

1. HDMI RX (`dvi2rgb`) recovers pixel data from TMDS at 74.25 MHz
2. Pixels cross into `clk_core` via `async_fifo_ingest`
3. **Layer 1 (Ingestor):** pixels processed 1→16ch, result written to DDR3 Feature Space A via VDMA. Raw frame simultaneously written to RAW_FRAME_BUFFER via a second VDMA (needed for residual subtraction at the end)
4. **Layers 2–16 (Workhorse × 15):** ARM reconfigures VDMA each iteration to read from A/write to B, then B/write to A, alternating. Each iteration processes the full 640×480 feature map
5. **Layer 17 (Finisher):** reads final feature space + RAW_FRAME_BUFFER simultaneously, subtracts noise prediction from original, writes clean RGB to DISPLAY_BUFFER
6. ARM flips display VDMA pointer to the new clean frame
7. `async_fifo_eject` bridges clean pixels back to `clk_pixel_out` for `rgb2dvi` HDMI TX

---

## 7. Clock Domains and Async FIFOs

Three independent clock domains exist in the design:

| Domain | Frequency | Source |
|---|---|---|
| `clk_pixel_in` | 74.25 MHz | MMCM driven by recovered TMDS clock from `dvi2rgb` |
| `clk_core` / `clk_fpga_0` | 100 MHz | PS FCLK0 |
| `clk_pixel_out` | 74.25 MHz | Second MMCM output for TX |

`clk_pixel_in` and `clk_pixel_out` appear to be the same frequency but come from different PLL outputs and are **not phase-correlated**. They must be treated as fully asynchronous to each other.

### Why Async FIFOs and Not Two-FF Synchronizers

A two-FF synchronizer works for single-bit control signals. Multi-bit data (8-bit pixels, 128-bit feature vectors) cannot be synchronized that way — you would need to ensure all bits are stable on the same clock edge, which is only guaranteed if the source domain freezes the bus while the capture domain samples it. An async FIFO solves this with Gray-coded read/write pointers: only one bit of the pointer changes per increment, so the 2-FF synchronizer only ever needs to resolve a 1-bit transition. The data itself never crosses the domain boundary — only the pointers do.

### Boundary A: HDMI RX → Core

```
Write side: clk_pixel_in  (74.25 MHz) — pixels arrive here from dvi2rgb
Read side:  clk_core      (100 MHz)   — accelerator consumes here

Core reads faster than pixels arrive (100 vs 74.25 MHz), so overflow
is not a concern. Depth: 1024 entries (one BRAM18 tile). Sized for
line-start synchronization headroom.
```

### Boundary B: Core → HDMI TX

```
Write side: clk_core      (100 MHz)
Read side:  clk_pixel_out (74.25 MHz) — HDMI needs pixels metered at exact rate

This FIFO does not buffer a whole frame. The frame sits in DDR3.
VDMA feeds HDMI TX continuously from the display buffer.
This async FIFO only bridges the core-clock output of the finisher
to the pixel clock domain before rgb2dvi.
Depth: 1024 entries (one BRAM18 tile).
```

The `set_false_path` constraints in XDC must cover the pointer synchronizer paths. The methodology report flags TIMING-9 (unknown CDC logic) because Vivado cannot recognize the custom async FIFO structure as a synchronizer. Switching to `XPM_CDC` primitives from the Xilinx library would resolve this warning and give Vivado visibility into the CDC paths.

---

## 8. DDR3 Memory Map

The Zynq PS DDR3 is 512 MB (0x0000_0000–0x1FFF_FFFF). Linux and PYNQ runtime occupy the lower region. The accelerator's buffers are allocated above OS space.

```
0x0000_0000  ├── Zynq Linux OS + heap              ~256 MB
0x1000_0000  ├── PYNQ Python buffers               ~16 MB
0x1100_0000  ├── DISPLAY_BUFFER_0                  640×480×3 = 0.88 MB  ← shown on monitor now
0x1120_0000  ├── DISPLAY_BUFFER_1                  0.88 MB               ← previous frame
0x1140_0000  ├── DISPLAY_BUFFER_2                  0.88 MB               ← DnCNN writing here
0x1160_0000  ├── FEATURE_SPACE_A                   640×480×16 = 4.68 MB  ← intermediate maps
0x1200_0000  ├── FEATURE_SPACE_B                   4.68 MB               ← ping-pong partner
0x12A0_0000  ├── RAW_FRAME_BUFFER                  640×480×1 = 0.29 MB   ← original noisy frame
0x12C0_0000  ├── WEIGHT_STORE                      ~34 KB (see below)
0x12CA_0000  └── free
```

### Weight Layout

Layer 1 weights: 1 IC × 16 OC × 3×3 = 144 bytes  
Layers 2–16 weights: 16 IC × 16 OC × 3×3 = 2,304 bytes each × 15 = 34,560 bytes  
Layer 17 weights: 16 IC × 1 OC × 3×3 = 144 bytes  
**Total: 34,848 bytes ≈ 34 KB** — trivially small in a 512 MB DDR3.

Weights are packed row-major: `[layer][oc][ic][ky][kx]`, all signed INT8.

---

## 9. The Layer-Loop Problem

### The RTL Has No AXI-MM Master

The `dncnn_top` Verilog has no AXI memory master port. It cannot touch DDR3 directly. It cannot reconfigure the VDMAs. Between layers, someone must stop the VDMA, swap the read/write addresses, load the next layer's weights, and restart everything. **That someone is the ARM.**

### Solution: IRQ-Driven Layer Loop in PS

The RTL's job per invocation is simple: process one full frame (640×480 pixels) through one pipeline stage, then assert `done_irq` and return to IDLE. It does not need to know the layer number. The ARM knows.

```
RTL FSM:  IDLE → RUNNING → DONE → IDLE
          (START pulse)   (done_irq)

ARM loop:
  # Layer 1: ingest raw HDMI + save copy to RAW_FRAME_BUFFER
  write REG_LAYER_TYPE = 0  (INGESTOR)
  configure VDMA_1 S2MM → FEATURE_SPACE_A
  configure VDMA_2 S2MM → RAW_FRAME_BUFFER
  pulse CR_CTRL START
  wait for done_irq

  # Layers 2–16: workhorse ping-pong
  for layer in range(1, 16):
      write REG_LAYER_TYPE = 1  (WORKHORSE)
      read_addr  = FEATURE_SPACE_A if (layer % 2 == 1) else FEATURE_SPACE_B
      write_addr = FEATURE_SPACE_B if (layer % 2 == 1) else FEATURE_SPACE_A
      configure VDMA_1 MM2S → read_addr, S2MM → write_addr
      load weights for this layer into on-chip BRAM
      pulse CR_CTRL START
      wait for done_irq

  # Layer 17: finisher + residual subtract → display buffer
  write REG_LAYER_TYPE = 2  (FINISHER)
  configure VDMA_1 MM2S → final FEATURE_SPACE
  configure VDMA_2 MM2S → RAW_FRAME_BUFFER
  configure VDMA_0 S2MM → DISPLAY_BUFFER_2
  load weights for layer 17
  pulse CR_CTRL START
  wait for done_irq

  # Flip display
  configure VDMA_0 MM2S → DISPLAY_BUFFER_2
```

### Why Not DMA Descriptors or Scatter-Gather

Scatter-gather would add complexity with little benefit here. The layer loop is slow (seconds per iteration) — the ARM overhead of writing a few registers between layers is microseconds. IRQ-driven is the cleanest approach.

---

## 10. Residual Subtraction

DnCNN outputs a noise map, not the clean image. The clean image requires subtracting this noise map from the original noisy input:

```
clean = clamp(noisy_original − noise_prediction, 0, 255)
```

The problem: the noisy input was ingested at Layer 1. By the time the Finisher produces the noise map, 8+ seconds have passed. It cannot be held in a FIFO.

### Solution: RAW_FRAME_BUFFER in DDR3

During Layer 1, the input pixel stream is split (via AXI-Stream Broadcaster) and written to two destinations simultaneously:
- **VDMA_1 S2MM** → FEATURE_SPACE_A (for the accelerator to process)
- **VDMA_2 S2MM** → RAW_FRAME_BUFFER (kept untouched for 17 layers)

At Layer 17, the Finisher receives two streams:
- `s_axis_noise` from the final FEATURE_SPACE — the network's noise prediction
- `s_axis_raw` from RAW_FRAME_BUFFER — the original noisy pixels

Both VDMAs start simultaneously and read at the same rate (sequential reads, same burst length), so the streams stay pixel-aligned as long as neither is independently stalled. AXI-Stream FIFOs on both inputs with a joint handshake (`output_ready = noise_tvalid AND raw_tvalid`) guarantee this.

```verilog
// residual_sub.v — final stage of finisher
wire signed [8:0] diff = $signed({1'b0, raw_pixel}) - $signed({1'b0, noise_pixel});
assign clean_pixel = (diff > 9'sd127)  ?  8'd127  :
                     (diff < -9'sd128) ? -8'd128  :
                     diff[7:0];
```

---

## 11. HDMI Underflow — Triple Buffering

DnCNN takes approximately 7 seconds per frame. HDMI TX needs pixels at 74.25 MHz without a single clock gap. If the pixel pipeline goes dry, the monitor loses signal.

### Solution: VDMA Circular Display Buffer

Three display buffers live in DDR3. VDMA_0 runs in MM2S circular mode, continuously reading from the "current" buffer and looping — independent of DnCNN's processing state. The monitor sees a steady 30 FPS stream at all times.

```
While DnCNN processes frame N+1:
  VDMA_0 loops DISPLAY_BUFFER[idx_display] → HDMI TX
  Monitor shows frame N (the last completed clean frame)

DnCNN finishes frame N+1:
  ARM writes new base address to VDMA_0 MM2S register (atomic flip)
  Monitor now shows frame N+1
  DnCNN starts frame N+2, writing to the third buffer
```

This is the same double/triple buffer mechanism used in every GPU display driver. The ARM flip is a single register write and is effectively instantaneous compared to the frame period.

---

## 12. RTL Modules

### `dncnn_top.v` — Top-Level Wrapper

Instantiates all submodules, routes clocks, connects AXI-Stream interfaces, and exposes the packaged IP ports. The FSM lives here:

```
IDLE    — wait for CR_CTRL[0] START pulse
RUNNING — pass-through: data flows from VDMA through pipeline to VDMA
DONE    — assert done_irq one cycle, clear to IDLE
```

The ARM sets `REG_LAYER_TYPE` before each START. Based on this register, the top-level muxes inputs and outputs to the correct submodule (ingestor, workhorse, or finisher). This is the key to reusing one physical circuit for 15 Workhorse iterations.

---

### `dncnn_ingestor.v` — Layer 1

**Job:** 1 input channel → 16 output channels, 3×3 Conv + ReLU.

**Why this is its own module and not just the workhorse with IC=1:** The ingestor's line buffer is `line_buffer_1ch` — a much smaller structure (only 2 rows × 640 × 1 channel). Running the full `line_buffer_16ch` for a 1-channel input would waste 15/16 of the BRAM. The ingestor has 16 DSPs (one per output channel), each running 9 MAC cycles per pixel through the 3×3 kernel. Total: 9 cycles per pixel.

**Submodules:** `line_buffer_1ch`, `mac_array_1x16`, `sat_relu` (ENABLE_RELU=1)

---

### `dncnn_workhorse.v` — Layers 2–16

**Job:** 16 input channels → 16 output channels, 3×3 Conv + BN-folded + ReLU. Runs 15 times.

This is the critical module and the timing bottleneck. The MAC array (`mac_array_16x16`) runs 8 IC in parallel and 16 OC in parallel, iterating over the 9 kernel positions and 2 IC passes = 144 cycles per pixel. With 307,200 pixels per frame, one layer takes ~44M cycles at 100 MHz = 442 ms. 15 layers = 6.6 seconds.

**The line buffer** (`line_buffer_16ch`) holds two rows of 16-channel feature data in BRAM (10 BRAM36 equivalent). The third row (currently being written) lives in LUTRAM — see Section 13 for why this matters.

**BN note:** There is no BN module here. The BN scale and shift were folded into `folded_weights` and `folded_biases` at export time. The hardware pipeline is: window tap assembly → 128 DSP MACs → 16-wide bias add → 16-wide sat_relu.

**Submodules:** `line_buffer_16ch`, `mac_array_16x16`, `weight_bram`, `sat_relu` (ENABLE_RELU=1)

---

### `dncnn_finisher.v` — Layer 17

**Job:** 16 input channels → 1 output channel, 3×3 Conv (no BN, no ReLU), then residual subtraction.

The convolution uses `mac_array_16x1` — 16 DSPs, one per input channel, accumulating into a single output. After accumulation, `residual_sub` subtracts the noise prediction from the original pixel and clamps to [0, 255].

**Submodules:** `line_buffer_16ch`, `mac_array_16x1`, `residual_sub`, `sat_relu` (ENABLE_RELU=0)

---

### `mac_unit.v` — Single DSP48E1 MAC

One `mac_unit` wraps one `DSP48E1` primitive. INT8 weight goes to the B port (18-bit, sign-extended from 8-bit). INT8 activation goes to the A port (30-bit, sign-extended). OPMODE is set to accumulate: `P = A×B + P_prev` using the internal PCIN cascade. The `clear` input resets the accumulator at the start of each new pixel/output-channel group. Output is the lower 22 bits of the 48-bit P register.

The `(* use_dsp = "yes" *)` attribute is set on all MAC arrays to prevent Vivado from accidentally inferring LUT-based multipliers.

---

### `line_buffer_16ch.v` and `line_buffer_1ch.v`

Sliding-window line buffers for the 3×3 convolution. See Section 13 for full math. Key points:
- Two BRAM rows (y−1 and y) for registered reads
- One LUTRAM row (current, y+1) for combinational x+1 lookahead
- Zero-padding applied at borders: left/right columns and top row produce zeros instead of out-of-bounds accesses
- `top_row_active` flag from the FSM forces all row-0 taps to zero (padding row above the image)

---

### `weight_bram.v` — On-Chip Weight Cache

16-lane byte-split true dual-port BRAM. The ARM writes weights here (via AXI or DMA) at layer start. The MAC arrays read from it during processing. No ping-pong: at C=16, loading 2,304 bytes from DDR3 takes ~3 µs. A full layer processes in 442 ms. The ratio is 0.065% — ping-pong adds design complexity to solve a non-problem.

---

### `axilite_slave.v` — Control Interface

Standard AXI4-Lite slave providing the register interface between ARM and accelerator. See Section 16 for the full register map.

---

### `async_fifo_ingest.v` / `async_fifo_eject.v`

Clock-domain crossing FIFOs using independent-clock BRAM FIFOs with Gray-coded pointers. FWFT (First Word Fall Through) mode is used — standard mode has a 1-cycle read latency that complicates pixel pipeline timing. `almost_full` drives TREADY backpressure to prevent overflow when the core clock is slower than arrival rate.

---

### `tlast_gen.v` — AXI-Stream TLAST Generator

A small module reference in the block design (not a packaged IP). Generates the TLAST signal at the end of each horizontal line (when `col_ptr == WIDTH−1`). Required because the VDMA needs TLAST to delineate frame lines, but the upstream pixel source may not provide it.

---

### `EEPROM_8b.vhd` and Dependencies

I2C EEPROM interface used by the Digilent HDMI subsystem for DDC (Display Data Channel) — the protocol monitors use to report their capabilities. This is a VHDL module reference in the block design. It requires four files: `EEPROM_8b.vhd` (top), `TWI_SlaveCtl.vhd` (I2C state machine), `GlitchFilter.vhd` (deglitches SCL/SDA), `SyncAsync.vhd` (async-to-sync bridge). All four must be added as sources or the design will not elaborate.

---

## 13. Line Buffer Math

### Why Two BRAM Rows, Not Three

A 3×3 convolution needs three consecutive rows simultaneously (y−1, y, y+1). Storing all three in BRAM would be simple, but the x+1 column tap of the current row (y+1) needs a **combinational read** — you need `cur_row[col_ptr+1]` available in the same clock cycle you're computing the window. BRAM outputs are registered (1-cycle latency). You cannot read BRAM and use the result combinationally without adding pipeline stalls.

The solution: rows y−1 and y go into BRAM (fine for registered reads), and the current row y+1 goes into LUTRAM (distributed RAM), which supports synchronous write + combinational read in the same cycle:

```
row0 (y-1) → BRAM bank A  : registered read, no problem
row1 (y)   → BRAM bank B  : registered read, no problem
row2 (y+1) → LUTRAM       : combinational read for col_ptr+1 lookahead
```

### BRAM Usage for C=16

```
One row = 640 pixels × 16 channels × 8 bits = 81,920 bits
BRAM36 in 512-deep × 36-bit config = 18,432 bits per tile
Tiles per row = ceil(81,920 / 18,432) = 5 BRAM36
Two rows = 10 BRAM36 tiles
```

In practice Vivado synthesized these as RAMB18 pairs (64 RAMB18 for workhorse + 64 for finisher = 128 RAMB18 = 64 RAMB36 equivalent, across all the line buffers in the design).

### LUTRAM for cur_row

```
cur_row = 640 entries × 16 channels × 8 bits = 81,920 bits
7-series LUTRAM: 32 bits per LUT
LUTs required = 81,920 / 32 = 2,560 LUTs = 4.8% of 53,200 total
```

Force Vivado to infer LUTRAM with the synthesis attribute:

```verilog
(* ram_style = "distributed" *)
reg [CHANNELS*8-1:0] cur_row [0:WIDTH-1];
```

Do not count this in the BRAM budget. It is in the LUT budget (and is why LUTRAMs show 4,254 in the utilization report).

---

## 14. MAC Array

### DSP48E1 Configuration

The Xilinx DSP48E1 computes `P = A×B + C` with A=30-bit, B=18-bit, P=48-bit. For INT8×INT8 multiplication:

- Weight (INT8) → sign-extended to 18 bits → B port
- Activation (INT8) → sign-extended to 18 bits (with A-port headroom) → A port
- Accumulation: OPMODE set to `P = A×B + P_prev` using internal feedback
- Output: lower 22 bits of the 48-bit P register (see Section 15 for why 22 bits)

### Parallelism in the Workhorse

The `mac_array_16x16` runs 8 IC in parallel × 16 OC in parallel = 128 DSP48E1 instantiations. Over a 3×3 kernel with 16 total IC channels, this requires 2 IC passes (8 IC per pass × 2 passes = 16 IC) × 9 kernel positions = 18 clock cycles per output pixel position. One full frame: 18 × 307,200 = 5.5M cycles per layer.

The reason it wasn't made 16 IC in parallel (which would halve cycle count): that would need 256 DSPs for the Workhorse alone, slightly over the 220 total available and far over the 187 safe budget.

---

## 15. Datapath Bit-Widths

### Accumulator Sizing

```
INT8 × INT8 product range: [−128×127, 127×127] = [−16,256, +16,129] → 15 bits signed

One filter tap accumulates IC × kernel_size products:
  Layer 1:     1 × 9 = 9 products
  Layers 2–16: 16 × 9 = 144 products

Worst-case accumulator (layers 2–16):
  144 × 16,256 = 2,340,864 → needs ceil(log2(2,340,864)) + 1 sign bit = 22 bits signed

Headroom: 2^21 = 2,097,152 → not enough
           2^22 = 4,194,304 → covers 2,340,864 ✅
```

**Use signed [21:0] (22-bit) accumulators.** The `mac_unit.v` output is 22 bits. An earlier version used 21 bits — this caused silent accumulator overflow on maximum-magnitude inputs.

The `sat_relu` module takes the 22-bit accumulator and clamps to INT8 range:
- If ENABLE_RELU=1: negative values → 0, values > 127 → 127, else pass [7:0]
- If ENABLE_RELU=0 (finisher): values < −128 → −128, values > 127 → 127, else pass [7:0]

---

## 16. AXI4-Lite Register Map

Base address assigned in the Vivado Address Editor. Default: `0x43C0_0000`.

| Offset | Name | R/W | Description |
|---|---|---|---|
| `0x00` | `CR_CTRL` | W | Bit[0]: START pulse — write 1 to begin, auto-clears next cycle |
| `0x04` | `SR_STATUS` | R | `2'b00`=IDLE, `2'b01`=RUNNING, `2'b10`=DONE, `2'b11`=ERROR |
| `0x08` | `REG_LAYER_TYPE` | W | `0`=INGESTOR, `1`=WORKHORSE, `2`=FINISHER |
| `0x0C` | `REG_IMG_WIDTH` | W | Image width in pixels (default 640) |
| `0x10` | `REG_IMG_HEIGHT` | W | Image height in pixels (default 480) |
| `0x14` | `REG_WEIGHT_BASE` | W | BRAM address offset for this layer's weight set |
| `0x1C` | `IRQ_STATUS` | R | Bit[0]: done interrupt — cleared on read |
| `0x20` | `IRQ_ENABLE` | W | Bit[0]: enable done_irq to PS IRQ_F2P |
| `0x24` | `DEBUG_PIXEL_CNT` | R | Pixels processed in current run — useful for debug |

### ARM Handshake Sequence

```python
# PYNQ Python — one layer invocation
def run_layer(dncnn_ip, layer_type, weight_bram_offset):
    dncnn_ip.write(0x08, layer_type)          # set layer type
    dncnn_ip.write(0x14, weight_bram_offset)  # set weight base
    dncnn_ip.write(0x20, 1)                   # enable IRQ
    dncnn_ip.write(0x00, 1)                   # START pulse
    # wait for interrupt on IRQ_F2P[0]
    # ISR clears IRQ_STATUS by reading 0x1C
```

---

## 17. Block Design

### IPs

| IP | Source | Role |
|---|---|---|
| `dncnn_top_v1_1` | `ip_repo/viip/` | Custom DnCNN accelerator (packaged) |
| `tlast_gen` | `src/design/tlast_gen.v` | RTL module ref — TLAST generator for AXI-Stream |
| `EEPROM_8b` | `src/design/EEPROM_8b.vhd` + 3 VHDL deps | RTL module ref — I2C EEPROM for HDMI DDC |
| `dvi2rgb_0` | Digilent submodule | HDMI/DVI input — TMDS decode + clock recovery |
| `rgb2dvi_0` | Digilent submodule | HDMI/DVI output — TMDS encode |
| `axi_vdma_0` | Xilinx built-in | Display VDMA — MM2S circular to HDMI TX |
| `axi_vdma_1` | Xilinx built-in | Feature space VDMA — MM2S + S2MM for layer ping-pong |
| `axi_vdma_2` | Xilinx built-in | Raw frame VDMA — S2MM capture + MM2S for finisher |
| `axis_broadcaster_0` | Xilinx built-in | 1→2 AXI-Stream split for dual VDMA write at Layer 1 |
| `axis_data_fifo_0/1` | Xilinx built-in | Stream buffering at finisher inputs for joint handshake |
| `v_tc_0` | Xilinx built-in | Video Timing Controller — generates sync signals for HDMI TX |
| `v_axi4s_vid_out_0` | Xilinx built-in | AXI-Stream to parallel video output |
| `axi_interconnect_0/1/2` | Xilinx built-in | AXI bus fabric — HP port routing |
| `proc_sys_reset_0/1` | Xilinx built-in | Synchronized resets per clock domain |
| `processing_system7_0` | Xilinx built-in | Zynq PS — ARM, DDR3, HP/GP AXI ports, FCLK, IRQ |

### Key Connections

- PS7 HP0 → `axi_interconnect_0` → VDMA_0, VDMA_1 (display + feature space)
- PS7 HP1 → `axi_interconnect_1` → VDMA_2 (raw frame)
- PS7 GP0 → `axi_interconnect_2` → `dncnn_top` AXI4-Lite slave
- `dncnn_top.done_irq` → `xlconcat_0` → PS7 IRQ_F2P[0]
- `dvi2rgb_0.RGB` → `axis_broadcaster_0` → VDMA_1 S2MM + VDMA_2 S2MM (Layer 1 dual write)
- `dncnn_top.m_axis_out` → (through `async_fifo_eject`) → `rgb2dvi_0`

---

## 18. Resource Utilization (Post-Implementation, Routed)

From `report_utilization -hierarchical` on the fully routed design.

### Top-Level Summary

| Resource | Used | Available | % |
|---|---|---|---|
| Total LUTs | 22,646 | 53,200 | 42.6% |
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
| **`dncnn_top_0` (total)** | **12,671** | **6,502** | **164** | **160** | Entire accelerator |
| ↳ `dncnn_workhorse` | 7,212 | 2,067 | 64 | 128 | Dominates — 15× loop, 16×16 MAC |
| ↳ `dncnn_finisher` | 3,681 | 1,869 | 64 | 16 | Line buffer heavy, 16×1 MAC |
| ↳ `dncnn_ingestor` | 859 | 923 | 4 | 16 | Lightweight — 1ch line buffer |
| ↳ `axilite_slave` | 328 | 152 | 0 | 0 | Register bank |
| ↳ `async_fifo_ingest` | 291 | 66 | 0 | 0 | HDMI RX → core CDC |
| `axi_vdma_0` | 2,442 | 3,938 | 0 (6×RAMB36) | 0 | |
| `axi_vdma_1` | 1,950 | 2,978 | 0 (4×RAMB36) | 0 | |
| `axi_vdma_2` | 895 | 1,688 | 0 (1×RAMB36) | 0 | |
| `axi_interconnect_0` | 2,133 | 2,231 | 0 | 0 | Includes AXI dwidth converter |
| `axi_interconnect_1/2` | 839 | 993 | 0 | 0 | |
| `v_tc_0` | 941 | 2,593 | 0 | 0 | Video timing controller |
| `dvi2rgb_0` | 300 | 396 | 0 | 0 | HDMI RX |
| `rgb2dvi_0` | 157 | 142 | 0 | 0 | HDMI TX |
| `EEPROM_8b_0` | 54 | 47 | 1 | 0 | I2C + glitch filter + sync |

The 4,254 LUTRAMs are the `cur_row` distributed RAM arrays in the line buffers — the third row that must support combinational read for the x+1 window tap. The 164 RAMB18 tiles in `dncnn_top` are the two BRAM rows of the line buffers in workhorse and finisher (64 RAMB18 each = 32 RAMB36 equivalent each).

---

## 19. Timing

### Status: NOT MET

| Metric | Value |
|---|---|
| WNS (Worst Negative Slack) | **−8.967 ns** |
| Failing setup endpoints | 4,563 |
| WHS (Worst Hold Slack) | +0.050 ns ✅ |
| Failing hold endpoints | 0 ✅ |

Hold timing passes. Setup timing does not.

### Clock Summary

| Clock | Period | Frequency |
|---|---|---|
| `clk_fpga_0` (core/AXI) | 10.000 ns | 100 MHz |
| `clk_fpga_1` | 5.000 ns | 200 MHz |
| `clk_pixel_in` (HDMI RX) | 13.468 ns | 74.25 MHz |

### Critical Path

The worst failing path is inside `dncnn_workhorse → mac_array_16x16 → dsp_inst.CLK` to `m_axis_tdata_reg`. Data path delay: **15.812 ns** against a 10 ns budget. The path has 26 logic levels — 17 CARRY4 primitives plus LUTs — forming the accumulator adder tree that sums partial products across output channels.

Two cross-domain paths (`clk_fpga_0 → clk_pixel_in`) also fail with WNS −3.65 ns. These are the async FIFO pointer synchronizers — the `set_false_path` XDC constraints may not be correctly scoped.

### How to Fix

**Option A — Pipeline the adder tree (correct fix):** Break the 17-stage CARRY4 chain in `mac_array_16x16.v` across 2–3 registered pipeline stages. This adds 2–3 cycles of latency per pixel but the throughput (one result per clock, steady-state) is unchanged. The 26-level logic becomes two 13-level stages, each well under 10 ns.

**Option B — Reduce core clock (quick fix):** Change `clk_fpga_0` from 100 MHz to 75 MHz in the PS clock configuration. This widens the timing budget to 13.3 ns, which the current 15.8 ns path still doesn't meet, but reduces the WNS magnitude. At 65 MHz (15.4 ns budget) it would likely close. Trade-off: slower layer processing time.

### Methodology Warnings

| ID | Severity | Description |
|---|---|---|
| TIMING-6/7/8 | Critical Warning | No common clock between related clock pairs (pixel ↔ core) |
| TIMING-9 | Warning | Unknown CDC logic — Vivado cannot verify synchronizer structure |
| TIMING-16 | Warning | Large setup violations (1,000 instances) |
| TIMING-18 | Warning | Missing input/output delay constraints (3 ports) |

TIMING-9 is the most meaningful: Vivado cannot prove the async FIFO CDC paths are safe because the custom FIFO structure isn't recognized as a synchronizer. Using `XPM_CDC` primitives from the Xilinx library would replace the custom async FIFOs with a known-safe structure that Vivado understands, resolving this warning and TIMING-6/7/8.

---

## 20. Power

| | |
|---|---|
| Total On-Chip Power | **1.899 W** |
| Dynamic | 1.736 W |
| Device Static | 0.163 W |
| Junction Temperature | 46.9 °C |
| Max Ambient | 63.1 °C |

Confidence level is **Low** — no switching activity file was provided. Numbers are estimated from default activity rates. Actual dynamic power under real video traffic (640×480 @ 30 FPS input, active MAC arrays) will differ. The PYNQ-Z2 board's power supply and thermal solution are rated for this range.

---

## 21. Replication Steps

### Prerequisites
- **Vivado 2023.1** — the BD TCL targets this version and will error on others (the error message explains the upgrade path)
- **PYNQ-Z2 board files** installed — [Digilent guide](https://digilent.com/reference/programmable-logic/guides/installing-vivado-and-vitis)

### 1. Clone

```bash
git clone --recurse-submodules https://github.com/<your-username>/<your-repo>.git
cd <your-repo>
```

> Forgot `--recurse-submodules`? Run: `git submodule update --init --recursive`

### 2. Create a new Vivado project

Vivado 2023.1 → **Create Project** → RTL Project → select **PYNQ-Z2**.

On the *Add Sources* screen, add **all files from `src/design/`** — both `.v` and all four `.vhd` files. The EEPROM will fail elaboration if any VHDL dependency (`TWI_SlaveCtl`, `GlitchFilter`, `SyncAsync`) is missing.

### 3. Add IP repo paths

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

**Note:** Timing is currently not met (WNS −8.967 ns). The bitstream will generate but the design may not function correctly at 100 MHz. See Section 19 for fixes before taping out.

---

## Reference

Zhang, K., Zuo, W., Chen, Y., Meng, D., & Zhang, L. (2017). *Beyond a Gaussian Denoiser: Residual Learning of Deep CNN for Image Denoising.* IEEE Transactions on Image Processing, 26(7), 3142–3155.

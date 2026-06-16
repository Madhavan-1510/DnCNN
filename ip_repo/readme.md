# ip_repo — Setup Instructions

This folder holds the IP cores needed to build the block design. You need to set up **3 things**:

---

## 1. `viip/` — Custom DnCNN Accelerator IP

This folder is already in the repo. It contains the packaged `dncnn_top_v1_1` IP core.

```
ip_repo/
└── viip/        ← already here, nothing to do
```

No extra steps needed.

---

## 2. Digilent IPs — `dvi2rgb` and `rgb2dvi`

Download the Digilent Vivado IP library from GitHub and place it inside this folder:

```bash
cd ip_repo/
git clone https://github.com/Digilent/vivado-library.git digilent-vivado-ip
```

Your folder should look like:

```
ip_repo/
├── viip/
└── digilent-vivado-ip/
    └── ip/
        ├── dvi2rgb/
        └── rgb2dvi/
```

---

## 3. RTL Sources — `tlast_gen` and `EEPROM_8b`

These are **not packaged IPs** — they are RTL source files that get added directly to your Vivado project.
All 5 files live in the `src/` folder at the root of this repo:

```
src/
├── tlast_gen.v           # AXI-Stream TLAST generator — standalone
│
├── EEPROM_8b.vhd         # EEPROM top-level module  ┐
├── TWI_SlaveCtl.vhd      # I2C slave controller     │ these 4 must all
├── GlitchFilter.vhd      # glitch filter for SCL/SDA│ be added together
└── SyncAsync.vhd         # async to sync bridge     ┘
```

> ⚠️ The EEPROM will not work if any of the 4 VHDL files are missing.
> `EEPROM_8b.vhd` is the top level — the other three are submodules it instantiates internally.

### Adding them to your Vivado project

In Vivado, before sourcing the BD TCL:

1. Go to **Flow Navigator → Add Sources → Add or Create Design Sources**
2. Click **Add Files** and select all 5 files:
   - `tlast_gen.v`
   - `EEPROM_8b.vhd`
   - `TWI_SlaveCtl.vhd`
   - `GlitchFilter.vhd`
   - `SyncAsync.vhd`
3. Make sure **Copy sources into project** is checked
4. Click **Finish**

---

## Final folder structure check

Before sourcing the BD TCL, confirm it looks like this:

```
ip_repo/
├── viip/                          ✅ custom dncnn IP
└── digilent-vivado-ip/            ✅ cloned from Digilent GitHub
src/
├── tlast_gen.v                    ✅ tlast_gen source
├── EEPROM_8b.vhd                  ✅ EEPROM top level
├── TWI_SlaveCtl.vhd               ✅ EEPROM dependency
├── GlitchFilter.vhd               ✅ EEPROM dependency
└── SyncAsync.vhd                  ✅ EEPROM dependency
bd/
└── design_1_bd.tcl
```

Then in the Vivado Tcl Console, set the IP repo paths:

```tcl
set_property ip_repo_paths {
    ./ip_repo/viip
    ./ip_repo/digilent-vivado-ip/ip
} [current_project]

update_ip_catalog
```

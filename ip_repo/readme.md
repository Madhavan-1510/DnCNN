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

These are not IPs — they are plain Verilog source files.  
Find them in the project under:

```
<your_project>/design_1/src/
```

Copy them into the `src/` folder at the root of this repo:

```
src/
├── tlast_gen.v
└── EEPROM_8b.v
```

They need to be **added as sources** in your Vivado project before sourcing the block design TCL — not added to `ip_repo`.

---

## Final folder structure check

Before sourcing the BD TCL, confirm it looks like this:

```
ip_repo/
├── viip/                          ✅ custom dncnn IP
└── digilent-vivado-ip/            ✅ cloned from Digilent GitHub
src/
├── tlast_gen.v                    ✅ copied from your project src/
└── EEPROM_8b.v                    ✅ copied from your project src/
bd/
└── design_1_bd.tcl
```

Then in Vivado Tcl Console, set the IP repo paths:

```tcl
set_property ip_repo_paths {
    ./ip_repo/viip
    ./ip_repo/digilent-vivado-ip/ip
} [current_project]

update_ip_catalog
```

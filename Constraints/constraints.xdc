## =============================================================================
## dncnn_constraints.xdc - Timing and IO Constraints for DnCNN Accelerator
## Target: XC7Z020-1CLG400C (PYNQ-Z2)
## =============================================================================

## =============================================================================
## IO Pin Assignments
## =============================================================================

## ---------------------------------------------------------
## HDMI Rx (Input from Camera -> mapped to TMDS_0)
## ---------------------------------------------------------
set_property -dict { PACKAGE_PIN N18   IOSTANDARD TMDS_33  } [get_ports { TMDS_0_clk_p }]
set_property -dict { PACKAGE_PIN P19   IOSTANDARD TMDS_33  } [get_ports { TMDS_0_clk_n }]
set_property -dict { PACKAGE_PIN V20   IOSTANDARD TMDS_33  } [get_ports { TMDS_0_data_p[0] }]
set_property -dict { PACKAGE_PIN W20   IOSTANDARD TMDS_33  } [get_ports { TMDS_0_data_n[0] }]
set_property -dict { PACKAGE_PIN T20   IOSTANDARD TMDS_33  } [get_ports { TMDS_0_data_p[1] }]
set_property -dict { PACKAGE_PIN U20   IOSTANDARD TMDS_33  } [get_ports { TMDS_0_data_n[1] }]
set_property -dict { PACKAGE_PIN N20   IOSTANDARD TMDS_33  } [get_ports { TMDS_0_data_p[2] }]
set_property -dict { PACKAGE_PIN P20   IOSTANDARD TMDS_33  } [get_ports { TMDS_0_data_n[2] }]

## ---------------------------------------------------------
## HDMI Rx I2C (EDID Spoofing)
## ---------------------------------------------------------
set_property -dict { PACKAGE_PIN U14   IOSTANDARD LVCMOS33 } [get_ports { SCL }]
set_property -dict { PACKAGE_PIN U15   IOSTANDARD LVCMOS33 } [get_ports { SDA }]

## ---------------------------------------------------------
## HDMI Tx (Output to Monitor -> mapped to TMDS_1)
## ---------------------------------------------------------
set_property -dict { PACKAGE_PIN L16   IOSTANDARD TMDS_33  } [get_ports { TMDS_1_clk_p }]
set_property -dict { PACKAGE_PIN L17   IOSTANDARD TMDS_33  } [get_ports { TMDS_1_clk_n }]
set_property -dict { PACKAGE_PIN K17   IOSTANDARD TMDS_33  } [get_ports { TMDS_1_data_p[0] }]
set_property -dict { PACKAGE_PIN K18   IOSTANDARD TMDS_33  } [get_ports { TMDS_1_data_n[0] }]
set_property -dict { PACKAGE_PIN K19   IOSTANDARD TMDS_33  } [get_ports { TMDS_1_data_p[1] }]
set_property -dict { PACKAGE_PIN J19   IOSTANDARD TMDS_33  } [get_ports { TMDS_1_data_n[1] }]
set_property -dict { PACKAGE_PIN J18   IOSTANDARD TMDS_33  } [get_ports { TMDS_1_data_p[2] }]
set_property -dict { PACKAGE_PIN H18   IOSTANDARD TMDS_33  } [get_ports { TMDS_1_data_n[2] }]

## =============================================================================
## Clock Constraints
## =============================================================================

## ---------------------------------------------------------
## HDMI RX pixel clock - FIX: this was MISSING, causing 3659 no_clock warnings
## and 225 TIMING-17 violations in u_fifo_in / rst_pixin_sync registers.
##
## 74.25 MHz = 13.468 ns period (720p / 480p HDMI pixel clock)
## The dvi2rgb IP derives clk_pixel from the TMDS clock via MMCM.
## This create_clock on TMDS_0_clk_p tells the timing engine the incoming
## frequency so it can constrain the MMCM output correctly.
## ---------------------------------------------------------
create_clock -period 13.468 -name clk_pixel_in [get_ports TMDS_0_clk_p]

## ---------------------------------------------------------
## Clock Domain Crossing constraints
##
## Three asynchronous domains:
##   clk_fpga_0   (100 MHz) - PS FCLK0, DnCNN core
##   clk_pixel_in (74.25 MHz) - HDMI RX pixel clock (TMDS input)
##   clk_pixel_out (74.25 MHz) - HDMI TX pixel clock (MMCM output to rgb2dvi)
##
## set_clock_groups -asynchronous suppresses false timing paths across the
## async FIFO boundaries where gray-code synchronisers handle the CDC.
## This is the correct annotation - do NOT use set_false_path on the
## individual FF paths; that would hide real CDC violations.
## ---------------------------------------------------------

## Core <-> HDMI RX pixel clock
set_clock_groups -asynchronous \
    -group [get_clocks clk_fpga_0] \
    -group [get_clocks clk_pixel_in]

## Core <-> HDMI TX pixel clock
## (clk_pixel_out is a generated clock from the dvi2rgb/rgb2dvi MMCM;
##  if Vivado names it differently, update the clock name to match
##  'report_clocks' output after synthesis.)
set_clock_groups -asynchronous \
    -group [get_clocks clk_fpga_0] \
    -group [get_clocks -of_objects [get_pins design_1_i/dvi2rgb_0/U0/TMDS_ClockingX/DVI_ClkGenerator/CLKOUT0]]

## =============================================================================
## Max-delay on Gray-code synchroniser paths (inside async FIFOs)
##
## The two-FF gray-code synchronisers in async_fifo_ingest and
## async_fifo_eject cross the clock boundary one bit at a time.
## The set_max_delay -datapath_only overrides Vivado's default hold
## analysis on these paths and lets the tools optimise placement of the
## synchroniser FFs for minimum metastability window.
## =============================================================================
set_max_delay -datapath_only 5.0 \
    -from [get_cells -hierarchical -filter {NAME =~ *u_fifo_in*wr_ptr_gray*}] \
    -to   [get_cells -hierarchical -filter {NAME =~ *u_fifo_in*wr_ptr_gray_s1*}]

set_max_delay -datapath_only 5.0 \
    -from [get_cells -hierarchical -filter {NAME =~ *u_fifo_in*rd_ptr_gray*}] \
    -to   [get_cells -hierarchical -filter {NAME =~ *u_fifo_in*rd_ptr_gray_s1*}]

set_max_delay -datapath_only 5.0 \
    -from [get_cells -hierarchical -filter {NAME =~ *u_fifo_eject*wr_ptr_gray*}] \
    -to   [get_cells -hierarchical -filter {NAME =~ *u_fifo_eject*wr_ptr_gray_s1*}]

set_max_delay -datapath_only 5.0 \
    -from [get_cells -hierarchical -filter {NAME =~ *u_fifo_eject*rd_ptr_gray*}] \
    -to   [get_cells -hierarchical -filter {NAME =~ *u_fifo_eject*rd_ptr_gray_s1*}]


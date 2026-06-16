# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  ipgui::add_param $IPINST -name "Component_Name"
  #Adding Page
  set Page_0 [ipgui::add_page $IPINST -name "Page 0"]
  ipgui::add_param $IPINST -name "HEIGHT" -parent ${Page_0}
  ipgui::add_param $IPINST -name "IC" -parent ${Page_0}
  ipgui::add_param $IPINST -name "OC" -parent ${Page_0}
  ipgui::add_param $IPINST -name "WIDTH" -parent ${Page_0}


}

proc update_PARAM_VALUE.HEIGHT { PARAM_VALUE.HEIGHT } {
	# Procedure called to update HEIGHT when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.HEIGHT { PARAM_VALUE.HEIGHT } {
	# Procedure called to validate HEIGHT
	return true
}

proc update_PARAM_VALUE.IC { PARAM_VALUE.IC } {
	# Procedure called to update IC when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.IC { PARAM_VALUE.IC } {
	# Procedure called to validate IC
	return true
}

proc update_PARAM_VALUE.OC { PARAM_VALUE.OC } {
	# Procedure called to update OC when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.OC { PARAM_VALUE.OC } {
	# Procedure called to validate OC
	return true
}

proc update_PARAM_VALUE.WIDTH { PARAM_VALUE.WIDTH } {
	# Procedure called to update WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.WIDTH { PARAM_VALUE.WIDTH } {
	# Procedure called to validate WIDTH
	return true
}


proc update_MODELPARAM_VALUE.WIDTH { MODELPARAM_VALUE.WIDTH PARAM_VALUE.WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.WIDTH}] ${MODELPARAM_VALUE.WIDTH}
}

proc update_MODELPARAM_VALUE.HEIGHT { MODELPARAM_VALUE.HEIGHT PARAM_VALUE.HEIGHT } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.HEIGHT}] ${MODELPARAM_VALUE.HEIGHT}
}

proc update_MODELPARAM_VALUE.IC { MODELPARAM_VALUE.IC PARAM_VALUE.IC } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.IC}] ${MODELPARAM_VALUE.IC}
}

proc update_MODELPARAM_VALUE.OC { MODELPARAM_VALUE.OC PARAM_VALUE.OC } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.OC}] ${MODELPARAM_VALUE.OC}
}


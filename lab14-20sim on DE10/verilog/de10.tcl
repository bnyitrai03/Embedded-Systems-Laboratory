##-----------------------------------------------------------------------------
## 
## ESL demo
## Version: 1.0
## Creator: Rene Moll
## Date: 10th April 2012
##
##-----------------------------------------------------------------------------
##
## This file is seperated in the following sections:
## - IP details
## - Required diles
## - IP parameters
## - Interface specifications
## - Validation/ elaboration functions
##
##-----------------------------------------------------------------------------

## 
## IP details
##  
set_module_property DESCRIPTION "ESL Controller (Avalon-MM slave port)"
set_module_property NAME ESL_Controller
set_module_property VERSION 1.0
set_module_property GROUP Templates
set_module_property AUTHOR Moll
set_module_property DISPLAY_NAME ESL_Controller
set_module_property TOP_LEVEL_HDL_FILE DE10_NANO_TOP.v
set_module_property TOP_LEVEL_HDL_MODULE DE10_NANO_TOP
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property EDITABLE false
set_module_property SIMULATION_MODEL_IN_VERILOG false
set_module_property SIMULATION_MODEL_IN_VHDL false
set_module_property SIMULATION_MODEL_HAS_TULIPS false
set_module_property SIMULATION_MODEL_IS_OBFUSCATED false

## 
## Link to the verification methods
## Defined at the botom of this file
##  

set_module_property ELABORATION_CALLBACK elaborate_me
set_module_property VALIDATION_CALLBACK validate_me


## 
## Files
## - List all files required by the IP
##  
add_file DE10_NANO_TOP.v {SYNTHESIS SIMULATION}
add_file Middleware.v {SYNTHESIS SIMULATION}
add_file PWM.v {SYNTHESIS SIMULATION}
add_file QuadDecoder.v {SYNTHESIS SIMULATION}
add

## 
## IP parameters
## - Generics defined in the VHDL can be modified from
##   a wizard in SOPC builder. This section defines how 
##   these parameters are presented to the user.
## - The actual link between the parametes and the generics
##   is made in the ¨elaborate_me¨ function.
##
add_parameter DATA_WIDTH int 32 "Data width for avalon interface"
set_parameter_property DATA_WIDTH DISPLAY_NAME "Word Size"
set_parameter_property DATA_WIDTH GROUP "Register File Properties"
set_parameter_property DATA_WIDTH AFFECTS_PORT_WIDTHS true
set_parameter_property DATA_WIDTH ALLOWED_RANGES {8 16 32}
set_parameter_property DATA_WIDTH HDL_PARAMETER true

## 
## Interface
## - Add clock and reset signals
##  
add_interface clock_reset clock end
set_interface_property clock_reset ptfSchematicName ""

add_interface_port clock_reset clk   clk   Input 1
add_interface_port clock_reset reset reset Input 1

##
## - Add the avalon interface
##   The properties descriptions may be found in the 
##   avalon interface specifications.
##
add_interface s0 avalon end
set_interface_property s0 holdTime 0
set_interface_property s0 linewrapBursts false
set_interface_property s0 minimumUninterruptedRunLength 1
set_interface_property s0 bridgesToMaster ""
set_interface_property s0 isMemoryDevice false
set_interface_property s0 burstOnBurstBoundariesOnly false
set_interface_property s0 addressSpan 8
set_interface_property s0 timingUnits Cycles
set_interface_property s0 setupTime 0
set_interface_property s0 writeWaitTime 0
set_interface_property s0 isNonVolatileStorage false
set_interface_property s0 addressAlignment DYNAMIC
set_interface_property s0 maximumPendingReadTransactions 0
set_interface_property s0 readWaitTime 0
set_interface_property s0 readLatency 3
set_interface_property s0 printableDevice false
set_interface_property s0 ASSOCIATED_CLOCK clock_reset

add_interface_port s0 slave_address address Input 8
add_interface_port s0 slave_read read Input 1
add_interface_port s0 slave_write write Input 1
add_interface_port s0 slave_readdata readdata Output -1
add_interface_port s0 slave_writedata writedata Input -1

##
## - Add the user interface
##   Syntax:
##   add_interface_port user_interface (YOUR_PORT) export (DIRECTION) (WIDTH)
##

add_interface pitch_enc_a conduit end
add_interface_port pitch_enc_a PITCH_ENC_A export Input 1
add_interface pitch_enc_b conduit end
add_interface_port pitch_enc_b PITCH_ENC_B export Input 1

add_interface pitch_pwm_val conduit end
add_interface_port pitch_pwm_val PITCH_PWM_VAL export Output 1
add_interface pitch_pwm_dir_a conduit end
add_interface_port pitch_pwm_dir_a PITCH_DIRA export Output 1
add_interface pitch_pwm_dir_b conduit end
add_interface_port pitch_pwm_dir_b PITCH_DIRB export Output 1

add_interface yaw_enc_a conduit end
add_interface_port yaw_enc_a YAW_ENC_A export Input 1
add_interface yaw_enc_b conduit end
add_interface_port yaw_enc_b YAW_ENC_B export Input 1

add_interface yaw_pwm_val conduit end
add_interface_port yaw_pwm_val YAW_PWM_VAL export Output 1
add_interface yaw_pwm_dir_a conduit end
add_interface_port yaw_pwm_dir_a YAW_DIRA export Output 1
add_interface yaw_pwm_dir_b conduit end
add_interface_port yaw_pwm_dir_b YAW_DIRB export Output 1

##
## - Validation/ elaboration functions
##
proc validate_me {} {
}

proc elaborate_me {}  {
  ## Retrieve the parameters from the wizard
  set the_data_width [get_parameter_value DATA_WIDTH]
  
  ## Set data width for the avalon interface
  set_port_property slave_readdata  WIDTH $the_data_width
  set_port_property slave_writedata WIDTH $the_data_width

  ## Set data with for the custom logic
  set_port_property PITCH_ENC_A WIDTH 1
  set_port_property PITCH_ENC_B WIDTH 1
  set_port_property PITCH_PWM_VAL WIDTH 1
  set_port_property PITCH_DIRA WIDTH 1
  set_port_property PITCH_DIRB WIDTH 1
  
  set_port_property YAW_ENC_A WIDTH 1
  set_port_property YAW_ENC_B WIDTH 1
  set_port_property YAW_PWM_VAL WIDTH 1
  set_port_property YAW_DIRA WIDTH 1
  set_port_property YAW_DIRB WIDTH 1
  
  ## DO NOT REMOVE:
  ## adding the slave_byteenable and user_byteenable signals only if the data width is greater than 8 bits
  if { $the_data_width != 8 } {
    add_interface_port s0 slave_byteenable byteenable Input [expr {$the_data_width / 8} ]
  }
}
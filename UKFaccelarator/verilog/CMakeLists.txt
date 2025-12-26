## NOTES TO MYSELF ##
## study cmake custom commands and targets
## understand add_custom_command and add_custom_target

cmake_minimum_required(VERSION 3.10)
project(ukf_verilog_sim NONE)

find_program(IVERILOG_EXECUTABLE iverilog)
find_program(VVP_EXECUTABLE vvp)

if(NOT IVERILOG_EXECUTABLE)
  message(FATAL_ERROR "iverilog not found")
endif()
if(NOT VVP_EXECUTABLE)
  message(FATAL_ERROR "vvp not found")
endif()

set(VERILOG_SOURCES
  ${CMAKE_SOURCE_DIR}/ukf.v
)

set(TESTBENCH
  ${CMAKE_SOURCE_DIR}/tb_ukf.v
)

set(SIM_OUT ${CMAKE_BINARY_DIR}/ukf_sim.out)
set(VCD_FILE ${CMAKE_BINARY_DIR}/ukf.vcd)

## run iverilog and vvp to generate the VCD file
add_custom_command(
  OUTPUT ${VCD_FILE}
  COMMAND ${IVERILOG_EXECUTABLE} -o ${SIM_OUT} ${VERILOG_SOURCES} ${TESTBENCH}
  COMMAND ${VVP_EXECUTABLE} ${SIM_OUT}
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS ${VERILOG_SOURCES} ${TESTBENCH}
  COMMENT "Compiling and running iverilog simulation"
)

## create a target to run the simulation
add_custom_target(run_sim ALL DEPENDS ${VCD_FILE})

## open waveform in gtkwave
add_custom_target(open_wave
  COMMAND ${CMAKE_COMMAND} -E echo "Opening waveform..."
  COMMAND gtkwave ${VCD_FILE}
  DEPENDS run_sim
)
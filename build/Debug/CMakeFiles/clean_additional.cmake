# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "stm32f103-micro-ros.bin"
  "stm32f103-micro-ros.elf"
  "stm32f103-micro-ros.hex"
  "stm32f103-micro-ros.map"
  )
endif()

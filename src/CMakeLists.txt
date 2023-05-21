add_executable(${CMAKE_PROJECT_NAME} main.c SEGGER_SYSVIEW_Config_NoOS_CM0.c)

pico_enable_stdio_usb(${CMAKE_PROJECT_NAME} 1)
pico_enable_stdio_uart(${CMAKE_PROJECT_NAME} 0)
pico_add_extra_outputs(${CMAKE_PROJECT_NAME})


# pull in common dependencies
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -O0 -g3)
target_link_libraries(
  ${CMAKE_PROJECT_NAME} PRIVATE pico_stdlib hardware_timer hardware_clocks
                                hardware_pwm segger_systemview hardware_pio)

target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# create map/bin/hex file etc.
pico_add_extra_outputs(${CMAKE_PROJECT_NAME})

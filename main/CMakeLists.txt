add_executable(main
        hc06.c
        main.c
)

target_link_libraries(main pico_stdlib freertos hardware_adc hardware_i2c Fusion)
pico_add_extra_outputs(main)

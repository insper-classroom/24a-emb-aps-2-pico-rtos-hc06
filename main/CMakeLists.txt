add_executable(main
        hc06.c
        main.c
)

target_link_libraries(main pico_stdlib hardware_adc freertos)
pico_add_extra_outputs(main)

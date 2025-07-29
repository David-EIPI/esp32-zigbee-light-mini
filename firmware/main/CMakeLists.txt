idf_component_register(
    SRCS
    "main.c"
    "clock.c"
    "light_driver.c"
    "ld2420_comm.c"
    "triac_dimmer.c"
    INCLUDE_DIRS "."
)
set_target_properties(${COMPONENT_LIB} PROPERTIES COMPILE_FLAGS "-save-temps")

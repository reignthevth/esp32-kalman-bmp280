idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
)

# Set SIMULATION_MODE=1 env var at build time to skip I2C hardware entirely.
# Used by GitHub Actions / Wokwi CI so the firmware runs without a real BMP280.
if(DEFINED ENV{SIMULATION_MODE})
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SIMULATION_MODE=1)
    message(STATUS "Building in SIMULATION_MODE (no hardware I2C)")
endif()

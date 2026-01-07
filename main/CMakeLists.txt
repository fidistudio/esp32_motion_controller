idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES lut_correction motor_pwm pid driver
)

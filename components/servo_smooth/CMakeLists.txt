set( servo_smooth_srcs "src/servo_smooth.c")

idf_component_register(SRCS ${servo_smooth_srcs}
                    INCLUDE_DIRS "include"
                    REQUIRES   nvs_flash bt)
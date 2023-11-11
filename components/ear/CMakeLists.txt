set( ear_srcs "src/ear.cpp")

idf_component_register(SRCS ${ear_srcs}
                    INCLUDE_DIRS "include"
                    REQUIRES   nvs_flash bt servo_smooth mpu6050with_carman)
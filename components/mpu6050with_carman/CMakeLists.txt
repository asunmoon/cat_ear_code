
set( inv_mpu_srcs1 "src/i2c.cpp")
set( MPU6050_srcs1 "src/mpu6050.cpp")
set( inv_mpu_dmp_motion_driver_srcs1 "src/kalmanfilter.cpp")
idf_component_register(SRCS ${MPU6050_srcs1} ${inv_mpu_srcs1} ${inv_mpu_dmp_motion_driver_srcs1}
                    INCLUDE_DIRS "include"
                    REQUIRES   )
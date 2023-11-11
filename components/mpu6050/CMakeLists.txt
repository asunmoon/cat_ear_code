
set( inv_mpu_srcs "src/inv_mpu.c")
set( MPU6050_srcs "src/MPU6050.c")
set( inv_mpu_dmp_motion_driver_srcs "src/inv_mpu_dmp_motion_driver.c")
idf_component_register(SRCS ${MPU6050_srcs} ${inv_mpu_srcs} ${inv_mpu_dmp_motion_driver_srcs}
                    INCLUDE_DIRS "include"
                    REQUIRES   )
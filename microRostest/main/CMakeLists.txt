idf_component_register(SRCS "sensor_temp.c"
                    INCLUDE_DIRS "."
                    REQUIRES micro_ros_espidf_component ds18b20 onewire_bus)
idf_component_register(SRCS "sensor_temp.c" "esp32_serial_transport.c"
                    INCLUDE_DIRS "."
                    REQUIRES micro_ros_espidf_component ds18b20 onewire_bus esp_driver_uart)

# Definir macro para indicar transporte custom (serial)
target_compile_definitions(${COMPONENT_LIB} PUBLIC RMW_UXRCE_TRANSPORT_CUSTOM)
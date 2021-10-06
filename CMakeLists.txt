idf_component_register(SRCS "src/mbus-protocol-aux.c" "src/mbus-serial-485.c" "src/mbus-protocol.c"
                    INCLUDE_DIRS "inc"
                    #PRIV_REQUIRES esp_i2c
)

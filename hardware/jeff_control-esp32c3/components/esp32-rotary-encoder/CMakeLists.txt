idf_component_register(
    SRCS "rotary_encoder.c"
    INCLUDE_DIRS include
    REQUIRES esp_driver_gpio driver esp_timer driver
)
#* Old 
# #idf_component_register(SRCS "rotary_encoder.c" INCLUDE_DIRS include)

# set(COMPONENT_ADD_INCLUDEDIRS include)
# set(COMPONENT_SRCS "rotary_encoder.c")
# register_component()

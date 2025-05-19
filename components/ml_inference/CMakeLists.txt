# Register ML Inference component
idf_component_register(
    SRCS 
        "ml_inference.c"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        "tensorflow-lite-esp32" 
        "esp_timer"
        "nvs_flash"
)
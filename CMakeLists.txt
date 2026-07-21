if("${IDF_TARGET}" STREQUAL "esp32p4")
if (USE_GENERIC_ESP_HOSTED)
	set(TM_LORA_EXTRA_REQUIREMENTS "esp_hosted")
else()
    set(TM_LORA_EXTRA_REQUIREMENTS "esp-hosted-tanmatsu")
endif()
endif()

idf_component_register(
	SRCS
		lora.c
	INCLUDE_DIRS
		"include"
	REQUIRES
		sx126x
		${TM_LORA_EXTRA_REQUIREMENTS}
)

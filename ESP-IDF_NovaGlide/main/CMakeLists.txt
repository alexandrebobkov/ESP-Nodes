idf_component_register(SRCS "ESP-IDF_NovaGlide.c" "system_init.c" "scheduler.c"
                    INCLUDE_DIRS "."
		    REQUIRES sensors connectivity)

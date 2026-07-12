idf_component_register(SRCS "drone.c" "input_task.c" "main.c" "pid_task.c"
                    INCLUDE_DIRS ".")

if(DEFINED ESC_PROGRAMMING_MODE)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE ESC_PROGRAMMING_MODE)
endif()

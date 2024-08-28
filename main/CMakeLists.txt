idf_component_register(SRCS "main.c" 
                            "interface/bt_interface.c" 
                            "implementation/spp_task.c"
                            "interface/weigh_interface.c"
                            "implementation/ad1115_task.c"
                    INCLUDE_DIRS "interface" "implementation")

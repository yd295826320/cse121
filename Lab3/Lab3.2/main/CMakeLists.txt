set(SOURCE "main.cpp" "DFRobot_LCD.cpp")

idf_component_register(SRCS ${SOURCE}
                    INCLUDE_DIRS ".")

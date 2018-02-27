rock_library(motors_elmo_ds402
    SOURCES Objects.cpp Controller.cpp
    HEADERS Objects.hpp Controller.hpp
    DEPS_PKGCONFIG canbus canopen_master)

rock_executable(motors_elmo_ds402_bin Main.cpp
    DEPS motors_elmo_ds402)


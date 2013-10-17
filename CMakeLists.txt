project(Robocpp_cmake)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)


file(GLOB SRC
    "*.h"
    "*.cpp"
)

# set the path to the library folder
#link_directories(/usr/local/lib)

# link the libraries to the executable
#target_link_libraries (my_node foo)

SET(CMAKE_CXX_FLAGS "-lrt -lach -larmadillo")
SET(CMAKE_SHARED_LINKER_FLAGS "-lrt -lach -larmadillo")
SET(CMAKE_EXE_LINKER_FLAGS "-lrt -lach -larmadillo")

add_executable(${PROJECT_NAME} ${SRC_LIST})


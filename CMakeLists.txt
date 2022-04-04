cmake_minimum_required(VERSION 3.5)

FILE(GLOB_RECURSE app_sources "src/*.*")

set(depends
    "LinkedList"        # https://github.com/vortigont/LinkedList
)

# this should be placed in Projetc's CMakeLists.txt file to inclue TimeSeries features
#set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build pzem-edl as an ESP-IDF component
if(ESP_PLATFORM)
    idf_component_register(
        SRCS ${app_sources}
        INCLUDE_DIRS "src"
        REQUIRES ${depends}
        #PRIV_REQUIRES
    )
#return()
endif()

project(PZEM_EDL VERSION 1.0.1)

# https://cmake.org/cmake/help/latest/prop_gbl/CMAKE_CXX_KNOWN_FEATURES.html
#set_target_properties(${COMPONENT_TARGET} PROPERTIES
#    CXX_STANDARD 14
#    CXX_STANDARD_REQUIRED YES
#    CXX_EXTENSIONS NO
#)
#add_subdirectory(src)
#target_compile_options(${COMPONENT_TARGET} PRIVATE -fno-rtti)


# External congestion-control module: identity (CC_EXT_MODULES entry EXT_IDENTITY, IP cc_ext_identity)
cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME cc_ext_identity)
project(${PROJECT_NAME})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake ${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../cmake)

find_package(VitisHLS REQUIRED)
if (NOT VITIS_HLS_FOUND)
   message(FATAL_ERROR "Vitis HLS not found.")
endif()

if(DEFINED ENV{IPREPO_DIR})
   set(IPREPO_DIR $ENV{IPREPO_DIR})
elseif(NOT IPREPO_DIR)
   set(IPREPO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/iprepo/)
endif()

# Same configuration header as the stack, so ccEvent / ccExtRateUpd and the CC constants match exactly
# (the cache variables are set by hls/rocev2/CMakeLists.txt, which is processed first)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../../rocev2/rocev2_config.hpp.in rocev2_config.hpp)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/make.tcl.in make.tcl)

set(HLS_DEPENDS
   ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/../../ib_transport_protocol/cc/cc_types.hpp
)

if (NOT TARGET services)
   add_custom_target(services)
endif()

if(VITIS_HLS_MODE STREQUAL "vitis_hls")
   add_custom_target(services.${PROJECT_NAME} COMMAND ${VITIS_HLS_BINARY} -f make.tcl DEPENDS ${HLS_DEPENDS})
else()
   add_custom_target(services.${PROJECT_NAME} COMMAND ${VITIS_HLS_BINARY} --tcl make.tcl --mode hls DEPENDS ${HLS_DEPENDS})
endif()

add_dependencies(services services.${PROJECT_NAME})

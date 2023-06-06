cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)

project(XsensPlugin LANGUAGES CXX VERSION 1.0.0)

include(CTest)
enable_testing()

# Find mc_rtc if we are not building inside mc_rtc itself
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

find_package(xsens_streaming REQUIRED)

#set(MC_PLUGINS_RUNTIME_INSTALL_PREFIX 
#  /usr/local/lib/mc_plugins)

#message(STATUS "MC_PLUGINS_RUNTIME_INSTALL_PREFIX: ${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}")
#message(STATUS "MTSDK_LIBRARY_DIR: ${MTSDK_LIBRARY_DIR}")


# Configure and install the plugin configuration
configure_file(etc/XsensPlugin.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/XsensPlugin.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/XsensPlugin.yaml" DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")

set(plugin_SRC
  src/XsensPlugin.cpp
  )

set(plugin_HDR
  src/XsensPlugin.h
)

# The add_plugin macro is provided by mc_rtc
add_plugin(XsensPlugin "${plugin_SRC}" "${plugin_HDR}")
target_link_libraries(XsensPlugin PUBLIC xsens_streaming::xsens_streaming)
set_target_properties(XsensPlugin PROPERTIES INSTALL_RPATH "${MTSDK_LIBRARY_DIR}")
#install(FILES "${CMAKE_CURRENT_BINARY_DIR}/XsensPlugin.so" DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}")


if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

cmake_minimum_required(VERSION 3.25)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PROJECT_NAME XsensPlugin)
set(PROJECT_DESCRIPTION "Xsens Plugin")
set(PROJECT_URL https://github.com/arntanguy/mc_xsens_plugin)
set(PROJECT_VERSION 1.0)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(INSTALL_DOCUMENTATION OFF CACHE BOOL "")
set(INSTALL_GENERATED_HEADERS OFF CACHE BOOL "" FORCE)
set(INSTALL_PKG_CONFIG_FILE OFF CACHE BOOL "" FORCE)
set(CXX_DISABLE_WERROR ON)

include(cmake/base.cmake)

project(XsensPlugin LANGUAGES CXX VERSION 1.0.0)

include(CTest)
enable_testing()

# Find mc_rtc if we are not building inside mc_rtc itself
if(NOT TARGET mc_rtc::mc_control)
  add_project_dependency(mc_rtc REQUIRED)
endif()

# Optional dependency. When present build live support for xsens data
add_project_dependency(xsens_streaming)

# Configure and install the plugin configuration
configure_file(etc/XsensPlugin.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/XsensPlugin.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/XsensPlugin.yaml" DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")

set(common_SRC
src/XsensDataInputDatastore.cpp
src/XsensPluginData.cpp
)

set(common_HDR
include/mc_xsens_plugin/XsensBodyConfiguration.h
include/mc_xsens_plugin/XsensBodyMappings.h
include/mc_xsens_plugin/XsensDataInputDatastore.h
include/mc_xsens_plugin/XsensDataInput.h
include/mc_xsens_plugin/XsensDataInputLive.h
include/mc_xsens_plugin/XsensPluginData.h
)
add_library(XsensPluginCommon SHARED "${common_SRC}" "${common_HDR}")
target_include_directories(XsensPluginCommon PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(XsensPluginCommon PUBLIC SpaceVecAlg::SpaceVecAlg)
target_link_libraries(XsensPluginCommon PUBLIC mc_rtc::mc_rtc_utils)

install(TARGETS XsensPluginCommon
 EXPORT "${TARGETS_EXPORT_NAME}"
 RUNTIME DESTINATION bin
 LIBRARY DESTINATION lib
 ARCHIVE DESTINATION lib
)


set(plugin_SRC
  src/XsensPlugin.cpp
  )

set(plugin_HDR
  src/XsensPlugin.h
)

# The add_plugin macro is provided by mc_rtc
add_plugin(XsensPlugin "${plugin_SRC}" "${plugin_HDR}")
target_link_libraries(XsensPlugin PUBLIC XsensPluginCommon)
target_include_directories(XsensPlugin PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
#install(FILES "${CMAKE_CURRENT_BINARY_DIR}/XsensPlugin.so" DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}")
# Build and link live version
if(TARGET xsens_streaming::xsens_streaming)
  add_library(XsensPluginLive SHARED src/XsensDataInputLive.cpp include/mc_xsens_plugin/XsensDataInputLive.h)
  target_link_libraries(XsensPluginLive PUBLIC xsens_streaming::xsens_streaming)
  set_target_properties(XsensPluginLive PROPERTIES INSTALL_RPATH "${MTSDK_LIBRARY_DIR}")
  target_link_libraries(XsensPluginLive PUBLIC XsensPluginCommon)
  target_include_directories(XsensPluginLive PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

  install(TARGETS XsensPluginLive
   EXPORT "${TARGETS_EXPORT_NAME}"
   RUNTIME DESTINATION bin
   LIBRARY DESTINATION lib
   ARCHIVE DESTINATION lib
  )

  target_link_libraries(XsensPlugin PUBLIC XsensPluginLive)
  target_compile_definitions(XsensPlugin PUBLIC "WITH_XSENS_STREAMING")
endif()

add_subdirectory(src/states)


set(PACKAGE_EXTRA_MACROS
  "set(XSENS_PLUGIN_STATES_LIBRARIES \"${MC_CONTROLLER_INSTALL_PREFIX}/${PROJECT_NAME}/states\")
  set(XSENS_PLUGIN_STATES_FILES \"${XSENS_PLUGIN_STATES_LIBRARIES}/data\")")

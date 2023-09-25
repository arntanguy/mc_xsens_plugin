add_fsm_state_simple(XsensRetargetting)
target_link_libraries(XsensRetargetting PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/states>)
target_link_libraries(XsensRetargetting PUBLIC XsensPluginCommon)

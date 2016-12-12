add_required_dependency(sot-core)
add_required_dependency(dynamic_graph_bridge)

add_executable(controller-talos
  talos-oh2sot.cpp
  talos-oh.cpp
  server-talos.cpp)

add_executable(seqplay-talos
  talos-oh2sqp.cpp
  talos-oh.cpp
  server-talossqp.cpp)

if(UNIX)

  target_link_libraries(
    controller-talos
    sot_loader roscpp xmlrpcpp
    ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_REGEX_LIBRARY}
    dl
    #boost_filesystem boost_program_options boost_regex
    #boost_system
    )
  pkg_config_use_dependency(controller-talos sot-core)
  pkg_config_use_dependency(controller-talos dynamic_graph_bridge)
  pkg_config_use_dependency(controller-talos openhrp3-simulator-wo-rtm)

  target_link_libraries(
    seqplay-talos
    roscpp xmlrpcpp
    ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_REGEX_LIBRARY}
    dl
    #boost_filesystem boost_program_options boost_regex
    #boost_system
    )
  pkg_config_use_dependency(seqplay-talos sot-core)
  pkg_config_use_dependency(seqplay-talos dynamic_graph_bridge)
  pkg_config_use_dependency(seqplay-talos openhrp3-simulator-wo-rtm)

endif()

install(TARGETS controller-talos DESTINATION bin CONFIGURATIONS Release Debug RelWithDebInfo)
install(TARGETS seqplay-talos DESTINATION bin CONFIGURATIONS Release Debug RelWithDebInfo)

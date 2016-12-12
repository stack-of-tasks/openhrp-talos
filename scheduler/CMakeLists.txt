set(program schedulerhrp2)

set(sources 
  simscheduler.cc
  scheduler.cpp
)

add_executable(${program} ${sources})

include_directories(${CMAKE_SOURCE_DIR}/scheduler)

pkg_config_use_dependency(${program} openhrp3-simulator-wo-rtm)

if (UNIX)
  SET(SH_SFX "sh")
endif()

macro(_config_sched_script sched_script_name)
   configure_file( "${CMAKE_CURRENT_SOURCE_DIR}/${sched_script_name}.in"
     "${CMAKE_CURRENT_BINARY_DIR}/${sched_script_name}")

   install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${sched_script_name} 
     DESTINATION example/${PROJECT_NAME}/scheduler
     PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

endmacro()

if(UNIX)

elseif(WIN32)
  set_target_properties(${program} PROPERTIES DEBUG_POSTFIX d )
  target_link_libraries(${program}
    optimized hrpUtil-${OPENHRP_LIBRARY_VERSION}
    optimized hrpModel-${OPENHRP_LIBRARY_VERSION}
    optimized hrpCorbaStubSkel-${OPENHRP_LIBRARY_VERSION}
    debug hrpUtil-${OPENHRP_LIBRARY_VERSION}d
    debug hrpModel-${OPENHRP_LIBRARY_VERSION}d
    debug hrpCorbaStubSkel-${OPENHRP_LIBRARY_VERSION}d
    ${OMNIORB_LIBRARIES})
endif()

install(TARGETS ${program} 
  DESTINATION example/${PROJECT_NAME}/scheduler
  CONFIGURATIONS Release Debug RelWithDebInfo)

# Create the list of bash scripts
set(sched_script_names
  scheduler-talos.${SH_SFX} 
  scheduler-talos-bush.${SH_SFX} 
  schedulerproject-talos.${SH_SFX} 
  schedulerproject-talos-bush.${SH_SFX} 
  schedulerproject-sqp-talos-bush.${SH_SFX} 
)

# Generate and install the bash scripts
foreach(sched_script_name ${sched_script_names})
  _config_sched_script(${sched_script_name})
endforeach()

install(PROGRAMS ${CMAKE_SOURCE_DIR}/scheduler/make-talos-walk.py
  DESTINATION example/${PROJECT_NAME}/scheduler
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

install(PROGRAMS ${CMAKE_SOURCE_DIR}/scheduler/PIDgains.dat
  DESTINATION example/${PROJECT_NAME}/scheduler/etc
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ )
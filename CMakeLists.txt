cmake_minimum_required(VERSION 2.6)

include(cmake/base.cmake)
include(cmake/pthread.cmake)
include(cmake/eigen.cmake)

set(PROJECT_NAME openhrp3-hrp2)
set(PROJECT_DESCRIPTION "Extension to simulate TALOS on openhrp")
set(PROJECT_URL "http://redmine.laas.fr/")

# Disable -Werror on Unix for now.
set(CXX_DISABLE_WERROR True)
set(CMAKE_VERBOSE_MAKEFILE True)

# Add std=c++11 flag
if (UNIX)
  SET(LFLAG -std=c++11)
  SET(CMAKE_CXX_FLAGS "${LFLAG} ${CMAKE_CXX_FLAGS}")
endif(UNIX)

setup_project()

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
  #cmake_policy(SET CMP0053 OLD)
endif(COMMAND cmake_policy)

# Check boost
find_package(Boost 1.35.0 REQUIRED COMPONENTS program_options filesystem signals system regex)

include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

add_required_dependency("openhrp3-simulator-wo-rtm")
include_directories(${OPENHRP3_SIMULATOR_WO_RTM_INCLUDE_DIRS})

search_for_eigen()

message("OPENHRP3_SIMULATOR_WO_RTM_INCLUDE_DIRS: ${OPENHRP3_SIMULATOR_WO_RTM_INCLUDE_DIRS}")

# Subdirectories
#add_subdirectory(customizer)
add_subdirectory(scheduler)
add_subdirectory(server)
add_subdirectory(project)
#add_subdirectory(data)

setup_project_finalize()
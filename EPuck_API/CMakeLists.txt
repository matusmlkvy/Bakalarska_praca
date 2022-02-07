cmake_minimum_required (VERSION 2.6)

# If the user specifies -DCMAKE_BUILD_TYPE on the command line, take their definition
# and dump it in the cache along with proper documentation, otherwise set CMAKE_BUILD_TYPE
# to Release prior to calling PROJECT()
#
IF(DEFINED CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
ELSE()
   SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
ENDIF()

project (EPuckAPI CXX)

# Add custom search modules.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

# Version number
set (EPUCKAPI_VERSION_MAJOR 1)
set (EPUCKAPI_VERSION_MINOR 0)
set (EPUCKAPI_VERSION ${EPUCKAPI_VERSION_MAJOR}.${EPUCKAPI_VERSION_MINOR})

#####################################################################
#           Options
#####################################################################

option(BUILD_SHARED_LIBS "Speficies whether to build libs as shared or as static." ON)

IF(BUILD_SHARED_LIBS)
	SET(LIBRARY_TYPE SHARED)

	add_definitions(
		-DGSYS_DLL -DALGOTREE_DLL
		-DGENETOR_DLL -DANNALYST_DLL
		-DMOTIVATOR_DLL -DRLEARNER_DLL
	)
ELSE(BUILD_SHARED_LIBS)
	SET(LIBRARY_TYPE STATIC)
ENDIF(BUILD_SHARED_LIBS)

#####################################################################
#           C++11 support
#####################################################################

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX14)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
else()
	message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

#####################################################################
#           Compiler-specific flags
#####################################################################

# Warnings for gcc.
if(CMAKE_COMPILER_IS_GNUCXX)
	SET(WARNINGS "${WARNINGS} -Wextra -Wall -Wconversion -pedantic -Wmain -Weffc++ -Wswitch-default -Wswitch-enum -Wmissing-include-dirs -Wmissing-declarations -Wfloat-equal -Wundef -Wcast-align -Wredundant-decls -Winit-self -Wshadow")
endif(CMAKE_COMPILER_IS_GNUCXX)

#####################################################################
#           		Search for the Libraries
#####################################################################

# Add custom search modules.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

# Find Boost.
SET(Boost_USE_MULTITHREADED ON)
find_package(Boost COMPONENTS system thread REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

#####################################################################
#           		Source files
#####################################################################

include_directories(../Firmware)

FILE( GLOB EPUCK_HEADERS
	*.h
	Firmware/packets.h
)

FILE( GLOB EPUCK_SRCS
	*.cpp
)

#####################################################################
#           		Instalation
#####################################################################

# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include/EPuckAPI CACHE PATH
  "Installation directory for header files")

if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/cmake/EPuckAPI)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
  "Installation directory for CMake files")
 
# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

# Add rpath for installation.
SET(CMAKE_INSTALL_RPATH "${INSTALL_LIB_DIR}")

#####################################################################
#           Building Doxygen documentation
#####################################################################

#-- Add an Option to toggle the generation of the API documentation
option(BUILD_DOCUMENTATION "Use Doxygen to create the HTML-based documentation." OFF)
option(DOCUMENTATION_ALL_TARGET "Whether to make documentation part of the default target." OFF)

if(DOCUMENTATION_ALL_TARGET)
	SET(DOCUMENTATION_TARGET ALL)
else(DOCUMENTATION_ALL_TARGET)
	SET(DOCUMENTATION_TARGET)
endif(DOCUMENTATION_ALL_TARGET)

if(BUILD_DOCUMENTATION)
  FIND_PACKAGE(Doxygen)
  if (NOT DOXYGEN_FOUND)
    message(FATAL_ERROR "Doxygen is needed to build the documentation.")
  endif()

  SET(HTML_OUTPUT .)

  #-- Configure the Template Doxyfile for our specific project
  configure_file(docs/EPuckAPI.doxy.in
                 ${PROJECT_BINARY_DIR}/EPuckAPI.doxy  @ONLY IMMEDIATE)
  #-- Add a custom target to run Doxygen when ever the project is built
  add_custom_target (docs ${DOCUMENTATION_TARGET}
		COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/EPuckAPI.doxy
		SOURCES ${PROJECT_BINARY_DIR}/EPuckAPI.doxy)
endif()

#####################################################################
#           		Subdirectories
#####################################################################

# SWIG bindings
ADD_SUBDIRECTORY(swig)

#####################################################################
#           The main library and executable
#####################################################################

add_library(EPuckAPI ${LIBRARY_TYPE} ${EPUCK_SRCS})
TARGET_LINK_LIBRARIES(EPuckAPI ${Boost_LIBRARIES})

set_target_properties(EPuckAPI PROPERTIES
  PUBLIC_HEADER "${EPUCK_HEADERS}")

# installation
install(TARGETS EPuckAPI
  EXPORT EPuckAPITargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}"
    COMPONENT dev)

#####################################################################
#           Installation
#####################################################################

# compute a list of all the libraries
set(EPUCKAPI_LIBRARIES EPuckAPI)

# Add all targets to the build-tree export set
export(TARGETS ${EPUCKAPI_LIBRARIES} FILE "${PROJECT_BINARY_DIR}/EPuckAPITargets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE EPuckAPI)

# Create the EPuckAPIConfig.cmake and EPuckAPIConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
   "${INSTALL_INCLUDE_DIR}")
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(cmake/EPuckAPIConfig.cmake.in
  "${PROJECT_BINARY_DIR}/EPuckAPIConfig.cmake" @ONLY)
# ... for the install tree
set(CONF_INCLUDE_DIRS "\${EPUCKAPI_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(cmake/EPuckAPIConfig.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/EPuckAPIConfig.cmake" @ONLY)
# ... for both
configure_file(cmake/EPuckAPIConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/EPuckAPIConfigVersion.cmake" @ONLY)

# Install the EPuckAPIConfig.cmake and EPuckAPIConfigVersion.cmake
install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/EPuckAPIConfig.cmake"
  "${PROJECT_BINARY_DIR}/EPuckAPIConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

# Install the export set for use with the install-tree
install(EXPORT EPuckAPITargets DESTINATION
  "${INSTALL_CMAKE_DIR}" COMPONENT dev)

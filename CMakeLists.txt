cmake_minimum_required( VERSION 2.8 FATAL_ERROR)
 
# Create Project
project( Rendering )
add_executable( project src/Source.cpp src/DrawPolygon.hpp include/Header.h include/picojson.h include/pointDefinition.h include/pointWaveLength.h src/visualizerConfig.hpp)
 
# Find Packages
find_package( PCL 1.8 REQUIRED )
 
if( PCL_FOUND )
  # Additional Include Directories
  # [C/C++]>[General]>[Additional Include Directories]
  include_directories( ${PCL_INCLUDE_DIRS} )
 
  # Preprocessor Definitions
  # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
  add_definitions( ${PCL_DEFINITIONS} )
  #add_definitions( -DPCL_NO_PRECOMPILE )
 
  # Additional Library Directories
  # [Linker]>[General]>[Additional Library Directories]
  link_directories( ${PCL_LIBRARY_DIRS} )
 
  # Additional Dependencies
  # [Linker]>[Input]>[Additional Dependencies]
  target_link_libraries( project ${PCL_LIBRARIES} )
endif()

find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories( ${OpenCV_LIBRARY_DIRS} )
target_link_libraries( project ${OpenCV_LIBRARIES} )

# Example CMakeLists.txt
# FindKInectSDK2.cmake copy to CMake\share\cmake-3.5\Modules or same directory as this file

cmake_minimum_required( VERSION 3.18 )
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH} )

message(STATUS ${CMAKE_MODULE_PATH})

project( kinfu-kinnect2 )
add_executable( ${PROJECT_NAME} main.cpp )
# set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "project" )

# Find Kinect SDK v2
find_package( KinectSDK2 REQUIRED )
# Settings Kinect SDK v2
if(KinectSDK2_FOUND)
  include_directories( ${KinectSDK2_INCLUDE_DIRS} )
  link_directories( ${KinectSDK2_LIBRARY_DIRS} )  
endif()

find_package( OpenCV REQUIRED )
if(OpenCV_FOUND)
	include_directories( ${OpenCV_INCLUDE_DIRS} )		
endif()

# set(Boost_USE_STATIC_LIBS OFF) 
# set(Boost_USE_MULTITHREADED ON)  
# set(Boost_USE_STATIC_RUNTIME OFF) 

find_package(Boost COMPONENTS program_options REQUIRED ) 
if(Boost_FOUND)    
	include_directories( ${Boost_INCLUDE_DIRS} )    
endif()

target_link_libraries( 
	${PROJECT_NAME} 
	${KinectSDK2_LIBRARIES} 
	${OpenCV_LIBS} 
	#Boost::filesystem
	#Boost::program_options
	${Boost_LIBRARIES}
)
add_custom_command( TARGET ${PROJECT_NAME} POST_BUILD ${KinectSDK2_COMMANDS} )
project( DRRT )

cmake_minimum_required( VERSION 2.8 )

set( CMAKE_BUILD_TYPE Default ) # Debug

list( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules )
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall" )

find_package( Eigen3 REQUIRED )
find_package( Pangolin REQUIRED )
find_package( SceneGraph REQUIRED )

set( _INCLUDE_DIRS
		${CMAKE_BINARY_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/include
                ${EIGEN3_INCLUDE_DIRS}
                ${Pangolin_INCLUDE_DIRS}
                ${SceneGraph_INCLUDE_DIRS}
		)

set( _LIBRARIES
                ${Pangolin_LIBRARIES}
                ${SceneGraph_LIBRARIES}
		)

# Expose these for third parties
set( DRRT_INCLUDE_DIRS ${_INCLUDE_DIRS}  CACHE INTERNAL "" FORCE )
set( DRRT_LIBRARIES  ${_LIBRARIES} CACHE INTERNAL "" FORCE )

include_directories( ${DRRT_INCLUDE_DIRS} )

set( HDRS
                include/DRRT/drrt.h
                include/DRRT/kdtree.h
                include/DRRT/heap.h
                include/DRRT/ghostPoint.h
                include/DRRT/list.h
                include/DRRT/jlist.h
                include/DRRT/kdtreenode.h
                include/DRRT/drrt_data_structures.h
                include/DRRT/edge.h
                include/DRRT/dubinsedge.h
                include/DRRT/holedge.h
                include/DRRT/drrt_distance_functions.h
                include/DRRT/visualizer.h
                include/DRRT/obstacle.h
		)

set( SRCS
		src/drrt.cpp
                src/kdtree.cpp
                src/heap.cpp
                src/ghostPoint.cpp
                src/list.cpp
                src/jlist.cpp
                src/dubinsedge.cpp
                src/holedge.cpp
                src/drrt_distance_functions.cpp
                src/visualizer.cpp
                src/obstacle.cpp
		)



set( LIBRARY_NAME ${PROJECT_NAME} )
string( TOLOWER ${LIBRARY_NAME} LIBRARY_NAME )
# VV Make this shared VV ( ... ${LIBRARY_NAME} SHARED ${SRCS} ... )?
add_library( ${LIBRARY_NAME} ${SRCS} ${HDRS} )
target_link_libraries( ${LIBRARY_NAME} ${DRRT_LIBRARIES} )

#add_executable( heaptest src/heap.cpp ${HDRS} )
#target_link_libraries( heaptest ${LIBRARY_NAME} )

#add_executable( listtest src/list.cpp ${HDRS} )
#target_link_libraries( listtest ${LIBRARY_NAME} )

#add_executable( jlisttest src/jlist.cpp ${HDRS} )
#target_link_libraries( jlisttest ${LIBRARY_NAME} )

add_executable( largetest src/largetest.cpp ${HDRS} )
target_link_libraries( largetest ${LIBRARY_NAME} )

add_executable( straighttest src/straighttest.cpp ${HDRS} )
target_link_libraries( straighttest ${LIBRARY_NAME} )

add_executable( smalltest src/smalltest.cpp ${HDRS} )
target_link_libraries( smalltest ${LIBRARY_NAME} )

#install_package(
#    PKG_NAME ${PROJECT_NAME}
#    LIB_NAME ${LIBRARY_NAME}
#    VERSION 0.1
#    DESCRIPTION "RRTx library."
#    INSTALL_INCLUDE_DIR true
#    INSTALL_GENERATED_HEADERS ${GENERATED_HEADERS}
#    DESTINATION ${CMAKE_INSTALL_PREFIX}
#    INCLUDE_DIRS ${DRRT_INCLUDE_DIRS}
#    LINK_LIBS ${DRRT_LIBRARIES}
#    )

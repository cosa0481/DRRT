project( DRRT )

cmake_minimum_required( VERSION 2.8 )

set( CMAKE_BUILD_TYPE Debug ) # Default

list( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules )
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall" )

find_package( Eigen3 REQUIRED )

set( _INCLUDE_DIRS
		${CMAKE_BINARY_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/include
                ${EIGEN3_INCLUDE_DIRS}
		)

set( _LIBRARIES
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
                include/DRRT/drrt_distance_functions.h
		)

set( SRCS
		src/drrt.cpp
		src/kdtree.cpp
                src/heap.cpp
                src/ghostPoint.cpp
                src/list.cpp
                src/jlist.cpp
                src/dubinsedge.cpp
                src/drrt_distance_functions.cpp
		)



set( LIBRARY_NAME ${PROJECT_NAME} )
string( TOLOWER ${LIBRARY_NAME} LIBRARY_NAME )
# VV Make this shared VV ( ... ${LIBRARY_NAME} SHARED ${SRCS} ... )
add_library( ${LIBRARY_NAME} ${SRCS} ${HDRS} )
target_link_libraries( ${LIBRARY_NAME} ${DRRT_LIBRARIES} )

#add_executable( heaptest src/heap.cpp ${HDRS} )
#target_link_libraries( heaptest ${LIBRARY_NAME} )

#add_executable( listtest src/list.cpp ${HDRS} )
#target_link_libraries( listtest ${LIBRARY_NAME} )

#add_executable( jlisttest src/jlist.cpp ${HDRS} )
#target_link_libraries( jlisttest ${LIBRARY_NAME} )

add_executable( test src/test.cpp ${HDRS} )
target_link_libraries( test ${LIBRARY_NAME} )

add_executable( unitTest src/unittest.cpp ${HDRS} )
target_link_libraries( unitTest ${LIBRARY_NAME} )

#install_package(
#    PKG_NAME ${PROJECT_NAME}
#    LIB_NAME ${LIBRARY_NAME}
#    VERSION 2.1
#    DESCRIPTION "DRRT library."
#    INSTALL_INCLUDE_DIR true
#    INSTALL_GENERATED_HEADERS ${GENERATED_HEADERS}
#    DESTINATION ${CMAKE_INSTALL_PREFIX}
#    INCLUDE_DIRS ${DRRT_INCLUDE_DIRS}
#    LINK_LIBS ${DRRT_LIBRARIES}
#    )

project( DRRT )

cmake_minimum_required( VERSION 2.8 )

list( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules )
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall" )

set( _INCLUDE_DIRS
		${CMAKE_BINARY_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/include
		)

set( _LIBRARIES
		)

include_directories( ${_INCLUDE_DIRS} )

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
		)

set( SRCS
		src/drrt.cpp
		src/kdtree.cpp
                src/heap.cpp
                src/ghostPoint.cpp
                src/list.cpp
                src/jlist.cpp
		)



set( LIBRARY_NAME ${PROJECT_NAME} )
string( TOLOWER ${LIBRARY_NAME} LIBRARY_NAME )
add_library( ${LIBRARY_NAME} ${SRCS} ${HDRS} )
target_link_libraries( ${LIBRARY_NAME} ${_LIBRARIES} )

add_executable( heaptest src/heap.cpp ${HDRS} )
target_link_libraries( heaptest ${LIBRARY_NAME} )

#add_executable( listtest src/list.cpp ${HDRS} )
#target_link_libraries( listtest ${LIBRARY_NAME} )

#add_executable( jlisttest src/jlist.cpp ${HDRS} )
#target_link_libraries( jlisttest ${LIBRARY_NAME} )

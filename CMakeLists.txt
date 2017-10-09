cmake_minimum_required( VERSION 2.8 )
project( DRRT )
set( VERSION 0.0.2 )

set( CMAKE_BUILD_TYPE Default )

list( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules )
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall" )

find_package( Eigen3        REQUIRED )
find_package( Pangolin      REQUIRED )
find_package( SceneGraph    REQUIRED )
find_package( Bullet        REQUIRED )

set( _INCLUDE_DIRS
        ${CMAKE_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
                ${EIGEN3_INCLUDE_DIRS}
                ${Pangolin_INCLUDE_DIRS}
                ${SceneGraph_INCLUDE_DIRS}
                ${BULLET_INCLUDE_DIR}
        )

set( _LIBRARIES
                ${Pangolin_LIBRARIES}
                ${SceneGraph_LIBRARIES}
                ${BULLET_LIBRARIES}
        )

# Expose these for third parties
set( DRRT_INCLUDE_DIRS  ${_INCLUDE_DIRS}    CACHE INTERNAL "" FORCE )
set( DRRT_LIBRARIES     ${_LIBRARIES}       CACHE INTERNAL "" FORCE )

# Required for bullet to compile everything with double precision instead of float
add_definitions(-DBT_USE_DOUBLE_PRECISION)

set( HDRS
                # exported header
                include/DRRT/drrt.h

                # global headers
                include/DRRT/libraries.h
                include/DRRT/data_structures.h
                include/DRRT/distance_functions.h
                include/DRRT/sampling.h

                # k-d tree
                include/DRRT/kdtree.h
                include/DRRT/ghost_point.h
                include/DRRT/kdnode.h
                include/DRRT/neighbor_iterator.h

                # obstacles
                include/DRRT/obstacle.h
                include/DRRT/collision_detection.h

                # edges
                include/DRRT/edge.h
                include/DRRT/dubins_edge.h

                # regions
                include/DRRT/region.h
                include/DRRT/region2d.h

                # lists
                include/DRRT/list.h
                include/DRRT/obstacle_listnode.h
                include/DRRT/edge_listnode.h
                include/DRRT/kdnode_listnode.h
                include/DRRT/range_listnode.h

                # heap
                include/DRRT/heap.h
                include/DRRT/heapnode.h
                include/DRRT/kdnode_heapnode.h

                # tripoly lib headers
                include/DRRT/tripolyinterface.h
                include/DRRT/triangulate.h
        )

set( SRCS
                # global functions
                src/distance_functions.cpp
                src/sampling.cpp
                src/drrt.cpp

                # k-d tree
                src/kdtree.cpp
                src/ghost_point.cpp
                src/kdnode.cpp
                src/neighbor_iterator.cpp

                # obstacles
                src/obstacle.cpp
                src/collision_detection.cpp

                # edges
                src/dubins_edge.cpp

                # regions
                src/region2d.cpp

                # lists
                src/list.cpp

                # heap
                src/heap.cpp

                # tripoly lib functions
                src/construct.c
                src/tri.c
                src/monotone.c
                src/misc.c
        )

option(BUILD_EXAMPLES "Build Examples" ON)
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

include_directories( ${DRRT_INCLUDE_DIRS} )

set( LIBRARY_NAME ${PROJECT_NAME} )
string( TOLOWER ${LIBRARY_NAME} LIBRARY_NAME )

# VV Make this shared VV ( ... ${LIBRARY_NAME} SHARED ${SRCS} ... )???
add_library( ${LIBRARY_NAME} ${SRCS} ${HDRS} )
target_link_libraries( ${LIBRARY_NAME} ${_LIBRARIES} )

# VV Didn't do anything
#install(TARGETS ${LIBRARY_NAME} DESTINATION lib)
#install(FILES include/DRRT/drrt.h DESTINATION include)

# add_executable( smalltest src/smalltest.cpp ${HDRS} )
# target_link_libraries( smalltest ${LIBRARY_NAME} )

# List class test
# add_executable( list_test src/list.cpp ${HDRS} )
# target_link_libraries( ${LIBRARY_NAME} ${DRRT_LIBRARIES} )

# Heap class test
# add_executable( heap_test src/heap.cpp ${HDRS} )
# target_link_libraries( ${LIBRARY_NAME} ${DRRT_LIBRARIES} )

# VV Needed for release???
#install_package(
#    PKG_NAME ${PROJECT_NAME}
#    LIB_NAME ${LIBRARY_NAME}
#    VERSION ${VERSION}
#        DESCRIPTION "RRTx library"
##        INSTALL_INCLUDE_DIR true
##        INSTALL_GENERATED_HEADERS ${GENERATED_HEADERS}
#    DESTINATION ${CMAKE_INSTALL_PREFIX}
#    INCLUDE_DIRS ${_INCLUDE_DIRS}
#    LINK_LIBS ${_LIBRARIES}
#)

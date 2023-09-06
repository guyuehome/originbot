cmake_minimum_required(VERSION 3.5)

project(sdk_core)

set(CMAKE_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME})

# Set the build type.  Options are:
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

include_directories(${PROJECT_SOURCE_DIR}/include)

set(sdk_core_sources
    # common sources
    ${PROJECT_SOURCE_DIR}/common/configurator.cc
    ${PROJECT_SOURCE_DIR}/common/loader.cc
    ${PROJECT_SOURCE_DIR}/common/scripts.cc

    # dirvers sources
    ${PROJECT_SOURCE_DIR}/drivers/ethernet_win.cc
    ${PROJECT_SOURCE_DIR}/drivers/uart_win.cc

    ${PROJECT_SOURCE_DIR}/drivers/ethernet_unix.cc
    ${PROJECT_SOURCE_DIR}/drivers/uart_unix.cc

    ${PROJECT_SOURCE_DIR}/drivers/protocol.cc
    ${PROJECT_SOURCE_DIR}/drivers/lidar.cc

    # filters sources
    ${PROJECT_SOURCE_DIR}/filters/outlierfilter.cc
    ${PROJECT_SOURCE_DIR}/filters/search.cc
    ${PROJECT_SOURCE_DIR}/filters/smoother.cc
    ${PROJECT_SOURCE_DIR}/filters/trailingfilter.cc

    ${PROJECT_SOURCE_DIR}/sdkcore.cc
)

# add third_party library
include_directories(${PROJECT_SOURCE_DIR}/third_party/lua/include)
set(lua_sources
    # CORE CODE
    ${PROJECT_SOURCE_DIR}/third_party/lua/lapi.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/lcode.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/lctype.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/ldebug.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/ldo.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/ldump.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/lfunc.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/lgc.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/llex.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/lmem.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/lobject.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/lopcodes.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/loslib.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/lparser.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/lstate.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/lstring.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/ltable.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/ltm.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/lundump.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/lvm.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/lzio.c

    # LIB CODE
    ${PROJECT_SOURCE_DIR}/third_party/lua/lauxlib.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/lbaselib.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/lcorolib.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/ldblib.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/liolib.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/lmathlib.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/loadlib.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/lstrlib.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/ltablib.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/lutf8lib.c
    ${PROJECT_SOURCE_DIR}/third_party/lua/linit.c
)

# add flann sources
include_directories(${PROJECT_SOURCE_DIR}/third_party/flann/src/cpp)
set(flann_sources
    ${PROJECT_SOURCE_DIR}/third_party/flann/src/cpp/flann/flann_cpp.cpp
)

# add lz4 sources
include_directories(${PROJECT_SOURCE_DIR}/third_party/lz4/lib)
set(lz4_sources
    ${PROJECT_SOURCE_DIR}/third_party/lz4/lib/lz4.c
    ${PROJECT_SOURCE_DIR}/third_party/lz4/lib/lz4frame.c
    ${PROJECT_SOURCE_DIR}/third_party/lz4/lib/lz4hc.c
    ${PROJECT_SOURCE_DIR}/third_party/lz4/lib/xxhash.c
)

add_library(${PROJECT_NAME} SHARED
    ${sdk_core_sources}
    ${lua_sources}
    ${flann_sources}
    ${lz4_sources}
)

if(CMAKE_HOST_WIN32)
target_link_libraries(${PROJECT_NAME} wsock32)
endif()

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/config/ DESTINATION config)

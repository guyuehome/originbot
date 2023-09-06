cmake_minimum_required(VERSION 2.8.3)

project(lua)

set(lua_headers
    ${PROJECT_SOURCE_DIR}/src/lprefix.h
    ${PROJECT_SOURCE_DIR}/src/lua.h
    ${PROJECT_SOURCE_DIR}/src/luaconf.h
    ${PROJECT_SOURCE_DIR}/src/lapi.h
    ${PROJECT_SOURCE_DIR}/src/llimits.h
    ${PROJECT_SOURCE_DIR}/src/lstate.h
    ${PROJECT_SOURCE_DIR}/src/lobject.h
    ${PROJECT_SOURCE_DIR}/src/ltm.h
    ${PROJECT_SOURCE_DIR}/src/lzio.h
    ${PROJECT_SOURCE_DIR}/src/lmem.h
    ${PROJECT_SOURCE_DIR}/src/ldebug.h
    ${PROJECT_SOURCE_DIR}/src/ldo.h
    ${PROJECT_SOURCE_DIR}/src/lfunc.h
    ${PROJECT_SOURCE_DIR}/src/lgc.h
    ${PROJECT_SOURCE_DIR}/src/lstring.h
    ${PROJECT_SOURCE_DIR}/src/ltable.h
    ${PROJECT_SOURCE_DIR}/src/lundump.h
    ${PROJECT_SOURCE_DIR}/src/lvm.h
    ${PROJECT_SOURCE_DIR}/src/lopcodes.h
    ${PROJECT_SOURCE_DIR}/src/lauxlib.h
    ${PROJECT_SOURCE_DIR}/src/lcode.h
    ${PROJECT_SOURCE_DIR}/src/lparser.h
    ${PROJECT_SOURCE_DIR}/src/ljumptab.h
    ${PROJECT_SOURCE_DIR}/src/lopnames.h
    ${PROJECT_SOURCE_DIR}/src/lualib.h
    ${PROJECT_SOURCE_DIR}/src/llex.h
    ${PROJECT_SOURCE_DIR}/src/lctype.h
)

set(lua_sources
    # CORE CODE
    ${PROJECT_SOURCE_DIR}/src/lapi.c
    ${PROJECT_SOURCE_DIR}/src/lcode.c
    ${PROJECT_SOURCE_DIR}/src/lctype.c
    ${PROJECT_SOURCE_DIR}/src/ldebug.c
    ${PROJECT_SOURCE_DIR}/src/ldo.c
    ${PROJECT_SOURCE_DIR}/src/ldump.c
    ${PROJECT_SOURCE_DIR}/src/lfunc.c
    ${PROJECT_SOURCE_DIR}/src/lgc.c
    ${PROJECT_SOURCE_DIR}/src/llex.c
    ${PROJECT_SOURCE_DIR}/src/lmem.c
    ${PROJECT_SOURCE_DIR}/src/lobject.c
    ${PROJECT_SOURCE_DIR}/src/lopcodes.c
    ${PROJECT_SOURCE_DIR}/src/loslib.c
    ${PROJECT_SOURCE_DIR}/src/lparser.c
    ${PROJECT_SOURCE_DIR}/src/lstate.c
    ${PROJECT_SOURCE_DIR}/src/lstring.c
    ${PROJECT_SOURCE_DIR}/src/ltable.c
    ${PROJECT_SOURCE_DIR}/src/ltm.c
    ${PROJECT_SOURCE_DIR}/src/lundump.c
    ${PROJECT_SOURCE_DIR}/src/lvm.c
    ${PROJECT_SOURCE_DIR}/src/lzio.c

    # LIB CODE
    ${PROJECT_SOURCE_DIR}/src/lauxlib.c
    ${PROJECT_SOURCE_DIR}/src/lbaselib.c
    ${PROJECT_SOURCE_DIR}/src/lcorolib.c
    ${PROJECT_SOURCE_DIR}/src/ldblib.c
    ${PROJECT_SOURCE_DIR}/src/liolib.c
    ${PROJECT_SOURCE_DIR}/src/lmathlib.c
    ${PROJECT_SOURCE_DIR}/src/loadlib.c
    ${PROJECT_SOURCE_DIR}/src/lstrlib.c
    ${PROJECT_SOURCE_DIR}/src/ltablib.c
    ${PROJECT_SOURCE_DIR}/src/lutf8lib.c
    ${PROJECT_SOURCE_DIR}/src/linit.c
)

add_library(${PROJECT_NAME} SHARED ${lua_headers} ${lua_sources})
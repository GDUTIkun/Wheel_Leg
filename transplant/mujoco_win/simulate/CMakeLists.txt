cmake_minimum_required(VERSION 3.20)
project(wheel_leg_simulate LANGUAGES CXX)

include(FetchContent)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT wheel_leg_simulate)

option(WHEEL_LEG_USE_SYSTEM_MUJOCO "Use an installed MuJoCo package when available." ON)
option(WHEEL_LEG_USE_SYSTEM_GLFW "Use an installed GLFW package when available." ON)

set(MUJOCO_PATH "" CACHE PATH "MuJoCo install root. Example: C:/mujoco-3.2.6")
set(GLFW_ROOT "" CACHE PATH "Optional GLFW install root")

find_package(Threads REQUIRED)

function(_wheel_leg_set_imported_runtime target runtime_path)
  if(WIN32 AND EXISTS "${runtime_path}")
    set_property(TARGET "${target}" APPEND PROPERTY
      IMPORTED_CONFIGURATIONS RELEASE DEBUG RELWITHDEBINFO MINSIZEREL)
    set_target_properties("${target}" PROPERTIES
      IMPORTED_LOCATION_RELEASE "${runtime_path}"
      IMPORTED_LOCATION_DEBUG "${runtime_path}"
      IMPORTED_LOCATION_RELWITHDEBINFO "${runtime_path}"
      IMPORTED_LOCATION_MINSIZEREL "${runtime_path}")
  endif()
endfunction()

set(MUJOCO_TARGET "")
set(MUJOCO_RUNTIME_DLL "")

if(MUJOCO_PATH)
  list(PREPEND CMAKE_PREFIX_PATH "${MUJOCO_PATH}")
endif()

if(WHEEL_LEG_USE_SYSTEM_MUJOCO)
  find_package(mujoco CONFIG QUIET)
endif()

if(TARGET mujoco::mujoco)
  set(MUJOCO_TARGET mujoco::mujoco)
elseif(TARGET mujoco)
  set(MUJOCO_TARGET mujoco)
else()
  if(NOT MUJOCO_PATH)
    if(WIN32 AND DEFINED ENV{MUJOCO_PATH} AND NOT "$ENV{MUJOCO_PATH}" STREQUAL "")
      file(TO_CMAKE_PATH "$ENV{MUJOCO_PATH}" MUJOCO_PATH)
    elseif(DEFINED ENV{MUJOCO_DIR} AND NOT "$ENV{MUJOCO_DIR}" STREQUAL "")
      file(TO_CMAKE_PATH "$ENV{MUJOCO_DIR}" MUJOCO_PATH)
    elseif(UNIX)
      set(MUJOCO_PATH "/opt/mujoco-3.7.0")
    endif()
  endif()

  if(MUJOCO_PATH)
    file(TO_CMAKE_PATH "${MUJOCO_PATH}" MUJOCO_PATH)
  endif()

  if(NOT MUJOCO_PATH OR NOT EXISTS "${MUJOCO_PATH}/include/mujoco/mujoco.h")
    message(FATAL_ERROR
      "MuJoCo was not found.\n"
      "Provide a package config, or configure -DMUJOCO_PATH=<MuJoCo install root>.\n"
      "On Windows, MUJOCO_PATH usually looks like C:/mujoco-<version>.")
  endif()

  add_library(mujoco_local SHARED IMPORTED GLOBAL)

  if(WIN32)
    set(_mujoco_import_candidates
      "${MUJOCO_PATH}/lib/mujoco.lib"
      "${MUJOCO_PATH}/bin/mujoco.lib")
    set(_mujoco_runtime_candidates
      "${MUJOCO_PATH}/bin/mujoco.dll"
      "${MUJOCO_PATH}/lib/mujoco.dll")
  elseif(APPLE)
    set(_mujoco_import_candidates
      "${MUJOCO_PATH}/lib/libmujoco.dylib")
    set(_mujoco_runtime_candidates
      "${MUJOCO_PATH}/lib/libmujoco.dylib")
  else()
    set(_mujoco_import_candidates
      "${MUJOCO_PATH}/lib/libmujoco.so")
    set(_mujoco_runtime_candidates
      "${MUJOCO_PATH}/lib/libmujoco.so")
  endif()

  set(_mujoco_library "")
  foreach(candidate IN LISTS _mujoco_import_candidates)
    if(EXISTS "${candidate}")
      set(_mujoco_library "${candidate}")
      break()
    endif()
  endforeach()

  if(NOT _mujoco_library)
    message(FATAL_ERROR
      "MuJoCo library was not found under ${MUJOCO_PATH}.\n"
      "Checked: ${_mujoco_import_candidates}")
  endif()

  if(WIN32)
    set_target_properties(mujoco_local PROPERTIES
      IMPORTED_IMPLIB "${_mujoco_library}"
      IMPORTED_LOCATION "${_mujoco_library}"
      INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_PATH}/include")
  else()
    set_target_properties(mujoco_local PROPERTIES
      IMPORTED_LOCATION "${_mujoco_library}"
      INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_PATH}/include")
  endif()

  if(WIN32)
    foreach(candidate IN LISTS _mujoco_runtime_candidates)
      if(EXISTS "${candidate}")
        set(MUJOCO_RUNTIME_DLL "${candidate}")
        break()
      endif()
    endforeach()
    _wheel_leg_set_imported_runtime(mujoco_local "${MUJOCO_RUNTIME_DLL}")
  endif()

  set(MUJOCO_TARGET mujoco_local)
endif()

set(GLFW_TARGET "")
set(GLFW_RUNTIME_DLL "")

if(GLFW_ROOT)
  file(TO_CMAKE_PATH "${GLFW_ROOT}" GLFW_ROOT)
  list(PREPEND CMAKE_PREFIX_PATH "${GLFW_ROOT}")
endif()

if(WHEEL_LEG_USE_SYSTEM_GLFW)
  find_package(glfw3 CONFIG QUIET)
endif()

if(TARGET glfw)
  set(GLFW_TARGET glfw)
elseif(TARGET glfw3::glfw)
  set(GLFW_TARGET glfw3::glfw)
elseif(TARGET glfw3)
  set(GLFW_TARGET glfw3)
else()
  if(WIN32)
    if(DEFINED ENV{GLFW_ROOT} AND NOT "$ENV{GLFW_ROOT}" STREQUAL "")
      file(TO_CMAKE_PATH "$ENV{GLFW_ROOT}" GLFW_ROOT_ENV)
    else()
      set(GLFW_ROOT_ENV "")
    endif()

    find_path(GLFW_INCLUDE_DIR
      NAMES GLFW/glfw3.h
      HINTS
        "${GLFW_ROOT}"
        "${GLFW_ROOT_ENV}"
      PATH_SUFFIXES include)
    find_library(GLFW_LIBRARY
      NAMES glfw3 glfw libglfw3 libglfw3dll
      HINTS
        "${GLFW_ROOT}"
        "${GLFW_ROOT_ENV}"
      PATH_SUFFIXES
        lib
        lib-mingw-w64
        lib-static-ucrt
        lib-vc2022
        lib-vc2019
        bin)
    find_file(GLFW_RUNTIME_DLL
      NAMES glfw3.dll
      HINTS
        "${GLFW_ROOT}"
        "${GLFW_ROOT_ENV}"
      PATH_SUFFIXES
        bin
        lib
        lib-mingw-w64
        lib-static-ucrt
        lib-vc2022
        lib-vc2019)

    if(GLFW_INCLUDE_DIR AND GLFW_LIBRARY)
      add_library(glfw_local UNKNOWN IMPORTED GLOBAL)
      if(WIN32)
        get_filename_component(_glfw_library_name "${GLFW_LIBRARY}" NAME)
        if(GLFW_RUNTIME_DLL AND (_glfw_library_name MATCHES "\\.lib$" OR _glfw_library_name MATCHES "dll\\.a$"))
          set_target_properties(glfw_local PROPERTIES
            IMPORTED_IMPLIB "${GLFW_LIBRARY}"
            IMPORTED_LOCATION "${GLFW_RUNTIME_DLL}"
            INTERFACE_INCLUDE_DIRECTORIES "${GLFW_INCLUDE_DIR}")
        else()
          set_target_properties(glfw_local PROPERTIES
            IMPORTED_LOCATION "${GLFW_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${GLFW_INCLUDE_DIR}")
        endif()
      else()
        set_target_properties(glfw_local PROPERTIES
          IMPORTED_LOCATION "${GLFW_LIBRARY}"
          INTERFACE_INCLUDE_DIRECTORIES "${GLFW_INCLUDE_DIR}")
      endif()
      _wheel_leg_set_imported_runtime(glfw_local "${GLFW_RUNTIME_DLL}")
      set(GLFW_TARGET glfw_local)
    endif()
  endif()
endif()

if(NOT GLFW_TARGET)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
  set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

  FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(glfw)
  set(GLFW_TARGET glfw)
endif()

add_executable(wheel_leg_simulate
  app/main.cc
  mujoco_ui/glfw_adapter.cc
  mujoco_ui/glfw_dispatch.cc
  mujoco_ui/platform_ui_adapter.cc
  mujoco_ui/simulate.cc
  wheel_leg/wheel_leg_hooks.cc
  tools/sensor.cc
  tools/actuator.cc
  tools/ekf.cc
  tools/lqr_k.cc
  tools/pid.cc
  tools/plotter.cc
  tools/math_utils.cc
  tools/vmc.cc
)

target_include_directories(wheel_leg_simulate PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/app
  ${CMAKE_CURRENT_SOURCE_DIR}/mujoco_ui
  ${CMAKE_CURRENT_SOURCE_DIR}/wheel_leg
  ${CMAKE_CURRENT_SOURCE_DIR}/tools
)

target_link_libraries(wheel_leg_simulate PRIVATE
  ${MUJOCO_TARGET}
  ${GLFW_TARGET}
  Threads::Threads
)

if(APPLE)
  target_sources(wheel_leg_simulate PRIVATE mujoco_ui/glfw_corevideo.mm)
endif()

if(UNIX AND NOT APPLE)
  target_link_libraries(wheel_leg_simulate PRIVATE dl)
endif()

if(WIN32)
  target_compile_definitions(wheel_leg_simulate PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)

  if(MUJOCO_RUNTIME_DLL)
    add_custom_command(TARGET wheel_leg_simulate POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${MUJOCO_RUNTIME_DLL}"
        "$<TARGET_FILE_DIR:wheel_leg_simulate>")
  endif()

  if(GLFW_RUNTIME_DLL)
    add_custom_command(TARGET wheel_leg_simulate POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${GLFW_RUNTIME_DLL}"
        "$<TARGET_FILE_DIR:wheel_leg_simulate>")
  endif()
endif()

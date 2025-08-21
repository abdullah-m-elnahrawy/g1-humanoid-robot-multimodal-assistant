cmake_minimum_required(VERSION 3.5)
project(g1_motions LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Let code find config/motion_data at runtime
add_definitions(-DPROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

# Binaries → build/bin
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ---------------- Branding header (configured) ----------------
set(PROJECT_SEMVER     "1.0.0")
set(PROJECT_NAME_STR   "g1-humanoid-voice-based-motions")
set(PROJECT_AUTHOR_STR "Abdullah El-Nahrawy")
set(PROJECT_EMAIL_STR  "abdullah.m.elnahrawy@outlook.com")
set(PROJECT_GITHUB_STR "https://github.com/abdullah-m-elnahrawy")

string(TIMESTAMP BUILD_TS "%Y-%m-%d %H:%M:%S %Z")
find_package(Git QUIET)
if(GIT_FOUND)
  execute_process(COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
                  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
                  OUTPUT_VARIABLE GIT_HASH
                  OUTPUT_STRIP_TRAILING_WHITESPACE)
else()
  set(GIT_HASH "nogit")
endif()

set(GENERATED_DIR ${CMAKE_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${GENERATED_DIR})
configure_file(${CMAKE_SOURCE_DIR}/config/project_branding.hpp.in
               ${GENERATED_DIR}/project_branding.hpp @ONLY)

# ---------------- Dependencies ----------------
# Try config package first (preferred)
find_package(unitree_sdk2 QUIET)

# If not found, allow a user hint or common prefixes
if(NOT unitree_sdk2_FOUND)
  set(UNITREE_SDK2_ROOT "$ENV{UNITREE_SDK2_ROOT}" CACHE PATH "Unitree SDK2 root")

  set(_UNITREE_HINTS
      ${UNITREE_SDK2_ROOT}
      $ENV{CMAKE_PREFIX_PATH}
      /opt/unitree_robotics
      /usr/local
      /usr
  )

  find_path(UNITREE_SDK2_INCLUDE_DIR
            NAMES unitree/robot/channel/channel_factory.hpp
            HINTS ${_UNITREE_HINTS}
            PATH_SUFFIXES include)

  find_library(UNITREE_SDK2_LIBRARY
               NAMES unitree_sdk2
               HINTS ${_UNITREE_HINTS}
               PATH_SUFFIXES lib)

  if(UNITREE_SDK2_INCLUDE_DIR AND UNITREE_SDK2_LIBRARY)
    add_library(unitree_sdk2 UNKNOWN IMPORTED)
    set_target_properties(unitree_sdk2 PROPERTIES
      IMPORTED_LOCATION "${UNITREE_SDK2_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${UNITREE_SDK2_INCLUDE_DIR}")
    message(STATUS "Unitree SDK2 found (manual): ${UNITREE_SDK2_INCLUDE_DIR}")
    get_filename_component(_UNITREE_LIBDIR "${UNITREE_SDK2_LIBRARY}" DIRECTORY)
    list(APPEND CMAKE_BUILD_RPATH "${_UNITREE_LIBDIR}")
    list(APPEND CMAKE_INSTALL_RPATH "${_UNITREE_LIBDIR}")
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
  else()
    message(FATAL_ERROR
      "unitree_sdk2 not found.\n"
      "Hint: pass -DUNITREE_SDK2_ROOT=/path/to/sdk or set CMAKE_PREFIX_PATH.")
  endif()
endif()

# Other packages
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenSSL REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Threads REQUIRED)

# ---------------- Internal static lib ----------------
add_library(audio_pipeline STATIC
    src/audio/audio_processing_pipeline.cpp
    src/wakeword/wake_word_gate.cpp
)
target_include_directories(audio_pipeline
    PUBLIC  ${CMAKE_SOURCE_DIR}/include
    PRIVATE ${CMAKE_SOURCE_DIR}/src
)

# ---------------- Executables ----------------
add_executable(abdullah_elnahrawy_g1_motions
    src/g1_motions.cpp
    src/branding_fingerprint.cpp
)
add_executable(voice_interface
    src/voice_interface.cpp
    src/branding_fingerprint.cpp
)

# Public + private includes (and generated header)
foreach(tgt IN ITEMS abdullah_elnahrawy_g1_motions voice_interface)
  target_include_directories(${tgt}
      PUBLIC  ${CMAKE_SOURCE_DIR}/include
      PRIVATE ${CMAKE_SOURCE_DIR}/src
              ${GENERATED_DIR}
  )
endforeach()

# If the official config exported include dirs
if(DEFINED unitree_sdk2_INCLUDE_DIRS)
  target_include_directories(abdullah_elnahrawy_g1_motions PRIVATE ${unitree_sdk2_INCLUDE_DIRS})
  target_include_directories(voice_interface              PRIVATE ${unitree_sdk2_INCLUDE_DIRS})
endif()

# If we did manual discovery, use that include dir too
if(UNITREE_SDK2_INCLUDE_DIR)
  target_include_directories(abdullah_elnahrawy_g1_motions PRIVATE ${UNITREE_SDK2_INCLUDE_DIR})
  target_include_directories(voice_interface              PRIVATE ${UNITREE_SDK2_INCLUDE_DIR})
endif()

# ---------------- Unitree include dir fallback ----------------
# Even if find_package(unitree_sdk2) succeeds, some configs don't export include dirs.
# We probe a few candidates (env/vars/common paths) for the header and add it explicitly.

set(_UT_CANDIDATES
    ${UNITREE_SDK2_INCLUDE_DIR}           # from our manual discovery (if any)
    ${unitree_sdk2_INCLUDE_DIRS}          # from the package, if provided
    $ENV{UNITREE_SDK2_INCLUDE_DIR}        # user override via env
    /opt/unitree_robotics/include         # common install prefix you used
    /usr/local/include
    /usr/include
)

set(UNITREE_INCLUDE_FINAL "")
foreach(_c IN LISTS _UT_CANDIDATES)
  if(_c AND EXISTS "${_c}/unitree/robot/channel/channel_factory.hpp")
    set(UNITREE_INCLUDE_FINAL "${_c}")
    break()
  endif()
endforeach()

if(NOT UNITREE_INCLUDE_FINAL)
  message(WARNING "Unitree headers not auto-detected. "
                  "Pass -DUNITREE_SDK2_INCLUDE_DIR=/opt/unitree_robotics/include to CMake.")
endif()

if(UNITREE_INCLUDE_FINAL)
  message(STATUS "Unitree headers: ${UNITREE_INCLUDE_FINAL}")
  target_include_directories(abdullah_elnahrawy_g1_motions PRIVATE ${UNITREE_INCLUDE_FINAL})
  target_include_directories(voice_interface              PRIVATE ${UNITREE_INCLUDE_FINAL})
endif()
# ----------------------------------------------------------------

# SDK-based code paths
target_compile_definitions(voice_interface PRIVATE ENABLE_UNITREE_SDK)
target_compile_definitions(abdullah_elnahrawy_g1_motions PRIVATE ENABLE_UNITREE_SDK)

# Hard-to-remove branding compiled into both binaries
foreach(tgt IN ITEMS abdullah_elnahrawy_g1_motions voice_interface)
  target_compile_definitions(${tgt} PRIVATE
    PROJECT_NAME="${PROJECT_NAME_STR}"
    PROJECT_AUTHOR="${PROJECT_AUTHOR_STR}"
    PROJECT_EMAIL="${PROJECT_EMAIL_STR}"
    PROJECT_GITHUB="${PROJECT_GITHUB_STR}"
    PROJECT_SEMVER="${PROJECT_SEMVER}"
    PROJECT_BUILD_DATE="${BUILD_TS}"
    PROJECT_BUILD_HASH="${GIT_HASH}"
  )
endforeach()

# Link libraries
target_link_libraries(abdullah_elnahrawy_g1_motions PRIVATE
    audio_pipeline
    unitree_sdk2
    yaml-cpp
)
target_link_libraries(voice_interface PRIVATE
    audio_pipeline
    unitree_sdk2
    yaml-cpp
    nlohmann_json::nlohmann_json
    Boost::system
    OpenSSL::SSL
    OpenSSL::Crypto
    Threads::Threads
)

# Feature levels
target_compile_features(abdullah_elnahrawy_g1_motions PRIVATE cxx_std_17)
target_compile_features(voice_interface PRIVATE cxx_std_17)

# Install (optional)
include(GNUInstallDirs)
install(TARGETS abdullah_elnahrawy_g1_motions voice_interface
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES ${CMAKE_SOURCE_DIR}/config/motions_registry.yaml
        DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/config)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/motion_data/
        DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/motion_data)

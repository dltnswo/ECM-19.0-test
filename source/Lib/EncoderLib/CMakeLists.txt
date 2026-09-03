# library
set( LIB_NAME EncoderLib )

# get source files
file( GLOB SRC_FILES "*.cpp" )

# get include files
file( GLOB INC_FILES "*.h" )

# NATVIS files for Visual Studio
if( MSVC )
  file( GLOB NATVIS_FILES "../../VisualStudio/*.natvis" )
endif()

if (EXTENSION_NNLF)
  include_directories(${PATH_TO_DIRECTORY_ROOT_SADL})
endif()

# library
add_library( ${LIB_NAME} STATIC ${SRC_FILES} ${INC_FILES} ${NATVIS_FILES} )
target_compile_definitions( ${LIB_NAME} PUBLIC )

if( EXTENSION_360_VIDEO )
  target_compile_definitions( ${LIB_NAME} PUBLIC EXTENSION_360_VIDEO=1 )
endif()

if( EXTENSION_HDRTOOLS )
  target_compile_definitions( ${LIB_NAME} PUBLIC EXTENSION_HDRTOOLS=1 )
endif()

if( DEFINED ENABLE_TRACING )
  if( ENABLE_TRACING )
    target_compile_definitions( ${LIB_NAME} PUBLIC ENABLE_TRACING=1 )
  else()
    target_compile_definitions( ${LIB_NAME} PUBLIC ENABLE_TRACING=0 )
  endif()
endif()

if( DEFINED ENABLE_HIGH_BITDEPTH )
  if( ENABLE_HIGH_BITDEPTH )
    target_compile_definitions( ${LIB_NAME} PUBLIC RExt__HIGH_BIT_DEPTH_SUPPORT=1 )
  else()
    target_compile_definitions( ${LIB_NAME} PUBLIC RExt__HIGH_BIT_DEPTH_SUPPORT=0 )
  endif()
endif()

target_include_directories( ${LIB_NAME} PUBLIC . )
target_link_libraries( ${LIB_NAME} CommonLib )

if( CMAKE_COMPILER_IS_GNUCC )
  # this is quite certainly a compiler problem
  set_property( SOURCE "EncCu.cpp" APPEND PROPERTY COMPILE_FLAGS "-Wno-array-bounds" )
endif()

if (NOT (CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64") )
  if( MSVC )
    set_property( SOURCE EncNNFilter.cpp APPEND PROPERTY COMPILE_FLAGS "/arch:AVX2 -DNDEBUG=1 ")
    set_property( SOURCE EncNNFilterUnified.cpp.cpp APPEND PROPERTY COMPILE_FLAGS "/arch:AVX2 -DNDEBUG=1 ")
  elseif( UNIX OR MINGW )
    if( NNLF_BUILD_WITH_AVX512 STREQUAL "1" )
      set_property( SOURCE EncNNFilter.cpp APPEND PROPERTY COMPILE_FLAGS "-DNDEBUG=1 -mavx512f -mavx512bw -ffast-math")
      set_property( SOURCE EncNNFilterUnified.cpp APPEND PROPERTY COMPILE_FLAGS "-DNDEBUG=1 -mavx512f -mavx512bw -ffast-math")
    else()
      set_property( SOURCE EncNNFilterUnified.cpp APPEND PROPERTY COMPILE_FLAGS "-DNDEBUG=1 -mavx2 -ffast-math")
    endif()
  endif()
endif()

# example: place header files in different folders
source_group( "Natvis Files" FILES ${NATVIS_FILES} )

# set the folder where to place the projects
set_target_properties( ${LIB_NAME} PROPERTIES FOLDER lib )

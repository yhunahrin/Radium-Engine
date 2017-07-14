set(app_target bbwcomputer)

# ///////////////////////////////


# BBW Computation depends on IGL and tetgen
set(IGL_DIR "${RADIUM_ROOT_DIR}/../igl/libigl/include")
set(TETGEN_DIR "${RADIUM_ROOT_DIR}/../tetgen/tetgen1.5.0")
set(TETGEN_LIB "${TETGEN_DIR}/build/libtet.a")

# Access to Radium headers and declarations/defintions
include_directories(
    .
    ${RADIUM_INCLUDE_DIR}
    ${IGL_DIR}
    ${TETGEN_DIR}
)

add_definitions(-DTETLIBRARY)
# Get files
file( GLOB file_sources *.cpp *.c )
file( GLOB file_headers *.hpp *.h )


# Generate an executable
add_executable( ${app_target} ${file_sources} ${file_headers} )

add_dependencies( ${app_target} radiumEngine radiumGuiBase radiumCore radium_assets)


# Link good libraries
target_link_libraries( ${app_target} # target
    ${RA_CORE_LIB}                # Radium libs
    ${RA_ENGINE_LIB}              # Radium libs
    ${GLBINDING_LIBRARIES}           # Radium dep
    ${TETGEN_LIB}               # the tetgen library
)

if (MSVC)
    set_property( TARGET ${app_target} PROPERTY IMPORTED_LOCATION "${RADIUM_SUBMODULES_INSTALL_DIRECTORY}/bin" )
endif(MSVC)

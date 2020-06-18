#########################################
# Download external dependencies.
#########################################
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/modules/ ${CMAKE_MODULE_PATH})
include(FetchContent)
cmake_policy(SET CMP0079 NEW)

# @todo - allow people to provide their own thrust/cub/jitify rather than downloading (or prevent downloading for metered connections?) Not sure how to best mask this out.
# Possibly: -DDOWNLOAD_DEPENDENCIES=ON/OFF (ON by default.). 
    # IF ON, Download, then run FindPackage looking at the downloaded location, which should fallback to system installs
    # If OFF, Run find, hinted to look at the expected location of externals/wherever.

# @todo - output messages to show that files are being downloaded and not that cmake has broken?

# @todo - move these to clean directories within the build dir, rather than _deps/thrust-source etc.

##############################
# Thrust (and CUB) >= 1.9.10 #
##############################
FetchContent_Declare(
    thrust
    GIT_REPOSITORY https://github.com/thrust/thrust.git
    GIT_TAG        1.9.10
    GIT_SHALLOW    1
)
FetchContent_GetProperties(thrust)
if(NOT thrust_POPULATED)
message("dl thrust?")
    FetchContent_Populate(thrust)   
    # Thrust >= 1.9.10 contains thrust/thrust/cmake/thrust-config.cmake so we can use find_package to find thrust, but not compile the tests.
    set(Thrust_DIR "${thrust_SOURCE_DIR}/thrust/cmake")
    find_package(Thrust REQUIRED CONFIG)
    # Thrust >= 1.9.8 ships with actual CUB, rather than a modified version, so we can just use that.
    # @todo - find cub's location from Thrust, incase it moves. Alternatively this is not requires as thrusts include paths include cubs.
    set(CUB_DIR "${thrust_SOURCE_DIR}/cub/cmake")
    find_package(CUB REQUIRED CONFIG)
endif()

##########
# Jitify #
##########
FetchContent_Declare(
    jitify
    PREFIX ${CMAKE_BINARY_DIR}/jitify
    GIT_REPOSITORY https://github.com/NVIDIA/jitify.git
    GIT_TAG        2a015bb
    GIT_SHALLOW    0
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
)
FetchContent_GetProperties(jitify)
if(NOT jitify_POPULATED)
    FetchContent_Populate(jitify)
    # Jitify is not a cmake project, so cannot use add_subdirectory, use custom find_package.
    set(Jitify_ROOT "${jitify_SOURCE_DIR}")
    find_package(Jitify REQUIRED)
    # Include path is ${Jitify_INCLUDE_DIRS}
endif()

############
# Tinyxml2 #
############

# @todo - fetch tinyxml2

# 6.2.0 is the previously used version, 8.0.0 available as of march 2019
# FetchContent_Declare(
#     tinyxml2
#     GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git
#     GIT_TAG        8.0.0
#     GIT_SHALLOW    1
# )
# FetchContent_GetProperties(tinyxml2)
# if(NOT tinyxml2_POPULATED)
#     cmake_policy(SET CMP0063 NEW)
#     FetchContent_Populate(tinyxml2)
#     message("tinyxml2_POPULATED ${tinyxml2_POPULATED}")
#     message("tinyxml2_BINARY_DIR ${tinyxml2_BINARY_DIR}")
#     message("tinyxml2_SOURCE_DIR ${tinyxml2_SOURCE_DIR}")
    
#     add_subdirectory(${tinyxml2_SOURCE_DIR} ${tinyxml2_BINARY_DIR})

#    if(TARGET tinyxml2)
#       message("TINYXML2_LIBRARY tinyxml2")
#     elseif(TARGET tinyxml2::tinyxml2)
#       message("TINYXML2_LIBRARY tinyxml2::tinyxml2")
#     endif()

#     # set(TinyXML2_DIR "")
#     find_package(TinyXML2 CONFIG REQUIRED)

# endif()

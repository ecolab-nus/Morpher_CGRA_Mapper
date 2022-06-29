set(CMAKE_C_COMPILER "gcc-7")
set(CMAKE_CXX_COMPILER "/usr/bin/g++-7")

FILE(GLOB_RECURSE MyCPPSources *.cpp)
FILE(GLOB_RECURSE MyHSources *.h)

set(PROJ "cgra_xml_mapper")

# set(CMAKE_BUILD_TYPE "Release")
find_package(nlohmann_json CONFIG REQUIRED)

add_executable(${PROJ} ${MyCPPSources} ${MyHSources})
# add_subdirectory(src)

# target_compile_features(${TARGET_NAME}  cxx_std_17)


# Use C++11 to compile our pass (i.e., supply -std=c++11).
target_compile_features(${PROJ} PRIVATE cxx_range_for cxx_auto_type )
target_compile_features(${PROJ} PRIVATE cxx_std_17 )
target_link_libraries(${PROJ} nlohmann_json::nlohmann_json)



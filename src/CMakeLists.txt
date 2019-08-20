FILE(GLOB MyCPPSources *.cpp)
FILE(GLOB MyHSources *.h)

add_executable(cgra_xml_mapper ${MyCPPSources} ${MyHSources})

# Use C++11 to compile our pass (i.e., supply -std=c++11).
target_compile_features(cgra_xml_mapper PRIVATE cxx_range_for cxx_auto_type)

# Morpher_CGRA_Mapper
This mapper is to map the DFG gnerated by Morpher DFG Generator on the CGRA architecture. The mapper allows the user to customize the archtecture in a hierarchical way. Please find the examples in json_arch folder.

## Build dependencies:
This version requires nlohmann-json libraries. 

### JSON:
You should have installed this on the Morpher DFG Generator
https://blog.csdn.net/jiaken2660/article/details/105155257

    git clone https://github.com/nlohmann/json.git
    mkdir build
    cd build
    cmake ../
    make -j2
    sudo make install

## Build CGRA Mapper:

    cd Morpher_CGRA_Mapper
    git checkout stable
    mkdir build
    cd build
    cmake ..
    make all -j

## Examples:

running command: cgra_xml_mapper -d dfg.xml -j arch.json.

    ./build/src/cgra_xml_mapper -d ./applications/pedometer/pedometer_INNERMOST_LN1_PartPred_DFG.xml -j  json_arch/hycube_original.json 

*-verbose.json is the output of mapper, not the input architecture file.
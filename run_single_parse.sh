(time ./build/src/cgra_xml_mapper -d $1 -j json_arch/$2) |& tee $1_$2.compiler.trace

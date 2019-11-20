(time ./Release/CGRA_xml_compiler -d $1 -j json_arch/$2) |& tee $1_$2.compiler.trace

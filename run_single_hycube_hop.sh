(time ./Release/CGRA_xml_compiler -d $1 -j json_arch/hycube_original.json -h $2) |& tee $1_hops$2.compiler.trace

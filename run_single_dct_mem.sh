(time ./build_hop/src/cgra_xml_mapper -d dct/jpeg_fdct_islow_INNERMOST_LN2_PartPred_DFG.xml -j json_arch/dct_various_mem_archs/$2) |& tee $1_$2.compiler.trace

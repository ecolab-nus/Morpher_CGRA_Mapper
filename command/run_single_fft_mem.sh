(time ./build_hop/src/cgra_xml_mapper -d fix_fft_npb/fix_fft_INNERMOST_LN111_PartPred_DFG.xml -j json_arch/fft_various_mem_archs/$2) |& tee $1_$2.compiler.trace

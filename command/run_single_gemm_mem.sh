(time ./build_hop/src/cgra_xml_mapper -d gemm_nt/gemm_nt_INNERMOST_LN111_PartPred_DFG.xml -j json_arch/gemm_various_mem_archs/$2) |& tee $1_$2.compiler.trace

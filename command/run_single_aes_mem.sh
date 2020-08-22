(time ./Release/CGRA_xml_compiler -d aes_encrypt/encrypt_INNERMOST_LN1_PartPred_DFG.xml -j json_arch/aes_various_mem_archs/$2) |& tee $1_$2.compiler.trace

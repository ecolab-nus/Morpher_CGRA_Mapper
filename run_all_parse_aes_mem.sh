parallel -j 12 ./run_single_aes_mem.sh ::: aes_encrypt/encrypt_INNERMOST_LN1_PartPred_DFG.xml ::: \
 stdnoc_mem_4port_single_bank.json stdnoc_mem_dual_port_two_banked_bus.json stdnoc_mem_dual_port_two_banked_const_mem.json stdnoc_mem_dual_port_two_banked.json stdnoc_mem_single_port_four_banked.json

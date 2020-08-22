parallel -j 12 ./run_single_hycube_hop.sh ::: \
\
 adpcm_decoder/adpcm_decoder_INNERMOST_LN1_PartPred_DFG.xml\
 adpcm_coder/adpcm_coder_INNERMOST_LN1_PartPred_DFG.xml\
 gemm_nt/gemm_nt_INNERMOST_LN111_PartPred_DFG.xml\
 finalSAD/finalSAD_INNERMOST_LN11_PartPred_DFG.xml\
 dwt/dwpt_per_INNERMOST_LN11_PartPred_DFG.xml\
 dct/jpeg_fdct_islow_INNERMOST_LN2_PartPred_DFG.xml\
 compare_neighb/compare_neighb_INNERMOST_LN21_PartPred_DFG.xml\
 aes_encrypt/encrypt_INNERMOST_LN1_PartPred_DFG.xml\
 aes_decrypt/decrypt_INNERMOST_LN1_PartPred_DFG.xml\
\
 ::: \
4 3 2 1 

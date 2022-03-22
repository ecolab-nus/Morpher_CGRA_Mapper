/* // This is the main file for the abstract mapping in HyCUBE

#include <iostream>
#include <assert.h>
#include <string.h>

#include "DFG.h"
#include "CGRA.h"
#include "HeuristicMapper.h"
#include "PathFinderMapper.h"
#include <math.h>
#include <time.h>
#include "SimulatedAnnealingMapper.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

using namespace std;
using namespace CGRAXMLCompile;

string benchmarkAddr[] = {
	// small example with 9 nodes
	"../applications/cgra_me_bench/mac/main_INNERMOST_LN1_noSemiOLOAD_PartPred_DFG.xml", // 0:cgra_me_bench
	"../applications/pedometer/pedometer_INNERMOST_LN1_PartPred_DFG.xml",				 // 1:pedometer
	"../applications/dwt/dwpt_per_INNERMOST_LN11_FullPred_DFG.xml",						 // 2:dwt
	"../applications/adpcm_coder/adpcm_coder_INNERMOST_LN1_PartPred_DFG.xml",			 // 3:adpcm_coder
	"../applications/gemm_nt/gemm_nt_INNERMOST_LN111_PartPred_DFG.xml",					 // 4:gemm_nt
	"../applications/fix_fft_npb/fix_fft_INNERMOST_LN111_PartPred_DFG.xml",				 // 5:fix_fft_npb
	"../applications/dct/jpeg_fdct_islow_INNERMOST_LN2_PartPred_DFG.xml",				 // 6:dct
	"../applications/adpcm_decoder/adpcm_decoder_INNERMOST_LN1_FullPred_DFG.xml",		 // 7:adpcm_decoder
	"../applications/taylor/Taylor_App_INNERMOST_LN21_FullPred_DFG.xml",				 // 8:taylor
	"../applications/aes_decrypt/decrypt_INNERMOST_LN1_PartPred_DFG.xml",				 // 9:aes_decrypt
	"../applications/aes_encrypt/encrypt_INNERMOST_LN1_PartPred_DFG.xml",				 // 10:aes_encrypt
	"../applications/array_add/array_add_INNERMOST_LN1_PartPred_DFG.xml"				 // 11:array_add

};

string benchmarkName[] = {
	"cgra_me_bench",
	"pedometer",
	"dwt",
	"adpcm_coder",
	"gemm_nt",
	"fix_fft_npb",
	"dct",
	"adpcm_decoder",
	"taylor",
	"aes_decrypt",
	"aes_encrypt"
	"array_add"

};

static string arch_json_file = "../json_arch/hycube_original.json";

static int mapping_method = 2;

int main(int argn, char *argc[])
{
	cout << "Begining of the main function..." << endl;
	int x_dim = 4, y_dim = 4;
	int maxII = 100;

	int i = 11;
	string inputDFGFilename = benchmarkAddr[i];
	cout << inputDFGFilename << endl;

	clock_t start_time, end_time;
	start_time = clock();

	// step 1: get the minimum II
	DFG currDFG;
	currDFG.parseXML(inputDFGFilename);
	currDFG.printDFG();

	// CGRA *cgra = new CGRA(NULL, "coreCGRA", 1, x_dim, y_dim, &currDFG, "HyCUBE_4REGF", 1);

	PathFinderMapper *mapper = new SAMapper(inputDFGFilename);
	SAMapper *sa_mapper = static_cast<SAMapper *>(mapper);

	CGRA *testcgra = new CGRA(arch_json_file, 1, x_dim, y_dim);
	int ii = sa_mapper->getAbstractII(testcgra, &currDFG);

	CGRA *cgra = new CGRA(arch_json_file, ii, x_dim, y_dim, mapper->getcongestedPortsPtr());

	bool mappingSuccess = sa_mapper->SAMap(cgra, &currDFG, 2);

	cout << "II=" << cgra->get_t_max() << endl;

	return 0;
}
 */
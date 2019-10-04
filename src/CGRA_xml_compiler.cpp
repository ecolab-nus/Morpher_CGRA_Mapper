//============================================================================
// Name        : CGRA_xml_compiler.cpp
// Author      : Manupa Karunaratne
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <assert.h>
#include <string.h>

#include "DFG.h"
#include "CGRA.h"
#include "HeuristicMapper.h"
#include "PathFinderMapper.h"
#include <math.h>

using namespace std;
using namespace CGRAXMLCompile;

int main(int argn, char *argc[])
{
	cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!

	if (argn < 7)
	{
		std::cout << "arguments : <DFG.xml> <peType::\nGENERIC_8REGF,\nHyCUBE_8REGF,\nHyCUBE_4REG,\nN2N_4REGF,\nN2N_8REGF,\nSTDNOC_8REGF,\nSTDNOC_4REGF,\nSTDNOC_4REG,\nSTDNOC_4REGF_1P\nMFU_HyCUBE_4REG\nMFU_HyCUBE_4REGF\nMFU_STDNOC_4REG\nMFU_STDNOC_4REGF> <XYDim> <numberofDPS> <backtracklimit> <initII> <-arch_json> <-noMTpath>\n";
	}

	assert(argn >= 7);
	std::string inputDFG_filename(argc[1]);

	//	int xdim=4;
	//	int ydim=4;

	int xydim = atoi(argc[3]);
	int xdim = xydim / 10;
	int ydim = xydim % 10;

	DFG currDFG;
	currDFG.parseXML(inputDFG_filename);
	currDFG.printDFG();

	bool isGenericPE;
	std::string PEType(argc[2]);
	int numberOfDPs = atoi(argc[4]);

	string json_file_name;
	if(argn >= 8){
		json_file_name = argc[7];
	}

	// CGRA testCGRA(NULL, "testCGRA", 1, ydim, xdim, &currDFG, PEType, numberOfDPs);

	CGRA *testCGRA;
	if(json_file_name.empty()){
		testCGRA = new CGRA(NULL, "coreCGRA", 1, ydim, xdim, &currDFG, PEType, numberOfDPs);
	}
	else{
		testCGRA = new CGRA(json_file_name,1);
	}

	//	HeuristicMapper hm(inputDFG_filename);
	PathFinderMapper hm(inputDFG_filename);

	int II = hm.getMinimumII(testCGRA, &currDFG);
	std::cout << "Minimum II = " << II << "\n";

	int initUserII = atoi(argc[6]);
	II = std::max(initUserII, II);

	std::cout << "Using II = " << II << "\n";

	hm.enableMutexPaths = true;
	if (argn == 9)
	{
		std::string noMutexPathStr(argc[8]);
		if (noMutexPathStr == "-noMTpath")
		{
			hm.enableMutexPaths = false;
		}
	}
	hm.enableBackTracking = true;
	hm.backTrackLimit = atoi(argc[5]);


	cout << "json_file_name = " << json_file_name << "\n";
	// exit(EXIT_SUCCESS);

	bool mappingSuccess = false;
	while (!mappingSuccess)
	{
		DFG tempDFG;
		tempDFG.parseXML(inputDFG_filename);
		tempDFG.printDFG();

		CGRA *tempCGRA;
		if(json_file_name.empty()){
			tempCGRA = new CGRA(NULL, "coreCGRA", II, ydim, xdim, &tempDFG, PEType, numberOfDPs, hm.getcongestedPortsPtr());
		}
		else{
			tempCGRA = new CGRA(json_file_name,II,hm.getcongestedPortsPtr());
		}


		hm.getcongestedPortsPtr()->clear();
		hm.getconflictedPortsPtr()->clear();
		mappingSuccess = hm.Map(tempCGRA, &tempDFG);
		hm.congestionInfoFile.close();
		if (!mappingSuccess)
		{

			for (DFGNode &node : currDFG.nodeList)
			{
				assert(node.rootDP == NULL);
			}

			delete tempCGRA;
			II++;

			if (II == 65)
			{
				std::cout << "II max of 32 has been reached and exiting...\n";
				return 0;
			}

			if (II > hm.upperboundII)
			{
				std::cout << "upperbound II reached : " << hm.upperboundII << "\n";
				std::cout << "Please use the mapping with II = " << hm.upperboundFoundBy << ",with Iter = " << hm.upperboundIter << "\n";
				//return 0;
			}

			std::cout << "Increasing II to " << II << "\n";
		}
		else
		{
			hm.sanityCheck();
		}
	}
	return 0;
}

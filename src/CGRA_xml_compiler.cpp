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
#include <math.h>

using namespace std;
using namespace CGRAXMLCompile;


int main(int argn, char* argc[]) {
	cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!

	if(argn != 6){
		std::cout << "arguments : <DFG.xml> <peType:GENERIC_8REGF,HyCUBE_8REGF,HyCUBE_4REG,N2N_4REGF,N2N_8REGF,STDNOC_8REGF,STDNOC_4REGF,STDNOC_4REG> <numberofDPS> <backtracklimit> <initII>\n";
	}

	assert(argn == 6);
	std::string inputDFG_filename (argc[1]);

	int xdim=4;
	int ydim=4;

	DFG currDFG;
	currDFG.parseXML(inputDFG_filename);
	currDFG.printDFG();

	bool isGenericPE;
	std::string PEType(argc[2]);
	int numberOfDPs=atoi(argc[3]);

	CGRA testCGRA(NULL,"testCGRA",1,ydim,xdim,PEType,numberOfDPs);

	HeuristicMapper hm(inputDFG_filename);
	int II = hm.getMinimumII(&testCGRA,&currDFG);
	std::cout << "Minimum II = " << II << "\n";

	int initUserII = atoi(argc[5]);
	II = std::max(initUserII,II);

	hm.enableBackTracking=true;
	hm.enableMutexPaths=true;
	hm.backTrackLimit=atoi(argc[4]);


	bool mappingSuccess=false;
	while(!mappingSuccess){
		DFG tempDFG;
		tempDFG.parseXML(inputDFG_filename);
		tempDFG.printDFG();
		CGRA* tempCGRA = new CGRA(NULL,"coreCGRA",II,ydim,xdim,PEType,numberOfDPs);
		mappingSuccess = hm.Map(tempCGRA,&tempDFG);
		if(!mappingSuccess){

			for(DFGNode& node : currDFG.nodeList){
				assert(node.rootDP==NULL);
			}

			delete tempCGRA;
			II++;
			std::cout << "Increasing II to " << II << "\n";
		}
	}
	return 0;
}

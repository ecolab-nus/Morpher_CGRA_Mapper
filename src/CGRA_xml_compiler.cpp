//============================================================================
// Name        : CGRA_xml_compiler.cpp
// Author      : Manupa Karunaratne
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <chrono>

#include <assert.h>
#include <string.h>
// 
#include <morpher/util/util.h>
#include <morpher/dfg/DFG.h>
#include <morpher/arch/CGRA.h>
#include <morpher/mapper/HeuristicMapper.h>
#include <morpher/mapper/PathFinderMapper.h>
#include <morpher/mapper/SimulatedAnnealingMapper.h>
#include <morpher/lisa/LISAMapper.h>
#include <math.h>

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

using namespace std;
using namespace CGRAXMLCompile;


int main(int argn, char *argc[])
{
	//cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!

	// if (argn < 7)
	// {
	// 	std::cout << "arguments : <DFG.xml> <peType::\nGENERIC_8REGF,\nHyCUBE_8REGF,\nHyCUBE_4REG,\nN2N_4REGF,\nN2N_8REGF,\nSTDNOC_8REGF,\nSTDNOC_4REGF,\nSTDNOC_4REG,\nSTDNOC_4REGF_1P\nMFU_HyCUBE_4REG\nMFU_HyCUBE_4REGF\nMFU_STDNOC_4REG\nMFU_STDNOC_4REGF> <XYDim> <numberofDPS> <backtracklimit> <initII> <-arch_json> <-noMTpath>\n";
	// }
	// assert(argn >= 7);
	std::cout<<"Mapper arguments:\n";
	arguments args = parse_arguments(argn,argc);
	std::cout<<"\n";
	std::string inputDFG_filename = args.dfg_filename;
	int xdim = args.xdim;
	int ydim = args.ydim;
	string PEType = args.PEType;
	int numberOfDPs = args.ndps;
	string json_file_name = args.json_file_name;
	int initUserII = args.userII;
	int mapping_method = args.mapping_method;
	int max_II = args.max_II;
	std::string arch_name  = args.arch_name;

	
	DFG currDFG;
	currDFG.parseXML(inputDFG_filename, args.synthetic_dfg);
	currDFG.printDFG();

	bool isGenericPE;

	//std::cout<<"mapping method:"<<mapping_method<<"\n";

	//std::cout<<"architecture json file:"<<json_file_name<<"\n";

	// CGRA testCGRA(NULL, "testCGRA", 1, ydim, xdim, &currDFG, PEType, numberOfDPs);

	CGRA *testCGRA;
	if (!args.use_json)
	{
		testCGRA = new CGRA(NULL, "coreCGRA", 1, ydim, xdim, &currDFG, PEType, numberOfDPs);
	}
	else
	{
		testCGRA = new CGRA(json_file_name, 1,xdim,ydim);
	}

	

	//	HeuristicMapper mapper(inputDFG_filename);
	TimeDistInfo tdi = testCGRA->analyzeTimeDist();
	PathFinderMapper * mapper;
	if (mapping_method == 0){
		mapper = new PathFinderMapper(inputDFG_filename);
	}else if(mapping_method  == 1){
		mapper = new SAMapper(inputDFG_filename);
		
		// assert(false && "convert to SA");
	}else if(mapping_method  == 2){
		mapper = new LISAMapper(inputDFG_filename);
	}else{
		assert(false && "did not set a valid mapping method");
	}

	
	mapper->setMaxIter(args.maxiter);
	std::cout << "MAP begin... \n";
	int resII = mapper->getMinimumII(testCGRA, &currDFG);
	int recII  = mapper->getRecMinimumII(&currDFG);
	// int recII = 0; // for SA initial mapping test.
//	std::cout << "Res Minimum II = " << resII << ", ";
//	std::cout << "Rec Minimum II = " << recII << ", ";
//	std::cout << "Init User II = " << initUserII << "\n";
	int II = std::max(recII, resII);

	II = std::max(initUserII, II);

	std::cout << "Initial target II = " << II << " (ResMin II = "<<resII<<" , RecMin II = "<<recII<<", UserGiven II = "<<initUserII<<" )\n";

	mapper->enableMutexPaths = true;
	if (args.noMutexPaths)
	{
		mapper->enableMutexPaths = false;
	}
	mapper->enableBackTracking = true;
	mapper->backTrackLimit = args.backtracklimit;

	//cout << "json_file_name = " << json_file_name << "\n";
	// exit(EXIT_SUCCESS);

	auto start = chrono::steady_clock::now();

	bool mappingSuccess = false;
	while (!mappingSuccess)
	{
		DFG tempDFG;
		if(args.lisa_arg.training){
			tempDFG.dfg_parse_synthetic_dfg= true;
		}
		tempDFG.parseXML(inputDFG_filename);
		tempDFG.printDFG();

		CGRA *tempCGRA;
		if (json_file_name.empty())
		{
			tempCGRA = new CGRA(NULL, "coreCGRA", II, ydim, xdim, &tempDFG, PEType, numberOfDPs, mapper->getcongestedPortsPtr());
		}
		else
		{
			tempCGRA = new CGRA(json_file_name, II,xdim,ydim, mapper->getcongestedPortsPtr());
		}

	
		//std::cout << "Using II = " << II << "\n";
		// return 0;
		// tempCGRA->analyzeTimeDist(tdi);
		tempCGRA->max_hops = args.max_hops;

		// return 0;

		mapper->getcongestedPortsPtr()->clear();
		mapper->getconflictedPortsPtr()->clear();
		tempCGRA->analyzeTimeDist(tdi);
		// mappingSuccess = mapper->Map(tempCGRA, &tempDFG);
		if (mapping_method == 0){
			mappingSuccess = mapper->Map(tempCGRA, &tempDFG);
		}else if(mapping_method  == 1){
			SAMapper * sa_mapper = static_cast<SAMapper*>(mapper);
			mappingSuccess = sa_mapper->SAMap(tempCGRA, &tempDFG);
		}else if(mapping_method  == 2){
			LISAMapper * lisa_mapper = static_cast<LISAMapper*>(mapper);
			mappingSuccess = lisa_mapper->LISAMap(  args, tdi, II);
			if(args.lisa_arg.dfg_id != "none"){
				// this is genreatring traning data. No need to dump results;
				return 0;
			}
		}else{
			assert(false && "did not set a valid mapping method");
		}

		mapper->congestionInfoFile.close();
		if (!mappingSuccess)
		{

			for (DFGNode &node : currDFG.nodeList)
			{
				assert(node.rootDP == NULL);
			}

			delete tempCGRA;
			II++;

			if (II == max_II)
			{
				std::cout << "### cannot map:  II max has been reached and exiting...\n";
				break;
			}

			if (II > mapper->upperboundII)
			{
				std::cout << "upperbound II reached : " << mapper->upperboundII << "\n";
				std::cout << "Please use the mapping with II = " << mapper->upperboundFoundBy << ",with Iter = " << mapper->upperboundIter << "\n";
				break;
			}

			std::cout << "Increasing II to " << II << "\n";
		}
		else
		{
			std::ofstream mapped_ii_file;
			mapped_ii_file.open("mapped_ii.txt", std::ios_base::trunc); 
			mapped_ii_file << II <<"\n";
			mapped_ii_file.close();

			std::cout << "Map Success with II = "<< II <<"!!!\n";
			if(mapping_method  == 2){
				break;
			}
			mapper->sanityCheck();
			//mapper.assignLiveInOutAddr(&tempDFG);
			if(PEType == "HyCUBE_4REG"){
				std::cout << "\nPrinting HyCUBE Binary...\n";
				mapper->printHyCUBEBinary(tempCGRA);
			}
			break;
			
		}
	}

	auto end = chrono::steady_clock::now();
	std::cout << "Elapsed time in seconds: " << chrono::duration_cast<chrono::seconds>(end - start).count() << " sec\n";
	std::ofstream result_file;
	result_file.open ("result.txt", std::ios_base::app); 
	result_file<< arch_name <<" "<<inputDFG_filename
		<<" method:"<<mapper->getMappingMethodName()<<" "<<II<<" "
		<<chrono::duration_cast<chrono::seconds>(end - start).count()<<"\n";
	result_file.close();
	 

	return 0;
}

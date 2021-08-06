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

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

using namespace std;
using namespace CGRAXMLCompile;

struct arguments
{
	string dfg_filename;
	int xdim = -1;
	int ydim = -1;
	string PEType;
	string json_file_name;
	int userII = 0;
	bool noMutexPaths=false;
	int backtracklimit = 0;
	bool use_json = false;
	int ndps = 1;
	int maxiter = 30;
	int max_hops = 4;
};

arguments parse_arguments(int argn, char *argc[])
{
	arguments ret;

	int aflag = 0;
	int bflag = 0;
	char *cvalue = NULL;
	int index;
	int c;

	opterr = 0;

	while ((c = getopt(argn, argc, "d:x:y:t:j:i:eb:m:h:")) != -1)
		switch (c)
		{
		case 'd':
			ret.dfg_filename = string(optarg);
			break;
		case 'x':
			ret.xdim = atoi(optarg);
			break;
		case 'y':
			ret.ydim = atoi(optarg);
			break;
		case 't':
			ret.PEType = string(optarg);
			break;
		case 'n':
			ret.ndps = atoi(optarg);
			break;
		case 'j':
			ret.json_file_name = string(optarg);
			ret.use_json = true;
			break;
		case 'i':
			ret.userII = atoi(optarg);
			break;
		case 'e':
			ret.noMutexPaths = true;
			break;
		case 'b':
			ret.backtracklimit = atoi(optarg);
			break;
		case 'm':
			ret.maxiter = atoi(optarg);
			break;
		case 'h':
			ret.max_hops = atoi(optarg);
			break;
		case '?':
			if (optopt == 'c')
				fprintf(stderr, "Option -%c requires an argument.\n", optopt);
			else if (isprint(optopt))
				fprintf(stderr, "Unknown option `-%c'.\n", optopt);
			else
				fprintf(stderr,
						"Unknown option character `\\x%x'.\n",
						optopt);
		default:
			abort();
		}
		return ret;
}

struct port_edge
{
	Port* a;
	Port* b;
	bool operator==(const port_edge &other) const
	{
		
		return other.a == a && other.b == b;
	}

	bool operator<(const port_edge &other) const
	{
		
		return a < other.a || b < other.b;
	}
};

void find_routing_resource(Module * md, std::set<Port*> & ports, std::set<port_edge> & port_edges){

		// std::cout<<"vist "<<md->getFullName()<<"\n";
	for (auto & port_conn: md->getconnectedTo()){
		auto master_port = port_conn.first;
		ports.insert(master_port);
		for(auto slave_port: port_conn.second ){
			ports.insert(slave_port);
			port_edges.insert(port_edge{master_port, slave_port});
		}
	}

	for (auto & port_conn: md->getconnectedFrom()){
		auto slave_port = port_conn.first;
		ports.insert(slave_port);
		for(auto master_port: port_conn.second ){
			ports.insert(slave_port);
			port_edges.insert(port_edge{master_port, slave_port});
		}
	}

	for(auto submod: md->subModules){
		find_routing_resource(submod, ports, port_edges);
	}

}

int main(int argn, char *argc[])
{
	cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!

	// if (argn < 7)
	// {
	// 	std::cout << "arguments : <DFG.xml> <peType::\nGENERIC_8REGF,\nHyCUBE_8REGF,\nHyCUBE_4REG,\nN2N_4REGF,\nN2N_8REGF,\nSTDNOC_8REGF,\nSTDNOC_4REGF,\nSTDNOC_4REG,\nSTDNOC_4REGF_1P\nMFU_HyCUBE_4REG\nMFU_HyCUBE_4REGF\nMFU_STDNOC_4REG\nMFU_STDNOC_4REGF> <XYDim> <numberofDPS> <backtracklimit> <initII> <-arch_json> <-noMTpath>\n";
	// }
	// assert(argn >= 7);

	arguments args = parse_arguments(argn,argc);
	std::string inputDFG_filename = args.dfg_filename;
	int xdim = args.xdim;
	int ydim = args.ydim;
	string PEType = args.PEType;
	int numberOfDPs = args.ndps;
	string json_file_name = args.json_file_name;
	int initUserII = args.userII;
	
	DFG currDFG;
	currDFG.parseXML(inputDFG_filename);
	//return 0;
	currDFG.printDFG();

	//exit(true);
	bool isGenericPE;

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
#ifdef NOTIMEDISTANCEFUNC
#else
	TimeDistInfo tdi = testCGRA->analyzeTimeDist();
#endif

#ifdef HIERARCHICAL
	TimeDistInfo tdi = testCGRA->analyzeTimeDist();
#endif
	//return 0;
	//	HeuristicMapper hm(inputDFG_filename);
	PathFinderMapper hm(inputDFG_filename);
	hm.setMaxIter(args.maxiter);

	int resII = hm.getMinimumII(testCGRA, &currDFG);
	int recII  = hm.getRecMinimumII(&currDFG);
	std::cout << "Res Minimum II = " << resII << "\n";
	std::cout << "Rec Minimum II = " << recII << "\n";
	std::cout << "Init User II = " << initUserII << "\n";
	int II = std::max(recII, resII);

	II = std::max(initUserII, II);

	std::cout << "Using II = " << II << "\n";
//exit(true);
	hm.enableMutexPaths = true;
	if (args.noMutexPaths)
	{
		hm.enableMutexPaths = false;
	}
	hm.enableBackTracking = true;
	hm.backTrackLimit = args.backtracklimit;

	cout << "json_file_name = " << json_file_name << "\n";
	// exit(EXIT_SUCCESS);

	bool mappingSuccess = false;
	while (!mappingSuccess)
	{
		DFG tempDFG;
		tempDFG.parseXML(inputDFG_filename);
		tempDFG.printDFG();

		CGRA *tempCGRA;
		if (json_file_name.empty())
		{
			tempCGRA = new CGRA(NULL, "coreCGRA", II, ydim, xdim, &tempDFG, PEType, numberOfDPs, hm.getcongestedPortsPtr());
		}
		else
		{
			tempCGRA = new CGRA(json_file_name, II,xdim,ydim, hm.getcongestedPortsPtr());
		}

		std::set<Port*>  ports; std::set<port_edge>  port_edges;
		for(auto submod: tempCGRA->Name2SubMod){
		
			find_routing_resource(submod.second, ports, port_edges);
		}
		std::cout << "Using II = " << II << "\n";
		std::cout<<"number of ports: "<<ports.size()<<" number of edge: "<<port_edges.size();
		// return 0;
#ifdef NOTIMEDISTANCEFUNC
#else
		tempCGRA->analyzeTimeDist(tdi);
#endif

#ifdef HIERARCHICAL
		tempCGRA->analyzeTimeDist(tdi);
#endif

		tempCGRA->max_hops = args.max_hops;



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
				std::cout << "II max of 65 has been reached and exiting...\n";
				// return 0;
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
//#ifdef HIERARCHICAL
//#else
			hm.sanityCheck();
//#endif
			//hm.assignLiveInOutAddr(&tempDFG);
			if(PEType == "HyCUBE_4REG"){
				std::cout << "Printing HyCUBE Binary...\n";
				hm.printHyCUBEBinary(tempCGRA);
			}
			
		}
	}
	return 0;
}

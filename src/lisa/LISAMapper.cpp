#include "../PathFinderMapper.h"
#include "../SimulatedAnnealingMapper.h"
#include "LISAMapper.h"
#include "../DataPath.h"
#include "../FU.h"


#include <queue>
#include <assert.h>
#include <math.h>
#include <algorithm> // std::reverse
#include <stack>
#include <functional>
#include <set>
#include <iostream>
#include <sstream>
#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <bitset>

namespace CGRAXMLCompile
{

} /* namespace CGRAXMLCompile */



bool CGRAXMLCompile::LISAMapper::LISAMap(CGRA *cgra, DFG *dfg)
{
	std::stack<DFGNode *> mappedNodes;
	std::stack<DFGNode *> unmappedNodes;
	std::map<DFGNode *, std::priority_queue<dest_with_cost>> estimatedRouteInfo;

	// Disable mutex paths to test pathfinder
	this->enableMutexPaths = true;

	check_parent_violation = false;

	this->cgra = cgra;
	this->dfg = dfg;

	Check_DFG_CGRA_Compatibility();

	if (cgra->is_spm_modelled)
	{
		UpdateVariableBaseAddr();
	}
	sortBackEdgePriorityASAP();

	std::string mappingLogFileName = fNameLog1 + cgra->getCGRAName() + "_MTP=" + std::to_string(enableMutexPaths);	// + ".mapping.csv";
	std::string mappingLog2FileName = fNameLog1 + cgra->getCGRAName() + "_MTP=" + std::to_string(enableMutexPaths); // + ".routeInfo.log";

	bool mapSuccess = false;

	std::string congestionInfoFileName = mappingLogFileName + ".congestion.info";
	cout << "Opening congestion file : " << congestionInfoFileName << "!\n";
	congestionInfoFile.open(congestionInfoFileName.c_str());
	assert(congestionInfoFile.is_open());

	std::string mappingLogFileName_withIter = mappingLogFileName + "_SA" + ".mapping.csv";
	std::string mappingLog2FileName_withIter = mappingLog2FileName + "_SA" + ".routeInfo.log";
	std::string mappingLog4FileName_withIter = mappingLogFileName + "_II=" + std::to_string(cgra->get_t_max()) + "_SA" + ".mappingwithlatency.txt";

	mappingLog.open(mappingLogFileName_withIter.c_str());
	mappingLog2.open(mappingLog2FileName_withIter.c_str());
	mappingLog4.open(mappingLog4FileName_withIter.c_str());

	cout << "Opening mapping csv file : " << mappingLogFileName_withIter << "\n";
	cout << "Opening routeInfo log file : " << mappingLog2FileName_withIter << "\n";

	assert(mappingLog.is_open());
	assert(mappingLog2.is_open());
	assert(mappingLog4.is_open());

	while (!mappedNodes.empty())
	{
		mappedNodes.pop();
	}
	while (!unmappedNodes.empty())
	{
		unmappedNodes.pop();
	}

	for (DFGNode *node : sortedNodeList)
	{
		unmappedNodes.push(node);
	}

	std::cout << "*******************************************************SA MAP begin***************************\n";

	data_routing_path.clear();
	dfg_node_placement.clear();
	//initial mapping
	if (!initMap())
	{
		assert(false && "how come?");
		std::cout << "cannot find an initial mapping, exit....\n";
		return false;
	}

	//get mapping information
	
	std::stringstream congestion_detail;
	int unmapped_node_numer = getNumberOfUnmappedNodes();
	int overuse_number = getCongestionNumber(congestion_detail);
	int conflict_number =  getConflictNumber(congestion_detail);
	std::cout << "Initial mapping done. unmapped node:" << unmapped_node_numer << " overuse:" << overuse_number<< " conflict:" << conflict_number<<" cost:"<<getCost() << " \n";
	std::cout<<"unmapped node: \n";
	for (auto node : sortedNodeList)
	{
		if (dfg_node_placement.find(node) == dfg_node_placement.end())
		{
			std::cout<<"\t"<<node->idx<<"  "<<node->op<<"\n";
		}
	}

	LOG(ROUTE)<<congestion_detail.str();


	
	if (unmapped_node_numer == 0 && overuse_number == 0)
	{
		std::cout << "find a valid initial mapping, exit....II =" << this->cgra->get_t_max() << "\n";
		return true;
	}


	//start Simulated Annealing mapping
	std::cout << "maximum temperature:" << maximum_temp << " minimum temperature:" << minimim_temp << "\n";
	curr_cost = getCost();
	curr_temp = maximum_temp;

	std::cout<<"###############current mapping: \n"<<dumpMapping();

	while (curr_temp > minimim_temp)
	{
		std::cout << "*******************************current temperature:" << curr_temp << "\n";
		float accept_rate = inner_map();

		congestion_detail.clear();
		std::cout << "accept_rate:" << accept_rate << " # of overuse:" << getCongestionNumber(congestion_detail)<< " # of conflict:" << getConflictNumber(congestion_detail)
			<< " unmapped nodes:" << getNumberOfUnmappedNodes()<<" cost:"<<curr_cost << "\n";
		std::cout<<"unmapped node: \n";
		for (auto node : sortedNodeList)
		{
			if (dfg_node_placement.find(node) == dfg_node_placement.end())
			{
				std::cout<<"\t"<<node->idx<<"  "<<node->op<<"\n";
			}
		}
		LOG(ROUTE)<<congestion_detail.str();
		std::cout<<"###############current mapping: \n"<<dumpMapping();

		curr_temp = updateTemperature(curr_temp, accept_rate);

		if (isCurrMappingValid())
		{
			std::cout << "find a valid mapping, exit...II =" << this->cgra->get_t_max() << "\n";
			break;
		}
	}

	return isCurrMappingValid();
}


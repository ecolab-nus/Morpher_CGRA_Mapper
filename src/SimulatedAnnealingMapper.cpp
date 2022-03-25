#include "PathFinderMapper.h"

#include "SimulatedAnnealingMapper.h"
#include <queue>
#include <assert.h>
#include <math.h>
#include <algorithm> // std::reverse
#include "DataPath.h"
#include "FU.h"

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

/*
@TODO Verify
0. Initial mapping
1. Resotre mapping.
2. Parent and Child Node
3. Congestion

*/
/* Things migh need to be improved
1. When selected node to unmap, I select unmapped nodes first and unmap one of their parents.
	If no umapped nodes, I will randomly select one node.
	Will it be better if we randomly select from any node, or we just give a bit higher priority to unmapped nodes?
*/
bool CGRAXMLCompile::SAMapper::SAMap(CGRA *cgra, DFG *dfg, int iniMethod)
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
	if (iniMethod == 1)
	{
		if (!initMap())
		{
			std::cout << "cannot find an initial mapping with first iteration in stable mapping, exit....\n";
			return false;
		}
	}
	else if (iniMethod == 2)
	{
		if (!initMapWithAbsMap())
		{
			std::cout << "cannot find an initial mapping with abstract mapping, exit....\n";
			return false;
		}
	}
	else
	{
		assert(false && "invalid method for initial value");
	}

	int unmapped_node_numer = getNumberOfUnmappedNodes();
	int overuse_number = getCongestionNumber();
	std::cout << "Initial mapping done. unmapped node:" << unmapped_node_numer << " overuse:" << overuse_number << " \n";

	if (unmapped_node_numer == 0 && overuse_number == 0)
	{
		std::cout << "find a valid initial mapping, exit....\n";
		return true;
	}
	std::cout << "maximum temperature:" << maximum_temp << " minimum temperature:" << minimim_temp << "\n";
	curr_cost = getCost();
	curr_temp = maximum_temp;
	while (curr_temp > minimim_temp)
	{
		std::cout << "*******************************current temperature:" << curr_temp << "\n";
		float accept_rate = inner_map();
		std::cout << "accept_rate:" << accept_rate << " # of overuse:" << getCongestionNumber() << " unmapped nodes:" << getNumberOfUnmappedNodes() << "\n";
		curr_temp = updateTemperature(curr_temp, accept_rate);

		if (isCurrMappingValid())
		{
			std::cout << "find a valid mapping, exit....\n";
			break;
		}
	}

	return isCurrMappingValid();
}

void CGRAXMLCompile::SAMapper::getStringFromDFGAddr(DFG *dfg)
{
	void *ss = reinterpret_cast<void *>(dfg);
	std::stringstream ss1;
	ss1 << ss;
	dfgAddress = ss1.str();
}

void CGRAXMLCompile::SAMapper::genNodePlacement(CGRA *cgra, DFG *dfg)
{
	int resII = getMinimumII(cgra, dfg);
	int recII = getRecMinimumII(dfg);
	int minII = std::max(resII, recII);

	for (int ii = minII; ii < 100; ++ii) // maximum II = 100
	{
		dfg->resetMappingState();
		AMRRG *mrrg = new AMRRG(ii, cgra->get_x_max(), cgra->get_y_max(), dfg);
		mrrg->getTopoOrder();
		mrrg->printTopoOrder();
		bool mapSuccess = mrrg->abstractMap(dfg, dfgAddress);
		if (mapSuccess)
		{
			break;
		}
		delete mrrg;
	}
}

int CGRAXMLCompile::SAMapper::getAbstractII(CGRA *cgra, DFG *dfg)
{
	// get the dfg address as the placement file name
	getStringFromDFGAddr(dfg);
	// call the abstract mapping to generate the DFG-node placement (in build/placement.json file)
	genNodePlacement(cgra, dfg);

	// string placementFile = "../build/src/placement_" + dfgAddress + ".json";
	string placementFile = "./placement_" + dfgAddress + ".json";
	ifstream fin(placementFile);
	assert(fin.is_open());

	json j;
	fin >> j;

	return j["II"];
}

bool CGRAXMLCompile::SAMapper::getDPFromPE(int t, int x, int y, DataPath **retDP)
{
	auto it = this->cgra->subModArr[t];
	for (Module *submod_pe : it)
	{
		if (PE *pe = dynamic_cast<PE *>(submod_pe))
		{
			if (pe->X == x && pe->Y == y)
			{
				for (Module *submod_fu : pe->subModules)
				{
					if (FU *fu = dynamic_cast<FU *>(submod_fu))
					{
						for (Module *submod_dp : fu->subModules)
						{
							if (DataPath *dp = dynamic_cast<DataPath *>(submod_dp))
							{
								assert(dp->getName() == "DP0");
								*retDP = dp;
								return true;
							}
						}
					}
				}
			}
		}
	}
	return false;
}

void CGRAXMLCompile::SAMapper::parsePlacement()
{
	// string placementFile = "../build/src/placement_" + dfgAddress + ".json";
	string placementFile = "./placement_" + dfgAddress + ".json";
	ifstream fin(placementFile);
	assert(fin.is_open());

	json j;
	fin >> j;

	for (DFGNode &node : this->dfg->nodeList)
	{
		string nodeIndex = "DFG_node_" + to_string(node.idx);

		int t = j[nodeIndex]["mapped_PE"]["t"];
		int x = j[nodeIndex]["mapped_PE"]["x"];
		int y = j[nodeIndex]["mapped_PE"]["y"];

		assert(this->cgra->subModArr.find(t) != this->cgra->subModArr.end());
		DataPath *retDP = nullptr;
		if (getDPFromPE(t, x, y, &retDP))
		{
			absNodePlacement[&node] = retDP;
		}
		else
		{
			cout << "getDPFromPE unsuccessfully... " << endl;
			assert(false);
		}
	}
	fin.close();
}

bool CGRAXMLCompile::SAMapper::initMapWithAbsMap()
{
	// parse the DFG placement from placement.json into a map absNodePlacement
	parsePlacement();

	// assign other nodes and route edges according to placement
	for (auto it = absNodePlacement.begin(); it != absNodePlacement.end(); ++it)
	{
		cout << "node " << it->first->idx << " mapped on " << it->second->getFullName() << ", ";
		bool route_succ = iniSARoute(it->first, it->second);
		if (!route_succ)
		{
			cout << "SARoute after abstract mapping " << BOLDRED << "unsuccessfully" << RESET << "\n";
		}
		else
		{
			cout << "SARoute after abstract mapping " << GREEN << "successfully" << RESET << "\n";
		}
	}
	return true;
}

bool CGRAXMLCompile::SAMapper::initMap()
{
	std::stack<DFGNode *> mappedNodes;
	std::stack<DFGNode *> unmappedNodes;
	std::map<DFGNode *, std::priority_queue<dest_with_cost>> estimatedRouteInfo;
	enableBackTracking = false;
	int backTrackLimit = 4;
	int backTrackCredits = 4;
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

	while (!unmappedNodes.empty())
	{

		DFGNode *node = unmappedNodes.top();
		unmappedNodes.pop();

		std::stringstream MapHeader;
		MapHeader << "current node = " << node->idx;
		MapHeader << ",op = " << node->op;
		MapHeader << ",unmapped nodes = " << unmappedNodes.size();
		MapHeader << ",mapped nodes = " << mappedNodes.size();
		MapHeader << ",freeMemNodes = " << cgra->freeMemNodes;
		MapHeader << ",unmappedMemNodes = " << dfg->unmappedMemOps;
		MapHeader << ",II = " << cgra->get_t_max();
		MapHeader << ",btCredits = " << backTrackCredits;

		// MapHeader << ",PEType = " << this->cgra->peType;
		// MapHeader << ",XDim = " << this->cgra->get_x_max();
		// MapHeader << ",YDim = " << this->cgra->get_y_max();
		// MapHeader << ",DPs = " << this->cgra->numberofDPs;

		MapHeader << ",CGRA=" << this->cgra->getCGRAName();
		MapHeader << ",MaxHops=" << this->cgra->max_hops;

		MapHeader << ",BB = " << node->BB;
		MapHeader << ",mutexPathEn = " << this->enableMutexPaths;
		MapHeader << "\n";

		std::cout << MapHeader.str();

		bool isEstRouteSucc = false;

		// fill the routing information
		if (estimatedRouteInfo.find(node) == estimatedRouteInfo.end())
		{
			// the routes are not estimated.

			DFGNode *failedNode;
			std::priority_queue<dest_with_cost> estimatedRoutes;
			isEstRouteSucc = estimateRouting(node, estimatedRoutes, &failedNode);
			if (isEstRouteSucc)
			{
				estimatedRouteInfo[node] = estimatedRoutes;
			}
		}

		bool isRouteSucc = false;
		DFGNode *failedNode = NULL;

		std::cout << "estimatedRouteInfo[node].size = " << estimatedRouteInfo[node].size() << "\n";
		mappingLog << "estimatedRouteInfo[node].size = " << estimatedRouteInfo[node].size() << "\n";
		if (!estimatedRouteInfo[node].empty())
		{
			isRouteSucc = Route(node, estimatedRouteInfo[node], &failedNode);
			if (!isRouteSucc)
			{
				LOG(ROUTE) << "route not successful!\n";
			}
			// else if(!estimatedRouteInfo[node].empty()){
			// 	//route not successful
			// }
		}
		else
		{
			isRouteSucc = false;
		}

		if (!isRouteSucc)
		{
			std::cout << "----------node" << node->idx << " not mapped in initial mapping\n";

			return false;

			if (enableBackTracking) // always false
			{
				if (backTrackCredits == 0)
				{
					std::cout << "Map Failed!.\n";
					return false;
				}
				//					assert(failedNode!=NULL);
				backTrackCredits--;

				DFGNode *prevNode = mappedNodes.top();
				mappedNodes.pop();
				unmappedNodes.push(node);
				unmappedNodes.push(prevNode);

				prevNode->clear(this->dfg);
				estimatedRouteInfo.erase(node);
				continue;
			}
			else
			{

				node->clear(this->dfg);
				std::cout << "----------node" << node->idx << " not mapped in initial mapping\n";
			}
		}
		backTrackCredits = std::min(this->backTrackLimit, backTrackCredits + 1);
		mappedNodes.push(node);
	}

	return true;
}

float CGRAXMLCompile::SAMapper::inner_map()
{
	int accepted_number = 0;

	for (int i = 0; i < movement_in_each_temp; i++)
	{
		LOG(SA) << "************ NO." << i << " movement, unmapped nodes:" << getNumberOfUnmappedNodes() << ", congestion:" << getCongestionNumber();

		// select DFG node to unmap. If this node is not mapped yet, will select a parent to unmap as well.
		std::vector<DFGNode *> moved_nodes;
		auto selected_dfg_node = selectDFGNodeToUnmap();
		DFGNode *child_node = NULL;
		if (dfg_node_placement.find(selected_dfg_node) == dfg_node_placement.end())
		{
			// this node is not placed yet because of routing failure.
			// The reason for routing failure is that it cannot route to its parent nodes or recurrent dependency constraint;
			child_node = selected_dfg_node;
			selected_dfg_node = selectAParentForDFGNode(child_node);
			LOG(SA) << "----------be careful. Have a child node " << child_node->idx << "\n";
		}
		// assert(dfg_node_placement.find(selected_dfg_node)!= dfg_node_placement.end());
		std::map<DFGNode *, std::pair<DataPath *, int>> old_dfg_node_placement;
		old_dfg_node_placement.insert(dfg_node_placement.begin(), dfg_node_placement.end());
		std::map<dfg_data, std::vector<LatPort>> old_data_routing_path;
		old_data_routing_path.insert(data_routing_path.begin(), data_routing_path.end());

		// start map
		clearNodeMapping(selected_dfg_node);
		auto dp_candidate = getRandomDPCandidate(selected_dfg_node).front();
		LOG(SA) << "map selected DFG node:" << selected_dfg_node->idx << " op:" << selected_dfg_node->op << " current_pe:" << dp_candidate->getPE()->getName() << "\n";
		bool route_succ = SARoute(selected_dfg_node, dp_candidate);
		moved_nodes.push_back(selected_dfg_node);

		// map the unmapped parents of this child node
		if (route_succ && child_node != NULL && false)
		{
			// or skip this

			// find the unmapped nodes
			std::vector<DFGNode *> unmappedParents;
			for (auto parent : child_node->parents)
			{
				if (dfg_node_placement.find(parent) == dfg_node_placement.end())
				{
					unmappedParents.push_back(parent);
				}
			}

			for (auto parent : unmappedParents)
			{
				moved_nodes.push_back(parent);
				clearNodeMapping(parent);
				auto dp_candidate = getRandomDPCandidate(parent).front();
				LOG(SA) << "route other parent node = " << parent->idx << " op:" << parent->op << " current_pe:" << dp_candidate->getPE()->getName() << "\n";
				if (!SARoute(parent, dp_candidate))
				{
					route_succ = false;
					break;
				}
			}
		}

		// process the child node
		if (route_succ && child_node != NULL)
		{
			LOG(SA) << "----------be careful. Processing the child node = " << child_node->idx << "\n";
			std::priority_queue<dest_with_cost> estimatedRoutes;
			DFGNode *failedNode;
			bool isEstRouteSucc = estimateRouting(child_node, estimatedRoutes, &failedNode);
			bool isRouteSucc = false;
			if (isEstRouteSucc)
			{
				isRouteSucc = Route(child_node, estimatedRoutes, &failedNode);
			}
			if (!isRouteSucc)
			{
				clearNodeMapping(child_node);
			}
			moved_nodes.push_back(child_node);
		}

		// decide accept or node
		bool accept = false;
		int attempted_cost = getCost();
		if (route_succ && child_node != NULL && dfg_node_placement.find(child_node) != dfg_node_placement.end())
		{
			// if route success, has a child_node and successfully route
			accept = true;
		}
		else if (route_succ)
		{
			// if not route success, then nothing changes and should restore mapping
			accept = whetherAcceptNewMapping(attempted_cost, curr_cost, curr_temp);
		}

		LOG(SA) << "accept " << accept << " route_succ:" << route_succ << " curr_cost:" << curr_cost << " attepmted cost:" << attempted_cost << "\n";

		if (accept)
		{
			// update overuse
			//  save the mapping state
			curr_cost = attempted_cost;
			accepted_number++;

			old_dfg_node_placement.clear();
			old_data_routing_path.clear();
			if (isCurrMappingValid())
			{
				std::cout << "find a valid mapping, exit inner_map....\n";
				break;
			}
		}
		else
		{
			for (auto node : moved_nodes)
			{
				clearNodeMapping(node);
				restoreMapping(node, old_dfg_node_placement, old_data_routing_path);
			}

			dfg_node_placement.clear();
			dfg_node_placement.insert(old_dfg_node_placement.begin(), old_dfg_node_placement.end());
			data_routing_path.clear();
			data_routing_path.insert(old_data_routing_path.begin(), old_data_routing_path.end());
			old_dfg_node_placement.clear();
			old_data_routing_path.clear();
		}
	}
	return (float(accepted_number)) / movement_in_each_temp;
}

CGRAXMLCompile::DFGNode *CGRAXMLCompile::SAMapper::selectDFGNodeToUnmap()
{
	std::vector<DFGNode *> unmapped_node;
	for (auto node : sortedNodeList)
	{
		if (dfg_node_placement.find(node) == dfg_node_placement.end())
		{
			unmapped_node.push_back(node);
		}
	}
	std::random_device r;
	std::default_random_engine e1(r());

	if (unmapped_node.size() == 0)
	{
		std::uniform_int_distribution<int> uniform_dist(0, sortedNodeList.size() - 1);
		return sortedNodeList[std::abs(std::round(uniform_dist(e1)))];
	}
	else
	{
		std::uniform_int_distribution<int> uniform_dist(0, unmapped_node.size() - 1);
		return unmapped_node[std::abs(std::round(uniform_dist(e1)))];
	}
}

CGRAXMLCompile::DFGNode *CGRAXMLCompile::SAMapper::selectAParentForDFGNode(DFGNode *target_node)
{
	// the parent node include: nodes has data dependency, or ones which has recurrent dependency (load/Store constraint)
	std::vector<DFGNode *> candidates;
	candidates.insert(candidates.end(), target_node->parents.begin(), target_node->parents.end());
	for (auto node : sortedNodeList)
	{
		auto rec_parent = node->recParents;
		// as the rec_parent is reversed in DFG, the rec_parent means rec_child actually.
		if (std::find(rec_parent.begin(), rec_parent.end(), target_node) != rec_parent.end())
		{
			candidates.push_back(node);
		}
	}
	assert(candidates.size() > 0);
	std::random_device r;
	std::default_random_engine e1(r());
	std::uniform_int_distribution<int> uniform_dist(0, candidates.size() - 1);
	return candidates[std::abs(std::round(uniform_dist(e1)))];
}

bool CGRAXMLCompile::SAMapper::clearNodeMapping(DFGNode *node)
{
	LOG(SA) << "clear node =" << node->idx;
	bool mapped = (dfg_node_placement.find(node) != dfg_node_placement.end());
	if (mapped)
	{
		dfg_node_placement.erase(dfg_node_placement.find(node));
	}
	bool routed = false;
	for (auto it = data_routing_path.begin(); it != data_routing_path.end();)
	{
		// LOG(SA)<<"routing path:" << it->first.first->idx<<" -> "<<it->first.second->idx;
		if (it->first.first == node || it->first.second == node)
		{
			routed = true;
			it = data_routing_path.erase(it);
		}
		else
		{
			it++;
		}
	}
	// assert(mapped ==  routed);

	node->clear(this->dfg);
	return true;
}

bool CGRAXMLCompile::SAMapper::restoreMapping(DFGNode *node, std::map<DFGNode *, std::pair<DataPath *, int>> &dfg_node_placement,
											  std::map<dfg_data, std::vector<LatPort>> &data_routing_path)
{
	// assign placement
	if (dfg_node_placement.find(node) != dfg_node_placement.end())
	{
		auto placement_info = dfg_node_placement[node];
		placement_info.first->assignNode(node, placement_info.second, this->dfg);
		node->rootDP = placement_info.first;
	}

	// assign routing
	for (auto &data_path : data_routing_path)
	{
		if (data_path.first.first == node || data_path.first.second == node)
		{
			assignPath(data_path.first.first, data_path.first.second, data_path.second);
		}
	}

	return true;
}

int CGRAXMLCompile::SAMapper::getCongestionNumber()
{
	int congestion_number = 0;

	for (std::pair<Port *, std::set<DFGNode *>> pair : congestedPorts)
	{
		Port *p = pair.first;
		if (pair.second.size() > 1)
		{
			for (DFGNode *node1 : pair.second)
			{
				for (DFGNode *node2 : pair.second)
				{
					if (node1 == node2)
					{
						continue;
					}
					if (this->dfg->isMutexNodes(node1, node2, p))
						continue;
					LOG(CONGEST) << "CONGESTION:" << p->getFullName();
					for (DFGNode *node : pair.second)
					{
						LOG(CONGEST) << "," << node->idx << "|BB=" << node->BB;
					}
					LOG(CONGEST) << "\n";
					congestion_number++;
					//					break;
				}
			}
		}
	}
	return congestion_number;
}

int CGRAXMLCompile::SAMapper::getPortUsage()
{
	int usage_number = 0;

	for (std::pair<Port *, std::set<DFGNode *>> pair : congestedPorts)
	{
		usage_number += pair.second.size();
	}
	return usage_number;
}

int CGRAXMLCompile::SAMapper::getCost()
{
	int unmapped_node_number = getNumberOfUnmappedNodes();
	int port_usage = getPortUsage();
	int congestion_number = getCongestionNumber();
	int total_cost = unmapped_node_number * 100 + congestion_number * 10 + port_usage;
	LOG(COST) << "unmapped node:" << unmapped_node_number << " port usage:" << port_usage << " congestion:" << congestion_number << " total cost:" << total_cost << "\n";
	return total_cost;
}

bool CGRAXMLCompile::SAMapper::SARoute(DFGNode *node, DataPath *candidateDP)
{
	std::map<DFGNode *, std::vector<Port *>> parent_output_ports;
	std::map<DFGNode *, Port *> alreadyMappedChildPorts;

	int minLat = 0;
	int II = 0;

	// this part is from PathFinder
	{
		for (DFGNode *parent : node->parents)
		{
			//		std::cout << "parent = " << parent->idx << "\n";
			if (parent->rootDP != NULL)
			{ // already mapped
				assert(parent->rootDP->getOutputDP()->getOutPort("T"));
				parent_output_ports[parent].push_back(parent->rootDP->getOutputDP()->getOutPort("T"));
			}
		}

		for (DFGNode *child : node->children)
		{
			if (child->rootDP != NULL)
			{ // already mapped
				LOG(ROUTE) << "child=" << child->idx << ",childOpType=" << node->childrenOPType[child] << "\n";
				assert(child->rootDP->getLat() != -1);
				if (node->childrenOPType[child] == "PS")
				{
					LOG(ROUTE) << "Skipping.....\n";
					continue;
				}
				assert(child->rootDP->getInPort(node->childrenOPType[child]));
				alreadyMappedChildPorts[child] = child->rootDP->getInPort(node->childrenOPType[child]);

				int ii = child->rootDP->getCGRA()->get_t_max();
				assert(child->rootDP->getLat() != -1);
				alreadyMappedChildPorts[child]->setLat(child->rootDP->getLat() + ii); // next iteration?
			}
			else if (child->idx == node->idx)
			{
				// adding a placeholder as this will be modified according to the destination in consideration.
				alreadyMappedChildPorts[child] == NULL;
			}
		}

		minLat = getlatMinStartsPHI(node, parent_output_ports);
		PE *pe = candidateDP->getPE();
		CGRA *cgra = candidateDP->getCGRA();
		II = cgra->get_t_max();
		int t = pe->T;
		int minLatmodii = minLat % II;

		if (minLatmodii > t)
		{
			minLat += II + (t - minLatmodii);
		}
		else
		{
			minLat += t - minLatmodii;
		}
	}

	LOG(ROUTE) << "***routing destination:" << candidateDP->getFullName() << "\n";
	// check latency constraint before mapping
	bool route_success = true;
	for (int iteration = 0; iteration < maximum_routing_iteration; iteration++)
	{
		//
		LOG(ROUTE) << "routing iteration:" << iteration << "\n";
		int minLatDestVal_prime = minLat + II * iteration;

		LatPort candiLatport; // get a latency according to the iteration

		std::map<DFGNode *, std::vector<LatPort>> mappedChildPaths;

		FU *parentFU = candidateDP->getFU();
		assert(parentFU->supportedOPs.find(node->op) != parentFU->supportedOPs.end());
		int latency = parentFU->supportedOPs[node->op];
		int minLatDestVal = minLatDestVal_prime;
		// route parent nodes
		for (auto &parent_info : parent_output_ports)
		{
			auto parent = parent_info.first;
			minLatDestVal = minLatDestVal_prime + parent->childNextIter[node] * II;
			if (parent->getOPtype(node) == "PS")
			{
				LOG(ROUTE) << "Skipping.....\n";
				continue;
			}

			Port *destPort = candidateDP->getInPort(parent->getOPtype(node));
			LatPort destPortLat = std::make_pair(minLatDestVal, destPort);
			if (!canExitCurrPE(destPortLat))
			{
				LOG(ROUTE) << "cannot exit current PE. Continue.....\n";
				route_success = false;
				break;
			}
			auto startCand = parent_info.second.front();
			LatPort startCandLat = std::make_pair(startCand->getLat(), startCand);
			int cost;
			std::vector<LatPort> path;
			std::map<Port *, std::set<DFGNode *>> mutexPaths;
			route_success = LeastCostPathAstar(startCandLat, destPortLat, candidateDP, path, cost, parent, mutexPaths, node);

			if (route_success)
			{
				LOG(ROUTE) << "routing success from " << parent->idx << "," << startCand->getFullName() << "," << startCandLat.first << " to "
						   << node->idx << "," << destPort->getFullName() << "," << destPortLat.first << "\n";
				cout << "routing" << BOLDCYAN << " success" << RESET " from parent=" << parent->idx << ","
					 << startCand->getFullName() << "," << startCandLat.first << " to node=" << node->idx << ","
					 << destPort->getFullName() << "," << destPortLat.first << "\n";
				assignPath(parent, node, path);
				data_routing_path.emplace(std::make_pair(parent, node), path);
			}
			else
			{
				LOG(ROUTE) << "routing failure from " << parent->idx << "," << startCand->getFullName() << "," << startCandLat.first << " to "
						   << node->idx << "," << destPort->getFullName() << "," << destPortLat.first << "\n";
				cout << "routing" << BOLDRED << " failure" << RESET " from parent=" << parent->idx << ","
					 << startCand->getFullName() << "," << startCandLat.first << " to node="
					 << node->idx << "," << destPort->getFullName() << "," << destPortLat.first << "\n";
				break;
			}
		}

		if (!route_success)
		{
			node->clear(this->dfg);
			continue;
		}

		// route child nodes
		for (std::pair<DFGNode *, Port *> pair : alreadyMappedChildPorts)
		{
			DFGNode *child = pair.first;
			Port *childDestPort = pair.second;
			DataPath *childDP = child->rootDP;

			if (node->childrenOPType[child] == "PS")
			{
				LOG(ROUTE) << "Skipping.....\n";
				continue;
			}

			if (child->idx == node->idx)
			{
				childDestPort = candidateDP->getInPort(node->childrenOPType[child]);
				// if (detailedDebug) cout << "setting latency = " << minLatDestVal + ii << "\n";
				childDestPort->setLat(minLatDestVal + II);
				childDP = candidateDP;
			}

			std::vector<LatPort> path;
			int cost;

			FU *parentFU = candidateDP->getFU();
			assert(parentFU->supportedOPs.find(node->op) != parentFU->supportedOPs.end());
			int latency = parentFU->supportedOPs[node->op];
			Port *destPort = candidateDP->getOutputPort(latency);

			std::map<Port *, std::set<DFGNode *>> mutexPaths;
			childDestPort->setLat(childDP->getLat());
			LatPort childDestPortLat = std::make_pair(childDestPort->getLat(), childDestPort);
			assert(childDestPort->getLat() != -1);
			LatPort destPortLat = std::make_pair(minLatDestVal + latency, destPort);

			route_success = LeastCostPathAstar(destPortLat, childDestPortLat, childDP, path, cost, node, mutexPaths, child);

			if (route_success)
			{
				LOG(ROUTE) << "routing success from " << node->idx << ","
						   << destPort->getFullName() << "," << destPortLat.first << " to " << child->idx << ","
						   << childDestPort->getFullName() << "," << childDestPortLat.first << "\n";
				cout << "routing" << BOLDCYAN << " success" << RESET " from node=" << node->idx << ","
					 << destPort->getFullName() << "," << destPortLat.first << " to child=" << child->idx << ","
					 << childDestPort->getFullName() << "," << childDestPortLat.first << "\n";
				assignPath(node, child, path);
				data_routing_path.emplace(std::make_pair(node, child), path);
			}
			else
			{
				LOG(ROUTE) << "routing success from " << node->idx << ","
						   << destPort->getFullName() << "," << destPortLat.first << " to " << child->idx << ","
						   << childDestPort->getFullName() << "," << childDestPortLat.first << "\n";
				cout << "routing" << BOLDRED << " failure" << RESET " from node=" << node->idx << ","
					 << destPort->getFullName() << "," << destPortLat.first << " to child=" << child->idx << ","
					 << childDestPort->getFullName() << "," << childDestPortLat.first << "\n";
				break;
			}
		}
		if (!route_success)
		{
			node->clear(this->dfg);
			continue;
		}

		candidateDP->assignNode(node, minLatDestVal, this->dfg);
		dfg_node_placement.emplace(node, std::make_pair(candidateDP, minLatDestVal));
		node->rootDP = candidateDP;
		break;
	}
	return route_success;
}

bool CGRAXMLCompile::SAMapper::iniSARoute(DFGNode *node, DataPath *candidateDP)
{
	std::map<DFGNode *, std::vector<Port *>> parent_output_ports;
	std::map<DFGNode *, Port *> alreadyMappedChildPorts;

	int minLat = 0;
	int II = 0;

	// this part is from PathFinder
	{
		for (DFGNode *parent : node->parents)
		{
			if (parent->rootDP != NULL)
			{ // already mapped
				assert(parent->rootDP->getOutputDP()->getOutPort("T"));
				parent_output_ports[parent].push_back(parent->rootDP->getOutputDP()->getOutPort("T"));
			}
		}

		for (DFGNode *child : node->children)
		{
			if (child->rootDP != NULL)
			{ // already mapped
				LOG(ROUTE) << "child=" << child->idx << ",childOpType=" << node->childrenOPType[child] << "\n";
				assert(child->rootDP->getLat() != -1);
				if (node->childrenOPType[child] == "PS")
				{
					LOG(ROUTE) << "Skipping.....\n";
					continue;
				}
				assert(child->rootDP->getInPort(node->childrenOPType[child]));
				alreadyMappedChildPorts[child] = child->rootDP->getInPort(node->childrenOPType[child]);

				int ii = child->rootDP->getCGRA()->get_t_max();
				assert(child->rootDP->getLat() != -1);
				alreadyMappedChildPorts[child]->setLat(child->rootDP->getLat() + ii); // next iteration?
			}
			else if (child->idx == node->idx)
			{
				// adding a placeholder as this will be modified according to the destination in consideration.
				alreadyMappedChildPorts[child] = NULL;
			}
		}

		minLat = getlatMinStartsPHI(node, parent_output_ports);
		PE *pe = candidateDP->getPE();
		CGRA *cgra = candidateDP->getCGRA();
		II = cgra->get_t_max();
		int t = pe->T;
		int minLatmodii = minLat % II;

		if (minLatmodii > t)
		{
			minLat += II + (t - minLatmodii);
		}
		else
		{
			minLat += t - minLatmodii;
		}
	}

	LOG(ROUTE) << "***routing destination:" << candidateDP->getFullName() << "\n";
	// check latency constraint before mapping
	bool route_success = true;
	for (int iteration = 0; iteration < maximum_routing_iteration; iteration++)
	{
		LOG(ROUTE) << "routing iteration:" << iteration << "\n";
		int minLatDestVal_prime = minLat + II * iteration;

		LatPort candiLatport; // get a latency according to the iteration
		std::map<DFGNode *, std::vector<LatPort>> mappedChildPaths;

		// FU *parentFU = candidateDP->getFU();
		// assert(parentFU->supportedOPs.find(node->op) != parentFU->supportedOPs.end());
		// int latency = parentFU->supportedOPs[node->op];
		int minLatDestVal = minLatDestVal_prime;
		// route parent nodes
		for (auto &parent_info : parent_output_ports)
		{
			auto parent = parent_info.first;
			minLatDestVal = minLatDestVal_prime + parent->childNextIter[node] * II;
			if (parent->getOPtype(node) == "PS")
			{
				LOG(ROUTE) << "Skipping.....\n";
				continue;
			}

			Port *destPort = candidateDP->getInPort(parent->getOPtype(node));
			LatPort destPortLat = std::make_pair(minLatDestVal, destPort);
			if (!canExitCurrPE(destPortLat))
			{
				LOG(ROUTE) << "cannot exit current PE. Continue.....\n";
				route_success = false;
				break;
			}
			auto startCand = parent_info.second.front();
			LatPort startCandLat = std::make_pair(startCand->getLat(), startCand);
			int cost;
			std::vector<LatPort> path;
			std::map<Port *, std::set<DFGNode *>> mutexPaths;
			route_success = LeastCostPathAstar(startCandLat, destPortLat, candidateDP,
											   path, cost, parent, mutexPaths, node);

			if (route_success)
			{
				LOG(ROUTE) << "routing success from " << parent->idx << "," << startCand->getFullName()
						   << "," << startCandLat.first << " to " << node->idx << ","
						   << destPort->getFullName() << "," << destPortLat.first << "\n";
				cout << "routing" << BOLDCYAN << " success" << RESET " from parent=" << parent->idx << ","
					 << startCand->getFullName() << "," << startCandLat.first << " to node=" << node->idx << ","
					 << destPort->getFullName() << "," << destPortLat.first << "\n";
				assignPath(parent, node, path);
				data_routing_path.emplace(std::make_pair(parent, node), path);
			}
			else
			{
				LOG(ROUTE) << "routing failure from " << parent->idx << ","
						   << startCand->getFullName() << "," << startCandLat.first << " to "
						   << node->idx << "," << destPort->getFullName() << "," << destPortLat.first << "\n";
				cout << "routing" << BOLDRED << " failure" << RESET " from parent=" << parent->idx << ","
					 << startCand->getFullName() << "," << startCandLat.first << " to node="
					 << node->idx << "," << destPort->getFullName() << "," << destPortLat.first << "\n";
				break;
			}
		}

		if (!route_success)
		{
			node->clear(this->dfg);
			continue;
		}

		for (int iter = 0; iter < maximum_routing_iteration; iter++) // add by peng
		{
			bool childRouteSucc = true;
			// route child nodes
			for (std::pair<DFGNode *, Port *> pair : alreadyMappedChildPorts)
			{
				DFGNode *child = pair.first;
				Port *childDestPort = pair.second;
				DataPath *childDP = child->rootDP;

				if (node->childrenOPType[child] == "PS")
				{
					LOG(ROUTE) << "Skipping.....\n";
					continue;
				}

				if (child->idx == node->idx)
				{
					childDestPort = candidateDP->getInPort(node->childrenOPType[child]);
					childDestPort->setLat(minLatDestVal + II);
					childDP = candidateDP;
				}

				std::vector<LatPort> path;
				int cost;

				FU *parentFU = candidateDP->getFU();
				assert(parentFU->supportedOPs.find(node->op) != parentFU->supportedOPs.end());
				int latency = parentFU->supportedOPs[node->op];
				Port *destPort = candidateDP->getOutputPort(latency);

				std::map<Port *, std::set<DFGNode *>> mutexPaths;
				childDestPort->setLat(childDP->getLat());

				LatPort childDestPortLat = std::make_pair(childDestPort->getLat() + II * iter, childDestPort);
				assert(childDestPort->getLat() != -1);
				LatPort destPortLat = std::make_pair(minLatDestVal + latency, destPort);

				route_success = LeastCostPathAstar(destPortLat, childDestPortLat, childDP,
												   path, cost, node, mutexPaths, child);

				if (route_success)
				{
					LOG(ROUTE) << "routing success from " << node->idx << ","
							   << destPort->getFullName() << "," << destPortLat.first << " to " << child->idx << ","
							   << childDestPort->getFullName() << "," << childDestPortLat.first << "\n";
					cout << "routing" << BOLDCYAN << " success" << RESET " from node=" << node->idx << ","
						 << destPort->getFullName() << "," << destPortLat.first << " to child=" << child->idx << ","
						 << childDestPort->getFullName() << "," << childDestPortLat.first << "\n";
					assignPath(node, child, path);
					data_routing_path.emplace(std::make_pair(node, child), path);
				}
				else
				{
					LOG(ROUTE) << "routing failure from " << node->idx << ","
							   << destPort->getFullName() << "," << destPortLat.first << " to " << child->idx << ","
							   << childDestPort->getFullName() << "," << childDestPortLat.first << "\n";
					cout << "routing" << BOLDRED << " failure" << RESET " from node=" << node->idx << ","
						 << destPort->getFullName() << "," << destPortLat.first << " to child=" << child->idx << ","
						 << childDestPort->getFullName() << "," << childDestPortLat.first << "\n";
					// assert(node->idx != 13);
					childRouteSucc = false;
					break;
				}
			}
			if (childRouteSucc)
			{
				break;
			}
		}
		if (!route_success)
		{
			node->clear(this->dfg);
			continue;
		}

		candidateDP->assignNode(node, minLatDestVal, this->dfg);
		dfg_node_placement.emplace(node, std::make_pair(candidateDP, minLatDestVal));
		node->rootDP = candidateDP;
		break;
	}
	// if(route_success)
	return route_success;
}

std::vector<CGRAXMLCompile::DataPath *> CGRAXMLCompile::SAMapper::getRandomDPCandidate(DFGNode *node)
{
	std::vector<DataPath *> candidateDests;
	int penalty = 0;
	std::map<DataPath *, int> dpPenaltyMap;
	unordered_set<PE *> allPEs = cgra->getAllPEList();
	for (PE *currPE : allPEs)
	{
		for (Module *submod : currPE->subModules)
		{
			if (FU *fu = dynamic_cast<FU *>(submod))
			{

				if (fu->supportedOPs.find(node->op) == fu->supportedOPs.end())
				{
					continue;
				}

				if (fu->currOP.compare(node->op) == 0)
				{
					for (Module *submodFU : fu->subModules)
					{
						if (DataPath *dp = dynamic_cast<DataPath *>(submodFU))
						{

							if (cgra->is_spm_modelled)
							{
								if (!node->base_pointer_name.empty())
								{
									// base pointer name is not empty
									if (dp->accesible_memvars.find(node->base_pointer_name) == dp->accesible_memvars.end())
									{
										// this dp does not support the variable
										continue;
									}
								}
							}

							if (checkDPFree(dp, node, penalty))
							{
								//									if(dp->getMappedNode()==NULL){
								//									if(dataPathCheck(dp,&node)){

								if (node->blacklistDest.find(dp) == node->blacklistDest.end())
								{
									candidateDests.push_back(dp);
									dpPenaltyMap[dp] = penalty;
								}
							}
						}
					}
				}
				else if (fu->currOP.compare("NOP") == 0)
				{
					for (Module *submodFU : fu->subModules)
					{
						if (DataPath *dp = dynamic_cast<DataPath *>(submodFU))
						{

							bool is_mem_op = node->op.find("LOAD") != string::npos || node->op.find("STORE") != string::npos;
							if (cgra->is_spm_modelled)
							{
								if (!node->base_pointer_name.empty() && is_mem_op)
								{
									// base pointer name is not empty
									if (dp->accesible_memvars.find(node->base_pointer_name) == dp->accesible_memvars.end())
									{
										// this dp does not support the variable
										LOG(ROUTE) << "memvar=" << node->base_pointer_name << " is not supported in " << dp->getFullName() << "\n";
										continue;
									}
								}
							}

							if (checkDPFree(dp, node, penalty))
							{
								//									if(dp->getMappedNode()==NULL){
								//									if(dataPathCheck(dp,&node)){

								if (node->blacklistDest.find(dp) == node->blacklistDest.end())
								{
									candidateDests.push_back(dp);
									dpPenaltyMap[dp] = penalty;
								}
							}
						}
					}
				}
			}
		}
	}

	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

	shuffle(candidateDests.begin(), candidateDests.end(), std::default_random_engine(seed));

	return candidateDests;
}

// compated to the PathFinderMapper one, add some additional information for SA.
bool CGRAXMLCompile::SAMapper::Route(DFGNode *node,
									 std::priority_queue<dest_with_cost> &estimatedRoutes,
									 DFGNode **failedNode)
{

	LOG(ROUTE) << "Route begin...\n";

	int parentRoutingPortCount = 0;
	int routedParents = 0;

	for (DFGNode *parent : node->parents)
	{
		int thisParentNodeCount = 0;
		if (parent->rootDP != NULL)
		{
			thisParentNodeCount = parent->routingPorts.size();
		}

		//		if(thisParentNodeCount>0){
		//			routedParents++;
		//			thisParentNodeCount--; //remove the T port in the cout
		//		}
		parentRoutingPortCount += thisParentNodeCount;
	}
	//	if(parentRoutingPortCount>0){
	//		parentRoutingPortCount-=1; //remove the T port in the cout
	//	}

	int addedRoutingParentPorts = 0;

	bool routeSucc = false;
	dest_with_cost currDest;
	while (!estimatedRoutes.empty())
	{
		currDest = estimatedRoutes.top();
		estimatedRoutes.pop();

		if (currDest.dest->getMappedNode() != NULL)
		{
			LOG(ROUTE) << "currDest is not NULL \n";
			LOG(ROUTE) << "currDP:" << currDest.dest->getName() << ",currPE:" << currDest.dest->getPE()->getName() << "\n";
			LOG(ROUTE) << "currNode:" << currDest.dest->getMappedNode()->idx << "\n";
		}
		assert(currDest.dest->getMappedNode() == NULL);
		LOG(ROUTE) << "alreadyMappedChilds = " << currDest.alreadyMappedChilds.size() << "\n";

		bool alreadMappedChildRouteSucc = true; // this will change to false if failure in alreadyMappedChilds
		std::map<DFGNode *, std::vector<LatPort>> mappedChildPaths;
		std::map<DFGNode *, std::map<Port *, std::set<DFGNode *>>> mappedChildMutexPaths;
		while (!currDest.alreadyMappedChilds.empty())
		{
			dest_child_with_cost dest_child_with_cost_ins = currDest.alreadyMappedChilds.top();
			currDest.alreadyMappedChilds.pop();

			std::vector<LatPort> possibleStarts;
			possibleStarts.clear();
			possibleStarts.push_back(dest_child_with_cost_ins.startPort);
			for (std::pair<Port *, int> pair : node->routingPorts)
			{
				possibleStarts.push_back(std::make_pair(pair.first->getLat(), pair.first));
				assert(pair.first->getLat() != -1);
			}

			std::priority_queue<cand_src_with_cost> q;
			std::map<Port *, std::set<DFGNode *>> mutexPathsTmp;
			std::vector<LatPort> pathTmp;
			for (LatPort p : possibleStarts)
			{
				int cost;
				if (LeastCostPathAstar(p, dest_child_with_cost_ins.childDest, dest_child_with_cost_ins.childDP, pathTmp, cost, node, mutexPathsTmp, dest_child_with_cost_ins.child))
				{
					pathTmp.clear();
					q.push(cand_src_with_cost(p, dest_child_with_cost_ins.childDest, cost));
				}
			}

			int cost;
			std::vector<LatPort> path;
			LatPort src = dest_child_with_cost_ins.startPort;
			LatPort dest = dest_child_with_cost_ins.childDest;

			while (!q.empty())
			{
				cand_src_with_cost head = q.top();
				q.pop();
				std::map<Port *, std::set<DFGNode *>> mutexPaths;
				alreadMappedChildRouteSucc = LeastCostPathAstar(head.src, dest, dest_child_with_cost_ins.childDP, path, cost, node, mutexPaths, dest_child_with_cost_ins.child);
				if (alreadMappedChildRouteSucc)
				{
					assignPath(node, dest_child_with_cost_ins.child, path);
					data_routing_path.emplace(std::make_pair(node, dest_child_with_cost_ins.child), path);
					mappedChildPaths[dest_child_with_cost_ins.child] = path;
					mappedChildMutexPaths[dest_child_with_cost_ins.child] = mutexPaths;
					LOG(ROUTE) << "Route success :: from=" << src.second->getFullName() << "--> to=" << dest.second->getFullName() << "|node=" << node->idx << "\n";
					cout << "Route" << BOLDCYAN << " success " << RESET << ":: from=" << src.second->getFullName()
						 << "--> to=" << dest.second->getFullName() << "|node=" << node->idx << "\n";
					break;
				}
				else
				{
					LOG(ROUTE) << "Route Failed :: from=" << src.second->getFullName() << "--> to=" << dest.second->getFullName() << "\n";
					cout << "Route" << BOLDRED << " Failed " << RESET << ":: from=" << src.second->getFullName()
						 << "--> to=" << dest.second->getFullName() << "\n";
					for (LatPort p : path)
					{
						if (p.second->getMod()->getPE())
						{
							LOG(ROUTE) << p.second->getMod()->getPE()->getName() << "-->";
						}
					}
					LOG(ROUTE) << "\n";

					for (LatPort p : path)
					{
						LOG(ROUTE) << p.second->getFullName() << "\n";
					}
				}
				path.clear();
			}
			if (!alreadMappedChildRouteSucc)
			{
				*failedNode = dest_child_with_cost_ins.child;
				break;
			}
		}

		if (alreadMappedChildRouteSucc)
		{
			for (std::pair<Port *, int> pair : node->routingPorts)
			{
				Port *p = pair.first;
				int destIdx = pair.second;
				LOG(ROUTE) << "to:" << destIdx << "," << p->getFullName() << "\n";
			}
		}

		if (!alreadMappedChildRouteSucc)
		{
			node->clear(this->dfg);
			continue; // try the next dest
		}
		else
		{
			std::stringstream ss;
			ss << "Already Mapped child Routes....\n";
			for (std::pair<DFGNode *, std::vector<LatPort>> pair : mappedChildPaths)
			{
				DFGNode *child = pair.first;
				for (LatPort lp : pair.second)
				{
					Port *p = lp.second;
					ss << "to:" << child->idx << " :: ";
					ss << p->getFullName();
					if (mappedChildMutexPaths[child].find(p) != mappedChildMutexPaths[child].end())
					{
						ss << "|mutex(";
						for (DFGNode *mutexnode : mappedChildMutexPaths[child][p])
						{
							ss << mutexnode->idx << ",";
						}
						ss << ")";
					}
					ss << "\n";
				}
				ss << "\n";
			}
			ss << "\n";
			LOG(ROUTE) << ss.str();
		}

		bool parentRoutSucc = true;
		addedRoutingParentPorts = 0;
		std::map<DFGNode *, std::map<Port *, std::set<DFGNode *>>> mappedParentMutexPaths;
		while (!currDest.parentStartLocs.empty())
		{
			parent_cand_src_with_cost pcswc = currDest.parentStartLocs.top();
			currDest.parentStartLocs.pop();
			DFGNode *parent = pcswc.parent;
			std::priority_queue<cand_src_with_cost> &q = pcswc.cswc;

			bool succ = false;
			while (!q.empty())
			{
				cand_src_with_cost cand_src_with_cost_ins = q.top();
				q.pop();
				LatPort src = cand_src_with_cost_ins.src;
				LatPort dest = cand_src_with_cost_ins.dest;
				std::vector<LatPort> path;
				std::map<Port *, std::set<DFGNode *>> mutexPath;
				int cost;
				succ = LeastCostPathAstar(src, dest, currDest.dest, path, cost, parent, mutexPath, node);
				if (succ)
				{
					//					bool routedParent=true;
					//					if(parent->routingPorts.size()==0){ //unrouted parent
					//						routedParent=false;
					//					}
					assignPath(parent, node, path);
					data_routing_path.emplace(std::make_pair(parent, node), path);

					mappedParentMutexPaths[parent] = mutexPath;
					addedRoutingParentPorts += path.size();
					//					if(routedParent){
					addedRoutingParentPorts -= 1;
					//					}
					//					for(Port* p : path){
					//						std::cout << p->getFullName() << ",\n";
					//					}
					//					std::cout << "\n";
					break;
				}
				else
				{
					addedRoutingParentPorts = 0;
					node->clear(this->dfg);
					LOG(ROUTE) << "Route Failed :: from=" << src.second->getFullName() << "--> to=" << dest.second->getFullName() << "\n";
				}
				path.clear();
			}
			if (!succ)
			{
				*failedNode = parent;
				node->clear(this->dfg);
				addedRoutingParentPorts = 0;
				parentRoutSucc = false; // at least one parent failed to route, try a new dest
				break;
			}
		}

		if (parentRoutSucc)
		{ // all parents routed succesfull + all mapped childs are connected
			routeSucc = true;
			std::stringstream ss;
			ss << "node=" << node->idx << ",op=" << node->op << " is mapped to " << currDest.dest->getPE()->getName() << ",lat=" << currDest.destLat << "\n";
			ss << "routing info ::\n";
			for (DFGNode *parent : node->parents)
			{
				ss << "parent routing port size = " << parent->routingPorts.size() << "\n";
				int prev_lat = -1;
				for (std::pair<Port *, int> pair : parent->routingPorts)
				{
					Port *p = pair.first;
					//					if(node.routingPortDestMap[p]==&node){
					ss << "fr:" << parent->idx << " :: ";
					ss << ",dest=" << pair.second << " :: ";
					ss << p->getFullName();
					ss << ",lat=" << p->getLat();

					if (mappedParentMutexPaths[parent].find(p) != mappedParentMutexPaths[parent].end())
					{
						ss << "|mutex(";
						for (DFGNode *mutexnode : mappedParentMutexPaths[parent][p])
						{
							ss << mutexnode->idx << ",";
						}
						ss << ")";
					}
					ss << std::endl;
					//					}
					if (prev_lat != -1)
					{
						//							assert(p->getLat() - prev_lat <= 1);
					}
					prev_lat = p->getLat();
				}
			}
			ss << "routing info done.\n";
			currDest.dest->assignNode(node, currDest.destLat, this->dfg);
			node->rootDP = currDest.dest;
			dfg_node_placement.emplace(node, std::make_pair(currDest.dest, currDest.destLat));

			mappingLog4 << node->idx << "," << currDest.dest->getPE()->X << "," << currDest.dest->getPE()->Y << "," << currDest.destLat << "\n";
			ss << "mappingLog4=" << node->idx << "," << currDest.dest->getPE()->X << "," << currDest.dest->getPE()->Y << "," << currDest.destLat << "\n";
			LOG(ROUTE) << ss.str();

			break;
		}
		node->clear(this->dfg);
	}

	if (routeSucc)
	{
		LOG(ROUTE) << "Route success...\n";

		int parentRoutingPortCountEnd = 0;
		//		int mappedParentCount=0;
		for (DFGNode *parent : node->parents)
		{
			if (parent->rootDP != NULL)
			{
				//				mappedParentCount++;
				parentRoutingPortCountEnd += parent->routingPorts.size();
			}
		}
		parentRoutingPortCountEnd = std::max(0, parentRoutingPortCountEnd - routedParents);
		if (parentRoutingPortCountEnd != parentRoutingPortCount + addedRoutingParentPorts)
		{
			LOG(ROUTE) << "parentRoutingPortCountEnd=" << parentRoutingPortCountEnd << "\n";
			LOG(ROUTE) << "addedRoutingParentPorts=" << addedRoutingParentPorts << "\n";
			LOG(ROUTE) << "parentRoutingPortCount=" << parentRoutingPortCount << "\n";
		}

		//		assert(parentRoutingPortCountEnd==parentRoutingPortCount+addedRoutingParentPorts);
		return true;
	}
	else
	{
		currDest.dest->assignNode(node, currDest.destLat, this->dfg);
		dfg_node_placement.emplace(node, std::make_pair(currDest.dest, currDest.destLat));
		node->rootDP = currDest.dest;
		node->clear(this->dfg);
		LOG(ROUTE) << "Route failed...\n";

		int parentRoutingPortCountEnd = 0;
		//		int mappedParentCount=0;
		for (DFGNode *parent : node->parents)
		{
			if (parent->rootDP != NULL)
			{
				//				mappedParentCount++;
				parentRoutingPortCountEnd += parent->routingPorts.size();
			}
		}
		parentRoutingPortCountEnd = std::max(0, parentRoutingPortCountEnd - routedParents);
		if (parentRoutingPortCountEnd != parentRoutingPortCount + addedRoutingParentPorts)
		{
			LOG(ROUTE) << "parentRoutingPortCountEnd=" << parentRoutingPortCountEnd << "\n";
			LOG(ROUTE) << "addedRoutingParentPorts=" << addedRoutingParentPorts << "\n";
			LOG(ROUTE) << "parentRoutingPortCount=" << parentRoutingPortCount << "\n";
		}
		//		assert(parentRoutingPortCountEnd==parentRoutingPortCount);
		assert(*failedNode != NULL);
		return false;
	}
}

std::string CGRAXMLCompile::SAMapper::mapStatus()
{
	std::stringstream ss;
	for (auto node : sortedNodeList)
	{
		if (dfg_node_placement.find(node) == dfg_node_placement.end())
		{
			ss << "node " << node->idx << " not placed.\n";
		}
		else
		{
			auto p = dfg_node_placement[node];
			ss << "node " << node->idx << " placed on " << p.first->getFullName() << " lat:" << p.second << "\n";
		}
	}
	return ss.str();
}

int CGRAXMLCompile::SAMapper::getNumberOfUnmappedNodes()
{
	int n = 0;
	for (auto node : sortedNodeList)
	{
		if (dfg_node_placement.find(node) == dfg_node_placement.end())
		{
			cout << "unmapped node idx = " << node->idx << "\n";
			n++;
		}
	}
	return n;
}
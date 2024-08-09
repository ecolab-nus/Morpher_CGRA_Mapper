#include <morpher/mapper/PathFinderMapper.h>

#include <morpher/mapper/SimulatedAnnealingMapper.h>
#include <queue>
#include <assert.h>
#include <math.h>
#include <algorithm> // std::reverse
#include <morpher/arch/DataPath.h>
#include <morpher/arch/FU.h>

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

bool CGRAXMLCompile::SAMapper::SAMap(CGRA *cgra, DFG *dfg)
{
	std::stack<DFGNode *> mappedNodes;
	std::stack<DFGNode *> unmappedNodes;
	std::map<DFGNode *, std::priority_queue<dest_with_cost>> estimatedRouteInfo;

	// Disable mutex paths to test pathfinder
	this->enableMutexPaths = true;


	this->cgra = cgra;
	this->dfg = dfg;

	Check_DFG_CGRA_Compatibility();

	if (cgra->is_spm_modelled)
	{
		UpdateVariableBaseAddr();
	}
	sortBackEdgePriorityASAP();

	bool mapSuccess = false;

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
	// initial mapping
	if (!initMap_with_PathFinder())
	// if(! initMap_by_random_way())
	{
		assert(false && "how come?");
		std::cout << "cannot find an initial mapping, exit....\n";
		return false;
	}
	mapping_method_name = "SA_pf";

	// get mapping information

	std::stringstream congestion_detail;
	int unmapped_node_numer = getNumberOfUnmappedNodes();
	int overuse_number = getCongestionNumber(congestion_detail);
	int conflict_number = getConflictNumber(congestion_detail);
	std::cout << "Initial mapping done. unmapped node:" << unmapped_node_numer << " overuse:" << overuse_number << " conflict:" << conflict_number << " cost:" << getCost() << " \n";
	std::cout << "unmapped node: \n";
	for (auto node : sortedNodeList)
	{
		if (dfg_node_placement.find(node) == dfg_node_placement.end())
		{
			std::cout << "\t" << node->idx << "  " << node->op << "\n";
		}
	}

	LOG(ROUTE) << congestion_detail.str();

	if (isCurrMappingValid())
	{
		std::cout << "find a valid initial mapping, exit....II =" << this->cgra->get_t_max() << "\n";
		mapped_mrrg_info mmi;
		get_mapped_mrrg_info(mmi);
		std::ofstream result_file;
		result_file.open("/home/zhaoying/gemver_before.txt");
		result_file<< mmi.toStr();
		result_file.close();
		return true;
	}

	// start Simulated Annealing mapping
	std::cout << "maximum temperature:" << maximum_temp << " minimum temperature:" << minimim_temp << "\n";
	curr_cost = getCost();
	curr_temp = maximum_temp;

	std::cout << "###############current mapping: \n"
			  << dumpMappingToStr();

	while (curr_temp > minimim_temp)
	{
		std::cout << "*******************************current temperature:" << curr_temp << "\n";
		float accept_rate = inner_map();

		if (freeze_mapping)
		{
			assert(false);
		}
		congestion_detail.clear();
		std::cout <<"II:"<<this->cgra->get_t_max() << " accept_rate:" << accept_rate << " # of overuse:" << getCongestionNumber(congestion_detail) << " # of conflict:" << getConflictNumber(congestion_detail)
				  << " unmapped nodes:" << getNumberOfUnmappedNodes() << " cost:" << curr_cost << "\n";
		std::cout << "unmapped node: \n";
		for (auto node : sortedNodeList)
		{
			if (dfg_node_placement.find(node) == dfg_node_placement.end())
			{
				std::cout << "\t" << node->idx << "  " << node->op << "\n";
			}
		}
		LOG(ROUTE) << congestion_detail.str();
		std::cout << "###############current mapping: \n"
				  << dumpMappingToStr();

		curr_temp = updateTemperature(curr_temp, accept_rate);

		if (isCurrMappingValid())
		{
			std::cout << "find a valid mapping, exit...II =" << this->cgra->get_t_max() << "\n";
			break;
		}
	}
	if(isCurrMappingValid()){
		mapped_mrrg_info mmi;
		get_mapped_mrrg_info(mmi);
		std::ofstream result_file;
		result_file.open("/home/zhaoying/gemver_before.txt");
		result_file<< mmi.toStr();
		result_file.close();
	}
	

	return isCurrMappingValid();
}
bool CGRAXMLCompile::SAMapper::initMap_by_random_way()
{
	using mapping_placement = std::map<DFGNode*, std::pair<DataPath*, int>>;
	std::vector<std::pair<mapping_placement, int>> mapping_records;
	auto generate_random_mapping = [&]()
	{
		dfg_node_placement.clear();
		data_routing_path.clear();
		congestedPorts.clear();
		std::stack<DFGNode *> unmappedNodes;
		std::stack<DFGNode *> mappedNodes;
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
			MapHeader << "current node = " << node->idx
					  << ",op = " << node->op
					  << ",unmapped nodes = " << unmappedNodes.size()
					  << ",mapped nodes = " << mappedNodes.size()
					  << ",freeMemNodes = " << cgra->freeMemNodes
					  << ",unmappedMemNodes = " << dfg->unmappedMemOps
					  << ",II = " << cgra->get_t_max()
					  << ",CGRA=" << this->cgra->getCGRAName()
					  << ",MaxHops=" << this->cgra->max_hops
					  << ",BB = " << node->BB
					  << ",mutexPathEn = " << this->enableMutexPaths
					  << "\n";

			LOG(ROUTE) << MapHeader.str();

			auto candidates = getRandomDPCandidate(node);
			if (candidates.size() == 0)
			{
				// TODO
			}
			bool curr_node_mapped = false;
			for (auto dp_candidate : candidates)
			{

				bool route_succ = SARoute(node, dp_candidate);
				if (route_succ)
				{
					curr_node_mapped = true;
					break;
				}
				else
				{
					clearNodeMapping(node);
				}
			}
		}

		//store the current placement
		std::map<DFGNode*, std::pair<DataPath*, int>> curr_dfg_node_placement;
		for(auto & [node, placement]: dfg_node_placement){
			curr_dfg_node_placement.emplace(node, placement);
		}

		int curr_cost = getCost();

		std::cout<<"init map: current placement:"<<curr_dfg_node_placement.size()<<" cost:"<<curr_cost<<"\n";
		
		for(auto node: sortedNodeList){
			clearNodeMapping(node, true);
		}	
		for (std::pair<Port *, std::set<DFGNode *>> pair : congestedPorts){
			auto p  = pair.first;
			p->clear();
		}
		dfg_node_placement.clear();
		data_routing_path.clear();
		congestedPorts.clear();

		mapping_records.push_back(std::make_pair(curr_dfg_node_placement, curr_cost));	
		// curr_dfg_node_placement.clear();

	};
	
	
	for(int i = 0; i < try_number_for_random_init_mapping; i++){
		generate_random_mapping();
	}
	std::sort(mapping_records.begin(), mapping_records.end(), 
	[](auto a, auto b){
		return a.second < b.second;
	});
	dfg_node_placement.clear();
	data_routing_path.clear();
	congestedPorts.clear();
	//regenerate mapping according the best one.
	std::cout<<"number of records:"<<mapping_records.size()<<"\t";
	auto best_placement = mapping_records.front().first;
	std::cout<<"placed nodes: "<<best_placement.size()<<"\n";
	auto reversed_node_list = sortedNodeList;
	std::reverse(reversed_node_list.begin(), reversed_node_list.end());
	for(auto node:reversed_node_list){
		if(best_placement.find(node) != best_placement.end()){
			bool route_succ = SARoute(node, best_placement[node].first);
			assert(route_succ);	
			if(!route_succ){
				clearNodeMapping(node);
			}
		}
	}
	return true;
}

bool CGRAXMLCompile::SAMapper::initMap_with_PathFinder()
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
			node->CarefulClear(this->dfg);
		}
	}

	return true;
}

float CGRAXMLCompile::SAMapper::inner_map()
{
	int accepted_number = 0;

	for (int i = 0; i < movement_in_each_temp; i++)
	{
		std::stringstream congestion_detail;
		LOG(SA) << "************ NO." << i << " movement, unmapped nodes:" << getNumberOfUnmappedNodes() << ", congestion:" << getCongestionNumber(congestion_detail)
				<< ", congestion:" << getConflictNumber(congestion_detail);
		LOG(ROUTE) << congestion_detail.str();

		// select DFG node to unmap. If this node is not mapped yet, will select a parent to unmap as well.
		// std::vector<DFGNode *> moved_nodes;
		auto selected_dfg_node = selectDFGNodeToUnmap();
		DFGNode *child_node = NULL;
		/*if (dfg_node_placement.find(selected_dfg_node) == dfg_node_placement.end())
		{

			// this node is not placed yet because of routing failure.
			// The reason for routing failure is that it cannot route to its parent nodes or recurrent dependency constraint;
			// child_node = selected_dfg_node;
			// selected_dfg_node = selectAParentForDFGNode(child_node);
			// LOG(SA) << "----------be careful. Have a child node " << child_node->idx << "\n";
		}*/
		// assert(dfg_node_placement.find(selected_dfg_node)!= dfg_node_placement.end());
		std::map<DFGNode *, std::pair<DataPath *, int>> old_dfg_node_placement;
		old_dfg_node_placement.insert(dfg_node_placement.begin(), dfg_node_placement.end());
		std::map<dfg_data, std::vector<LatPort>> old_data_routing_path;
		old_data_routing_path.insert(data_routing_path.begin(), data_routing_path.end());

		LOG(SA) << "select DFG node " << selected_dfg_node->idx << " to unmap";
		// start map
		if (dfg_node_placement.find(selected_dfg_node) != dfg_node_placement.end())
		{
			LOG(SA) << "current placement:" << selected_dfg_node->rootDP->getFullName();
			clearNodeMapping(selected_dfg_node);
		}
		else
		{
			LOG(SA) << "this DFG node is not placed yet";
		}
		auto candidates = getRandomDPCandidate(selected_dfg_node);
		bool route_succ = false;
		if (candidates.size() == 0)
		{
			std::cout<<"did not find a candidate\n";
		}else{

			auto dp_candidate = candidates.front();
			LOG(SA) << "map this DFG node:" << selected_dfg_node->idx << " op:" << selected_dfg_node->op << "to pe:" << dp_candidate->getPE()->getName();
			route_succ = SARoute(selected_dfg_node, dp_candidate);
		}
		// moved_nodes.push_back(selected_dfg_node);

		// map the unmapped parents of this child node
		/*if (route_succ && child_node != NULL && false)
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
		}*/

		// process the child node
		/*if (route_succ && child_node != NULL)
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
		}*/

		// decide accept or node
		bool accept = false;
		int attempted_cost = getCost();
		if (route_succ )
		{
			// if not route success, then nothing changes and should restore mapping
			accept = whetherAcceptNewMapping(attempted_cost, curr_cost, curr_temp);
		}

		LOG(SA) << "accept " << (accept ? "ture" : "false") << " route_succ:" << route_succ << " curr_cost:" << curr_cost << " attepmted cost:" << attempted_cost << "\n";

		if(route_succ){
			LOG(SA)<<"route success \t"<<"temp:"<<curr_temp<<" curr cost:"<<curr_cost<<" cost change:"<<(attempted_cost-curr_cost)<<" "<< "accept " << (accept ? "ture" : "false")<<"\n"; 
		}else{
			LOG(SA)<<"route failure";
		}
		if (accept)
		{
			// update overuse
			//  save the mapping state
			curr_cost = attempted_cost;
			accepted_number++;
			total_accepted_number ++;

			old_dfg_node_placement.clear();
			old_data_routing_path.clear();

			// printHyCUBEBinary(this->cgra);
			if (isCurrMappingValid())
			{
				std::cout << "find a valid mapping, exit inner_map....\n";
				return (float(accepted_number)) / movement_in_each_temp;
			}
		}
		else
		{
			clearNodeMapping(selected_dfg_node);
			restoreMapping(selected_dfg_node, old_dfg_node_placement, old_data_routing_path);

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

	std::random_device r;
	std::default_random_engine e1(r());
	std::uniform_int_distribution<int> uniform_dist(0, sortedNodeList.size() - 1);
	return sortedNodeList[std::abs(std::round(uniform_dist(e1)))];

	// select random node;

	std::vector<DFGNode *> unmapped_node;
	for (auto node : sortedNodeList)
	{
		if (dfg_node_placement.find(node) == dfg_node_placement.end())
		{
			unmapped_node.push_back(node);
		}
	}
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

// even not mapped yet, should call it to clear for the mapping.
bool CGRAXMLCompile::SAMapper::clearNodeMapping(DFGNode *node, bool violenceClear)
{
	LOG(ROUTE) << "clear node =" << node->idx;
	DataPath* dp = nullptr;
	bool mapped = (dfg_node_placement.find(node) != dfg_node_placement.end());
	if (mapped)
	{
		dp  = dfg_node_placement[node].first;
		assert(dp == node->rootDP);
		dfg_node_placement.erase(dfg_node_placement.find(node));
		assert(dfg_node_placement.find(node) == dfg_node_placement.end());
	}else{
		assert(node->rootDP == NULL);
	}
	
	bool routed = false;
	for (auto it = data_routing_path.begin(); it != data_routing_path.end();)
	{
		// LOG(SA)<<"routing path:" << it->first.first->idx<<" -> "<<it->first.second->idx;
		if (it->first.first == node || it->first.second == node)
		{
			routed = true;
			// std::stringstream ss;
			// ss<<it->first.first->idx<<" to "<<it->first.second->idx<<"\n";
			// for(auto node: it->second){
			// 	ss<<node.second->getFullName()<<"->";
			// }
			// LOG(ROUTE)<<"clear path:"<<ss.str();

			it = data_routing_path.erase(it);
		}
		else
		{
			it++;
		}
	}
	// assert(mapped ==  routed);
	if(violenceClear){
		node->clear(this->dfg);
	}
	else{
		node->CarefulClear(this->dfg);
	}
	assert(node->rootDP == NULL);
	if(dp!=nullptr){
		assert(dp->getMappedNode() ==  NULL);
	}
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

bool CGRAXMLCompile::SAMapper::restoreMapping(std::vector< DFGNode*> nodes, std::map<DFGNode *, std::pair<DataPath *, int>> &dfg_node_placement,
											  std::map<dfg_data, std::vector<LatPort>> &data_routing_path)
{
	// assign placement
	for(auto node: nodes){
		if (dfg_node_placement.find(node) != dfg_node_placement.end())
		{
			auto placement_info = dfg_node_placement[node];
			placement_info.first->assignNode(node, placement_info.second, this->dfg);
			node->rootDP = placement_info.first;
		}
	}

	// assign routing
	std::set<dfg_data> added_data;
	for(auto node: nodes){

		for (auto &data_path : data_routing_path)
		{
			if (data_path.first.first == node || data_path.first.second == node)
			{
				// auto find_item = std::find_if(added_data.begin(), added_data.end(),
				// [&](auto a, auto b) { return a.first->idx == b.first->idx && a.second->idx && b.second->idx; });
				if(added_data.find(data_path.first) != added_data.end()){
					continue;
				}
				assignPath(data_path.first.first, data_path.first.second, data_path.second);
				added_data.emplace(data_path.first);
			}
		}
	}

	return true;
}

std::vector<CGRAXMLCompile::dfg_data> CGRAXMLCompile::SAMapper::find_congestion_paths(){
	std::vector<dfg_data> congested_path;
	for (std::pair<Port *, std::set<DFGNode *>> pair : congestedPorts)
	{

		std::set<DFGNode*> mapped_value;
		for(auto node: pair.first->mapped_nodes){
			auto n = std::get<0>(node);
			mapped_value.insert(n);
		}
		if(mapped_value.size()<=1){
			continue;
		}
		
		//there is a congestion
		for(auto info: pair.first->mapped_nodes){
			congested_path.push_back(std::make_pair(std::get<0>(info), this->dfg->findNode(::get<1>(info))));
		}
	}
	return congested_path;
} 

bool  CGRAXMLCompile::SAMapper::does_node_cause_congestion(DFGNode* node, std::vector<dfg_data>  &congested_path){
	for(auto data: congested_path){
		if(data.first->idx == node->idx || data.second->idx == node->idx){
			return true;
		}
	}
	return false;
}

int CGRAXMLCompile::SAMapper::getCongestionNumber(std::stringstream &congestion_detail)
{
	int congestion_number = 0;


	for (std::pair<Port *, std::set<DFGNode *>> pair : congestedPorts)
	{
		Port *p = pair.first;
		if (pair.first->mapped_nodes.size() > 1)
		{
			std::set<DFGNode*> mapped_value;
			for(auto node: pair.first->mapped_nodes){
				auto n = std::get<0>(node);
				mapped_value.insert(n);
			}
			if(mapped_value.size()<=1){
				continue;
			}
			congestion_number+= mapped_value.size();
			auto mapped_nodes = pair.first->mapped_nodes;
			for(auto & [node, dest, lat]: mapped_nodes){
				congestion_detail<<p->getFullName()<<" " <<node->idx<<"->"<<dest<<"(lat:"<<lat<<"),\n";
			}
			// for (DFGNode *node1 : pair.second)
			// {
			// 	for (DFGNode *node2 : pair.second)
			// 	{
			// 		if (node1 == node2)
			// 		{
			// 			continue;
			// 		}
			// 		if (this->dfg->isMutexNodes(node1, node2, p))
			// 			continue;
			// 		congestion_detail << "CONGESTION:" << p->getFullName();
			// 		for (DFGNode *node : pair.second)
			// 		{
			// 			congestion_detail << "\t ," << node->idx << "|BB=" << node->BB;
			// 		}
			// 		congestion_detail << "\n";
			// 		congestion_number++;
			// 		//					break;
			// 	}
			// }
		}
	}
	// LOG(SA)<<congestion_detail.str();
	return congestion_number;
}

int CGRAXMLCompile::SAMapper::getConflictNumber(std::stringstream &congestion_detail)
{
	int conflict_number = 0;

	for (std::pair<Port *, std::set<DFGNode *>> pair : conflictedPorts)
	{
		Port *p = pair.first;
		if (p->getNode() != NULL)
		{
			conflict_number += pair.second.size();

			congestion_detail << "CONFLICT :" << p->getFullName();
			for (DFGNode *node : pair.second)
			{
				congestion_detail << "\t ," << node->idx << "|BB=" << node->BB;
			}
			congestion_detail << ", with MAPPED = " << p->getNode()->idx << "|BB=" << p->getNode()->BB;
		}
	}

	return conflict_number;
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
	int conflict_number = getConflictNumber();
	int total_cost = unmapped_node_number * 200 + congestion_number * 5 + conflict_number * 5 + port_usage;
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
				// Previsouly, this assumes that mapped children are for next iteration.
				//  For SA, the mapped children might not be so.
				if (node->childNextIter[child] == 1)
				{
					alreadyMappedChildPorts[child]->setLat(child->rootDP->getLat() + ii);
				}
				else if (node->childNextIter[child] == 0)
				{
					alreadyMappedChildPorts[child]->setLat(child->rootDP->getLat());
				}
				else
				{
					assert(false);
				}
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

		// this is to check recurrent store-load constraint.
		std::map<DataPath *, beParentInfo> beParentDests;
		int downStreamOps = 0;
		int maxLat = getMaxLatencyBE(node, beParentDests, downStreamOps);
		if (maxLat != LARGE_VALUE)
			assert(!beParentDests.empty());
		if(minLatDestVal_prime > maxLat){
			return false;
		}

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

				LOG(ROUTE) << "routing success from node" << parent->idx << "," << startCand->getFullName() << "," << startCandLat.first << " to "
						<< node->idx << ", node" << destPort->getFullName() << "," << destPortLat.first << "\n";
				assignPath(parent, node, path);
				data_routing_path.emplace(std::make_pair(parent, node), path);
			}
			else
			{

				LOG(ROUTE) << "routing failure from " << parent->idx << "," << startCand->getFullName() << "," << startCandLat.first << " to "
						<< node->idx << "," << destPort->getFullName() << "," << destPortLat.first << "\n";
				break;
			}
		}

		if (!route_success)
		{
			node->CarefulClear(this->dfg);
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
			// if (detailedDebug)
			// 	std::cout << "already child Estimating Path" << destPort->getFullName() << "," << minLatDestVal + latency << ","
			// 				<< "--->" << childDestPort->getFullName() << "," << childDestPort->getLat() << "," << "exist_child = " << child->idx
			// 				<< "\n";
			// if (detailedDebug)
			// 	std::cout << "lat = " << childDestPort->getLat() << ",PE=" << childDestPort->getMod()->getPE()->getName() << ",t=" << childDestPort->getMod()->getPE()->T << "\n";
			childDestPort->setLat(childDP->getLat() + node->childNextIter[child] * II);
			LatPort childDestPortLat = std::make_pair(childDestPort->getLat(), childDestPort);
			assert(childDestPort->getLat() != -1);
			LatPort destPortLat = std::make_pair(minLatDestVal + latency, destPort);

			route_success = LeastCostPathAstar(destPortLat, childDestPortLat, childDP, path, cost, node, mutexPaths, child);

			if (route_success)
			{

				LOG(ROUTE) << "routing success from " << node->idx << "," << destPort->getFullName() << "," << destPortLat.first << " to "
						<< child->idx << "," << childDestPort->getFullName() << "," << childDestPortLat.first << "\n";
				assignPath(node, child, path);
				data_routing_path.emplace(std::make_pair(node, child), path);
			}
			else
			{

				LOG(ROUTE) << "routing failure from " << node->idx << "," << destPort->getFullName() << "," << destPortLat.first << " to "
						<< child->idx << "," << childDestPort->getFullName() << "," << childDestPortLat.first << "\n";
				break;
			}
		}
		if (!route_success)
		{
			node->CarefulClear(this->dfg);
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

// compared to the PathFinderMapper one, add some additional information for SA.
bool CGRAXMLCompile::SAMapper::Route(DFGNode *node,
									 std::priority_queue<dest_with_cost> &estimatedRoutes,
									 DFGNode **failedNode)
{
	int queue_number = 0;
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
		queue_number++;
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
					q.push(cand_src_with_cost(p, dest_child_with_cost_ins.childDest, cost, path_toStr(pathTmp)));
					pathTmp.clear();
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
					LOG(SA) << "Route success :: from=" << src.second->getFullName() << "--> to=" << dest.second->getFullName() << "|node=" << node->idx << "\n";
					break;
				}
				else
				{
					LOG(SA) << "Route Failed :: from=" << src.second->getFullName() << "--> to=" << dest.second->getFullName() << "\n";
					for (LatPort p : path)
					{
						if (p.second->getMod()->getPE())
						{
							LOG(SA) << p.second->getMod()->getPE()->getName() << "-->";
						}
					}
					LOG(SA) << "\n";

					for (LatPort p : path)
					{
						LOG(SA) << p.second->getFullName() << "\n";
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
			node->CarefulClear(this->dfg);
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
					node->CarefulClear(this->dfg);
					LOG(SA) << "Route Failed :: from=" << src.second->getFullName() << "--> to=" << dest.second->getFullName() << "\n";
				}
				path.clear();
			}
			if (!succ)
			{
				*failedNode = parent;
				node->CarefulClear(this->dfg);
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
		node->CarefulClear(this->dfg);
	}

	if (routeSucc)
	{
		// LOG(SA) << "Route success... for the "<<queue_number<<" one in queue\n";

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
		node->CarefulClear(this->dfg);
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
			// std::cout<<"unmapped node:"<<node->idx<<"\n";
			n++;
		}
	}
	return n;
}

int CGRAXMLCompile::SAMapper::checkAnyUnroutedEdge()
{
	int n = 0;
	for (auto node : sortedNodeList)
	{
		for (auto child : node->children)
		{
			if (node->childrenOPType[child] == "PS")
				continue;
			if (data_routing_path.find(std::make_pair(node, child)) == data_routing_path.end())
			{
				LOG(ROUTE) << "did not route edge: " << node->idx << " ->" << child->idx << "\n";
				n++;
			}
		}
	}
	return n;
}

void CGRAXMLCompile::SAMapper::get_mapped_mrrg_info(mapped_mrrg_info & m_mrrg){
	m_mrrg.nodes.clear();
	m_mrrg.edges.clear();
	m_mrrg.congestion.clear();
	m_mrrg.unmapped_nodes.clear();

	int max_latency = 0;

	std::vector< std::set<DFGNode *>> latency_to_node_vector; // the index represend the latency
	for(int i = 0; i < 1000; i++){
		std::set<DFGNode *> temp;
		latency_to_node_vector.push_back( temp);
	}
	std::set<DFGNode *> unmapped_dfg_nodes;
	std::map<DFGNode *, int> node_latency;

	
	//get the latency of each operation
	for(auto node: sortedNodeList){
		if(node->rootDP == NULL){
			unmapped_dfg_nodes.insert(node);
			// std::cout<<"unmapped_node:"<<node->idx<<"\n";
			continue;
		}
		int lat = node->rootDP->getLat();
		// std::cout<<"node:"<<node->idx<<" lat "<<lat<<"\n";
		latency_to_node_vector[lat].insert(node);
		node_latency.emplace(node, lat);
		max_latency = max_latency>lat ? max_latency:lat;
	}

	//verify that the latency satisfies the data dependency
	for(auto& [node, lat]: node_latency){
		for(auto parent: node->parents){
			if (parent->childrenOPType[node] == "PS")
			{
				continue;
			}
			if(parent->childNextIter[node] == 0){
				if(node_latency.find(parent)!= node_latency.end()){
					assert(node_latency[parent] <  lat);
				}
			}else if (parent->childNextIter[node] == 1){
				if(node_latency.find(parent)!= node_latency.end()){
					assert(node_latency[parent] <  lat + this->cgra->get_t_max());
				}
			}else {
				assert(false && "why this value");
			}
			
		}

		for(auto child: node->children){
			if (node->childrenOPType[child] == "PS")
			{
				continue;
			}
			if(node->childNextIter[child] == 0){
				if(node_latency.find(child)!= node_latency.end()){
					assert(node_latency[child] >  lat);
				}
			}else if (node->childNextIter[child] == 1){
				if(node_latency.find(child)!= node_latency.end()){
					assert(node_latency[child]  + this->cgra->get_t_max()>  lat );
				}
			}else {
				assert(false && "why this value");
			}
		}
	}

	//get the node placement
	auto & mrrg_info_nodes = m_mrrg.nodes;
	for(auto &[node, info]: dfg_node_placement){
		auto & dp = info.first;
		mrrg_info_nodes.emplace(node->idx, mrrg_node_info{dp->getPE()->getPosition_X(),dp->getPE()->getPosition_Y(),
									dp->get_t(), info.second});
	}	

	//get the routing path
	auto & mrrg_info_edges = m_mrrg.edges;
	for(auto & [data, path]: data_routing_path){
		mrrg_edge_info m_edge;
		mrrg_info_edges.push_back(m_edge);
		auto & the_edge = mrrg_info_edges.back();
		the_edge.parent_node_id  = data.first->idx;
		the_edge.child_node_id = data.second->idx;
		auto & routing_nodes = the_edge.routing_nodes;
        auto dp = data.first->rootDP; 

        // add the src node, which not stored in the path
        routing_nodes.push_back(mrrg_node_info{dp->getPE()->getPosition_X(),dp->getPE()->getPosition_Y(),
									dp->get_t(), dp->getLat()});
		
        //add the node in routing path      
        for(auto & l_port: path){
			auto & port = l_port.second; 
            mrrg_node_info tmp{port->getPE()->getPosition_X(), 
							port->getPE()->getPosition_Y(),port->getPE()->get_t(),l_port.first};
            if(std::find(routing_nodes.begin(), routing_nodes.end(), tmp) == routing_nodes.end()){
                // avoid redundancy 
                routing_nodes.push_back(tmp);
            }
		}

	}
	auto & mrrg_info_congestion = m_mrrg.congestion;
	for (std::pair<Port *, std::set<DFGNode *>> pair : congestedPorts)
	{
		Port *p = pair.first;
		// assert(pair.second.size()  == pair.first->mapped_nodes.size());
		if (pair.first->mapped_nodes.size() > 1)
		{
			// for (DFGNode *node1 : pair.second)
			// {
			// 	for (DFGNode *node2 : pair.second)
			// 	{
			// 		if (node1 == node2)
			// 		{
			// 			continue;
			// 		}
			// 		if (this->dfg->isMutexNodes(node1, node2, p))
			// 			continue;
					
			// 		//					break;
			// 	}
			// 	mrrg_node_info tmp_node{p->getPE()->getPosition_X(), 
			// 				p->getPE()->getPosition_Y(),p->getPE()->get_t(), p->getLat()};
			// 	if(mrrg_info_congestion.find(tmp_node) == mrrg_info_congestion.end()){
			// 		mrrg_info_congestion.emplace(tmp_node, 1);
			// 	}else{
			// 		mrrg_info_congestion[tmp_node] += 1;
			// 	}
			// }

			std::set<DFGNode*> mapped_value;
			for(auto node: pair.first->mapped_nodes){
				auto n = std::get<0>(node);
				mapped_value.insert(n);
			}
			if(mapped_value.size()<=1){
				continue;
			}

			// add routing information
			auto mapped_nodes = pair.first->mapped_nodes;
			std::stringstream routing_info;
			for(auto & [node, dest, lat]: mapped_nodes){
				routing_info<<node->idx<<"->"<<dest<<"(lat:"<<lat<<"),";
			}
			mrrg_node_info tmp_node{p->getPE()->getPosition_X(), 
			 				p->getPE()->getPosition_Y(),p->getPE()->get_t(), p->getLat()};
			// assert(mrrg_info_congestion.find(tmp_node) == mrrg_info_congestion.end());
			mrrg_info_congestion.emplace(tmp_node, std::make_pair(mapped_nodes.size(), routing_info.str()));
		}
	}
	for(auto node: unmapped_dfg_nodes){
		m_mrrg.unmapped_nodes.push_back(node->idx);
	}
	m_mrrg.cgra_x = this->cgra->get_x_max();
	m_mrrg.cgra_y = this->cgra->get_y_max();
	m_mrrg.II = this->cgra->get_t_max();
	m_mrrg.max_lat = max_latency;
}

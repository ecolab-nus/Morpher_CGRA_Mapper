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
	LISAMapCore(cgra, dfg);
}

bool CGRAXMLCompile::LISAMapper::LISAMapCore(CGRA *cgra, DFG *dfg){
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

	std::cout<<"###############current mapping: \n"<<dumpMappingToStr();

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
		std::cout<<"###############current mapping: \n"<<dumpMappingToStr();

		curr_temp = updateTemperature(curr_temp, accept_rate);

		if (isCurrMappingValid())
		{
			std::cout << "find a valid mapping, exit...II =" << this->cgra->get_t_max() << "\n";
			break;
		}
	}

	return isCurrMappingValid();
}


CGRAXMLCompile::DataPath *  CGRAXMLCompile::LISAMapper::getLISADPCandidate(DFGNode *dfg_node, int accepted = 1 , int total_tried =1, int num_swap = 1){
	assert(dfg_label_->find(node_to_id_[dfg_node]) != dfg_label_->end() );
	auto & dfg_node_label = dfg_label_->at(node_to_id_[dfg_node]);

	std::set<DFGNode *> mapped_node;
	// we should calculate based on mapped node
	for(auto node: sortedNodeList){
		if(dfg_node_placement.find(node) != dfg_node_placement.end()){
			mapped_node.insert(node);
		}
	}

	if(mapped_node.size() == 0){
		//should select fu that II is 1
		auto candidates = getCandidateByIIConstraint( 0 , 0, dfg_node);
		std::random_device r;
		std::default_random_engine e1(r());
		std::uniform_int_distribution<int> uniform_dist(0, candidates.size()-1);
		int mean = uniform_dist(e1);
		return candidates[mean];
	} 

	std::vector<DataPath *> candidateDests = getRandomDPCandidate(dfg_node);
	int schedule_order =   dfg_node_label.schedule_order;
	auto dumped_mapping = dumpMapping();

	auto interval = getIntervalByScheduleOrder( dumped_mapping, dfg_node, schedule_order);
		// if we only use label in initial mapping, then the second value is not useful
	int early_II =  interval.first;
	int start_II = 0;
	if(early_II +1 == this->cgra->get_t_max()) {
		start_II = 0;
	}else{
			start_II = early_II +1;
	}

	assert(candidateDests.size() > 0);
	std::map<DataPath *, int> comm_cost;
	for(auto candi: candidateDests){
            comm_cost.emplace(candi,0);
    }
	auto samelevel_node_cost=  getCostForSameLevelNode(dumped_mapping, candidateDests, dfg_node );
	auto ass_cost = getCostByAssociation( dumped_mapping, candidateDests, dfg_node, start_II);

	std::stringstream output;
	std::map<DataPath*, int> timing_cost ;
	auto & ass = dfg_label_->at(node_to_id_[dfg_node]).association;
	for(auto node: candidateDests){
		int ii_value = node->get_t();
		int t_cost = 0;
		for(auto node_ass: ass){
			int node_id = node_ass.first;
			if(dumped_mapping.find(node_id) == dumped_mapping.end()) continue;
			auto & m = dumped_mapping[node_id];
			int spatial_diff =  (std::abs(node->getPE()->getPosition_X() - m.x) + std::abs(node->getPE()->getPosition_Y()  - m.y));
			int tempoal_diff = (m.t - ii_value);
			if(spatial_diff > tempoal_diff){
				t_cost += spatial_diff - tempoal_diff ;
			}
			
		}
		timing_cost.emplace(node, t_cost);
	}

	output<<"early II: "<<early_II<<"\n";
	output<<"start II: "<<start_II<<"\n";
	output<<"\n\ttiming cost:";
	for(auto node: timing_cost){
		output<<"("<<node.first->getFullName()<<","<<node.second<<") ";
	}

	output<<"\n\tassociation cost:";
	for(auto node: ass_cost){
		output<<"("<<node.first->getFullName()<<","<<node.second<<") ";
	}
	output<<"\n\tcommunication cost:";
	for(auto node: comm_cost){
		output<<"("<<node.first->getFullName()<<","<<node.second<<") ";
	}

	output<<"\n\tstart_node_cost cost:";
	for(auto node: samelevel_node_cost){
		output<<"("<<node.first->getFullName()<<","<<node.second<<") ";
	}

	
	std::map<DataPath *, int> node_cost ;
	for(auto node : candidateDests){
		int cost = timing_cost[node]  + ass_cost[node] + samelevel_node_cost[node] ;
		node_cost.emplace(node, cost);
	}

	output<<"\n\t total cost:";
	for(auto node: node_cost){
		output<<"("<<node.first->getFullName()<<","<<node.second<<") ";
	}
	
	std::sort(candidateDests.begin(), candidateDests.end(), [&](DataPath * a, DataPath * b){
		return node_cost[a] < node_cost[b];
	});

	int min_cost = node_cost[candidateDests.front()];
	int max_cost = node_cost[candidateDests.back()];
        int max_diff = max_cost -  min_cost;

	std::random_device rd{};
	std::mt19937 gen{rd()};
	double deviation = 1;
	// std::cout<<"1,2,3:"<<accpepted<<","<<total_tried<<","<<num_swap<<"\n";
	if(total_tried > 0){
		deviation = 0.1 * total_tried - accepted;
		deviation = std::max(deviation, 1.0);
			
	}
	
	std::normal_distribution<> d{0,deviation};
	int selected_cost = std::abs(std::round(d(gen)));
	if(selected_cost > max_diff)  selected_cost = max_diff;
	selected_cost  = min_cost + selected_cost;

	if(!finish_init) selected_cost = min_cost;
	std::vector<DataPath *> suitable_candidates;
	while(suitable_candidates.size() == 0){
		for(auto c : node_cost){
			if(c.second == selected_cost){
				suitable_candidates.push_back(c.first);
			}
		}
		selected_cost --;
	}
	std::random_device r;
	std::default_random_engine e1(r());
	std::uniform_int_distribution<int> uniform_dist(0, suitable_candidates.size()-1);
	int mean = uniform_dist(e1);
	auto selected_fu = suitable_candidates[mean];

	if(!is_training){
		LOG(LISA)<<"curr mapping: "<<dumpMappingToStr();
		LOG(LISA)<<"label of "<<dfg_node->idx<<"  " <<dfg_node_label.toStr();
		LOG(LISA)<<output.str();
		LOG(LISA)<<"best mapping: "<<lisa_ctrl->bestMappingToStr();
		LOG(LISA)<<" min cost:"<< min_cost<<"   selected cost: "<<selected_cost +1 <<"\n";
		LOG(LISA)<<"lisa op:"<<dfg_node->idx<<" selectFU: "<<selected_fu->getFullName()<<"\n";
	}
        
	return selected_fu;
}


CGRAXMLCompile::DataPath *  CGRAXMLCompile::LISAMapper::getCloseRandomFU(DFGNode* dfg_node, DataPath * old_dp, int max_physical_dis,  int max_temp_dis ){
	
	std::vector<DataPath *> opt_candidates;
	std::vector<DataPath *> all_candidates;

	int old_cycle = old_dp->get_t();
	std::set<int> allowed_cycle ;
	for(int start_cycle = old_cycle - max_temp_dis ; start_cycle <= old_cycle + max_temp_dis;start_cycle ++ ){
		allowed_cycle.insert( (start_cycle + this->cgra->get_t_max())%this->cgra->get_t_max());
	}

	LOG(RANDOMFU)<<"select CloseRandomFU for dfg node:"<<dfg_node->idx<<" previous datapath"<<old_dp->getFullName();
	// assert(false);

	std::vector<DataPath *> candidateDests = getRandomDPCandidate(dfg_node);

	for (auto &dp : candidateDests)
	{
		if(  std::abs(old_dp->getPE()->getPosition_X() - dp->getPE()->getPosition_X()) + std::abs(old_dp->getPE()->getPosition_Y() - dp->getPE()->getPosition_Y()) <= max_physical_dis  
			&& allowed_cycle.find(dp->get_t())!= allowed_cycle.end()){
				opt_candidates.push_back(dp);
		}
	}
	if (all_candidates.size() < 1)
	{
		cout << "Could not place: " << dfg_node->idx << endl;
		assert(all_candidates.size() > 0);
	}
	std::random_device r;
	std::default_random_engine e1(r());
	
	if(opt_candidates.size() == 0){
		auto cal_dist = [](DataPath * old_dp, DataPath * new_dp){
			return std::abs(old_dp->getPE()->getPosition_X() - new_dp->getPE()->getPosition_X()) + 
				std::abs(old_dp->getPE()->getPosition_Y() - new_dp->getPE()->getPosition_Y()) + 
					std::abs(int(old_dp->get_t()) - int(new_dp->get_t()) );
		};
		std::sort(all_candidates.begin(), all_candidates.end(), [&](DataPath * a, DataPath * b){
			return cal_dist(old_dp, a) < cal_dist(old_dp, b);
		});
		std::uniform_int_distribution<int> uniform_dist(0, all_candidates.size()-1);
		int mean = uniform_dist(e1);
		auto sel_dp = all_candidates[mean];
		LOG(LISA)<<"  select fu:"<<sel_dp->getFullName();
		return sel_dp;
	}
	
	std::uniform_int_distribution<int> uniform_dist(0, opt_candidates.size()-1);
	int mean = uniform_dist(e1);
	auto sel_dp = opt_candidates[mean];
	LOG(LISA)<<"  select fu:"<<sel_dp->getFullName();
	return sel_dp;
        
}




//TODO: change interval from II to cycle
std::pair<int,int> CGRAXMLCompile::LISAMapper::getIntervalByScheduleOrder( std::map<int, pos3d> & dumped_mapping, DFGNode * node, int scheduler_order ){
	int start_time = -1, end_time = -1;

	auto  early_ops_comp =   [ &](DFGNode * a, DFGNode * b) {
		return dumped_mapping[node_to_id_ [a]].t  > dumped_mapping [node_to_id_ [b]].t;
	};

	auto  late_ops_comp =    [ &](DFGNode * a, DFGNode * b) {
		return dumped_mapping[node_to_id_ [a]].t < dumped_mapping [node_to_id_ [b]].t;
	};

	std::set<DFGNode *, decltype(early_ops_comp)> early_ops(early_ops_comp);
	std::set<DFGNode *, decltype(late_ops_comp)> late_ops(late_ops_comp);
        // std::cout<<"this node"<<scheduler_order<<"\n";
	for(auto node: sortedNodeList){
		// std::cout<< node<<" order: "<<dfg_label_->at(node_to_id_[node]).schedule_order<<"\n"; 
		if(dfg_label_->at(node_to_id_[node]).schedule_order < scheduler_order  && dfg_node_placement.find(node)!= dfg_node_placement.end()) early_ops.insert(node);
		if(dfg_label_->at(node_to_id_[node]).schedule_order > scheduler_order  && dfg_node_placement.find(node)!= dfg_node_placement.end()) late_ops.insert(node);
	}
       
        if(early_ops.size() != 0){
           start_time = dfg_node_placement[*(early_ops.begin())].first->get_t();
        }
        if(late_ops.size() != 0){
           end_time = dfg_node_placement[*(late_ops.begin())].first->get_t();
        }
		return std::make_pair(start_time, end_time);
}


std::map<CGRAXMLCompile::DataPath *, int> CGRAXMLCompile::LISAMapper::getCostByComm
		( std::map<int, pos3d> & dumped_mapping, std::vector<DataPath *> candidates, DFGNode *node ){
	
	auto find_routing_resource_ = [&](DataPath * a , bool up_directoin){
		int curr_t = a->get_t();
		std::set<DataPath * > overall_routing_resource;
		std::set<DataPath * > curr_routing_resource;
		curr_routing_resource.insert(a);
		while(true){
			std::set<DataPath * > new_node;
			if(curr_routing_resource.size() ==  0){
				break;
			}

			//check whether blocked
			bool path_blocked = false;
			for(auto node: curr_routing_resource){
				int x = node->getPE()->getPosition_X(), y = node->getPE()->getPosition_Y();
				int blocked_side = 0;
				if(x-1 < 0)  blocked_side ++;
				if(x+1 >= this->cgra->get_x_max())  blocked_side ++;
				if(y-1 < 0)  blocked_side ++;
				if(y+1 >= this->cgra->get_y_max())  blocked_side ++;
				if( blocked_side >= 2) {
					break;
					path_blocked = true;
				}    

			}
			if(path_blocked){
				break;
			}

			curr_t = (curr_t + 2*cgra->get_t_max() )%cgra->get_t_max();
			int next_t ;
			if(up_directoin) {next_t = curr_t - 1;}  else {next_t = curr_t + 1 ;}
			next_t = (next_t + 2*cgra->get_t_max() )%cgra->get_t_max();

			for(auto node: curr_routing_resource){
				if( node->get_t() == curr_t){
					//find the nieghbors
					int x = node->getPE()->getPosition_X(), y = node->getPE()->getPosition_Y();
					//TODO: add routing node function
				// 	auto temp_node = getRoutingNode(mrrg, x-1, y, next_t); if(temp_node && occupancy[temp_node] == 0) new_node.insert(temp_node);
				// 	temp_node = getRoutingNode(mrrg, x+1, y, next_t); if(temp_node && occupancy[temp_node] == 0) new_node.insert(temp_node);
				// 	temp_node = getRoutingNode(mrrg, x, y-1, next_t); if(temp_node && occupancy[temp_node] == 0) new_node.insert(temp_node);
				// 	temp_node = getRoutingNode(mrrg, x, y+1, next_t); if(temp_node && occupancy[temp_node] == 0) new_node.insert(temp_node);
				// 
				}
			}
			curr_routing_resource.clear();
			for(auto node: new_node){
				if(overall_routing_resource.find(node) == overall_routing_resource.end()){
					curr_routing_resource.insert(node);
					overall_routing_resource.insert(node);
				}
				
			}
			if(up_directoin) {curr_t --;}  else {curr_t ++ ;}
			
		}
		return overall_routing_resource;
	};

	std::map<DataPath *, int> candi_routing_resource_num;
	for(auto candi : candidates){
		auto up_routing_resource = find_routing_resource_(candi, true);
		auto down_routing_resource = find_routing_resource_(candi, false);
		std::set<DataPath *> overall_routing_resource;
		for(auto r :  up_routing_resource){
			overall_routing_resource.insert(r);
		}
		for(auto r :  down_routing_resource){
			overall_routing_resource.insert(r);
		}
		candi_routing_resource_num[candi] =  overall_routing_resource.size();
	}
	return candi_routing_resource_num;
}


CGRAXMLCompile::DataPath *   CGRAXMLCompile::LISAMapper::getRoutingNode(int  x, int  y, int t){
	return NULL;
}


std::map<CGRAXMLCompile::DataPath *, int> CGRAXMLCompile::LISAMapper::getCostByAssociation
			( std::map<int, pos3d> & dumped_mapping, std::vector<DataPath *> candidates, DFGNode *dfg_node, int start_II ){
	auto & ass = dfg_label_->at(node_to_id_[dfg_node]).association;
	int earliest_execution_time = 0;
	int op_id = node_to_id_[dfg_node];

	for(auto parent: dfg_node->parents){
		if (parent->childrenOPType[dfg_node] == "PS"){
			continue;
		}
		int temp_time = dumped_mapping[parent->idx].t;
		earliest_execution_time = std::max(earliest_execution_time, temp_time);
	}
	
	int earliest_execution_time_II = (earliest_execution_time_II + 2*cgra->get_t_max() )%cgra->get_t_max();

	//TODO use latency to fix this.
	auto get_cost = [&, this ](DataPath * a) {
		int mrrgnode_II =  a->get_t();
		int total_spatial_cost = 0, total_temp_cost = 0;
		int guess_time = 0;

		if(mrrgnode_II < earliest_execution_time_II){ 
			guess_time = earliest_execution_time + ( mrrgnode_II + this->cgra->get_t_max()) - earliest_execution_time_II;
		}else{
			guess_time = earliest_execution_time + mrrgnode_II + - earliest_execution_time_II;
		}
		int mapped_ass_node = 0;
		for(auto node_ass: ass){
			int node_id = node_ass.first;
			if(dumped_mapping.find(node_id) == dumped_mapping.end()) continue;
			mapped_ass_node++;
			auto & m = dumped_mapping[node_id];
			total_spatial_cost += std::abs(std::abs(node_ass.second.first) - (std::abs(a->x_ - m.x) + std::abs(a->y_ - m.y)));
			total_temp_cost += std::abs(m.t - guess_time);
			
		}
		if(mapped_ass_node == 0) return std::make_pair(0,0);
		return std::make_pair(int(total_spatial_cost/mapped_ass_node), int(total_temp_cost/mapped_ass_node));
	};

	std::map<DataPath *, int> candi_ass_cost;
	for(auto candi: candidates){
		auto cost = get_cost(candi);
		candi_ass_cost.emplace(candi, cost.first + cost.second);
	}

	return candi_ass_cost;
}
std::map<CGRAXMLCompile::DataPath *, int> CGRAXMLCompile::LISAMapper::getCostForSameLevelNode
		( std::map<int, pos3d> & dumped_mapping, std::vector<DataPath *> candidates, DFGNode *node ){
	
}

bool CGRAXMLCompile::LISAMapper::optimizeMapping(){

}


std::map<int, pos3d> CGRAXMLCompile::LISAMapper::dumpMapping()
{
	std::map<int, pos3d> dumped_mapping;

	std::vector< std::set<DFGNode *>> latency_to_node_vector; // the index represend the latency
	for(int i = 0; i < 1000; i++){
		std::set<DFGNode *> temp;
		latency_to_node_vector.push_back( temp);
	}
	std::set<DFGNode *> unmapped_dfg_nodes;
	std::map<DFGNode *, int> node_latency;

	
	//get the latency of each operation
	int max_latency = 0;
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

	//print the mapping
	
	for(int lat = 0;  lat <= max_latency ; lat++ ){
		for(auto node: latency_to_node_vector[lat]){
			dumped_mapping.emplace(node->idx, pos3d{node->rootDP->getPE()->getPosition_X(), node->rootDP->getPE()->getPosition_Y(), lat});
		}
	}

	return dumped_mapping;
}




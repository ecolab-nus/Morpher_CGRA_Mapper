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



bool CGRAXMLCompile::LISAMapper::LISAMap( arguments arg, TimeDistInfo &tdi, int & start_II )
{
	DFG tempDFG;
	tempDFG.parseXML(arg.dfg_filename);
	// tempDFG.printDFG();
	this->dfg = & tempDFG;


	pass_lisa_arg(arg.lisa_arg);
	// set lisa contoller
	set_lisa_controller( arg );
	

	
	//here are two different ways to get labｅl: 1) training 2) GNN　inference
	if(is_training){
		do_training( arg, tdi, start_II );
	}else{
		// do GNN_inference
		assert(arg.lisa_arg.arch_name != "");
		lisa_ctrl->callGNNInference();
	}


	this->dfg_label_ = lisa_ctrl->getBestLabel();
	int curr_II  =  start_II;
	bool mapped = false;
	auto start = std::chrono::steady_clock::now();
	for(; curr_II < arg.max_II ; curr_II++){
		DFG dfg;
		dfg.parseXML(arg.dfg_filename);
		CGRA *tempCGRA  = new CGRA(arg.json_file_name, curr_II, arg.xdim, arg.ydim, this->getcongestedPortsPtr());
		tempCGRA->max_hops = arg.max_hops;
		this->getcongestedPortsPtr()->clear();
		this->getconflictedPortsPtr()->clear();
		tempCGRA->analyzeTimeDist(tdi);
		mapped = LISAMapCore(tempCGRA, &dfg);
		
		if(mapped){
			auto end = std::chrono::steady_clock::now();
			std::chrono::duration<double> elapsed_seconds = end-start;
			std::cout<<"mapping method"<<mapping_method_name<<"mapping II"<<curr_II<<" running time"<< elapsed_seconds.count()<<std::endl;
			LOG(LISA)<<"mapping:"<<dumpMappingToStr();
			this->sanityCheck();
			//mapper.assignLiveInOutAddr(&tempDFG);
			if(arg.PEType == "HyCUBE_4REG"){
				std::cout << "Printing HyCUBE Binary...\n";
				this->printHyCUBEBinary(tempCGRA);
			}
			delete tempCGRA;
			break;
		}
		delete tempCGRA;
		std::cout<<"mapping failed. Increasing II to "<<(curr_II + 1)<<std::endl;
	}
	if(!mapped){
		
		std::cout<<"trying all the II. mapping failed..........."<<std::endl;
	}
	start_II = curr_II;
	return mapped;

	
}

bool CGRAXMLCompile::LISAMapper::pass_lisa_arg(lisa_arguments la){
	this->lisa_eval_routing_priority = la.lisa_eval_routing_priority;
	this->is_training =  la.training; 
	this->max_training_iteration = la.max_training_iteration;
}

void CGRAXMLCompile::LISAMapper::set_lisa_controller( arguments arg){
	sortBackEdgePriorityASAP();
	CGRA *testCGRA  = new CGRA(arg.json_file_name, 1, arg.xdim, arg.ydim, this->getcongestedPortsPtr());
	std::set<int> node_list;
	std::map<int, std::string> node_op;
	std::vector<std::pair<int,int>> lisa_edges;
	std::vector<std::pair<int,int>> back_edges;
	for(auto node: sortedNodeList){
		node_list.insert(node->idx);
		node_op.emplace(node->idx, node->op);
		for(auto p: node->parents){
			if (p->childrenOPType[node] == "PS")
			{
				continue;
			}
			else if (p->childNextIter[node] == 1)
			{
				back_edges.push_back(std::make_pair(p->idx, node->idx));
			}else{
				lisa_edges.push_back(std::make_pair(p->idx, node->idx));
			}
		}
	}
	lisa_ctrl = std::make_shared<LISAController>( LISAController(testCGRA->get_x_max(), testCGRA->get_y_max() ,
						dfg_id, node_list, node_op, lisa_edges, back_edges));
	lisa_ctrl->setArchandDFGFileName(arg.json_file_name, arg.dfg_filename, arg.lisa_arg.arch_name);
}

void CGRAXMLCompile::LISAMapper::do_training(  arguments arg, TimeDistInfo &tdi, int start_II ){
	std::vector<perf_metric> perf_hist;
	perf_metric  best_perf = {100 , 0 , 0};
	//iterative method
	for(int traning_iteration  = 0; traning_iteration < max_training_iteration;  traning_iteration++){
		std::cout << "******************************************************* training iteration: " <<traning_iteration<<" ***************************\n";

		dfg_label_ = lisa_ctrl->getCurrLabel();

		auto start = std::chrono::steady_clock::now();
		int curr_II  =  start_II;
		bool mapped = false;

		// mapping
		{
			for(; curr_II < arg.max_II; curr_II++){
				DFG dfg;
				dfg.parseXML(arg.dfg_filename);
				CGRA *tempCGRA  = new CGRA(arg.json_file_name, curr_II, arg.xdim, arg.ydim, this->getcongestedPortsPtr());
				tempCGRA->max_hops = arg.max_hops;
				this->getcongestedPortsPtr()->clear();
				this->getconflictedPortsPtr()->clear();
				tempCGRA->analyzeTimeDist(tdi);
				mapped = LISAMapCore(tempCGRA, &dfg);
				delete tempCGRA;
				if(mapped){
					auto end = std::chrono::steady_clock::now();
					std::chrono::duration<double> elapsed_seconds = end-start;
					std::cout<<"training iteration:"<< traning_iteration<<"\t II:"<<curr_II<<"\t running time:"<< elapsed_seconds.count()<<"\n";
					std::cout<<" mapping:"<<dumpMappingToStr();
					
					perf_metric this_iter_perf { curr_II, getCost() , elapsed_seconds.count() };
					bool is_best = false;
							
					if(traning_iteration == 0 || this_iter_perf < best_perf) {
							best_perf = this_iter_perf;
							is_best = true;
					}
					int max_latency;
					auto dumped_mapping = dumpMapping(max_latency);
					lisa_ctrl->passMapping(is_best,dumped_mapping, max_latency, this_iter_perf);
					std::cout<<"current label: "<<lisa_ctrl->DFGLabelToStr(*(lisa_ctrl->getCurrLabel()))<<"\n";
					break;
				}
			}
		}


		

		lisa_ctrl->generateCombinedBestLabelHistorically(best_perf);
		LOG(LISA)<<"best label"<<lisa_ctrl->labelToStrForGNNDataSet(*(lisa_ctrl->getBestLabel()));
		
		mapping_method_name = "t-LISA";
	}
}

bool CGRAXMLCompile::LISAMapper::LISAMapCore(CGRA *cgra, DFG *dfg){
	

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

	bool mapSuccess = false;

	

	std::cout << "***************************LISA MAP begin***************************\n";

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

bool CGRAXMLCompile::LISAMapper::initMap()
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
		MapHeader  << "current node = " << node->idx
					<< ",op = " << node->op
					<< ",unmapped nodes = " << unmappedNodes.size()
					<< ",mapped nodes = " << mappedNodes.size()
					<< ",freeMemNodes = " << cgra->freeMemNodes
					<< ",unmappedMemNodes = " << dfg->unmappedMemOps
					<< ",II = " << cgra->get_t_max()
					<< ",btCredits = " << backTrackCredits
					<< ",CGRA=" << this->cgra->getCGRAName()
					<< ",MaxHops=" << this->cgra->max_hops
					<< ",BB = " << node->BB
					<< ",mutexPathEn = " << this->enableMutexPaths
					<< "\n";

		LOG(ROUTE) << MapHeader.str();


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

		LOG(ROUTE) << "estimatedRouteInfo[node].size = " << estimatedRouteInfo[node].size() << "\n";
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
			node->SAClear(this->dfg);
		}
	}

	return true;
}

float CGRAXMLCompile::LISAMapper::inner_map()
{
	int accepted_number = 0;

	for (int i = 0; i < movement_in_each_temp; i++)
	{
		std::stringstream congestion_detail;
		LOG(LISA) << "************ NO." << i << " movement, unmapped nodes:" << getNumberOfUnmappedNodes() << ", congestion:" << getCongestionNumber(congestion_detail) 
			<< ", congestion:" << getConflictNumber(congestion_detail);
		LOG(ROUTE)<<congestion_detail.str();

		// select DFG node to unmap. If this node is not mapped yet, will select a parent to unmap as well.
		std::vector<DFGNode *> moved_nodes;
		auto selected_dfg_node = selectDFGNodeToUnmap();

		// assert(dfg_node_placement.find(selected_dfg_node)!= dfg_node_placement.end());
		std::map<DFGNode *, std::pair<DataPath *, int>> old_dfg_node_placement;
		old_dfg_node_placement.insert(dfg_node_placement.begin(), dfg_node_placement.end());
		std::map<dfg_data, std::vector<LatPort>> old_data_routing_path;
		old_data_routing_path.insert(data_routing_path.begin(), data_routing_path.end());

		LOG(LISA)<<"select DFG node "<< selected_dfg_node->idx <<" to unmap";
		// start map
		if (dfg_node_placement.find(selected_dfg_node) != dfg_node_placement.end()){
			LOG(LISA)<<"current placement:" << selected_dfg_node->rootDP->getFullName();
			clearNodeMapping(selected_dfg_node);
		}else{
			LOG(LISA)<<"this DFG node is not placed yet" ;
		}
		
		auto dp_candidate = getLISADPCandidate(selected_dfg_node);
		LOG(LISA) << "map this DFG node:" << selected_dfg_node->idx << " op:" << selected_dfg_node->op << "to pe:" << dp_candidate->getPE()->getName() ;
		bool route_succ = SARoute(selected_dfg_node, dp_candidate);
		moved_nodes.push_back(selected_dfg_node);


		// decide accept or node
		bool accept = false;
		int attempted_cost = getCost();
		if (route_succ)
		{
			// if not route success, then nothing changes and should restore mapping
			accept = whetherAcceptNewMapping(attempted_cost, curr_cost, curr_temp);
		}

		LOG(LISA) << "accept " << (accept? "ture" : "false") << " route_succ:" << route_succ << " curr_cost:" << curr_cost << " attepmted cost:" << attempted_cost << "\n";

		if (accept)
		{
			// update overuse
			//  save the mapping state
			curr_cost = attempted_cost;
			accepted_number++;

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


CGRAXMLCompile::DataPath *  CGRAXMLCompile::LISAMapper::getLISADPCandidate(DFGNode *dfg_node, int accepted  , int total_tried , int num_swap ){
	assert(dfg_label_->find(dfg_node->idx) != dfg_label_->end() );
	auto & dfg_node_label = dfg_label_->at(dfg_node->idx);

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
	auto & ass = dfg_label_->at(dfg_node->idx).association;
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

//TODO: add this function
std::vector<CGRAXMLCompile::DataPath*> CGRAXMLCompile::LISAMapper::getCandidateByIIConstraint( int start_II, int end_II, DFGNode * node){
	std::vector<DataPath*> temp;
	return temp;
}

CGRAXMLCompile::DataPath *  CGRAXMLCompile::LISAMapper::getCloseRandomDP(DFGNode* dfg_node, DataPath * old_dp, int max_physical_dis,  int max_temp_dis ){
	
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
std::pair<int,int> CGRAXMLCompile::LISAMapper::getIntervalByScheduleOrder( std::map<int, pos3d> & dumped_mapping, DFGNode * dfg_node, int scheduler_order ){
	int start_time = -1, end_time = -1;

	auto  early_ops_comp =   [ &](DFGNode * a, DFGNode * b) {
		return dumped_mapping[a->idx].t  > dumped_mapping [b->idx].t;
	};

	auto  late_ops_comp =    [ &](DFGNode * a, DFGNode * b) {
		return dumped_mapping[a->idx].t < dumped_mapping [b->idx].t;
	};

	std::set<DFGNode *, decltype(early_ops_comp)> early_ops(early_ops_comp);
	std::set<DFGNode *, decltype(late_ops_comp)> late_ops(late_ops_comp);
        // std::cout<<"this node"<<scheduler_order<<"\n";
	for(auto node: sortedNodeList){
		// std::cout<< node<<" order: "<<dfg_label_->at(node_to_id_[node]).schedule_order<<"\n"; 
		if(dfg_label_->at(node->idx).schedule_order < scheduler_order  && dfg_node_placement.find(node)!= dfg_node_placement.end()) early_ops.insert(node);
		if(dfg_label_->at(node->idx).schedule_order > scheduler_order  && dfg_node_placement.find(node)!= dfg_node_placement.end()) late_ops.insert(node);
	}
       
        if(early_ops.size() != 0){
           start_time = dfg_node_placement[*(early_ops.begin())].first->get_t();
        }
        if(late_ops.size() != 0){
           end_time = dfg_node_placement[*(late_ops.begin())].first->get_t();
        }
		return std::make_pair(start_time, end_time);
}


//This function is not used in LISA
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
					//TODO: add routing node function. Probably will not fix it as this is not used by LISA
					
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
	auto & ass = dfg_label_->at(dfg_node->idx).association;
	int earliest_execution_time = 0;
	int op_id = dfg_node->idx;

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
			total_spatial_cost += std::abs(std::abs(node_ass.second.first) - (std::abs(a->getPE()->getPosition_X() - m.x)
									 + std::abs(a->getPE()->getPosition_Y() - m.y)));
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
		( std::map<int, pos3d> & dumped_mapping, std::vector<DataPath *> candidates, DFGNode *dfg_node ){
	
	int node_id = dfg_node->idx;
	std::map<DataPath *, int> node_cost;
	for(auto node: candidates){
		node_cost[node] = 0;
	}

	// if(! lisa_ctrl->isStartNode(node_id)) return node_cost;

	auto relevant_samv_level_nodes = lisa_ctrl->getSameLevelNodes(node_id);
	std::set<int> mapped_sameLevel_nodes;
	for(auto rele_node: relevant_samv_level_nodes){
		if(dumped_mapping.find(rele_node)!= dumped_mapping.end()){
			mapped_sameLevel_nodes.insert(rele_node);
		}
	}
	// if(mapped_sameLevel_nodes.size()== 0) assert(false);
	auto & dist_label = dfg_label_->at(node_id).sameLevel_node_distance; 
	auto cal_cost = [&](DataPath * a){
		if(mapped_sameLevel_nodes.size()==0) return 0;
		int total_cost = 0;
		for(auto node: mapped_sameLevel_nodes){
			auto & m = dumped_mapping[node];
			int dist = std::abs(a->getPE()->getPosition_X() - m.x) + std::abs(a->getPE()->getPosition_Y() - m.y);
			total_cost += std::abs(dist_label[node] - dist);
		}
		return (int)(total_cost/(mapped_sameLevel_nodes.size()));
	};

	for(auto node: candidates){
		node_cost[node] = cal_cost(node);
	}

	return node_cost;
}

// this is for training. After a successful mapping, this process still do SA to minimize the cost
bool CGRAXMLCompile::LISAMapper::optimizeMappingToMinimizeCost(){
	int total_accepted = 0;

	for (int i = 0; i < movement_in_each_temp; i++)
	{
		std::stringstream congestion_detail;
		LOG(LISA) << "************ NO." << i << " movement, unmapped nodes:" << getNumberOfUnmappedNodes() << ", congestion:" << getCongestionNumber(congestion_detail) 
			<< ", congestion:" << getConflictNumber(congestion_detail);
		LOG(ROUTE)<<congestion_detail.str();

		// select DFG node to unmap. If this node is not mapped yet, will select a parent to unmap as well.
		std::vector<DFGNode *> moved_nodes;
		auto selected_dfg_node = selectDFGNodeToUnmap();
		
		std::map<DFGNode *, std::pair<DataPath *, int>> old_dfg_node_placement;
		old_dfg_node_placement.insert(dfg_node_placement.begin(), dfg_node_placement.end());
		std::map<dfg_data, std::vector<LatPort>> old_data_routing_path;
		old_data_routing_path.insert(data_routing_path.begin(), data_routing_path.end());

		LOG(LISA)<<"select DFG node "<< selected_dfg_node->idx <<" to unmap";
		// start map
		auto dp_candidate = getCloseRandomDP(selected_dfg_node, dfg_node_placement[selected_dfg_node].first,2, 2);
		if (dfg_node_placement.find(selected_dfg_node) != dfg_node_placement.end()){
			LOG(LISA)<<"current placement:" << selected_dfg_node->rootDP->getFullName();
			clearNodeMapping(selected_dfg_node);
		}else{
			assert(false);
		}
		
		LOG(LISA) << "map this DFG node:" << selected_dfg_node->idx << " op:" << selected_dfg_node->op << "to pe:" << dp_candidate->getPE()->getName() ;
		bool route_succ = SARoute(selected_dfg_node, dp_candidate);
		moved_nodes.push_back(selected_dfg_node);

		

		// decide accept or node
		bool accept = false;
		int attempted_cost = getCost();
		if (route_succ)
		{
			// if not route success, then nothing changes and should restore mapping
			accept = whetherAcceptNewMapping(attempted_cost, curr_cost, curr_temp);
		}

		LOG(LISA) << "accept " << (accept? "ture" : "false") << " route_succ:" << route_succ << " curr_cost:" << curr_cost << " attepmted cost:" << attempted_cost << "\n";

		if (accept)
		{
			// update overuse
			//  save the mapping state
			curr_cost = attempted_cost;
			total_accepted++;

			old_dfg_node_placement.clear();
			old_data_routing_path.clear();

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

	return true;

}


std::map<int, pos3d> CGRAXMLCompile::LISAMapper::dumpMapping(int & max_latency)
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
	max_latency = 0;
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




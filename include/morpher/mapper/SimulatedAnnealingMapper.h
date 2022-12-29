/*
 * SimulatedAnnealingMapper.h
 * Since PathFinderMapper has implemented many functions and other source file also use PathFinderMapper, better to create a mapper which bases
 * on PathFinderMapper
 */

#include <morpher/mapper/PathFinderMapper.h>
#include <morpher/arch/DataPath.h>
#include <morpher/util/debug.h>

#include <string>
#include <random>
#include <algorithm> 
#include <chrono>
#include <sys/time.h>
#ifndef SAMAPPER_H_
#define SAMAPPER_H_

namespace CGRAXMLCompile
{


using port_util = std::pair<int,int>;
using dfg_data = std::pair<DFGNode*,DFGNode*>; // <src, des>

class SAMapper : public PathFinderMapper
{
public:
	SAMapper(std::string fName) : PathFinderMapper(fName){
		mapping_method_name  = "SA";
	};

	bool SAMap(CGRA *cgra, DFG *dfg);

	bool initMap_with_PathFinder();
	bool initMap_by_random_way();
	bool Route(DFGNode *node, std::priority_queue<dest_with_cost> &estimatedRoutes, DFGNode **failedNode);

	float inner_map();
	DFGNode* selectDFGNodeToUnmap();
	DFGNode* selectAParentForDFGNode(DFGNode* target_node);
	bool clearNodeMapping(DFGNode * node, bool violenceClear = false);
	std::vector<DataPath*> getRandomDPCandidate(DFGNode *node);
	bool SARoute(DFGNode *node, DataPath * candidate);
	int getCost();
	bool restoreMapping(DFGNode *node, 	std::map<DFGNode*, std::pair<DataPath*, int>> & dfg_node_placement, 
										std::map<dfg_data, std::vector<LatPort>> & data_routing_path);
	
	
	bool restoreMapping(std::vector<DFGNode*> nodes, 	std::map<DFGNode*, std::pair<DataPath*, int>> & dfg_node_placement, 
										std::map<dfg_data, std::vector<LatPort>> & data_routing_path);
	int getCongestionNumber(std::stringstream & congestion_detail);
	int getCongestionNumber(){
		std::stringstream ss;
		return getCongestionNumber(ss);
	};
	std::vector<dfg_data> find_congestion_paths(); 
	bool  does_node_cause_congestion(DFGNode* node, std::vector<dfg_data>& congested_path); 

	int getConflictNumber(std::stringstream & congestion_detail);
	int getConflictNumber(){
		std::stringstream ss;
		return getConflictNumber(ss);
	};
	int getPortUsage();
	int getNumberOfUnmappedNodes();
	int checkAnyUnroutedEdge();
	std::string mapStatus();
	bool isCurrMappingValid(){
		int overuse_number = getCongestionNumber();
		int conflict_number = getConflictNumber();
		int unmapped_node_number = getNumberOfUnmappedNodes();
		int unrouted_edge = checkAnyUnroutedEdge();
		// LOG(SA)<<"overuse:"<<overuse_number<<" unmaped node:"<<unmapped_node_number
		// <<" unrouted_edge:"<<unrouted_edge<<" conflict_number:"<<conflict_number;
		if(unmapped_node_number == 0 && conflict_number == 0 && unrouted_edge > 0) {
			return false;
		};
		
		if(unmapped_node_number == 0 && conflict_number == 0 && unrouted_edge == 0 && overuse_number == 0){
			return true;
		}else{
			return false;
		}
	}

	float updateTemperature(float t, float acceptance_rate)
	{
			if (acceptance_rate > 0.96)
			{
					return t * 0.5;
			}
			else if (acceptance_rate > 0.8)
			{
					return t * 0.9;
			}
			else if (acceptance_rate > 0.15)
			{
					return t * 0.98;
			}
			else
			{
					return t * 0.95;
			}
	}

	static inline double getcurrenttime()
	{
			struct timeval t;
			gettimeofday(&t, NULL);
			return t.tv_sec + t.tv_usec * 0.000001;
	}

	bool whetherAcceptNewMapping(float new_cost, float old_cost, float temperature, int accepted = 1, int tried = 1)
	{		
		if(is_mapping_valid){
			if (isCurrMappingValid()){
				return true;
			}else{
				return false;
			}
		}
		if (new_cost < old_cost)
				return true;
		tried = std::max(1, tried);
		float accept_rate = accepted / tried;
		float probability = exp(-(new_cost - old_cost) * old_cost / 100 / accept_rate / temperature);

		return probability > ((float)rand() / (float)RAND_MAX);
	}
	//	bool LeastCostPathAstar(Port* start, Port* end, DataPath* endDP, std::vector<Port*>& path, int& cost, DFGNode* node, std::map<Port*,std::set<DFGNode*>>& mutexPaths, DFGNode* currNode);


	int total_accepted_number = 0;
protected:

	int congest_num;
	int routing_num;

	float maximum_temp = 200;
	float minimim_temp = 10;
	float curr_temp;
	int movement_in_each_temp = 100;
 	float cold_accept_rate = 0.01;
	std::map<DFGNode*, std::pair<DataPath*, int>> dfg_node_placement;
	std::map<dfg_data, std::vector<LatPort>> data_routing_path;
	int maximum_routing_iteration = 3;

	int try_number_for_random_init_mapping = 10;

	int curr_cost;

	int backtrack_credit = 3; // this is different from the PathFinder one.

	bool freeze_mapping = false;
	bool is_mapping_valid = false;

	
	
	
};

} /* namespace CGRAXMLCompile */

#endif /* SAMAPPER_H_ */

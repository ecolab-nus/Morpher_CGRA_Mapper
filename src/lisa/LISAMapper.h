/*
 * SimulatedAnnealingMapper.h
 * Since PathFinderMapper has implemented many functions and other source file also use PathFinderMapper, better to create a mapper which bases
 * on PathFinderMapper
 */

#include "../SimulatedAnnealingMapper.h"
#include "../PathFinderMapper.h"
#include "../DataPath.h"
#include "../debug.h"

#include <string>
#include <random>
#include <algorithm> 
#include <chrono>
#include <sys/time.h>
#ifndef LISAMAPPER_H_
#define LISAMAPPER_H_

namespace CGRAXMLCompile
{


using port_util = std::pair<int,int>;
using dfg_data = std::pair<DFGNode*,DFGNode*>; // <src, des>

class LISAMapper : public SAMapper
{
public:
	LISAMapper(std::string fName) : SAMapper(fName){
		mapping_method_name  = "LISA";
	};

	bool LISAMap(CGRA *cgra, DFG *dfg);

	bool initMap();
	bool Route(DFGNode *node, std::priority_queue<dest_with_cost> &estimatedRoutes, DFGNode **failedNode);

	float inner_map();
	DFGNode* selectDFGNodeToUnmap();
	DFGNode* selectAParentForDFGNode(DFGNode* target_node);
	bool clearNodeMapping(DFGNode * node);
	std::vector<DataPath*> getRandomDPCandidate(DFGNode *node);
	bool SARoute(DFGNode *node, DataPath * candidate);
	int getCost();
	bool restoreMapping(DFGNode *node, 	std::map<DFGNode*, std::pair<DataPath*, int>> & dfg_node_placement, 
										std::map<dfg_data, std::vector<LatPort>> & data_routing_path);
	
	
	int getCongestionNumber(std::stringstream & congestion_detail);
	int getCongestionNumber(){
		std::stringstream ss;
		return getCongestionNumber(ss);
	};

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
		if(unmapped_node_number == 0 && conflict_number == 0 && unrouted_edge > 0) assert(false && "some edges are not routed!");
		
		if(unmapped_node_number == 0 && conflict_number == 0  && overuse_number == 0){
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

	bool whetherAcceptNewMapping(float new_cost, float old_cost, float temperature)
	{
			if (new_cost < old_cost)
					return true;

			float probability = exp(-(new_cost - old_cost) / temperature);

			return probability > ((float)rand() / (float)RAND_MAX);
	}
	//	bool LeastCostPathAstar(Port* start, Port* end, DataPath* endDP, std::vector<Port*>& path, int& cost, DFGNode* node, std::map<Port*,std::set<DFGNode*>>& mutexPaths, DFGNode* currNode);



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

	int curr_cost;

	int backtrack_credit = 3; // this is different from the PathFinder one.



	

	
};

} /* namespace CGRAXMLCompile */

#endif /* LISAMAPPER_H_ */

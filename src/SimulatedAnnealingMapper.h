/*
 * SimulatedAnnealingMapper.h
 * Since PathFinderMapper has implemented many functions and other source file also use PathFinderMapper, better to create a mapper which bases
 * on PathFinderMapper
 */

#include "PathFinderMapper.h"
#include "DataPath.h"
#include "debug.h"

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
using dfg_data = std::pair<DFGNode*,DFGNode*>; // <parent, child>

class SAMapper : public PathFinderMapper
{
public:
	SAMapper(std::string fName) : PathFinderMapper(fName){

										  };

	bool SAMap(CGRA *cgra, DFG *dfg);
	float inner_map();
	bool initMap();
	port_util getRoutingAndCongestUtil();
	std::vector<DataPath*> getRandomCandidate(DFGNode *node);
	bool restoreMapping(DFGNode *node, 	std::map<DFGNode*, std::pair<DataPath*, int>> & dfg_node_placement, 
										std::map<dfg_data, std::vector<LatPort>> & data_routing_path);
	bool SARoute(DFGNode *node, DataPath * candidate);
	int getCost();
	int getCongestionNumber();
	int getPortUsage();

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

	bool acceptNewMapping(float new_cost, float old_cost, float temperature)
	{
			if (new_cost < old_cost)
					return true;

			float probability = exp(-(new_cost - old_cost) / temperature);

			return probability > ((float)rand() / (float)RAND_MAX);
	}
	//	bool LeastCostPathAstar(Port* start, Port* end, DataPath* endDP, std::vector<Port*>& path, int& cost, DFGNode* node, std::map<Port*,std::set<DFGNode*>>& mutexPaths, DFGNode* currNode);




private:
	int congest_num;
	int routing_num;

	float maximum_temp;
	float minimim_temp;
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

#endif /* SAMAPPER_H_ */

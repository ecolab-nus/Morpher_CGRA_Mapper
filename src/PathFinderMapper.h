/*
 * PathFinderMapper.h
 *
 *  Created on: 31 Mar 2018
 *      Author: manupa
 */

#include "HeuristicMapper.h"
#include <string>

#ifndef PATHFINDERMAPPER_H_
#define PATHFINDERMAPPER_H_

namespace CGRAXMLCompile {

class PathFinderMapper : public HeuristicMapper {
public:
	PathFinderMapper(std::string fName) : HeuristicMapper(fName){


	};

	bool Map(CGRA* cgra, DFG* dfg);
//	bool LeastCostPathAstar(Port* start, Port* end, DataPath* endDP, std::vector<Port*>& path, int& cost, DFGNode* node, std::map<Port*,std::set<DFGNode*>>& mutexPaths, DFGNode* currNode);
	bool LeastCostPathAstar(LatPort start, LatPort end, DataPath* endDP, std::vector<LatPort>& path, int& cost, DFGNode* node, std::map<Port*,std::set<DFGNode*>>& mutexPaths, DFGNode* currNode);
	bool estimateRouting(DFGNode* node, std::priority_queue<dest_with_cost>& estimatedRoutes, DFGNode** failedNode);
	bool Route(DFGNode* node, std::priority_queue<dest_with_cost>& estimatedRoutes, DFGNode** failedNode);
	int calculateCost(LatPort src, LatPort next_to_src, LatPort dest);
	void assignPath(DFGNode* src, DFGNode* dest, std::vector<LatPort> path);

	bool updateCongestionCosts(int iter);
	bool clearCurrMapping();
	std::map<Port*,std::set<DFGNode*>>* getcongestedPortsPtr(){return &congestedPorts;}
	std::map<Port*,std::set<DFGNode*>>* getconflictedPortsPtr(){return &conflictedPorts;}

	bool checkConflictedPortCompatibility();
	bool checkRegALUConflicts();
	bool checkDPFree(DataPath* dp, DFGNode* node, int& penalty);

	bool updateConflictedTimeSteps(int timeStep, int conflicts);
	int getTimeStepConflicts(int timeStep);

	void sortBackEdgePriority();
	std::ofstream congestionInfoFile;

private:
	std::map<Port*,std::set<DFGNode*>> congestedPorts;
	std::map<Port*,std::set<DFGNode*>> conflictedPorts;
	int maxIter = 30;


	std::map<int,int> conflictedTimeStepMap;


};

} /* namespace CGRAXMLCompile */

#endif /* PATHFINDERMAPPER_H_ */

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
	PathFinderMapper(std::string fName) : HeuristicMapper(fName){};

	bool Map(CGRA* cgra, DFG* dfg);
	bool LeastCostPathAstar(Port* start, Port* end, std::vector<Port*>& path, int& cost, DFGNode* node, std::map<Port*,std::set<DFGNode*>>& mutexPaths, DFGNode* currNode);
	bool estimateRouting(DFGNode* node, std::priority_queue<dest_with_cost>& estimatedRoutes, DFGNode** failedNode);
	bool Route(DFGNode* node, std::priority_queue<dest_with_cost>& estimatedRoutes, DFGNode** failedNode);
	int calculateCost(Port* src, Port* next_to_src, Port* dest);
	void assignPath(DFGNode* src, DFGNode* dest, std::vector<Port*> path);

	bool updateCongestionCosts();
	bool clearCurrMapping();
	std::map<Port*,std::set<DFGNode*>>* getcongestedPortsPtr(){return &congestedPorts;}

private:
	std::map<Port*,std::set<DFGNode*>> congestedPorts;
	int maxIter = 15;

};

} /* namespace CGRAXMLCompile */

#endif /* PATHFINDERMAPPER_H_ */



// this is for mogpher-liht
#include <morpher/mapper/HeuristicMapper.h>
#include <morpher/mapper/PathFinderMapper.h>
#include <morpher/util/debug.h>
#include <string>

#ifndef QUICKMAPPER_H_
#define QUICKMAPPER_H_

namespace CGRAXMLCompile
{




class QuickMapper : public PathFinderMapper
{
public:
	QuickMapper(std::string fName) : PathFinderMapper(fName){
		mapping_method_name  = "QuickMapper";
										  };

	bool QuickMap(CGRA *cgra, DFG *dfg);

	bool QuickRoute(DFGNode *node, std::priority_queue<dest_with_cost> &estimatedRoutes, DFGNode **failedNode);
	bool quickEstimateRouting(DFGNode *node, std::priority_queue<dest_with_cost> &estimatedRoutes, DFGNode **failedNode);
	bool quickLeastCostPathAstar(LatPort start, LatPort end, DataPath *endDP, std::vector<LatPort> &path, int &cost, DFGNode *node, std::map<Port *, std::set<DFGNode *>> &mutexPaths, DFGNode *currNode);


	

protected:


// private:
	
	
};

} /* namespace CGRAXMLCompile */

#endif /* QUICKMAPPER_H_ */

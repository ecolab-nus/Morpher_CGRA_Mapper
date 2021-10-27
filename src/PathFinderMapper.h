/*
 * PathFinderMapper.h
 *
 *  Created on: 31 Mar 2018
 *      Author: manupa
 */

#include "HeuristicMapper.h"
#include "bfs_shortes_path.hpp"
#include <string>

#ifndef PATHFINDERMAPPER_H_
#define PATHFINDERMAPPER_H_

namespace CGRAXMLCompile
{

#define LARGE_VALUE 100000000

struct beParentInfo
{
	DFGNode *beParent;
	int lat;
	int downStreamOps;

	bool dsMEMfound = false;
	int uptoMEMops = -1;
	bool isLDST = false;

	bool operator<(const beParentInfo &other) const
	{
		return this->beParent < other.beParent;
	}
};

struct InsFormat{
	std::string easto;
	std::string southo;
	std::string westo;
	std::string northo;
	std::string alu_i1;
	std::string alu_i2;
	std::string alu_p;

	std::string treg_we;

	std::string east_reg_we;
	std::string south_reg_we;
	std::string west_reg_we;
	std::string north_reg_we;

	std::string east_reg_bypass;
	std::string south_reg_bypass;
	std::string west_reg_bypass;
	std::string north_reg_bypass;

	std::string opcode;
	std::string constant;
	std::string constant_valid;
	std::string negated_predicate;
};


class PathFinderMapper : public HeuristicMapper
{
public:
	PathFinderMapper(std::string fName) : HeuristicMapper(fName){
		 for(int y=0;y<m;y++)
		    {
		        for(int x=0;x<n;x++) map_cgra[x][y]=0;
		    }

										  };

	bool Map(CGRA *cgra, DFG *dfg, std::ofstream& sumFile);
	//	bool LeastCostPathAstar(Port* start, Port* end, DataPath* endDP, std::vector<Port*>& path, int& cost, DFGNode* node, std::map<Port*,std::set<DFGNode*>>& mutexPaths, DFGNode* currNode);
	bool LeastCostPathAstar(LatPort start, LatPort end, DataPath *endDP, std::vector<LatPort> &path, int &cost, DFGNode *node, std::map<Port *, std::set<DFGNode *>> &mutexPaths, DFGNode *currNode);
    string AstarShortestPath(LatPort start, LatPort end);
    int BFSShortestDistance(int  xStart,int  yStart,int  xFinish,int  yFinish, CGRA* cgra);
	bool estimateRouting(DFGNode *node, std::priority_queue<dest_with_cost> &estimatedRoutes, DFGNode **failedNode);
	int predictiveRoute(DFGNode *node,
						DataPath *dest,
						const std::map<DFGNode *, Port *> routingSourcesIn,
						const std::map<DFGNode *, DataPath *> mappedNodesIn,
						std::map<DFGNode *, Port *> &routingSourcesOut,
						std::map<DFGNode *, DataPath *> &mappedNodesOut);

	bool Route(DFGNode *node, std::priority_queue<dest_with_cost> &estimatedRoutes, DFGNode **failedNode);
	int calculateCost(LatPort src, LatPort next_to_src, LatPort dest);
	void assignPath(DFGNode *src, DFGNode *dest, std::vector<LatPort> path);

	bool updateCongestionCosts(int iter);
	bool clearCurrMapping();
	std::map<Port *, std::set<DFGNode *>> *getcongestedPortsPtr() { return &congestedPorts; }
	std::map<Port *, std::set<DFGNode *>> *getconflictedPortsPtr() { return &conflictedPorts; }

	bool checkConflictedPortCompatibility();
	bool checkRegALUConflicts();
	bool checkDPFree(DataPath *dp, DFGNode *node, int &penalty);

	bool updateConflictedTimeSteps(int timeStep, int conflicts);
	int getTimeStepConflicts(int timeStep);

	void sortBackEdgePriorityASAP();
	void sortBackEdgePriorityALAP();
	std::ofstream congestionInfoFile;

	void addPseudoEdgesOrphans(DFG *dfg);

	std::vector<DFGNode *> getLongestDFGPath(DFGNode *src, DFGNode *dest);
	int getFreeMEMPeDist(PE *currPE);

	bool canExitCurrPE(LatPort p);
	static bool checkMEMOp(string op);
	void setMaxIter(int m){maxIter = m;}

	bool Check_DFG_CGRA_Compatibility();

	void UpdateVariableBaseAddr();

	void printHyCUBEBinary(CGRA* cgra);
	void printBinFile(const std::vector<std::vector<std::vector<InsFormat>>>& insFArr, std::string fName, CGRA* cgra);
	std::string skip_inter_or_intra;//which edges to skip INTER (includes backedges) or INTRA

	//https://code.activestate.com/recipes/577457-a-star-shortest-path-algorithm/

	const int n=60; // horizontal size of the map
	const int m=60; // vertical size size of the map
	 int map_cgra[60][60];
	 int closed_nodes_map[60][60]; // map of closed (tried-out) nodes
	 int open_nodes_map[60][60]; // map of open (not-yet-tried) nodes
	 int dir_map[60][60]; // map of directions
	 int dir=4; // number of possible directions to go at any position
	// if dir==4
	 int dx[4]={1, 0, -1, 0};
	 int dy[4]={0, 1, 0, -1};
	// if dir==8
	//static int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1};
	//static int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1};

	std::set<std::pair<int,int>> astar_abstract_open_set; // open set generated from abstract aster routing
	bool open_set_limit_1 = false;// openset contains the neighbors of original abstract astart search openset
	bool open_set_limit_2 = false;//limit openset to original abstract astart search openset (this cannot be true if limit 1 is false)
	std::map<std::pair<int,int>, int> shortest_dist_map;
	//bool failed_due_to_estimate_routing = false;

	int num_of_congestions = 0;
	int num_of_conflicts = 0;
	time_t start_time;
	int maxIterationTime = 6;//hours


#ifdef SIM_ANNEAL
#else
private:
#endif
	std::map<Port *, std::set<DFGNode *>> congestedPorts;
	std::map<Port *, std::set<DFGNode *>> conflictedPorts;
	int maxIter = 30;

	std::map<int, int> conflictedTimeStepMap;

	std::set<DFGNode *> RecPHIs;
	int getlatMinStartsPHI(const DFGNode *currNode, const std::map<DFGNode *, std::vector<Port *>> &possibleStarts);
	std::set<DFGNode *> getElders(DFGNode *node);

	std::map<BackEdge, std::set<DFGNode *>> RecCycles;
	std::map<BackEdge, std::set<DFGNode *>> RecCyclesLS;
	int getMaxLatencyBE(DFGNode *node, std::map<DataPath *, beParentInfo> &beParentDests, int &downStreamOps);
	std::vector<DataPath *> modifyMaxLatCandDest(std::map<DataPath *, int> candDestIn, DFGNode *node, bool &changed);

	void GetAllSupportedOPs(Module* currmod, unordered_set<string>& supp_ops, unordered_set<string>& supp_pointers);
};

} /* namespace CGRAXMLCompile */

#endif /* PATHFINDERMAPPER_H_ */

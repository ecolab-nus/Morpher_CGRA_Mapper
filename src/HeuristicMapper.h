/*
 * HeuristicMapper.h
 *
 *  Created on: 28 Feb 2018
 *      Author: manupa
 */

#ifndef HEURISTICMAPPER_H_
#define HEURISTICMAPPER_H_

#include "DFG.h"
#include "CGRA.h"
#include "DataPath.h"
#include <queue>
#include <assert.h>
#include <iostream>
#include <fstream>


namespace CGRAXMLCompile {

//struct definitions

struct dest_child_with_cost{
	DFGNode* child;
	Port* childDest;
	Port* startPort;
	int cost;

	dest_child_with_cost(DFGNode* child, Port* childDest, Port* startPort, int cost) :
		child(child),childDest(childDest), startPort(startPort), cost(cost){}
	bool operator<(const dest_child_with_cost& rhs) const{
		return cost > rhs.cost;
	}
};


struct cand_src_with_cost{
			Port* src;
			Port* dest;
			int cost;
			cand_src_with_cost(Port* src, Port* dest, int cost):src(src), dest(dest), cost(cost){}

			bool operator<(const cand_src_with_cost& rhs) const{
				return cost > rhs.cost;
			}
		};

struct parent_cand_src_with_cost{
	DFGNode* parent;
	std::priority_queue<cand_src_with_cost> cswc;
	int cost;
	parent_cand_src_with_cost(DFGNode* parent, std::priority_queue<cand_src_with_cost> cswc) : parent(parent), cswc(cswc){
		cost = cswc.top().cost;
	}

	bool operator<(const parent_cand_src_with_cost& rhs) const{
		return this->cost > rhs.cost;
	}
};

struct dest_with_cost{
//	std::map<DFGNode*,std::priority_queue<cand_src_with_cost>> parentStartLocs;
	std::priority_queue<parent_cand_src_with_cost> parentStartLocs;
	std::priority_queue<dest_child_with_cost> alreadyMappedChilds;
	int bestCost;
	DataPath* dest;
	dest_with_cost(){}
	dest_with_cost(std::priority_queue<parent_cand_src_with_cost> parentStartLocs,
				   std::priority_queue<dest_child_with_cost> alreadyMappedChilds,
				   DataPath* dest, int cost=0) :
		parentStartLocs(parentStartLocs),alreadyMappedChilds(alreadyMappedChilds), dest(dest){
		bestCost = sumBestCosts();
	}


	int sumBestCosts(){
		int cost=0;
//		for(std::pair<DFGNode*,std::priority_queue<cand_src_with_cost>> pair : parentStartLocs){
//			assert(!pair.second.empty());
//			cost+=pair.second.top().cost;
//		}
		std::priority_queue<parent_cand_src_with_cost> parentStartLocsCopy = parentStartLocs;
		while(!parentStartLocsCopy.empty()){
			parent_cand_src_with_cost pcswc = parentStartLocsCopy.top();
			parentStartLocsCopy.pop();
			cost+=pcswc.cswc.top().cost;
		}

//		std::cout << "sumBestCosts :: alreadyMappedChilds size=" << alreadyMappedChilds.size() << "\n";
		std::priority_queue<dest_child_with_cost> alreadyMappedChildCopy = alreadyMappedChilds;
		while(!alreadyMappedChildCopy.empty()){
			dest_child_with_cost dcwc = alreadyMappedChildCopy.top();
			alreadyMappedChildCopy.pop();
			cost+=dcwc.cost;
		}
//		std::cout << "sumBestCosts :: alreadyMappedChilds size=" << alreadyMappedChilds.size() << "\n";

		if(cost==0){
			int freePorts=0;
			for(Port &p : dest->getPE()->outputPorts){
				Module* parent = dest->getPE()->getParent();
				if(parent->connections[&p].empty()) continue;
				if(p.node==NULL){
					freePorts++;
				}
			}
			cost = cost + (15 - freePorts)*100;
			std::cout << dest->getPE()->getName() << ",cost=" << cost << "\n";
		}

		return cost;
	}

	bool operator<(const dest_with_cost& rhs) const{
		return bestCost > rhs.bestCost;
	}
};



class HeuristicMapper {
public:
//	HeuristicMapper(CGRA* cgra, DFG* dfg) : cgra(cgra), dfg(dfg){};
	HeuristicMapper(std::string fName){
		fName = fName + ".mapping.csv";
		mappingLog.open(fName.c_str());
	}
	CGRA* cgra;
	DFG* dfg;

	int getMinimumII(CGRA* cgra, DFG* dfg);
	void SortTopoGraphicalDFG();
	void SortSCCDFG();
	bool Map(CGRA* cgra, DFG* dfg);
	bool LeastCostPath(Port* start, Port* end, std::vector<Port*>& path, int& cost, DFGNode* node, std::map<Port*,std::set<DFGNode*>>& mutexPaths);
	int calculateCost(Port* src, Port* next_to_src);

	bool estimateRouting(DFGNode& node, std::priority_queue<dest_with_cost>& estimatedRoutes);
	bool Route(DFGNode& node, std::priority_queue<dest_with_cost>& estimatedRoutes);
	void assignPath(DFGNode* src, DFGNode* dest, std::vector<Port*> path);

	bool enableBackTracking=false;
	bool enableMutexPaths=false;
	int backTrackLimit=4;


private:
	int regDiscourageFactor=10;
	int PETransitionCostFactor=100;
	int PortTransitionCost=1;
	int UOPCostFactor=100;
	int MEMResourceCost = 10000;

	std::ofstream mappingLog;
	std::vector<DFGNode*> sortedNodeList;
	void printMappingLog();
	void printMappingLog2();


};

} /* namespace CGRAXMLCompile */

#endif /* HEURISTICMAPPER_H_ */

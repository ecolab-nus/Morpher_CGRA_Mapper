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
#include <math.h>

#define MRC 8000
#define UOP 1000


namespace CGRAXMLCompile {

//struct definitions
class HeuristicMapper;

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
	DFGNode* node;
	dest_with_cost(){}
	dest_with_cost(std::priority_queue<parent_cand_src_with_cost> parentStartLocs,
				   std::priority_queue<dest_child_with_cost> alreadyMappedChilds,
				   DataPath* dest, DFGNode* node, int cost,
				   int unmappedMemNodeCount, HeuristicMapper* hm) :
		parentStartLocs(parentStartLocs),alreadyMappedChilds(alreadyMappedChilds), dest(dest), node(node){
		bestCost = sumBestCosts(unmappedMemNodeCount, hm);
	}


	int sumBestCosts(int unmappedMemNodeCount, HeuristicMapper* hm){
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
				if(parent->getNextPorts(&p,hm).empty()) continue;
				if(p.getNode()==NULL){
					freePorts++;
				}
			}
//			cost = cost + (dest->getPE()->outputPorts.size() - freePorts)*UOP;
			cost = cost + (15-freePorts)*UOP;

			int primaryCost=0;
			for(DFGNode* child : node->children){
				for(DFGNode* parentil : child->parents){
					if(parentil->rootDP!=NULL && parentil != node){
						PE* parentilPE = parentil->rootDP->getPE();
						PE* destPE = dest->getPE();
						CGRA* cgra = destPE->getCGRA();

						int dx = std::abs(parentilPE->X-destPE->X);
						int dy = std::abs(parentilPE->Y-destPE->Y);
						int dt = (destPE->T - parentilPE->T + cgra->get_t_max()) % cgra->get_t_max();
						primaryCost = primaryCost + dt;
					}
				}
			}
			cost = cost + primaryCost*100;

			int secondaryCost=0;
			for(DFGNode* child : node->children){
				for(DFGNode* childchild : node->children){
					for(DFGNode* childparent : childchild->parents){
						if(childparent!=childchild){
							for(DFGNode* parent : childparent->parents){
								if(parent!=node && parent->rootDP!=NULL){
									PE* parentilPE = parent->rootDP->getPE();
									PE* destPE = dest->getPE();
									CGRA* cgra = destPE->getCGRA();

									int dx = std::abs(parentilPE->X-destPE->X);
									int dy = std::abs(parentilPE->Y-destPE->Y);
									int dt = (destPE->T - parentilPE->T + cgra->get_t_max()) % cgra->get_t_max();
									secondaryCost = secondaryCost + dx + dy + dt;
								}
							}
						}
					}
				}
			}
//			cost = cost + secondaryCost*10;

			FU* fu = dest->getFU();
			CGRA* cgra = dest->getCGRA();
			int memcost=0;
			if(fu->supportedOPs.find("LOAD")!=fu->supportedOPs.end()){
				double memrescost_dbl = (double)unmappedMemNodeCount/(double)cgra->freeMemNodes;
				memrescost_dbl = memrescost_dbl*(double)MRC;
				memcost = (int)memrescost_dbl;
			}

			cost = cost + memcost;


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
		fNameLog1 = fName;
//		mappingLog.open(fName.c_str());
//		mappingLog2.open(fName2.c_str());
	}
	CGRA* cgra;
	DFG* dfg;

	int getMinimumII(CGRA* cgra, DFG* dfg);
	void SortTopoGraphicalDFG();
	void SortSCCDFG();
	bool Map(CGRA* cgra, DFG* dfg);
	bool LeastCostPathAstar(Port* start, Port* end, std::vector<Port*>& path, int& cost, DFGNode* node, std::map<Port*,std::set<DFGNode*>>& mutexPaths, DFGNode* currNode);
	bool LeastCostPathDjk(Port* start, Port* end, std::vector<Port*>& path, int& cost, DFGNode* node, std::map<Port*,std::set<DFGNode*>>& mutexPaths);
	int calculateCost(Port* src, Port* next_to_src, Port* dest);

	bool estimateRouting(DFGNode* node, std::priority_queue<dest_with_cost>& estimatedRoutes, DFGNode** failedNode);
	bool Route(DFGNode* node, std::priority_queue<dest_with_cost>& estimatedRoutes, DFGNode** failedNode);
	void assignPath(DFGNode* src, DFGNode* dest, std::vector<Port*> path);
	bool dataPathCheck(DataPath* dp, DFGNode* node);

	bool sanityCheck();

	bool enableBackTracking=false;
	bool enableMutexPaths=false;
	int backTrackLimit=4;

	void printMappingLog();
	void printMappingLog2();

private:
	int regDiscourageFactor=10;
	int PETransitionCostFactor=100;
	int PortTransitionCost=1;
	int UOPCostFactor=UOP;
	int MEMResourceCost= MRC;

	std::string fNameLog1;
	std::string fNameLog2;

	std::ofstream mappingLog;
	std::ofstream mappingLog2;
	std::vector<DFGNode*> sortedNodeList;

	void removeFailedNode(std::stack<DFGNode*>& mappedNodes, std::stack<DFGNode*>& unmappedNodes, DFGNode* failedNode);


};

} /* namespace CGRAXMLCompile */

#endif /* HEURISTICMAPPER_H_ */

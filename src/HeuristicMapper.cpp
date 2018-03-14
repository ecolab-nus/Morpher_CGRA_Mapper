/*
 * HeuristicMapper.cpp
 *
 *  Created on: 28 Feb 2018
 *      Author: manupa
 */

#include "HeuristicMapper.h"
#include <queue>
#include <assert.h>
#include <math.h>
#include <algorithm>    // std::reverse
#include "DataPath.h"
#include "FU.h"

#include <stack>
#include <functional>
#include <set>
#include <iostream>
#include <sstream>

namespace CGRAXMLCompile {

//HeuristicMapper::HeuristicMapper() {
//	// TODO Auto-generated constructor stub
//
//}

} /* namespace CGRAXMLCompile */

void CGRAXMLCompile::HeuristicMapper::SortTopoGraphicalDFG() {
	sortedNodeList.clear();
	std::map<int,std::vector<DFGNode*>> asapLevelNodeList;
	for(DFGNode& node : dfg->nodeList){
		asapLevelNodeList[node.ASAP].push_back(&node);
	}

	int maxASAPlevel=0;
	for(std::pair<int,std::vector<DFGNode*>> pair : asapLevelNodeList){
		if(pair.first > maxASAPlevel){
			maxASAPlevel = pair.first;
		}
	}

	for (int i = 0; i <= maxASAPlevel; ++i) {
		for (DFGNode* node : asapLevelNodeList[i]){
			sortedNodeList.push_back(node);
		}
	}
	std::reverse(sortedNodeList.begin(),sortedNodeList.end());

	int unmappedMemNodeCount=0;
	for(DFGNode* node : this->sortedNodeList){
		if(node->isMemOp()){
			if(node->rootDP==NULL){
				unmappedMemNodeCount++;
			}
		}
	}
	dfg->unmappedMemOps = unmappedMemNodeCount;
}

bool CGRAXMLCompile::HeuristicMapper::Map(CGRA* cgra, DFG* dfg) {

	std::stack<DFGNode*> mappedNodes;
	std::stack<DFGNode*> unmappedNodes;
	std::map<DFGNode*,std::priority_queue<dest_with_cost>> estimatedRouteInfo;

	int backTrackCredits=this->backTrackLimit;


	this->cgra = cgra;
	this->dfg =dfg;
//	SortSCCDFG();
	SortTopoGraphicalDFG();

	std::string mappingLogFileName = fNameLog1 + "_II=" + std::to_string(cgra->get_t_max()) + ".mapping.csv";
	std::string mappingLog2FileName = fNameLog1 + "_II=" + std::to_string(cgra->get_t_max()) + ".routeInfo.log";
	mappingLog.open(mappingLogFileName.c_str());
	mappingLog2.open(mappingLog2FileName.c_str());


	for (DFGNode* node : sortedNodeList){
		unmappedNodes.push(node);
	}

	std::cout << "MAP begin...\n";

	while(!unmappedNodes.empty()){
		DFGNode* node = unmappedNodes.top();
		unmappedNodes.pop();

		std::stringstream MapHeader;
		MapHeader << "current node = " << node->idx;
		MapHeader << ",op = " << node->op;
		MapHeader << ",unmapped nodes = " << unmappedNodes.size();
		MapHeader << ",mapped nodes = " << mappedNodes.size();
		MapHeader << ",freeMemNodes = " << cgra->freeMemNodes;
		MapHeader << ",unmappedMemNodes = " << dfg->unmappedMemOps;
		MapHeader << ",II = " << cgra->get_t_max();
		MapHeader << ",btCredits = " << backTrackCredits;
		MapHeader << ",PEType = " << this->cgra->peType;
		MapHeader << ",DPs = " << this->cgra->numberofDPs;
		MapHeader << ",BB = " << node->BB;
		MapHeader << "\n";

		std::cout << MapHeader.str();
		mappingLog << MapHeader.str();

		bool isEstRouteSucc=false;

		//fill the routing information
		if(estimatedRouteInfo.find(node)==estimatedRouteInfo.end()){
			//the routes are not estimated.
			std::priority_queue<dest_with_cost> estimatedRoutes;
			isEstRouteSucc = estimateRouting(node,estimatedRoutes);

			if(!isEstRouteSucc){
				printMappingLog();
				printMappingLog2();
				if(enableBackTracking){
					if(backTrackCredits==0){
						std::cout << "route estimation failed...\n";
						std::cout << "Map Failed!.\n";
						mappingLog << "route estimation failed...\n";
						mappingLog << "Map Failed!.\n";

						mappingLog.close();
						mappingLog2.close();
						return false;
					}
					backTrackCredits--;

					DFGNode* prevNode = mappedNodes.top();
					mappedNodes.pop();
					unmappedNodes.push(node);
					unmappedNodes.push(prevNode);
					prevNode->clear(this->dfg);
					std::cout << "route estimation failed...\n";
					mappingLog << "route estimation failed...\n";
					continue;
				}
				else{
					while(!mappedNodes.empty()){
						DFGNode* prevNode = mappedNodes.top();
						mappedNodes.pop();
						prevNode->clear(this->dfg);
					}
					std::cout << "Map Failed!.\n";
					mappingLog << "Map Failed!.\n";
					mappingLog.close();
					mappingLog2.close();
					return false;
				}
			}
			estimatedRouteInfo[node]=estimatedRoutes;
		}

		bool isRouteSucc=false;

		std::cout << "estimatedRouteInfo[node].size = " << estimatedRouteInfo[node].size() << "\n";
		mappingLog << "estimatedRouteInfo[node].size = " << estimatedRouteInfo[node].size() << "\n";
		if(!estimatedRouteInfo[node].empty()){
			isRouteSucc=Route(node,estimatedRouteInfo[node]);
		}
		else{
			if(mappedNodes.empty()){
				mappingLog << "Map Failed!.\n";
				std::cout << "Map Failed!.\n";
				mappingLog.close();
				mappingLog2.close();
				return false;
			}
		}


		if(!isRouteSucc){
			this->printMappingLog();
			this->printMappingLog2();
			if(mappedNodes.empty()){
				mappingLog << "Map Failed!.\n";
				std::cout << "Map Failed!.\n";
				mappingLog.close();
				mappingLog2.close();
				return false;
			}

			if(enableBackTracking){
				if(backTrackCredits==0){
					mappingLog << "Map Failed!.\n";
					std::cout << "Map Failed!.\n";
					mappingLog.close();
					mappingLog2.close();
					return false;
				}
				backTrackCredits--;

				DFGNode* prevNode = mappedNodes.top();
				mappedNodes.pop();
				unmappedNodes.push(node);
				unmappedNodes.push(prevNode);

				prevNode->clear(this->dfg);
				estimatedRouteInfo.erase(node);
				continue;
			}
			else{
				while(!mappedNodes.empty()){
					DFGNode* prevNode = mappedNodes.top();
					mappedNodes.pop();
					prevNode->clear(this->dfg);
				}
				mappingLog << "Map Failed!.\n";
				std::cout << "Map Failed!.\n";
				mappingLog.close();
				mappingLog2.close();
				return false;
			}
		}

//		this->printMappingLog();
//		this->printMappingLog2();
		backTrackCredits = std::min(this->backTrackLimit,backTrackCredits+1);
		mappedNodes.push(node);
	}

	mappingLog << "Map Success!.\n";
	mappingLog2 << "Map Success!.\n";
	this->printMappingLog();
	this->printMappingLog2();

	std::cout << "Map Success!.\n";
	mappingLog.close();
	mappingLog2.close();
	return true;
}

bool CGRAXMLCompile::HeuristicMapper::estimateRouting(DFGNode* node,
		std::priority_queue<dest_with_cost>& estimatedRoutes) {

	std::map<DFGNode*,std::vector<Port*>> possibleStarts;
	std::map<DFGNode*,Port*> alreadyMappedChildPorts;

	bool detailedDebug=false;
//	if(node->idx==48)detailedDebug=true;

//	std::cout << "EstimateEouting begin...\n";

	for(DFGNode* parent : node->parents){
//		std::cout << "parent = " << parent->idx << "\n";
		if(parent->rootDP!=NULL){ //already mapped
			assert(parent->rootDP->getOutputDP()->getOutPort("T"));
			possibleStarts[parent].push_back(parent->rootDP->getOutputDP()->getOutPort("T"));

			for(std::pair<Port*,int> pair : parent->routingPorts){
				Port* p = pair.first;
				possibleStarts[parent].push_back(p);
			}
		}
	}

	for(DFGNode* child : node->children){
		if(child->rootDP!=NULL){// already mapped
//			std::cout << "child="<< child->idx << ",childOpType=" << node.childrenOPType[child] << "\n";
			assert(child->rootDP->getInPort(node->childrenOPType[child]));
			alreadyMappedChildPorts[child]=child->rootDP->getInPort(node->childrenOPType[child]);
		}
	}

	std::vector<DataPath*> candidateDests;
	for (int t = 0; t < cgra->get_t_max(); ++t) {
		for (int y = 0; y < cgra->get_y_max(); ++y) {
			for (int x = 0; x < cgra->get_x_max(); ++x) {
				PE* currPE = cgra->PEArr[t][y][x];
				for(Module* submod : currPE->subModules){
					if(FU* fu = dynamic_cast<FU*>(submod)){

						if(fu->supportedOPs.find(node->op)==fu->supportedOPs.end()){
							continue;
						}

						if(fu->currOP.compare(node->op)==0){
							for(Module* submodFU : fu->subModules){
								if(DataPath* dp = dynamic_cast<DataPath*>(submodFU)){
									if(dp->getMappedNode()==NULL){
//									if(dataPathCheck(dp,&node)){
										candidateDests.push_back(dp);
									}
								}
							}
						}
						else if(fu->currOP.compare("NOP")==0){
							for(Module* submodFU : fu->subModules){
								if(DataPath* dp = dynamic_cast<DataPath*>(submodFU)){
									if(dp->getMappedNode()==NULL){
//									if(dataPathCheck(dp,&node)){
										candidateDests.push_back(dp);
									}
								}
							}
						}

					}
				}
			}
		}
	}

//	std::cout << "Candidate Dests = " << candidateDests.size() << "\n";

	//Route Estimation
	for(DataPath* dest : candidateDests){
//		std::cout << "Candidate Dest =" ;
//		std::cout << dest->getPE()->getName() << ".";
//		std::cout << dest->getFU()->getName() << ".";
//		std::cout << dest->getName() << "\n";

//		std::map<DFGNode*,std::priority_queue<cand_src_with_cost>> parentStartLocs;
		std::priority_queue<parent_cand_src_with_cost> parentStartLocs;

    	bool pathFromParentExist=true;
		for(std::pair<DFGNode*,std::vector<Port*>> pair : possibleStarts){
			DFGNode* parent = pair.first;
			Port* destPort = dest->getInPort(parent->getOPtype(node));

			std::priority_queue<cand_src_with_cost> res;

			for(Port* startCand : pair.second){
				int cost;
				std::vector<Port*> path;
				std::map<Port*,std::set<DFGNode*>> mutexPaths;
if(detailedDebug)				std::cout << "Estimating Path" << startCand->getFullName() << "--->" << destPort->getFullName() << "\n";
				bool pathExist = LeastCostPathAstar(startCand,destPort,path,cost,parent,mutexPaths,node);
				if(!pathExist){
if(detailedDebug)			    std::cout << "Estimate Path Failed :: " << startCand->getFullName() << "--->" << destPort->getFullName() << "\n";
					continue;
				}
				res.push(cand_src_with_cost(startCand,destPort,cost));
			}
			if(res.empty()){
				pathFromParentExist=false;
				break;
			}
			parent_cand_src_with_cost pcswc(parent,res);
			parentStartLocs.push(pcswc);
		}

		if(!pathFromParentExist){
			continue;
		}


//		for(std::pair<DFGNode*,std::priority_queue<cand_src_with_cost>> pair : parentStartLocs){
//			DFGNode* parent = pair.first;
//			pathFromParentExist = pathFromParentExist & (!parentStartLocs[parent].empty());
//		}
//		if(!pathFromParentExist){
//			continue;
//		}

		bool pathExistMappedChild = true;
		std::priority_queue<dest_child_with_cost> alreadyMappedChilds;
		for(std::pair<DFGNode*,Port*> pair : alreadyMappedChildPorts){
			DFGNode* child = pair.first;
			Port* childDestPort = pair.second;
			std::vector<Port*> path;
			int cost;

			FU* parentFU = dest->getFU();
			assert(parentFU->supportedOPs.find(node->op)!=parentFU->supportedOPs.end());
			int latency = parentFU->supportedOPs[node->op];
			Port* destPort = dest->getOutputPort(latency);

			std::map<Port*,std::set<DFGNode*>> mutexPaths;
if(detailedDebug)				std::cout << "Estimating Path" << destPort->getFullName() << "--->" << childDestPort->getFullName() << "\n";
		    pathExistMappedChild = pathExistMappedChild & LeastCostPathAstar(destPort,childDestPort,path,cost,node,mutexPaths,node);

		    dest_child_with_cost dcwc(child,childDestPort,destPort,cost);
		    alreadyMappedChilds.push(dcwc);
		}
	    if(!pathExistMappedChild){
	    	if(detailedDebug)				std::cout << "Estimating Path Failed!\n";
	    	continue;  //if it cannot be mapped to child abort the estimation for this dest
	    }

	    assert(pathFromParentExist);
	    assert(pathExistMappedChild);
		dest_with_cost dest_with_cost_ins(parentStartLocs,alreadyMappedChilds,dest,node,0,this->dfg->unmappedMemOps);
		estimatedRoutes.push(dest_with_cost_ins);
	}
//	std::cout << "EstimateEouting end!\n";
	return !estimatedRoutes.empty();
}


bool CGRAXMLCompile::HeuristicMapper::Route(DFGNode* node,
		std::priority_queue<dest_with_cost>& estimatedRoutes) {

	std::cout << "Route begin...\n";

	int parentRoutingPortCount=0;
	int routedParents=0;

	for(DFGNode* parent : node->parents){
		int thisParentNodeCount=0;
		if(parent->rootDP!=NULL){
			thisParentNodeCount = parent->routingPorts.size();
		}

		if(thisParentNodeCount>0){
			routedParents++;
			thisParentNodeCount--; //remove the T port in the cout
		}
		parentRoutingPortCount+=thisParentNodeCount;
	}
//	if(parentRoutingPortCount>0){
//		parentRoutingPortCount-=1; //remove the T port in the cout
//	}

	int addedRoutingParentPorts=0;

	bool routeSucc=false;
	dest_with_cost currDest;
	while(!estimatedRoutes.empty()){
		currDest = estimatedRoutes.top();
		estimatedRoutes.pop();

		std::cout << "alreadyMappedChilds = " << currDest.alreadyMappedChilds.size() << "\n";

		bool alreadMappedChildRouteSucc=true; //this will change to false if failure in alreadyMappedChilds
		std::map<DFGNode*,std::vector<Port*>> mappedChildPaths;
		std::map<DFGNode*,std::map<Port*,std::set<DFGNode*>>> mappedChildMutexPaths;
		while(!currDest.alreadyMappedChilds.empty()){
			dest_child_with_cost dest_child_with_cost_ins = currDest.alreadyMappedChilds.top();
			currDest.alreadyMappedChilds.pop();

			std::vector<Port*> possibleStarts; possibleStarts.clear();
			possibleStarts.push_back(dest_child_with_cost_ins.startPort);
			for(std::pair<Port*,int> pair : node->routingPorts){
				possibleStarts.push_back(pair.first);
			}

			std::priority_queue<cand_src_with_cost> q;
			std::map<Port*,std::set<DFGNode*>> mutexPathsTmp;
			std::vector<Port*> pathTmp;
			for(Port* p : possibleStarts){
				int cost;
				if(LeastCostPathAstar(p,dest_child_with_cost_ins.childDest,pathTmp,cost,node,mutexPathsTmp,node)){
					q.push(cand_src_with_cost(p,dest_child_with_cost_ins.childDest,cost));
				}
			}

			int cost;
			std::vector<Port*> path;
			Port* src = dest_child_with_cost_ins.startPort;
			Port* dest = dest_child_with_cost_ins.childDest;

			while(!q.empty()){
				cand_src_with_cost head = q.top();
				q.pop();
				std::map<Port*,std::set<DFGNode*>> mutexPaths;
				alreadMappedChildRouteSucc = LeastCostPathAstar(head.src,dest,path,cost,node,mutexPaths,node);
				if(alreadMappedChildRouteSucc){
					assignPath(node,dest_child_with_cost_ins.child,path);
					mappedChildPaths[dest_child_with_cost_ins.child]=path;
					mappedChildMutexPaths[dest_child_with_cost_ins.child]=mutexPaths;
					std::cout << "Route success :: from=" << src->getFullName() << "--> to=" << dest->getFullName() << "|node=" << node->idx << "\n";
					break;
				}
				else{
					std::cout << "Route Failed :: from=" << src->getFullName() << "--> to=" << dest->getFullName() << "\n";
					for(Port* p : path){
						if(p->getMod()->getPE()){
							std::cout << p->getMod()->getPE()->getName() << "-->";
						}
					}
					std::cout << "\n";

					for(Port* p : path){
						std::cout << p->getFullName() << "\n";
					}
				}
			}
			if(!alreadMappedChildRouteSucc){
				break;
			}
		}

		if(alreadMappedChildRouteSucc){
			for(std::pair<Port*,int> pair : node->routingPorts){
				Port* p = pair.first;
				int destIdx = pair.second;
				std::cout << "to:" << destIdx << "," << p->getFullName() << "\n";
			}
		}


		if(!alreadMappedChildRouteSucc){
			node->clear(this->dfg);
			continue; //try the next dest
		}
		else{
			std::cout << "Already Mapped child Routes....\n";
			for(std::pair<DFGNode*,std::vector<Port*>> pair : mappedChildPaths){
				DFGNode* child = pair.first;
				for(Port* p : pair.second){
					std::cout << "to:" <<child->idx << " :: ";
					std::cout << p->getFullName();
					if(mappedChildMutexPaths[child].find(p)!=mappedChildMutexPaths[child].end()){
						std::cout << "|mutex(";
						for(DFGNode* mutexnode : mappedChildMutexPaths[child][p]){
							std::cout << mutexnode->idx << ",";
						}
						std::cout << ")";
					}
					std::cout << "\n";
				}
				std::cout << "\n";
			}
			std::cout << "\n";
		}

		bool parentRoutSucc=true;
		addedRoutingParentPorts = 0;
		std::map<DFGNode*,std::map<Port*,std::set<DFGNode*>>> mappedParentMutexPaths;
		while(!currDest.parentStartLocs.empty()){
			parent_cand_src_with_cost pcswc = currDest.parentStartLocs.top();
			currDest.parentStartLocs.pop();
			DFGNode* parent = pcswc.parent;
			std::priority_queue<cand_src_with_cost> &q = pcswc.cswc;

			bool succ=false;
			while(!q.empty()){
				cand_src_with_cost cand_src_with_cost_ins = q.top();
				q.pop();
				Port* src = cand_src_with_cost_ins.src;
				Port* dest = cand_src_with_cost_ins.dest;
				std::vector<Port*> path;
				std::map<Port*,std::set<DFGNode*>> mutexPath;
				int cost;
				succ = LeastCostPathAstar(src,dest,path,cost,parent,mutexPath,node);
				if(succ){
					bool routedParent=true;
					if(parent->routingPorts.size()==0){ //unrouted parent
						routedParent=false;
					}
					assignPath(parent,node,path);
					mappedParentMutexPaths[parent]=mutexPath;
					addedRoutingParentPorts += path.size();
					if(routedParent){
						addedRoutingParentPorts-=1;
					}
//					for(Port* p : path){
//						std::cout << p->getFullName() << ",\n";
//					}
//					std::cout << "\n";
					break;
				}
				else{
					addedRoutingParentPorts=0;
					node->clear(this->dfg);
					std::cout << "Route Failed :: from=" << src->getFullName() << "--> to=" << dest->getFullName() << "\n";
				}
			}
			if(!succ){
				node->clear(this->dfg);
				addedRoutingParentPorts=0;
				parentRoutSucc=false; // at least one parent failed to route, try a new dest
				break;
			}
		}

		if(parentRoutSucc){ //all parents routed succesfull + all mapped childs are connected
			routeSucc=true;
			std::cout << "node=" << node->idx <<",op=" << node->op << " is mapped to " << currDest.dest->getPE()->getName() << "\n";
			std::cout << "routing info ::\n";
			for(DFGNode* parent : node->parents){
				std::cout << "parent routing port size = " << parent->routingPorts.size() << "\n";
				for(std::pair<Port*,int> pair : parent->routingPorts){
					Port* p = pair.first;
//					if(node.routingPortDestMap[p]==&node){
						std::cout << "fr:" <<parent->idx << " :: ";
						std::cout << p->getFullName();
						if(mappedParentMutexPaths[parent].find(p)!=mappedParentMutexPaths[parent].end()){
							std::cout << "|mutex(";
							for(DFGNode* mutexnode : mappedParentMutexPaths[parent][p]){
								std::cout << mutexnode->idx << ",";
							}
							std::cout << ")";
						}
						std::cout << "\n";
//					}
				}
			}
			std::cout << "routing info done.\n";
			currDest.dest->assignNode(node,this->dfg);
			node->rootDP = currDest.dest;
			break;
		}
		node->clear(this->dfg);
	}


	if(routeSucc){
		std::cout << "Route success...\n";

		int parentRoutingPortCountEnd=0;
//		int mappedParentCount=0;
		for(DFGNode* parent : node->parents){
			if(parent->rootDP!=NULL){
//				mappedParentCount++;
				parentRoutingPortCountEnd += parent->routingPorts.size();
			}
		}
		parentRoutingPortCountEnd=std::max(0,parentRoutingPortCountEnd-routedParents);
		if(parentRoutingPortCountEnd!=parentRoutingPortCount+addedRoutingParentPorts){
			std::cout << "parentRoutingPortCountEnd=" << parentRoutingPortCountEnd << "\n";
			std::cout << "addedRoutingParentPorts=" << addedRoutingParentPorts << "\n";
			std::cout << "parentRoutingPortCount=" << parentRoutingPortCount << "\n";
		}

		assert(parentRoutingPortCountEnd==parentRoutingPortCount+addedRoutingParentPorts);
		return true;
	}
	else{
		currDest.dest->assignNode(node,this->dfg);
		node->rootDP = currDest.dest;
		node->clear(this->dfg);
		std::cout << "Route failed...\n";

		int parentRoutingPortCountEnd=0;
//		int mappedParentCount=0;
		for(DFGNode* parent : node->parents){
			if(parent->rootDP!=NULL){
//				mappedParentCount++;
				parentRoutingPortCountEnd += parent->routingPorts.size();
			}
		}
		parentRoutingPortCountEnd=std::max(0,parentRoutingPortCountEnd-routedParents);
		if(parentRoutingPortCountEnd!=parentRoutingPortCount+addedRoutingParentPorts){
			std::cout << "parentRoutingPortCountEnd=" << parentRoutingPortCountEnd << "\n";
			std::cout << "addedRoutingParentPorts=" << addedRoutingParentPorts << "\n";
			std::cout << "parentRoutingPortCount=" << parentRoutingPortCount << "\n";
		}
		assert(parentRoutingPortCountEnd==parentRoutingPortCount);
		return false;
	}
}

template <typename T>
std::set<T> getUnion(const std::set<T>& a, const std::set<T>& b)
{
  std::set<T> result = a;
  result.insert(b.begin(), b.end());
  return result;
}

int CGRAXMLCompile::HeuristicMapper::getMinimumII(CGRA* cgra, DFG* dfg) {

	std::map<std::string,int> opHist;

	for(DFGNode& node : dfg->nodeList){

		if(opHist.find(node.op)==opHist.end()){
			opHist[node.op]=0;
		}
		opHist[node.op]++;
	}

	std::map<std::string,std::set<DataPath*>> opFUs;

	assert(cgra->get_t_max() > 0);
	for (int y = 0; y < cgra->get_y_max(); ++y) {
		for (int x = 0; x < cgra->get_x_max(); ++x) {
			PE* currPE = cgra->PEArr[0][y][x];

			for(Module* FU_mod : currPE->subModules){
				if(FU* fu = dynamic_cast<FU*>(FU_mod)){
					for(std::pair<std::string,int> pair : fu->supportedOPs){
						std::string suppOp = pair.first;
						for(Module* DP_mod : fu->subModules){
							if(DataPath* dp = dynamic_cast<DataPath*>(DP_mod)){
								opFUs[suppOp].insert(dp);
							}
						}
					}
				}
			}

		}
	}

	struct opFUstr{
		std::string op;
		int count;
		opFUstr(std::string op, int count) : op(op), count(count){};

		bool operator<(const opFUstr& rhs){
			return this->count < rhs.count;
		}
	};

	std::vector<opFUstr> opFUstr_vec;
	for(std::pair<std::string,std::set<DataPath*>> pair : opFUs){
		opFUstr_vec.push_back(opFUstr(pair.first,pair.second.size()));
	}
	std::sort(opFUstr_vec.begin(),opFUstr_vec.end());

	int ii=1;
	int cummOp=0;
	std::set<DataPath*> cummDP;
	for(opFUstr a : opFUstr_vec){
		std::cout << "op=" << a.op << ",opcount=" << opHist[a.op] << ",dpcount=" << a.count;
		int new_ratio = (opHist[a.op] + a.count - 1) / a.count; // int ceil
		std::cout << ",ratio=" << new_ratio ;
		ii=std::max(ii,new_ratio);

		cummOp+=opHist[a.op];
		for(DataPath* dp : opFUs[a.op]){
			cummDP.insert(dp);
		}
		std::cout << ",cummOp=" << cummOp << ",cummDP=" << cummDP.size();
		int cumm_ratio = (cummOp + cummDP.size() - 1)/cummDP.size();
		std::cout << ",ratio=" << cumm_ratio << "\n";
		ii=std::max(ii,cumm_ratio);
	}
	std::cout << "Min II = " << ii << "\n";
	return ii;
}

void CGRAXMLCompile::HeuristicMapper::SortSCCDFG() {

	sortedNodeList.clear();
	assert(this->dfg);
	std::vector<std::set<DFGNode*>> SCCs = dfg->getSCCs();

	struct scc_with_size{
		std::set<DFGNode*> scc;
		int nodesize;
		scc_with_size(std::set<DFGNode*> scc, int nodesize) : scc(scc), nodesize(nodesize){}

		bool operator<(const scc_with_size& rhs){
			return this->nodesize < rhs.nodesize;
		}
		struct greater{
			bool operator()(const scc_with_size& lhs, const scc_with_size& rhs){
				return lhs.nodesize > rhs.nodesize;
			}
		};
	};

	std::vector<scc_with_size> scc_with_size_vec;
	for(std::set<DFGNode*> scc : SCCs){
		scc_with_size_vec.push_back(scc_with_size(scc,scc.size()));
	}
	std::sort(scc_with_size_vec.begin(),scc_with_size_vec.end());

	std::cout << "SortSCCDFG::";
	for(scc_with_size sccws : scc_with_size_vec){
		for(DFGNode* node : sccws.scc){
			std::cout << node->idx << ",";
			sortedNodeList.push_back(node);
		}
	}
	std::cout << "\n";
}

void CGRAXMLCompile::HeuristicMapper::assignPath(DFGNode* src, DFGNode* dest,
		std::vector<Port*> path) {

	std::cout << "assigning path from:" << src->idx << " to:" << dest->idx << "\n";

	int srcPortCount=0;
	for(Port* p : path){
		if(p->getNode() == src) {
			srcPortCount++;
			continue;
		}
		p->setNode(src);
		if(std::find(src->routingPorts.begin(),src->routingPorts.end(),std::make_pair(p,dest->idx))==src->routingPorts.end()){
			src->routingPorts.push_back(std::make_pair(p,dest->idx));
		}
//		src->routingPortDestMap[p]=dest->idx;
	}
	std::cout << "srcPortCount = " << srcPortCount << "\n";

}

bool CGRAXMLCompile::HeuristicMapper::LeastCostPathAstar(Port* start, Port* end,
		std::vector<Port*>& path, int& cost, DFGNode* node, std::map<Port*,std::set<DFGNode*>>& mutexPaths,  DFGNode* currNode) {

//	std::cout << "LeastCoastPath started with start=" << start->getFullName() << " to end=" << end->getFullName() << "\n";

	std::map<Port*,int> cost_to_port;
	std::map<Port*,Port*> cameFrom;

	path.clear();
	mutexPaths.clear();

	bool detailedDebug=false;
//	if(currNode->idx==48)detailedDebug=true;

	struct port_heuristic{
		Port* p;
		int heuristic;

		int calc_heuristic(Port* src, Port* dest){
			PE* srcPE = src->findParentPE();
			assert(srcPE);
			PE* destPE = dest->findParentPE();
			assert(destPE);

			CGRA* currCGRA = srcPE->getCGRA();
			assert(currCGRA);

			int dist_dest = std::abs(destPE->Y - srcPE->Y) + std::abs(destPE->X - srcPE->X)
			                + std::abs((destPE->T - srcPE->T + currCGRA->get_t_max())%currCGRA->get_t_max());
			return dist_dest;
		}

		port_heuristic(Port* p, Port* dest){
			this->p=p;
			heuristic=calc_heuristic(p,dest);
		}

		port_heuristic(Port* p, int cost){
			this->p=p;
			this->heuristic=cost;
		}

		port_heuristic(Port* p, Port* dest, int cost){
			this->p=p;
			this->heuristic=cost*100 + calc_heuristic(p,dest);
		}

		bool operator<(const port_heuristic& rhs) const{
			return this->heuristic > rhs.heuristic;
		}

//		bool operator>(const port_heuristic& rhs) const{
//			return this->heuristic > rhs.heuristic;
//		}

	};

//	std::queue<Port*> q;
	std::priority_queue<port_heuristic> q;

	q.push(port_heuristic(start,0));
//	path.push_back(start);

	cost_to_port[start]=0;

	Port* currPort;
	std::vector<Port*> deadEnds;

	while(!q.empty()){
		port_heuristic curr = q.top();
		currPort = curr.p;
		q.pop();

if(detailedDebug) std::cout << "currPort=" << currPort->getFullName() << "\n";

		if(currPort == end){
			break;
		}

		std::vector<Port*> nextPorts = currPort->getMod()->connections[currPort];
		if(currPort->getType()==OUT){
			if(currPort->getMod()->getParent()){
				for(Port* p : currPort->getMod()->getParent()->connections[currPort]){
//					std::cout << currPort->getMod()->getParent()->getName() << "***************\n";
					nextPorts.push_back(p);
				}
			}
		}

//		std::cout << "nextPorts size = " << nextPorts.size() << "\n";
		int q_len = q.size();
		for(Port* nextPort : nextPorts){
			bool isNextPortFree=false;
			bool isNextPortMutex=false;
			if(enableMutexPaths){
				if(nextPort->getNode()==NULL){
					isNextPortFree=true;
				}
				else if(dfg->mutexBBs[nextPort->getNode()->BB].find(node->BB)!=dfg->mutexBBs[nextPort->getNode()->BB].end()){
					// next BB is mutually exclusive with current nodes BB, then this can be mapped.
					isNextPortFree=true;
					isNextPortMutex=true;
					mutexPaths[nextPort].insert(nextPort->getNode());
					mutexPaths[nextPort].insert(node);
				}
			}
			else{
				if(nextPort->getNode()==NULL){
					isNextPortFree=true;
				}
			}
			if(isNextPortFree){ // unmapped port
if(detailedDebug)				std::cout << "\tnextPort=" << nextPort->getFullName() << ",";
				int nextPortCost = cost_to_port[currPort] + calculateCost(currPort,nextPort,end);
if(detailedDebug)				std::cout << "cost=" << nextPortCost << "\n";
				if(isNextPortMutex){
					//no cost is added in using mutually exclusive routes
					nextPortCost = cost_to_port[currPort];
				}

				if(cost_to_port.find(nextPort)!=cost_to_port.end()){
					if(cost_to_port[nextPort] > nextPortCost){
						cost_to_port[nextPort]=nextPortCost;
						cameFrom[nextPort]=currPort;
						q.push(port_heuristic(nextPort,end,nextPortCost));
					}
					else{
if(detailedDebug)		std::cout << "Port is not inserted..\n";
					}
				}
				else{
					cost_to_port[nextPort]=nextPortCost;
					cameFrom[nextPort]=currPort;
					q.push(port_heuristic(nextPort,end,nextPortCost));
				}
			}
			else{
if(detailedDebug)		std::cout << "\t[MAPPED="<< nextPort->getNode()->idx << "]nextPort=" << nextPort->getFullName() << "\n";
			}
		}
		if(q.size() == q_len){
			deadEnds.push_back(currPort);
		}
	}


	if(currPort!=end){
//		std::cout << "LeastCostPath failed!\n";
		path.clear();
		for(Port* p : deadEnds){
			std::vector<Port*> tmpPath;
			while(p !=start){
				tmpPath.push_back(p);
				assert(cameFrom.find(p)!=cameFrom.end());
				p = cameFrom[p];
			}
			tmpPath.push_back(start);
			std::reverse(tmpPath.begin(),tmpPath.end());

			for(Port* p2 : tmpPath){
				path.push_back(p2);
			}
		}
		return false; //routing failure
	}

	path.clear();
	assert(currPort==end);
	while(currPort!=start){
		path.push_back(currPort);
		assert(cameFrom.find(currPort)!=cameFrom.end());
		currPort = cameFrom[currPort];
	}
	path.push_back(start);
	std::reverse(path.begin(),path.end());
	cost=cost_to_port[end];

//	std::cout << "Path::";
//	for(Port* p : path){
//		std::cout  << p->getFullName() << "-->";
//	}
//	std::cout << "\n";
//	std::cout << "LeastCostPath success!\n";
	return true;
}

int CGRAXMLCompile::HeuristicMapper::calculateCost(Port* src,
		Port* next_to_src,  Port* dest) {

	PE* srcPE = src->findParentPE();
	assert(srcPE);
	PE* nextPE = next_to_src->findParentPE();
	assert(nextPE);

	int distance = abs(nextPE->Y-srcPE->Y) + abs(nextPE->X-srcPE->X)
			       + regDiscourageFactor*((nextPE->T - srcPE->T + cgra->get_t_max())%cgra->get_t_max());

	distance = distance*PETransitionCostFactor + PortTransitionCost;

	if(srcPE!=nextPE){
		int freePorts=0;
		for(Port &p : nextPE->outputPorts){
			Module* parent = nextPE->getParent();
			if(parent->connections[&p].empty()) continue;
			if(p.getNode()==NULL){
				freePorts++;
			}
		}
		distance = distance + (15 - freePorts)*UOPCostFactor;
	}

//	int unmappedMemNodeCount=0;
//	for(DFGNode* node : this->sortedNodeList){
//		if(node->isMemOp()){
//			if(node->rootDP==NULL){
//				unmappedMemNodeCount++;
//			}
//		}
//	}
//	dfg->unmappedMemOps = unmappedMemNodeCount;

	if((next_to_src->getName().compare("P")==0)
	   || (next_to_src->getName().compare("I1")==0)
	   || (next_to_src->getName().compare("I2")==0)){

		FU* fu = next_to_src->getMod()->getFU();
		if((fu->supportedOPs.find("LOAD")!=fu->supportedOPs.end())&&(dest==next_to_src)){
			double memrescost_dbl = (double)this->dfg->unmappedMemOps/(double)cgra->freeMemNodes;
			memrescost_dbl = memrescost_dbl*(double)MEMResourceCost;
			distance = distance + (int)memrescost_dbl;
		}
	}


	return distance;
}

void CGRAXMLCompile::HeuristicMapper::printMappingLog() {

	struct util{
		void static repeatedPush(std::stringstream& ss, std::string pushStr, int count){
			for (int i = 0; i < count; ++i) {
				ss << pushStr;
			}
		}
	};

	std::map<int,std::map<int,std::map<int,std::vector<std::string>>>> lineMatrix;

	for (int t = 0; t < cgra->get_t_max(); ++t) {
		for (int y = 0; y < cgra->get_y_max(); ++y) {
			for (int x = 0; x < cgra->get_x_max(); ++x) {

				std::stringstream peHeader;
				std::stringstream fuHeader;
				std::stringstream dpHeader;
				std::stringstream dpOp;

				peHeader << "PE_" << t << y << x  << ",";
				PE* pe = cgra->PEArr[t][y][x];

				int fuCount=0;
				int dpCount=0;
				for(Module* mod : pe->subModules){
					if(FU* fu = dynamic_cast<FU*>(mod)){
						fuCount++;
						for(Module* mod : fu->subModules){
							if(DataPath* dp = dynamic_cast<DataPath*>(mod)){
								dpCount++;
							}
						}
					}
				}

				int inputPortCount=pe->inputPorts.size();
				int outputPortCount=pe->outputPorts.size();
				int totalColumns = dpCount+inputPortCount+outputPortCount;

				for(Module* mod : pe->subModules){
					if(FU* fu = dynamic_cast<FU*>(mod)){
						fuHeader << fu->getName() << ",";
						for(Module* mod : fu->subModules){
							if(DataPath* dp = dynamic_cast<DataPath*>(mod)){
								dpHeader << dp->getName() << ",";
								if(dp->getMappedNode()){
									dpOp << dp->getMappedNode()->idx << ":" << dp->getMappedNode()->op << ",";
								}
								else{
									dpOp << "---,";
								}
							}
						}
					}
				}

				for(Port& ip : pe->inputPorts){
					dpHeader << ip.getName() << ",";
					if(ip.getNode()){
						dpOp << ip.getNode()->idx << ":" << ip.getNode()->op << ",";
					}
					else{
						dpOp << "---,";
					}
				}

				for(Port& op : pe->outputPorts){
					dpHeader << op.getName() << ",";
					if(op.getNode()){
						dpOp << op.getNode()->idx << ":" << op.getNode()->op << ",";
					}
					else{
						dpOp << "---,";
					}
				}

				util::repeatedPush(peHeader,",",totalColumns);
				util::repeatedPush(fuHeader,",",totalColumns-fuCount);

				lineMatrix[t][y][x].push_back(peHeader.str());
				lineMatrix[t][y][x].push_back(fuHeader.str());
				lineMatrix[t][y][x].push_back(dpHeader.str());
				lineMatrix[t][y][x].push_back(dpOp.str());
			}
		}
	}

	int lineCount = 4;

	//print line matrix
	for (int t = 0; t < cgra->get_t_max(); ++t) {
		for (int y = 0; y < cgra->get_y_max(); ++y) {
			for (int l = 0; l < lineCount; ++l) {
				if(t>0 || y>0){
					if(l!=3)continue;
				}
				for (int x = 0; x < cgra->get_x_max(); ++x) {
					assert(lineMatrix[t][y][x].size()==lineCount);
					mappingLog << lineMatrix[t][y][x][l] << ",";
				}
				mappingLog << "\n";
			}
//			mappingLog << "\n";
		}
		mappingLog << "************************************\n";
	}
}

bool CGRAXMLCompile::HeuristicMapper::LeastCostPathDjk(Port* start, Port* end,
		std::vector<Port*>& path, int& cost, DFGNode* node,
		std::map<Port*, std::set<DFGNode*> >& mutexPaths) {

	//	std::cout << "LeastCoastPath started with start=" << start->getFullName() << " to end=" << end->getFullName() << "\n";

		std::map<Port*,int> cost_to_port;
		std::map<Port*,Port*> cameFrom;

		struct port_heuristic{
			Port* p;
			int heuristic;

			int calc_heuristic(Port* src, Port* dest){
				PE* srcPE = src->findParentPE();
				assert(srcPE);
				PE* destPE = dest->findParentPE();
				assert(destPE);

				CGRA* currCGRA = srcPE->getCGRA();
				assert(currCGRA);

				int dist_dest = std::abs(destPE->Y - srcPE->Y) + std::abs(destPE->X - srcPE->X)
				                + std::abs((destPE->T - srcPE->T + currCGRA->get_t_max())%currCGRA->get_t_max());
				return dist_dest;
			}

			port_heuristic(Port* p, Port* dest){
				this->p=p;
				heuristic=calc_heuristic(p,dest);
			}

			port_heuristic(Port* p, int cost){
				this->p=p;
				this->heuristic=cost;
			}

			port_heuristic(Port* p, Port* dest, int cost){
				this->p=p;
				this->heuristic=cost*100 + calc_heuristic(p,dest);
			}

			bool operator<(const port_heuristic& rhs) const{
				return this->heuristic > rhs.heuristic;
			}

	//		bool operator>(const port_heuristic& rhs) const{
	//			return this->heuristic > rhs.heuristic;
	//		}

		};

	//	std::queue<Port*> q;
		std::priority_queue<port_heuristic> q;

		q.push(port_heuristic(start,0));
	//	path.push_back(start);

		cost_to_port[start]=0;

		Port* currPort;
		while(!q.empty()){
			port_heuristic curr = q.top();
			currPort = curr.p;
			q.pop();

	//		std::cout << "currPort=" << currPort->getFullName() << "\n";

			if(currPort == end){
				break;
			}

			std::vector<Port*> nextPorts = currPort->getMod()->connections[currPort];
			if(currPort->getType()==OUT){
				if(currPort->getMod()->getParent()){
					for(Port* p : currPort->getMod()->getParent()->connections[currPort]){
	//					std::cout << currPort->getMod()->getParent()->getName() << "***************\n";
						nextPorts.push_back(p);
					}
				}
			}

	//		std::cout << "nextPorts size = " << nextPorts.size() << "\n";
			for(Port* nextPort : nextPorts){
				bool isNextPortFree=false;
				bool isNextPortMutex=false;
				if(enableMutexPaths){
					if(nextPort->getNode()==NULL){
						isNextPortFree=true;
					}
					else if(dfg->mutexBBs[nextPort->getNode()->BB].find(node->BB)!=dfg->mutexBBs[nextPort->getNode()->BB].end()){
						// next BB is mutually exclusive with current nodes BB, then this can be mapped.
						isNextPortFree=true;
						isNextPortMutex=true;
						mutexPaths[nextPort].insert(nextPort->getNode());
						mutexPaths[nextPort].insert(node);
					}
				}
				else{
					if(nextPort->getNode()==NULL){
						isNextPortFree=true;
					}
				}
				if(isNextPortFree){ // unmapped port
	//				std::cout << "\tnextPort=" << nextPort->getFullName() << ",";
					int nextPortCost = cost_to_port[currPort] + calculateCost(currPort,nextPort,end);
	//				std::cout << "cost=" << nextPortCost << "\n";
					if(isNextPortMutex){
						//no cost is added in using mutually exclusive routes
						nextPortCost = cost_to_port[currPort];
					}

					if(cost_to_port.find(nextPort)!=cost_to_port.end()){
						if(cost_to_port[nextPort] > nextPortCost){
							cost_to_port[nextPort]=nextPortCost;
							cameFrom[nextPort]=currPort;
							q.push(port_heuristic(nextPort,end,nextPortCost));
						}
					}
					else{
						cost_to_port[nextPort]=nextPortCost;
						cameFrom[nextPort]=currPort;
						q.push(port_heuristic(nextPort,end,nextPortCost));
					}
				}
				else{
	//				std::cout << "\t[MAPPED="<< nextPort->node->idx << "]nextPort=" << nextPort->getFullName() << ",";
				}
			}
		}


		if(currPort!=end){
	//		std::cout << "LeastCostPath failed!\n";
			path.clear();
			while(currPort!=start){
				path.push_back(currPort);
				assert(cameFrom.find(currPort)!=cameFrom.end());
				currPort = cameFrom[currPort];
			}
			path.push_back(start);
			std::reverse(path.begin(),path.end());
			return false; //routing failure
		}

		path.clear();
		assert(currPort==end);
		while(currPort!=start){
			path.push_back(currPort);
			assert(cameFrom.find(currPort)!=cameFrom.end());
			currPort = cameFrom[currPort];
		}
		path.push_back(start);
		std::reverse(path.begin(),path.end());
		cost=cost_to_port[end];

	//	std::cout << "Path::";
	//	for(Port* p : path){
	//		std::cout  << p->getFullName() << "-->";
	//	}
	//	std::cout << "\n";
	//	std::cout << "LeastCostPath success!\n";
		return true;



}

bool CGRAXMLCompile::HeuristicMapper::dataPathCheck(DataPath* dp,
		DFGNode* node) {

	if(dp->getMappedNode()!=NULL){
		return false;
	}

	PE* pe = dp->getPE();
	FU* fu = dp->getFU();
	CGRA* cgra = dp->getCGRA();
	int fanoutNode = node->children.size();

	int latency = fu->supportedOPs[node->op];
	int next_t = (pe->T + latency) % cgra->get_t_max();
	PE* nextPE = cgra->PEArr[next_t][pe->Y][pe->X];
	FU* nextFU = static_cast<FU*>(nextPE->getSubMod(fu->getName()));
	DataPath* nextDP = static_cast<DataPath*>(nextFU->getSubMod(dp->getName()));

	Port* outputPort = nextDP->getOutPort("T");
	assert(outputPort);

	std::queue<Port*> q;
	q.push(outputPort);

	std::set<DataPath*> connectingDPs;
	std::set<Port*> alreadyVisitedPorts;
	while(!q.empty()){
		Port* currPort = q.front();
		q.pop();

		if(alreadyVisitedPorts.find(currPort)!=alreadyVisitedPorts.end()){
			continue;
		}
		alreadyVisitedPorts.insert(currPort);

		if(DataPath* newDP = dynamic_cast<DataPath*>(currPort->getMod())){
			if(newDP != dp){
				connectingDPs.insert(newDP);
			}
		}

		for(Port* p : currPort->getMod()->connections[currPort]){
			q.push(p);
		}

		if(currPort->getType()==OUT){
			for(Port* p : currPort->getMod()->getParent()->connections[currPort]){
				q.push(p);
			}
		}

		if(connectingDPs.size() == fanoutNode){
			return true;
		}
	}
	return false;
}

void CGRAXMLCompile::HeuristicMapper::printMappingLog2() {
	mappingLog2 << "--------------------------------------------------------\n";
	for(DFGNode& node : dfg->nodeList){
		if(node.rootDP!=NULL){
			mappingLog2 << "node=" << node.idx << ",mapped=" << node.rootDP->getPE()->getName() << "\n";
			for(DFGNode* parent : node.parents){
				if(parent->rootDP!=NULL){
					mappingLog2 << "\t" << "parent:" << parent->idx << "\n";
					for(std::pair<Port*,int> pair : parent->routingPorts){
						Port* p = pair.first;
						if(pair.second==node.idx){
							mappingLog2 << "\t\t" << p->getFullName() << "\n";
						}
					}
				}
			}
		}
	}
	mappingLog2 << "--------------------------------------------------------\n";
}

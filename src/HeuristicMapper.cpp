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


	for (DFGNode* node : sortedNodeList){
		unmappedNodes.push(node);
	}

	std::cout << "MAP begin...\n";

	while(!unmappedNodes.empty()){
		DFGNode* node = unmappedNodes.top();
		unmappedNodes.pop();

		std::stringstream MapHeader;
		MapHeader << "current node = " << node->idx;
		MapHeader << ",unmapped nodes = " << unmappedNodes.size();
		MapHeader << ",mapped nodes = " << mappedNodes.size();
		MapHeader << ",II = " << cgra->get_t_max();
		MapHeader << ",btCredits = " << backTrackCredits;
		MapHeader << ",PEType = " << this->cgra->peType;
		MapHeader << ",DPs = " << this->cgra->numberofDPs;
		MapHeader << "\n";

		std::cout << MapHeader.str();
		mappingLog << MapHeader.str();

		bool isEstRouteSucc=false;

		//fill the routing information
		if(estimatedRouteInfo.find(node)==estimatedRouteInfo.end()){
			//the routes are not estimated.
			std::priority_queue<dest_with_cost> estimatedRoutes;
			isEstRouteSucc = estimateRouting(*node,estimatedRoutes);

			if(!isEstRouteSucc){
				if(enableBackTracking){
					if(backTrackCredits==0){
						std::cout << "route estimation failed...\n";
						std::cout << "Map Failed!.\n";
						mappingLog << "route estimation failed...\n";
						mappingLog << "Map Failed!.\n";
						return false;
					}
					backTrackCredits--;

					DFGNode* prevNode = mappedNodes.top();
					mappedNodes.pop();
					unmappedNodes.push(node);
					unmappedNodes.push(prevNode);
					prevNode->clear();
					std::cout << "route estimation failed...\n";
					mappingLog << "route estimation failed...\n";
					continue;
				}
				else{
					while(!mappedNodes.empty()){
						DFGNode* prevNode = mappedNodes.top();
						mappedNodes.pop();
						prevNode->clear();
					}
					std::cout << "Map Failed!.\n";
					mappingLog << "Map Failed!.\n";
					return false;
				}
			}
			estimatedRouteInfo[node]=estimatedRoutes;
		}

		bool isRouteSucc=false;

		std::cout << "estimatedRouteInfo[node].size = " << estimatedRouteInfo[node].size() << "\n";
		mappingLog << "estimatedRouteInfo[node].size = " << estimatedRouteInfo[node].size() << "\n";
		if(!estimatedRouteInfo[node].empty()){
			isRouteSucc=Route(*node,estimatedRouteInfo[node]);
		}
		else{
			if(mappedNodes.empty()){
				mappingLog << "Map Failed!.\n";
				std::cout << "Map Failed!.\n";
				return false;
			}
		}


		if(!isRouteSucc){
			if(mappedNodes.empty()){
				mappingLog << "Map Failed!.\n";
				std::cout << "Map Failed!.\n";
				return false;
			}

			if(enableBackTracking){
				if(backTrackCredits==0){
					mappingLog << "Map Failed!.\n";
					std::cout << "Map Failed!.\n";
					return false;
				}
				backTrackCredits--;

				DFGNode* prevNode = mappedNodes.top();
				mappedNodes.pop();
				unmappedNodes.push(node);
				unmappedNodes.push(prevNode);

				prevNode->clear();
				estimatedRouteInfo.erase(node);
				continue;
			}
			else{
				while(!mappedNodes.empty()){
					DFGNode* prevNode = mappedNodes.top();
					mappedNodes.pop();
					prevNode->clear();
				}
				mappingLog << "Map Failed!.\n";
				std::cout << "Map Failed!.\n";
				return false;
			}
		}

		this->printMappingLog();
		backTrackCredits = std::min(this->backTrackLimit,backTrackCredits+1);
		mappedNodes.push(node);
	}

	mappingLog << "Map Success!.\n";
	std::cout << "Map Success!.\n";
	return true;
}

bool CGRAXMLCompile::HeuristicMapper::estimateRouting(DFGNode& node,
		std::priority_queue<dest_with_cost>& estimatedRoutes) {

	std::map<DFGNode*,std::vector<Port*>> possibleStarts;
	std::map<DFGNode*,Port*> alreadyMappedChildPorts;

//	std::cout << "EstimateEouting begin...\n";

	for(DFGNode* parent : node.parents){
//		std::cout << "parent = " << parent->idx << "\n";
		if(parent->rootDP!=NULL){ //already mapped
			assert(parent->rootDP->getOutputDP()->getOutPort("T"));
			possibleStarts[parent].push_back(parent->rootDP->getOutputDP()->getOutPort("T"));

			for(Port* p : parent->routingPorts){
				possibleStarts[parent].push_back(p);
			}
		}
	}

	for(DFGNode* child : node.children){
		if(child->rootDP!=NULL){// already mapped
//			std::cout << "child="<< child->idx << ",childOpType=" << node.childrenOPType[child] << "\n";
			assert(child->rootDP->getInPort(node.childrenOPType[child]));
			alreadyMappedChildPorts[child]=child->rootDP->getInPort(node.childrenOPType[child]);
		}
	}

	std::vector<DataPath*> candidateDests;
	for (int t = 0; t < cgra->get_t_max(); ++t) {
		for (int y = 0; y < cgra->get_y_max(); ++y) {
			for (int x = 0; x < cgra->get_x_max(); ++x) {
				PE* currPE = cgra->PEArr[t][y][x];
				for(Module* submod : currPE->subModules){
					if(FU* fu = dynamic_cast<FU*>(submod)){

						if(fu->supportedOPs.find(node.op)==fu->supportedOPs.end()){
							continue;
						}

						if(fu->currOP.compare(node.op)==0){
							for(Module* submodFU : fu->subModules){
								if(DataPath* dp = dynamic_cast<DataPath*>(submodFU)){
									if(dp->getMappedNode()==NULL){
										candidateDests.push_back(dp);
									}
								}
							}
						}
						else if(fu->currOP.compare("NOP")==0){
							for(Module* submodFU : fu->subModules){
								if(DataPath* dp = dynamic_cast<DataPath*>(submodFU)){
									if(dp->getMappedNode()==NULL){
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
			Port* destPort = dest->getInPort(parent->getOPtype(&node));

			std::priority_queue<cand_src_with_cost> res;

			for(Port* startCand : pair.second){
				int cost;
				std::vector<Port*> path;
				std::map<Port*,std::set<DFGNode*>> mutexPaths;
				bool pathExist = LeastCostPath(startCand,destPort,path,cost,parent,mutexPaths);
				if(!pathExist){
//					std::cout << "Estimate Path Failed :: " << startCand->getFullName() << "--->" << destPort->getFullName() << "\n";
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
			assert(parentFU->supportedOPs.find(node.op)!=parentFU->supportedOPs.end());
			int latency = parentFU->supportedOPs[node.op];
			Port* destPort = dest->getOutputPort(latency);

			std::map<Port*,std::set<DFGNode*>> mutexPaths;
		    pathExistMappedChild = pathExistMappedChild & LeastCostPath(destPort,childDestPort,path,cost,&node,mutexPaths);

		    dest_child_with_cost dcwc(child,childDestPort,destPort,cost);
		    alreadyMappedChilds.push(dcwc);
		}
	    if(!pathExistMappedChild) continue;  //if it cannot be mapped to child abort the estimation for this dest

	    assert(pathFromParentExist);
	    assert(pathExistMappedChild);
		dest_with_cost dest_with_cost_ins(parentStartLocs,alreadyMappedChilds,dest);
		estimatedRoutes.push(dest_with_cost_ins);
	}
//	std::cout << "EstimateEouting end!\n";
	return !estimatedRoutes.empty();
}


bool CGRAXMLCompile::HeuristicMapper::Route(DFGNode& node,
		std::priority_queue<dest_with_cost>& estimatedRoutes) {

	std::cout << "Route begin...\n";

	bool routeSucc=false;
	dest_with_cost currDest;
	while(!estimatedRoutes.empty()){
		currDest = estimatedRoutes.top();
		estimatedRoutes.pop();

		bool alreadMappedChildRouteSucc=true; //this will change to false if failure in alreadyMappedChilds
		std::map<DFGNode*,std::vector<Port*>> mappedChildPaths;
		std::map<DFGNode*,std::map<Port*,std::set<DFGNode*>>> mappedChildMutexPaths;
		while(!currDest.alreadyMappedChilds.empty()){
			dest_child_with_cost dest_child_with_cost_ins = currDest.alreadyMappedChilds.top();
			currDest.alreadyMappedChilds.pop();

			int cost;
			std::vector<Port*> path;
			Port* src = dest_child_with_cost_ins.startPort;
			Port* dest = dest_child_with_cost_ins.childDest;

			std::map<Port*,std::set<DFGNode*>> mutexPaths;
			alreadMappedChildRouteSucc = alreadMappedChildRouteSucc & LeastCostPath(src,dest,path,cost,&node,mutexPaths);
			if(alreadMappedChildRouteSucc){
				assignPath(&node,dest_child_with_cost_ins.child,path);
				mappedChildPaths[dest_child_with_cost_ins.child]=path;
				mappedChildMutexPaths[dest_child_with_cost_ins.child]=mutexPaths;
			}
			else{
				std::cout << "Route Failed :: from=" << src->getFullName() << "--> to=" << dest->getFullName() << "\n";
				for(Port* p : path){
					if(p->getMod()->getPE()){
						std::cout << p->getMod()->getPE()->getName() << "-->";
					}
				}
				std::cout << "\n";
				break;
			}
		}

		if(!alreadMappedChildRouteSucc){
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
		std::map<DFGNode*,std::map<Port*,std::set<DFGNode*>>> mappedParentMutexPaths;
		while(!currDest.parentStartLocs.empty()){
			parent_cand_src_with_cost pcswc = currDest.parentStartLocs.top();
			currDest.parentStartLocs.pop();
			DFGNode* parent = pcswc.parent;
			std::priority_queue<cand_src_with_cost> &q = pcswc.cswc;

			bool succ;
			while(!q.empty()){
				cand_src_with_cost cand_src_with_cost_ins = q.top();
				q.pop();
				Port* src = cand_src_with_cost_ins.src;
				Port* dest = cand_src_with_cost_ins.dest;
				std::vector<Port*> path;
				std::map<Port*,std::set<DFGNode*>> mutexPath;
				int cost;
				succ = LeastCostPath(src,dest,path,cost,parent,mutexPath);
				if(succ){
					assignPath(parent,&node,path);
					mappedParentMutexPaths[parent]=mutexPath;
//					for(Port* p : path){
//						std::cout << p->getFullName() << ",\n";
//					}
//					std::cout << "\n";
					break;
				}
				else{
					std::cout << "Route Failed :: from=" << src->getFullName() << "--> to=" << dest->getFullName() << "\n";
				}
			}
			if(!succ){
				parentRoutSucc=false; // at least one parent failed to route, try a new dest
				break;
			}
		}

		if(parentRoutSucc){ //all parents routed succesfull + all mapped childs are connected
			routeSucc=true;
			std::cout << "node=" << node.idx <<",op=" << node.op << " is mapped to " << currDest.dest->getPE()->getName() << "\n";
			std::cout << "routing info ::\n";
			for(DFGNode* parent : node.parents){
				std::cout << "parent routing port size = " << parent->routingPorts.size() << "\n";
				for(Port* p : parent->routingPorts){
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
			currDest.dest->assignNode(&node);
			node.rootDP = currDest.dest;
			break;
		}
	}


	if(routeSucc){
		std::cout << "Route success...\n";
		return true;
	}
	else{
		currDest.dest->assignNode(&node);
		node.rootDP = currDest.dest;
		node.clear();
		std::cout << "Route failed...\n";
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

	for(Port* p : path){
		p->node = src;
		src->routingPorts.push_back(p);
		src->routingPortDestMap[p]=dest;
	}

}

bool CGRAXMLCompile::HeuristicMapper::LeastCostPath(Port* start, Port* end,
		std::vector<Port*>& path, int& cost, DFGNode* node, std::map<Port*,std::set<DFGNode*>>& mutexPaths) {

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
				if(nextPort->node==NULL){
					isNextPortFree=true;
				}
				else if(dfg->mutexBBs[nextPort->node->BB].find(node->BB)!=dfg->mutexBBs[nextPort->node->BB].end()){
					// next BB is mutually exclusive with current nodes BB, then this can be mapped.
					isNextPortFree=true;
					isNextPortMutex=true;
					mutexPaths[nextPort].insert(nextPort->node);
					mutexPaths[nextPort].insert(node);
				}
			}
			else{
				if(nextPort->node==NULL){
					isNextPortFree=true;
				}
			}
			if(isNextPortFree){ // unmapped port
//				std::cout << "\tnextPort=" << nextPort->getFullName() << ",";
				int nextPortCost = cost_to_port[currPort] + calculateCost(currPort,nextPort);
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

int CGRAXMLCompile::HeuristicMapper::calculateCost(Port* src,
		Port* next_to_src) {

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
			if(p.node==NULL){
				freePorts++;
			}
		}
		distance = distance + (15 - freePorts)*UOPCostFactor;
	}

	if((next_to_src->getName().compare("P")==0)
	   && (next_to_src->getName().compare("I1")==0)
	   && (next_to_src->getName().compare("I2")==0)){

		FU* fu = next_to_src->getMod()->getFU();
		if(fu->supportedOPs.find("LOAD")!=fu->supportedOPs.end()){
			distance = distance + MEMResourceCost;
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
					if(ip.node){
						dpOp << ip.node->idx << ":" << ip.node->op << ",";
					}
					else{
						dpOp << "---,";
					}
				}

				for(Port& op : pe->outputPorts){
					dpHeader << op.getName() << ",";
					if(op.node){
						dpOp << op.node->idx << ":" << op.node->op << ",";
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
				for (int x = 0; x < cgra->get_x_max(); ++x) {
					assert(lineMatrix[t][y][x].size()==lineCount);
					mappingLog << lineMatrix[t][y][x][l] << ",";
				}
				mappingLog << "\n";
			}
			mappingLog << "\n";
		}
		mappingLog << "************************************\n";
	}
}

void CGRAXMLCompile::HeuristicMapper::printMappingLog2() {
}

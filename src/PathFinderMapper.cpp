/*
 * PathFinderMapper.cpp
 *
 *  Created on: 31 Mar 2018
 *      Author: manupa
 */

#include "PathFinderMapper.h"

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


} /* namespace CGRAXMLCompile */

bool CGRAXMLCompile::PathFinderMapper::LeastCostPathAstar(Port* start,
		Port* end, std::vector<Port*>& path, int& cost, DFGNode* node,
		std::map<Port*, std::set<DFGNode*> >& mutexPaths, DFGNode* currNode) {

	//	std::cout << "LeastCoastPath started with start=" << start->getFullName() << " to end=" << end->getFullName() << "\n";

		std::map<Port*,int> cost_to_port;
		std::map<Port*,Port*> cameFrom;

		path.clear();
		mutexPaths.clear();

		bool detailedDebug=false;
	//	if(currNode->idx==85)detailedDebug=true;

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

	//		std::vector<Port*> nextPorts = currPort->getMod()->connections[currPort];
	//		if(currPort->getType()==OUT){
	//			if(currPort->getMod()->getParent()){
	//				for(Port* p : currPort->getMod()->getParent()->connections[currPort]){
	////					std::cout << currPort->getMod()->getParent()->getName() << "***************\n";
	//					nextPorts.push_back(p);
	//				}
	//			}
	//		}
			std::vector<Port*> nextPorts = currPort->getMod()->getNextPorts(currPort,this);

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
				if(true){ // unmapped port
	if(detailedDebug)				std::cout << "\tnextPort=" << nextPort->getFullName() << ",";
					int nextPortCost = cost_to_port[currPort] + calculateCost(currPort,nextPort,end);
	if(detailedDebug)				std::cout << "cost=" << nextPortCost << "\n";
					if(isNextPortMutex){
						//no cost is added in using mutually exclusive routes
						nextPortCost = cost_to_port[currPort];
					}

					if(nextPortCost <= cost_to_port[currPort]){
						std::cout << "nextPortCost = " << nextPortCost << "\n";
						std::cout << "cost_to_port[currPort] = " << cost_to_port[currPort] << "\n";
					}
					assert(nextPortCost > cost_to_port[currPort]);

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
			assert(currPort != cameFrom[currPort]);
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

bool CGRAXMLCompile::PathFinderMapper::estimateRouting(DFGNode* node,
		std::priority_queue<dest_with_cost>& estimatedRoutes,
		DFGNode** failedNode) {

	std::map<DFGNode*,std::vector<Port*>> possibleStarts;
	std::map<DFGNode*,Port*> alreadyMappedChildPorts;

	bool detailedDebug=false;
	if(node->idx==17)detailedDebug=true;

//	std::cout << "EstimateEouting begin...\n";

	for(DFGNode* parent : node->parents){
//		std::cout << "parent = " << parent->idx << "\n";
		if(parent->rootDP!=NULL){ //already mapped
			assert(parent->rootDP->getOutputDP()->getOutPort("T"));
//			possibleStarts[parent].push_back(parent->rootDP->getOutputDP()->getOutPort("T"));

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

										if(node->blacklistDest.find(dp)==node->blacklistDest.end()){
											candidateDests.push_back(dp);
										}
									}
								}
							}
						}
						else if(fu->currOP.compare("NOP")==0){
							for(Module* submodFU : fu->subModules){
								if(DataPath* dp = dynamic_cast<DataPath*>(submodFU)){
									if(dp->getMappedNode()==NULL){
//									if(dataPathCheck(dp,&node)){

										if(node->blacklistDest.find(dp)==node->blacklistDest.end()){
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
	}

	std::cout << "Candidate Dests = " << candidateDests.size() << "\n";
	if(candidateDests.empty()) return false;
//	assert(candidateDests.size()!=0);
//	node->blacklistDest.clear();

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
				path.clear();
				if(!pathExist){
if(detailedDebug)			    std::cout << "Estimate Path Failed :: " << startCand->getFullName() << "--->" << destPort->getFullName() << "\n";
					continue;
				}
				res.push(cand_src_with_cost(startCand,destPort,cost));
			}
			if(res.empty()){
				pathFromParentExist=false;
				*failedNode=parent;
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

		    if(!pathExistMappedChild){
		    	*failedNode=child;
		    	break;
		    }

		    dest_child_with_cost dcwc(child,childDestPort,destPort,cost);
		    alreadyMappedChilds.push(dcwc);
		}
	    if(!pathExistMappedChild){
	    	if(detailedDebug)				std::cout << "Estimating Path Failed!\n";
	    	continue;  //if it cannot be mapped to child abort the estimation for this dest
	    }

	    assert(pathFromParentExist);
	    assert(pathExistMappedChild);
		dest_with_cost dest_with_cost_ins(parentStartLocs,alreadyMappedChilds,dest,node,0,this->dfg->unmappedMemOps,this);
		estimatedRoutes.push(dest_with_cost_ins);
	}
//	std::cout << "EstimateEouting end!\n";
	if(estimatedRoutes.empty()) assert(*failedNode!=NULL);
	return !estimatedRoutes.empty();

}

bool CGRAXMLCompile::PathFinderMapper::Route(DFGNode* node,
		std::priority_queue<dest_with_cost>& estimatedRoutes,
		DFGNode** failedNode) {


	std::cout << "Route begin...\n";

	int parentRoutingPortCount=0;
	int routedParents=0;

	for(DFGNode* parent : node->parents){
		int thisParentNodeCount=0;
		if(parent->rootDP!=NULL){
			thisParentNodeCount = parent->routingPorts.size();
		}

//		if(thisParentNodeCount>0){
//			routedParents++;
//			thisParentNodeCount--; //remove the T port in the cout
//		}
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

		if(currDest.dest->getMappedNode()!=NULL){
			std::cout << "currDest is not NULL \n";
			std::cout << "currDP:" << currDest.dest->getName() << ",currPE:" << currDest.dest->getPE()->getName() << "\n";
			std::cout << "currNode:" << currDest.dest->getMappedNode()->idx << "\n";
		}
		assert(currDest.dest->getMappedNode()==NULL);
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
					pathTmp.clear();
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
				path.clear();
			}
			if(!alreadMappedChildRouteSucc){
				*failedNode = dest_child_with_cost_ins.child;
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
//					bool routedParent=true;
//					if(parent->routingPorts.size()==0){ //unrouted parent
//						routedParent=false;
//					}
					assignPath(parent,node,path);
					mappedParentMutexPaths[parent]=mutexPath;
					addedRoutingParentPorts += path.size();
//					if(routedParent){
						addedRoutingParentPorts-=1;
//					}
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
				path.clear();
			}
			if(!succ){
				*failedNode = parent;
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

//		assert(parentRoutingPortCountEnd==parentRoutingPortCount+addedRoutingParentPorts);
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
//		assert(parentRoutingPortCountEnd==parentRoutingPortCount);
		assert(*failedNode!=NULL);
		return false;
	}

}

int CGRAXMLCompile::PathFinderMapper::calculateCost(Port* src,
		Port* next_to_src, Port* dest) {

	PE* srcPE = src->findParentPE();
	assert(srcPE);
	PE* nextPE = next_to_src->findParentPE();
	assert(nextPE);

	int distance = abs(nextPE->Y-srcPE->Y) + abs(nextPE->X-srcPE->X)
			       + regDiscourageFactor*((nextPE->T - srcPE->T + cgra->get_t_max())%cgra->get_t_max());

	distance = distance*PETransitionCostFactor + next_to_src->getCongCost() + PortTransitionCost;
	assert(distance>0);


	if(srcPE!=nextPE){
		int freePorts=0;

		for(Port &p : nextPE->outputPorts){
			Module* parent = nextPE->getParent();
			if(parent->getNextPorts(&p,this).empty()) continue;
			if(p.getNode()==NULL){
				freePorts++;
			}
		}

//		for(Port &p : nextPE->inputPorts){
//			Module* parent = nextPE->getParent();
//			if(parent->getFromPorts(&p,this).empty()) continue;
//			if(p.getNode()==NULL){
//				freePorts++;
//			}
//		}

//		distance = distance + (nextPE->outputPorts.size() + nextPE->inputPorts.size() - freePorts)*UOPCostFactor;
//		distance = distance + (nextPE->outputPorts.size() - freePorts)*UOPCostFactor;
//		distance = distance + (1 + nextPE->outputPorts.size() - freePorts)*UOPCostFactor;
		distance = distance + (nextPE->outputPorts.size()*2-(freePorts))*UOPCostFactor;

		if(nextPE->outputPorts.size()*2 < freePorts){
			std::cout << "outportsize = " << nextPE->outputPorts.size() << "\n";
			std::cout << "freePorts = " << freePorts << "\n";
		}

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
	assert(distance>0);

	if((next_to_src->getName().compare("P")==0)
	   || (next_to_src->getName().compare("I1")==0)
	   || (next_to_src->getName().compare("I2")==0)){

		FU* fu = next_to_src->getMod()->getFU();
		if((fu->supportedOPs.find("LOAD")!=fu->supportedOPs.end())&&(dest==next_to_src)){
			double memrescost_dbl = (double)this->dfg->unmappedMemOps/(double)cgra->freeMemNodes;
			memrescost_dbl = memrescost_dbl*(double)MEMResourceCost;
			distance = distance + (int)memrescost_dbl;
			if(this->dfg->unmappedMemOps == cgra->freeMemNodes){
				distance = distance + MRC*10;
			}
		}
	}

	assert(distance>0);
	return distance;

}

bool CGRAXMLCompile::PathFinderMapper::Map(CGRA* cgra, DFG* dfg) {

	std::stack<DFGNode*> mappedNodes;
	std::stack<DFGNode*> unmappedNodes;
	std::map<DFGNode*,std::priority_queue<dest_with_cost>> estimatedRouteInfo;

	int backTrackCredits=this->backTrackLimit;

	//Disable mutex paths to test pathfinder
	this->enableMutexPaths=false;


	this->cgra = cgra;
	this->dfg =dfg;
//	SortSCCDFG();
	SortTopoGraphicalDFG();

	std::string mappingLogFileName = fNameLog1 + cgra->peType + "_DP" + std::to_string(this->cgra->numberofDPs) + "_II=" + std::to_string(cgra->get_t_max()) + "_MTP=" + std::to_string(enableMutexPaths) + ".mapping.csv";
	std::string mappingLog2FileName = fNameLog1 + cgra->peType + "_DP" + std::to_string(this->cgra->numberofDPs) + "_II=" + std::to_string(cgra->get_t_max()) + "_MTP=" + std::to_string(enableMutexPaths) + ".routeInfo.log";
	mappingLog.open(mappingLogFileName.c_str());
	mappingLog2.open(mappingLog2FileName.c_str());

	bool mapSuccess=false;

	for (int i = 0; i < this->maxIter; ++i) {

		while(!mappedNodes.empty()){
			mappedNodes.pop();
		}
		while(!unmappedNodes.empty()){
			unmappedNodes.pop();
		}

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
			MapHeader << ",mutexPathEn = " << this->enableMutexPaths;
			MapHeader << ",Iter = " << i;
			MapHeader << "\n";

			std::cout << MapHeader.str();
			mappingLog << MapHeader.str();

			bool isEstRouteSucc=false;

			//fill the routing information
			if(estimatedRouteInfo.find(node)==estimatedRouteInfo.end()){
				//the routes are not estimated.
				std::priority_queue<dest_with_cost> estimatedRoutes;
				DFGNode* failedNode;
				isEstRouteSucc = estimateRouting(node,estimatedRoutes,&failedNode);

				if(!isEstRouteSucc){
					printMappingLog();
					printMappingLog2();
					if(enableBackTracking){
						if(backTrackCredits==0 || failedNode==NULL){
							std::cout << "route estimation failed...\n";
							std::cout << "Map Failed!.\n";
							mappingLog << "route estimation failed...\n";
							mappingLog << "Map Failed!.\n";

							mappingLog.close();
							mappingLog2.close();
							return false;
						}
						backTrackCredits--;

	//					DFGNode* prevNode = mappedNodes.top();
	//					mappedNodes.pop();
	//					unmappedNodes.push(node);
	//					unmappedNodes.push(prevNode);
	//					prevNode->clear(this->dfg);
	//					std::cout << "route estimation failed...\n";
	//					mappingLog << "route estimation failed...\n";
	//					continue;

						//				DFGNode* prevNode = mappedNodes.top();
						//				mappedNodes.pop();
						//				unmappedNodes.push(node);
						//				unmappedNodes.push(prevNode);
						//
						//				prevNode->clear(this->dfg);
						//				estimatedRouteInfo.erase(node);


										assert(failedNode!=NULL);
										unmappedNodes.push(node);
										removeFailedNode(mappedNodes,unmappedNodes,failedNode);
										failedNode->blacklistDest.insert(failedNode->rootDP);
										(failedNode)->clear(this->dfg);
										estimatedRouteInfo.erase(node);
										estimatedRouteInfo.erase(failedNode);

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
			DFGNode* failedNode=NULL;

			std::cout << "estimatedRouteInfo[node].size = " << estimatedRouteInfo[node].size() << "\n";
			mappingLog << "estimatedRouteInfo[node].size = " << estimatedRouteInfo[node].size() << "\n";
			if(!estimatedRouteInfo[node].empty()){
				isRouteSucc=Route(node,estimatedRouteInfo[node],&failedNode);
				if(!isRouteSucc) std::cout << "BLAAAAAAAAAAA!\n";
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
					assert(failedNode!=NULL);
					backTrackCredits--;

	//				DFGNode* prevNode = mappedNodes.top();
	//				mappedNodes.pop();
	//				unmappedNodes.push(node);
	//				unmappedNodes.push(prevNode);
	//
	//				prevNode->clear(this->dfg);
	//				estimatedRouteInfo.erase(node);

					unmappedNodes.push(node);
					removeFailedNode(mappedNodes,unmappedNodes,failedNode);
					failedNode->blacklistDest.insert(failedNode->rootDP);
					(failedNode)->clear(this->dfg);
					estimatedRouteInfo.erase(node);
					estimatedRouteInfo.erase(failedNode);
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
		mapSuccess=updateCongestionCosts();
		if(mapSuccess){
			break;
		}
		clearCurrMapping();
		estimatedRouteInfo.clear();
	}

	if(mapSuccess){
		mappingLog << "Map Success!.\n";
		mappingLog2 << "Map Success!.\n";
		this->printMappingLog();
		this->printMappingLog2();

		std::cout << "Map Success!.\n";
		mappingLog.close();
		mappingLog2.close();

		std::cout << "Checking conflict compatibility!\n";
		checkConflictedPortCompatibility();
		return true;
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

void CGRAXMLCompile::PathFinderMapper::assignPath(DFGNode* src, DFGNode* dest,
		std::vector<Port*> path) {


	std::cout << "assigning path from:" << src->idx << " to:" << dest->idx << "\n";

	int srcPortCount=0;
	for(Port* p : path){

//		if(p->getName().compare("T")==0){
//			assert(p->getNode()==src);
//		}

		if(p->getNode() == src) {
			srcPortCount++;
			continue;
		}

		p->setNode(src,this);
		congestedPorts[p].insert(src);
		p->increaseConflictedUse();

		if(std::find(src->routingPorts.begin(),src->routingPorts.end(),std::make_pair(p,dest->idx))==src->routingPorts.end()){
			if(std::find(src->routingPorts.begin(),src->routingPorts.end(),std::make_pair(p,src->idx))==src->routingPorts.end()){
				src->routingPorts.push_back(std::make_pair(p,dest->idx));
			}
			else{
				std::cout << p->getFullName() << "\n";
				assert(p->getName().compare("T")==0);
			}
		}
//		src->routingPortDestMap[p]=dest->idx;
	}
	std::cout << "srcPortCount = " << srcPortCount << "\n";


}

bool CGRAXMLCompile::PathFinderMapper::updateCongestionCosts() {
	bool noCongestion=true;
	for(std::pair<Port*,std::set<DFGNode*>> pair : congestedPorts){
		Port* p = pair.first;
		if(pair.second.size() > 1){
			std::cout << "CONGESTION:" << p->getFullName();
			for(DFGNode* node : pair.second){
				std::cout << "," << node->idx;
			}
			std::cout << "\n";
			p->increastCongCost();
			noCongestion=false;
		}

		if(p->getHistoryCost() > 0){
			std::cout << "HISTORY_COST :: " << p->getFullName() << "," << p->getHistoryCost() << "\n";
		}

	}
    congestedPorts.clear();
    if(noCongestion) std::cout << "noCongestion!\n";
    return noCongestion;
}


bool CGRAXMLCompile::PathFinderMapper::clearCurrMapping(){
	for(DFGNode* node : sortedNodeList){
		std::cout << "CLEARING :: node=" << node->idx << ",destDP=" << node->rootDP->getName() << ",destPE=" << node->rootDP->getPE()->getName() << "\n";
		node->clear(this->dfg);
	}

	std::stack<Module*> searchStack;
	searchStack.push(this->cgra);

	while(!searchStack.empty()){
		Module* top = searchStack.top();
		searchStack.pop();
		for(Port& p : top->inputPorts){
			assert(p.getNode()==NULL);
		}
		for(Port& p : top->internalPorts){
			assert(p.getNode()==NULL);
		}
		for(Port& p : top->outputPorts){
			assert(p.getNode()==NULL);
		}
		for(Module* submod : top->subModules){
			searchStack.push(submod);
		}
	}
	return true;
}

bool CGRAXMLCompile::PathFinderMapper::checkConflictedPortCompatibility() {

	std::stack<Module*> searchStack;
	searchStack.push(this->cgra);

	while(!searchStack.empty()){
		Module* top = searchStack.top();
		searchStack.pop();
		for(Port& p : top->inputPorts){
			if(p.getNode()!=NULL){
				for(Port* cp : top->getConflictPorts(&p)){
					std::cout << "p : " << p.getFullName() << ", cp : " << cp->getFullName() << "\n";
					if(cp!=NULL){
						std::cout << "Conflict ERR!\n";
						std::cout << p.getFullName() << ":" << p.getNode()->idx << "," << cp->getFullName() << ":" << cp->getNode()->idx << "\n";
					}
					assert(cp==NULL);
				}
			}
		}
		for(Port& p : top->internalPorts){
			if(p.getNode()!=NULL){
				for(Port* cp : top->getConflictPorts(&p)){
					std::cout << "p : " << p.getFullName() << ", cp : " << cp->getFullName() << "\n";
					if(cp!=NULL){
						std::cout << "Conflict ERR!\n";
						std::cout << p.getFullName() << ":" << p.getNode()->idx << "," << cp->getFullName() << ":" << cp->getNode()->idx << "\n";
					}
					assert(cp==NULL);
				}
			}
		}
		for(Port& p : top->outputPorts){
			if(p.getNode()!=NULL){
				for(Port* cp : top->getConflictPorts(&p)){
					std::cout << "p : " << p.getFullName() << ", cp : " << cp->getFullName() << "\n";
					if(cp!=NULL){
						std::cout << "Conflict ERR!\n";
						std::cout << p.getFullName() << ":" << p.getNode()->idx << "," << cp->getFullName() << ":" << cp->getNode()->idx << "\n";
					}
					assert(cp==NULL);
				}
			}
		}
		for(Module* submod : top->subModules){
			searchStack.push(submod);
		}
	}

}

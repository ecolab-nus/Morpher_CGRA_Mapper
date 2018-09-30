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
#include <unordered_set>

namespace CGRAXMLCompile {


} /* namespace CGRAXMLCompile */

bool CGRAXMLCompile::PathFinderMapper::LeastCostPathAstar(LatPort start,
		LatPort end, DataPath* endDP, std::vector<LatPort>& path, int& cost, DFGNode* node,
		std::map<Port*, std::set<DFGNode*> >& mutexPaths, DFGNode* currNode) {

	//	std::cout << "LeastCoastPath started with start=" << start->getFullName() << " to end=" << end->getFullName() << "\n";

		std::map<LatPort,int> cost_to_port;
		std::map<LatPort,LatPort> cameFrom;

		path.clear();
		mutexPaths.clear();

		bool detailedDebug=false;
//		if(currNode->idx==29)detailedDebug=true;

//		struct port_heuristic{
//			Port* p;
//			int heuristic;
//
//			int calc_heuristic(Port* src, Port* dest){
//				PE* srcPE = src->findParentPE();
//				assert(srcPE);
//				PE* destPE = dest->findParentPE();
//				assert(destPE);
//
//				CGRA* currCGRA = srcPE->getCGRA();
//				assert(currCGRA);
//
//				int dist_dest = std::abs(destPE->Y - srcPE->Y) + std::abs(destPE->X - srcPE->X)
//				                + std::abs((destPE->T - srcPE->T + currCGRA->get_t_max())%currCGRA->get_t_max());
//				return dist_dest;
//			}
//
//			port_heuristic(Port* p, Port* dest){
//				this->p=p;
//				heuristic=calc_heuristic(p,dest);
//			}
//
//			port_heuristic(Port* p, int cost){
//				this->p=p;
//				this->heuristic=cost;
//			}
//
//			port_heuristic(Port* p, Port* dest, int cost){
//				this->p=p;
//				this->heuristic=cost*100 + calc_heuristic(p,dest);
//			}
//
//			bool operator<(const port_heuristic& rhs) const{
//				return this->heuristic > rhs.heuristic;
//			}
//
//	//		bool operator>(const port_heuristic& rhs) const{
//	//			return this->heuristic > rhs.heuristic;
//	//		}
//
//		};

		struct port_heuristic{
					LatPort p;
					int heuristic;

					int calc_heuristic(LatPort src, LatPort dest){
						PE* srcPE = src.second->findParentPE();
						assert(srcPE);
						PE* destPE = dest.second->findParentPE();
						assert(destPE);

						CGRA* currCGRA = srcPE->getCGRA();
						assert(currCGRA);

						int dist_dest = std::abs(destPE->Y - srcPE->Y) + std::abs(destPE->X - srcPE->X)
						                + std::abs(dest.first-src.first);
						return dist_dest;
					}

					port_heuristic(LatPort p, LatPort dest){
						this->p=p;
						heuristic=calc_heuristic(p,dest);
					}

					port_heuristic(LatPort p, int cost){
						this->p=p;
						this->heuristic=cost;
					}

					port_heuristic(LatPort p, LatPort dest, int cost){
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

		LatPort currPort;
		std::vector<LatPort> deadEnds;


		std::map<LatPort,std::unordered_set<Port*>> paths;
		paths[start].insert(start.second);

		while(!q.empty()){
			port_heuristic curr = q.top();
			currPort = curr.p;
			q.pop();

	if(detailedDebug) std::cout << "currPort=" << currPort.second->getFullName() << "\n";

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
			std::vector<LatPort> nextPorts = currPort.second->getMod()->getNextPorts(currPort,this);

	//		std::cout << "nextPorts size = " << nextPorts.size() << "\n";
			int q_len = q.size();



			for(LatPort nextLatPort : nextPorts){
				Port* nextPort = nextLatPort.second;

				if(nextLatPort.first > end.first) continue; //continue if the next port has higher latency
				assert(nextLatPort.first - currPort.first <= 1);

				//visiting the past port but if the latency is different then its not usable
				//need to check whether its visited on the same path
//				if(std::find(paths[currPort].begin(),paths[currPort].end(),nextPort) != paths[currPort].end()){
//					continue;
//				}
				if(paths[currPort].find(nextPort) != paths[currPort].end()){
					continue;
				}


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
					int nextPortCost = cost_to_port[currPort] + calculateCost(currPort,nextLatPort,end);

					if(nextPort->getNode() == node){
						nextPortCost = cost_to_port[currPort];
					}

					if(checkRecParentViolation(node,nextLatPort)) {
						std::cout << "Port is not inserted, since it violated recurrence parent..\n";
						continue;
					}
	if(detailedDebug)				std::cout << "cost=" << nextPortCost << "\n";
//					if(isNextPortMutex){
//						//no cost is added in using mutually exclusive routes
//						nextPortCost = cost_to_port[currPort];
//					}

					if(nextPortCost < cost_to_port[currPort]){
						std::cout << "nextPortCost = " << nextPortCost << "\n";
						std::cout << "cost_to_port[currPort] = " << cost_to_port[currPort] << "\n";
					}
					assert(nextPortCost >= cost_to_port[currPort]);

					if(cost_to_port.find(nextLatPort)!=cost_to_port.end()){
						if(cost_to_port[nextLatPort] > nextPortCost){
							cost_to_port[nextLatPort]=nextPortCost;
							cameFrom[nextLatPort]=currPort;


							paths[nextLatPort]=paths[currPort];
							paths[nextLatPort].insert(nextLatPort.second);
							q.push(port_heuristic(nextLatPort,end,nextPortCost));
						}
						else{
	if(detailedDebug)		std::cout << "Port is not inserted..\n";
						}
					}
					else{
						cost_to_port[nextLatPort]=nextPortCost;
						cameFrom[nextLatPort]=currPort;

						paths[nextLatPort]=paths[currPort];
						paths[nextLatPort].insert(nextLatPort.second);
						q.push(port_heuristic(nextLatPort,end,nextPortCost));
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
			path.clear();
			for(LatPort p : deadEnds){
				std::vector<LatPort> tmpPath;
				while(p !=start){
					tmpPath.push_back(p);
					assert(cameFrom.find(p)!=cameFrom.end());
					p = cameFrom[p];
				}
				tmpPath.push_back(start);
				std::reverse(tmpPath.begin(),tmpPath.end());

				for(LatPort p2 : tmpPath){
					path.push_back(p2);
				}
			}

//			if(currNode->idx == 29){
//				std::cout << "LeastCostPath failed!\n";
//				std::cout << "Path::";
//				for(LatPort p : path){
////					if()
//					std::cout  << p.second->getFullName() << ",lat=" << p.first << "-->\n";
//				}
//				std::cout << "\n";
//			}


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

		cost += endDP->getPotOutputPort(currNode)->getCongCost();



//		if(currNode->idx == 9){
//			std::cout << "Path::";
//			for(LatPort p : path){
//				std::cout  << p.second->getFullName() << ",lat=" << p.first << "-->\n";
//			}
//			std::cout << "\n";
//			std::cout << "LeastCostPath success!\n";
//		}

		//check if paths is working
		if(paths[end].size() != path.size()){
			std::cout << "paths[end] size = " << paths[end].size() << ",path.size() = " << path.size() << "\n";
		}
		assert(paths[end].size() == path.size());
//		for (int i = 0; i < path.size(); ++i) {
//			assert(paths[end][i] == path[i].second);
//		}

		return true;




}


bool CGRAXMLCompile::PathFinderMapper::estimateRouting(DFGNode* node,
		std::priority_queue<dest_with_cost>& estimatedRoutes,
		DFGNode** failedNode) {

	std::map<DFGNode*,std::vector<Port*>> possibleStarts;
	std::map<DFGNode*,Port*> alreadyMappedChildPorts;

	bool detailedDebug=false;
	if(node->idx==41)detailedDebug=true;

//	std::cout << "EstimateEouting begin...\n";

	for(DFGNode* parent : node->parents){
//		std::cout << "parent = " << parent->idx << "\n";
		if(parent->rootDP!=NULL){ //already mapped
			assert(parent->rootDP->getOutputDP()->getOutPort("T"));
			possibleStarts[parent].push_back(parent->rootDP->getOutputDP()->getOutPort("T"));

			for(std::pair<Port*,int> pair : parent->routingPorts){
				Port* p = pair.first;
				assert(p->getLat()!=-1);
//				possibleStarts[parent].push_back(p);
			}
		}
	}

	for(DFGNode* child : node->children){
		if(child->rootDP!=NULL){// already mapped
			std::cout << "child="<< child->idx << ",childOpType=" << node->childrenOPType[child] << "\n";
			assert(child->rootDP->getLat()!=-1);
			assert(child->rootDP->getInPort(node->childrenOPType[child]));
			alreadyMappedChildPorts[child]=child->rootDP->getInPort(node->childrenOPType[child]);

			int ii = child->rootDP->getCGRA()->get_t_max();
			assert(child->rootDP->getLat()!=-1);
			alreadyMappedChildPorts[child]->setLat(child->rootDP->getLat()+ii);
		}
	}

	std::vector<DataPath*> candidateDests;
	int penalty=0;
	std::map<DataPath*,int> dpPenaltyMap;
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
									if(checkDPFree(dp,node,penalty)){
//									if(dp->getMappedNode()==NULL){
//									if(dataPathCheck(dp,&node)){

										if(node->blacklistDest.find(dp)==node->blacklistDest.end()){
											candidateDests.push_back(dp);
											dpPenaltyMap[dp] = penalty;
										}
									}
								}
							}
						}
						else if(fu->currOP.compare("NOP")==0){
							for(Module* submodFU : fu->subModules){
								if(DataPath* dp = dynamic_cast<DataPath*>(submodFU)){
									if(checkDPFree(dp,node,penalty)){
//									if(dp->getMappedNode()==NULL){
//									if(dataPathCheck(dp,&node)){

										if(node->blacklistDest.find(dp)==node->blacklistDest.end()){
											candidateDests.push_back(dp);
											dpPenaltyMap[dp] = penalty;
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

	int minLat = getlatMinStarts(possibleStarts);
	std::map<DataPath*,int> minLatDests = getLatCandDests(candidateDests,minLat);
	int ii = this->cgra->get_t_max();

	//Route Estimation
	for (int i = 0; i < 1; ++i) {
		bool pathFromParentExist=false;
		bool pathExistMappedChild=false;

		for(DataPath* dest : candidateDests){
			int minLatDestVal_prime = minLatDests[dest]+ii*i;
	//		std::cout << "Candidate Dest =" ;
	//		std::cout << dest->getPE()->getName() << ".";
	//		std::cout << dest->getFU()->getName() << ".";
	//		std::cout << dest->getName() << "\n";

	//		std::map<DFGNode*,std::priority_queue<cand_src_with_cost>> parentStartLocs;
			std::priority_queue<parent_cand_src_with_cost> parentStartLocs;
			int minLatDestVal = minLatDestVal_prime;
	    	pathFromParentExist=true;
			for(std::pair<DFGNode*,std::vector<Port*>> pair : possibleStarts){
				DFGNode* parent = pair.first;
				Port* destPort = dest->getInPort(parent->getOPtype(node));
				minLatDestVal = minLatDestVal_prime + parent->childNextIter[node]*ii;

				std::priority_queue<cand_src_with_cost> res;

				for(Port* startCand : pair.second){
					int cost;
					std::vector<LatPort> path;
					std::map<Port*,std::set<DFGNode*>> mutexPaths;
	if(detailedDebug)				std::cout << "par Estimating Path" << startCand->getFullName() << "," << startCand->getLat() << "," << "--->" << destPort->getFullName() << "," << minLatDestVal << "," << "\n";


					LatPort startCandLat = std::make_pair(startCand->getLat(),startCand); assert(startCand->getLat()!=-1);
					LatPort destPortLat = std::make_pair(minLatDestVal,destPort);

//	if(detailedDebug)               std::cout << "lat = " << destPortLat.first << ",PE=" << destPort->getMod()->getPE()->getName() << ",t=" <<  destPort->getMod()->getPE()->T << "\n";
					assert((minLatDestVal)%destPort->getMod()->getCGRA()->get_t_max()==destPort->getMod()->getPE()->T);

					bool pathExist = LeastCostPathAstar(startCandLat,destPortLat,dest,path,cost,parent,mutexPaths,node);
					path.clear();
					if(!pathExist){
	if(detailedDebug)			    std::cout << "par Estimate Path Failed :: " << startCand->getFullName() << "--->" << destPort->getFullName() << "\n";
						continue;
					}
					cost += dpPenaltyMap[dest];
					res.push(cand_src_with_cost(startCandLat,destPortLat,cost));
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

			pathExistMappedChild = true;
			std::priority_queue<dest_child_with_cost> alreadyMappedChilds;
			for(std::pair<DFGNode*,Port*> pair : alreadyMappedChildPorts){
				DFGNode* child = pair.first;
				Port* childDestPort = pair.second;
				std::vector<LatPort> path;
				int cost;

				FU* parentFU = dest->getFU();
				assert(parentFU->supportedOPs.find(node->op)!=parentFU->supportedOPs.end());
				int latency = parentFU->supportedOPs[node->op];
				Port* destPort = dest->getOutputPort(latency);

				std::map<Port*,std::set<DFGNode*>> mutexPaths;
	if(detailedDebug)				std::cout << "already child Estimating Path" << destPort->getFullName() << "," << minLatDestVal+latency << "," << "--->" << childDestPort->getFullName() << "," << childDestPort->getLat() << "," << "\n";
	if(detailedDebug)               std::cout << "lat = " << childDestPort->getLat() << ",PE=" << childDestPort->getMod()->getPE()->getName() << ",t=" <<  childDestPort->getMod()->getPE()->T << "\n";

				LatPort childDestPortLat = std::make_pair(childDestPort->getLat(),childDestPort); assert(childDestPort->getLat() != -1);
				LatPort destPortLat = std::make_pair(minLatDestVal+latency,destPort);
			    pathExistMappedChild = pathExistMappedChild & LeastCostPathAstar(destPortLat,childDestPortLat,child->rootDP,path,cost,node,mutexPaths,child);

			    if(!pathExistMappedChild){
			    	*failedNode=child;
			    	break;
			    }

			    dest_child_with_cost dcwc(child,childDestPortLat,destPortLat,cost);
			    alreadyMappedChilds.push(dcwc);
			}
		    if(!pathExistMappedChild){
		    	if(detailedDebug)				std::cout << "already child Estimating Path Failed!\n";
		    	continue;  //if it cannot be mapped to child abort the estimation for this dest
		    }

		    assert(pathFromParentExist);
		    assert(pathExistMappedChild);
			dest_with_cost dest_with_cost_ins(parentStartLocs,alreadyMappedChilds,dest,minLatDestVal,node,0,this->dfg->unmappedMemOps,this);
			estimatedRoutes.push(dest_with_cost_ins);
		}
		if(pathFromParentExist & pathExistMappedChild) break;
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
		std::map<DFGNode*,std::vector<LatPort>> mappedChildPaths;
		std::map<DFGNode*,std::map<Port*,std::set<DFGNode*>>> mappedChildMutexPaths;
		while(!currDest.alreadyMappedChilds.empty()){
			dest_child_with_cost dest_child_with_cost_ins = currDest.alreadyMappedChilds.top();
			currDest.alreadyMappedChilds.pop();

			std::vector<LatPort> possibleStarts; possibleStarts.clear();
			possibleStarts.push_back(dest_child_with_cost_ins.startPort);
			for(std::pair<Port*,int> pair : node->routingPorts){
				possibleStarts.push_back(std::make_pair(pair.first->getLat(),pair.first)); assert(pair.first->getLat() != -1);
			}

			std::priority_queue<cand_src_with_cost> q;
			std::map<Port*,std::set<DFGNode*>> mutexPathsTmp;
			std::vector<LatPort> pathTmp;
			for(LatPort p : possibleStarts){
				int cost;
				if(LeastCostPathAstar(p,dest_child_with_cost_ins.childDest,dest_child_with_cost_ins.child->rootDP,pathTmp,cost,node,mutexPathsTmp,dest_child_with_cost_ins.child)){
					pathTmp.clear();
					q.push(cand_src_with_cost(p,dest_child_with_cost_ins.childDest,cost));
				}
			}

			int cost;
			std::vector<LatPort> path;
			LatPort src = dest_child_with_cost_ins.startPort;
			LatPort dest = dest_child_with_cost_ins.childDest;

			while(!q.empty()){
				cand_src_with_cost head = q.top();
				q.pop();
				std::map<Port*,std::set<DFGNode*>> mutexPaths;
				alreadMappedChildRouteSucc = LeastCostPathAstar(head.src,dest,dest_child_with_cost_ins.child->rootDP,path,cost,node,mutexPaths,dest_child_with_cost_ins.child);
				if(alreadMappedChildRouteSucc){
					assignPath(node,dest_child_with_cost_ins.child,path);
					mappedChildPaths[dest_child_with_cost_ins.child]=path;
					mappedChildMutexPaths[dest_child_with_cost_ins.child]=mutexPaths;
					std::cout << "Route success :: from=" << src.second->getFullName() << "--> to=" << dest.second->getFullName() << "|node=" << node->idx << "\n";
					break;
				}
				else{
					std::cout << "Route Failed :: from=" << src.second->getFullName() << "--> to=" << dest.second->getFullName() << "\n";
					for(LatPort p : path){
						if(p.second->getMod()->getPE()){
							std::cout << p.second->getMod()->getPE()->getName() << "-->";
						}
					}
					std::cout << "\n";

					for(LatPort p : path){
						std::cout << p.second->getFullName() << "\n";
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
			for(std::pair<DFGNode*,std::vector<LatPort>> pair : mappedChildPaths){
				DFGNode* child = pair.first;
				for(LatPort lp : pair.second){
					Port* p = lp.second;
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
				LatPort src = cand_src_with_cost_ins.src;
				LatPort dest = cand_src_with_cost_ins.dest;
				std::vector<LatPort> path;
				std::map<Port*,std::set<DFGNode*>> mutexPath;
				int cost;
				succ = LeastCostPathAstar(src,dest,currDest.dest,path,cost,parent,mutexPath,node);
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
					std::cout << "Route Failed :: from=" << src.second->getFullName() << "--> to=" << dest.second->getFullName() << "\n";
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
			std::cout << "node=" << node->idx <<",op=" << node->op << " is mapped to " << currDest.dest->getPE()->getName() << ",lat=" << currDest.destLat << "\n";
			std::cout << "routing info ::\n";
			for(DFGNode* parent : node->parents){
				std::cout << "parent routing port size = " << parent->routingPorts.size() << "\n";
				int prev_lat =-1;
				for(std::pair<Port*,int> pair : parent->routingPorts){
					Port* p = pair.first;
//					if(node.routingPortDestMap[p]==&node){
						std::cout << "fr:" <<parent->idx << " :: ";
						std::cout << ",dest=" << pair.second << " :: ";
						std::cout << p->getFullName();
						std::cout << ",lat=" << p->getLat();


						if(mappedParentMutexPaths[parent].find(p)!=mappedParentMutexPaths[parent].end()){
							std::cout << "|mutex(";
							for(DFGNode* mutexnode : mappedParentMutexPaths[parent][p]){
								std::cout << mutexnode->idx << ",";
							}
							std::cout << ")";
						}
						std::cout << std::endl;
//					}
						if(prev_lat != -1){
//							assert(p->getLat() - prev_lat <= 1);
						}
						prev_lat = p->getLat();
				}
			}
			std::cout << "routing info done.\n";
			currDest.dest->assignNode(node,currDest.destLat,this->dfg);
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
		currDest.dest->assignNode(node,currDest.destLat,this->dfg);
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

int CGRAXMLCompile::PathFinderMapper::calculateCost(LatPort src,
		LatPort next_to_src, LatPort dest) {

	std::string srcName = src.second->getName();
//	std::cout << src->getName() << ",";

	std::string next_to_srcName = next_to_src.second->getName();
//	std::cout << next_to_srcName << "\n";

	assert(src.second);
	assert(next_to_src.second);
	assert(dest.second);

	PE* srcPE = src.second->findParentPE();
	assert(srcPE);
	PE* nextPE = next_to_src.second->findParentPE();
	assert(nextPE);

	int distance = abs(nextPE->Y-srcPE->Y) + abs(nextPE->X-srcPE->X)
			       + regDiscourageFactor*((nextPE->T - srcPE->T + cgra->get_t_max())%cgra->get_t_max());

	distance = distance*PETransitionCostFactor + next_to_src.second->getCongCost() + PortTransitionCost;
	assert(distance>0);


	if(srcPE!=nextPE){
		int freePorts=0;

		for(Port *p : nextPE->outputPorts){
			Module* parent = nextPE->getParent();
			if(parent->getNextPorts(std::make_pair(next_to_src.first,p),this).empty()) continue;
			if(p->getNode()==NULL){
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

	if((next_to_src.second->getName().compare("P")==0)
	   || (next_to_src.second->getName().compare("I1")==0)
	   || (next_to_src.second->getName().compare("I2")==0)){

		FU* fu = next_to_src.second->getMod()->getFU();
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
	this->enableMutexPaths=true;


	this->cgra = cgra;
	this->dfg = dfg;
//	SortSCCDFG();
//	SortTopoGraphicalDFG();
	sortBackEdgePriority();

	std::string mappingLogFileName = fNameLog1 + cgra->peType + "_DP" + std::to_string(this->cgra->numberofDPs)  + "_XDim=" + std::to_string(this->cgra->get_x_max()) + "_YDim=" + std::to_string(this->cgra->get_y_max()) + "_II=" + std::to_string(cgra->get_t_max()) + "_MTP=" + std::to_string(enableMutexPaths);// + ".mapping.csv";
	std::string mappingLog2FileName = fNameLog1 + cgra->peType + "_DP" + std::to_string(this->cgra->numberofDPs) + "_XDim=" + std::to_string(this->cgra->get_x_max()) + "_YDim=" + std::to_string(this->cgra->get_y_max()) + "_II=" + std::to_string(cgra->get_t_max()) + "_MTP=" + std::to_string(enableMutexPaths);// + ".routeInfo.log";

	bool mapSuccess=false;

	std::string congestionInfoFileName = mappingLogFileName + ".congestion.info";
	congestionInfoFile.open(congestionInfoFileName.c_str());

	for (int i = 0; i < this->maxIter; ++i) {


		std::string mappingLogFileName_withIter = mappingLogFileName + "_Iter=" + std::to_string(i) + ".mapping.csv";
		std::string mappingLog2FileName_withIter = mappingLog2FileName + "_Iter=" + std::to_string(i) + ".routeInfo.log";

		mappingLog.open(mappingLogFileName_withIter.c_str());
		mappingLog2.open(mappingLog2FileName_withIter.c_str());

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
			MapHeader << ",XDim = " << this->cgra->get_x_max();
			MapHeader << ",YDim = " << this->cgra->get_y_max();
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
		mapSuccess=updateCongestionCosts(i);
		if(mapSuccess){
			break;
		}
		clearCurrMapping();
		estimatedRouteInfo.clear();
		mappingLog.close();
		mappingLog2.close();
	}

//	congestionInfoFile.close();

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

		if(this->cgra->peType == "STDNOC_4REGF_1P"){
			checkRegALUConflicts();
		}
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
		std::vector<LatPort> path) {


	std::cout << "assigning path from:" << src->idx << " to:" << dest->idx << "\n";

	int srcPortCount=0;

	int prevLat = -1;
	LatPort prevPort;
	for(LatPort p : path){

		if(prevLat != -1){
			if(p.first - prevLat > 1){
				std::cout << prevPort.second->getFullName() << ",Lat = " << prevPort.first << "\n";
				std::cout << p.second->getFullName() << ",Lat = " << p.first << "\n";
			}
			assert(p.first-prevLat <= 1);
		}

//		if(p->getName().compare("T")==0){
//			assert(p->getNode()==src);
//		}

//		if(p.second->getNode() == src) {
//			srcPortCount++;
//			continue;
//		}

		p.second->setNode(src,p.first,this);
		congestedPorts[p.second].insert(src);
		p.second->increaseConflictedUse(src,this);

		if(std::find(src->routingPorts.begin(),src->routingPorts.end(),std::make_pair(p.second,dest->idx))==src->routingPorts.end()){
			if(std::find(src->routingPorts.begin(),src->routingPorts.end(),std::make_pair(p.second,src->idx))==src->routingPorts.end()){
				src->routingPorts.push_back(std::make_pair(p.second,dest->idx));
			}
			else{
				std::cout << p.second->getFullName() << "\n";
				assert(p.second->getName().compare("T")==0);
			}
		}
//		src->routingPortDestMap[p]=dest->idx;
		prevLat = p.first;
		prevPort = p;
	}
	std::cout << "srcPortCount = " << srcPortCount << "\n";


}

bool CGRAXMLCompile::PathFinderMapper::updateCongestionCosts(int iter) {
	bool noCongestion=true;

	std::set<int> conflictedTimeSteps;

	congestionInfoFile << "**********************************\n";
	congestionInfoFile << "II = " << this->cgra->get_t_max() << ",iter = " << iter << "\n";
	congestionInfoFile << "**********************************\n";

	for(std::pair<Port*,std::set<DFGNode*>> pair : congestedPorts){
		Port* p = pair.first;
		if(pair.second.size() > 1){
			for(DFGNode* node1 : pair.second){
				for(DFGNode* node2 : pair.second){
					if(node1==node2){
						continue;
					}
					if(this->dfg->isMutexNodes(node1,node2)) continue;
					std::cout << "CONGESTION:" << p->getFullName();
					congestionInfoFile << "CONGESTION:" << p->getFullName();
					for(DFGNode* node : pair.second){
						std::cout << "," << node->idx << "|BB=" << node->BB;
						congestionInfoFile << "," << node->idx << "|BB=" << node->BB;
					}
					std::cout << "\n";
					congestionInfoFile << "\n";
					p->increastCongCost();
					noCongestion=false;
					conflictedTimeSteps.insert(p->getMod()->getPE()->T);
//					break;
				}
				if(!noCongestion){
//					break;
				}
			}
		}
		if(p->getHistoryCost() > 0){
			std::cout << "HISTORY_COST :: " << p->getFullName() << "," << p->getHistoryCost() << "\n";
			congestionInfoFile << "HISTORY_COST :: " << p->getFullName() << "," << p->getHistoryCost() << "\n";
		}
	}

	bool noConflicts=true;
	for(std::pair<Port*,std::set<DFGNode*>> pair  : conflictedPorts){
		Port* p = pair.first;

		if(p->getNode()!=NULL){
			std::cout << "CONFLICT :" << p->getFullName();
			congestionInfoFile << "CONFLICT :" << p->getFullName();
			for(DFGNode* node : pair.second){
				std::cout << "," << node->idx << "|BB=" << node->BB;
				congestionInfoFile << "," << node->idx << "|BB=" << node->BB;
			}
			std::cout << ", with MAPPED = " << p->getNode()->idx << "|BB=" << p->getNode()->BB;
			std::cout << "\n";

			congestionInfoFile << ", with MAPPED = " << p->getNode()->idx << "|BB=" << p->getNode()->BB;
			congestionInfoFile << "\n";

			for (int i = 0; i < pair.second.size(); ++i) {
				p->increastCongCost();
			}
			conflictedTimeSteps.insert(p->getMod()->getPE()->T);
			noConflicts=false;
		}

		if(p->getHistoryCost() > 0){
			std::cout << "HISTORY_COST :: " << p->getFullName() << "," << p->getHistoryCost() << "\n";
			congestionInfoFile << "HISTORY_COST :: " << p->getFullName() << "," << p->getHistoryCost() << "\n";
		}
	}

	if(this->upperboundII > conflictedTimeSteps.size() + this->cgra->get_t_max()){
		this->upperboundII = conflictedTimeSteps.size() + this->cgra->get_t_max();
		this->upperboundIter = iter;
		this->upperboundFoundBy = this->cgra->get_t_max();
		std::cout << "****************************************\n";
		std::cout << "Upperbound II = " << this->upperboundII << "\n";
		std::cout << "On iter = " << iter << "\n";
		std::cout << "****************************************\n";

		congestionInfoFile << "****************************************\n";
		congestionInfoFile << "Upperbound II = " << this->upperboundII << "\n";
		congestionInfoFile << "On iter = " << iter << "\n";
		congestionInfoFile << "****************************************\n";
	}

	congestionInfoFile << std::endl;

    congestedPorts.clear();
    conflictedPorts.clear();
    conflictedTimeStepMap.clear();
    if(noCongestion) std::cout << "noCongestion!\n";
    if(noConflicts) std::cout << "noConflicts!\n";

    if(noCongestion) congestionInfoFile << "noCongestion!\n";
    if(noConflicts) congestionInfoFile << "noConflicts!\n";

    return noCongestion&noConflicts;
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
		for(Port* p : top->inputPorts){
			assert(p->getNode()==NULL);
		}
		for(Port* p : top->internalPorts){
			assert(p->getNode()==NULL);
		}
		for(Port* p : top->outputPorts){
			assert(p->getNode()==NULL);
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
		for(Port* p : top->inputPorts){
			if(p->getNode()!=NULL){
				for(Port* cp : top->getConflictPorts(p)){
					std::cout << "p : " << p->getFullName() << ", cp : " << cp->getFullName() << "\n";
					if(cp->getNode()!=NULL){
						std::cout << "Conflict ERR!\n";
						std::cout << p->getFullName() << ":" << p->getNode()->idx << "," << cp->getFullName() << ":" << cp->getNode()->idx << "\n";
					}
					assert(cp->getNode()==NULL);
				}
			}
		}
		for(Port* p : top->internalPorts){
			if(p->getNode()!=NULL){
				for(Port* cp : top->getConflictPorts(p)){
					std::cout << "p : " << p->getFullName() << ", cp : " << cp->getFullName() << "\n";
					if(cp->getNode()!=NULL){
						std::cout << "Conflict ERR!\n";
						std::cout << p->getFullName() << ":" << p->getNode()->idx << "," << cp->getFullName() << ":" << cp->getNode()->idx << "\n";
					}
					assert(cp->getNode()==NULL);
				}
			}
		}
		for(Port* p : top->outputPorts){
			if(p->getNode()!=NULL){
				for(Port* cp : top->getConflictPorts(p)){
					std::cout << "p : " << p->getFullName() << ", cp : " << cp->getFullName() << "\n";
					if(cp->getNode()!=NULL){
						std::cout << "Conflict ERR!\n";
						std::cout << p->getFullName() << ":" << p->getNode()->idx << "," << cp->getFullName() << ":" << cp->getNode()->idx << "\n";
					}
					assert(cp->getNode()==NULL);
				}
			}
		}
		for(Module* submod : top->subModules){
			searchStack.push(submod);
		}
	}

}

bool CGRAXMLCompile::PathFinderMapper::checkRegALUConflicts() {
	for (int t = 0; t < this->cgra->get_t_max(); ++t) {
		int timeslice_count = 0;
		for (int y = 0; y < this->cgra->get_y_max(); ++y) {
			for (int x = 0; x < this->cgra->get_x_max(); ++x) {

				PE* currPE = this->cgra->PEArr[t][y][x];
				int usage=0;

				for(Module* submod_fu : currPE->subModules){
					if(FU* fu = dynamic_cast<FU*>(submod_fu)){
						for(Module* submod_dp : fu->subModules){
							if(DataPath* dp = dynamic_cast<DataPath*>(submod_dp)){
								if(dp->getMappedNode()!=NULL){
									std::cout << dp->getFullName() << ":" << dp->getMappedNode()->idx << ",";
									usage++;
									break;
								}
							}
						}
					}
				}

				for(RegFile* RF : currPE->allRegs){
					for (int i = 0; i < RF->get_nWRPs(); ++i) {
						std::string wrpName = "WRP" + std::to_string(i);
						Port* wrp = RF->getInPort(wrpName);
						if(wrp->getNode()!=NULL){
							std::cout << wrp->getFullName() << ":" << wrp->getNode()->idx << ",";
							usage++;
						}
					}

					for (int i = 0; i < RF->get_nRDPs(); ++i) {
						std::string rdpName = "RDP" + std::to_string(i);
						Port* rdp = RF->getOutPort(rdpName);
						if(rdp->getNode()!=NULL){
							std::cout << rdp->getFullName() << ":" << rdp->getNode()->idx << ",";
							usage++;
						}
					}
				}

				if(timeslice_count <= usage - 1){
					timeslice_count = usage - 1;
				}

				std::cout << "\n";

			}
		}

		std::cout << "t=" << t << "," << "timeslice=" << timeslice_count << "\n";
	}
}

bool CGRAXMLCompile::PathFinderMapper::checkDPFree(DataPath* dp, DFGNode* node, int& penalty) {
	PE* currPE = dp->getPE();
	FU* currFU = dp->getFU();


	int numberFUs=0;
	int numberUsedFUs=0;
	int numberConstants=0;
	bool memfu_found=false;
	bool memop_found=false;
	for(Module* submod_fu : currPE->subModules){
		if(FU* fu = dynamic_cast<FU*>(submod_fu)){
			int dp_used = 0;
			if(!memfu_found) memfu_found = fu->isMEMFU();
			for(Module* submod_dp : fu->subModules){
				if(DataPath* dp = dynamic_cast<DataPath*>(submod_dp)){
					if(dp->getMappedNode()!=NULL){
						dp_used = 1;
						if(!memop_found) memop_found = dp->getMappedNode()->isMemOp();
						if(dp->getMappedNode()->hasConst){
							numberConstants++;
						}
					}
				}
			}
			numberUsedFUs += dp_used;
			numberFUs+=1;
		}
	}

	//increment for the current node
	numberUsedFUs++;
	if(node->hasConst){
		numberConstants++;
	}

	assert(this->dfg->unmappedMemOps == this->dfg->unmappedMemOpSet.size());
	assert(this->cgra->freeMemNodes == this->cgra->freeMemNodeSet.size());

    penalty = 0;
	if(memfu_found){
		int memnode_const_count=0;
		for(DFGNode* memnode : this->dfg->unmappedMemOpSet){
			if(memnode->hasConst){
				memnode_const_count++;
			}
		}

		int freeMemPEs_const = 0;
		for(DataPath* memdp : this->cgra->freeMemNodeSet){
			int memPEConstants = 0;
			int memUsedFUs = 0;
			PE* memPE = memdp->getPE();
			for(Module* submod_fu : memPE->subModules){
				if(FU* fu = dynamic_cast<FU*>(submod_fu)){
					int dp_used = 0;
					for(Module* submod_dp : fu->subModules){
						if(DataPath* dp = dynamic_cast<DataPath*>(submod_dp)){
							if(dp->getMappedNode()!=NULL){
								dp_used = 1;
								if(!memop_found) memop_found = dp->getMappedNode()->isMemOp();
								if(dp->getMappedNode()->hasConst){
									memPEConstants++;
								}
							}
						}
					}
					memUsedFUs += dp_used;
				}
			}
			if(memUsedFUs + memPEConstants <= 1){
				freeMemPEs_const++;
			}
		}


		if((!node->isMemOp()) && (!memop_found)){
			double penalty_ratio_dbl = (double)memnode_const_count / (double)freeMemPEs_const;
			double penalty_dbl = penalty_ratio_dbl* (double)MRC;
			penalty = (int)penalty_dbl;
		}
	}

	//with current node it should be less than or equal to number of FUs
	if(numberConstants + numberUsedFUs <= numberFUs || numberFUs == 1){
		if(dp->getMappedNode()==NULL){
			return true;
		}
	}
	return false;
}

bool CGRAXMLCompile::PathFinderMapper::updateConflictedTimeSteps(int timeStep,
		int conflicts) {

	int presentConflicts = conflictedTimeStepMap[timeStep];
	if(conflicts > presentConflicts){
		conflictedTimeStepMap[timeStep] = conflicts;
		return true;
	}
	return false;
}

int CGRAXMLCompile::PathFinderMapper::getTimeStepConflicts(int timeStep) {
	return conflictedTimeStepMap[timeStep];
}

void CGRAXMLCompile::PathFinderMapper::sortBackEdgePriority() {
	sortedNodeList.clear();

	struct BEDist{
		DFGNode* parent;
		DFGNode* child;
		int dist;
		BEDist(DFGNode* parent,DFGNode* child,int dist) : parent(parent), child(child), dist(dist){}
		bool operator<(const BEDist& other) const{
			if(dist == other.dist){
				return true;
			}
			return dist > other.dist;
		}
//		bool operator==(const BEDist& other) const{
//			return parent==other.parent & child==other.child;
//		}
	};

	std::set<BEDist> backedges;

	for(DFGNode& node : dfg->nodeList){

		if(node.idx == 97){
			std::cout << "node_idx:97,node_ASAP:" << node.ASAP << "\n";
		}
		for(DFGNode* child : node.children){

			if(node.idx == 97){
				std::cout << "child_idx:" << child->idx << "child_ASAP:" << child->ASAP << "\n";
			}

			if(child->ASAP <= node.ASAP){
				std::cout << "inserting for : node=" << node.idx << ",child:" << child->idx << "\n";
				backedges.insert(BEDist(&node,child,node.ASAP-child->ASAP));
			}
		}
	}

	for(DFGNode& node : dfg->nodeList){
		for(DFGNode* recParent : node.recParents){
			backedges.insert(BEDist(&node,recParent,node.ASAP-recParent->ASAP));
		}
	}

	std::map<DFGNode*,std::vector<DFGNode*>> beparentAncestors;
//	std::map<DFGNode*,std::vector<DFGNode*>> bechildAncestors;

	for(BEDist be : backedges){
		std::cout << "BE PARENT = " << be.parent->idx << ",dist=" << be.dist << "\n";
		std::cout << "BE CHILD = " << be.child->idx << "\n";

		std::cout << "Ancestory : " << "\n";
		beparentAncestors[be.parent]=dfg->getAncestory(be.parent);
		std::cout << "\n";
//		bechildAncestors[be.child]=dfg->getAncestory(be.child);
	}

//	std::vector<DFGNode*> mergedAncestory;
	mergedAncestories.clear();
	std::map<DFGNode*,DFGNode*> mergedKeys;
	for(BEDist be : backedges){
//		write a logic to merge ancestories where if one be's child is present in some other be's parent's ancesotory'
		bool merged=false;
		for(std::pair<DFGNode*,std::vector<DFGNode*>> pair : mergedAncestories){
			DFGNode* key = pair.first;
			if(std::find(mergedAncestories[key].begin(),mergedAncestories[key].end(),be.child) != mergedAncestories[key].end()){
				std::cout << "Merging :: " << key->idx << ", " << be.parent->idx << "\n";
				mergedAncestories[key] = dfg->mergeAncestory(mergedAncestories[key],beparentAncestors[be.parent]); merged=true;
				mergedKeys[be.parent]=key;
			}
		}
		if(!merged){
			mergedAncestories[be.parent]=dfg->getAncestory(be.parent);
			mergedKeys[be.parent]=be.parent;
		}
	}

	for(BEDist be : backedges){
		assert(mergedKeys.find(be.parent) != mergedKeys.end());
		std::vector<DFGNode*> ancestoryNodes = mergedAncestories[mergedKeys[be.parent]];
		for(DFGNode* ancestorNode : ancestoryNodes){
			if(std::find(sortedNodeList.begin(),sortedNodeList.end(),ancestorNode) == sortedNodeList.end()){
				sortedNodeList.push_back(ancestorNode);
			}
		}
		std::cout << "BE PARENT = " << be.parent->idx << ",dist=" << be.dist << "\n";
		if(std::find(sortedNodeList.begin(),sortedNodeList.end(),be.parent) == sortedNodeList.end()){
			sortedNodeList.push_back(be.parent);
		}

		ancestoryNodes = dfg->getAncestory(be.child);
		for(DFGNode* ancestorNode : ancestoryNodes){
			if(std::find(sortedNodeList.begin(),sortedNodeList.end(),ancestorNode) == sortedNodeList.end()){
				sortedNodeList.push_back(ancestorNode);
			}
		}
		std::cout << "BE CHILD = " << be.child->idx << "\n";
		if(std::find(sortedNodeList.begin(),sortedNodeList.end(),be.child) == sortedNodeList.end()){
			sortedNodeList.push_back(be.child);
		}
	}

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
			if(std::find(sortedNodeList.begin(),sortedNodeList.end(),node) == sortedNodeList.end()){
				sortedNodeList.push_back(node);
			}
		}
	}

	std::cout << "***********SORTED LIST*******************\n";
	for(DFGNode* node : sortedNodeList){
		std::cout << "Node=" << node->idx << ",ASAP=" << node->ASAP << "\n";
	}
//	assert(false);

	std::reverse(sortedNodeList.begin(),sortedNodeList.end());



	int unmappedMemNodeCount=0;
	for(DFGNode* node : this->sortedNodeList){
		if(node->isMemOp()){
			if(node->rootDP==NULL){
				unmappedMemNodeCount++;
				dfg->unmappedMemOpSet.insert(node);
			}
		}
	}
	dfg->unmappedMemOps = unmappedMemNodeCount;

}

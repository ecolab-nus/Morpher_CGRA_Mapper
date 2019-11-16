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
#include <algorithm> // std::reverse
#include "DataPath.h"
#include "FU.h"

#include <stack>
#include <functional>
#include <set>
#include <iostream>
#include <sstream>
#include <unordered_set>
#include <unordered_map>
#include <memory>

namespace CGRAXMLCompile
{

} /* namespace CGRAXMLCompile */

struct hash_LatPort { 
    size_t operator()(const pair<int, CGRAXMLCompile::Port*>& p) const
    { 
        auto hash1 = hash<int>{}(p.first); 
        auto hash2 = hash<CGRAXMLCompile::Port*>{}(p.second); 
        return hash1 ^ hash2; 
    } 
}; 

bool CGRAXMLCompile::PathFinderMapper::LeastCostPathAstar(LatPort start,
														  LatPort end, DataPath *endDP, std::vector<LatPort> &path, int &cost, DFGNode *node,
														  std::map<Port *, std::set<DFGNode *>> &mutexPaths, DFGNode *currNode)
{

	//	std::cout << "LeastCoastPath started with start=" << start->getFullName() << " to end=" << end->getFullName() << "\n";

	std::unordered_map<LatPort, int, hash_LatPort> cost_to_port;
	std::unordered_map<LatPort, LatPort, hash_LatPort> cameFrom;
	std::unordered_map<LatPort, int, hash_LatPort> curr_hops_to_port;

	path.clear();
	mutexPaths.clear();

	bool detailedDebug = false;
	// if(currNode->idx==53)detailedDebug=true;

	bool lessthanII = false;
	CGRA *cgra = endDP->getCGRA();
	int II = cgra->get_t_max();
	int latDiff = end.first - start.first;
	if (latDiff < II)
		lessthanII = true;

	struct port_heuristic
	{
		LatPort p;
		int heuristic;
		std::shared_ptr<std::unordered_set<Port *>> path;
		std::shared_ptr<std::vector<LatPort>> pathVec;

		int calc_heuristic(LatPort src, LatPort dest)
		{
			PE *srcPE = src.second->findParentPE();
			assert(srcPE);
			PE *destPE = dest.second->findParentPE();
			assert(destPE);

			CGRA *currCGRA = srcPE->getCGRA();
			assert(currCGRA);

			int dist_dest = std::abs(destPE->Y - srcPE->Y) + std::abs(destPE->X - srcPE->X) + std::abs(dest.first - src.first);
			// int dist_dest = std::abs(dest.first - src.first);
			return dist_dest;
		}

		//					port_heuristic(LatPort p, LatPort dest){
		//						this->p=p;
		//						heuristic=calc_heuristic(p,dest);
		//					}

		port_heuristic(LatPort p, int cost, bool islessThanII = true)
		{
			this->p = p;
			this->heuristic = cost;
			if (!islessThanII)
			{
				this->path = std::shared_ptr<std::unordered_set<Port *>>(new std::unordered_set<Port *>());
				this->pathVec = std::shared_ptr<std::vector<LatPort>>(new std::vector<LatPort>());
			}
		}

		port_heuristic(LatPort p, LatPort dest, int cost)
		{
			this->p = p;
			this->heuristic = cost * 100 + calc_heuristic(p, dest);
		}

		port_heuristic(LatPort p, LatPort dest, int cost, std::shared_ptr<std::unordered_set<Port *>> &path)
		{
			this->p = p;
			this->heuristic = cost * 100 + calc_heuristic(p, dest);
			this->path = path;
		}

		bool operator<(const port_heuristic &rhs) const
		{
			return this->heuristic > rhs.heuristic;
		}

		//		bool operator>(const port_heuristic& rhs) const{
		//			return this->heuristic > rhs.heuristic;
		//		}
	};

	std::priority_queue<port_heuristic> q;

	q.push(port_heuristic(start, 0, lessthanII));

	//	path.push_back(start);

	cost_to_port[start] = 0;
	curr_hops_to_port[start] = 0;

	LatPort currPort;
	std::vector<LatPort> deadEnds;

	std::map<LatPort, std::shared_ptr<std::unordered_set<Port *>>> paths;

	std::unordered_set<Port *> emptyset;
	//		paths[start] = emptyset;
	//		paths[start].insert(start.second);

	std::vector<LatPort> finalPath;

	Port *newNodeDPOut = endDP->getPotOutputPort(currNode);
	std::set<Port *> newNodeDPOutCP = newNodeDPOut->getMod()->getConflictPorts(newNodeDPOut);
	std::set<Port *> endPortCP = end.second->getMod()->getConflictPorts(end.second);

	int curr_least_cost_to_end = INT32_MAX;

	while (!q.empty())
	{
		port_heuristic curr = q.top();
		currPort = curr.p;
		q.pop();
		std::unordered_set<Port *> *currPath;
		std::vector<LatPort> *currPathVec;

		if (!lessthanII)
		{
			currPath = curr.path.get();
			currPathVec = curr.pathVec.get();
			paths[currPort] = curr.path;
			if (currPort == end)
			{
				finalPath = *curr.pathVec;
			}
		}

		if (detailedDebug){
			std::cout << "currPort=" << currPort.second->getFullName() << ",";
			if(currPort.second->getType() == IN) cout << "type=IN,";
			if(currPort.second->getType() == OUT) cout << "type=OUT,";
			if(currPort.second->getType() == INT) cout << "type=INT,";
		}
		if (detailedDebug)
			std::cout << "latency = " << currPort.first << "\n";

		assert(curr_hops_to_port.find(currPort) != curr_hops_to_port.end());
		if(curr_hops_to_port[currPort] > cgra->max_hops){
			continue;
		}

		if (currPort == end)
		{
			if(cost_to_port[currPort] < curr_least_cost_to_end){
				curr_least_cost_to_end = cost_to_port[currPort];
			}
			continue;
		}

		if(cost_to_port[currPort] > curr_least_cost_to_end){
			continue;
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
		std::vector<LatPort> nextPorts = currPort.second->getMod()->getNextPorts(currPort, this);

		//		std::cout << "nextPorts size = " << nextPorts.size() << "\n";
		int q_len = q.size();

		for (LatPort nextLatPort : nextPorts)
		{
			Port *nextPort = nextLatPort.second;

			if (nextLatPort.first > end.first)
				continue; //continue if the next port has higher latency
			assert(nextLatPort.first - currPort.first <= 1);


			//visiting the past port but if the latency is different then its not usable
			//need to check whether its visited on the same path
			//				if(std::find(paths[currPort].begin(),paths[currPort].end(),nextPort) != paths[currPort].end()){
			//					continue;
			//				}
			//				assert(paths.find(currPort)!=paths.end());
			//				assert(paths[currPort].size() == pathsLatPort[currPort].size());

			if (!lessthanII)
			{
				if (currPath->find(nextPort) != currPath->end())
				{
					continue;
				}
				for (Port *cp : nextPort->getMod()->getConflictPorts(nextPort))
				{
					if (currPath->find(cp) != currPath->end())
					{
						continue;
					}
				}
			}

			if (newNodeDPOutCP.find(nextPort) != newNodeDPOutCP.end())
			{
				continue;
			}

			if (endPortCP.find(nextPort) != endPortCP.end())
			{
				continue;
			}

			//				NodeLat nl = std::make_pair(node,nextLatPort.first);
			//				if(conflictedPorts[nextPort].find(nl) != conflictedPorts[nextPort].end()){
			//					continue;
			//				}

			//				bool isNextPortFree=false;
			//				bool isNextPortMutex=false;
			//				if(enableMutexPaths){
			//					if(nextPort->getNode()==NULL){
			//						isNextPortFree=true;
			//					}
			//					else if(dfg->mutexBBs[nextPort->getNode()->BB].find(node->BB)!=dfg->mutexBBs[nextPort->getNode()->BB].end()){
			//						// next BB is mutually exclusive with current nodes BB, then this can be mapped.
			//						isNextPortFree=true;
			//						isNextPortMutex=true;
			//						mutexPaths[nextPort].insert(nextPort->getNode());
			//						mutexPaths[nextPort].insert(node);
			//					}
			//				}
			//				else{
			//					if(nextPort->getNode()==NULL){
			//						isNextPortFree=true;
			//					}
			//				}

			if (currPort.second->getMod()->regCons[std::make_pair(currPort.second, nextLatPort.second)])
			{
				assert(nextLatPort.first != currPort.first);
			}

			bool isRegConType1 = currPort.second->getName().find("REG_O") != std::string::npos &&
								 nextLatPort.second->getName().find("REG_I") != std::string::npos;
			bool isRegConType2 = currPort.second->getName().find("_RO") != std::string::npos &&
								 nextLatPort.second->getName().find("_RI") != std::string::npos;

			if (isRegConType1 || isRegConType2)
			{
				// std::cout << "src=" << currPort.second->getFullName() << ",dest=" << nextLatPort.second->getFullName() << "\n";
				if (nextLatPort.first == currPort.first)
				{
					nextLatPort.first = nextLatPort.first + 1;
				}
			}

			if (true)
			{ // unmapped port
				if (detailedDebug)
					std::cout << "\tnextPort=" << nextPort->getFullName() << ",";
				if (detailedDebug)
					std::cout << "latency = " << nextLatPort.first << ",";
				int nextPortCost = cost_to_port[currPort] + calculateCost(currPort, nextLatPort, end);
			

				if (nextPort->getNode() == node)
				{
					nextPortCost = cost_to_port[currPort];
				}

				if (checkRecParentViolation(currNode, nextLatPort))
				{
					std::cout << "Port is not inserted, since it violated recurrence parent..\n";
					continue;
				}
				if (detailedDebug)
					std::cout << "cost=" << nextPortCost << "\n";
				//					if(isNextPortMutex){
				//						//no cost is added in using mutually exclusive routes
				//						nextPortCost = cost_to_port[currPort];
				//					}

				if (nextPortCost < cost_to_port[currPort])
				{
					std::cout << "nextPortCost = " << nextPortCost << "\n";
					std::cout << "cost_to_port[currPort] = " << cost_to_port[currPort] << "\n";
				}
				assert(nextPortCost >= cost_to_port[currPort]);

				if (cost_to_port.find(nextLatPort) != cost_to_port.end())
				{
					if (cost_to_port[nextLatPort] > nextPortCost)
					{
						cost_to_port[nextLatPort] = nextPortCost;
						cameFrom[nextLatPort] = currPort;

						if(nextLatPort.first == currPort.first && nextLatPort.second->getPE() != currPort.second->getPE()){
							//next latport is inter-PE connection and it is not increasing latency
							//therefore it should be a hop
							curr_hops_to_port[nextLatPort] = curr_hops_to_port[currPort] + 1;
						}
						else if(nextLatPort.first != currPort.first){
							curr_hops_to_port[nextLatPort] = 0;
						}
						else{
							curr_hops_to_port[nextLatPort] = curr_hops_to_port[currPort];
						}	

						//							paths[nextLatPort]=paths[currPort];
						//							paths[nextLatPort].insert(nextLatPort.second);
						//							currPath.insert(currPort.second);

						//							pathsLatPort[nextLatPort]=pathsLatPort[currPort];
						//							pathsLatPort[nextLatPort].push_back(currPort);
						if (!lessthanII)
						{
							std::shared_ptr<std::unordered_set<Port *>> newPath = std::shared_ptr<std::unordered_set<Port *>>(new std::unordered_set<Port *>(*currPath));
							newPath->insert(currPort.second);
							port_heuristic ph(nextLatPort, end, nextPortCost, newPath);
							ph.pathVec = std::shared_ptr<std::vector<LatPort>>(new std::vector<LatPort>(*currPathVec));
							ph.pathVec->push_back(currPort);
							q.push(ph);
						}
					}
					else
					{
						if (detailedDebug)
							std::cout << "Port is not inserted..\n";
					}
				}
				else
				{
					cost_to_port[nextLatPort] = nextPortCost;
					cameFrom[nextLatPort] = currPort;

					if(nextLatPort.first == currPort.first && nextLatPort.second->getPE() != currPort.second->getPE()){
						//next latport is inter-PE connection and it is not increasing latency
						//therefore it should be a hop
						curr_hops_to_port[nextLatPort] = curr_hops_to_port[currPort] + 1;
					}
					else if(nextLatPort.first != currPort.first){
						curr_hops_to_port[nextLatPort] = 0;
					}	
					else{
						curr_hops_to_port[nextLatPort] = curr_hops_to_port[currPort];
					}	

					//						assert(paths.find(nextLatPort)==paths.end());
					//						paths[nextLatPort]=paths[currPort];
					//						paths[nextLatPort].insert(nextLatPort.second);
					//						paths[nextLatPort].insert(currPort.second);
					//						currPath.insert(currPort.second);

					//						pathsLatPort[nextLatPort]=pathsLatPort[currPort];
					//						pathsLatPort[nextLatPort].push_back(currPort);

					if (!lessthanII)
					{
						std::shared_ptr<std::unordered_set<Port *>> newPath = std::shared_ptr<std::unordered_set<Port *>>(new std::unordered_set<Port *>(*currPath));
						newPath->insert(currPort.second);
						port_heuristic ph(nextLatPort, end, nextPortCost, newPath);
						ph.pathVec = std::shared_ptr<std::vector<LatPort>>(new std::vector<LatPort>(*currPathVec));
						ph.pathVec->push_back(currPort);
						q.push(ph);
					}
					else
					{
						q.push(port_heuristic(nextLatPort, end, nextPortCost));
					}
				}
			}
			else
			{
				assert(false);
				if (detailedDebug)
					std::cout << "\t[MAPPED=" << nextPort->getNode()->idx << "]nextPort=" << nextPort->getFullName() << "\n";
			}
		}
		if (q.size() == q_len)
		{
			deadEnds.push_back(currPort);
		}
	}

	//		if(detailedDebug) assert(false);

	//		if(currPort!=end){
	if (cameFrom.find(end) == cameFrom.end())
	{
		path.clear();
		for (LatPort p : deadEnds)
		{
			std::vector<LatPort> tmpPath;
			while (p != start)
			{
				tmpPath.push_back(p);
				assert(cameFrom.find(p) != cameFrom.end());
				p = cameFrom[p];
			}
			tmpPath.push_back(start);
			std::reverse(tmpPath.begin(), tmpPath.end());

			for (LatPort p2 : tmpPath)
			{
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
	//		assert(currPort==end);
	//		assert(currPort==end);
	currPort = end;
	while (currPort != start)
	{
		path.push_back(currPort);
		assert(cameFrom.find(currPort) != cameFrom.end());
		assert(currPort != cameFrom[currPort]);
		currPort = cameFrom[currPort];
	}
	path.push_back(start);
	std::reverse(path.begin(), path.end());
	cost = cost_to_port[end];

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
	if (!lessthanII)
	{
		paths[end]->insert(end.second);
		finalPath.push_back(end);
		if (paths[end]->size() != path.size())
		{
			std::cout << "paths[end] size = " << paths[end]->size() << ",path.size() = " << path.size() << "\n";

			std::cout << "path = \n";
			for (LatPort lp : path)
			{
				std::cout << lp.second->getFullName() << ",lat=" << lp.first << "\n";
				if (paths[end]->find(lp.second) == paths[end]->end())
				{
					std::cout << "Not found in paths!\n";
					//					assert(false);
				}
			}

			std::cout << "paths[end] = \n";
			for (Port *p : *paths[end])
			{
				std::cout << p->getFullName() << "\n";
			}

			std::cout << "finalPath = \n";
			for (LatPort lp : finalPath)
			{
				std::cout << lp.second->getFullName() << ",lat=" << lp.first << "\n";
			}
			//				assert(false);
		}
		//			assert(paths[end]->size() == path.size());
		path.clear();
		path = finalPath;
	}
	//		for (int i = 0; i < path.size(); ++i) {
	//			assert(paths[end][i] == path[i].second);
	//		}

	return true;
}

bool CGRAXMLCompile::PathFinderMapper::estimateRouting(DFGNode *node,
													   std::priority_queue<dest_with_cost> &estimatedRoutes,
													   DFGNode **failedNode)
{

	std::map<DFGNode *, std::vector<Port *>> possibleStarts;
	std::map<DFGNode *, Port *> alreadyMappedChildPorts;

	bool detailedDebug = false;
	// if(node->idx==1)detailedDebug=true;

	//	std::cout << "EstimateEouting begin...\n";

	for (DFGNode *parent : node->parents)
	{
		//		std::cout << "parent = " << parent->idx << "\n";
		if (parent->rootDP != NULL)
		{ //already mapped
			assert(parent->rootDP->getOutputDP()->getOutPort("T"));
			possibleStarts[parent].push_back(parent->rootDP->getOutputDP()->getOutPort("T"));

			for (std::pair<Port *, int> pair : parent->routingPorts)
			{
				Port *p = pair.first;
				assert(p->getLat() != -1);
				//				possibleStarts[parent].push_back(p);
			}
		}
	}

	for (DFGNode *child : node->children)
	{
		if (child->rootDP != NULL)
		{ // already mapped
			std::cout << "child=" << child->idx << ",childOpType=" << node->childrenOPType[child] << "\n";
			assert(child->rootDP->getLat() != -1);
			if (node->childrenOPType[child] == "PS")
			{
				std::cout << "Skipping.....\n";
				continue;
			}
			assert(child->rootDP->getInPort(node->childrenOPType[child]));
			alreadyMappedChildPorts[child] = child->rootDP->getInPort(node->childrenOPType[child]);

			int ii = child->rootDP->getCGRA()->get_t_max();
			assert(child->rootDP->getLat() != -1);
			alreadyMappedChildPorts[child]->setLat(child->rootDP->getLat() + ii);
		}
		else if(child->idx == node->idx){
			//adding a placeholder as this will be modified according to the destination in consideration.
			alreadyMappedChildPorts[child] == NULL;
		}
	}

	std::vector<DataPath *> candidateDests;
	int penalty = 0;
	std::map<DataPath *, int> dpPenaltyMap;

	unordered_set<PE *> allPEs = cgra->getAllPEList();
	for (PE *currPE : allPEs)
	{
		for (Module *submod : currPE->subModules)
		{
			if (FU *fu = dynamic_cast<FU *>(submod))
			{

				if (fu->supportedOPs.find(node->op) == fu->supportedOPs.end())
				{
					continue;
				}

				if (fu->currOP.compare(node->op) == 0)
				{
					for (Module *submodFU : fu->subModules)
					{
						if (DataPath *dp = dynamic_cast<DataPath *>(submodFU))
						{

							if(cgra->is_spm_modelled){
								if(!node->base_pointer_name.empty()){
									//base pointer name is not empty
									if(dp->accesible_memvars.find(node->base_pointer_name) == dp->accesible_memvars.end()){
										//this dp does not support the variable
										continue;
									}
								}
							}
							
							if (checkDPFree(dp, node, penalty))
							{
								//									if(dp->getMappedNode()==NULL){
								//									if(dataPathCheck(dp,&node)){

								if (node->blacklistDest.find(dp) == node->blacklistDest.end())
								{
									candidateDests.push_back(dp);
									dpPenaltyMap[dp] = penalty;
								}
							}
						}
					}
				}
				else if (fu->currOP.compare("NOP") == 0)
				{
					for (Module *submodFU : fu->subModules)
					{
						if (DataPath *dp = dynamic_cast<DataPath *>(submodFU))
						{
							if (checkDPFree(dp, node, penalty))
							{
								//									if(dp->getMappedNode()==NULL){
								//									if(dataPathCheck(dp,&node)){

								if (node->blacklistDest.find(dp) == node->blacklistDest.end())
								{
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

	std::cout << "Candidate Dests = " << candidateDests.size() << "\n";
	if (candidateDests.empty())
		return false;
	//	assert(candidateDests.size()!=0);
	//	node->blacklistDest.clear();

	//	int minLat = getlatMinStarts(possibleStarts);
	int minLat = getlatMinStartsPHI(node, possibleStarts);
	std::map<DataPath *, int> minLatDests = getLatCandDests(candidateDests, minLat);
	bool changed = false;
	candidateDests = modifyMaxLatCandDest(minLatDests, node, changed);
	std::cout << "Candidate Dests = " << candidateDests.size() << "\n";
	int ii = this->cgra->get_t_max();

	int minLatSucc = 1000000000;
	std::priority_queue<dest_with_cost> estimatedRoutesTemp;

	int allowed_time_steps_for_connection = 30;
	int iterations = allowed_time_steps_for_connection / ii;

	//Route Estimation
	for (int i = 0; i < iterations; ++i)
	{
		bool pathFromParentExist = false;
		bool pathExistMappedChild = false;

		for (DataPath *dest : candidateDests)
		{
			int minLatDestVal_prime = minLatDests[dest] + ii * i;
			//		std::cout << "Candidate Dest =" ;
			//		std::cout << dest->getPE()->getName() << ".";
			//		std::cout << dest->getFU()->getName() << ".";
			//		std::cout << dest->getName() << "\n";

			//		std::map<DFGNode*,std::priority_queue<cand_src_with_cost>> parentStartLocs;
			std::priority_queue<parent_cand_src_with_cost> parentStartLocs;
			int minLatDestVal = minLatDestVal_prime;
			pathFromParentExist = true;
			for (std::pair<DFGNode *, std::vector<Port *>> pair : possibleStarts)
			{
				DFGNode *parent = pair.first;

				//Skip parent if the edge is pseudo
				if (parent->getOPtype(node) == "PS")
					continue;

				Port *destPort = dest->getInPort(parent->getOPtype(node));
				minLatDestVal = minLatDestVal_prime + parent->childNextIter[node] * ii;

				std::priority_queue<cand_src_with_cost> res;

				for (Port *startCand : pair.second)
				{
					int cost;
					std::vector<LatPort> path;
					std::map<Port *, std::set<DFGNode *>> mutexPaths;
					if (detailedDebug)
						std::cout << "par Estimating Path" << startCand->getFullName() << "," << startCand->getLat() << ","
								  << "--->" << destPort->getFullName() << "," << minLatDestVal << "," << ",parent_node = " << parent->idx
								  << "\n";

					LatPort startCandLat = std::make_pair(startCand->getLat(), startCand);
					assert(startCand->getLat() != -1);
					LatPort destPortLat = std::make_pair(minLatDestVal, destPort);

					//	if(detailedDebug)               std::cout << "lat = " << destPortLat.first << ",PE=" << destPort->getMod()->getPE()->getName() << ",t=" <<  destPort->getMod()->getPE()->T << "\n";
					assert((minLatDestVal) % destPort->getMod()->getCGRA()->get_t_max() == destPort->getMod()->getPE()->T);

					bool pathExist = false;
					{
						FU *parentFU = dest->getFU();
						assert(parentFU->supportedOPs.find(node->op) != parentFU->supportedOPs.end());
						int latency = parentFU->supportedOPs[node->op];
						Port *destPort = dest->getOutputPort(latency);
						LatPort destPortLat = std::make_pair(minLatDestVal + latency, destPort);

						if (canExitCurrPE(destPortLat))
						{
							pathExist = true;
						}
						else
						{
							std::cout << "Cannot exit from :" << destPortLat.second->getFullName() << "\n";
						}
					}

					pathExist = pathExist & LeastCostPathAstar(startCandLat, destPortLat, dest, path, cost, parent, mutexPaths, node);
					path.clear();
					if (!pathExist)
					{
						if (detailedDebug)
							std::cout << "par Estimate Path Failed :: " << startCand->getFullName() << "--->" << destPort->getFullName() << "\n";
						continue;
					}
					cost += dpPenaltyMap[dest];
					res.push(cand_src_with_cost(startCandLat, destPortLat, cost));
				}
				if (res.empty())
				{
					pathFromParentExist = false;
					*failedNode = parent;
					break;
				}
				parent_cand_src_with_cost pcswc(parent, res);
				parentStartLocs.push(pcswc);
			}

			if (!pathFromParentExist)
			{
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
			for (std::pair<DFGNode *, Port *> pair : alreadyMappedChildPorts)
			{
				DFGNode *child = pair.first;
				Port *childDestPort = pair.second;
				DataPath* childDP = child->rootDP;

				if (child->idx == node->idx)
				{
					childDestPort = dest->getInPort(node->childrenOPType[child]);
					if (detailedDebug) cout << "setting latency = " << minLatDestVal + ii << "\n";
					childDestPort->setLat(minLatDestVal + ii);
					childDP = dest;
				}

				std::vector<LatPort> path;
				int cost;

				FU *parentFU = dest->getFU();
				assert(parentFU->supportedOPs.find(node->op) != parentFU->supportedOPs.end());
				int latency = parentFU->supportedOPs[node->op];
				Port *destPort = dest->getOutputPort(latency);

				std::map<Port *, std::set<DFGNode *>> mutexPaths;
				if (detailedDebug)
					std::cout << "already child Estimating Path" << destPort->getFullName() << "," << minLatDestVal + latency << ","
							  << "--->" << childDestPort->getFullName() << "," << childDestPort->getLat() << "," << "exist_child = " << child->idx  
							  << "\n";
				if (detailedDebug)
					std::cout << "lat = " << childDestPort->getLat() << ",PE=" << childDestPort->getMod()->getPE()->getName() << ",t=" << childDestPort->getMod()->getPE()->T << "\n";

				LatPort childDestPortLat = std::make_pair(childDestPort->getLat(), childDestPort);
				assert(childDestPort->getLat() != -1);
				LatPort destPortLat = std::make_pair(minLatDestVal + latency, destPort);

				pathExistMappedChild = pathExistMappedChild & LeastCostPathAstar(destPortLat, childDestPortLat, childDP, path, cost, node, mutexPaths, child);

				if (!pathExistMappedChild)
				{
					*failedNode = child;
					break;
				}

				dest_child_with_cost dcwc(child,childDP, childDestPortLat, destPortLat, cost);
				alreadyMappedChilds.push(dcwc);
			}
			if (!pathExistMappedChild)
			{
				if (detailedDebug)
					std::cout << "already child Estimating Path Failed!\n";
				continue; //if it cannot be mapped to child abort the estimation for this dest
			}

			assert(pathFromParentExist);
			assert(pathExistMappedChild);
			dest_with_cost dest_with_cost_ins(parentStartLocs, alreadyMappedChilds, dest, minLatDestVal_prime, node, 0, this->dfg->unmappedMemOps, this);

			if (minLatDestVal_prime < minLatSucc)
			{
				minLatSucc = minLatDestVal_prime;
			}

			estimatedRoutesTemp.push(dest_with_cost_ins);
		}
		if (pathFromParentExist & pathExistMappedChild)
			break;
	}

	while (!estimatedRoutesTemp.empty())
	{
		dest_with_cost top = estimatedRoutesTemp.top();
		estimatedRoutesTemp.pop();
		if (minLatDests[top.dest] == minLatSucc || !changed)
			estimatedRoutes.push(top);
	}

	//	std::cout << "EstimateEouting end!\n";
	//	if(estimatedRoutes.empty()) assert(*failedNode!=NULL);
	return !estimatedRoutes.empty();
}

bool CGRAXMLCompile::PathFinderMapper::Route(DFGNode *node,
											 std::priority_queue<dest_with_cost> &estimatedRoutes,
											 DFGNode **failedNode)
{

	std::cout << "Route begin...\n";

	int parentRoutingPortCount = 0;
	int routedParents = 0;

	for (DFGNode *parent : node->parents)
	{
		int thisParentNodeCount = 0;
		if (parent->rootDP != NULL)
		{
			thisParentNodeCount = parent->routingPorts.size();
		}

		//		if(thisParentNodeCount>0){
		//			routedParents++;
		//			thisParentNodeCount--; //remove the T port in the cout
		//		}
		parentRoutingPortCount += thisParentNodeCount;
	}
	//	if(parentRoutingPortCount>0){
	//		parentRoutingPortCount-=1; //remove the T port in the cout
	//	}

	int addedRoutingParentPorts = 0;

	bool routeSucc = false;
	dest_with_cost currDest;
	while (!estimatedRoutes.empty())
	{
		currDest = estimatedRoutes.top();
		estimatedRoutes.pop();

		if (currDest.dest->getMappedNode() != NULL)
		{
			std::cout << "currDest is not NULL \n";
			std::cout << "currDP:" << currDest.dest->getName() << ",currPE:" << currDest.dest->getPE()->getName() << "\n";
			std::cout << "currNode:" << currDest.dest->getMappedNode()->idx << "\n";
		}
		assert(currDest.dest->getMappedNode() == NULL);
		std::cout << "alreadyMappedChilds = " << currDest.alreadyMappedChilds.size() << "\n";

		bool alreadMappedChildRouteSucc = true; //this will change to false if failure in alreadyMappedChilds
		std::map<DFGNode *, std::vector<LatPort>> mappedChildPaths;
		std::map<DFGNode *, std::map<Port *, std::set<DFGNode *>>> mappedChildMutexPaths;
		while (!currDest.alreadyMappedChilds.empty())
		{
			dest_child_with_cost dest_child_with_cost_ins = currDest.alreadyMappedChilds.top();
			currDest.alreadyMappedChilds.pop();

			std::vector<LatPort> possibleStarts;
			possibleStarts.clear();
			possibleStarts.push_back(dest_child_with_cost_ins.startPort);
			for (std::pair<Port *, int> pair : node->routingPorts)
			{
				possibleStarts.push_back(std::make_pair(pair.first->getLat(), pair.first));
				assert(pair.first->getLat() != -1);
			}

			std::priority_queue<cand_src_with_cost> q;
			std::map<Port *, std::set<DFGNode *>> mutexPathsTmp;
			std::vector<LatPort> pathTmp;
			for (LatPort p : possibleStarts)
			{
				int cost;
				if (LeastCostPathAstar(p, dest_child_with_cost_ins.childDest, dest_child_with_cost_ins.childDP, pathTmp, cost, node, mutexPathsTmp, dest_child_with_cost_ins.child))
				{
					pathTmp.clear();
					q.push(cand_src_with_cost(p, dest_child_with_cost_ins.childDest, cost));
				}
			}

			int cost;
			std::vector<LatPort> path;
			LatPort src = dest_child_with_cost_ins.startPort;
			LatPort dest = dest_child_with_cost_ins.childDest;

			while (!q.empty())
			{
				cand_src_with_cost head = q.top();
				q.pop();
				std::map<Port *, std::set<DFGNode *>> mutexPaths;
				alreadMappedChildRouteSucc = LeastCostPathAstar(head.src, dest, dest_child_with_cost_ins.childDP, path, cost, node, mutexPaths, dest_child_with_cost_ins.child);
				if (alreadMappedChildRouteSucc)
				{
					assignPath(node, dest_child_with_cost_ins.child, path);
					mappedChildPaths[dest_child_with_cost_ins.child] = path;
					mappedChildMutexPaths[dest_child_with_cost_ins.child] = mutexPaths;
					std::cout << "Route success :: from=" << src.second->getFullName() << "--> to=" << dest.second->getFullName() << "|node=" << node->idx << "\n";
					break;
				}
				else
				{
					std::cout << "Route Failed :: from=" << src.second->getFullName() << "--> to=" << dest.second->getFullName() << "\n";
					for (LatPort p : path)
					{
						if (p.second->getMod()->getPE())
						{
							std::cout << p.second->getMod()->getPE()->getName() << "-->";
						}
					}
					std::cout << "\n";

					for (LatPort p : path)
					{
						std::cout << p.second->getFullName() << "\n";
					}
				}
				path.clear();
			}
			if (!alreadMappedChildRouteSucc)
			{
				*failedNode = dest_child_with_cost_ins.child;
				break;
			}
		}

		if (alreadMappedChildRouteSucc)
		{
			for (std::pair<Port *, int> pair : node->routingPorts)
			{
				Port *p = pair.first;
				int destIdx = pair.second;
				std::cout << "to:" << destIdx << "," << p->getFullName() << "\n";
			}
		}

		if (!alreadMappedChildRouteSucc)
		{
			node->clear(this->dfg);
			continue; //try the next dest
		}
		else
		{
			std::cout << "Already Mapped child Routes....\n";
			for (std::pair<DFGNode *, std::vector<LatPort>> pair : mappedChildPaths)
			{
				DFGNode *child = pair.first;
				for (LatPort lp : pair.second)
				{
					Port *p = lp.second;
					std::cout << "to:" << child->idx << " :: ";
					std::cout << p->getFullName();
					if (mappedChildMutexPaths[child].find(p) != mappedChildMutexPaths[child].end())
					{
						std::cout << "|mutex(";
						for (DFGNode *mutexnode : mappedChildMutexPaths[child][p])
						{
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

		bool parentRoutSucc = true;
		addedRoutingParentPorts = 0;
		std::map<DFGNode *, std::map<Port *, std::set<DFGNode *>>> mappedParentMutexPaths;
		while (!currDest.parentStartLocs.empty())
		{
			parent_cand_src_with_cost pcswc = currDest.parentStartLocs.top();
			currDest.parentStartLocs.pop();
			DFGNode *parent = pcswc.parent;
			std::priority_queue<cand_src_with_cost> &q = pcswc.cswc;

			bool succ = false;
			while (!q.empty())
			{
				cand_src_with_cost cand_src_with_cost_ins = q.top();
				q.pop();
				LatPort src = cand_src_with_cost_ins.src;
				LatPort dest = cand_src_with_cost_ins.dest;
				std::vector<LatPort> path;
				std::map<Port *, std::set<DFGNode *>> mutexPath;
				int cost;
				succ = LeastCostPathAstar(src, dest, currDest.dest, path, cost, parent, mutexPath, node);
				if (succ)
				{
					//					bool routedParent=true;
					//					if(parent->routingPorts.size()==0){ //unrouted parent
					//						routedParent=false;
					//					}
					assignPath(parent, node, path);
					mappedParentMutexPaths[parent] = mutexPath;
					addedRoutingParentPorts += path.size();
					//					if(routedParent){
					addedRoutingParentPorts -= 1;
					//					}
					//					for(Port* p : path){
					//						std::cout << p->getFullName() << ",\n";
					//					}
					//					std::cout << "\n";
					break;
				}
				else
				{
					addedRoutingParentPorts = 0;
					node->clear(this->dfg);
					std::cout << "Route Failed :: from=" << src.second->getFullName() << "--> to=" << dest.second->getFullName() << "\n";
				}
				path.clear();
			}
			if (!succ)
			{
				*failedNode = parent;
				node->clear(this->dfg);
				addedRoutingParentPorts = 0;
				parentRoutSucc = false; // at least one parent failed to route, try a new dest
				break;
			}
		}

		if (parentRoutSucc)
		{ //all parents routed succesfull + all mapped childs are connected
			routeSucc = true;
			std::cout << "node=" << node->idx << ",op=" << node->op << " is mapped to " << currDest.dest->getPE()->getName() << ",lat=" << currDest.destLat << "\n";
			std::cout << "routing info ::\n";
			for (DFGNode *parent : node->parents)
			{
				std::cout << "parent routing port size = " << parent->routingPorts.size() << "\n";
				int prev_lat = -1;
				for (std::pair<Port *, int> pair : parent->routingPorts)
				{
					Port *p = pair.first;
					//					if(node.routingPortDestMap[p]==&node){
					std::cout << "fr:" << parent->idx << " :: ";
					std::cout << ",dest=" << pair.second << " :: ";
					std::cout << p->getFullName();
					std::cout << ",lat=" << p->getLat();

					if (mappedParentMutexPaths[parent].find(p) != mappedParentMutexPaths[parent].end())
					{
						std::cout << "|mutex(";
						for (DFGNode *mutexnode : mappedParentMutexPaths[parent][p])
						{
							std::cout << mutexnode->idx << ",";
						}
						std::cout << ")";
					}
					std::cout << std::endl;
					//					}
					if (prev_lat != -1)
					{
						//							assert(p->getLat() - prev_lat <= 1);
					}
					prev_lat = p->getLat();
				}
			}
			std::cout << "routing info done.\n";
			currDest.dest->assignNode(node, currDest.destLat, this->dfg);
			node->rootDP = currDest.dest;
			break;
		}
		node->clear(this->dfg);
	}

	if (routeSucc)
	{
		std::cout << "Route success...\n";

		int parentRoutingPortCountEnd = 0;
		//		int mappedParentCount=0;
		for (DFGNode *parent : node->parents)
		{
			if (parent->rootDP != NULL)
			{
				//				mappedParentCount++;
				parentRoutingPortCountEnd += parent->routingPorts.size();
			}
		}
		parentRoutingPortCountEnd = std::max(0, parentRoutingPortCountEnd - routedParents);
		if (parentRoutingPortCountEnd != parentRoutingPortCount + addedRoutingParentPorts)
		{
			std::cout << "parentRoutingPortCountEnd=" << parentRoutingPortCountEnd << "\n";
			std::cout << "addedRoutingParentPorts=" << addedRoutingParentPorts << "\n";
			std::cout << "parentRoutingPortCount=" << parentRoutingPortCount << "\n";
		}

		//		assert(parentRoutingPortCountEnd==parentRoutingPortCount+addedRoutingParentPorts);
		return true;
	}
	else
	{
		currDest.dest->assignNode(node, currDest.destLat, this->dfg);
		node->rootDP = currDest.dest;
		node->clear(this->dfg);
		std::cout << "Route failed...\n";

		int parentRoutingPortCountEnd = 0;
		//		int mappedParentCount=0;
		for (DFGNode *parent : node->parents)
		{
			if (parent->rootDP != NULL)
			{
				//				mappedParentCount++;
				parentRoutingPortCountEnd += parent->routingPorts.size();
			}
		}
		parentRoutingPortCountEnd = std::max(0, parentRoutingPortCountEnd - routedParents);
		if (parentRoutingPortCountEnd != parentRoutingPortCount + addedRoutingParentPorts)
		{
			std::cout << "parentRoutingPortCountEnd=" << parentRoutingPortCountEnd << "\n";
			std::cout << "addedRoutingParentPorts=" << addedRoutingParentPorts << "\n";
			std::cout << "parentRoutingPortCount=" << parentRoutingPortCount << "\n";
		}
		//		assert(parentRoutingPortCountEnd==parentRoutingPortCount);
		assert(*failedNode != NULL);
		return false;
	}
}

int CGRAXMLCompile::PathFinderMapper::calculateCost(LatPort src,
													LatPort next_to_src, LatPort dest)
{

	std::string srcName = src.second->getName();
	//	std::cout << src->getName() << ",";

	std::string next_to_srcName = next_to_src.second->getName();
	//	std::cout << next_to_srcName << "\n";

	assert(src.second);
	assert(next_to_src.second);
	assert(dest.second);

	PE *srcPE = src.second->findParentPE();
	assert(srcPE);
	PE *nextPE = next_to_src.second->findParentPE();
	assert(nextPE);

	// int distance = abs(nextPE->Y - srcPE->Y) + abs(nextPE->X - srcPE->X) + regDiscourageFactor * ((nextPE->T - srcPE->T + cgra->get_t_max()) % cgra->get_t_max());
	int distance = regDiscourageFactor * ((nextPE->T - srcPE->T + cgra->get_t_max()) % cgra->get_t_max());

	distance = distance * PETransitionCostFactor + next_to_src.second->getCongCost() + PortTransitionCost;
	assert(distance > 0);

	if (srcPE != nextPE)
	{
		int freePorts = 0;

		for (Port *p : nextPE->outputPorts)
		{
			Module *parent = nextPE->getParent();
			if (parent->getNextPorts(std::make_pair(next_to_src.first, p), this).empty())
				continue;
			if (p->getNode() == NULL)
			{
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
		distance = distance + (nextPE->outputPorts.size() * 2 - (freePorts)) * UOPCostFactor;

		if (nextPE->outputPorts.size() * 2 < freePorts)
		{
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
	assert(distance > 0);

	if ((next_to_src.second->getName().compare("P") == 0) || (next_to_src.second->getName().compare("I1") == 0) || (next_to_src.second->getName().compare("I2") == 0))
	{

		FU *fu = next_to_src.second->getMod()->getFU();
		if ((fu->supportedOPs.find("LOAD") != fu->supportedOPs.end()) && (dest == next_to_src))
		{
			double memrescost_dbl = (double)this->dfg->unmappedMemOps / (double)cgra->freeMemNodes;
			memrescost_dbl = memrescost_dbl * (double)MEMResourceCost;
			distance = distance + (int)memrescost_dbl;
			if (this->dfg->unmappedMemOps == cgra->freeMemNodes)
			{
				distance = distance + MRC * 10;
			}
		}
	}

	assert(distance > 0);
	return distance;
}

bool CGRAXMLCompile::PathFinderMapper::Map(CGRA *cgra, DFG *dfg)
{
	std::stack<DFGNode *> mappedNodes;
	std::stack<DFGNode *> unmappedNodes;
	std::map<DFGNode *, std::priority_queue<dest_with_cost>> estimatedRouteInfo;

	int backTrackCredits = this->backTrackLimit;

	//Disable mutex paths to test pathfinder
	this->enableMutexPaths = true;

	this->cgra = cgra;
	this->dfg = dfg;

	Check_DFG_CGRA_Compatibility();
	UpdateVariableBaseAddr();

	//Testing 1 2 3
	//getLongestDFGPath(dfg->findNode(1093),dfg->findNode(82));

	//	SortSCCDFG();
	//	SortTopoGraphicalDFG();
	sortBackEdgePriorityASAP();
	//	sortBackEdgePriorityALAP();

	std::string mappingLogFileName = fNameLog1 + cgra->getCGRAName() + "_MTP=" + std::to_string(enableMutexPaths);  // + ".mapping.csv";
	std::string mappingLog2FileName = fNameLog1 + cgra->getCGRAName() + "_MTP=" + std::to_string(enableMutexPaths); // + ".routeInfo.log";

	bool mapSuccess = false;

	std::string congestionInfoFileName = mappingLogFileName + ".congestion.info";
	cout << "Opening congestion file : " << congestionInfoFileName << "!\n";
	congestionInfoFile.open(congestionInfoFileName.c_str());
	assert(congestionInfoFile.is_open());

	for (int i = 0; i < this->maxIter; ++i)
	{

		std::string mappingLogFileName_withIter = mappingLogFileName + "_Iter=" + std::to_string(i) + ".mapping.csv";
		std::string mappingLog2FileName_withIter = mappingLog2FileName + "_Iter=" + std::to_string(i) + ".routeInfo.log";

		mappingLog.open(mappingLogFileName_withIter.c_str());
		mappingLog2.open(mappingLog2FileName_withIter.c_str());

		cout << "Opening mapping csv file : " << mappingLogFileName_withIter << "\n";
		cout << "Opening routeInfo log file : " << mappingLog2FileName_withIter << "\n";

		assert(mappingLog.is_open());
		assert(mappingLog2.is_open());

		while (!mappedNodes.empty())
		{
			mappedNodes.pop();
		}
		while (!unmappedNodes.empty())
		{
			unmappedNodes.pop();
		}

		for (DFGNode *node : sortedNodeList)
		{
			unmappedNodes.push(node);
		}

		std::cout << "MAP begin...\n";

		while (!unmappedNodes.empty())
		{

			DFGNode *node = unmappedNodes.top();
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

			// MapHeader << ",PEType = " << this->cgra->peType;
			// MapHeader << ",XDim = " << this->cgra->get_x_max();
			// MapHeader << ",YDim = " << this->cgra->get_y_max();
			// MapHeader << ",DPs = " << this->cgra->numberofDPs;

			MapHeader << ",CGRA=" << this->cgra->getCGRAName();

			MapHeader << ",BB = " << node->BB;
			MapHeader << ",mutexPathEn = " << this->enableMutexPaths;
			MapHeader << ",Iter = " << i;
			MapHeader << "\n";

			std::cout << MapHeader.str();
			mappingLog << MapHeader.str();

			bool isEstRouteSucc = false;

			//fill the routing information
			if (estimatedRouteInfo.find(node) == estimatedRouteInfo.end())
			{
				//the routes are not estimated.
				std::priority_queue<dest_with_cost> estimatedRoutes;
				DFGNode *failedNode;
				isEstRouteSucc = estimateRouting(node, estimatedRoutes, &failedNode);

				if (!isEstRouteSucc)
				{
					printMappingLog();
					printMappingLog2();
					if (enableBackTracking)
					{
						if (backTrackCredits == 0 || failedNode == NULL)
						{
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

						DFGNode *prevNode = mappedNodes.top();
						mappedNodes.pop();
						unmappedNodes.push(node);
						unmappedNodes.push(prevNode);

						prevNode->clear(this->dfg);
						estimatedRouteInfo.erase(node);

						//										assert(failedNode!=NULL);
						//										unmappedNodes.push(node);
						//										removeFailedNode(mappedNodes,unmappedNodes,failedNode);
						//										failedNode->blacklistDest.insert(failedNode->rootDP);
						//										(failedNode)->clear(this->dfg);
						//										estimatedRouteInfo.erase(node);
						//										estimatedRouteInfo.erase(failedNode);

						continue;
					}
					else
					{
						while (!mappedNodes.empty())
						{
							DFGNode *prevNode = mappedNodes.top();
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
				estimatedRouteInfo[node] = estimatedRoutes;
			}

			bool isRouteSucc = false;
			DFGNode *failedNode = NULL;

			std::cout << "estimatedRouteInfo[node].size = " << estimatedRouteInfo[node].size() << "\n";
			mappingLog << "estimatedRouteInfo[node].size = " << estimatedRouteInfo[node].size() << "\n";
			if (!estimatedRouteInfo[node].empty())
			{
				isRouteSucc = Route(node, estimatedRouteInfo[node], &failedNode);
				if (!isRouteSucc)
					std::cout << "BLAAAAAAAAAAA!\n";
			}
			else
			{
				if (mappedNodes.empty())
				{
					mappingLog << "Map Failed!.\n";
					std::cout << "Map Failed!.\n";
					mappingLog.close();
					mappingLog2.close();
					return false;
				}
			}

			if (!isRouteSucc)
			{
				this->printMappingLog();
				this->printMappingLog2();
				if (mappedNodes.empty())
				{
					mappingLog << "Map Failed!.\n";
					std::cout << "Map Failed!.\n";
					mappingLog.close();
					mappingLog2.close();
					return false;
				}

				if (enableBackTracking)
				{
					if (backTrackCredits == 0)
					{
						mappingLog << "Map Failed!.\n";
						std::cout << "Map Failed!.\n";
						mappingLog.close();
						mappingLog2.close();
						return false;
					}
					//					assert(failedNode!=NULL);
					backTrackCredits--;

					DFGNode *prevNode = mappedNodes.top();
					mappedNodes.pop();
					unmappedNodes.push(node);
					unmappedNodes.push(prevNode);

					prevNode->clear(this->dfg);
					estimatedRouteInfo.erase(node);

					//					unmappedNodes.push(node);
					//					removeFailedNode(mappedNodes,unmappedNodes,failedNode);
					//					failedNode->blacklistDest.insert(failedNode->rootDP);
					//					(failedNode)->clear(this->dfg);
					//					estimatedRouteInfo.erase(node);
					//					estimatedRouteInfo.erase(failedNode);
					continue;
				}
				else
				{
					while (!mappedNodes.empty())
					{
						DFGNode *prevNode = mappedNodes.top();
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
			backTrackCredits = std::min(this->backTrackLimit, backTrackCredits + 1);
			mappedNodes.push(node);
		}
		mapSuccess = updateCongestionCosts(i);
		if (mapSuccess)
		{
			break;
		}
		clearCurrMapping();
		estimatedRouteInfo.clear();
		mappingLog.close();
		mappingLog2.close();
	}

	//	congestionInfoFile.close();

	if (mapSuccess)
	{
		mappingLog << "Map Success!.\n";
		mappingLog2 << "Map Success!.\n";
		this->printMappingLog();
		this->printMappingLog2();

		cgra->PrintMappedJSON(fNameLog1 + cgra->getCGRAName() + "mapping.json");

		std::cout << "Map Success!.\n";
		mappingLog.close();
		mappingLog2.close();

		std::cout << "Checking conflict compatibility!\n";
		checkConflictedPortCompatibility();

		if (this->cgra->peType == "STDNOC_4REGF_1P")
		{
			checkRegALUConflicts();
		}
		return true;
	}
	else
	{
		while (!mappedNodes.empty())
		{
			DFGNode *prevNode = mappedNodes.top();
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

void CGRAXMLCompile::PathFinderMapper::assignPath(DFGNode *src, DFGNode *dest,
												  std::vector<LatPort> path)
{

	std::cout << "assigning path from:" << src->idx << " to:" << dest->idx << "\n";

	int srcPortCount = 0;

	int prevLat = -1;
	LatPort prevPort;
	for (LatPort p : path)
	{

		if (prevLat != -1)
		{
			if (p.first - prevLat > 1)
			{
				std::cout << prevPort.second->getFullName() << ",Lat = " << prevPort.first << "\n";
				std::cout << p.second->getFullName() << ",Lat = " << p.first << "\n";
			}
			assert(p.first - prevLat <= 1);
		}

		//		if(p->getName().compare("T")==0){
		//			assert(p->getNode()==src);
		//		}

		prevLat = p.first;
		prevPort = p;

		if (p.second->getNode() == src)
		{
			srcPortCount++;
			continue;
		}

		p.second->setNode(src, p.first, this);
		congestedPorts[p.second].insert(src);
		p.second->increaseConflictedUse(src, this);

		if (std::find(src->routingPorts.begin(), src->routingPorts.end(), std::make_pair(p.second, dest->idx)) == src->routingPorts.end())
		{
			if (std::find(src->routingPorts.begin(), src->routingPorts.end(), std::make_pair(p.second, src->idx)) == src->routingPorts.end())
			{
				src->routingPorts.push_back(std::make_pair(p.second, dest->idx));
			}
			else
			{
				std::cout << p.second->getFullName() << "\n";
				assert(p.second->getName().compare("T") == 0);
			}
		}
		//		src->routingPortDestMap[p]=dest->idx;
	}
	std::cout << "srcPortCount = " << srcPortCount << "\n";
}

bool CGRAXMLCompile::PathFinderMapper::updateCongestionCosts(int iter)
{
	bool noCongestion = true;

	std::set<int> conflictedTimeSteps;

	congestionInfoFile << "**********************************\n";
	congestionInfoFile << "II = " << this->cgra->get_t_max() << ",iter = " << iter << "\n";
	congestionInfoFile << "**********************************\n";

	for (std::pair<Port *, std::set<DFGNode *>> pair : congestedPorts)
	{
		Port *p = pair.first;
		if (pair.second.size() > 1)
		{
			for (DFGNode *node1 : pair.second)
			{
				for (DFGNode *node2 : pair.second)
				{
					if (node1 == node2)
					{
						continue;
					}
					if (this->dfg->isMutexNodes(node1, node2, p))
						continue;
					std::cout << "CONGESTION:" << p->getFullName();
					congestionInfoFile << "CONGESTION:" << p->getFullName();
					for (DFGNode *node : pair.second)
					{
						std::cout << "," << node->idx << "|BB=" << node->BB;
						congestionInfoFile << "," << node->idx << "|BB=" << node->BB;
					}
					std::cout << "\n";
					congestionInfoFile << "\n";
					p->increastCongCost();
					noCongestion = false;
					conflictedTimeSteps.insert(p->getMod()->getPE()->T);
					//					break;
				}
				if (!noCongestion)
				{
					//					break;
				}
			}
		}
		if (p->getHistoryCost() > 0)
		{
			std::cout << "HISTORY_COST :: " << p->getFullName() << "," << p->getHistoryCost() << "\n";
			congestionInfoFile << "HISTORY_COST :: " << p->getFullName() << "," << p->getHistoryCost() << "\n";
		}
	}

	bool noConflicts = true;
	for (std::pair<Port *, std::set<DFGNode *>> pair : conflictedPorts)
	{
		Port *p = pair.first;

		if (p->getNode() != NULL)
		{
			for (DFGNode *node : pair.second)
			{
				//				if(node == p->getNode()){
				//					bool isRDP = p->getName().find("RDP") != std::string::npos;
				//					bool isINT = p->getName().find("INT") != std::string::npos;
				//					bool isT = p->getName().find("_T") != std::string::npos;
				//					if((isRDP&isINT) || isT) continue;
				//				}
				noConflicts = false;
			}

			if (noConflicts)
				continue;

			std::cout << "CONFLICT :" << p->getFullName();
			congestionInfoFile << "CONFLICT :" << p->getFullName();
			for (DFGNode *node : pair.second)
			{
				std::cout << "," << node->idx << "|BB=" << node->BB;
				congestionInfoFile << "," << node->idx << "|BB=" << node->BB;
			}
			std::cout << ", with MAPPED = " << p->getNode()->idx << "|BB=" << p->getNode()->BB;
			std::cout << "\n";

			congestionInfoFile << ", with MAPPED = " << p->getNode()->idx << "|BB=" << p->getNode()->BB;
			congestionInfoFile << "\n";

			for (int i = 0; i < pair.second.size(); ++i)
			{
				p->increastCongCost();
			}
			conflictedTimeSteps.insert(p->getMod()->getPE()->T);
		}

		if (p->getHistoryCost() > 0)
		{
			std::cout << "HISTORY_COST :: " << p->getFullName() << "," << p->getHistoryCost() << "\n";
			congestionInfoFile << "HISTORY_COST :: " << p->getFullName() << "," << p->getHistoryCost() << "\n";
		}
	}

	if (this->upperboundII > conflictedTimeSteps.size() + this->cgra->get_t_max())
	{
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
	if (noCongestion)
		std::cout << "noCongestion!\n";
	if (noConflicts)
		std::cout << "noConflicts!\n";

	if (noCongestion)
		congestionInfoFile << "noCongestion!\n";
	if (noConflicts)
		congestionInfoFile << "noConflicts!\n";

	return noCongestion & noConflicts;
}

bool CGRAXMLCompile::PathFinderMapper::clearCurrMapping()
{
	for (DFGNode *node : sortedNodeList)
	{
		std::cout << "CLEARING :: node=" << node->idx << ",destDP=" << node->rootDP->getName() << ",destPE=" << node->rootDP->getPE()->getName() << "\n";
		node->clear(this->dfg);
	}

	std::stack<Module *> searchStack;
	searchStack.push(this->cgra);

	while (!searchStack.empty())
	{
		Module *top = searchStack.top();
		searchStack.pop();
		for (Port *p : top->inputPorts)
		{
			assert(p->getNode() == NULL);
		}
		for (Port *p : top->internalPorts)
		{
			assert(p->getNode() == NULL);
		}
		for (Port *p : top->outputPorts)
		{
			assert(p->getNode() == NULL);
		}
		for (Module *submod : top->subModules)
		{
			searchStack.push(submod);
		}
	}
	return true;
}

bool CGRAXMLCompile::PathFinderMapper::checkConflictedPortCompatibility()
{

	std::stack<Module *> searchStack;
	searchStack.push(this->cgra);

	while (!searchStack.empty())
	{
		Module *top = searchStack.top();
		searchStack.pop();
		for (Port *p : top->inputPorts)
		{
			if (p->getNode() != NULL)
			{
				for (Port *cp : top->getConflictPorts(p))
				{
					std::cout << "p : " << p->getFullName() << ", cp : " << cp->getFullName() << "\n";
					if (cp->getNode() != NULL)
					{
						std::cout << "Conflict ERR!\n";
						std::cout << p->getFullName() << ":" << p->getNode()->idx << "," << cp->getFullName() << ":" << cp->getNode()->idx << "\n";
					}
					assert(cp->getNode() == NULL);
				}
			}
		}
		for (Port *p : top->internalPorts)
		{
			if (p->getNode() != NULL)
			{
				for (Port *cp : top->getConflictPorts(p))
				{
					//					if(cp->)
					std::cout << "p : " << p->getFullName() << ", cp : " << cp->getFullName() << "\n";
					if (cp->getNode() != NULL)
					{
						std::cout << "Conflict ERR!\n";
						std::cout << p->getFullName() << ":" << p->getNode()->idx << "," << cp->getFullName() << ":" << cp->getNode()->idx << "\n";
					}
					assert(cp->getNode() == NULL);
				}
			}
		}
		for (Port *p : top->outputPorts)
		{
			if (p->getNode() != NULL)
			{
				for (Port *cp : top->getConflictPorts(p))
				{
					std::cout << "p : " << p->getFullName() << ", cp : " << cp->getFullName() << "\n";
					if (cp->getNode() != NULL)
					{
						std::cout << "Conflict ERR!\n";
						std::cout << p->getFullName() << ":" << p->getNode()->idx << "," << cp->getFullName() << ":" << cp->getNode()->idx << "\n";
					}
					assert(cp->getNode() == NULL);
				}
			}
		}
		for (Module *submod : top->subModules)
		{
			searchStack.push(submod);
		}
	}
}

bool CGRAXMLCompile::PathFinderMapper::checkRegALUConflicts()
{
	for (int t = 0; t < this->cgra->get_t_max(); ++t)
	{
		int timeslice_count = 0;
		vector<PE *> PEList = this->cgra->getSpatialPEList(t);
		// for (int y = 0; y < this->cgra->get_y_max(); ++y)
		// {
		// 	for (int x = 0; x < this->cgra->get_x_max(); ++x)
		// 	{
		for (PE *currPE : PEList)
		{

			// PE *currPE = this->cgra->getPE(t, y, x);
			int usage = 0;

			for (Module *submod_fu : currPE->subModules)
			{
				if (FU *fu = dynamic_cast<FU *>(submod_fu))
				{
					for (Module *submod_dp : fu->subModules)
					{
						if (DataPath *dp = dynamic_cast<DataPath *>(submod_dp))
						{
							if (dp->getMappedNode() != NULL)
							{
								std::cout << dp->getFullName() << ":" << dp->getMappedNode()->idx << ",";
								usage++;
								break;
							}
						}
					}
				}
			}

			for (RegFile *RF : currPE->allRegs)
			{
				for (int i = 0; i < RF->get_nWRPs(); ++i)
				{
					std::string wrpName = "WRP" + std::to_string(i);
					Port *wrp = RF->getInPort(wrpName);
					if (wrp->getNode() != NULL)
					{
						std::cout << wrp->getFullName() << ":" << wrp->getNode()->idx << ",";
						usage++;
					}
				}

				for (int i = 0; i < RF->get_nRDPs(); ++i)
				{
					std::string rdpName = "RDP" + std::to_string(i);
					Port *rdp = RF->getOutPort(rdpName);
					if (rdp->getNode() != NULL)
					{
						std::cout << rdp->getFullName() << ":" << rdp->getNode()->idx << ",";
						usage++;
					}
				}
			}

			if (timeslice_count <= usage - 1)
			{
				timeslice_count = usage - 1;
			}

			std::cout << "\n";
		}
		std::cout << "t=" << t << ","
			  << "timeslice=" << timeslice_count << "\n";
	}
}

bool CGRAXMLCompile::PathFinderMapper::checkDPFree(DataPath *dp, DFGNode *node, int &penalty)
{
	PE *currPE = dp->getPE();
	FU *currFU = dp->getFU();

	int numberFUs = 0;
	int numberUsedFUs = 0;
	int numberConstants = 0;
	bool memfu_found = false;
	bool memop_found = false;
	for (Module *submod_fu : currPE->subModules)
	{
		if (FU *fu = dynamic_cast<FU *>(submod_fu))
		{
			int dp_used = 0;
			if (!memfu_found)
				memfu_found = fu->isMEMFU();
			for (Module *submod_dp : fu->subModules)
			{
				if (DataPath *dp = dynamic_cast<DataPath *>(submod_dp))
				{
					if (dp->getMappedNode() != NULL)
					{
						dp_used = 1;
						if (!memop_found)
							memop_found = dp->getMappedNode()->isMemOp();
						if (dp->getMappedNode()->hasConst)
						{
							numberConstants++;
						}
					}
				}
			}
			numberUsedFUs += dp_used;
			numberFUs += 1;
		}
	}

	//increment for the current node
	numberUsedFUs++;
	if (node->hasConst)
	{
		numberConstants++;
	}

	assert(this->dfg->unmappedMemOps == this->dfg->unmappedMemOpSet.size());
	assert(this->cgra->freeMemNodes == this->cgra->freeMemNodeSet.size());

	penalty = 0;
	if (memfu_found)
	{
		int memnode_const_count = 0;
		for (DFGNode *memnode : this->dfg->unmappedMemOpSet)
		{
			if (memnode->hasConst)
			{
				memnode_const_count++;
			}
		}

		int freeMemPEs_const = 0;
		for (DataPath *memdp : this->cgra->freeMemNodeSet)
		{
			int memPEConstants = 0;
			int memUsedFUs = 0;
			PE *memPE = memdp->getPE();
			for (Module *submod_fu : memPE->subModules)
			{
				if (FU *fu = dynamic_cast<FU *>(submod_fu))
				{
					int dp_used = 0;
					for (Module *submod_dp : fu->subModules)
					{
						if (DataPath *dp = dynamic_cast<DataPath *>(submod_dp))
						{
							if (dp->getMappedNode() != NULL)
							{
								dp_used = 1;
								if (!memop_found)
									memop_found = dp->getMappedNode()->isMemOp();
								if (dp->getMappedNode()->hasConst)
								{
									memPEConstants++;
								}
							}
						}
					}
					memUsedFUs += dp_used;
				}
			}
			if (memUsedFUs + memPEConstants <= 1)
			{
				freeMemPEs_const++;
			}
		}

		if ((!node->isMemOp()) && (!memop_found))
		{
			double penalty_ratio_dbl = (double)memnode_const_count / (double)freeMemPEs_const;
			double penalty_dbl = penalty_ratio_dbl * (double)MRC;
			penalty = (int)penalty_dbl;
		}
	}

	//with current node it should be less than or equal to number of FUs
	if (numberConstants + numberUsedFUs <= numberFUs || numberFUs == 1)
	{
		if (dp->getMappedNode() == NULL)
		{
			return true;
		}
	}
	return false;
}

bool CGRAXMLCompile::PathFinderMapper::updateConflictedTimeSteps(int timeStep,
																 int conflicts)
{

	int presentConflicts = conflictedTimeStepMap[timeStep];
	if (conflicts > presentConflicts)
	{
		conflictedTimeStepMap[timeStep] = conflicts;
		return true;
	}
	return false;
}

int CGRAXMLCompile::PathFinderMapper::getTimeStepConflicts(int timeStep)
{
	return conflictedTimeStepMap[timeStep];
}

void CGRAXMLCompile::PathFinderMapper::sortBackEdgePriorityASAP()
{
	sortedNodeList.clear();

	struct BEDist
	{
		DFGNode *parent;
		DFGNode *child;
		int dist;
		BEDist(DFGNode *parent, DFGNode *child, int dist) : parent(parent), child(child), dist(dist) {}
		bool operator<(const BEDist &other) const
		{
			if (dist == other.dist)
			{
				return true;
			}
			return dist > other.dist;
		}
		//		bool operator==(const BEDist& other) const{
		//			return parent==other.parent & child==other.child;
		//		}
	};

	std::set<BEDist> backedges;

	for (DFGNode &node : dfg->nodeList)
	{

		if (node.idx == 97)
		{
			std::cout << "node_idx:97,node_ASAP:" << node.ASAP << "\n";
		}
		for (DFGNode *child : node.children)
		{

			if (node.idx == 97)
			{
				std::cout << "child_idx:" << child->idx << "child_ASAP:" << child->ASAP << "\n";
			}

			if (child->ASAP <= node.ASAP)
			{
				std::cout << "inserting for : node=" << node.idx << ",child:" << child->idx << "\n";
				backedges.insert(BEDist(&node, child, node.ASAP - child->ASAP));
			}
		}
	}

	//populate reccycles
	std::cout << "Populate Rec Cycles!\n";
	RecCycles.clear();
	for (BEDist be : backedges)
	{
		//		std::set<DFGNode*> backedgePath;
		std::vector<DFGNode *> backedgePathVec = dfg->getAncestoryASAP(be.parent);

		std::cout << "REC_CYCLE :: BE_Parent = " << be.parent->idx << "\n";
		std::cout << "REC_CYCLE :: BE_Child = " << be.child->idx << "\n";
		std::cout << "REC_CYCLE :: BE_Parent's ancesotry : \n";
		for (DFGNode *n : backedgePathVec)
		{
			if (RecCycles[BackEdge(be.parent, be.child)].find(n) == RecCycles[BackEdge(be.parent, be.child)].end())
			{
				std::cout << n->idx << ",";
			}
			RecCycles[BackEdge(be.parent, be.child)].insert(n);
		}
		std::cout << "REC_CYCLE :: Done!\n";
		//= dfg->getAncestoryASAP(be.parent);
		//		if(dfg->getAncestoryASAPUntil(be.parent,be.child,backedgePath)){
		//			backedgePath.insert(be.parent);
		//			RecCycles[BackEdge(be.parent,be.child)]=backedgePath;
		//		}
	}

	RecCyclesLS.clear();
	for (DFGNode &node : dfg->nodeList)
	{
		for (DFGNode *recParent : node.recParents)
		{
			BEDist be_temp(&node, recParent, node.ASAP - recParent->ASAP);

			std::vector<DFGNode *> backedgePathVec = dfg->getAncestoryASAP(be_temp.parent);

			std::cout << "REC_CYCLELS :: BE_Parent = " << be_temp.parent->idx << "\n";
			std::cout << "REC_CYCLELS :: BE_Child = " << be_temp.child->idx << "\n";
			std::cout << "REC_CYCLELS :: BE_Parent's ancesotry : \n";
			for (DFGNode *n : backedgePathVec)
			{
				if (n == be_temp.parent)
					continue;
				if (RecCyclesLS[BackEdge(be_temp.parent, be_temp.child)].find(n) == RecCyclesLS[BackEdge(be_temp.parent, be_temp.child)].end())
				{
					std::cout << n->idx << ",";
				}
				RecCyclesLS[BackEdge(be_temp.parent, be_temp.child)].insert(n);
			}
			std::cout << "REC_CYCLELS :: Done!\n";

			backedges.insert(be_temp);
		}
	}

	std::map<DFGNode *, std::vector<DFGNode *>> beparentAncestors;
	std::map<DFGNode *, std::vector<DFGNode *>> bechildAncestors;
	//	std::map<DFGNode*,std::vector<DFGNode*>> bechildAncestors;
	std::map<std::pair<DFGNode *, DFGNode *>, bool> trueBackedges;

	for (BEDist be : backedges)
	{
		std::cout << "BE PARENT = " << be.parent->idx << ",dist=" << be.dist << "\n";
		std::cout << "BE CHILD = " << be.child->idx << "\n";

		std::cout << "Ancestory : "
				  << "\n";
		beparentAncestors[be.parent] = dfg->getAncestoryASAP(be.parent);
		bechildAncestors[be.child] = dfg->getAncestoryASAP(be.child);
		std::cout << "\n";

		if (std::find(beparentAncestors[be.parent].begin(),
					  beparentAncestors[be.parent].end(),
					  be.child) == beparentAncestors[be.parent].end())
		{
			std::cout << "BE CHILD does not belong BE Parent's Ancestory\n";

			//Hack to force all backedges to be true backedges
			trueBackedges[std::make_pair(be.parent, be.child)] = false;
		}
		else
		{
			//change this be.parent if PHI nodes are not removed
			std::cout << "RecPHI inserted : " << be.child->idx << "\n";
			trueBackedges[std::make_pair(be.parent, be.child)] = false;
			//			RecPHIs.insert(be.child);
		}

		//		bechildAncestors[be.child]=dfg->getAncestory(be.child);
	}

	//	{ //true backedges children are placed high priority :MERGED
	//		std::vector<DFGNode*> mergedAncestoriesChild;
	//		std::map<DFGNode*,DFGNode*> mergedKeysChild;
	//
	//		for(BEDist be : backedges){
	//			if(trueBackedges[std::make_pair(be.parent,be.child)] == true){
	//				mergedAncestoriesChild = dfg->mergeAncestoryASAP(mergedAncestoriesChild,bechildAncestors[be.child],RecCycles);
	//			}
	//		}
	//
	//		for(DFGNode* ancestorNode : mergedAncestoriesChild){
	//			if(std::find(sortedNodeList.begin(),sortedNodeList.end(),ancestorNode) == sortedNodeList.end()){
	//				sortedNodeList.push_back(ancestorNode);
	//			}
	//		}
	//
	//	}

	std::map<DFGNode *, std::set<DFGNode *>> superiorChildren;

	//	std::vector<DFGNode*> mergedAncestory;
	std::map<DFGNode *, std::vector<DFGNode *>> mergedAncestories;
	mergedAncestories.clear();
	std::map<DFGNode *, DFGNode *> mergedKeys;
	for (BEDist be : backedges)
	{
		//		write a logic to merge ancestories where if one be's child is present in some other be's parent's ancesotory'
		bool merged = false;

		//		if(trueBackedges[std::make_pair(be.parent,be.child)] == true){
		//			// if true backede place the child first so that the parent's path will
		//			// be adjusted accordingly
		////			for(DFGNode* ancestorNode : bechildAncestors[be.child]){
		////				if(std::find(sortedNodeList.begin(),sortedNodeList.end(),ancestorNode) == sortedNodeList.end()){
		////					sortedNodeList.push_back(ancestorNode);
		////				}
		////			}
		//			superiorChildren[be.parent].insert(be.child);
		//		}

		for (std::pair<DFGNode *, std::vector<DFGNode *>> pair : mergedAncestories)
		{
			DFGNode *key = pair.first;
			//			if(trueBackedges[std::make_pair(be.parent,be.child)] == false) continue;
			if (std::find(mergedAncestories[key].begin(), mergedAncestories[key].end(), be.child) != mergedAncestories[key].end())
			{

				if (trueBackedges[std::make_pair(be.parent, be.child)] == true)
				{
					superiorChildren[key].insert(be.child);
				}

				std::cout << "Merging :: " << key->idx << ", " << be.parent->idx << "\n";
				mergedAncestories[key] = dfg->mergeAncestoryASAP(mergedAncestories[key], beparentAncestors[be.parent], RecCycles);
				merged = true;
				std::cout << "Merging Done :: " << key->idx << ", " << be.parent->idx << "\n";
				mergedKeys[be.parent] = key;
				//				break;
			}
		}
		if (!merged)
		{
			mergedAncestories[be.parent] = dfg->getAncestoryASAP(be.parent);
			mergedKeys[be.parent] = be.parent;

			if (trueBackedges[std::make_pair(be.parent, be.child)] == true)
			{
				superiorChildren[be.parent].insert(be.child);
			}
		}
	}

	for (BEDist be : backedges)
	{
		std::vector<DFGNode *> mergedSuperiorChildren;
		for (DFGNode *sChild : superiorChildren[mergedKeys[be.parent]])
		{
			mergedSuperiorChildren = dfg->mergeAncestoryASAP(mergedSuperiorChildren, bechildAncestors[sChild], RecCycles);
		}

		//		for(DFGNode* sChild : superiorChildren[mergedKeys[be.parent]]){
		//			for(DFGNode* ancestorNode : bechildAncestors[sChild]){
		for (DFGNode *ancestorNode : mergedSuperiorChildren)
		{
			if (std::find(sortedNodeList.begin(), sortedNodeList.end(), ancestorNode) == sortedNodeList.end())
			{
				sortedNodeList.push_back(ancestorNode);
			}
		}
		//		}

		for (DFGNode *ancestorNode : mergedAncestories[mergedKeys[be.parent]])
		{
			if (std::find(sortedNodeList.begin(), sortedNodeList.end(), ancestorNode) == sortedNodeList.end())
			{
				sortedNodeList.push_back(ancestorNode);
			}
		}
	}

	for (BEDist be : backedges)
	{
		assert(mergedKeys.find(be.parent) != mergedKeys.end());
		std::vector<DFGNode *> ancestoryNodes = mergedAncestories[mergedKeys[be.parent]];
		for (DFGNode *ancestorNode : ancestoryNodes)
		{
			if (std::find(sortedNodeList.begin(), sortedNodeList.end(), ancestorNode) == sortedNodeList.end())
			{
				sortedNodeList.push_back(ancestorNode);
			}
		}
		std::cout << "BE PARENT = " << be.parent->idx << ",dist=" << be.dist << "\n";
		if (std::find(sortedNodeList.begin(), sortedNodeList.end(), be.parent) == sortedNodeList.end())
		{
			sortedNodeList.push_back(be.parent);
		}

		ancestoryNodes = dfg->getAncestoryASAP(be.child);
		for (DFGNode *ancestorNode : ancestoryNodes)
		{
			if (std::find(sortedNodeList.begin(), sortedNodeList.end(), ancestorNode) == sortedNodeList.end())
			{
				sortedNodeList.push_back(ancestorNode);
			}
		}
		std::cout << "BE CHILD = " << be.child->idx << "\n";
		if (std::find(sortedNodeList.begin(), sortedNodeList.end(), be.child) == sortedNodeList.end())
		{
			sortedNodeList.push_back(be.child);
		}
	}

	std::map<int, std::vector<DFGNode *>> asapLevelNodeList;
	for (DFGNode &node : dfg->nodeList)
	{
		asapLevelNodeList[node.ASAP].push_back(&node);
	}

	int maxASAPlevel = 0;
	for (std::pair<int, std::vector<DFGNode *>> pair : asapLevelNodeList)
	{
		if (pair.first > maxASAPlevel)
		{
			maxASAPlevel = pair.first;
		}
	}

	for (int i = 0; i <= maxASAPlevel; ++i)
	{
		for (DFGNode *node : asapLevelNodeList[i])
		{
			if (std::find(sortedNodeList.begin(), sortedNodeList.end(), node) == sortedNodeList.end())
			{
				sortedNodeList.push_back(node);
			}
		}
	}

	std::cout << "***********SORTED LIST*******************\n";
	for (DFGNode *node : sortedNodeList)
	{
		std::cout << "Node=" << node->idx << ",ASAP=" << node->ASAP << "\n";
	}
	//	assert(false);

	std::reverse(sortedNodeList.begin(), sortedNodeList.end());

	int unmappedMemNodeCount = 0;
	for (DFGNode *node : this->sortedNodeList)
	{
		if (node->isMemOp())
		{
			if (node->rootDP == NULL)
			{
				unmappedMemNodeCount++;
				dfg->unmappedMemOpSet.insert(node);
			}
		}
	}
	dfg->unmappedMemOps = unmappedMemNodeCount;
}

void CGRAXMLCompile::PathFinderMapper::sortBackEdgePriorityALAP()
{

	sortedNodeList.clear();

	struct BEDist
	{
		DFGNode *parent;
		DFGNode *child;
		int dist;
		BEDist(DFGNode *parent, DFGNode *child, int dist) : parent(parent), child(child), dist(dist) {}
		bool operator<(const BEDist &other) const
		{
			if (dist == other.dist)
			{
				return true;
			}
			return dist > other.dist;
		}
		//		bool operator==(const BEDist& other) const{
		//			return parent==other.parent & child==other.child;
		//		}
	};

	std::set<BEDist> backedges;

	for (DFGNode &node : dfg->nodeList)
	{

		if (node.idx == 97)
		{
			std::cout << "node_idx:97,node_ALAP:" << node.ALAP << "\n";
		}
		for (DFGNode *child : node.children)
		{

			if (node.idx == 97)
			{
				std::cout << "child_idx:" << child->idx << "child_ALAP:" << child->ALAP << "\n";
			}

			if (child->ALAP <= node.ALAP)
			{
				std::cout << "inserting for : node=" << node.idx << ",child:" << child->idx << "\n";
				backedges.insert(BEDist(&node, child, node.ALAP - child->ALAP));
			}
		}
	}

	for (DFGNode &node : dfg->nodeList)
	{
		for (DFGNode *recParent : node.recParents)
		{
			backedges.insert(BEDist(&node, recParent, node.ALAP - recParent->ALAP));
		}
	}

	std::map<DFGNode *, std::vector<DFGNode *>> beparentAncestors;
	//	std::map<DFGNode*,std::vector<DFGNode*>> bechildAncestors;

	for (BEDist be : backedges)
	{
		std::cout << "BE PARENT = " << be.parent->idx << ",dist=" << be.dist << "\n";
		std::cout << "BE CHILD = " << be.child->idx << "\n";

		std::cout << "Ancestory : "
				  << "\n";
		beparentAncestors[be.parent] = dfg->getAncestoryALAP(be.parent);
		std::cout << "\n";

		if (std::find(beparentAncestors[be.parent].begin(),
					  beparentAncestors[be.parent].end(),
					  be.child) == beparentAncestors[be.parent].end())
		{
			std::cout << "BE CHILD does not belong BE Parent's Ancestory\n";
		}
		else
		{
			//change this be.parent if PHI nodes are not removed
			//			RecPHIs.insert(be.parent);
		}

		//		bechildAncestors[be.child]=dfg->getAncestory(be.child);
	}

	//	std::vector<DFGNode*> mergedAncestory;
	std::map<DFGNode *, std::vector<DFGNode *>> mergedAncestories;
	mergedAncestories.clear();
	std::map<DFGNode *, DFGNode *> mergedKeys;
	for (BEDist be : backedges)
	{
		//		write a logic to merge ancestories where if one be's child is present in some other be's parent's ancesotory'
		bool merged = false;
		for (std::pair<DFGNode *, std::vector<DFGNode *>> pair : mergedAncestories)
		{
			DFGNode *key = pair.first;
			if (std::find(mergedAncestories[key].begin(), mergedAncestories[key].end(), be.child) != mergedAncestories[key].end())
			{
				std::cout << "Merging :: " << key->idx << ", " << be.parent->idx << "\n";
				mergedAncestories[key] = dfg->mergeAncestoryALAP(mergedAncestories[key], beparentAncestors[be.parent]);
				merged = true;
				mergedKeys[be.parent] = key;
			}
		}
		if (!merged)
		{
			mergedAncestories[be.parent] = beparentAncestors[be.parent];
			mergedKeys[be.parent] = be.parent;
		}
	}

	for (BEDist be : backedges)
	{
		assert(mergedKeys.find(be.parent) != mergedKeys.end());
		std::vector<DFGNode *> ancestoryNodes = mergedAncestories[mergedKeys[be.parent]];
		for (DFGNode *ancestorNode : ancestoryNodes)
		{
			if (std::find(sortedNodeList.begin(), sortedNodeList.end(), ancestorNode) == sortedNodeList.end())
			{
				sortedNodeList.push_back(ancestorNode);
			}
		}
		std::cout << "BE PARENT = " << be.parent->idx << ",dist=" << be.dist << "\n";
		if (std::find(sortedNodeList.begin(), sortedNodeList.end(), be.parent) == sortedNodeList.end())
		{
			sortedNodeList.push_back(be.parent);
		}

		ancestoryNodes = dfg->getAncestoryALAP(be.child);
		for (DFGNode *ancestorNode : ancestoryNodes)
		{
			if (std::find(sortedNodeList.begin(), sortedNodeList.end(), ancestorNode) == sortedNodeList.end())
			{
				sortedNodeList.push_back(ancestorNode);
			}
		}
		std::cout << "BE CHILD = " << be.child->idx << "\n";
		if (std::find(sortedNodeList.begin(), sortedNodeList.end(), be.child) == sortedNodeList.end())
		{
			sortedNodeList.push_back(be.child);
		}
	}

	std::map<int, std::vector<DFGNode *>> alapLevelNodeList;
	for (DFGNode &node : dfg->nodeList)
	{
		alapLevelNodeList[node.ALAP].push_back(&node);
	}

	int maxALAPlevel = 0;
	for (std::pair<int, std::vector<DFGNode *>> pair : alapLevelNodeList)
	{
		if (pair.first > maxALAPlevel)
		{
			maxALAPlevel = pair.first;
		}
	}

	for (int i = 0; i <= maxALAPlevel; ++i)
	{
		for (DFGNode *node : alapLevelNodeList[i])
		{
			if (std::find(sortedNodeList.begin(), sortedNodeList.end(), node) == sortedNodeList.end())
			{
				sortedNodeList.push_back(node);
			}
		}
	}

	std::cout << "***********SORTED LIST*******************\n";
	for (DFGNode *node : sortedNodeList)
	{
		std::cout << "Node=" << node->idx << ",ALAP=" << node->ALAP << "\n";
	}
	//	assert(false);

	std::reverse(sortedNodeList.begin(), sortedNodeList.end());

	int unmappedMemNodeCount = 0;
	for (DFGNode *node : this->sortedNodeList)
	{
		if (node->isMemOp())
		{
			if (node->rootDP == NULL)
			{
				unmappedMemNodeCount++;
				dfg->unmappedMemOpSet.insert(node);
			}
		}
	}
	dfg->unmappedMemOps = unmappedMemNodeCount;
}

int CGRAXMLCompile::PathFinderMapper::getlatMinStartsPHI(const DFGNode *currNode,
														 const std::map<DFGNode *, std::vector<Port *>> &possibleStarts)
{

	int min;
	std::map<DFGNode *, int> minLat;

	for (std::pair<DFGNode *, std::vector<Port *>> pair : possibleStarts)
	{
		int latm = 100000000;

		for (Port *p : pair.second)
		{
			if (p->getLat() < latm)
			{
				latm = p->getLat();
			}
		}
		assert(latm != 100000000);
		minLat[pair.first] = latm;
	}

	int max = 0;
	for (std::pair<DFGNode *, int> pair : minLat)
	{
		if (max < pair.second)
		{
			max = pair.second;
		}
		std::cout << "getlatMinStartsPHI :: minLat = " << max << "\n";
	}

	//	std::map<std::string,int> oplatencyMap;
	//	cgra->PEArr[0][0][0]->getMEMIns(oplatencyMap);

	std::unordered_map<std::string, int> oplatencyMap = cgra->getGlobalOPMinLatencyMap();

	int recphi_lat = 0;
	if (RecPHIs.find((DFGNode *)currNode) != RecPHIs.end())
	{
		std::cout << "RecPHI found!!!! : " << currNode->idx << "\n";

		for (DFGNode *child : currNode->children)
		{
			for (DFGNode *childparent : child->parents)
			{
				if (childparent == currNode)
					continue;

				int oplatency = oplatencyMap[childparent->op];
				for (DFGNode *parentchildparent : childparent->parents)
				{
					if (parentchildparent->rootDP != NULL)
					{
						int newlat = parentchildparent->rootDP->getLat() + oplatency;
						if (newlat > recphi_lat)
						{
							std::cout << "RecPhi Lat = " << newlat << "\n";
							recphi_lat = newlat;
						}
					}
				}
			}
		}
	}

	if (recphi_lat > max)
		max = recphi_lat;

	//	assert(max!=-1);
	return max;
}

std::set<CGRAXMLCompile::DFGNode *> CGRAXMLCompile::PathFinderMapper::getElders(DFGNode *node)
{
	//	std::set<DFGNode*> res;
	//
	//	std::map<int,DFGNode*> descendents;
	//	std::queue<std::set<DFGNode*>> q;
	//	std::set<DFGNode*> initLevel; initLevel.insert(node);
	//	q.push(initLevel);
	//	int level_ctr=0;
	//
	//	while(!q.empty()){
	//		std::set<DFGNode*> level = q.front(); q.pop();
	//		level_ctr++;
	//		for(DFGNode* levelnode : level){
	//			for(DFGNode* child : levelnode->children){
	//				if(levelnode->childNextIter[child] == 1) continue; //ignore backedges;
	//				if(descendents.find(child) == descendents.end()){
	//					descendents.insert(child);
	//					q.push(child);
	//				}
	//			}
	//
	//
	//		}
	//
	//
	//		for(DFGNode* child : top->children){
	//			if(top->childNextIter[child] == 1) continue; //ignore backedges
	//			if(descendents.find(child) == descendents.end()){
	//				descendents.insert(child);
	//				q.push(child);
	//			}
	//		}
	//	}
	//
	//
	//
	//	return res;
}

int CGRAXMLCompile::PathFinderMapper::getMaxLatencyBE(DFGNode *node, std::map<DataPath *, beParentInfo> &beParentDests, int &downSteamOps)
{

	std::set<BackEdge> setBackEdges;
	std::cout << "getMaxLatencyBE started!\n";

	std::cout << "NODE ASAP = " << node->ASAP << "\n";

	for (std::pair<BackEdge, std::set<DFGNode *>> pair : RecCycles)
	{
		BackEdge be = pair.first;
		std::set<DFGNode *> rec_nodes = pair.second;
		if (rec_nodes.find(node) != rec_nodes.end())
		{
			if (be.second->rootDP != NULL)
			{
				std::cout << "RecSet(" << be.first->idx << "," << be.second->idx << ")"
						  << " : ";
				for (DFGNode *n : rec_nodes)
				{
					std::cout << n->idx << ",";
				}
				std::cout << "\n";
				setBackEdges.insert(pair.first);
			}
		}
	}

	for (std::pair<BackEdge, std::set<DFGNode *>> pair : RecCyclesLS)
	{
		BackEdge be = pair.first;
		std::set<DFGNode *> rec_nodes = pair.second;
		if (rec_nodes.find(node) != rec_nodes.end())
		{
			if (be.second->rootDP != NULL)
			{
				std::cout << "RecSet(" << be.first->idx << "," << be.second->idx << ")"
						  << " : ";
				for (DFGNode *n : rec_nodes)
				{
					std::cout << n->idx << ",";
				}
				std::cout << "\n";
				setBackEdges.insert(pair.first);
			}
		}
	}

	// PE *samplePE = cgra->PEArr[0][0][0];
	std::unordered_map<std::string, int> OpLatency = cgra->getGlobalOPMinLatencyMap();
	// samplePE->getNonMEMIns(OpLatency);
	// samplePE->getMemOnlyIns(OpLatency);

	int maxLat = LARGE_VALUE;

	for (BackEdge be : setBackEdges)
	{
		int maxLatency = be.second->rootDP->getLat() + cgra->get_t_max();
		int noDownStreamOps = 0;
		//		maxLatency = maxLatency - OpLatency[be.first->op];
		std::cout << "maxLatency = " << maxLatency << "\n";
		std::map<int, std::set<DFGNode *>> asapOrder;
		std::map<int, int> asapMaxOpLat;
		std::map<int, int> asapMaxLat;

		if (RecCycles.find(be) != RecCycles.end())
		{
			//			for(DFGNode* n : RecCycles[be]){
			//				asapOrder[n->ASAP].insert(n);
			//			}
			std::vector<DFGNode *> longPath = getLongestDFGPath(node, be.first);
			for (int i = 0; i < longPath.size(); ++i)
			{
				asapOrder[longPath[i]->ASAP].insert(longPath[i]);
			}
		}

		beParentInfo bpi;
		bpi.dsMEMfound = false;

		if (RecCyclesLS.find(be) != RecCyclesLS.end())
		{
			//			for(DFGNode* n : RecCyclesLS[be]){
			//				asapOrder[n->ASAP].insert(n);
			//			}
			std::vector<DFGNode *> longPath = getLongestDFGPath(node, be.first);
			for (int i = 0; i < longPath.size(); ++i)
			{
				asapOrder[longPath[i]->ASAP].insert(longPath[i]);
			}
			maxLatency = maxLatency + 2; //the store need not to finish
			bpi.isLDST = true;
		}

		int upstreamOPs = 0;
		for (std::pair<int, std::set<DFGNode *>> pair : asapOrder)
		{
			int maxOplatency = 0;
			std::cout << "ops : ";
			for (DFGNode *n : pair.second)
			{
				std::cout << "idx=" << n->idx << "[" << n->op << "]"
						  << "(" << OpLatency[n->op] << ")"
						  << ",";
				int new_lat = OpLatency[n->op];
				if (new_lat > maxOplatency)
					maxOplatency = new_lat;
			}
			std::cout << "\n";
			std::cout << "ASAP=" << pair.first << ",OPLAT=" << maxOplatency << "\n";

			if ((bpi.dsMEMfound == false) && (node->ASAP < pair.first))
			{
				if (maxOplatency == 2)
				{
					std::cout << "MEM FOUND SET TRUE!\n";
					bpi.dsMEMfound = true;
					bpi.uptoMEMops = upstreamOPs;
				}
			}

			if (node->ASAP < pair.first)
			{
				upstreamOPs++;
			}

			asapMaxOpLat[pair.first] = maxOplatency;
		}

		std::map<int, std::set<DFGNode *>>::reverse_iterator rit = asapOrder.rbegin();

		int prevLat = maxLatency;
		while (rit != asapOrder.rend())
		{
			int asap = (*rit).first;
			asapMaxLat[asap] = prevLat - asapMaxOpLat[asap];
			prevLat = asapMaxLat[asap];

			if (asap > node->ASAP)
			{
				noDownStreamOps++;
			}

			rit++;
		}

		//		beParentInfo bpi;
		bpi.beParent = be.first;
		bpi.lat = asapMaxLat[node->ASAP];
		bpi.downStreamOps = noDownStreamOps;
		beParentDests[be.second->rootDP] = bpi;

		if (asapMaxLat[node->ASAP] < maxLat)
		{
			maxLat = asapMaxLat[node->ASAP];
			downSteamOps = noDownStreamOps;
		}
	}

	if (maxLat != LARGE_VALUE)
	{
		std::cout << "getMaxLatencyBE :: node=" << node->idx << " maxLat = " << maxLat << "\n";
		//		assert(false);
	}
	std::cout << "getMaxLatencyBE done!\n";
	return maxLat;
}

void CGRAXMLCompile::PathFinderMapper::addPseudoEdgesOrphans(DFG *dfg)
{

	std::set<int> orphanNodes;

	for (DFGNode &node : dfg->nodeList)
	{
		if (node.parents.empty())
		{
			orphanNodes.insert(node.idx);
		}
	}

	for (int nodeIdx : orphanNodes)
	{
		DFGNode *node = dfg->findNode(nodeIdx);

		std::map<int, DFGNode *> asapchild;
		for (DFGNode *child : node->children)
		{
			if (node->childNextIter[child])
				continue;
			asapchild[child->ASAP] = child;
		}

		assert(!asapchild.empty());
		DFGNode *earliestChild = (*asapchild.begin()).second;

		std::map<int, DFGNode *> asapcousin;
		for (DFGNode *parent : earliestChild->parents)
		{
			if (parent == node)
				continue;
			if (parent->childNextIter[earliestChild])
				continue;
			asapcousin[parent->ASAP] = parent;
		}

		if (!asapcousin.empty())
		{
			DFGNode *latestCousin = (*asapcousin.rbegin()).second;

			std::cout << "Adding Pseudo Connection :: parent=" << latestCousin->idx << ",to" << node->idx << "\n";
			latestCousin->children.push_back(node);
			latestCousin->childNextIter[node] = 0;
			latestCousin->childrenOPType[node] = "P";
			node->parents.push_back(latestCousin);
		}
	}

	assert(false);
}

std::vector<CGRAXMLCompile::DFGNode *> CGRAXMLCompile::PathFinderMapper::getLongestDFGPath(
	DFGNode *src, DFGNode *dest)
{

	std::vector<DFGNode *> result;
	if (src == dest)
	{
		result.push_back(src);
		return result;
	}

	std::set<std::pair<DFGNode *, int>> q_init;
	std::queue<std::set<std::pair<DFGNode *, int>>> q;

	// PE *samplePE = cgra->PEArr[0][0][0];
	std::unordered_map<std::string, int> oplatencyMap = cgra->getGlobalOPMinLatencyMap();
	// samplePE->getNonMEMIns(oplatencyMap);
	// samplePE->getMemOnlyIns(oplatencyMap);

	q_init.insert(std::make_pair(src, oplatencyMap[src->op]));
	std::map<DFGNode *, std::map<int, DFGNode *>> cameFrom;
	q.push(q_init);

	while (!q.empty())
	{
		std::set<std::pair<DFGNode *, int>> curr = q.front();
		q.pop();
		std::set<std::pair<DFGNode *, int>> next;
		for (std::pair<DFGNode *, int> p1 : curr)
		{
			DFGNode *node = p1.first;
			std::cout << node->idx << ",";
			for (DFGNode *child : node->children)
			{
				if (node->childNextIter[child] == 1)
					continue;
				int nextLat = p1.second + oplatencyMap[child->op];
				next.insert(std::make_pair(child, nextLat));
				cameFrom[child][nextLat] = node;
			}
		}
		std::cout << "\n";
		if (!next.empty())
			q.push(next);
	}

	assert(cameFrom.find(dest) != cameFrom.end());

	DFGNode *temp = dest;
	while (temp != src)
	{
		std::cout << temp->idx << " <-- ";
		result.push_back(temp);
		temp = (*cameFrom[temp].rbegin()).second;
	}
	result.push_back(src);
	std::cout << "\n";
	//	assert(false);

	std::reverse(result.begin(), result.end());
	return result;
}

int CGRAXMLCompile::PathFinderMapper::getFreeMEMPeDist(PE *currPE)
{
	int currT = currPE->T;

	// for (int y = 0; y < this->cgra->get_y_max(); ++y)
	// {
	// 	//		int tdiff = std::abs()
	// 	//		PE* destPE = this->cgra->PEArr
	// }
}

std::vector<CGRAXMLCompile::DataPath *> CGRAXMLCompile::PathFinderMapper::modifyMaxLatCandDest(
	std::map<DataPath *, int> candDestIn, DFGNode *node, bool &changed)
{

	std::vector<DataPath *> res;

	std::map<DataPath *, beParentInfo> beParentDests;
	int downStreamOps = 0;
	int maxLat = getMaxLatencyBE(node, beParentDests, downStreamOps);

	if (maxLat != LARGE_VALUE)
		assert(!beParentDests.empty());

	changed = false;

	bool isMeMOp = checkMEMOp(node->op);

	std::cout << "MaxLat = " << maxLat << "\n";
	std::cout << "IsMEMOp = " << isMeMOp << "\n";
	std::cout << "candDestIn size = " << candDestIn.size() << "\n";

	for (std::pair<DataPath *, int> pair : candDestIn)
	{

		DataPath *dp = pair.first;
		FU* fu = dp->getFU();
		PE *pe = dp->getPE();
		int offset = 0;

		if ((fu->isMEMFU()) && (isMeMOp == false))
		{
			offset = 1;
		}

		if (cgra->minLatBetweenPEs > 0)
		{
			assert(cgra->minLatBetweenPEs == 1);
			int max_dist = 0;
			for (std::pair<DataPath *, beParentInfo> pair : beParentDests)
			{
				//				if(pair.second.isLDST == false){
				PE *bePE = pair.first->getPE();
				// int dx = std::abs(bePE->X - pe->X);
				// int dy = std::abs(bePE->Y - pe->Y);
				// int dist = dx + dy;
				int dist = cgra->getQuickTimeDistBetweenPEs(bePE,pe);

				if (pair.second.isLDST == true)
				{
					dist = 0;
				}

				int dsOps = pair.second.downStreamOps;
				if (pair.second.dsMEMfound)
				{
					// dist = pe->X;
					dist = cgra->getTimeClosestMEMPE(pe);
					dsOps = pair.second.uptoMEMops;
					std::cout << "**MEM FOUND DOWN**\n";
				}

				if (maxLat != LARGE_VALUE)
				{
					std::cout << "pe=" << pe->getName() << ",";
					std::cout << "dist=" << dist << ",";
					std::cout << "slack=" << pair.second.lat - maxLat << ",";
					std::cout << "downstreamOps=" << dsOps << "\n";
				}

				int lat_slack = pair.second.lat - maxLat;
				assert(lat_slack >= 0);
				dist = dist - lat_slack - dsOps;

				if (dist > max_dist)
					max_dist = dist;
				//				}
			}

			if (max_dist > 0)
				max_dist = max_dist - 1; // can reach the neighbours in the same cycle
			offset += max_dist;
		}

		if (pair.second <= maxLat - offset)
		{
			//				std::cout << "pe=" << pe->getName() << ",";
			//				std::cout << "isMeMPE=" << pe->isMemPE << ",";
			//				std::cout << "Lat= " << pair.second << "\n";
			//				std::cout << "OK\n";
			res.push_back(pair.first);
		}
		else
		{
			changed = true;
		}
	}

	return res;
}

bool CGRAXMLCompile::PathFinderMapper::canExitCurrPE(LatPort p)
{

	std::set<LatPort> alreadyVisited;

	std::stack<LatPort> st;
	st.push(p);

	//Todo check currDP can execute the operation
	DataPath *dp = static_cast<DataPath *>(p.second->getMod());
	if (dp->getMappedNode() == NULL)
		return true;

	PE *srcPE = p.second->getMod()->getPE();
	assert(srcPE);

	while (!st.empty())
	{
		LatPort currPort = st.top();
		st.pop();
		PE *currPE = currPort.second->getMod()->getPE();
		assert(currPE);
		if (currPE != srcPE)
			return true;
		alreadyVisited.insert(currPort);
		std::vector<LatPort> nextPorts = currPort.second->getMod()->getNextPorts(currPort, this);
		for (LatPort lp : nextPorts)
		{
			if (alreadyVisited.find(lp) != alreadyVisited.end())
				continue;
			st.push(lp);
		}
	}
	return false;
}

bool CGRAXMLCompile::PathFinderMapper::checkMEMOp(string op)
{
	if (op.find("OLOAD") != string::npos || op.find("OSTORE") != string::npos)
	{
		return false;
	}

	if (op.find("LOAD") != string::npos || op.find("STORE") != string::npos)
	{
		return true;
	}

	return false;
}

void CGRAXMLCompile::PathFinderMapper::GetAllSupportedOPs(Module* currmod, unordered_set<string>& supp_ops, unordered_set<string>& supp_pointers){
	// cout << "GetAllSupportedOPs :: currmod = " << currmod->getFullName() << "\n";

	if(FU* fu = dynamic_cast<FU*>(currmod)){
		for(auto it = fu->supportedOPs.begin(); it != fu->supportedOPs.end(); it++){
			supp_ops.insert(it->first);
		}
	}

	if(DataPath* dp = dynamic_cast<DataPath*>(currmod)){
		for(string s : dp->accesible_memvars){
			supp_pointers.insert(s);
		}
	}

	if(CGRA* cgra_ins = dynamic_cast<CGRA*>(currmod)){
		for(Module* submod : cgra_ins->subModArr[0]){
			GetAllSupportedOPs(submod,supp_ops,supp_pointers);
		}
	}
	else{
		for(Module* submod : currmod->subModules){
			GetAllSupportedOPs(submod,supp_ops,supp_pointers);
		}
	}

}

bool CGRAXMLCompile::PathFinderMapper::Check_DFG_CGRA_Compatibility(){

	unordered_set<string> all_supp_ops;
	unordered_set<string> all_supp_pointers;

	GetAllSupportedOPs(cgra,all_supp_ops,all_supp_pointers);
	unordered_set<string> base_pointers;

	cout << "all supported pointers : \n";
	for(string ptr : all_supp_pointers){
		cout << "\t" << ptr << "\n";
	}

	cout << "all required pointers : \n";
	for(auto it = dfg->pointer_sizes.begin(); it != dfg->pointer_sizes.end(); it++){
		cout << "\t" << it->first << ",size = " << it->second << "\n";
	}

	for(DFGNode& node : dfg->nodeList){
		string op = node.op;
		if(all_supp_ops.find(op) == all_supp_ops.end()){
			cout << "op=" << op << " is not supported in this CGRA, exiting....\n";
			exit(EXIT_FAILURE);
			return false;
		}
	}

	if(cgra->is_spm_modelled){
		for(auto it = dfg->pointer_sizes.begin(); it != dfg->pointer_sizes.end(); it++){
			string pointer = it->first;
			if(all_supp_pointers.find(pointer) == all_supp_pointers.end()){
				cout << "pointer=" << pointer << " is not present in the CGRA, exiting....\n";
				exit(EXIT_FAILURE);
				return false;
			}
		}
	}
	else{
		cout << "SPMs are not modelled, therefore ignoring supported pointers check.\n";
	}

	return true;
}

void CGRAXMLCompile::PathFinderMapper::UpdateVariableBaseAddr(){

	assert(cgra);
	assert(dfg);

	// unordered_map<Module*,int> spm_addr;
	// for(auto it = cgra->Variable2SPM.begin(); it != cgra->Variable2SPM.end(); it++){
	// 	string var = it->first;
	// 	Module* spm = it->second;

	// 	if(spm_addr.find(spm) == spm_addr.end()){
	// 		spm_addr[spm] = 0;
	// 	}

	// 	int size = dfg->pointer_sizes[var];
	// 	cout << "UpdateVariableBaseAddr :: var = " << var << ", spm = " << spm->getFullName() << ", base_addr = " << spm_addr[spm] << "\n";
	// 	cgra->Variable2SPMAddr[var] = spm_addr[spm];

	// 	spm_addr[spm] = spm_addr[spm] + size;
	// }


	for(DFGNode& node : dfg->nodeList){
		if(node.gep_offset != -1){
			assert(!node.base_pointer_name.empty());
			assert(cgra->Variable2SPMAddr.find(node.base_pointer_name) != cgra->Variable2SPMAddr.end());
			node.constant = node.gep_offset + cgra->Variable2SPMAddr[node.base_pointer_name];
		}
		else if(node.op.find("OLOAD") != string::npos || node.op.find("OSTORE") != string::npos){
			//for outer loop load and stores, set the constant to the base address
			assert(!node.base_pointer_name.empty());
			assert(cgra->Variable2SPMAddr.find(node.base_pointer_name) != cgra->Variable2SPMAddr.end());
			node.constant = cgra->Variable2SPMAddr[node.base_pointer_name];
		}
	}
	// exit(EXIT_SUCCESS);
}

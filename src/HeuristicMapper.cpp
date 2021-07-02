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
#include <algorithm> // std::reverse
#include "DataPath.h"
#include "FU.h"

#include <stack>
#include <functional>
#include <set>
#include <iostream>
#include <sstream>

namespace CGRAXMLCompile
{

//HeuristicMapper::HeuristicMapper() {
//	// TODO Auto-generated constructor stub
//
//}

} /* namespace CGRAXMLCompile */

void CGRAXMLCompile::HeuristicMapper::SortTopoGraphicalDFG()
{
	sortedNodeList.clear();
	assert(false);
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
			sortedNodeList.push_back(node);
		}
	}
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

bool CGRAXMLCompile::HeuristicMapper::Map(CGRA *cgra, DFG *dfg)
{

	std::stack<DFGNode *> mappedNodes;
	std::stack<DFGNode *> unmappedNodes;
	std::map<DFGNode *, std::priority_queue<dest_with_cost>> estimatedRouteInfo;

	int backTrackCredits = this->backTrackLimit;

	this->cgra = cgra;
	this->dfg = dfg;
	//	SortSCCDFG();
	SortTopoGraphicalDFG();

	std::string mappingLogFileName = fNameLog1 + cgra->peType + "_DP" + std::to_string(this->cgra->numberofDPs) + "_II=" + std::to_string(cgra->get_t_max()) + "_MTP=" + std::to_string(enableMutexPaths) + ".mapping.csv";
	std::string mappingLog2FileName = fNameLog1 + cgra->peType + "_DP" + std::to_string(this->cgra->numberofDPs) + "_II=" + std::to_string(cgra->get_t_max()) + "_MTP=" + std::to_string(enableMutexPaths) + ".routeInfo.log";
	mappingLog.open(mappingLogFileName.c_str());
	mappingLog2.open(mappingLog2FileName.c_str());

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
		MapHeader << ",PEType = " << this->cgra->peType;
		MapHeader << ",DPs = " << this->cgra->numberofDPs;
		MapHeader << ",BB = " << node->BB;
		MapHeader << ",mutexPathEn = " << this->enableMutexPaths;
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

					//				DFGNode* prevNode = mappedNodes.top();
					//				mappedNodes.pop();
					//				unmappedNodes.push(node);
					//				unmappedNodes.push(prevNode);
					//
					//				prevNode->clear(this->dfg);
					//				estimatedRouteInfo.erase(node);

					assert(failedNode != NULL);
					unmappedNodes.push(node);
					removeFailedNode(mappedNodes, unmappedNodes, failedNode);
					failedNode->blacklistDest.insert(failedNode->rootDP);
					(failedNode)->clear(this->dfg);
					estimatedRouteInfo.erase(node);
					estimatedRouteInfo.erase(failedNode);

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
				assert(failedNode != NULL);
				backTrackCredits--;

				//				DFGNode* prevNode = mappedNodes.top();
				//				mappedNodes.pop();
				//				unmappedNodes.push(node);
				//				unmappedNodes.push(prevNode);
				//
				//				prevNode->clear(this->dfg);
				//				estimatedRouteInfo.erase(node);

				unmappedNodes.push(node);
				removeFailedNode(mappedNodes, unmappedNodes, failedNode);
				failedNode->blacklistDest.insert(failedNode->rootDP);
				(failedNode)->clear(this->dfg);
				estimatedRouteInfo.erase(node);
				estimatedRouteInfo.erase(failedNode);
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

	mappingLog << "Map Success!.\n";
	mappingLog2 << "Map Success!.\n";
	this->printMappingLog();
	this->printMappingLog2();

	std::cout << "Map Success!.\n";
	mappingLog.close();
	mappingLog2.close();
	return true;
}

bool CGRAXMLCompile::HeuristicMapper::estimateRouting(DFGNode *node,
		std::priority_queue<dest_with_cost> &estimatedRoutes, DFGNode **failedNode)
{

	std::map<DFGNode *, std::vector<Port *>> possibleStarts;
	std::map<DFGNode *, Port *> alreadyMappedChildPorts;

	bool detailedDebug = false;
	if (node->idx == 17)
		detailedDebug = true;

	//	std::cout << "EstimateEouting begin...\n";

	for (DFGNode *parent : node->parents)
	{
		//		std::cout << "parent = " << parent->idx << "\n";
		if (parent->rootDP != NULL)
		{ //already mapped
			assert(parent->rootDP->getOutputDP()->getOutPort("T"));
			//			possibleStarts[parent].push_back(parent->rootDP->getOutputDP()->getOutPort("T"));

			for (std::pair<Port *, int> pair : parent->routingPorts)
			{
				Port *p = pair.first;
				possibleStarts[parent].push_back(p);
			}
		}
	}

	for (DFGNode *child : node->children)
	{
		if (child->rootDP != NULL)
		{ // already mapped
			//			std::cout << "child="<< child->idx << ",childOpType=" << node.childrenOPType[child] << "\n";
			assert(child->rootDP->getInPort(node->childrenOPType[child]));
			alreadyMappedChildPorts[child] = child->rootDP->getInPort(node->childrenOPType[child]);
			alreadyMappedChildPorts[child]->setLat(child->rootDP->getLat());
		}
	}

	std::vector<DataPath *> candidateDests;

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
							if (dp->getMappedNode() == NULL)
							{
								//									if(dataPathCheck(dp,&node)){

								if (node->blacklistDest.find(dp) == node->blacklistDest.end())
								{
									candidateDests.push_back(dp);
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
							if (dp->getMappedNode() == NULL)
							{
								//									if(dataPathCheck(dp,&node)){

								if (node->blacklistDest.find(dp) == node->blacklistDest.end())
								{
									candidateDests.push_back(dp);
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

	int minLat = getlatMinStarts(possibleStarts);
	std::map<DataPath *, int> minLatDests = getLatCandDests(candidateDests, minLat);

	//Route Estimation
	for (DataPath *dest : candidateDests)
	{
		//		std::cout << "Candidate Dest =" ;
		//		std::cout << dest->getPE()->getName() << ".";
		//		std::cout << dest->getFU()->getName() << ".";
		//		std::cout << dest->getName() << "\n";

		//		std::map<DFGNode*,std::priority_queue<cand_src_with_cost>> parentStartLocs;
		std::priority_queue<parent_cand_src_with_cost> parentStartLocs;

		bool pathFromParentExist = true;
		for (std::pair<DFGNode *, std::vector<Port *>> pair : possibleStarts)
		{
			DFGNode *parent = pair.first;
			Port *destPort = dest->getInPort(parent->getOPtype(node));

			std::priority_queue<cand_src_with_cost> res;

			for (Port *startCand : pair.second)
			{
				int cost;
				std::vector<LatPort> path;
				std::map<Port *, std::set<DFGNode *>> mutexPaths;
				if (detailedDebug)
					std::cout << "Estimating Path" << startCand->getFullName() << "--->" << destPort->getFullName() << "\n";

				LatPort startCandLat = std::make_pair(startCand->getLat(), startCand);
				assert(startCand->getLat() != -1);
				LatPort destPortLat = std::make_pair(minLatDests[dest], destPort);
				bool pathExist = LeastCostPathAstar(startCandLat, destPortLat, path, cost, parent, mutexPaths, node);
				if (!pathExist)
				{
					if (detailedDebug)
						std::cout << "Estimate Path Failed :: " << startCand->getFullName() << "--->" << destPort->getFullName() << "\n";
					continue;
				}
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

		bool pathExistMappedChild = true;
		std::priority_queue<dest_child_with_cost> alreadyMappedChilds;
		for (std::pair<DFGNode *, Port *> pair : alreadyMappedChildPorts)
		{
			DFGNode *child = pair.first;
			Port *childDestPort = pair.second;
			std::vector<LatPort> path;
			int cost;

			FU *parentFU = dest->getFU();
			assert(parentFU->supportedOPs.find(node->op) != parentFU->supportedOPs.end());
			int latency = parentFU->supportedOPs[node->op];
			Port *destPort = dest->getOutputPort(latency);

			std::map<Port *, std::set<DFGNode *>> mutexPaths;
			if (detailedDebug)
				std::cout << "Estimating Path" << destPort->getFullName() << "--->" << childDestPort->getFullName() << "\n";

			LatPort childDestPortLat = std::make_pair(childDestPort->getLat(), childDestPort);
			assert(childDestPort->getLat() != -1);
			LatPort destPortLat = std::make_pair(minLatDests[dest] + latency, destPort);

			pathExistMappedChild = pathExistMappedChild & LeastCostPathAstar(destPortLat, childDestPortLat, path, cost, node, mutexPaths, node);

			if (!pathExistMappedChild)
			{
				*failedNode = child;
				break;
			}

			dest_child_with_cost dcwc(child, child->rootDP, childDestPortLat, destPortLat, cost);
			alreadyMappedChilds.push(dcwc);
		}
		if (!pathExistMappedChild)
		{
			if (detailedDebug)
				std::cout << "Estimating Path Failed!\n";
			continue; //if it cannot be mapped to child abort the estimation for this dest
		}

		assert(pathFromParentExist);
		assert(pathExistMappedChild);
		dest_with_cost dest_with_cost_ins(parentStartLocs, alreadyMappedChilds, dest, minLatDests[dest], node, 0, this->dfg->unmappedMemOps, this);
		estimatedRoutes.push(dest_with_cost_ins);
	}
	//	std::cout << "EstimateEouting end!\n";
	if (estimatedRoutes.empty())
		assert(*failedNode != NULL);
	return !estimatedRoutes.empty();
}

bool CGRAXMLCompile::HeuristicMapper::Route(DFGNode *node,
		std::priority_queue<dest_with_cost> &estimatedRoutes, DFGNode **failedNode)
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

				if (LeastCostPathAstar(p, dest_child_with_cost_ins.childDest, pathTmp, cost, node, mutexPathsTmp, node))
				{
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
				alreadMappedChildRouteSucc = LeastCostPathAstar(head.src, dest, path, cost, node, mutexPaths, node);
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
				succ = LeastCostPathAstar(src, dest, path, cost, parent, mutexPath, node);
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
			std::cout << "node=" << node->idx << ",op=" << node->op << " is mapped to " << currDest.dest->getPE()->getName() << "\n";
			std::cout << "routing info ::\n";
			for (DFGNode *parent : node->parents)
			{
				std::cout << "parent routing port size = " << parent->routingPorts.size() << "\n";
				for (std::pair<Port *, int> pair : parent->routingPorts)
				{
					Port *p = pair.first;
					//					if(node.routingPortDestMap[p]==&node){
					std::cout << "fr:" << parent->idx << " :: ";
					std::cout << p->getFullName();
					if (mappedParentMutexPaths[parent].find(p) != mappedParentMutexPaths[parent].end())
					{
						std::cout << "|mutex(";
						for (DFGNode *mutexnode : mappedParentMutexPaths[parent][p])
						{
							std::cout << mutexnode->idx << ",";
						}
						std::cout << ")";
					}
					std::cout << "\n";
					//					}
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

template <typename T>
std::set<T> getUnion(const std::set<T> &a, const std::set<T> &b)
{
	std::set<T> result = a;
	result.insert(b.begin(), b.end());
	return result;
}

int CGRAXMLCompile::HeuristicMapper::getMinimumII(CGRA *cgra, DFG *dfg)
{

	std::map<std::string, int> opHist;

	for (DFGNode &node : dfg->nodeList)
	{

		if (opHist.find(node.op) == opHist.end())
		{
			opHist[node.op] = 0;
		}
		opHist[node.op]++;
	}

	std::map<std::string, std::set<DataPath *>> opFUs;

	assert(cgra->get_t_max() > 0);

	for (PE *currPE : cgra->getSpatialPEList(0))
	{
		for (Module *FU_mod : currPE->subModules)
		{
			if (FU *fu = dynamic_cast<FU *>(FU_mod))
			{
				for (std::pair<std::string, int> pair : fu->supportedOPs)
				{
					std::string suppOp = pair.first;
					for (Module *DP_mod : fu->subModules)
					{
						if (DataPath *dp = dynamic_cast<DataPath *>(DP_mod))
						{
							opFUs[suppOp].insert(dp);
						}
					}
				}
			}
		}
	}

	struct opFUstr
	{
		std::string op;
		int count;
		opFUstr(std::string op, int count) : op(op), count(count){};

		bool operator<(const opFUstr &rhs)
		{
			return this->count < rhs.count;
		}
	};

	std::vector<opFUstr> opFUstr_vec;
	for (std::pair<std::string, std::set<DataPath *>> pair : opFUs)
	{
		opFUstr_vec.push_back(opFUstr(pair.first, pair.second.size()));
	}
	std::sort(opFUstr_vec.begin(), opFUstr_vec.end());

	int ii = 1;
	int cummOp = 0;
	std::set<DataPath *> cummDP;
	for (opFUstr a : opFUstr_vec)
	{
		std::cout << "op=" << a.op << ",opcount=" << opHist[a.op] << ",dpcount=" << a.count;
		int new_ratio = (opHist[a.op] + a.count - 1) / a.count; // int ceil
		std::cout << ",ratio=" << new_ratio;
		ii = std::max(ii, new_ratio);

		cummOp += opHist[a.op];
		for (DataPath *dp : opFUs[a.op])
		{
			cummDP.insert(dp);
		}
		std::cout << ",cummOp=" << cummOp << ",cummDP=" << cummDP.size();
		int cumm_ratio = (cummOp + cummDP.size() - 1) / cummDP.size();
		std::cout << ",ratio=" << cumm_ratio << "\n";
		ii = std::max(ii, cumm_ratio);
	}
	std::cout << "Min II = " << ii << "\n";
	return ii;
}

void CGRAXMLCompile::HeuristicMapper::SortSCCDFG()
{

	sortedNodeList.clear();
	assert(this->dfg);
	std::vector<std::set<DFGNode *>> SCCs = dfg->getSCCs();

	struct scc_with_size
	{
		std::set<DFGNode *> scc;
		int nodesize;
		scc_with_size(std::set<DFGNode *> scc, int nodesize) : scc(scc), nodesize(nodesize) {}

		bool operator<(const scc_with_size &rhs)
		{
			return this->nodesize < rhs.nodesize;
		}
		struct greater
		{
			bool operator()(const scc_with_size &lhs, const scc_with_size &rhs)
			{
				return lhs.nodesize > rhs.nodesize;
			}
		};
	};

	std::vector<scc_with_size> scc_with_size_vec;
	for (std::set<DFGNode *> scc : SCCs)
	{
		scc_with_size_vec.push_back(scc_with_size(scc, scc.size()));
	}
	std::sort(scc_with_size_vec.begin(), scc_with_size_vec.end());

	std::cout << "SortSCCDFG::";
	for (scc_with_size sccws : scc_with_size_vec)
	{
		for (DFGNode *node : sccws.scc)
		{
			std::cout << node->idx << ",";
			sortedNodeList.push_back(node);
		}
	}
	std::cout << "\n";
}

void CGRAXMLCompile::HeuristicMapper::assignPath(DFGNode *src, DFGNode *dest,
		std::vector<LatPort> path)
{

	std::cout << "assigning path from:" << src->idx << " to:" << dest->idx << "\n";

	int srcPortCount = 0;
	for (LatPort p : path)
	{

		//		if(p->getName().compare("T")==0){
		//			assert(p->getNode()==src);
		//		}

		if (p.second->getNode() == src)
		{
			srcPortCount++;
			continue;
		}

		p.second->setNode(src, p.first);

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

bool CGRAXMLCompile::HeuristicMapper::LeastCostPathAstar(LatPort start, LatPort end, std::vector<LatPort> &path, int &cost, DFGNode *node, std::map<Port *, std::set<DFGNode *>> &mutexPaths, DFGNode *currNode)
{

	//	std::cout << "LeastCoastPath started with start=" << start->getFullName() << " to end=" << end->getFullName() << "\n";

	std::map<LatPort, int> cost_to_port;
	std::map<LatPort, LatPort> cameFrom;

	path.clear();
	mutexPaths.clear();

	bool detailedDebug = false;
	//	if(currNode->idx==85)detailedDebug=true;

	//	struct port_heuristic{
	//		Port* p;
	//		int heuristic;
	//
	//		int calc_heuristic(Port* src, Port* dest){
	//			PE* srcPE = src->findParentPE();
	//			assert(srcPE);
	//			PE* destPE = dest->findParentPE();
	//			assert(destPE);
	//
	//			CGRA* currCGRA = srcPE->getCGRA();
	//			assert(currCGRA);
	//
	//			int dist_dest = std::abs(destPE->Y - srcPE->Y) + std::abs(destPE->X - srcPE->X)
	//			                + std::abs((destPE->T - srcPE->T + currCGRA->get_t_max())%currCGRA->get_t_max());
	//			return dist_dest;
	//		}
	//
	//		port_heuristic(Port* p, Port* dest){
	//			this->p=p;
	//			heuristic=calc_heuristic(p,dest);
	//		}
	//
	//		port_heuristic(Port* p, int cost){
	//			this->p=p;
	//			this->heuristic=cost;
	//		}
	//
	//		port_heuristic(Port* p, Port* dest, int cost){
	//			this->p=p;
	//			this->heuristic=cost*100 + calc_heuristic(p,dest);
	//		}
	//
	//		bool operator<(const port_heuristic& rhs) const{
	//			return this->heuristic > rhs.heuristic;
	//		}
	//
	////		bool operator>(const port_heuristic& rhs) const{
	////			return this->heuristic > rhs.heuristic;
	////		}
	//
	//	};

	struct port_heuristic
	{
		LatPort p;
		int heuristic;

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

		port_heuristic(LatPort p, LatPort dest)
		{
			this->p = p;
			heuristic = calc_heuristic(p, dest);
		}

		port_heuristic(LatPort p, int cost)
		{
			this->p = p;
			this->heuristic = cost;
		}

		port_heuristic(LatPort p, LatPort dest, int cost)
		{
			this->p = p;
			this->heuristic = cost * 100 + calc_heuristic(p, dest);
		}

		bool operator<(const port_heuristic &rhs) const
		{
			return this->heuristic > rhs.heuristic;
		}

		//		bool operator>(const port_heuristic& rhs) const{
		//			return this->heuristic > rhs.heuristic;
		//		}
	};

	//	std::queue<Port*> q;
	std::priority_queue<port_heuristic> q;

	q.push(port_heuristic(start, 0));
	//	path.push_back(start);

	cost_to_port[start] = 0;

	LatPort currPort;
	std::vector<LatPort> deadEnds;

	while (!q.empty())
	{
		port_heuristic curr = q.top();
		currPort = curr.p;
		q.pop();

		if (detailedDebug)
			std::cout << "currPort=" << currPort.second->getFullName() << "\n";

		if (currPort == end)
		{
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
		std::vector<LatPort> nextPorts = currPort.second->getMod()->getNextPorts(currPort, this);

		//		std::cout << "nextPorts size = " << nextPorts.size() << "\n";
		int q_len = q.size();
		for (LatPort nextLatPort : nextPorts)
		{
			Port *nextPort = nextLatPort.second;
			bool isNextPortFree = false;
			bool isNextPortMutex = false;
			if (enableMutexPaths)
			{
				if (nextPort->getNode() == NULL)
				{
					isNextPortFree = true;
				}
				else if (dfg->mutexBBs[nextPort->getNode()->BB].find(node->BB) != dfg->mutexBBs[nextPort->getNode()->BB].end())
				{
					// next BB is mutually exclusive with current nodes BB, then this can be mapped.
					isNextPortFree = true;
					isNextPortMutex = true;
					mutexPaths[nextPort].insert(nextPort->getNode());
					mutexPaths[nextPort].insert(node);
				}
			}
			else
			{
				if (nextPort->getNode() == NULL)
				{
					isNextPortFree = true;
				}
			}
			if (isNextPortFree)
			{ // unmapped port
				if (detailedDebug)
					std::cout << "\tnextPort=" << nextPort->getFullName() << ",";
				int nextPortCost = cost_to_port[currPort] + calculateCost(currPort, nextLatPort, end);
				if (detailedDebug)
					std::cout << "cost=" << nextPortCost << "\n";
				if (isNextPortMutex)
				{
					//no cost is added in using mutually exclusive routes
					nextPortCost = cost_to_port[currPort];
				}

				if (cost_to_port.find(nextLatPort) != cost_to_port.end())
				{
					if (cost_to_port[nextLatPort] > nextPortCost)
					{
						cost_to_port[nextLatPort] = nextPortCost;
						cameFrom[nextLatPort] = currPort;
						q.push(port_heuristic(nextLatPort, end, nextPortCost));
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
					q.push(port_heuristic(nextLatPort, end, nextPortCost));
				}
			}
			else
			{
				if (detailedDebug)
					std::cout << "\t[MAPPED=" << nextPort->getNode()->idx << "]nextPort=" << nextPort->getFullName() << "\n";
			}
		}
		if (q.size() == q_len)
		{
			deadEnds.push_back(currPort);
		}
	}

	if (currPort != end)
	{
		std::cout << "LeastCostPath failed!\n";
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
		return false; //routing failure
	}

	path.clear();
	assert(currPort == end);
	while (currPort != start)
	{
		path.push_back(currPort);
		assert(cameFrom.find(currPort) != cameFrom.end());
		currPort = cameFrom[currPort];
	}
	path.push_back(start);
	std::reverse(path.begin(), path.end());
	cost = cost_to_port[end];

	//	std::cout << "Path::";
	//	for(Port* p : path){
	//		std::cout  << p->getFullName() << "-->";
	//	}
	//	std::cout << "\n";
	//	std::cout << "LeastCostPath success!\n";
	return true;
}

int CGRAXMLCompile::HeuristicMapper::calculateCost(LatPort src,
		LatPort next_to_src, LatPort dest)
{

	PE *srcPE = src.second->findParentPE();
	assert(srcPE);
	PE *nextPE = next_to_src.second->findParentPE();
	assert(nextPE);

	int distance = abs(nextPE->Y - srcPE->Y) + abs(nextPE->X - srcPE->X) + regDiscourageFactor * ((nextPE->T - srcPE->T + cgra->get_t_max()) % cgra->get_t_max());
	// int distance = regDiscourageFactor * ((nextPE->T - srcPE->T + cgra->get_t_max()) % cgra->get_t_max());

	distance = distance * PETransitionCostFactor + PortTransitionCost;

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

	return distance;
}

void CGRAXMLCompile::HeuristicMapper::printMappingLog()
{

	struct util
	{
		void static repeatedPush(std::stringstream &ss, std::string pushStr, int count)
								{
			for (int i = 0; i < count; ++i)
			{
				ss << pushStr;
			}
								}
	};

	vector<string> lineHeaders;

	vector<PE *> zeroth_spatialPEs = cgra->getSpatialPEList(0);
	std::stringstream peHeader;
	std::stringstream fuHeader;
	std::stringstream dpHeader;
	for (PE *pe : zeroth_spatialPEs)
	{
		peHeader << pe->getName();

		int fuCount = 0;
		int dpCount = 0;
		for (Module *mod : pe->subModules)
		{
			if (FU *fu = dynamic_cast<FU *>(mod))
			{
				fuCount++;
				for (Module *mod : fu->subModules)
				{
					if (DataPath *dp = dynamic_cast<DataPath *>(mod))
					{
						dpCount++;
					}
				}
			}
		}

		int inputPortCount = pe->inputPorts.size();
		int outputPortCount = pe->outputPorts.size();
		int regConPortCount = pe->getRegConPorts().size()*2;
		int totalColumns = dpCount + inputPortCount + outputPortCount + regConPortCount;

		for (Module *mod : pe->subModules)
		{
			if (FU *fu = dynamic_cast<FU *>(mod))
			{
				fuHeader << fu->getName() << ",";
				for (Module *mod : fu->subModules)
				{
					if (DataPath *dp = dynamic_cast<DataPath *>(mod))
					{
						dpHeader << dp->getName() << ",";
					}
				}
			}
		}

		for (Port *ip : pe->inputPorts)
		{
			dpHeader << ip->getName() << ",";
		}

		for (Port *op : pe->outputPorts)
		{
			dpHeader << op->getName() << ",";
		}

		for (pair<Port*,Port*> pp : pe->getRegConPorts())
		{
			Port* ri = pp.first;
			Port* ro = pp.second;

			dpHeader << ri->getName() << ",";
			dpHeader << ro->getName() << ",";
		}

		util::repeatedPush(peHeader, ",", totalColumns);
		util::repeatedPush(fuHeader, ",", totalColumns - fuCount);
	}

	mappingLog << "," << peHeader.str() << "\n";
	mappingLog << "," << fuHeader.str() << "\n";
	mappingLog << "," << dpHeader.str() << "\n";

	std::map<int, std::vector<std::vector<std::string>>> lineMatrix;

	for (int t = 0; t < cgra->get_t_max(); ++t)
	{

		//				peHeader << "PE_" << t << y << x  << ",";
		// peHeader << "X=" << x << ",";
		vector<PE *> spatialPEs = cgra->getSpatialPEList(t);
		for (PE *pe : spatialPEs)
		{
			// PE *pe = cgra->PEArr[t][y][x];

			std::stringstream peHeader;
			std::stringstream fuHeader;
			std::stringstream dpHeader;
			std::stringstream dpOp;

			peHeader << "T=" << t << "," << pe->getName();

			int fuCount = 0;
			int dpCount = 0;
			for (Module *mod : pe->subModules)
			{
				if (FU *fu = dynamic_cast<FU *>(mod))
				{
					fuCount++;
					for (Module *mod : fu->subModules)
					{
						if (DataPath *dp = dynamic_cast<DataPath *>(mod))
						{
							dpCount++;
						}
					}
				}
			}

			int inputPortCount = pe->inputPorts.size();
			int outputPortCount = pe->outputPorts.size();
			int regConPortCount = pe->getRegConPorts().size()*2;
			int totalColumns = dpCount + inputPortCount + outputPortCount + regConPortCount;

			for (Module *mod : pe->subModules)
			{
				if (FU *fu = dynamic_cast<FU *>(mod))
				{
					fuHeader << fu->getName() << ",";
					for (Module *mod : fu->subModules)
					{
						if (DataPath *dp = dynamic_cast<DataPath *>(mod))
						{
							dpHeader << dp->getName() << ",";
							if (dp->getMappedNode())
							{
								dpOp << dp->getMappedNode()->idx << ":" << dp->getMappedNode()->op;
								if (dp->getMappedNode()->hasConst)
								{
									dpOp << "+C";
								}
								dpOp << "(";
								for (DFGNode *parent : dp->getMappedNode()->parents)
								{
									dpOp << parent->idx << "-";
								}
								dpOp << "|";
								for (DFGNode *child : dp->getMappedNode()->children)
								{
									dpOp << child->idx << "-";
								}
								dpOp << ")";
								dpOp << ",";
							}
							else
							{
								dpOp << "---,";
							}
						}
					}
				}
			}

			for (Port *ip : pe->inputPorts)
			{
				dpHeader << ip->getName() << ",";
				if (ip->getNode())
				{
					dpOp << ip->getNode()->idx /*<< ":" << ip.getNode()->op */ << ",";
				}
				else
				{
					dpOp << "---,";
				}
			}

			for (Port *op : pe->outputPorts)
			{
				dpHeader << op->getName() << ",";
				if (op->getNode())
				{
					dpOp << op->getNode()->idx /*<< ":" << op.getNode()->op */ << ",";
				}
				else
				{
					dpOp << "---,";
				}
			}

			for (pair<Port*,Port*> pp : pe->getRegConPorts())
			{
				Port* ri = pp.first;
				Port* ro = pp.second;

				dpHeader << ri->getName() << ",";
				if (ri->getNode())
				{
					dpOp << ri->getNode()->idx /*<< ":" << op.getNode()->op */ << ",";
				}
				else
				{
					dpOp << "---,";
				}

				dpHeader << ro->getName() << ",";
				if (ro->getNode())
				{
					dpOp << ro->getNode()->idx /*<< ":" << op.getNode()->op */ << ",";
				}
				else
				{
					dpOp << "---,";
				}

			}

			std::vector<string> lineWord;
			// lineWord.push_back(peHeader.str());
			// lineWord.push_back(fuHeader.str());
			// lineWord.push_back(dpHeader.str());
			lineWord.push_back(dpOp.str());

			lineMatrix[t].push_back(lineWord);
		}
	}

	int lineCount = 1;

	//print line matrix
	for (int t = 0; t < cgra->get_t_max(); ++t)
	{
		mappingLog << "T=" << t << ",";
		for (vector<string> linewords : lineMatrix[t])
		{
			for (int l = 0; l < lineCount; ++l)
			{
				mappingLog << linewords[l];
			}
			// mappingLog << "\n";
		}
		mappingLog << "\n";
	}
	mappingLog << "************************************\n";
}

bool CGRAXMLCompile::HeuristicMapper::LeastCostPathDjk(Port *start, Port *end,
		std::vector<Port *> &path, int &cost, DFGNode *node,
		std::map<Port *, std::set<DFGNode *>> &mutexPaths)
{
	//
	//	//	std::cout << "LeastCoastPath started with start=" << start->getFullName() << " to end=" << end->getFullName() << "\n";
	//
	//		std::map<Port*,int> cost_to_port;
	//		std::map<Port*,Port*> cameFrom;
	//
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
	//
	//	//	std::queue<Port*> q;
	//		std::priority_queue<port_heuristic> q;
	//
	//		q.push(port_heuristic(start,0));
	//	//	path.push_back(start);
	//
	//		cost_to_port[start]=0;
	//
	//		Port* currPort;
	//		while(!q.empty()){
	//			port_heuristic curr = q.top();
	//			currPort = curr.p;
	//			q.pop();
	//
	//	//		std::cout << "currPort=" << currPort->getFullName() << "\n";
	//
	//			if(currPort == end){
	//				break;
	//			}
	//
	//
	//			std::vector<Port*> nextPorts = currPort->getMod()->getNextPorts(currPort,this);
	////			std::vector<Port*> nextPorts = currPort->getMod()->connections[currPort];
	////			if(currPort->getType()==OUT){
	////				if(currPort->getMod()->getParent()){
	////					for(Port* p : currPort->getMod()->getParent()->connections[currPort]){
	////	//					std::cout << currPort->getMod()->getParent()->getName() << "***************\n";
	////						nextPorts.push_back(p);
	////					}
	////				}
	////			}
	//
	//	//		std::cout << "nextPorts size = " << nextPorts.size() << "\n";
	//			for(Port* nextPort : nextPorts){
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
	//				if(isNextPortFree){ // unmapped port
	//	//				std::cout << "\tnextPort=" << nextPort->getFullName() << ",";
	//					int nextPortCost = cost_to_port[currPort] + calculateCost(currPort,nextPort,end);
	//	//				std::cout << "cost=" << nextPortCost << "\n";
	//					if(isNextPortMutex){
	//						//no cost is added in using mutually exclusive routes
	//						nextPortCost = cost_to_port[currPort];
	//					}
	//
	//					if(cost_to_port.find(nextPort)!=cost_to_port.end()){
	//						if(cost_to_port[nextPort] > nextPortCost){
	//							cost_to_port[nextPort]=nextPortCost;
	//							cameFrom[nextPort]=currPort;
	//							q.push(port_heuristic(nextPort,end,nextPortCost));
	//						}
	//					}
	//					else{
	//						cost_to_port[nextPort]=nextPortCost;
	//						cameFrom[nextPort]=currPort;
	//						q.push(port_heuristic(nextPort,end,nextPortCost));
	//					}
	//				}
	//				else{
	//	//				std::cout << "\t[MAPPED="<< nextPort->node->idx << "]nextPort=" << nextPort->getFullName() << ",";
	//				}
	//			}
	//		}
	//
	//
	//		if(currPort!=end){
	//	//		std::cout << "LeastCostPath failed!\n";
	//			path.clear();
	//			while(currPort!=start){
	//				path.push_back(currPort);
	//				assert(cameFrom.find(currPort)!=cameFrom.end());
	//				currPort = cameFrom[currPort];
	//			}
	//			path.push_back(start);
	//			std::reverse(path.begin(),path.end());
	//			return false; //routing failure
	//		}
	//
	//		path.clear();
	//		assert(currPort==end);
	//		while(currPort!=start){
	//			path.push_back(currPort);
	//			assert(cameFrom.find(currPort)!=cameFrom.end());
	//			currPort = cameFrom[currPort];
	//		}
	//		path.push_back(start);
	//		std::reverse(path.begin(),path.end());
	//		cost=cost_to_port[end];
	//
	//	//	std::cout << "Path::";
	//	//	for(Port* p : path){
	//	//		std::cout  << p->getFullName() << "-->";
	//	//	}
	//	//	std::cout << "\n";
	//	//	std::cout << "LeastCostPath success!\n";
	//		return true;
	//
	//
}

bool CGRAXMLCompile::HeuristicMapper::dataPathCheck(DataPath *dp,
		DFGNode *node)
{

	if (dp->getMappedNode() != NULL)
	{
		return false;
	}

	PE *pe = dp->getPE();
	FU *fu = dp->getFU();
	CGRA *cgra = dp->getCGRA();
	int fanoutNode = node->children.size();

	int latency = fu->supportedOPs[node->op];
	int next_t = (pe->T + latency) % cgra->get_t_max();

	// PE *nextPE = cgra->PEArr[next_t][pe->Y][pe->X];
	PE *nextPE = cgra->getLatencyPE(pe, latency);

	FU *nextFU = static_cast<FU *>(nextPE->getSubMod(fu->getName()));
	DataPath *nextDP = static_cast<DataPath *>(nextFU->getSubMod(dp->getName()));

	Port *outputPort = nextDP->getOutPort("T");
	assert(outputPort);

	std::queue<Port *> q;
	q.push(outputPort);

	std::set<DataPath *> connectingDPs;
	std::set<Port *> alreadyVisitedPorts;
	while (!q.empty())
	{
		Port *currPort = q.front();
		q.pop();

		if (alreadyVisitedPorts.find(currPort) != alreadyVisitedPorts.end())
		{
			continue;
		}
		alreadyVisitedPorts.insert(currPort);

		if (DataPath *newDP = dynamic_cast<DataPath *>(currPort->getMod()))
		{
			if (newDP != dp)
			{
				connectingDPs.insert(newDP);
			}
		}

		for (LatPort lp : currPort->getMod()->getNextPorts(std::make_pair(currPort->getMod()->getPE()->T, currPort), this))
		{
			q.push(lp.second);
		}
		//
		//		if(currPort->getType()==OUT){
		//			for(Port* p : currPort->getMod()->getParent()->connections[currPort]){
		//				q.push(p);
		//			}
		//		}

		if (connectingDPs.size() == fanoutNode)
		{
			return true;
		}
	}
	return false;
}

void CGRAXMLCompile::HeuristicMapper::printMappingLog2()
{
	mappingLog2 << "--------------------------------------------------------\n";
	for (DFGNode &node : dfg->nodeList)
	{
		if (node.rootDP != NULL)
		{
			mappingLog2 << "node=" << node.idx << ",mapped=" << node.rootDP->getPE()->getName() << "\n";
			for (DFGNode *parent : node.parents)
			{
				if (parent->rootDP != NULL)
				{
					mappingLog2 << "\t"
							<< "parent:" << parent->idx << "\n";
					for (std::pair<Port *, int> pair : parent->routingPorts)
					{
						Port *p = pair.first;
						if (pair.second == node.idx)
						{
							mappingLog2 << "\t\t" << p->getFullName() << "\n";
						}
					}
				}
			}
		}
	}
	mappingLog2 << "--------------------------------------------------------\n";
}

bool CGRAXMLCompile::HeuristicMapper::sanityCheck()
{
	std::map<Port *, std::set<DFGNode *>> mapInfo;

	for (DFGNode *node : this->sortedNodeList)
	{
		assert(node->rootDP != NULL);
		assert(!node->routingPorts.empty());
		for (std::pair<Port *, int> pair : node->routingPorts)
		{
			mapInfo[pair.first].insert(node);
		}
	}

	for (std::pair<Port *, std::set<DFGNode *>> pair : mapInfo)
	{
		Port *p = pair.first;

		if (pair.second.size() > 1)
		{
			std::cout << p->getFullName() << ":";
			for (DFGNode *node2 : pair.second)
			{
				std::cout << node2->idx << "(" << node2->BB << "),";
			}
			std::cout << "\n";
		}
	}
	return true;
}

void CGRAXMLCompile::HeuristicMapper::removeFailedNode(std::stack<DFGNode *> &mappedNodes,
		std::stack<DFGNode *> &unmappedNodes, DFGNode *failedNode)
{

	DFGNode *fNode = failedNode;

	std::stack<DFGNode *> temp;
	DFGNode *curr = NULL;

	while (curr != fNode)
	{
		assert(!mappedNodes.empty());
		curr = mappedNodes.top();
		mappedNodes.pop();

		if (curr != fNode)
		{
			temp.push(curr);
		}
	}
	assert(curr == fNode);

	unmappedNodes.push(curr);
	while (!temp.empty())
	{
		curr = temp.top();
		temp.pop();
		mappedNodes.push(curr);
	}
}

bool CGRAXMLCompile::HeuristicMapper::checkRecParentViolation(DFGNode *node,
		LatPort nextPort)
{
	//assert(false);
	for (DFGNode *recParent : node->recParents)
	{
		assert(recParent->rootDP != NULL); //should be mapped
		DataPath *recParentDP = recParent->rootDP;
		//	assert(false);
		//	std::cout << "RecParent = " << recParent->idx << ",";
		//	std::cout << "Lat=" << recParentDP->getLat() << ",";
		//	std::cout << "NextPort Lat=" << nextPort.first << "\n";
		if (nextPort.first > recParentDP->getLat() + this->cgra->get_t_max())
		{
			return true;
		}
	}
	return false;
}

int CGRAXMLCompile::HeuristicMapper::getlatMinStarts(
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
	}

	//	assert(max!=-1);
	return max;
}

std::map<CGRAXMLCompile::DataPath *, int> CGRAXMLCompile::HeuristicMapper::getLatCandDests(
		const std::vector<DataPath *> &candidateDests, int minLat)
{

	std::map<CGRAXMLCompile::DataPath *, int> res;

	for (DataPath *dp : candidateDests)
	{
		PE *pe = dp->getPE();
		CGRA *cgra = dp->getCGRA();
		int ii = cgra->get_t_max();
		int t = pe->T;
		int minLatmodii = minLat % ii;

		int lat = minLat;
		if (minLatmodii > t)
		{
			lat += ii + (t - minLatmodii);
		}
		else
		{
			lat += t - minLatmodii;
		}

		if (lat % ii != t)
		{
			std::cout << "ii = " << ii << "\n";
			std::cout << "minLat = " << minLat << "\n";
			std::cout << "t = " << t << "\n";
			std::cout << "minLatmodii = " << minLatmodii << "\n";
			std::cout << "lat = " << lat << "\n";
		}

		assert(lat % ii == t);
		res[dp] = lat;
	}
	return res;
}

//#ifdef HIERARCHICAL
void CGRAXMLCompile::HeuristicMapper::printMappingLog3()
{

	struct util
	{
		void static repeatedPush(std::stringstream &ss, std::string pushStr, int count)
								{
			for (int i = 0; i < count; ++i)
			{
				ss << pushStr;
			}
								}
	};

	vector<string> lineHeaders;

	vector<PE *> zeroth_spatialPEs = cgra->getSpatialPEList(0);
	std::stringstream peHeader;
	std::stringstream fuHeader;
	std::stringstream dpHeader;
	for (PE *pe : zeroth_spatialPEs)
	{
		peHeader << pe->getName();

		int fuCount = 0;
		int dpCount = 0;
		for (Module *mod : pe->subModules)
		{
			if (FU *fu = dynamic_cast<FU *>(mod))
			{
				fuCount++;
				for (Module *mod : fu->subModules)
				{
					if (DataPath *dp = dynamic_cast<DataPath *>(mod))
					{
						dpCount++;
					}
				}
			}
		}

		int inputPortCount = pe->inputPorts.size();
		int outputPortCount = pe->outputPorts.size();
		int regConPortCount = pe->getRegConPorts().size()*2;
		int totalColumns = dpCount;// + inputPortCount + outputPortCount + regConPortCount;

		for (Module *mod : pe->subModules)
		{
			if (FU *fu = dynamic_cast<FU *>(mod))
			{
				fuHeader << fu->getName() << ",";
				for (Module *mod : fu->subModules)
				{
					if (DataPath *dp = dynamic_cast<DataPath *>(mod))
					{
						dpHeader << dp->getName() << ",";
					}
				}
			}
		}
		//
		//		for (Port *ip : pe->inputPorts)
		//		{
		//			dpHeader << ip->getName() << ",";
		//		}
		//
		//		for (Port *op : pe->outputPorts)
		//		{
		//			dpHeader << op->getName() << ",";
		//		}
		//
		//		for (pair<Port*,Port*> pp : pe->getRegConPorts())
		//		{
		//			Port* ri = pp.first;
		//			Port* ro = pp.second;
		//
		//			dpHeader << ri->getName() << ",";
		//			dpHeader << ro->getName() << ",";
		//		}

		util::repeatedPush(peHeader, ",", totalColumns);
		util::repeatedPush(fuHeader, ",", totalColumns - fuCount);
	}

	mappingLog3 << "," << peHeader.str() << "\n";
	mappingLog3 << "," << fuHeader.str() << "\n";
	mappingLog3 << "," << dpHeader.str() << "\n";

	std::map<int, std::vector<std::vector<std::string>>> lineMatrix;
	std::map<int, std::vector<std::string>> SchedulelineMatrix;
	std::map<std::pair<int,std::string>, int> schedule;
	int maxlat=0;

	for (int t = 0; t < cgra->get_t_max(); ++t)
	{

		//				peHeader << "PE_" << t << y << x  << ",";
		// peHeader << "X=" << x << ",";
		vector<PE *> spatialPEs = cgra->getSpatialPEList(t);
		for (PE *pe : spatialPEs)
		{
			// PE *pe = cgra->PEArr[t][y][x];

			std::stringstream peHeader;
			std::stringstream fuHeader;
			std::stringstream dpHeader;
			std::stringstream dpOp;

			peHeader << "T=" << t << "," << pe->getName();

			int fuCount = 0;
			int dpCount = 0;
			for (Module *mod : pe->subModules)
			{
				if (FU *fu = dynamic_cast<FU *>(mod))
				{
					fuCount++;
					for (Module *mod : fu->subModules)
					{
						if (DataPath *dp = dynamic_cast<DataPath *>(mod))
						{
							dpCount++;
						}
					}
				}
			}

			int inputPortCount = pe->inputPorts.size();
			int outputPortCount = pe->outputPorts.size();
			int regConPortCount = pe->getRegConPorts().size()*2;
			int totalColumns = dpCount + inputPortCount + outputPortCount + regConPortCount;

			for (Module *mod : pe->subModules)
			{
				if (FU *fu = dynamic_cast<FU *>(mod))
				{
					fuHeader << fu->getName() << ",";
					for (Module *mod : fu->subModules)
					{
						if (DataPath *dp = dynamic_cast<DataPath *>(mod))
						{
							dpHeader << dp->getName() << ",";
							if (dp->getMappedNode())
							{
								dpOp << dp->getMappedNode()->idx << ": " << dp->getLat();


								schedule[make_pair(dp->getLat(),pe->getName())]=dp->getMappedNode()->idx;
								//								std::cout << "IDYYYY:"<< dp->getLat()<<":"<<pe->getName()<<dp->getMappedNode()->idx<<","<<schedule[make_pair(dp->getLat(),pe->getName())]<<endl;
								if(dp->getLat()>maxlat){maxlat=dp->getLat();}
								//								dpOp << dp->getMappedNode()->idx << ":" << dp->getMappedNode()->op<< ":" << dp->getLat();
								//								if (dp->getMappedNode()->hasConst)
								//								{
								//									dpOp << "+C";
								//								}
								//								dpOp << "(";
								//								for (DFGNode *parent : dp->getMappedNode()->parents)
								//								{
								//									dpOp << parent->idx << "-";
								//								}
								//								dpOp << "|";
								//								for (DFGNode *child : dp->getMappedNode()->children)
								//								{
								//									dpOp << child->idx << "-";
								//								}
								//								dpOp << ")";
								dpOp << ",";
							}
							else
							{
								dpOp << "---,";
							}
						}
					}
				}
			}



			std::vector<string> lineWord;
			// lineWord.push_back(peHeader.str());
			// lineWord.push_back(fuHeader.str());
			// lineWord.push_back(dpHeader.str());
			lineWord.push_back(dpOp.str());

			lineMatrix[t].push_back(lineWord);
		}
	}

	int lineCount = 1;

	//print line matrix
	for (int t = 0; t < cgra->get_t_max(); ++t)
	{
		mappingLog3 << "T=" << t << ",";
		for (vector<string> linewords : lineMatrix[t])
		{
			for (int l = 0; l < lineCount; ++l)
			{
				mappingLog3 << linewords[l];
			}
			// mappingLog << "\n";
		}
		mappingLog3 << "\n";
	}
	mappingLog3 << "************************************\n";
	mappingLog3 << "***************Schedule:*********************\n";
	for (int l = 0; l <= maxlat;++l)//cgra->get_t_max(); ++t)
	{
		int t = l%cgra->get_t_max();
		vector<PE *> spatialPEs = cgra->getSpatialPEList(t);
		for (PE *pe : spatialPEs)
		{
			// PE *pe = cgra->PEArr[t][y][x];

			std::stringstream dpHeader;
			std::stringstream dpOp;




			if(schedule.find(make_pair(l,pe->getName()))!=schedule.end()){
				dpOp << schedule[make_pair(l,pe->getName())];
				dpOp << ",";
				//					std::cout << "IDXXXX:"<<schedule[make_pair(l,pe->getName())]<<endl;
			}else{
				dpOp << "---,";
			}


			SchedulelineMatrix[l].push_back(dpOp.str());
		}
	}



	//print line matrix
	for (int l = 0; l <= maxlat;++l)//int t = 0; t < cgra->get_t_max(); ++t)
	{
		mappingLog3 << "L=" << l << ",";

		for (std::string linewords : SchedulelineMatrix[l])
		{
			//				for (int li = 0; li < lineCount; ++li)
			//				{
			mappingLog3 << linewords;
			//				}
			// mappingLog << "\n";
		}
		mappingLog3 << "\n";
	}

}

//#endif

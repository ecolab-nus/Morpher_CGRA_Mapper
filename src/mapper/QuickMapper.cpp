//

#include <cstddef>
#include <morpher/mapper/PathFinderMapper.h>

#include <morpher/mapper/HeuristicMapper.h>
#include <morpher/mapper/QuickMapper.h>
#include <queue>
#include <assert.h>
#include <algorithm> // std::reverse
#include <morpher/arch/DataPath.h>
#include <morpher/arch/FU.h>

#include <stack>
#include <functional>
#include <set>
#include <iostream>
#include <sstream>
#include <unordered_set>
#include <chrono>


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





bool CGRAXMLCompile::QuickMapper::QuickMap(CGRA *cgra, DFG *dfg)
{
	std::cout<<"#####################  Quick Mapper: \n";
	std::stack<DFGNode *> mappedNodes;
	std::stack<DFGNode *> unmappedNodes;
	std::map<DFGNode *, std::priority_queue<dest_with_cost>> estimatedRouteInfo;

	int backTrackCredits = this->backTrackLimit;

	//Disable mutex paths to test pathfinder
	this->enableMutexPaths = true;

	this->cgra = cgra;
	this->dfg = dfg;

	Check_DFG_CGRA_Compatibility();

	if(cgra->is_spm_modelled){
		UpdateVariableBaseAddr();
	}
	//Testing 1 2 3
	//getLongestDFGPath(dfg->findNode(1093),dfg->findNode(82));

	//	SortSCCDFG();
	//	SortTopoGraphicalDFG();
	sortBackEdgePriorityASAP();
	//	sortBackEdgePriorityALAP();

	
	

	bool mapSuccess = false;


	for (int i = 0; i < this->maxIter; ++i)
	{




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

		std::cout << "Current target II = " << cgra->get_t_max() <<", Iteration = " << i << "\n";

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
			MapHeader << ",MaxHops=" << this->cgra->max_hops;

			MapHeader << ",BB = " << node->BB;
			MapHeader << ",mutexPathEn = " << this->enableMutexPaths;
			MapHeader << ",Iter = " << i;
			MapHeader << "\n";


			bool isEstRouteSucc = false;

			//fill the routing information
			if (estimatedRouteInfo.find(node) == estimatedRouteInfo.end())
			{
				//the routes are not estimated.
				std::priority_queue<dest_with_cost> estimatedRoutes;
				DFGNode *failedNode;
				isEstRouteSucc = quickEstimateRouting(node, estimatedRoutes, &failedNode);

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
							mappingLog4.close();
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

						prevNode->CarefulClear(this->dfg);
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
						return false;
					}
				}
				estimatedRouteInfo[node] = estimatedRoutes;
			}

			bool isRouteSucc = false;
			DFGNode *failedNode = NULL;

			LOG(MAPPING) << "estimatedRouteInfo[node].size = " << estimatedRouteInfo[node].size() << "\n";
			mappingLog << "estimatedRouteInfo[node].size = " << estimatedRouteInfo[node].size() << "\n";
			if (!estimatedRouteInfo[node].empty())
			{
				isRouteSucc = Route(node, estimatedRouteInfo[node], &failedNode);
				if (!isRouteSucc)
					std::cout << "Route Failed!\n";
			}
			else
			{
				if (mappedNodes.empty())
				{
					mappingLog << "Map Failed!.\n";
					std::cout << "Map Failed!.\n";
					return false;
				}
			}

			if (!isRouteSucc)
			{
				if (mappedNodes.empty())
				{
					std::cout << "Map Failed!.\n";
					return false;
				}

				if (enableBackTracking)
				{
					if (backTrackCredits == 0)
					{
						std::cout << "Map Failed!.\n";
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
					std::cout << "Map Failed!.\n";
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
		mappingLog4.close();
	}

	//	congestionInfoFile.close();

	if (mapSuccess)
	{

		// by Yujie
		// cgra->PrintMappedJSON(fNameLog1 + cgra->getCGRAName() + "mapping.json");

		//std::cout << "Map Success!.\n";

		LOG(MAPPING) << "Checking conflict compatibility!\n";

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
		std::cout << "Map Failed!.\n";
		return false;
	}
}

bool CGRAXMLCompile::QuickMapper::QuickRoute(DFGNode *node,
											 std::priority_queue<dest_with_cost> &estimatedRoutes,
											 DFGNode **failedNode)
{

	LOG(ROUTE) << "Route begin...\n";

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
			LOG(ROUTE) << "currDest is not NULL \n";
			LOG(ROUTE) << "currDP:" << currDest.dest->getName() << ",currPE:" << currDest.dest->getPE()->getName() << "\n";
			LOG(ROUTE) << "currNode:" << currDest.dest->getMappedNode()->idx << "\n";
		}
		assert(currDest.dest->getMappedNode() == NULL);
		LOG(ROUTE) << "alreadyMappedChilds = " << currDest.alreadyMappedChilds.size() << "\n";

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
				if (quickLeastCostPathAstar(p, dest_child_with_cost_ins.childDest, dest_child_with_cost_ins.childDP, pathTmp, cost, node, mutexPathsTmp, dest_child_with_cost_ins.child))
				{
					q.push(cand_src_with_cost(p, dest_child_with_cost_ins.childDest, cost, path_toStr(pathTmp)));
					pathTmp.clear();
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
				alreadMappedChildRouteSucc = quickLeastCostPathAstar(head.src, dest, dest_child_with_cost_ins.childDP, path, cost, node, mutexPaths, dest_child_with_cost_ins.child);
				if (alreadMappedChildRouteSucc)
				{
					assignPath(node, dest_child_with_cost_ins.child, path);
					mappedChildPaths[dest_child_with_cost_ins.child] = path;
					mappedChildMutexPaths[dest_child_with_cost_ins.child] = mutexPaths;
					LOG(MAPPING) << "Route success :: from=" << src.second->getFullName() << "--> to=" << dest.second->getFullName() << "|node=" << node->idx << "\n";
					break;
				}
				else
				{
					LOG(MAPPING) << "Route Failed :: from=" << src.second->getFullName() << "--> to=" << dest.second->getFullName() << "\n";
					for (LatPort p : path)
					{
						if (p.second->getMod()->getPE())
						{
							LOG(ROUTE) << p.second->getMod()->getPE()->getName() << "-->";
						}
					}
					LOG(ROUTE) << "\n";

					for (LatPort p : path)
					{
						LOG(ROUTE) << p.second->getFullName() << "\n";
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
				LOG(ROUTE) << "to:" << destIdx << "," << p->getFullName() << "\n";
			}
		}

		if (!alreadMappedChildRouteSucc)
		{
			node->clear(this->dfg);
			continue; //try the next dest
		}
		else
		{
			LOG(ROUTE) << "Already Mapped child Routes....\n";
			for (std::pair<DFGNode *, std::vector<LatPort>> pair : mappedChildPaths)
			{
				DFGNode *child = pair.first;
				for (LatPort lp : pair.second)
				{
					Port *p = lp.second;
					LOG(ROUTE) << "to:" << child->idx << " :: ";
					LOG(ROUTE) << p->getFullName();
					if (mappedChildMutexPaths[child].find(p) != mappedChildMutexPaths[child].end())
					{
						LOG(ROUTE) << "|mutex(";
						for (DFGNode *mutexnode : mappedChildMutexPaths[child][p])
						{
							LOG(ROUTE) << mutexnode->idx << ",";
						}
						LOG(ROUTE) << ")";
					}
					LOG(ROUTE) << "\n";
				}
				LOG(ROUTE) << "\n";
			}
			LOG(ROUTE) << "\n";
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
				succ = quickLeastCostPathAstar(src, dest, currDest.dest, path, cost, parent, mutexPath, node);
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
					//						LOG(ROUTE) << p->getFullName() << ",\n";
					//					}
					//					LOG(ROUTE) << "\n";
					break;
				}
				else
				{
					addedRoutingParentPorts = 0;
					node->CarefulClear(this->dfg);
					LOG(MAPPING) << "Route Failed :: from=" << src.second->getFullName() << "--> to=" << dest.second->getFullName() << "\n";
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
			LOG(MAPPING) << "node=" << node->idx << ",op=" << node->op << " is mapped to " << currDest.dest->getPE()->getName() << ",lat=" << currDest.destLat << "\n";
			LOG(MAPPING) << "routing info ::\n";
			for (DFGNode *parent : node->parents)
			{
				LOG(MAPPING) << "parent routing port size = " << parent->routingPorts.size() << "\n";
				int prev_lat = -1;
				for (std::pair<Port *, int> pair : parent->routingPorts)
				{
					Port *p = pair.first;
					//					if(node.routingPortDestMap[p]==&node){
					LOG(MAPPING) << "fr:" << parent->idx << " :: "
					 			<< ",dest=" << pair.second << " :: "
								<< p->getFullName()
								<< ",lat=" << p->getLat();

					if (mappedParentMutexPaths[parent].find(p) != mappedParentMutexPaths[parent].end())
					{
						LOG(MAPPING) << "|mutex(";
						for (DFGNode *mutexnode : mappedParentMutexPaths[parent][p])
						{
							LOG(MAPPING) << mutexnode->idx << ",";
						}
						LOG(MAPPING) << ")";
					}
					//					}
					if (prev_lat != -1)
					{
						//							assert(p->getLat() - prev_lat <= 1);
					}
					prev_lat = p->getLat();
				}
			}
			LOG(ROUTE) << "routing info done.\n";
			currDest.dest->assignNode(node, currDest.destLat, this->dfg);
			mappingLog4 << node->idx << "," << currDest.dest->getPE()->X << ","<< currDest.dest->getPE()->Y << "," << currDest.destLat << "\n";
			LOG(ROUTE) << "mappingLog4=" << node->idx << "," << currDest.dest->getPE()->X << ","<< currDest.dest->getPE()->Y << "," << currDest.destLat << "\n";
			node->rootDP = currDest.dest;
			break;
		}
		node->CarefulClear(this->dfg);
	}

	if (routeSucc)
	{
		LOG(MAPPING) << "Route success...\n";

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
			LOG(ROUTE) << "parentRoutingPortCountEnd=" << parentRoutingPortCountEnd << "\n";
			LOG(ROUTE) << "addedRoutingParentPorts=" << addedRoutingParentPorts << "\n";
			LOG(ROUTE) << "parentRoutingPortCount=" << parentRoutingPortCount << "\n";
		}

		//		assert(parentRoutingPortCountEnd==parentRoutingPortCount+addedRoutingParentPorts);
		return true;
	}
	else
	{
		currDest.dest->assignNode(node, currDest.destLat, this->dfg);
		node->rootDP = currDest.dest;
		node->CarefulClear(this->dfg);
		LOG(ROUTE) << "Route failed...\n";

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
			LOG(ROUTE) << "parentRoutingPortCountEnd=" << parentRoutingPortCountEnd << "\n";
			LOG(ROUTE) << "addedRoutingParentPorts=" << addedRoutingParentPorts << "\n";
			LOG(ROUTE) << "parentRoutingPortCount=" << parentRoutingPortCount << "\n";
		}
		//		assert(parentRoutingPortCountEnd==parentRoutingPortCount);
		assert(*failedNode != NULL);
		return false;
	}
}


bool CGRAXMLCompile::QuickMapper::quickEstimateRouting(DFGNode *node,
													   std::priority_queue<dest_with_cost> &estimatedRoutes,
													   DFGNode **failedNode)
{

	std::map<DFGNode *, std::vector<Port *>> possibleStarts;
	std::map<DFGNode *, Port *> alreadyMappedChildPorts;

	bool detailedDebug = false;
	// if(node->idx==1)detailedDebug=true;

	LOG(ROUTE) << "Quick EstimateEouting begin...\n";

	for (DFGNode *parent : node->parents)
	{
		LOG(ROUTE) << "parent = " << parent->idx << "\n";
		if (parent->rootDP != NULL)
		{ //already mapped
			LOG(ROUTE) << "add parent to starts = " << parent->idx << "\n";
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
			LOG(ROUTE)<< "child=" << child->idx << ",childOpType=" << node->childrenOPType[child] << "\n";
			assert(child->rootDP->getLat() != -1);
			if (node->childrenOPType[child] == "PS")
			{
				LOG(ROUTE)<< "Skipping.....\n";
				continue;
			}
			assert(child->rootDP->getInPort(node->childrenOPType[child]));
			alreadyMappedChildPorts[child] = child->rootDP->getInPort(node->childrenOPType[child]);

			int ii = child->rootDP->getCGRA()->get_t_max();
			assert(child->rootDP->getLat() != -1);

			//Previsouly, this assumes that mapped children are for next iteration.
			// For SA, the mapped children might not be so. 
			if(node->childNextIter[child] == 1){
				alreadyMappedChildPorts[child]->setLat(child->rootDP->getLat() + ii);
			}else if (node->childNextIter[child] == 0){
				if(mapping_method_name.find("PathFinder") != std::string::npos){
					assert(false);
				}
				alreadyMappedChildPorts[child]->setLat(child->rootDP->getLat() );
			}else {
				assert(false);
			}
			
		}
		else if(child->idx == node->idx){
			//adding a placeholder as this will be modified according to the destination in consideration.
			alreadyMappedChildPorts[child] = NULL;
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

							bool is_mem_op = node->op.find("LOAD") != string::npos || node->op.find("STORE") != string::npos;
							if(cgra->is_spm_modelled){
								if(!node->base_pointer_name.empty() && is_mem_op){
									//base pointer name is not empty
									if(dp->accesible_memvars.find(node->base_pointer_name) == dp->accesible_memvars.end()){
										//this dp does not support the variable
										cout << "memvar=" << node->base_pointer_name <<  " is not supported in " << dp->getFullName() << "\n";
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
			}
		}
	}
	
	// std::cout<< "Candidate Dests = " << candidateDests.size() << "\n";
	LOG(ROUTE)<< "Candidate Dests = " << candidateDests.size() << "\n";
	if (candidateDests.empty())
		return false;
	//	assert(candidateDests.size()!=0);
	//	node->blacklistDest.clear();

	//	int minLat = getlatMinStarts(possibleStarts);
	LOG(ROUTE)<< "getlatMinStartsPHI\n";
	int minLat = getlatMinStartsPHI(node, possibleStarts);
	LOG(ROUTE)<< "getLatCandDests\n";
	std::map<DataPath *, int> minLatDests = getLatCandDests(candidateDests, minLat);
	bool changed = false;
	
	LOG(ROUTE)<< "Candidate Dests = " << candidateDests.size() << "\n";
	int ii = this->cgra->get_t_max();

	int minLatSucc = 1000000000;
	std::priority_queue<dest_with_cost> estimatedRoutesTemp;

	int allowed_time_steps_for_connection = 5;
	int iterations = allowed_time_steps_for_connection;

	//Route Estimation
	for (int i = 0; i < iterations; ++i)
	{
		if(estimatedRoutesTemp.size() > 5){
			break;
		}
		bool pathFromParentExist = false;
		bool pathExistMappedChild = false;

		for (DataPath *dest : candidateDests)
		{
			int minLatDestVal_prime = minLatDests[dest] + ii * i;
			std::stringstream output_stream;
			output_stream << "Candidate Dest =" ;
			output_stream << dest->getPE()->getName() << ".";
			output_stream << dest->getFU()->getName() << ".";
			output_stream<< dest->getName() << "\n";
			LOG(ROUTE)<<output_stream.str();

			//		std::map<DFGNode*,std::priority_queue<cand_src_with_cost>> parentStartLocs;
			std::priority_queue<parent_cand_src_with_cost> parentStartLocs;
			int minLatDestVal = minLatDestVal_prime;
			pathFromParentExist = true;
			for (std::pair<DFGNode *, std::vector<Port *>> pair : possibleStarts)
			{
				LOG(ROUTE)<<"estimate parent:"<<pair.first->idx<<" port size:"<<pair.second.size();
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
					LOG(ROUTE)<< "par Estimating Path" << startCand->getFullName() << "," << startCand->getLat() << ","
								  << "--->" << destPort->getFullName() << "," << minLatDestVal << "," << ",parent_node = " << parent->idx
								  << "\n";

					LatPort startCandLat = std::make_pair(startCand->getLat(), startCand);
					assert(startCand->getLat() != -1);
					LatPort destPortLat = std::make_pair(minLatDestVal, destPort);

					LOG(ROUTE) << "lat = " << destPortLat.first << ",PE=" << destPort->getMod()->getPE()->getName() << ",t=" <<  destPort->getMod()->getPE()->T << "\n";
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
							LOG(ROUTE)<< "Cannot exit from :" << destPortLat.second->getFullName() << "\n";
						}
					}
					
					
					auto start = std::chrono::steady_clock::now();
					pathExist = pathExist & quickLeastCostPathAstar(startCandLat, destPortLat, dest, path, cost, parent, mutexPaths, node);
					
					{ // test quick estimation
						bool test_quick_estimation = false;
						if (test_quick_estimation){
							bool truepathExistMappedChild  = pathExist;
							auto end = std::chrono::steady_clock::now();
							float quick_time=  chrono::duration_cast<chrono::microseconds>(end - start).count();;
							std::vector<LatPort> truepath;
							start = std::chrono::steady_clock::now();
							truepathExistMappedChild =truepathExistMappedChild & LeastCostPathAstar(startCandLat, destPortLat, dest, truepath, cost, parent, mutexPaths, node);
							end = std::chrono::steady_clock::now();
							float true_time= 	chrono::duration_cast<chrono::microseconds>(end - start).count();

								std::cout<<true_time<<" "<< quick_time<<" "<<truepath.size()<<" "<<path.size();
								std::cout<<"\n";
						}
					}
					if (!pathExist)
					{
						LOG(ROUTE)<< "par Estimate Path Failed :: " << startCand->getFullName() << "--->" << destPort->getFullName() << "\n";
						path.clear();
						continue;
					}
					// std::stringstream output_stream;
					// output_stream<<"____________________";
					// output_stream<< parent->idx<< "," << startCand->getFullName() << "," << startCandLat.first << " to "
					// << node->idx << ", node" << destPort->getFullName() << "," << destPortLat.first << "\n path: ";
					// for(auto p: path){
					// 	output_stream<<p.first<<","<<p.second->getFullName()<<" -> ";
					// }
					// output_stream<<"\n";
					// std::cout<<output_stream.str();

					cost += dpPenaltyMap[dest];
					res.push(cand_src_with_cost(startCandLat, destPortLat, cost, path_toStr(path)));
					path.clear();
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
					LOG(ROUTE) << "setting latency = " << minLatDestVal + ii << "\n";
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
				LOG(ROUTE)<< "already child Estimating Path" << destPort->getFullName() << "," << minLatDestVal + latency << ","
							  << "--->" << childDestPort->getFullName() << "," << childDestPort->getLat() << "," << "exist_child = " << child->idx  
							  << "\n";
				LOG(ROUTE)<< "lat = " << childDestPort->getLat() << ",PE=" << childDestPort->getMod()->getPE()->getName() << ",t=" << childDestPort->getMod()->getPE()->T << "\n";

				LatPort childDestPortLat = std::make_pair(childDestPort->getLat(), childDestPort);
				assert(childDestPort->getLat() != -1);
				LatPort destPortLat = std::make_pair(minLatDestVal + latency, destPort);
				pathExistMappedChild = pathExistMappedChild & quickLeastCostPathAstar(destPortLat, childDestPortLat, childDP, path, cost, node, mutexPaths, child);
				
				{ // test quick estimation
					bool test_quick_estimation = false;
					if(test_quick_estimation){
						bool truepathExistMappedChild  = pathExistMappedChild;

						std::vector<LatPort> truepath;
						truepathExistMappedChild= truepathExistMappedChild& quickLeastCostPathAstar(destPortLat, childDestPortLat, childDP, truepath, cost, node, mutexPaths, child);
						if(truepathExistMappedChild != pathExistMappedChild && truepathExistMappedChild){
							LOG(SA)<<"true path:"<<path_toStr(truepath);
							LOG(SA)<<"quick path:"<<path_toStr(path);
							LOG(SA)<<"";
						}
					}
				}
				if (!pathExistMappedChild)
				{
					*failedNode = child;
					break;
				}

				// std::stringstream output_stream;
				// output_stream<<"____________________";
				// output_stream<< node->idx<< "," << destPort->getFullName() << "," << destPortLat.first << " to "
				// << child->idx << ", node" << childDestPort->getFullName() << "," << childDestPortLat.first << "\n path: ";
				// for(auto p: path){
				// 	output_stream<<p.first<<","<<p.second->getFullName()<<" -> ";
				// }
				// output_stream<<"\n";
				// std::cout<<output_stream.str();

				dest_child_with_cost dcwc(child,childDP, childDestPortLat, destPortLat, cost);
				alreadyMappedChilds.push(dcwc);
			}
			if (!pathExistMappedChild)
			{
				if (detailedDebug)
					LOG(ROUTE)<< "already child Estimating Path Failed!\n";
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


bool CGRAXMLCompile::QuickMapper::quickLeastCostPathAstar(LatPort start,
														  LatPort end, DataPath *endDP, std::vector<LatPort> &path, int &cost, DFGNode *node,
														  std::map<Port *, std::set<DFGNode *>> &mutexPaths, DFGNode *currNode)
{
	auto is_port_in_rectangle_range =  [ &]( Port *port){
		int port_x = port->getPE()->getPosition_X();
		int port_y = port->getPE()->getPosition_Y();
		bool in_range = true;

		//release the constraint by +1/-1 as some memory PE might cause a problem
		int max_x = std::max(start.second->getPE()->getPosition_X(), end.second->getPE()->getPosition_X()) ;
		int min_x = std::min(start.second->getPE()->getPosition_X(), end.second->getPE()->getPosition_X())-1 ;
		int max_y = std::max(start.second->getPE()->getPosition_Y(), end.second->getPE()->getPosition_Y()) ;
		int min_y = std::min(start.second->getPE()->getPosition_Y(), end.second->getPE()->getPosition_Y())-1 ;

		if(port_x > max_x || port_x < min_x){
			in_range = false;
		}

		if(port_y > max_y || port_y < min_y){
			in_range = false;
		}
		return in_range;
	};


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
	auto time_start = std::chrono::steady_clock::now();
	int connected_num = 0;
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
			if (currPort.second == end.second)
			{
				finalPath = *curr.pathVec;
				break;
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
		// std::vector<LatPort> nextPorts = currPort.second->getMod()->getNextPorts(currPort, this);
		std::vector<LatPort> nextPorts = currPort.second->getMod()->getNextPortsForQuickRoute(currPort, start.second, end.second , end.first, this);
		//		std::cout << "nextPorts size = " << nextPorts.size() << "\n";
		int q_len = q.size();
		// if(astar_search_only_in_retangle_area){
		// 	std::cout<<"test rectangle: start:"<<start.second->getFullName()<<" end:"<<end.second->getFullName()<<"\n";
		// }
		for (LatPort nextLatPort : nextPorts)
		{
			Port *nextPort = nextLatPort.second;
			// if(astar_search_only_in_retangle_area){
			// 	if(!is_port_in_rectangle_range(nextPort)){
			// 		// std::cout<<"not in rectangle:"<<nextPort->getFullName()<<"\n";
			// 		continue;
			// 	}
			// }
			if (nextLatPort.first > end.first)
				continue; //continue if the next port has higher latency

			if((nextLatPort.second->getNode() == node))
			{
				if(nextLatPort.first != nextLatPort.second->getLat())
					continue;

			}// add code from Thilini

			// comment this for QuickEstimation assert(nextLatPort.first - currPort.first <= 1);


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
				// do not think need to check this one for quick estimation
				// for (Port *cp : nextPort->getMod()->getConflictPorts(nextPort))
				// {
				// 	if (currPath->find(cp) != currPath->end())
				// 	{
				// 		continue;
				// 	}
				// }
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

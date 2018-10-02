/*
 * dfgnode.cpp
 *
 *  Created on: 20 Feb 2018
 *      Author: manupa
 */

#include "DFGNode.h"
#include "Port.h"
#include <algorithm>
#include <assert.h>
#include <iostream>
#include "DataPath.h"
#include "CGRA.h"
#include "DFG.h"

namespace CGRAXMLCompile {

DFGNode::DFGNode() {
	// TODO Auto-generated constructor stub

}

} /* namespace CGRAXMLCompile */


void CGRAXMLCompile::DFGNode::clear(DFG* dfg) {
	if(rootDP!=NULL){
		rootDP->getOutputDP()->getOutPort("T")->clear();

		rootDP->clear();

		CGRA* cgra = rootDP->getCGRA();
		FU* fu = rootDP->getFU();
		if(fu->supportedOPs.find("LOAD")!=fu->supportedOPs.end()){
			cgra->freeMemNodes++;
			cgra->freeMemNodeSet.insert(rootDP);
		}

		if(this->isMemOp()){
			dfg->unmappedMemOps++;
			dfg->unmappedMemOpSet.insert(this);
		}

		rootDP=NULL;
	}

	for(std::pair<Port*,int> pair : routingPorts){
		Port* p = pair.first;
		p->clear();
	}

	this->routingPorts.clear();


	for(DFGNode* parent : parents){
		std::set<std::pair<Port*,int>> delPorts;
		for(std::pair<Port*,int> pair : parent->routingPorts){
			Port* p = pair.first;
			int destIdx = pair.second;
//			assert(parent->routingPortDestMap.find(p)!=parent->routingPortDestMap.end());
//			int dest_idx = routingPortDestMap[p];
//			assert(dest!=NULL);

			if(parent->rootDP->getOutputDP()->getOutPort("T") == p) continue;

			if(destIdx==this->idx){
				p->clear();
				delPorts.insert(pair);
			}

		}
		std::cout << "delPorts.size = " << delPorts.size() << "\n";
		std::cout << "parentRoutingPort size(before) = " << parent->routingPorts.size() << "\n";
		for(std::pair<Port*,int> pair : delPorts){
			parent->routingPorts.erase(std::find(parent->routingPorts.begin(),parent->routingPorts.end(),pair));
		}
		std::cout << "parentRoutingPort size(after) = " << parent->routingPorts.size() << "\n";
	}


}

std::string CGRAXMLCompile::DFGNode::getOPtype(DFGNode* child) {
	assert(std::find(children.begin(),children.end(),child)!=children.end());
	assert(childrenOPType.find(child)!=childrenOPType.end());
	return childrenOPType[child];
}

bool CGRAXMLCompile::DFGNode::isMemOp() {
	if(this->op.compare("LOAD")==0){
		return true;
	}
	else if(this->op.compare("LOADH")==0){
		return true;
	}
	else if(this->op.compare("LOADB")==0){
		return true;
	}
	else if(this->op.compare("STORE")==0){
		return true;
	}
	else if(this->op.compare("STOREH")==0){
		return true;
	}
	else if(this->op.compare("STOREB")==0){
		return true;
	}
	else{
		return false;
	}
}

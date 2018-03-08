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
#include "DataPath.h"

namespace CGRAXMLCompile {

DFGNode::DFGNode() {
	// TODO Auto-generated constructor stub

}

} /* namespace CGRAXMLCompile */


void CGRAXMLCompile::DFGNode::clear() {
	rootDP->clear();
	rootDP=NULL;
	for(Port* P : routingPorts){
		P->clear();
	}

	for(DFGNode* parent : parents){
		for(Port* p : parent->routingPorts){
			assert(parent->routingPortDestMap.find(p)!=parent->routingPortDestMap.end());
			DFGNode* dest = routingPortDestMap[p];

			if(dest==this){
				p->clear();
			}
		}
	}

}

std::string CGRAXMLCompile::DFGNode::getOPtype(DFGNode* child) {
	assert(std::find(children.begin(),children.end(),child)!=children.end());
	assert(childrenOPType.find(child)!=childrenOPType.end());
	return childrenOPType[child];
}

/*
 * Port.cpp
 *
 *  Created on: 26 Feb 2018
 *      Author: manupa
 */

#include "Port.h"
#include "PE.h"
#include "Module.h"
#include "CGRA.h"
#include <stack>

namespace CGRAXMLCompile {

Port::Port(std::string name, PortType pType, Module* mod) {
	// TODO Auto-generated constructor stub
	this->name = name;
	this->pType = pType;
	this->mod = mod;
}

} /* namespace CGRAXMLCompile */

CGRAXMLCompile::PE* CGRAXMLCompile::Port::findParentPE() {

	Module* m = mod;

	while(m){
		if(PE* ret = dynamic_cast<PE*>(m)){
			return ret;
		}
		m = m->getParent();
	}
	return NULL;
}

std::string CGRAXMLCompile::Port::getFullName() {
	Module* mod = this->mod;
	std::stack<std::string> fullNameSt;

	while(mod){
		fullNameSt.push(mod->getName());
		mod = mod->getParent();
	}

	std::string fullName;
	while(!fullNameSt.empty()){
		fullName = fullName + fullNameSt.top() + ".";
		fullNameSt.pop();
	}
	fullName = fullName + name;
	return fullName;
}

void CGRAXMLCompile::Port::increastCongCost() {
	if(history_cost == 0){
		history_cost = INIT_CONG_COST/10;
	}
	else{
		history_cost = history_cost + history_cost/10;
	}
}

int CGRAXMLCompile::Port::getCongCost() {
	int cost = base_cost*number_signals + history_cost*(number_signals+1);
	return cost;
}


void CGRAXMLCompile::Port::clear() {
	node=NULL;
	if(node!=NULL){
		(*this->mod->getCGRA()->getCongestedPortPtr())[this].erase(node);
	}
	number_signals=0;
}

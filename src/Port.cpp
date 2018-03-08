/*
 * Port.cpp
 *
 *  Created on: 26 Feb 2018
 *      Author: manupa
 */

#include "Port.h"
#include "PE.h"
#include "Module.h"
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

/*
 * Module.cpp
 *
 *  Created on: 26 Feb 2018
 *      Author: manupa
 */

#include "Module.h"
#include "CGRA.h"
#include "FU.h"
#include <assert.h>

namespace CGRAXMLCompile {

Module::Module(const Module* Parent, std::string name) {
	// TODO Auto-generated constructor stub
	this->Parent=Parent;
	this->name=name;
}

Module::~Module(){
	for(Module* M : subModules){
		delete M;
	}
}

FU* Module::getFU() {
	Module* mod = this->getParent();
	while(mod){
		if(FU* fu = dynamic_cast<FU*>(mod)){
			return fu;
		}
		mod = mod->getParent();
	}
	return NULL;
}

std::vector<Port*> Module::getNextPorts(Port* currPort, HeuristicMapper* hm) {
	std::vector<Port*> nextPorts;

	for(Port* p : connectedTo[currPort]){
		bool conflicted=false;
		for(Port* conflict_port : conflictPorts[p]){
			if(conflict_port->getNode()!=NULL){
				conflicted=true;
				break;
			}
		}
		if(!conflicted){
			nextPorts.push_back(p);
		}
	}

	if(currPort->getType()==OUT){
		if(getParent()){
			for(Port* p : currPort->getMod()->getParent()->connectedTo[currPort]){
//					std::cout << currPort->getMod()->getParent()->getName() << "***************\n";
//				nextPorts.push_back(p);

				bool conflicted=false;
				for(Port* conflict_port : getParent()->getConflictPorts(currPort)){
					if(conflict_port->getNode()!=NULL){
						conflicted=true;
						break;
					}
				}
				if(!conflicted){
					nextPorts.push_back(p);
				}

			}
		}
	}
	return nextPorts;
}

std::vector<Port*> Module::getConflictPorts(Port* currPort) {
	return conflictPorts[currPort];
}

std::vector<Port*> Module::getFromPorts(Port* currPort, HeuristicMapper* hm) {
	std::vector<Port*> fromPorts;

	for(Port* p : connectedFrom[currPort]){
		bool conflicted=false;
		for(Port* conflict_port : conflictPorts[p]){
			if(conflict_port->getNode()!=NULL){
				conflicted=true;
				break;
			}
		}
		if(!conflicted){
			fromPorts.push_back(p);
		}
	}

	if(currPort->getType()==IN){
		if(getParent()){
			for(Port* p : currPort->getMod()->getParent()->connectedFrom[currPort]){
//					std::cout << currPort->getMod()->getParent()->getName() << "***************\n";
//				nextPorts.push_back(p);

				bool conflicted=false;
				for(Port* conflict_port : getParent()->getConflictPorts(currPort)){
					if(conflict_port->getNode()!=NULL){
						conflicted=true;
						break;
					}
				}
				if(!conflicted){
					fromPorts.push_back(p);
				}

			}
		}
	}

	return fromPorts;
}

void Module::insertConnection(Port* src, Port* dest) {
	connectedTo[src].push_back(dest);
	connectedFrom[dest].push_back(src);
}

} /* namespace CGRAXMLCompile */

CGRAXMLCompile::Port* CGRAXMLCompile::Module::getInPort(std::string Pname) {
	for(Port &p : inputPorts){
		if(p.getName().compare(Pname)==0){
			return &p;
		}
	}
}

CGRAXMLCompile::Port* CGRAXMLCompile::Module::getOutPort(std::string Pname) {
	for(Port &p : outputPorts){
		if(p.getName().compare(Pname)==0){
			return &p;
		}
	}
}

CGRAXMLCompile::Port* CGRAXMLCompile::Module::getInternalPort(std::string Pname) {
	for(Port &p : internalPorts){
		if(p.getName().compare(Pname)==0){
			return &p;
		}
	}
}

CGRAXMLCompile::Module* CGRAXMLCompile::Module::getSubMod(std::string Mname) {
	for(Module* m : subModules){
		if(m->getName().compare(Mname)==0){\
			return m;
		}
	}
}

CGRAXMLCompile::CGRA* CGRAXMLCompile::Module::getCGRA() {
	Module* mod = this->getParent();
	while(true){
		if(CGRA* cgra = dynamic_cast<CGRA*>(mod)){
			return cgra;
		}
		mod = mod->getParent();
		if(!mod){
			assert(false);
		}
	}
}

CGRAXMLCompile::PE* CGRAXMLCompile::Module::getPE() {
	Module* mod = this->getParent();
	while(mod){
		if(PE* pe = dynamic_cast<PE*>(mod)){
			return pe;
		}
		mod = mod->getParent();
	}
	return NULL;
}

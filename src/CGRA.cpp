/*
 * cgra.cpp
 *
 *  Created on: 20 Feb 2018
 *      Author: manupa
 */

#include "CGRA.h"
#include "PE.h"
#include "Port.h"
#include <iostream>

namespace CGRAXMLCompile {

//CGRA::CGRA() {
//	// TODO Auto-generated constructor stub
//
//}

} /* namespace CGRAXMLCompile */

void CGRAXMLCompile::CGRA::createGenericCGRA(int x_max, int y_max, int t_max, std::string peType, int numberofDPs) {

	this->x_max=x_max;
	this->y_max=y_max;
	this->t_max=t_max;

	this->minLatBetweenPEs = 1;
	if( peType.find("HyCUBE") != std::string::npos || peType.find("HYCUBE") != std::string::npos ){
		this->minLatBetweenPEs = 0;
	}

	for (int t = 0; t < t_max; ++t) {
		for (int y = 0; y < y_max; ++y) {
			for (int x = 0; x < x_max; ++x) {
				PE* newPE;
				std::string PE_name = "PE_" + std::to_string(t) + "," + std::to_string(y) + "," + std::to_string(x);

//				bool isCorner = (x==0)&&(y==0);
//				isCorner = isCorner | ((x==0)&&(y==y_max-1));
//				isCorner = isCorner | ((x==x_max-1)&&(y==0));
//				isCorner = isCorner | ((x==x_max-1)&&(y==y_max-1));

				if(x==0){
					newPE = new PE(this,PE_name,x,y,t,peType,true,numberofDPs);
				}
				else{
					newPE = new PE(this,PE_name,x,y,t,peType,false,numberofDPs);
				}
				subModules.push_back(newPE);
				PEArr[t][y][x]=newPE;
			}
		}
	}


	//create connections
	for (int t = 0; t < t_max; ++t) {
		for (int y = 0; y < y_max; ++y) {
			for (int x = 0; x < x_max; ++x) {

				PE* currPE = PEArr[t][y][x];
				if(x-1 >= 0){
					Port* currPE_WestO = currPE->getOutPort("WEST_O");
					PE* westPE = PEArr[t][y][x-1];
					Port* westPE_EastI = westPE->getInPort("EAST_I");
//					connectedTo[currPE_WestO].push_back(westPE_EastI);
					insertConnection(currPE_WestO,westPE_EastI);
				}

				if(x+1 < x_max){
					Port* currPE_EastO = currPE->getOutPort("EAST_O");
					PE* eastPE = PEArr[t][y][x+1];
					Port* eastPE_WestI = eastPE->getInPort("WEST_I");
//					connectedTo[currPE_EastO].push_back(eastPE_WestI);
					insertConnection(currPE_EastO,eastPE_WestI);
				}

				if(y-1 >= 0){
					Port* currPE_NorthO = currPE->getOutPort("NORTH_O");
					PE* northPE = PEArr[t][y-1][x];
					Port* northPE_SouthI = northPE->getInPort("SOUTH_I");
//					connectedTo[currPE_NorthO].push_back(northPE_SouthI);
					insertConnection(currPE_NorthO,northPE_SouthI);
				}

				if(y+1 < y_max){
					Port* currPE_SouthO = currPE->getOutPort("SOUTH_O");
					PE* southPE = PEArr[t][y+1][x];
					Port* southPE_NorthI = southPE->getInPort("NORTH_I");
//					connectedTo[currPE_SouthO].push_back(southPE_NorthI);
					insertConnection(currPE_SouthO,southPE_NorthI);
				}

				int t_next = (t+1)%t_max;
				PE* nextCyclePE = PEArr[t_next][y][x];
				for(RegFile* RF : currPE->allRegs){
					for (int i = 0; i < RF->get_nRegs(); ++i) {
						Port* reg_i = nextCyclePE->getInPort(RF->getName() + "_REG_I" + std::to_string(i));
						Port* reg_o = currPE->getOutPort(RF->getName() + "_REG_O" + std::to_string(i));

//						connectedTo[reg_o].push_back(reg_i);
						insertConnection(reg_o,reg_i);
						regCons[std::make_pair(reg_o,reg_i)]=true;
					}
				}


				//Create Input Registers linked to next time instance : N2N_FIX
//				if(peType == "N2N_4REGF"){
//					for(Module* subMod : currPE->subModules){
//						if(FU* fu = dynamic_cast<FU*>(subMod)){
//							for(Port *ip : fu->inputPorts){
//								std::string regRI_name = fu->getName() + "_" + ip->getName() + "RI";
//								std::string regRO_name = fu->getName() + "_" + ip->getName() + "RO";
//
//								Port* ri = nextCyclePE->getInPort(regRI_name); assert(ri);
//								Port* ro = currPE->getOutPort(regRO_name); assert(ro);
//
//								insertConnection(ro,ri);
//
//							}
//						}
//					}
//				}

				for(std::pair<Port*,Port*> curr_portpair : currPE->getRegConPorts()){
					std::pair<Port*,Port*> next_portpair = nextCyclePE->getRegConPort(curr_portpair.first->getName());
					Module* mod = curr_portpair.first->getMod();
					mod->insertConnection(curr_portpair.second,next_portpair.first);
					mod->regCons[std::make_pair(curr_portpair.second,next_portpair.first)]=true;
				}

			}
		}
	}
}

std::set<CGRAXMLCompile::Port*> CGRAXMLCompile::CGRA::getConflictPorts(Port* p) {
	if(conflictPorts.find(p)==conflictPorts.end()){
		std::set<Port*> emptyVec;
		conflictPorts[p]=emptyVec;
	}
//	std::cout << "size = " << conflictPorts[p].size() << "\n";

	for(Port* tp : conflictPorts[p]){
		assert(tp!=NULL);
//		std::cout << tp->getName() << "\n";
//		std::cout << tp->getFullName() << "\n";
	}

	return conflictPorts[p];
}

void CGRAXMLCompile::CGRA::insertConflictPort(Port* a, Port* b) {
	assert(a!=NULL);
	assert(b!=NULL);
	std::cout << "insertConflict Port b : " << b->getFullName() ;
	std::cout << ", a : " << a->getFullName() << "\n";
	conflictPorts[a].insert(b);
}

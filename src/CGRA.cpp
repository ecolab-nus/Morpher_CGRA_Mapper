/*
 * cgra.cpp
 *
 *  Created on: 20 Feb 2018
 *      Author: manupa
 */

#include "CGRA.h"
#include "PE.h"

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

	for (int t = 0; t < t_max; ++t) {
		for (int y = 0; y < y_max; ++y) {
			for (int x = 0; x < x_max; ++x) {
				PE* newPE;
				std::string PE_name = "PE_" + std::to_string(t) + "," + std::to_string(y) + "," + std::to_string(x);
				if(x==0){
					newPE = new PE(this,PE_name,x,y,t,peType,true,1);
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
					connections[currPE_WestO].push_back(westPE_EastI);
				}

				if(x+1 < x_max){
					Port* currPE_EastO = currPE->getOutPort("EAST_O");
					PE* eastPE = PEArr[t][y][x+1];
					Port* eastPE_WestI = eastPE->getInPort("WEST_I");
					connections[currPE_EastO].push_back(eastPE_WestI);
				}

				if(y-1 >= 0){
					Port* currPE_NorthO = currPE->getOutPort("NORTH_O");
					PE* northPE = PEArr[t][y-1][x];
					Port* northPE_SouthI = northPE->getInPort("SOUTH_I");
					connections[currPE_NorthO].push_back(northPE_SouthI);
				}

				if(y+1 < y_max){
					Port* currPE_SouthO = currPE->getOutPort("SOUTH_O");
					PE* southPE = PEArr[t][y+1][x];
					Port* southPE_NorthI = southPE->getInPort("NORTH_I");
					connections[currPE_SouthO].push_back(southPE_NorthI);
				}

				int t_next = (t+1)%t_max;
				PE* nextCyclePE = PEArr[t_next][y][x];
				for(RegFile* RF : currPE->allRegs){
					for (int i = 0; i < RF->get_nRegs(); ++i) {
						Port* reg_i = nextCyclePE->getInPort(RF->getName() + "_REG_I" + std::to_string(i));
						Port* reg_o = currPE->getOutPort(RF->getName() + "_REG_O" + std::to_string(i));

						connections[reg_o].push_back(reg_i);
					}
				}
			}
		}
	}
}



/*
 * FU.cpp
 *
 *  Created on: 26 Feb 2018
 *      Author: manupa
 */

#include "FU.h"
#include "DataPath.h"
#include <sstream>
#include <iostream>
#include <assert.h>

namespace CGRAXMLCompile {

//FU::FU() {
//	// TODO Auto-generated constructor stub
//
//}

} /* namespace CGRAXMLCompile */

void CGRAXMLCompile::FU::createFU(int numberDPs) {


//	std::cout << "----------------------------\n";
//	for (std::pair<Port*,std::vector<Port*> > pair : connections){
//		std::cout << pair.first->getFullName() << "::";
//		for(Port* p : pair.second){
//			std::cout << p->getFullName() << ",";
//		}
//		std::cout << "\n";
//	}
//	std::cout << "----------------------------\n";

	for (int i = 0; i < numberDPs; ++i) {
		std::string portName_P; portName_P.clear();
		std::string portName_I1; portName_I1.clear();
		std::string portName_I2; portName_I2.clear();

		portName_P  += "DP" + std::to_string(i) + "_P";
		portName_I1 += "DP" + std::to_string(i) + "_I1";
		portName_I2 += "DP" + std::to_string(i) + "_I2";

		//insert inputs ports
		inputPorts.push_back(new Port(portName_P,IN,this));
		inputPorts.push_back(new Port(portName_I1,IN,this));
		inputPorts.push_back(new Port(portName_I2,IN,this));

		//N2NFIX
//		outputPorts.push_back(Port(portName_P + "_RO",OUT,this));
//		insertConnection(getInPort(portName_P),getOutPort(portName_P + "_RO"));
//		outputPorts.push_back(Port(portName_I1 + "_RO",OUT,this));
//		insertConnection(getInPort(portName_I1),getOutPort(portName_I1 + "_RO"));
//		outputPorts.push_back(Port(portName_I2 + "_RO",OUT,this));
//		insertConnection(getInPort(portName_I2),getOutPort(portName_I2 + "_RO"));

		//insert output ports
		std::string portName_T; portName_T.clear();
		portName_T += "DP" + std::to_string(i) + "_T";
		outputPorts.push_back(new Port(portName_T,OUT,this));

		//create DataPaths
		std::string dataPathName; dataPathName.clear();
		dataPathName += "DP" + std::to_string(i);
		DataPath* newDP = new DataPath(this,dataPathName);
		subModules.push_back(newDP);
		assert(newDP == static_cast<DataPath*>(subModules.back()));
//		std::cout << "Submodule size = "<< subModules.size() << "\n";

//		std::cout << "DEBUG : " << portName_P << "," << portName_I1 << "," << portName_I2 << "," << dataPathName << "\n";

//		std::cout << getInPort(portName_P)->getFullName() << "\n";
//		std::cout << getInPort(portName_I1)->getFullName() << "\n";
//		std::cout << getInPort(portName_I2)->getFullName() << "\n";
//		std::cout << newDP->getOutPort("T")->getFullName() << "\n";
//
//		std::cout << newDP->getInPort("P")->getFullName() << "\n";
//		std::cout << newDP->getInPort("I1")->getFullName() << "\n";
//		std::cout << newDP->getInPort("I2")->getFullName() << "\n";
//		std::cout << getOutPort(portName_T)->getFullName() << "\n";

	}

	createFUInputRegCreate();

	for (int i = 0; i < numberDPs; ++i) {
		std::string portName_P; portName_P.clear();
		std::string portName_I1; portName_I1.clear();
		std::string portName_I2; portName_I2.clear();

		portName_P  += "DP" + std::to_string(i) + "_P";
		portName_I1 += "DP" + std::to_string(i) + "_I1";
		portName_I2 += "DP" + std::to_string(i) + "_I2";

		std::string portName_T; portName_T.clear();
		portName_T += "DP" + std::to_string(i) + "_T";

		std::string dataPathName; dataPathName.clear();
		dataPathName += "DP" + std::to_string(i);
		DataPath* newDP = static_cast<DataPath*>(this->getSubMod(dataPathName));

		//make connections
//		connectedTo[getInPort(portName_P)].push_back(newDP->getInPort("P"));
//		connectedTo[getInPort(portName_I1)].push_back(newDP->getInPort("I1"));
//		connectedTo[getInPort(portName_I2)].push_back(newDP->getInPort("I2"));
//		connectedTo[newDP->getOutPort("T")].push_back(getOutPort(portName_T));

		insertConnection(getInPort(portName_P),newDP->getInPort("P"));
		insertConnection(getInPort(portName_I1),newDP->getInPort("I1"));
		insertConnection(getInPort(portName_I2),newDP->getInPort("I2"));
		insertConnection(newDP->getOutPort("T"),getOutPort(portName_T));


	}


	for(Module* submod : this->subModules){
		if(DataPath* dp1 = dynamic_cast<DataPath*>(submod)){

			for(Module* submod : this->subModules){
				if(DataPath* dp2 = dynamic_cast<DataPath*>(submod)){
					if(dp1 == dp2) continue;
//					connectedTo[dp1->getOutPort("T")].push_back(dp2->getInPort("P"));
//					connectedTo[dp1->getOutPort("T")].push_back(dp2->getInPort("I1"));
//					connectedTo[dp1->getOutPort("T")].push_back(dp2->getInPort("I2"));

					insertConnection(dp1->getOutPort("T"),dp2->getInPort("P"));
					insertConnection(dp1->getOutPort("T"),dp2->getInPort("I1"));
					insertConnection(dp1->getOutPort("T"),dp2->getInPort("I2"));

				}
			}


		}
	}


}

bool CGRAXMLCompile::FU::isMEMFU() {
	if(supportedOPs.find("LOAD")!=supportedOPs.end()){
		return true;
	}
	return false;
}

void CGRAXMLCompile::FU::createFUInputRegConnections() {

	for (int i = 0; i < numberDPs; ++i) {
		std::string portName_P; portName_P.clear();
		std::string portName_I1; portName_I1.clear();
		std::string portName_I2; portName_I2.clear();

		portName_P  += "DP" + std::to_string(i) + "_P";
		portName_I1 += "DP" + std::to_string(i) + "_I1";
		portName_I2 += "DP" + std::to_string(i) + "_I2";


		assert(!getNextPorts(getOutPort("DP0_T")).empty());
		//N2NFIX
//		outputPorts.push_back(Port(portName_P + "_RO",OUT,this));
		assert(!getNextPorts(getOutPort("DP0_T")).empty());
		insertConnection(getInPort(portName_P),getOutPort(portName_P + "_RO"));
		assert(!getNextPorts(getOutPort("DP0_T")).empty());
//		outputPorts.push_back(Port(portName_I1 + "_RO",OUT,this));
		insertConnection(getInPort(portName_I1),getOutPort(portName_I1 + "_RO"));
		assert(!getNextPorts(getOutPort("DP0_T")).empty());
//		outputPorts.push_back(Port(portName_I2 + "_RO",OUT,this));
		insertConnection(getInPort(portName_I2),getOutPort(portName_I2 + "_RO"));
		assert(!getNextPorts(getOutPort("DP0_T")).empty());
		assert(!getNextPorts(getOutPort("DP0_T")).empty());

	}
}

void CGRAXMLCompile::FU::createFUInputRegCreate() {


	for (int i = 0; i < numberDPs; ++i) {
		std::string portName_P; portName_P.clear();
		std::string portName_I1; portName_I1.clear();
		std::string portName_I2; portName_I2.clear();

		portName_P  += "DP" + std::to_string(i) + "_P";
		portName_I1 += "DP" + std::to_string(i) + "_I1";
		portName_I2 += "DP" + std::to_string(i) + "_I2";

		outputPorts.push_back(new Port(portName_P + "_RO",OUT,this));
		outputPorts.push_back(new Port(portName_I1 + "_RO",OUT,this));
		outputPorts.push_back(new Port(portName_I2 + "_RO",OUT,this));
	}

}

/*
 * RegFile.cpp
 *
 *  Created on: 26 Feb 2018
 *      Author: manupa
 */

#include "RegFile.h"
#include <sstream>

namespace CGRAXMLCompile
{

//RegFile::RegFile() {
//	// TODO Auto-generated constructor stub
//
//}

} /* namespace CGRAXMLCompile */

void CGRAXMLCompile::RegFile::createRegFile(int nWRPs, int nRegs, int nRDPs)
{

	//create write ports
	for (int i = 0; i < nWRPs; ++i)
	{
		std::stringstream wrpName;
		wrpName << "WRP" << i;
		inputPorts.push_back(new Port(wrpName.str(), IN, this));
	}

	//create read nRDPs
	for (int i = 0; i < nRDPs; ++i)
	{
		std::stringstream rdpName;
		rdpName << "RDP" << i;
		outputPorts.push_back(new Port(rdpName.str(), OUT, this));
	}

	//create Reg Ports
	for (int i = 0; i < nRegs; ++i)
	{
		std::stringstream regIName;
		regIName << "REG_I" << i;
		inputPorts.push_back(new Port(regIName.str(), IN, this));

		std::stringstream regOName;
		regOName << "REG_O" << i;
		outputPorts.push_back(new Port(regOName.str(), OUT, this));
	}

	//make connections
	for (int i = 0; i < nWRPs; ++i)
	{
		std::stringstream wrpName;
		wrpName << "WRP" << i;
		for (int j = 0; j < nRegs; ++j)
		{
			std::stringstream regOName;
			regOName << "REG_O" << j;
			//			connectedTo[getInPort(wrpName.str())].push_back(getOutPort(regOName.str()));
			insertConnection(getInPort(wrpName.str()), getOutPort(regOName.str()));
		}
	}

	for (int i = 0; i < nRegs; ++i)
	{
		std::stringstream regIName;
		regIName << "REG_I" << i;
		for (int j = 0; j < nRDPs; ++j)
		{
			std::stringstream rdpName;
			rdpName << "RDP" << j;
			//			connectedTo[getInPort(regIName.str())].push_back(getOutPort(rdpName.str()));
			insertConnection(getInPort(regIName.str()), getOutPort(rdpName.str()));
		}

		//		std::stringstream regOName;
		//		regOName << "REG_O" << i;
		//		connections[getInPort(regIName.str())].push_back(getInPort(regOName.str()));
	}

	for (int i = 0; i < nRegs; ++i)
	{
		std::stringstream regIName;
		regIName << "REG_I" << i;
		std::stringstream regOName;
		regOName << "REG_O" << i;
		insertConnection(getInPort(regIName.str()), getOutPort(regOName.str()));
	}
}

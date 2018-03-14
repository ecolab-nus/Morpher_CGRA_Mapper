/*
 * PE.cpp
 *
 *  Created on: 26 Feb 2018
 *      Author: manupa
 */

#include "PE.h"
#include "FU.h"
#include "RegFile.h"
#include <assert.h>
#include <sstream>

namespace CGRAXMLCompile {

//PE::PE() {
//	// TODO Auto-generated constructor stub
//
//}

} /* namespace CGRAXMLCompile */

void CGRAXMLCompile::PE::createGenericPE(bool isMEMpe, int numberofDPs, int regs) {

	assert(!alreadyInit);
	alreadyInit=true;
	this->isMemPE=isMemPE;

//	int numberofDPs=1;
	std::map<std::string,int> supportedOPs;

	if(isMEMpe){
		getMEMIns(supportedOPs);
	}
	else{
		getNonMEMIns(supportedOPs);
	}


	//create FU
	FU* nonMemFU0 = new FU(this,"NMEM_FU0",numberofDPs,supportedOPs);
	subModules.push_back(nonMemFU0);

	//create R0
	RegFile* RF0 = new RegFile(this,"RF0",regs,regs,regs); subModules.push_back(RF0); allRegs.push_back(RF0);
//	RegFile* R1 = new RegFile(this,"R1",1,1,1); subModules.push_back(R1); allRegs.push_back(R1);
//	RegFile* R2 = new RegFile(this,"R2",1,1,1); subModules.push_back(R2); allRegs.push_back(R2);
//	RegFile* R3 = new RegFile(this,"R3",1,1,1); subModules.push_back(R3); allRegs.push_back(R3);
//	RegFile* RES = new RegFile(this,"RES",1,1,1); subModules.push_back(RES);allRegs.push_back(RES);

	//create input and output ports
	inputPorts.push_back(Port("NORTH_I",IN,this));
	inputPorts.push_back(Port("EAST_I",IN,this));
	inputPorts.push_back(Port("WEST_I",IN,this));
	inputPorts.push_back(Port("SOUTH_I",IN,this));

	outputPorts.push_back(Port("NORTH_O",OUT,this));
	outputPorts.push_back(Port("EAST_O",OUT,this));
	outputPorts.push_back(Port("WEST_O",OUT,this));
	outputPorts.push_back(Port("SOUTH_O",OUT,this));

	for(RegFile* RF : allRegs){
		for (int i = 0; i < RF->get_nRegs(); ++i) {
			inputPorts.push_back(Port(RF->getName() + "_REG_I" + std::to_string(i),IN,this));
			outputPorts.push_back(Port(RF->getName() + "_REG_O" + std::to_string(i),OUT,this));
		}
	}

	//make connections
	Port* NORTH_I = getInPort("NORTH_I");
	Port* EAST_I = getInPort("EAST_I");
	Port* WEST_I = getInPort("WEST_I");
	Port* SOUTH_I = getInPort("SOUTH_I");

	Port* NORTH_O = getOutPort("NORTH_O");
	Port* EAST_O = getOutPort("EAST_O");
	Port* WEST_O = getOutPort("WEST_O");
	Port* SOUTH_O = getOutPort("SOUTH_O");

	//connect inputs to register file(s)
	for(RegFile* RF : allRegs){
		for (int i = 0; i < RF->get_nWRPs(); ++i) {
			std::stringstream wrpName;
			wrpName << "WRP" << i;
			Port* wrp = RF->getInPort(wrpName.str());
			connections[NORTH_I].push_back(wrp);
			connections[EAST_I].push_back(wrp);
			connections[WEST_I].push_back(wrp);
			connections[SOUTH_I].push_back(wrp);
		}
	}

	//connect register file ports to PE ports
	for(RegFile* RF : allRegs){
		for (int i = 0; i < RF->get_nRegs(); ++i) {
			Port* reg_i = getInPort(RF->getName() + "_REG_I" + std::to_string(i));
			Port* reg_o = getOutPort(RF->getName() + "_REG_O" + std::to_string(i));

			connections[reg_i].push_back(RF->getInPort("REG_I" + std::to_string(i)));
			connections[RF->getOutPort("REG_O" + std::to_string(i))].push_back(reg_o);
		}
	}

	//connecting inputs to the FU
	for(Port &p : nonMemFU0->inputPorts){
		connections[NORTH_I].push_back(&p);
		connections[EAST_I].push_back(&p);
		connections[WEST_I].push_back(&p);
		connections[SOUTH_I].push_back(&p);
	}

	//connect register file readports to FU and writeports to main directional outputs
	for(RegFile* RF : allRegs){
		for (int i = 0; i < RF->get_nRDPs(); ++i) {
			std::stringstream rdpName;
			rdpName << "RDP" << i;
			Port* rdp = RF->getOutPort(rdpName.str());

			for(Port &p : nonMemFU0->inputPorts){
				connections[rdp].push_back(&p);
			}

			connections[rdp].push_back(NORTH_O);
			connections[rdp].push_back(EAST_O);
			connections[rdp].push_back(WEST_O);
			connections[rdp].push_back(SOUTH_O);
		}
	}

	//connect output of FU to wrp of RegFile and main directional outputs
	for(Port &p : nonMemFU0->outputPorts){
		for(RegFile* RF : allRegs){
			for (int i = 0; i < RF->get_nWRPs(); ++i) {
				std::stringstream wrpName;
				wrpName << "WRP" << i;
				Port* wrp = RF->getInPort(wrpName.str());
				connections[&p].push_back(wrp);
			}
		}
		connections[&p].push_back(NORTH_O);
		connections[&p].push_back(EAST_O);
		connections[&p].push_back(WEST_O);
		connections[&p].push_back(SOUTH_O);
	}



}

void CGRAXMLCompile::PE::getNonMEMIns(
		std::map<std::string, int>& supportedOPs) {

	supportedOPs["NOP"]=1;
	supportedOPs["ADD"]=1;
	supportedOPs["SUB"]=1;
	supportedOPs["MUL"]=1;
	supportedOPs["SEXT"]=1;
	supportedOPs["DIV"]=1;
	supportedOPs["LS"]=1;
	supportedOPs["RS"]=1;
	supportedOPs["ARS"]=1;
	supportedOPs["AND"]=1;
	supportedOPs["OR"]=1;
	supportedOPs["XOR"]=1;
	supportedOPs["SELECT"]=1;
	supportedOPs["CMERGE"]=1;
	supportedOPs["CMP"]=1;
	supportedOPs["CLT"]=1;
	supportedOPs["BR"]=1;
	supportedOPs["CGT"]=1;
	supportedOPs["MOVCL"]=1;
	supportedOPs["JUMPL"]=1;
	supportedOPs["MOVC"]=1;

}

void CGRAXMLCompile::PE::getMEMIns(std::map<std::string, int>& supportedOPs) {

	supportedOPs["NOP"]=2;
	supportedOPs["ADD"]=2;
	supportedOPs["SUB"]=2;
	supportedOPs["MUL"]=2;
	supportedOPs["SEXT"]=2;
	supportedOPs["DIV"]=2;
	supportedOPs["LS"]=2;
	supportedOPs["RS"]=2;
	supportedOPs["ARS"]=2;
	supportedOPs["AND"]=2;
	supportedOPs["OR"]=2;
	supportedOPs["XOR"]=2;
	supportedOPs["SELECT"]=2;
	supportedOPs["CMERGE"]=2;
	supportedOPs["CMP"]=2;
	supportedOPs["CLT"]=2;
	supportedOPs["BR"]=2;
	supportedOPs["CGT"]=2;
	supportedOPs["MOVCL"]=2;

	supportedOPs["LOADCL"]=2;
	supportedOPs["LOAD"]=2;
	supportedOPs["LOADH"]=2;
	supportedOPs["LOADB"]=2;
	supportedOPs["STORE"]=2;
	supportedOPs["STOREH"]=2;
	supportedOPs["STOREB"]=2;

	supportedOPs["JUMPL"]=2;
	supportedOPs["MOVC"]=2;
}

void CGRAXMLCompile::PE::createHyCUBEPE_RegFile(bool isMEMpe, int numberofDPs, int regs) {


	assert(!alreadyInit);
	alreadyInit=true;
	this->isMemPE=isMemPE;

//	int numberofDPs=1;
	std::map<std::string,int> supportedOPs;

	if(isMEMpe){
		getMEMIns(supportedOPs);
	}
	else{
		getNonMEMIns(supportedOPs);
	}


	//create FU
	FU* nonMemFU0 = new FU(this,"NMEM_FU0",numberofDPs,supportedOPs);
	subModules.push_back(nonMemFU0);

	//create R0
	RegFile* RF0 = new RegFile(this,"RF0",regs,regs,regs); subModules.push_back(RF0); allRegs.push_back(RF0);
//	RegFile* R1 = new RegFile(this,"R1",1,1,1); subModules.push_back(R1); allRegs.push_back(R1);
//	RegFile* R2 = new RegFile(this,"R2",1,1,1); subModules.push_back(R2); allRegs.push_back(R2);
//	RegFile* R3 = new RegFile(this,"R3",1,1,1); subModules.push_back(R3); allRegs.push_back(R3);
//	RegFile* RES = new RegFile(this,"RES",1,1,1); subModules.push_back(RES);allRegs.push_back(RES);

	//create input and output ports
	inputPorts.push_back(Port("NORTH_I",IN,this));
	inputPorts.push_back(Port("EAST_I",IN,this));
	inputPorts.push_back(Port("WEST_I",IN,this));
	inputPorts.push_back(Port("SOUTH_I",IN,this));

	outputPorts.push_back(Port("NORTH_O",OUT,this));
	outputPorts.push_back(Port("EAST_O",OUT,this));
	outputPorts.push_back(Port("WEST_O",OUT,this));
	outputPorts.push_back(Port("SOUTH_O",OUT,this));

	for(RegFile* RF : allRegs){
		for (int i = 0; i < RF->get_nRegs(); ++i) {
			inputPorts.push_back(Port(RF->getName() + "_REG_I" + std::to_string(i),IN,this));
			outputPorts.push_back(Port(RF->getName() + "_REG_O" + std::to_string(i),OUT,this));
		}
	}

	//make connections
	Port* NORTH_I = getInPort("NORTH_I");
	Port* EAST_I = getInPort("EAST_I");
	Port* WEST_I = getInPort("WEST_I");
	Port* SOUTH_I = getInPort("SOUTH_I");

	Port* NORTH_O = getOutPort("NORTH_O");
	Port* EAST_O = getOutPort("EAST_O");
	Port* WEST_O = getOutPort("WEST_O");
	Port* SOUTH_O = getOutPort("SOUTH_O");

	//connect inputs to register file(s)
	for(RegFile* RF : allRegs){
		for (int i = 0; i < RF->get_nWRPs(); ++i) {
			std::stringstream wrpName;
			wrpName << "WRP" << i;
			Port* wrp = RF->getInPort(wrpName.str());
			connections[NORTH_I].push_back(wrp);
			connections[EAST_I].push_back(wrp);
			connections[WEST_I].push_back(wrp);
			connections[SOUTH_I].push_back(wrp);
		}
	}

	//connect register file ports to PE ports
	for(RegFile* RF : allRegs){
		for (int i = 0; i < RF->get_nRegs(); ++i) {
			Port* reg_i = getInPort(RF->getName() + "_REG_I" + std::to_string(i));
			Port* reg_o = getOutPort(RF->getName() + "_REG_O" + std::to_string(i));

			connections[reg_i].push_back(RF->getInPort("REG_I" + std::to_string(i)));
			connections[RF->getOutPort("REG_O" + std::to_string(i))].push_back(reg_o);
		}
	}

	//connecting inputs to the FU
	for(Port &p : nonMemFU0->inputPorts){
		connections[NORTH_I].push_back(&p);
		connections[EAST_I].push_back(&p);
		connections[WEST_I].push_back(&p);
		connections[SOUTH_I].push_back(&p);
	}

	//connecting input to outputs :: HyCUBE Property
	connections[NORTH_I].push_back(NORTH_O);
	connections[NORTH_I].push_back(EAST_O);
	connections[NORTH_I].push_back(WEST_O);
	connections[NORTH_I].push_back(SOUTH_O);
	connections[EAST_I].push_back(NORTH_O);
	connections[EAST_I].push_back(EAST_O);
	connections[EAST_I].push_back(WEST_O);
	connections[EAST_I].push_back(SOUTH_O);
	connections[WEST_I].push_back(NORTH_O);
	connections[WEST_I].push_back(EAST_O);
	connections[WEST_I].push_back(WEST_O);
	connections[WEST_I].push_back(SOUTH_O);
	connections[SOUTH_I].push_back(NORTH_O);
	connections[SOUTH_I].push_back(EAST_O);
	connections[SOUTH_I].push_back(WEST_O);
	connections[SOUTH_I].push_back(SOUTH_O);

	//connect register file readports to FU and writeports to main directional outputs
	for(RegFile* RF : allRegs){
		for (int i = 0; i < RF->get_nRDPs(); ++i) {
			std::stringstream rdpName;
			rdpName << "RDP" << i;
			Port* rdp = RF->getOutPort(rdpName.str());

			for(Port &p : nonMemFU0->inputPorts){
				connections[rdp].push_back(&p);
			}

			connections[rdp].push_back(NORTH_O);
			connections[rdp].push_back(EAST_O);
			connections[rdp].push_back(WEST_O);
			connections[rdp].push_back(SOUTH_O);
		}
	}

	//connect output of FU to wrp of RegFile and main directional outputs
	for(Port &p : nonMemFU0->outputPorts){
		for(RegFile* RF : allRegs){
			for (int i = 0; i < RF->get_nWRPs(); ++i) {
				std::stringstream wrpName;
				wrpName << "WRP" << i;
				Port* wrp = RF->getInPort(wrpName.str());
				connections[&p].push_back(wrp);
			}
		}
		connections[&p].push_back(NORTH_O);
		connections[&p].push_back(EAST_O);
		connections[&p].push_back(WEST_O);
		connections[&p].push_back(SOUTH_O);
	}


}

void CGRAXMLCompile::PE::createOriginalHyCUBEPE(bool isMEMpe, int numberofDPs) {


	assert(!alreadyInit);
	alreadyInit=true;
	this->isMemPE=isMemPE;

//	int numberofDPs=1;
	std::map<std::string,int> supportedOPs;

	if(isMEMpe){
		getMEMIns(supportedOPs);
	}
	else{
		getNonMEMIns(supportedOPs);
	}


	//create FU
	FU* nonMemFU0 = new FU(this,"NMEM_FU0",numberofDPs,supportedOPs);
	subModules.push_back(nonMemFU0);

	//create R0
	RegFile* RF0 = new RegFile(this,"RF0",1,1,1); subModules.push_back(RF0); allRegs.push_back(RF0);
	RegFile* RF1 = new RegFile(this,"RF1",1,1,1); subModules.push_back(RF1); allRegs.push_back(RF1);
	RegFile* RF2 = new RegFile(this,"RF2",1,1,1); subModules.push_back(RF2); allRegs.push_back(RF2);
	RegFile* RF3 = new RegFile(this,"RF3",1,1,1); subModules.push_back(RF3); allRegs.push_back(RF3);
	RegFile* RFT = new RegFile(this,"RFT",numberofDPs,numberofDPs,numberofDPs); subModules.push_back(RFT); allRegs.push_back(RFT);

	//create input and output ports
	inputPorts.push_back(Port("NORTH_I",IN,this));
	inputPorts.push_back(Port("EAST_I",IN,this));
	inputPorts.push_back(Port("WEST_I",IN,this));
	inputPorts.push_back(Port("SOUTH_I",IN,this));

	outputPorts.push_back(Port("NORTH_O",OUT,this));
	outputPorts.push_back(Port("EAST_O",OUT,this));
	outputPorts.push_back(Port("WEST_O",OUT,this));
	outputPorts.push_back(Port("SOUTH_O",OUT,this));

	//create xbar input ports
	internalPorts.push_back( Port("NORTH_XBARI",OUT,this) );
	internalPorts.push_back( Port("EAST_XBARI",OUT,this) );
	internalPorts.push_back( Port("WEST_XBARI",OUT,this) );
	internalPorts.push_back( Port("SOUTH_XBARI",OUT,this) );

	Port* NORTH_XBARI = getInternalPort("NORTH_XBARI");
	Port* EAST_XBARI = getInternalPort("EAST_XBARI");
	Port* WEST_XBARI = getInternalPort("WEST_XBARI");
	Port* SOUTH_XBARI = getInternalPort("SOUTH_XBARI");


	//create PE-level time extended ports
	for(RegFile* RF : allRegs){
		for (int i = 0; i < RF->get_nRegs(); ++i) {
			inputPorts.push_back(Port(RF->getName() + "_REG_I" + std::to_string(i),IN,this));
			outputPorts.push_back(Port(RF->getName() + "_REG_O" + std::to_string(i),OUT,this));
		}
	}

	//make connections
	Port* NORTH_I = getInPort("NORTH_I");
	Port* EAST_I = getInPort("EAST_I");
	Port* WEST_I = getInPort("WEST_I");
	Port* SOUTH_I = getInPort("SOUTH_I");

	Port* NORTH_O = getOutPort("NORTH_O");
	Port* EAST_O = getOutPort("EAST_O");
	Port* WEST_O = getOutPort("WEST_O");
	Port* SOUTH_O = getOutPort("SOUTH_O");

	//connect main directional outputs to write ports of the registers
	connections[NORTH_I].push_back(RF0->getInPort("WRP0"));
	connections[EAST_I].push_back(RF1->getInPort("WRP0"));
	connections[WEST_I].push_back(RF2->getInPort("WRP0"));
	connections[SOUTH_I].push_back(RF3->getInPort("WRP0"));

	//connect main directional inputs to xbar input ports
	connections[NORTH_I].push_back(NORTH_XBARI);
	connections[EAST_I].push_back(EAST_XBARI);
	connections[WEST_I].push_back(WEST_XBARI);
	connections[SOUTH_I].push_back(SOUTH_XBARI);

	//connect the readports of the register file to xbar inputs
	connections[RF0->getOutPort("RDP0")].push_back(NORTH_XBARI);
	connections[RF1->getOutPort("RDP0")].push_back(EAST_XBARI);
	connections[RF2->getOutPort("RDP0")].push_back(WEST_XBARI);
	connections[RF3->getOutPort("RDP0")].push_back(SOUTH_XBARI);

	//connect register file ports to PE time extension ports
	for(RegFile* RF : allRegs){
		for (int i = 0; i < RF->get_nRegs(); ++i) {
			Port* reg_i = getInPort(RF->getName() + "_REG_I" + std::to_string(i));
			Port* reg_o = getOutPort(RF->getName() + "_REG_O" + std::to_string(i));

			connections[reg_i].push_back(RF->getInPort("REG_I" + std::to_string(i)));
			connections[RF->getOutPort("REG_O" + std::to_string(i))].push_back(reg_o);
		}
	}

	//connecting xbar inputs to the FU
	for(Port &p : nonMemFU0->inputPorts){
		connections[NORTH_XBARI].push_back(&p);
		connections[EAST_XBARI].push_back(&p);
		connections[WEST_XBARI].push_back(&p);
		connections[SOUTH_XBARI].push_back(&p);
	}

	//connecting xbar inputs to outputs :: HyCUBE Property
	connections[NORTH_XBARI].push_back(NORTH_O);
	connections[NORTH_XBARI].push_back(EAST_O);
	connections[NORTH_XBARI].push_back(WEST_O);
	connections[NORTH_XBARI].push_back(SOUTH_O);
	connections[EAST_XBARI].push_back(NORTH_O);
	connections[EAST_XBARI].push_back(EAST_O);
	connections[EAST_XBARI].push_back(WEST_O);
	connections[EAST_XBARI].push_back(SOUTH_O);
	connections[WEST_XBARI].push_back(NORTH_O);
	connections[WEST_XBARI].push_back(EAST_O);
	connections[WEST_XBARI].push_back(WEST_O);
	connections[WEST_XBARI].push_back(SOUTH_O);
	connections[SOUTH_XBARI].push_back(NORTH_O);
	connections[SOUTH_XBARI].push_back(EAST_O);
	connections[SOUTH_XBARI].push_back(WEST_O);
	connections[SOUTH_XBARI].push_back(SOUTH_O);

	//connect RFT readports to FU
	{
		RegFile* RF = RFT;
		for (int i = 0; i < RF->get_nRDPs(); ++i) {
			std::stringstream rdpName;
			rdpName << "RDP" << i;
			Port* rdp = RF->getOutPort(rdpName.str());

			for(Port &p : nonMemFU0->inputPorts){
				connections[rdp].push_back(&p);
			}
		}
	}


	for (int i = 0; i < numberofDPs; ++i) {
		std::stringstream rdpName;
		rdpName << "RDP" << i;
		connections[RFT->getOutPort(rdpName.str())].push_back(NORTH_O);
		connections[RFT->getOutPort(rdpName.str())].push_back(EAST_O);
		connections[RFT->getOutPort(rdpName.str())].push_back(WEST_O);
		connections[RFT->getOutPort(rdpName.str())].push_back(SOUTH_O);
	}

	//connect output of FU to wrp of RegFile and main directional outputs
	for(Port &p : nonMemFU0->outputPorts){
		connections[&p].push_back(NORTH_O);
		connections[&p].push_back(EAST_O);
		connections[&p].push_back(WEST_O);
		connections[&p].push_back(SOUTH_O);

		for (int i = 0; i < numberofDPs; ++i) {
			std::stringstream wrpName;
			wrpName << "WRP" << i;
			connections[&p].push_back(RFT->getInPort(wrpName.str()));
		}
	}

}


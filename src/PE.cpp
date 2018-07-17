/*
 * PE.cpp
 *
 *  Created on: 26 Feb 2018
 *      Author: manupa
 */

#include "PE.h"
#include "FU.h"
#include "RegFile.h"
#include "CGRA.h"
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
//			connectedTo[NORTH_I].push_back(wrp);
//			connectedTo[EAST_I].push_back(wrp);
//			connectedTo[WEST_I].push_back(wrp);
//			connectedTo[SOUTH_I].push_back(wrp);

			insertConnection(NORTH_I,wrp);
			insertConnection(EAST_I,wrp);
			insertConnection(WEST_I,wrp);
			insertConnection(SOUTH_I,wrp);

		}
	}

	//connect register file ports to PE ports
	for(RegFile* RF : allRegs){
		for (int i = 0; i < RF->get_nRegs(); ++i) {
			Port* reg_i = getInPort(RF->getName() + "_REG_I" + std::to_string(i));
			Port* reg_o = getOutPort(RF->getName() + "_REG_O" + std::to_string(i));

//			connectedTo[reg_i].push_back(RF->getInPort("REG_I" + std::to_string(i)));
//			connectedTo[RF->getOutPort("REG_O" + std::to_string(i))].push_back(reg_o);

			insertConnection(reg_i,RF->getInPort("REG_I" + std::to_string(i)));
			insertConnection(RF->getOutPort("REG_O" + std::to_string(i)),reg_o);

		}
	}

	//connecting inputs to the FU
	for(Port &p : nonMemFU0->inputPorts){
//		connectedTo[NORTH_I].push_back(&p);
//		connectedTo[EAST_I].push_back(&p);
//		connectedTo[WEST_I].push_back(&p);
//		connectedTo[SOUTH_I].push_back(&p);

		insertConnection(NORTH_I,&p);
		insertConnection(EAST_I,&p);
		insertConnection(WEST_I,&p);
		insertConnection(SOUTH_I,&p);


	}

	//connect register file readports to FU and writeports to main directional outputs
	for(RegFile* RF : allRegs){
		for (int i = 0; i < RF->get_nRDPs(); ++i) {
			std::stringstream rdpName;
			rdpName << "RDP" << i;
			Port* rdp = RF->getOutPort(rdpName.str());

			for(Port &p : nonMemFU0->inputPorts){
//				connectedTo[rdp].push_back(&p);
				insertConnection(rdp,&p);
			}

//			connectedTo[rdp].push_back(NORTH_O);
//			connectedTo[rdp].push_back(EAST_O);
//			connectedTo[rdp].push_back(WEST_O);
//			connectedTo[rdp].push_back(SOUTH_O);

			insertConnection(rdp,NORTH_O);
			insertConnection(rdp,EAST_O);
			insertConnection(rdp,WEST_O);
			insertConnection(rdp,SOUTH_O);
		}
	}

	//connect output of FU to wrp of RegFile and main directional outputs
	for(Port &p : nonMemFU0->outputPorts){
		for(RegFile* RF : allRegs){
			for (int i = 0; i < RF->get_nWRPs(); ++i) {
				std::stringstream wrpName;
				wrpName << "WRP" << i;
				Port* wrp = RF->getInPort(wrpName.str());
//				connectedTo[&p].push_back(wrp);
				insertConnection(&p,wrp);
			}
		}
//		connectedTo[&p].push_back(NORTH_O);
//		connectedTo[&p].push_back(EAST_O);
//		connectedTo[&p].push_back(WEST_O);
//		connectedTo[&p].push_back(SOUTH_O);

		insertConnection(&p,NORTH_O);
		insertConnection(&p,EAST_O);
		insertConnection(&p,WEST_O);
		insertConnection(&p,SOUTH_O);

	}



}

void CGRAXMLCompile::PE::createStdNoCPE(bool isMEMpe, int numberofDPs) {

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
	internalPorts.push_back( Port("NORTH_XBARI",INT,this) );
	internalPorts.push_back( Port("EAST_XBARI",INT,this) );
	internalPorts.push_back( Port("WEST_XBARI",INT,this) );
	internalPorts.push_back( Port("SOUTH_XBARI",INT,this) );

	internalPorts.push_back( Port("NORTH_PRE_XBARI",INT,this) );
	internalPorts.push_back( Port("EAST_PRE_XBARI",INT,this) );
	internalPorts.push_back( Port("WEST_PRE_XBARI",INT,this) );
	internalPorts.push_back( Port("SOUTH_PRE_XBARI",INT,this) );

	Port* NORTH_XBARI = getInternalPort("NORTH_XBARI");
	Port* EAST_XBARI = getInternalPort("EAST_XBARI");
	Port* WEST_XBARI = getInternalPort("WEST_XBARI");
	Port* SOUTH_XBARI = getInternalPort("SOUTH_XBARI");

	Port* NORTH_PRE_XBARI = getInternalPort("NORTH_PRE_XBARI");
	Port* EAST_PRE_XBARI = getInternalPort("EAST_PRE_XBARI");
	Port* WEST_PRE_XBARI = getInternalPort("WEST_PRE_XBARI");
	Port* SOUTH_PRE_XBARI = getInternalPort("SOUTH_PRE_XBARI");

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
//	connectedTo[NORTH_I].push_back(RF0->getInPort("WRP0"));
//	connectedTo[EAST_I].push_back(RF1->getInPort("WRP0"));
//	connectedTo[WEST_I].push_back(RF2->getInPort("WRP0"));
//	connectedTo[SOUTH_I].push_back(RF3->getInPort("WRP0"));

	insertConnection(NORTH_I,RF0->getInPort("WRP0"));
	insertConnection(EAST_I,RF1->getInPort("WRP0"));
	insertConnection(WEST_I,RF2->getInPort("WRP0"));
	insertConnection(SOUTH_I,RF3->getInPort("WRP0"));


	//connect main directional inputs to xbar input ports
//	connectedTo[NORTH_I].push_back(NORTH_XBARI);
//	connectedTo[EAST_I].push_back(EAST_XBARI);
//	connectedTo[WEST_I].push_back(WEST_XBARI);
//	connectedTo[SOUTH_I].push_back(SOUTH_XBARI);

	insertConnection(NORTH_I,NORTH_PRE_XBARI);
	insertConnection(EAST_I,EAST_PRE_XBARI);
	insertConnection(WEST_I,WEST_PRE_XBARI);
	insertConnection(SOUTH_I,SOUTH_PRE_XBARI);

	insertConnection(NORTH_PRE_XBARI,NORTH_XBARI);
	insertConnection(EAST_PRE_XBARI,EAST_XBARI);
	insertConnection(WEST_PRE_XBARI,WEST_XBARI);
	insertConnection(SOUTH_PRE_XBARI,SOUTH_XBARI);

	//connect the readports of the register file to xbar inputs
//	connectedTo[RF0->getOutPort("RDP0")].push_back(NORTH_XBARI);
//	connectedTo[RF1->getOutPort("RDP0")].push_back(EAST_XBARI);
//	connectedTo[RF2->getOutPort("RDP0")].push_back(WEST_XBARI);
//	connectedTo[RF3->getOutPort("RDP0")].push_back(SOUTH_XBARI);

	insertConnection(RF0->getOutPort("RDP0"),NORTH_XBARI);
	insertConnection(RF1->getOutPort("RDP0"),EAST_XBARI);
	insertConnection(RF2->getOutPort("RDP0"),WEST_XBARI);
	insertConnection(RF3->getOutPort("RDP0"),SOUTH_XBARI);

	//connect register file ports to PE time extension ports
	for(RegFile* RF : allRegs){
		for (int i = 0; i < RF->get_nRegs(); ++i) {
			Port* reg_i = getInPort(RF->getName() + "_REG_I" + std::to_string(i));
			Port* reg_o = getOutPort(RF->getName() + "_REG_O" + std::to_string(i));

//			connectedTo[reg_i].push_back(RF->getInPort("REG_I" + std::to_string(i)));
//			connectedTo[RF->getOutPort("REG_O" + std::to_string(i))].push_back(reg_o);

			insertConnection(reg_i,RF->getInPort("REG_I" + std::to_string(i)));
			insertConnection(RF->getOutPort("REG_O" + std::to_string(i)),reg_o);

		}
	}

	//connecting xbar inputs to the FU
	for(Port &p : nonMemFU0->inputPorts){
//		connectedTo[NORTH_XBARI].push_back(&p);
//		connectedTo[EAST_XBARI].push_back(&p);
//		connectedTo[WEST_XBARI].push_back(&p);
//		connectedTo[SOUTH_XBARI].push_back(&p);

		insertConnection(NORTH_XBARI,&p);
		insertConnection(EAST_XBARI,&p);
		insertConnection(WEST_XBARI,&p);
		insertConnection(SOUTH_XBARI,&p);
	}

	//connecting input registers to outputs :: STDNOC Property
//	connectedTo[RF0->getOutPort("RDP0")].push_back(NORTH_O);
//	connectedTo[RF0->getOutPort("RDP0")].push_back(EAST_O);
//	connectedTo[RF0->getOutPort("RDP0")].push_back(WEST_O);
//	connectedTo[RF0->getOutPort("RDP0")].push_back(SOUTH_O);
//	connectedTo[RF1->getOutPort("RDP0")].push_back(NORTH_O);
//	connectedTo[RF1->getOutPort("RDP0")].push_back(EAST_O);
//	connectedTo[RF1->getOutPort("RDP0")].push_back(WEST_O);
//	connectedTo[RF1->getOutPort("RDP0")].push_back(SOUTH_O);
//	connectedTo[RF2->getOutPort("RDP0")].push_back(NORTH_O);
//	connectedTo[RF2->getOutPort("RDP0")].push_back(EAST_O);
//	connectedTo[RF2->getOutPort("RDP0")].push_back(WEST_O);
//	connectedTo[RF2->getOutPort("RDP0")].push_back(SOUTH_O);
//	connectedTo[RF3->getOutPort("RDP0")].push_back(NORTH_O);
//	connectedTo[RF3->getOutPort("RDP0")].push_back(EAST_O);
//	connectedTo[RF3->getOutPort("RDP0")].push_back(WEST_O);
//	connectedTo[RF3->getOutPort("RDP0")].push_back(SOUTH_O);

	CGRA* currCGRA = getCGRA();

	currCGRA->insertConflictPort(RF0->getOutPort("RDP0"),NORTH_PRE_XBARI);
	currCGRA->insertConflictPort(RF1->getOutPort("RDP0"),EAST_PRE_XBARI);
	currCGRA->insertConflictPort(RF2->getOutPort("RDP0"),WEST_PRE_XBARI);
	currCGRA->insertConflictPort(RF3->getOutPort("RDP0"),SOUTH_PRE_XBARI);

//	conflictPorts[RF0->getOutPort("RDP0")].push_back(NORTH_I);
//	conflictPorts[RF1->getOutPort("RDP0")].push_back(EAST_I);
//	conflictPorts[RF2->getOutPort("RDP0")].push_back(WEST_I);
//	conflictPorts[RF3->getOutPort("RDP0")].push_back(SOUTH_I);

	currCGRA->insertConflictPort(NORTH_PRE_XBARI,RF0->getOutPort("RDP0"));
	currCGRA->insertConflictPort(EAST_PRE_XBARI,RF1->getOutPort("RDP0"));
	currCGRA->insertConflictPort(WEST_PRE_XBARI,RF2->getOutPort("RDP0"));
	currCGRA->insertConflictPort(SOUTH_PRE_XBARI,RF3->getOutPort("RDP0"));

//	conflictPorts[NORTH_I].push_back(RF0->getOutPort("RDP0"));
//	conflictPorts[EAST_I].push_back(RF1->getOutPort("RDP0"));
//	conflictPorts[WEST_I].push_back(RF2->getOutPort("RDP0"));
//	conflictPorts[SOUTH_I].push_back(RF3->getOutPort("RDP0"));

	insertConnection(RF0->getOutPort("RDP0"),NORTH_O);
	insertConnection(RF0->getOutPort("RDP0"),EAST_O);
	insertConnection(RF0->getOutPort("RDP0"),WEST_O);
	insertConnection(RF0->getOutPort("RDP0"),SOUTH_O);
	insertConnection(RF1->getOutPort("RDP0"),NORTH_O);
	insertConnection(RF1->getOutPort("RDP0"),EAST_O);
	insertConnection(RF1->getOutPort("RDP0"),WEST_O);
	insertConnection(RF1->getOutPort("RDP0"),SOUTH_O);
	insertConnection(RF2->getOutPort("RDP0"),NORTH_O);
	insertConnection(RF2->getOutPort("RDP0"),EAST_O);
	insertConnection(RF2->getOutPort("RDP0"),WEST_O);
	insertConnection(RF2->getOutPort("RDP0"),SOUTH_O);
	insertConnection(RF3->getOutPort("RDP0"),NORTH_O);
	insertConnection(RF3->getOutPort("RDP0"),EAST_O);
	insertConnection(RF3->getOutPort("RDP0"),WEST_O);
	insertConnection(RF3->getOutPort("RDP0"),SOUTH_O);


	//connect RFT readports to FU
	{
		RegFile* RF = RFT;
		for (int i = 0; i < RF->get_nRDPs(); ++i) {
			std::stringstream rdpName;
			rdpName << "RDP" << i;
			Port* rdp = RF->getOutPort(rdpName.str());

			for(Port &p : nonMemFU0->inputPorts){
//				connectedTo[rdp].push_back(&p);
				insertConnection(rdp,&p);
			}
		}
	}


	for (int i = 0; i < numberofDPs; ++i) {
		std::stringstream rdpName;
		rdpName << "RDP" << i;
//		connectedTo[RFT->getOutPort(rdpName.str())].push_back(NORTH_O);
//		connectedTo[RFT->getOutPort(rdpName.str())].push_back(EAST_O);
//		connectedTo[RFT->getOutPort(rdpName.str())].push_back(WEST_O);
//		connectedTo[RFT->getOutPort(rdpName.str())].push_back(SOUTH_O);

		insertConnection(RFT->getOutPort(rdpName.str()),NORTH_O);
		insertConnection(RFT->getOutPort(rdpName.str()),EAST_O);
		insertConnection(RFT->getOutPort(rdpName.str()),WEST_O);
		insertConnection(RFT->getOutPort(rdpName.str()),SOUTH_O);
	}

	//connect output of FU to wrp of RegFile and main directional outputs
	for(Port &p : nonMemFU0->outputPorts){
//		connectedTo[&p].push_back(NORTH_O);
//		connectedTo[&p].push_back(EAST_O);
//		connectedTo[&p].push_back(WEST_O);
//		connectedTo[&p].push_back(SOUTH_O);

		insertConnection(&p,NORTH_O);
		insertConnection(&p,EAST_O);
		insertConnection(&p,WEST_O);
		insertConnection(&p,SOUTH_O);

		for (int i = 0; i < numberofDPs; ++i) {
			std::stringstream wrpName;
			wrpName << "WRP" << i;
//			connectedTo[&p].push_back(RFT->getInPort(wrpName.str()));
			insertConnection(&p,RFT->getInPort(wrpName.str()));
		}
	}


}

void CGRAXMLCompile::PE::createStdNoCPE_RegFile(bool isMEMpe, int numberofDPs,
		int regs, int nWRP, int nRDP) {

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
	RegFile* RF0 = new RegFile(this,"RF0",nWRP,regs,nRDP); subModules.push_back(RF0); allRegs.push_back(RF0);
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
//			connectedTo[NORTH_I].push_back(wrp);
//			connectedTo[EAST_I].push_back(wrp);
//			connectedTo[WEST_I].push_back(wrp);
//			connectedTo[SOUTH_I].push_back(wrp);

			insertConnection(NORTH_I,wrp);
			insertConnection(EAST_I,wrp);
			insertConnection(WEST_I,wrp);
			insertConnection(SOUTH_I,wrp);
		}
	}

	//connect register file ports to PE ports
	for(RegFile* RF : allRegs){
		for (int i = 0; i < RF->get_nRegs(); ++i) {
			Port* reg_i = getInPort(RF->getName() + "_REG_I" + std::to_string(i));
			Port* reg_o = getOutPort(RF->getName() + "_REG_O" + std::to_string(i));

//			connectedTo[reg_i].push_back(RF->getInPort("REG_I" + std::to_string(i)));
//			connectedTo[RF->getOutPort("REG_O" + std::to_string(i))].push_back(reg_o);

			insertConnection(reg_i,RF->getInPort("REG_I" + std::to_string(i)));
			insertConnection(RF->getOutPort("REG_O" + std::to_string(i)),reg_o);
		}
	}

	//connecting inputs to the FU
	for(Port &p : nonMemFU0->inputPorts){
//		connectedTo[NORTH_I].push_back(&p);
//		connectedTo[EAST_I].push_back(&p);
//		connectedTo[WEST_I].push_back(&p);
//		connectedTo[SOUTH_I].push_back(&p);

		insertConnection(NORTH_I,&p);
		insertConnection(EAST_I,&p);
		insertConnection(WEST_I,&p);
		insertConnection(SOUTH_I,&p);
	}

	//connecting input to outputs :: HyCUBE Property
//	connections[NORTH_I].push_back(NORTH_O);
//	connections[NORTH_I].push_back(EAST_O);
//	connections[NORTH_I].push_back(WEST_O);
//	connections[NORTH_I].push_back(SOUTH_O);
//	connections[EAST_I].push_back(NORTH_O);
//	connections[EAST_I].push_back(EAST_O);
//	connections[EAST_I].push_back(WEST_O);
//	connections[EAST_I].push_back(SOUTH_O);
//	connections[WEST_I].push_back(NORTH_O);
//	connections[WEST_I].push_back(EAST_O);
//	connections[WEST_I].push_back(WEST_O);
//	connections[WEST_I].push_back(SOUTH_O);
//	connections[SOUTH_I].push_back(NORTH_O);
//	connections[SOUTH_I].push_back(EAST_O);
//	connections[SOUTH_I].push_back(WEST_O);
//	connections[SOUTH_I].push_back(SOUTH_O);

	//connect register file readports to FU and writeports to main directional outputs
	for(RegFile* RF : allRegs){
		for (int i = 0; i < RF->get_nRDPs(); ++i) {
			std::stringstream rdpName;
			rdpName << "RDP" << i;
			Port* rdp = RF->getOutPort(rdpName.str());

			for(Port &p : nonMemFU0->inputPorts){
//				connectedTo[rdp].push_back(&p);
				insertConnection(rdp,&p);
			}

//			connectedTo[rdp].push_back(NORTH_O);
//			connectedTo[rdp].push_back(EAST_O);
//			connectedTo[rdp].push_back(WEST_O);
//			connectedTo[rdp].push_back(SOUTH_O);

			insertConnection(rdp,NORTH_O);
			insertConnection(rdp,EAST_O);
			insertConnection(rdp,WEST_O);
			insertConnection(rdp,SOUTH_O);
		}

	}

	//connect output of FU to wrp of RegFile and main directional outputs
	for(Port &p : nonMemFU0->outputPorts){
		for(RegFile* RF : allRegs){
			for (int i = 0; i < RF->get_nWRPs(); ++i) {
				std::stringstream wrpName;
				wrpName << "WRP" << i;
				Port* wrp = RF->getInPort(wrpName.str());
//				connectedTo[&p].push_back(wrp);
				insertConnection(&p,wrp);
			}
		}
//		connectedTo[&p].push_back(NORTH_O);
//		connectedTo[&p].push_back(EAST_O);
//		connectedTo[&p].push_back(WEST_O);
//		connectedTo[&p].push_back(SOUTH_O);

		insertConnection(&p,NORTH_O);
		insertConnection(&p,EAST_O);
		insertConnection(&p,WEST_O);
		insertConnection(&p,SOUTH_O);
	}

}


void CGRAXMLCompile::PE::createHyCUBEPE_RegFile(bool isMEMpe, int numberofDPs, int regs, int nWRP, int nRDP) {


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
	RegFile* RF0 = new RegFile(this,"RF0",nWRP,regs,nRDP); subModules.push_back(RF0); allRegs.push_back(RF0);
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
//			connectedTo[NORTH_I].push_back(wrp);
//			connectedTo[EAST_I].push_back(wrp);
//			connectedTo[WEST_I].push_back(wrp);
//			connectedTo[SOUTH_I].push_back(wrp);

			insertConnection(NORTH_I,wrp);
			insertConnection(EAST_I,wrp);
			insertConnection(WEST_I,wrp);
			insertConnection(SOUTH_I,wrp);
		}
	}

	//connect register file ports to PE ports
	for(RegFile* RF : allRegs){
		for (int i = 0; i < RF->get_nRegs(); ++i) {
			Port* reg_i = getInPort(RF->getName() + "_REG_I" + std::to_string(i));
			Port* reg_o = getOutPort(RF->getName() + "_REG_O" + std::to_string(i));

//			connectedTo[reg_i].push_back(RF->getInPort("REG_I" + std::to_string(i)));
//			connectedTo[RF->getOutPort("REG_O" + std::to_string(i))].push_back(reg_o);

			insertConnection(reg_i,RF->getInPort("REG_I" + std::to_string(i)));
			insertConnection(RF->getOutPort("REG_O" + std::to_string(i)),reg_o);
		}
	}

	//connecting inputs to the FU
	for(Port &p : nonMemFU0->inputPorts){
//		connectedTo[NORTH_I].push_back(&p);
//		connectedTo[EAST_I].push_back(&p);
//		connectedTo[WEST_I].push_back(&p);
//		connectedTo[SOUTH_I].push_back(&p);

		insertConnection(NORTH_I,&p);
		insertConnection(EAST_I,&p);
		insertConnection(WEST_I,&p);
		insertConnection(SOUTH_I,&p);
	}

	//connecting input to outputs :: HyCUBE Property
//	connectedTo[NORTH_I].push_back(NORTH_O);
//	connectedTo[NORTH_I].push_back(EAST_O);
//	connectedTo[NORTH_I].push_back(WEST_O);
//	connectedTo[NORTH_I].push_back(SOUTH_O);
//	connectedTo[EAST_I].push_back(NORTH_O);
//	connectedTo[EAST_I].push_back(EAST_O);
//	connectedTo[EAST_I].push_back(WEST_O);
//	connectedTo[EAST_I].push_back(SOUTH_O);
//	connectedTo[WEST_I].push_back(NORTH_O);
//	connectedTo[WEST_I].push_back(EAST_O);
//	connectedTo[WEST_I].push_back(WEST_O);
//	connectedTo[WEST_I].push_back(SOUTH_O);
//	connectedTo[SOUTH_I].push_back(NORTH_O);
//	connectedTo[SOUTH_I].push_back(EAST_O);
//	connectedTo[SOUTH_I].push_back(WEST_O);
//	connectedTo[SOUTH_I].push_back(SOUTH_O);

	insertConnection(NORTH_I,NORTH_O);
	insertConnection(NORTH_I,EAST_O);
	insertConnection(NORTH_I,WEST_O);
	insertConnection(NORTH_I,SOUTH_O);

	insertConnection(EAST_I,NORTH_O);
	insertConnection(EAST_I,EAST_O);
	insertConnection(EAST_I,WEST_O);
	insertConnection(EAST_I,SOUTH_O);

	insertConnection(WEST_I,NORTH_O);
	insertConnection(WEST_I,EAST_O);
	insertConnection(WEST_I,WEST_O);
	insertConnection(WEST_I,SOUTH_O);

	insertConnection(SOUTH_I,NORTH_O);
	insertConnection(SOUTH_I,EAST_O);
	insertConnection(SOUTH_I,WEST_O);
	insertConnection(SOUTH_I,SOUTH_O);

	//connect register file readports to FU and writeports to main directional outputs
	for(RegFile* RF : allRegs){
		for (int i = 0; i < RF->get_nRDPs(); ++i) {
			std::stringstream rdpName;
			rdpName << "RDP" << i;
			Port* rdp = RF->getOutPort(rdpName.str());

			for(Port &p : nonMemFU0->inputPorts){
//				connectedTo[rdp].push_back(&p);
				insertConnection(rdp,&p);
			}

//			connectedTo[rdp].push_back(NORTH_O);
//			connectedTo[rdp].push_back(EAST_O);
//			connectedTo[rdp].push_back(WEST_O);
//			connectedTo[rdp].push_back(SOUTH_O);

			insertConnection(rdp,NORTH_O);
			insertConnection(rdp,EAST_O);
			insertConnection(rdp,WEST_O);
			insertConnection(rdp,SOUTH_O);
		}
	}

	//connect output of FU to wrp of RegFile and main directional outputs
	for(Port &p : nonMemFU0->outputPorts){
		for(RegFile* RF : allRegs){
			for (int i = 0; i < RF->get_nWRPs(); ++i) {
				std::stringstream wrpName;
				wrpName << "WRP" << i;
				Port* wrp = RF->getInPort(wrpName.str());
//				connectedTo[&p].push_back(wrp);
				insertConnection(&p,wrp);
			}
		}
//		connectedTo[&p].push_back(NORTH_O);
//		connectedTo[&p].push_back(EAST_O);
//		connectedTo[&p].push_back(WEST_O);
//		connectedTo[&p].push_back(SOUTH_O);

		insertConnection(&p,NORTH_O);
		insertConnection(&p,EAST_O);
		insertConnection(&p,WEST_O);
		insertConnection(&p,SOUTH_O);

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
	internalPorts.push_back( Port("NORTH_XBARI",INT,this) );
	internalPorts.push_back( Port("EAST_XBARI",INT,this) );
	internalPorts.push_back( Port("WEST_XBARI",INT,this) );
	internalPorts.push_back( Port("SOUTH_XBARI",INT,this) );

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
//	connectedTo[NORTH_I].push_back(RF0->getInPort("WRP0"));
//	connectedTo[EAST_I].push_back(RF1->getInPort("WRP0"));
//	connectedTo[WEST_I].push_back(RF2->getInPort("WRP0"));
//	connectedTo[SOUTH_I].push_back(RF3->getInPort("WRP0"));

	insertConnection(NORTH_I,RF0->getInPort("WRP0"));
	insertConnection(EAST_I,RF1->getInPort("WRP0"));
	insertConnection(WEST_I,RF2->getInPort("WRP0"));
	insertConnection(SOUTH_I,RF3->getInPort("WRP0"));

	//connect main directional inputs to xbar input ports
//	connectedTo[NORTH_I].push_back(NORTH_XBARI);
//	connectedTo[EAST_I].push_back(EAST_XBARI);
//	connectedTo[WEST_I].push_back(WEST_XBARI);
//	connectedTo[SOUTH_I].push_back(SOUTH_XBARI);

	insertConnection(NORTH_I,NORTH_XBARI);
	insertConnection(EAST_I,EAST_XBARI);
	insertConnection(WEST_I,WEST_XBARI);
	insertConnection(SOUTH_I,SOUTH_XBARI);


	//connect the readports of the register file to xbar inputs
//	connectedTo[RF0->getOutPort("RDP0")].push_back(NORTH_XBARI);
//	connectedTo[RF1->getOutPort("RDP0")].push_back(EAST_XBARI);
//	connectedTo[RF2->getOutPort("RDP0")].push_back(WEST_XBARI);
//	connectedTo[RF3->getOutPort("RDP0")].push_back(SOUTH_XBARI);

	insertConnection(RF0->getOutPort("RDP0"),NORTH_XBARI);
	insertConnection(RF1->getOutPort("RDP0"),EAST_XBARI);
	insertConnection(RF2->getOutPort("RDP0"),WEST_XBARI);
	insertConnection(RF3->getOutPort("RDP0"),SOUTH_XBARI);


	//connect register file ports to PE time extension ports
	for(RegFile* RF : allRegs){
		for (int i = 0; i < RF->get_nRegs(); ++i) {
			Port* reg_i = getInPort(RF->getName() + "_REG_I" + std::to_string(i));
			Port* reg_o = getOutPort(RF->getName() + "_REG_O" + std::to_string(i));

//			connectedTo[reg_i].push_back(RF->getInPort("REG_I" + std::to_string(i)));
//			connectedTo[RF->getOutPort("REG_O" + std::to_string(i))].push_back(reg_o);

			insertConnection(reg_i,RF->getInPort("REG_I" + std::to_string(i)));
			insertConnection(RF->getOutPort("REG_O" + std::to_string(i)),reg_o);
		}
	}

	//connecting xbar inputs to the FU
	for(Port &p : nonMemFU0->inputPorts){
//		connectedTo[NORTH_XBARI].push_back(&p);
//		connectedTo[EAST_XBARI].push_back(&p);
//		connectedTo[WEST_XBARI].push_back(&p);
//		connectedTo[SOUTH_XBARI].push_back(&p);

		insertConnection(NORTH_XBARI,&p);
		insertConnection(EAST_XBARI,&p);
		insertConnection(WEST_XBARI,&p);
		insertConnection(SOUTH_XBARI,&p);
	}

	//connecting xbar inputs to outputs :: HyCUBE Property
//	connectedTo[NORTH_XBARI].push_back(NORTH_O);
//	connectedTo[NORTH_XBARI].push_back(EAST_O);
//	connectedTo[NORTH_XBARI].push_back(WEST_O);
//	connectedTo[NORTH_XBARI].push_back(SOUTH_O);
//	connectedTo[EAST_XBARI].push_back(NORTH_O);
//	connectedTo[EAST_XBARI].push_back(EAST_O);
//	connectedTo[EAST_XBARI].push_back(WEST_O);
//	connectedTo[EAST_XBARI].push_back(SOUTH_O);
//	connectedTo[WEST_XBARI].push_back(NORTH_O);
//	connectedTo[WEST_XBARI].push_back(EAST_O);
//	connectedTo[WEST_XBARI].push_back(WEST_O);
//	connectedTo[WEST_XBARI].push_back(SOUTH_O);
//	connectedTo[SOUTH_XBARI].push_back(NORTH_O);
//	connectedTo[SOUTH_XBARI].push_back(EAST_O);
//	connectedTo[SOUTH_XBARI].push_back(WEST_O);
//	connectedTo[SOUTH_XBARI].push_back(SOUTH_O);

	insertConnection(NORTH_XBARI,NORTH_O);
	insertConnection(NORTH_XBARI,EAST_O);
	insertConnection(NORTH_XBARI,WEST_O);
	insertConnection(NORTH_XBARI,SOUTH_O);

	insertConnection(EAST_XBARI,NORTH_O);
	insertConnection(EAST_XBARI,EAST_O);
	insertConnection(EAST_XBARI,WEST_O);
	insertConnection(EAST_XBARI,SOUTH_O);

	insertConnection(WEST_XBARI,NORTH_O);
	insertConnection(WEST_XBARI,EAST_O);
	insertConnection(WEST_XBARI,WEST_O);
	insertConnection(WEST_XBARI,SOUTH_O);

	insertConnection(SOUTH_XBARI,NORTH_O);
	insertConnection(SOUTH_XBARI,EAST_O);
	insertConnection(SOUTH_XBARI,WEST_O);
	insertConnection(SOUTH_XBARI,SOUTH_O);

	//connect RFT readports to FU
	{
		RegFile* RF = RFT;
		for (int i = 0; i < RF->get_nRDPs(); ++i) {
			std::stringstream rdpName;
			rdpName << "RDP" << i;
			Port* rdp = RF->getOutPort(rdpName.str());

			for(Port &p : nonMemFU0->inputPorts){
//				connectedTo[rdp].push_back(&p);
				insertConnection(rdp,&p);
			}
		}
	}


	for (int i = 0; i < numberofDPs; ++i) {
		std::stringstream rdpName;
		rdpName << "RDP" << i;
//		connectedTo[RFT->getOutPort(rdpName.str())].push_back(NORTH_O);
//		connectedTo[RFT->getOutPort(rdpName.str())].push_back(EAST_O);
//		connectedTo[RFT->getOutPort(rdpName.str())].push_back(WEST_O);
//		connectedTo[RFT->getOutPort(rdpName.str())].push_back(SOUTH_O);

		insertConnection(RFT->getOutPort(rdpName.str()),NORTH_O);
		insertConnection(RFT->getOutPort(rdpName.str()),EAST_O);
		insertConnection(RFT->getOutPort(rdpName.str()),WEST_O);
		insertConnection(RFT->getOutPort(rdpName.str()),SOUTH_O);
	}

	//connect output of FU to wrp of RegFile and main directional outputs
	for(Port &p : nonMemFU0->outputPorts){
//		connectedTo[&p].push_back(NORTH_O);
//		connectedTo[&p].push_back(EAST_O);
//		connectedTo[&p].push_back(WEST_O);
//		connectedTo[&p].push_back(SOUTH_O);

		insertConnection(&p,NORTH_O);
		insertConnection(&p,EAST_O);
		insertConnection(&p,WEST_O);
		insertConnection(&p,SOUTH_O);

		for (int i = 0; i < numberofDPs; ++i) {
			std::stringstream wrpName;
			wrpName << "WRP" << i;
//			connectedTo[&p].push_back(RFT->getInPort(wrpName.str()));
			insertConnection(&p,RFT->getInPort(wrpName.str()));
		}
	}

}

void CGRAXMLCompile::PE::createN2NPE(bool isMEMpe, int numberofDPs, int regs,
		int nWRP, int nRDP) {


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
	FU* FU0 = new FU(this,"NMEM_FU0",numberofDPs,supportedOPs);
	subModules.push_back(FU0);

	//create R0
	RegFile* RF0 = new RegFile(this,"RF0",nWRP,regs,nRDP); subModules.push_back(RF0); allRegs.push_back(RF0);
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
//			connectedTo[NORTH_I].push_back(wrp);
//			connectedTo[EAST_I].push_back(wrp);
//			connectedTo[WEST_I].push_back(wrp);
//			connectedTo[SOUTH_I].push_back(wrp);

			insertConnection(NORTH_I,wrp);
			insertConnection(EAST_I,wrp);
			insertConnection(WEST_I,wrp);
			insertConnection(SOUTH_I,wrp);
		}
	}

	//connect register file ports to PE ports
	for(RegFile* RF : allRegs){
		for (int i = 0; i < RF->get_nRegs(); ++i) {
			Port* reg_i = getInPort(RF->getName() + "_REG_I" + std::to_string(i));
			Port* reg_o = getOutPort(RF->getName() + "_REG_O" + std::to_string(i));

//			connectedTo[reg_i].push_back(RF->getInPort("REG_I" + std::to_string(i)));
//			connectedTo[RF->getOutPort("REG_O" + std::to_string(i))].push_back(reg_o);

			insertConnection(reg_i,RF->getInPort("REG_I" + std::to_string(i)));
			insertConnection(RF->getOutPort("REG_O" + std::to_string(i)),reg_o);
		}
	}

	//connecting inputs to the FU
	for(Port &p : FU0->inputPorts){
//		connectedTo[NORTH_I].push_back(&p);
//		connectedTo[EAST_I].push_back(&p);
//		connectedTo[WEST_I].push_back(&p);
//		connectedTo[SOUTH_I].push_back(&p);

		insertConnection(NORTH_I,&p);
		insertConnection(EAST_I,&p);
		insertConnection(WEST_I,&p);
		insertConnection(SOUTH_I,&p);
	}

	//connect register file readports to FU and writeports to main directional outputs
	for(RegFile* RF : allRegs){
		for (int i = 0; i < RF->get_nRDPs(); ++i) {
			std::stringstream rdpName;
			rdpName << "RDP" << i;
			Port* rdp = RF->getOutPort(rdpName.str());

			for(Port &p : FU0->inputPorts){
//				connectedTo[rdp].push_back(&p);
				insertConnection(rdp,&p);
			}

//			connectedTo[rdp].push_back(NORTH_O);
//			connectedTo[rdp].push_back(EAST_O);
//			connectedTo[rdp].push_back(WEST_O);
//			connectedTo[rdp].push_back(SOUTH_O);

			insertConnection(rdp,NORTH_O);
			insertConnection(rdp,EAST_O);
			insertConnection(rdp,WEST_O);
			insertConnection(rdp,SOUTH_O);
		}

	}

	//connect output of FU to wrp of RegFile and main directional outputs
	for(Port &p : FU0->outputPorts){
		for(RegFile* RF : allRegs){
			for (int i = 0; i < RF->get_nWRPs(); ++i) {
				std::stringstream wrpName;
				wrpName << "WRP" << i;
				Port* wrp = RF->getInPort(wrpName.str());
//				connectedTo[&p].push_back(wrp);
				insertConnection(&p,wrp);
			}
		}
//		connectedTo[&p].push_back(NORTH_O);
//		connectedTo[&p].push_back(EAST_O);
//		connectedTo[&p].push_back(WEST_O);
//		connectedTo[&p].push_back(SOUTH_O);

		insertConnection(&p,NORTH_O);
		insertConnection(&p,EAST_O);
		insertConnection(&p,WEST_O);
		insertConnection(&p,SOUTH_O);
	}

	//adding conflicted ports
	for(RegFile* RF : allRegs){
		for(Port &ip : FU0->inputPorts){
			for (int i = 0; i < RF->get_nWRPs(); ++i) {
				std::stringstream wrpName;
				wrpName << "WRP" << i;
				Port* wrp = RF->getInPort(wrpName.str());

				getCGRA()->insertConflictPort(wrp,&ip);
				getCGRA()->insertConflictPort(&ip,wrp);
//				conflictPorts[wrp].push_back(&ip);
//				conflictPorts[&ip].push_back(wrp);
			}
		}

		for(Port &op : FU0->outputPorts){
			for (int i = 0; i < RF->get_nRDPs(); ++i) {
				std::stringstream rdpName;
				rdpName << "RDP" << i;
				Port* rdp = RF->getOutPort(rdpName.str());
				assert(rdp!=NULL);

				getCGRA()->insertConflictPort(rdp,&op);
				getCGRA()->insertConflictPort(&op,rdp);
			}
		}

	}

//	assert(conflictPorts.size() > 0);

}

void CGRAXMLCompile::PE::getMEMIns(std::map<std::string, int>& supportedOPs) {

	supportedOPs["NOP"]=2; //1
	supportedOPs["ADD"]=2; //2
	supportedOPs["SUB"]=2; //3
	supportedOPs["MUL"]=2; //4
	supportedOPs["SEXT"]=2; //5
	supportedOPs["DIV"]=2; //6
	supportedOPs["LS"]=2; //7
	supportedOPs["RS"]=2; //8
	supportedOPs["ARS"]=2; //9
	supportedOPs["AND"]=2; //10
	supportedOPs["OR"]=2;  //11
	supportedOPs["XOR"]=2; //12
	supportedOPs["SELECT"]=2; //13
	supportedOPs["CMERGE"]=2; //14
	supportedOPs["CMP"]=2; //15
	supportedOPs["CLT"]=2; //16
	supportedOPs["BR"]=2;  //17
	supportedOPs["CGT"]=2; //18
	supportedOPs["MOVCL"]=2; //19

	supportedOPs["LOADCL"]=2;
	supportedOPs["LOAD"]=2;
	supportedOPs["LOADH"]=2;
	supportedOPs["LOADB"]=2;
	supportedOPs["STORE"]=2;
	supportedOPs["STOREH"]=2;
	supportedOPs["STOREB"]=2;

	supportedOPs["OLOADCL"]=2;
	supportedOPs["OLOAD"]=2;
	supportedOPs["OLOADH"]=2;
	supportedOPs["OLOADB"]=2;
	supportedOPs["OSTORE"]=2;
	supportedOPs["OSTOREH"]=2;
	supportedOPs["OSTOREB"]=2;

	supportedOPs["JUMPL"]=2; //20
	supportedOPs["MOVC"]=2; //21

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

void CGRAXMLCompile::PE::getLogicalIns(
		std::map<std::string, int>& supportedOPs) {

	supportedOPs["NOP"]=1; //1
	supportedOPs["SEXT"]=1; //2
	supportedOPs["AND"]=1; //3
	supportedOPs["OR"]=1; //4
	supportedOPs["XOR"]=1; //5
	supportedOPs["SELECT"]=1; //6
	supportedOPs["CMERGE"]=1; //7
	supportedOPs["BR"]=1; //8
	supportedOPs["MOVCL"]=1; //9
	supportedOPs["JUMPL"]=1; //10
	supportedOPs["MOVC"]=1; //11
}

void CGRAXMLCompile::PE::getArithmeticIns(
		std::map<std::string, int>& supportedOPs) {

	supportedOPs["ADD"]=1; //12
	supportedOPs["SUB"]=1; //13
	supportedOPs["MUL"]=1; //14
	supportedOPs["DIV"]=1; //15
	supportedOPs["LS"]=1; //16
	supportedOPs["RS"]=1; //17
	supportedOPs["ARS"]=1; //18
	supportedOPs["CMP"]=1; //19
	supportedOPs["CLT"]=1; //20
	supportedOPs["CGT"]=1; //21

	supportedOPs["MOVC"]=1; //11

}

void CGRAXMLCompile::PE::getMemOnlyIns(std::map<std::string, int>& supportedOPs) {

	supportedOPs["LOADCL"]=2;
	supportedOPs["LOAD"]=2;
	supportedOPs["LOADH"]=2;
	supportedOPs["LOADB"]=2;
	supportedOPs["STORE"]=2;
	supportedOPs["STOREH"]=2;
	supportedOPs["STOREB"]=2;

	supportedOPs["OLOADCL"]=1;
	supportedOPs["OLOAD"]=1;
	supportedOPs["OLOADH"]=1;
	supportedOPs["OLOADB"]=1;
	supportedOPs["OSTORE"]=1;
	supportedOPs["OSTOREH"]=1;
	supportedOPs["OSTOREB"]=1;

	supportedOPs["MOVC"]=1; //11
}

void CGRAXMLCompile::PE::getOutMemOnlyIns(
		std::map<std::string, int>& supportedOPs) {

	supportedOPs["OLOADCL"]=1;
	supportedOPs["OLOAD"]=1;
	supportedOPs["OLOADH"]=1;
	supportedOPs["OLOADB"]=1;
	supportedOPs["OSTORE"]=1;
	supportedOPs["OSTOREH"]=1;
	supportedOPs["OSTOREB"]=1;

	supportedOPs["MOVC"]=1; //11

}

void CGRAXMLCompile::PE::createMultiFU_HyCUBEPE_RegFile(bool isMEMpe,
		int numberofDPs, int regs, int nWRP, int nRDP) {


	assert(!alreadyInit);
	alreadyInit=true;
	this->isMemPE=isMemPE;

//	int numberofDPs=1;
	std::map<std::string,int> supportedOPs;

	FU* memFU;
	FU* arithFU;
	FU* logicFU;

	if(isMEMpe){
		supportedOPs.clear();
		getMemOnlyIns(supportedOPs);
		memFU=new FU(this,"memFU0",numberofDPs,supportedOPs);
		subModules.push_back(memFU);
		allFUs.push_back(memFU);
	}
	else{
		supportedOPs.clear();
		getOutMemOnlyIns(supportedOPs);
		memFU=new FU(this,"memFU0",numberofDPs,supportedOPs);
		subModules.push_back(memFU);
		allFUs.push_back(memFU);
	}

	supportedOPs.clear();
	getArithmeticIns(supportedOPs);
	arithFU=new FU(this,"arithFU0",numberofDPs,supportedOPs);
	subModules.push_back(arithFU);
	allFUs.push_back(arithFU);

	supportedOPs.clear();
	getLogicalIns(supportedOPs);
	logicFU=new FU(this,"logicFU0",numberofDPs,supportedOPs);
	subModules.push_back(logicFU);
	allFUs.push_back(logicFU);

	//create FU
//	FU* nonMemFU0 = new FU(this,"NMEM_FU0",numberofDPs,supportedOPs);
//	subModules.push_back(nonMemFU0);

	//create R0
	RegFile* RF0 = new RegFile(this,"RF0",nWRP,regs,nRDP); subModules.push_back(RF0); allRegs.push_back(RF0);
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
//			connectedTo[NORTH_I].push_back(wrp);
//			connectedTo[EAST_I].push_back(wrp);
//			connectedTo[WEST_I].push_back(wrp);
//			connectedTo[SOUTH_I].push_back(wrp);

			insertConnection(NORTH_I,wrp);
			insertConnection(EAST_I,wrp);
			insertConnection(WEST_I,wrp);
			insertConnection(SOUTH_I,wrp);
		}
	}

	//connect register file ports to PE ports
	for(RegFile* RF : allRegs){
		for (int i = 0; i < RF->get_nRegs(); ++i) {
			Port* reg_i = getInPort(RF->getName() + "_REG_I" + std::to_string(i));
			Port* reg_o = getOutPort(RF->getName() + "_REG_O" + std::to_string(i));

//			connectedTo[reg_i].push_back(RF->getInPort("REG_I" + std::to_string(i)));
//			connectedTo[RF->getOutPort("REG_O" + std::to_string(i))].push_back(reg_o);

			insertConnection(reg_i,RF->getInPort("REG_I" + std::to_string(i)));
			insertConnection(RF->getOutPort("REG_O" + std::to_string(i)),reg_o);
		}
	}

	//connecting inputs to the FU
	for(FU* fu : allFUs){
		for(Port &p : fu->inputPorts){
	//		connectedTo[NORTH_I].push_back(&p);
	//		connectedTo[EAST_I].push_back(&p);
	//		connectedTo[WEST_I].push_back(&p);
	//		connectedTo[SOUTH_I].push_back(&p);

			insertConnection(NORTH_I,&p);
			insertConnection(EAST_I,&p);
			insertConnection(WEST_I,&p);
			insertConnection(SOUTH_I,&p);
		}
	}

	//connecting input to outputs :: HyCUBE Property
//	connectedTo[NORTH_I].push_back(NORTH_O);
//	connectedTo[NORTH_I].push_back(EAST_O);
//	connectedTo[NORTH_I].push_back(WEST_O);
//	connectedTo[NORTH_I].push_back(SOUTH_O);
//	connectedTo[EAST_I].push_back(NORTH_O);
//	connectedTo[EAST_I].push_back(EAST_O);
//	connectedTo[EAST_I].push_back(WEST_O);
//	connectedTo[EAST_I].push_back(SOUTH_O);
//	connectedTo[WEST_I].push_back(NORTH_O);
//	connectedTo[WEST_I].push_back(EAST_O);
//	connectedTo[WEST_I].push_back(WEST_O);
//	connectedTo[WEST_I].push_back(SOUTH_O);
//	connectedTo[SOUTH_I].push_back(NORTH_O);
//	connectedTo[SOUTH_I].push_back(EAST_O);
//	connectedTo[SOUTH_I].push_back(WEST_O);
//	connectedTo[SOUTH_I].push_back(SOUTH_O);

	insertConnection(NORTH_I,NORTH_O);
	insertConnection(NORTH_I,EAST_O);
	insertConnection(NORTH_I,WEST_O);
	insertConnection(NORTH_I,SOUTH_O);

	insertConnection(EAST_I,NORTH_O);
	insertConnection(EAST_I,EAST_O);
	insertConnection(EAST_I,WEST_O);
	insertConnection(EAST_I,SOUTH_O);

	insertConnection(WEST_I,NORTH_O);
	insertConnection(WEST_I,EAST_O);
	insertConnection(WEST_I,WEST_O);
	insertConnection(WEST_I,SOUTH_O);

	insertConnection(SOUTH_I,NORTH_O);
	insertConnection(SOUTH_I,EAST_O);
	insertConnection(SOUTH_I,WEST_O);
	insertConnection(SOUTH_I,SOUTH_O);

	//connect register file readports to FU and writeports to main directional outputs
	for(RegFile* RF : allRegs){
		for (int i = 0; i < RF->get_nRDPs(); ++i) {
			std::stringstream rdpName;
			rdpName << "RDP" << i;
			Port* rdp = RF->getOutPort(rdpName.str());

			for(FU* fu : allFUs){
				for(Port &p : fu->inputPorts){
	//				connectedTo[rdp].push_back(&p);
					insertConnection(rdp,&p);
				}
			}

//			connectedTo[rdp].push_back(NORTH_O);
//			connectedTo[rdp].push_back(EAST_O);
//			connectedTo[rdp].push_back(WEST_O);
//			connectedTo[rdp].push_back(SOUTH_O);

			insertConnection(rdp,NORTH_O);
			insertConnection(rdp,EAST_O);
			insertConnection(rdp,WEST_O);
			insertConnection(rdp,SOUTH_O);
		}
	}

	//connect output of FU to wrp of RegFile and main directional outputs
	for(FU* fu : allFUs){
		for(Port &p : fu->outputPorts){
			for(RegFile* RF : allRegs){
				for (int i = 0; i < RF->get_nWRPs(); ++i) {
					std::stringstream wrpName;
					wrpName << "WRP" << i;
					Port* wrp = RF->getInPort(wrpName.str());
	//				connectedTo[&p].push_back(wrp);
					insertConnection(&p,wrp);
				}
			}
	//		connectedTo[&p].push_back(NORTH_O);
	//		connectedTo[&p].push_back(EAST_O);
	//		connectedTo[&p].push_back(WEST_O);
	//		connectedTo[&p].push_back(SOUTH_O);

			insertConnection(&p,NORTH_O);
			insertConnection(&p,EAST_O);
			insertConnection(&p,WEST_O);
			insertConnection(&p,SOUTH_O);

		}
	}

}

void CGRAXMLCompile::PE::createMultiFU_HyCUBEPE(bool isMEMpe, int numberofDPs) {


	assert(!alreadyInit);
	alreadyInit=true;
	this->isMemPE=isMemPE;

//	int numberofDPs=1;
	std::map<std::string,int> supportedOPs;

	FU* memFU=NULL;
	FU* arithFU=NULL;
	FU* logicFU=NULL;

	if(isMEMpe){
		supportedOPs.clear();
		getMemOnlyIns(supportedOPs);
		memFU=new FU(this,"memFU0",numberofDPs,supportedOPs);
		subModules.push_back(memFU);
		allFUs.push_back(memFU);
	}
	else{
		supportedOPs.clear();
		getOutMemOnlyIns(supportedOPs);
		memFU=new FU(this,"memFU0",numberofDPs,supportedOPs);
		subModules.push_back(memFU);
		allFUs.push_back(memFU);
	}

	supportedOPs.clear();
	getArithmeticIns(supportedOPs);
	arithFU=new FU(this,"arithFU0",numberofDPs,supportedOPs);
	subModules.push_back(arithFU);
	allFUs.push_back(arithFU);

	supportedOPs.clear();
	getLogicalIns(supportedOPs);
	logicFU=new FU(this,"logicFU0",numberofDPs,supportedOPs);
	subModules.push_back(logicFU);
	allFUs.push_back(logicFU);


//	//create FU
//	FU* nonMemFU0 = new FU(this,"NMEM_FU0",numberofDPs,supportedOPs);
//	subModules.push_back(nonMemFU0);

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
	internalPorts.push_back( Port("NORTH_XBARI",INT,this) );
	internalPorts.push_back( Port("EAST_XBARI",INT,this) );
	internalPorts.push_back( Port("WEST_XBARI",INT,this) );
	internalPorts.push_back( Port("SOUTH_XBARI",INT,this) );

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
//	connectedTo[NORTH_I].push_back(RF0->getInPort("WRP0"));
//	connectedTo[EAST_I].push_back(RF1->getInPort("WRP0"));
//	connectedTo[WEST_I].push_back(RF2->getInPort("WRP0"));
//	connectedTo[SOUTH_I].push_back(RF3->getInPort("WRP0"));

	insertConnection(NORTH_I,RF0->getInPort("WRP0"));
	insertConnection(EAST_I,RF1->getInPort("WRP0"));
	insertConnection(WEST_I,RF2->getInPort("WRP0"));
	insertConnection(SOUTH_I,RF3->getInPort("WRP0"));

	//connect main directional inputs to xbar input ports
//	connectedTo[NORTH_I].push_back(NORTH_XBARI);
//	connectedTo[EAST_I].push_back(EAST_XBARI);
//	connectedTo[WEST_I].push_back(WEST_XBARI);
//	connectedTo[SOUTH_I].push_back(SOUTH_XBARI);

	insertConnection(NORTH_I,NORTH_XBARI);
	insertConnection(EAST_I,EAST_XBARI);
	insertConnection(WEST_I,WEST_XBARI);
	insertConnection(SOUTH_I,SOUTH_XBARI);


	//connect the readports of the register file to xbar inputs
//	connectedTo[RF0->getOutPort("RDP0")].push_back(NORTH_XBARI);
//	connectedTo[RF1->getOutPort("RDP0")].push_back(EAST_XBARI);
//	connectedTo[RF2->getOutPort("RDP0")].push_back(WEST_XBARI);
//	connectedTo[RF3->getOutPort("RDP0")].push_back(SOUTH_XBARI);

	insertConnection(RF0->getOutPort("RDP0"),NORTH_XBARI);
	insertConnection(RF1->getOutPort("RDP0"),EAST_XBARI);
	insertConnection(RF2->getOutPort("RDP0"),WEST_XBARI);
	insertConnection(RF3->getOutPort("RDP0"),SOUTH_XBARI);


	//connect register file ports to PE time extension ports
	for(RegFile* RF : allRegs){
		for (int i = 0; i < RF->get_nRegs(); ++i) {
			Port* reg_i = getInPort(RF->getName() + "_REG_I" + std::to_string(i));
			Port* reg_o = getOutPort(RF->getName() + "_REG_O" + std::to_string(i));

//			connectedTo[reg_i].push_back(RF->getInPort("REG_I" + std::to_string(i)));
//			connectedTo[RF->getOutPort("REG_O" + std::to_string(i))].push_back(reg_o);

			insertConnection(reg_i,RF->getInPort("REG_I" + std::to_string(i)));
			insertConnection(RF->getOutPort("REG_O" + std::to_string(i)),reg_o);
		}
	}

	//connecting xbar inputs to the FU

	for(FU* fu : allFUs){
		for(Port &p : fu->inputPorts){
	//		connectedTo[NORTH_XBARI].push_back(&p);
	//		connectedTo[EAST_XBARI].push_back(&p);
	//		connectedTo[WEST_XBARI].push_back(&p);
	//		connectedTo[SOUTH_XBARI].push_back(&p);

			insertConnection(NORTH_XBARI,&p);
			insertConnection(EAST_XBARI,&p);
			insertConnection(WEST_XBARI,&p);
			insertConnection(SOUTH_XBARI,&p);
		}
	}


	//connecting xbar inputs to outputs :: HyCUBE Property
//	connectedTo[NORTH_XBARI].push_back(NORTH_O);
//	connectedTo[NORTH_XBARI].push_back(EAST_O);
//	connectedTo[NORTH_XBARI].push_back(WEST_O);
//	connectedTo[NORTH_XBARI].push_back(SOUTH_O);
//	connectedTo[EAST_XBARI].push_back(NORTH_O);
//	connectedTo[EAST_XBARI].push_back(EAST_O);
//	connectedTo[EAST_XBARI].push_back(WEST_O);
//	connectedTo[EAST_XBARI].push_back(SOUTH_O);
//	connectedTo[WEST_XBARI].push_back(NORTH_O);
//	connectedTo[WEST_XBARI].push_back(EAST_O);
//	connectedTo[WEST_XBARI].push_back(WEST_O);
//	connectedTo[WEST_XBARI].push_back(SOUTH_O);
//	connectedTo[SOUTH_XBARI].push_back(NORTH_O);
//	connectedTo[SOUTH_XBARI].push_back(EAST_O);
//	connectedTo[SOUTH_XBARI].push_back(WEST_O);
//	connectedTo[SOUTH_XBARI].push_back(SOUTH_O);

	insertConnection(NORTH_XBARI,NORTH_O);
	insertConnection(NORTH_XBARI,EAST_O);
	insertConnection(NORTH_XBARI,WEST_O);
	insertConnection(NORTH_XBARI,SOUTH_O);

	insertConnection(EAST_XBARI,NORTH_O);
	insertConnection(EAST_XBARI,EAST_O);
	insertConnection(EAST_XBARI,WEST_O);
	insertConnection(EAST_XBARI,SOUTH_O);

	insertConnection(WEST_XBARI,NORTH_O);
	insertConnection(WEST_XBARI,EAST_O);
	insertConnection(WEST_XBARI,WEST_O);
	insertConnection(WEST_XBARI,SOUTH_O);

	insertConnection(SOUTH_XBARI,NORTH_O);
	insertConnection(SOUTH_XBARI,EAST_O);
	insertConnection(SOUTH_XBARI,WEST_O);
	insertConnection(SOUTH_XBARI,SOUTH_O);

	//connect RFT readports to FU
	{
		RegFile* RF = RFT;
		for (int i = 0; i < RF->get_nRDPs(); ++i) {
			std::stringstream rdpName;
			rdpName << "RDP" << i;
			Port* rdp = RF->getOutPort(rdpName.str());

			for(FU* fu : allFUs){
				for(Port &p : fu->inputPorts){
	//				connectedTo[rdp].push_back(&p);
					insertConnection(rdp,&p);
				}
			}
		}
	}


	for (int i = 0; i < numberofDPs; ++i) {
		std::stringstream rdpName;
		rdpName << "RDP" << i;
//		connectedTo[RFT->getOutPort(rdpName.str())].push_back(NORTH_O);
//		connectedTo[RFT->getOutPort(rdpName.str())].push_back(EAST_O);
//		connectedTo[RFT->getOutPort(rdpName.str())].push_back(WEST_O);
//		connectedTo[RFT->getOutPort(rdpName.str())].push_back(SOUTH_O);

		insertConnection(RFT->getOutPort(rdpName.str()),NORTH_O);
		insertConnection(RFT->getOutPort(rdpName.str()),EAST_O);
		insertConnection(RFT->getOutPort(rdpName.str()),WEST_O);
		insertConnection(RFT->getOutPort(rdpName.str()),SOUTH_O);
	}

	//connect output of FU to wrp of RegFile and main directional outputs

	for(FU* fu : allFUs){
		for(Port &p : fu->outputPorts){
	//		connectedTo[&p].push_back(NORTH_O);
	//		connectedTo[&p].push_back(EAST_O);
	//		connectedTo[&p].push_back(WEST_O);
	//		connectedTo[&p].push_back(SOUTH_O);

			insertConnection(&p,NORTH_O);
			insertConnection(&p,EAST_O);
			insertConnection(&p,WEST_O);
			insertConnection(&p,SOUTH_O);

			for (int i = 0; i < numberofDPs; ++i) {
				std::stringstream wrpName;
				wrpName << "WRP" << i;
	//			connectedTo[&p].push_back(RFT->getInPort(wrpName.str()));
				insertConnection(&p,RFT->getInPort(wrpName.str()));
			}
		}
	}
}

void CGRAXMLCompile::PE::createMultiFU_StdNoCPE_RegFile(bool isMEMpe,
		int numberofDPs, int regs, int nWRP, int nRDP) {



	assert(!alreadyInit);
	alreadyInit=true;
	this->isMemPE=isMemPE;

//	int numberofDPs=1;
	std::map<std::string,int> supportedOPs;

	FU* memFU=NULL;
	FU* arithFU=NULL;
	FU* logicFU=NULL;

	if(isMEMpe){
		supportedOPs.clear();
		getMemOnlyIns(supportedOPs);
		memFU=new FU(this,"memFU0",numberofDPs,supportedOPs);
		subModules.push_back(memFU);
		allFUs.push_back(memFU);
	}
	else{
		supportedOPs.clear();
		getOutMemOnlyIns(supportedOPs);
		memFU=new FU(this,"memFU0",numberofDPs,supportedOPs);
		subModules.push_back(memFU);
		allFUs.push_back(memFU);
	}

	supportedOPs.clear();
	getArithmeticIns(supportedOPs);
	arithFU=new FU(this,"arithFU0",numberofDPs,supportedOPs);
	subModules.push_back(arithFU);
	allFUs.push_back(arithFU);

	supportedOPs.clear();
	getLogicalIns(supportedOPs);
	logicFU=new FU(this,"logicFU0",numberofDPs,supportedOPs);
	subModules.push_back(logicFU);
	allFUs.push_back(logicFU);


	//create FU
//	FU* nonMemFU0 = new FU(this,"NMEM_FU0",numberofDPs,supportedOPs);
//	subModules.push_back(nonMemFU0);

	//create R0
	RegFile* RF0 = new RegFile(this,"RF0",nWRP,regs,nRDP); subModules.push_back(RF0); allRegs.push_back(RF0);
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
//			connectedTo[NORTH_I].push_back(wrp);
//			connectedTo[EAST_I].push_back(wrp);
//			connectedTo[WEST_I].push_back(wrp);
//			connectedTo[SOUTH_I].push_back(wrp);

			insertConnection(NORTH_I,wrp);
			insertConnection(EAST_I,wrp);
			insertConnection(WEST_I,wrp);
			insertConnection(SOUTH_I,wrp);
		}
	}

	//connect register file ports to PE ports
	for(RegFile* RF : allRegs){
		for (int i = 0; i < RF->get_nRegs(); ++i) {
			Port* reg_i = getInPort(RF->getName() + "_REG_I" + std::to_string(i));
			Port* reg_o = getOutPort(RF->getName() + "_REG_O" + std::to_string(i));

//			connectedTo[reg_i].push_back(RF->getInPort("REG_I" + std::to_string(i)));
//			connectedTo[RF->getOutPort("REG_O" + std::to_string(i))].push_back(reg_o);

			insertConnection(reg_i,RF->getInPort("REG_I" + std::to_string(i)));
			insertConnection(RF->getOutPort("REG_O" + std::to_string(i)),reg_o);
		}
	}

	//connecting inputs to the FU
	for(FU* fu : allFUs){
		for(Port &p : fu->inputPorts){
	//		connectedTo[NORTH_I].push_back(&p);
	//		connectedTo[EAST_I].push_back(&p);
	//		connectedTo[WEST_I].push_back(&p);
	//		connectedTo[SOUTH_I].push_back(&p);

			insertConnection(NORTH_I,&p);
			insertConnection(EAST_I,&p);
			insertConnection(WEST_I,&p);
			insertConnection(SOUTH_I,&p);
		}
	}

	//connecting input to outputs :: HyCUBE Property
//	connections[NORTH_I].push_back(NORTH_O);
//	connections[NORTH_I].push_back(EAST_O);
//	connections[NORTH_I].push_back(WEST_O);
//	connections[NORTH_I].push_back(SOUTH_O);
//	connections[EAST_I].push_back(NORTH_O);
//	connections[EAST_I].push_back(EAST_O);
//	connections[EAST_I].push_back(WEST_O);
//	connections[EAST_I].push_back(SOUTH_O);
//	connections[WEST_I].push_back(NORTH_O);
//	connections[WEST_I].push_back(EAST_O);
//	connections[WEST_I].push_back(WEST_O);
//	connections[WEST_I].push_back(SOUTH_O);
//	connections[SOUTH_I].push_back(NORTH_O);
//	connections[SOUTH_I].push_back(EAST_O);
//	connections[SOUTH_I].push_back(WEST_O);
//	connections[SOUTH_I].push_back(SOUTH_O);

	//connect register file readports to FU and writeports to main directional outputs
	for(RegFile* RF : allRegs){
		for (int i = 0; i < RF->get_nRDPs(); ++i) {
			std::stringstream rdpName;
			rdpName << "RDP" << i;
			Port* rdp = RF->getOutPort(rdpName.str());

			for(FU* fu : allFUs){
				for(Port &p : fu->inputPorts){
	//				connectedTo[rdp].push_back(&p);
					insertConnection(rdp,&p);
				}
			}

//			connectedTo[rdp].push_back(NORTH_O);
//			connectedTo[rdp].push_back(EAST_O);
//			connectedTo[rdp].push_back(WEST_O);
//			connectedTo[rdp].push_back(SOUTH_O);

			insertConnection(rdp,NORTH_O);
			insertConnection(rdp,EAST_O);
			insertConnection(rdp,WEST_O);
			insertConnection(rdp,SOUTH_O);
		}

	}

	//connect output of FU to wrp of RegFile and main directional outputs
	for(FU* fu : allFUs){
		for(Port &p : fu->outputPorts){
			for(RegFile* RF : allRegs){
				for (int i = 0; i < RF->get_nWRPs(); ++i) {
					std::stringstream wrpName;
					wrpName << "WRP" << i;
					Port* wrp = RF->getInPort(wrpName.str());
	//				connectedTo[&p].push_back(wrp);
					insertConnection(&p,wrp);
				}
			}
	//		connectedTo[&p].push_back(NORTH_O);
	//		connectedTo[&p].push_back(EAST_O);
	//		connectedTo[&p].push_back(WEST_O);
	//		connectedTo[&p].push_back(SOUTH_O);

			insertConnection(&p,NORTH_O);
			insertConnection(&p,EAST_O);
			insertConnection(&p,WEST_O);
			insertConnection(&p,SOUTH_O);
		}
	}

}

void CGRAXMLCompile::PE::createMultiFU_StdNoCPE(bool isMEMpe, int numberofDPs) {



	assert(!alreadyInit);
	alreadyInit=true;
	this->isMemPE=isMemPE;

//	int numberofDPs=1;
	std::map<std::string,int> supportedOPs;

	FU* memFU=NULL;
	FU* arithFU=NULL;
	FU* logicFU=NULL;

	if(isMEMpe){
		supportedOPs.clear();
		getMemOnlyIns(supportedOPs);
		memFU=new FU(this,"memFU0",numberofDPs,supportedOPs);
		subModules.push_back(memFU);
		allFUs.push_back(memFU);
	}
	else{
		supportedOPs.clear();
		getOutMemOnlyIns(supportedOPs);
		memFU=new FU(this,"memFU0",numberofDPs,supportedOPs);
		subModules.push_back(memFU);
		allFUs.push_back(memFU);
	}

	supportedOPs.clear();
	getArithmeticIns(supportedOPs);
	arithFU=new FU(this,"arithFU0",numberofDPs,supportedOPs);
	subModules.push_back(arithFU);
	allFUs.push_back(arithFU);

	supportedOPs.clear();
	getLogicalIns(supportedOPs);
	logicFU=new FU(this,"logicFU0",numberofDPs,supportedOPs);
	subModules.push_back(logicFU);
	allFUs.push_back(logicFU);


	//create FU
//	FU* nonMemFU0 = new FU(this,"NMEM_FU0",numberofDPs,supportedOPs);
//	subModules.push_back(nonMemFU0);

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
	internalPorts.push_back( Port("NORTH_XBARI",INT,this) );
	internalPorts.push_back( Port("EAST_XBARI",INT,this) );
	internalPorts.push_back( Port("WEST_XBARI",INT,this) );
	internalPorts.push_back( Port("SOUTH_XBARI",INT,this) );

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
//	connectedTo[NORTH_I].push_back(RF0->getInPort("WRP0"));
//	connectedTo[EAST_I].push_back(RF1->getInPort("WRP0"));
//	connectedTo[WEST_I].push_back(RF2->getInPort("WRP0"));
//	connectedTo[SOUTH_I].push_back(RF3->getInPort("WRP0"));

	insertConnection(NORTH_I,RF0->getInPort("WRP0"));
	insertConnection(EAST_I,RF1->getInPort("WRP0"));
	insertConnection(WEST_I,RF2->getInPort("WRP0"));
	insertConnection(SOUTH_I,RF3->getInPort("WRP0"));


	//connect main directional inputs to xbar input ports
//	connectedTo[NORTH_I].push_back(NORTH_XBARI);
//	connectedTo[EAST_I].push_back(EAST_XBARI);
//	connectedTo[WEST_I].push_back(WEST_XBARI);
//	connectedTo[SOUTH_I].push_back(SOUTH_XBARI);

	insertConnection(NORTH_I,NORTH_XBARI);
	insertConnection(EAST_I,EAST_XBARI);
	insertConnection(WEST_I,WEST_XBARI);
	insertConnection(SOUTH_I,SOUTH_XBARI);

	//connect the readports of the register file to xbar inputs
//	connectedTo[RF0->getOutPort("RDP0")].push_back(NORTH_XBARI);
//	connectedTo[RF1->getOutPort("RDP0")].push_back(EAST_XBARI);
//	connectedTo[RF2->getOutPort("RDP0")].push_back(WEST_XBARI);
//	connectedTo[RF3->getOutPort("RDP0")].push_back(SOUTH_XBARI);

	insertConnection(RF0->getOutPort("RDP0"),NORTH_XBARI);
	insertConnection(RF1->getOutPort("RDP0"),EAST_XBARI);
	insertConnection(RF2->getOutPort("RDP0"),WEST_XBARI);
	insertConnection(RF3->getOutPort("RDP0"),SOUTH_XBARI);

	//connect register file ports to PE time extension ports
	for(RegFile* RF : allRegs){
		for (int i = 0; i < RF->get_nRegs(); ++i) {
			Port* reg_i = getInPort(RF->getName() + "_REG_I" + std::to_string(i));
			Port* reg_o = getOutPort(RF->getName() + "_REG_O" + std::to_string(i));

//			connectedTo[reg_i].push_back(RF->getInPort("REG_I" + std::to_string(i)));
//			connectedTo[RF->getOutPort("REG_O" + std::to_string(i))].push_back(reg_o);

			insertConnection(reg_i,RF->getInPort("REG_I" + std::to_string(i)));
			insertConnection(RF->getOutPort("REG_O" + std::to_string(i)),reg_o);

		}
	}

	//connecting xbar inputs to the FU
	for(FU* fu : allFUs){
		for(Port &p : fu->inputPorts){
	//		connectedTo[NORTH_XBARI].push_back(&p);
	//		connectedTo[EAST_XBARI].push_back(&p);
	//		connectedTo[WEST_XBARI].push_back(&p);
	//		connectedTo[SOUTH_XBARI].push_back(&p);

			insertConnection(NORTH_XBARI,&p);
			insertConnection(EAST_XBARI,&p);
			insertConnection(WEST_XBARI,&p);
			insertConnection(SOUTH_XBARI,&p);
		}
	}

	//connecting input registers to outputs :: STDNOC Property
//	connectedTo[RF0->getOutPort("RDP0")].push_back(NORTH_O);
//	connectedTo[RF0->getOutPort("RDP0")].push_back(EAST_O);
//	connectedTo[RF0->getOutPort("RDP0")].push_back(WEST_O);
//	connectedTo[RF0->getOutPort("RDP0")].push_back(SOUTH_O);
//	connectedTo[RF1->getOutPort("RDP0")].push_back(NORTH_O);
//	connectedTo[RF1->getOutPort("RDP0")].push_back(EAST_O);
//	connectedTo[RF1->getOutPort("RDP0")].push_back(WEST_O);
//	connectedTo[RF1->getOutPort("RDP0")].push_back(SOUTH_O);
//	connectedTo[RF2->getOutPort("RDP0")].push_back(NORTH_O);
//	connectedTo[RF2->getOutPort("RDP0")].push_back(EAST_O);
//	connectedTo[RF2->getOutPort("RDP0")].push_back(WEST_O);
//	connectedTo[RF2->getOutPort("RDP0")].push_back(SOUTH_O);
//	connectedTo[RF3->getOutPort("RDP0")].push_back(NORTH_O);
//	connectedTo[RF3->getOutPort("RDP0")].push_back(EAST_O);
//	connectedTo[RF3->getOutPort("RDP0")].push_back(WEST_O);
//	connectedTo[RF3->getOutPort("RDP0")].push_back(SOUTH_O);


	getCGRA()->insertConflictPort(RF0->getOutPort("RDP0"),NORTH_I);
	getCGRA()->insertConflictPort(RF1->getOutPort("RDP0"),EAST_I);
	getCGRA()->insertConflictPort(RF2->getOutPort("RDP0"),WEST_I);
	getCGRA()->insertConflictPort(RF3->getOutPort("RDP0"),SOUTH_I);


//	conflictPorts[RF0->getOutPort("RDP0")].push_back(NORTH_I);
//	conflictPorts[RF1->getOutPort("RDP0")].push_back(EAST_I);
//	conflictPorts[RF2->getOutPort("RDP0")].push_back(WEST_I);
//	conflictPorts[RF3->getOutPort("RDP0")].push_back(SOUTH_I);

	getCGRA()->insertConflictPort(NORTH_I,RF0->getOutPort("RDP0"));
	getCGRA()->insertConflictPort(EAST_I,RF1->getOutPort("RDP0"));
	getCGRA()->insertConflictPort(WEST_I,RF2->getOutPort("RDP0"));
	getCGRA()->insertConflictPort(SOUTH_I,RF3->getOutPort("RDP0"));

//	conflictPorts[NORTH_I].push_back(RF0->getOutPort("RDP0"));
//	conflictPorts[EAST_I].push_back(RF1->getOutPort("RDP0"));
//	conflictPorts[WEST_I].push_back(RF2->getOutPort("RDP0"));
//	conflictPorts[SOUTH_I].push_back(RF3->getOutPort("RDP0"));

	insertConnection(RF0->getOutPort("RDP0"),NORTH_O);
	insertConnection(RF0->getOutPort("RDP0"),EAST_O);
	insertConnection(RF0->getOutPort("RDP0"),WEST_O);
	insertConnection(RF0->getOutPort("RDP0"),SOUTH_O);
	insertConnection(RF1->getOutPort("RDP0"),NORTH_O);
	insertConnection(RF1->getOutPort("RDP0"),EAST_O);
	insertConnection(RF1->getOutPort("RDP0"),WEST_O);
	insertConnection(RF1->getOutPort("RDP0"),SOUTH_O);
	insertConnection(RF2->getOutPort("RDP0"),NORTH_O);
	insertConnection(RF2->getOutPort("RDP0"),EAST_O);
	insertConnection(RF2->getOutPort("RDP0"),WEST_O);
	insertConnection(RF2->getOutPort("RDP0"),SOUTH_O);
	insertConnection(RF3->getOutPort("RDP0"),NORTH_O);
	insertConnection(RF3->getOutPort("RDP0"),EAST_O);
	insertConnection(RF3->getOutPort("RDP0"),WEST_O);
	insertConnection(RF3->getOutPort("RDP0"),SOUTH_O);


	//connect RFT readports to FU
	{
		RegFile* RF = RFT;
		for (int i = 0; i < RF->get_nRDPs(); ++i) {
			std::stringstream rdpName;
			rdpName << "RDP" << i;
			Port* rdp = RF->getOutPort(rdpName.str());

			for(FU* fu : allFUs){
				for(Port &p : fu->inputPorts){
	//				connectedTo[rdp].push_back(&p);
					insertConnection(rdp,&p);
				}
			}
		}
	}


	for (int i = 0; i < numberofDPs; ++i) {
		std::stringstream rdpName;
		rdpName << "RDP" << i;
//		connectedTo[RFT->getOutPort(rdpName.str())].push_back(NORTH_O);
//		connectedTo[RFT->getOutPort(rdpName.str())].push_back(EAST_O);
//		connectedTo[RFT->getOutPort(rdpName.str())].push_back(WEST_O);
//		connectedTo[RFT->getOutPort(rdpName.str())].push_back(SOUTH_O);

		insertConnection(RFT->getOutPort(rdpName.str()),NORTH_O);
		insertConnection(RFT->getOutPort(rdpName.str()),EAST_O);
		insertConnection(RFT->getOutPort(rdpName.str()),WEST_O);
		insertConnection(RFT->getOutPort(rdpName.str()),SOUTH_O);
	}

	//connect output of FU to wrp of RegFile and main directional outputs
	for(FU* fu : allFUs){
		for(Port &p : fu->outputPorts){
	//		connectedTo[&p].push_back(NORTH_O);
	//		connectedTo[&p].push_back(EAST_O);
	//		connectedTo[&p].push_back(WEST_O);
	//		connectedTo[&p].push_back(SOUTH_O);

			insertConnection(&p,NORTH_O);
			insertConnection(&p,EAST_O);
			insertConnection(&p,WEST_O);
			insertConnection(&p,SOUTH_O);

			for (int i = 0; i < numberofDPs; ++i) {
				std::stringstream wrpName;
				wrpName << "WRP" << i;
	//			connectedTo[&p].push_back(RFT->getInPort(wrpName.str()));
				insertConnection(&p,RFT->getInPort(wrpName.str()));
			}
		}
	}


}



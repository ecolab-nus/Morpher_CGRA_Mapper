/*
 * PE.h
 *
 *  Created on: 26 Feb 2018
 *      Author: manupa
 */

#ifndef PE_H_
#define PE_H_

#include "Module.h"
#include "RegFile.h"
#include <assert.h>

namespace CGRAXMLCompile {

class PE : public Module{
public:
//	using Module::Module;
	PE(const Module* Parent,
	   std::string name,
	   int x, int y, int t,
	   std::string peType = "GENERIC_8REGF",
	   bool isMEMpe = false, int numDPs = 1) : Module(Parent,name){
		if(peType=="GENERIC_8REGF"){
			createGenericPE(isMEMpe,numDPs,8);
		}
		else if(peType=="GENERIC_4REGF"){
			createGenericPE(isMEMpe,numDPs,4);
		}
		else if(peType == "HyCUBE_8REGF"){
			createHyCUBEPE_RegFile(isMEMpe,numDPs,8);
		}
		else if(peType == "HyCUBE_4REGF"){
			createHyCUBEPE_RegFile(isMEMpe,numDPs,4);
		}
		else if(peType == "HyCUBE_4REG"){
			createOriginalHyCUBEPE(isMEMpe,numDPs);
		}
		else if(peType == "N2N_8REGF"){
			createN2NPE(isMEMpe,numDPs,8);
		}
		else if(peType == "N2N_4REGF"){
			createN2NPE(isMEMpe,numDPs,4);
		}
		else if(peType == "STDNOC_8REGF"){
			createStdNoCPE_RegFile(isMEMpe,numDPs,8);
		}
		else if(peType == "STDNOC_4REGF"){
			createStdNoCPE_RegFile(isMEMpe,numDPs,4);
		}
		else if(peType == "STDNOC_4REG"){
			createStdNoCPE(isMEMpe,numDPs);
		}
		else{
			assert(false);
		}
		this->X = x;
		this->Y = y;
		this->T = t;
	}
	int X;
	int Y;
	int T;
	std::vector<RegFile*> allRegs;
	bool isMemPE=false;

	void createGenericPE(bool isMEMpe, int numberofDPs=1, int regs=8);
	void createStdNoCPE(bool isMEMpe, int numberofDPs=1);
	void createStdNoCPE_RegFile(bool isMEMpe, int numberofDPs=1, int regs=8, int nWRP=1, int nRDP=2);
	void createN2NPE(bool isMEMpe, int numberofDPs=1, int regs=8, int nWRP=1, int nRDP=2);
	void createHyCUBEPE_RegFile(bool isMEMpe, int numberofDPs=1, int regs=8, int nWRP=1, int nRDP=2);
	void createOriginalHyCUBEPE(bool isMEMpe, int numberofDPs=1);
	void getNonMEMIns(std::map<std::string,int>& supportedOPs);
	void getMEMIns(std::map<std::string,int>& supportedOPs);

private:
	bool alreadyInit=false;


};

} /* namespace CGRAXMLCompile */

#endif /* PE_H_ */

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
#include <unordered_set>

namespace CGRAXMLCompile
{

class PE : public Module
{
public:
	//	using Module::Module;
	PE(const Module *Parent,
	   std::string name,
	   int x, int y, int t,
	   std::string peType,
	   bool isMEMpe = false, int numDPs = 1) : Module(Parent, name, "PE", t)
	{
		if (peType == "GENERIC_8REGF")
		{
			createGenericPE(isMEMpe, numDPs, 8);
		}
		else if (peType == "GENERIC_4REGF")
		{
			createGenericPE(isMEMpe, numDPs, 4);
		}
		else if (peType == "HyCUBE_8REGF")
		{
			createHyCUBEPE_RegFile(isMEMpe, numDPs, 8);
		}
		else if (peType == "HyCUBE_4REGF")
		{
			createHyCUBEPE_RegFile(isMEMpe, numDPs, 4);
		}
		else if (peType == "HyCUBE_4REG")
		{
			createOriginalHyCUBEPE(isMEMpe, numDPs);
		}
		else if (peType == "N2N_8REGF")
		{
			createN2NPE(isMEMpe, numDPs, 8);
		}
		else if (peType == "N2N_4REGF")
		{
			createN2NPE(isMEMpe, numDPs, 4);
		}
		else if (peType == "STDNOC_8REGF")
		{
			createStdNoCPE_RegFile(isMEMpe, numDPs, 8);
		}
		else if (peType == "STDNOC_4REGF")
		{
			createStdNoCPE_RegFile(isMEMpe, numDPs, 4);
		}
		else if (peType == "STDNOC_4REG")
		{
			createStdNoCPE(isMEMpe, numDPs);
		}
		else if (peType == "STDNOC_4REGF_1P")
		{
			createStdNoCPE_RegFile(isMEMpe, numDPs, 4, 1, 1);
		}
		else if (peType == "MFU_HyCUBE_4REG")
		{
			createMultiFU_HyCUBEPE(isMEMpe, numDPs);
		}
		else if (peType == "MFU_HyCUBE_4REGF")
		{
			createMultiFU_HyCUBEPE_RegFile(isMEMpe, numDPs, 4, 4, 4);
		}
		else if (peType == "MFU_HyCUBE_4REGF2P")
		{
			createMultiFU_HyCUBEPE_RegFile(isMEMpe, numDPs, 4, 2, 2);
		}
		else if (peType == "MFU_STDNOC_4REG")
		{
			createMultiFU_StdNoCPE(isMEMpe, numDPs);
		}
		else if (peType == "MFU_STDNOC_4REGF")
		{
			createMultiFU_StdNoCPE_RegFile(isMEMpe, numDPs, 4, 4, 4);
		}
		else
		{
			assert(false);
		}

		assert(y != -1);
		assert(x != -1);
		this->X = x;
		this->Y = y;

		this->T = t;
	}

	//PE(const Module *Parent, std::string name, int t, int y, int x) : Module(Parent,name,"PE",t){
	PE(const Module *Parent, std::string name, int t, int x, int y) : Module(Parent,name,"PE",t){
			this->T = t;

		assert(y != -1);
		assert(x != -1);

		this->Y = y;
		this->X = x;
	};

	int X;
	int Y;
	int T;
	std::vector<RegFile *> allRegs;
	std::vector<FU *> allFUs;
	bool isMemPE = false;

	void createGenericPE(bool isMEMpe, int numberofDPs = 1, int regs = 8);
	void createStdNoCPE(bool isMEMpe, int numberofDPs = 1);
	void createStdNoCPE_noConflict(bool isMEMpe, int numberofDPs = 1);
	void createStdNoCPE_RegFile(bool isMEMpe, int numberofDPs = 1, int regs = 8, int nWRP = 2, int nRDP = 2);
	void createN2NPE(bool isMEMpe, int numberofDPs = 1, int regs = 8, int nWRP = 2, int nRDP = 2);
	void createHyCUBEPE_RegFile(bool isMEMpe, int numberofDPs = 1, int regs = 8, int nWRP = 2, int nRDP = 2);
	void createOriginalHyCUBEPE(bool isMEMpe, int numberofDPs = 1);

	void createMultiFU_HyCUBEPE_RegFile(bool isMEMpe,
										int numberofDPs, int regs, int nWRP, int nRDP);
	void createMultiFU_HyCUBEPE(bool isMEMpe, int numberofDPs);
	void createMultiFU_StdNoCPE_RegFile(bool isMEMpe,
										int numberofDPs, int regs, int nWRP, int nRDP);
	void createMultiFU_StdNoCPE(bool isMEMpe, int numberofDPs);

	void getNonMEMIns(std::map<std::string, int> &supportedOPs);
	void getMEMIns(std::map<std::string, int> &supportedOPs);

	void getLogicalIns(std::map<std::string, int> &supportedOPs);
	void getArithmeticIns(std::map<std::string, int> &supportedOPs);
	void getMemOnlyIns(std::map<std::string, int> &supportedOPs);
	void getOutMemOnlyIns(std::map<std::string, int> &supportedOPs);
	

	void insertRegConPort(std::pair<Port *, Port *> portPair) { allRegConPorts.insert(portPair); }
	std::pair<Port *, Port *> getRegConPort(std::string pName);
	std::set<std::pair<Port *, Port *>> getRegConPorts() { return allRegConPorts; }

	// by Yujie
	int getPosition_X(){ return X;}
	int getPosition_Y(){ return Y;}

private:
	bool alreadyInit = false;
	std::set<std::pair<Port *, Port *>> allRegConPorts;


};

} /* namespace CGRAXMLCompile */

#endif /* PE_H_ */

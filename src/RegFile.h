/*
 * RegFile.h
 *
 *  Created on: 26 Feb 2018
 *      Author: manupa
 */

#ifndef REGFILE_H_
#define REGFILE_H_

#include "Module.h"

namespace CGRAXMLCompile
{

class RegFile : public Module
{
public:
	RegFile(const Module *Parent, std::string name, int nWRPs, int nRegs, int nRDPs) : Module(Parent, name)
	{
		createRegFile(nWRPs, nRegs, nRDPs);

		this->nWRPs = nWRPs;
		this->nRegs = nRegs;
		this->nRDPs = nRDPs;
	}
	int get_nWRPs() { return nWRPs; }
	int get_nRDPs() { return nRDPs; }
	int get_nRegs() { return nRegs; }

private:
	void createRegFile(int nWRPs, int nRegs, int nRDPs);
	int nWRPs;
	int nRegs;
	int nRDPs;
};

} /* namespace CGRAXMLCompile */

#endif /* REGFILE_H_ */

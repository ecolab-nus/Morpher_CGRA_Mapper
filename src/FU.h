/*
 * FU.h
 *
 *  Created on: 26 Feb 2018
 *      Author: manupa
 */

#ifndef FU_H_
#define FU_H_

#include "Module.h"
#include <set>

namespace CGRAXMLCompile
{

class FU : public Module
{
public:
	//	using Module::Module;
	FU(const Module *Parent, std::string name, int numberDPs, std::map<std::string, int> supportedOPs, int t) : Module(Parent, name, t)
	{
		this->numberDPs = numberDPs;
		createFU(numberDPs);
		this->supportedOPs = supportedOPs;
	}

	FU(const Module *Parent, std::string name, int t) : Module(Parent,name,t){}

	void createFU(int numberDPs);
	int getNumberDPs() { return numberDPs; }

	void createFUInputRegConnections();
	void createFUInputRegCreate();

	std::map<std::string, int> supportedOPs;
	std::string currOP = "NOP";

	bool isMEMFU();

private:
	int numberDPs;
};

} /* namespace CGRAXMLCompile */

#endif /* FU_H_ */

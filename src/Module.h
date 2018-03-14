/*
 * Module.h
 *
 *  Created on: 26 Feb 2018
 *      Author: manupa
 */

#ifndef MODULE_H_
#define MODULE_H_

#include <string>
#include <vector>
#include <map>

#include "Port.h"


namespace CGRAXMLCompile {

class CGRA;
class PE;
class FU;

class Module {
public:
	Module(const Module* Parent, std::string name);
	virtual ~Module();
	std::vector<Port> inputPorts;
	std::vector<Port> outputPorts;
	std::vector<Port> internalPorts;
	std::vector<Module*> subModules;
	std::map<Port*,std::vector<Port*> > connections;

	Port* getInPort(std::string Pname);
	Port* getOutPort(std::string Pname);
	Port* getInternalPort(std::string Pname);
	Module* getSubMod(std::string Mname);

	std::string getName(){return name;}
	Module* getParent(){return (Module*)Parent;}

	CGRA* getCGRA();
	PE* getPE();
	FU* getFU();


	std::map<Port*,std::vector<Port*>> conflictPorts;

private:
	const Module* Parent;
	std::string name;
};

} /* namespace CGRAXMLCompile */

#endif /* MODULE_H_ */

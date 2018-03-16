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
class HeuristicMapper;

class Module {
public:
	Module(const Module* Parent, std::string name);
	virtual ~Module();
	std::vector<Port> inputPorts;
	std::vector<Port> outputPorts;
	std::vector<Port> internalPorts;
	std::vector<Module*> subModules;


	Port* getInPort(std::string Pname);
	Port* getOutPort(std::string Pname);
	Port* getInternalPort(std::string Pname);
	Module* getSubMod(std::string Mname);

	std::string getName(){return name;}
	Module* getParent(){return (Module*)Parent;}

	CGRA* getCGRA();
	PE* getPE();
	FU* getFU();

	std::vector<Port*> getNextPorts(Port* currPort, HeuristicMapper* hm);
	std::vector<Port*> getFromPorts(Port* currPort, HeuristicMapper* hm);

	std::vector<Port*> getConflictPorts(Port* currPort);
	void insertConnection(Port* src, Port* dest);

protected:
	std::map<Port*,std::vector<Port*>> conflictPorts;

private:
	std::map<Port*,std::vector<Port*>> connectedTo;
	std::map<Port*,std::vector<Port*>> connectedFrom;
	const Module* Parent;
	std::string name;
};

} /* namespace CGRAXMLCompile */

#endif /* MODULE_H_ */

/*
 * Module.cpp
 *
 *  Created on: 26 Feb 2018
 *      Author: manupa
 */

#include <morpher/arch/Module.h>
#include <morpher/arch/CGRA.h>
#include <morpher/arch/FU.h>
#include <morpher/mapper/PathFinderMapper.h>
#include <assert.h>

namespace CGRAXMLCompile
{

Module::Module(const Module *Parent, std::string name, string type, int t)
{
	// TODO Auto-generated constructor stub
	this->Parent = Parent;
	this->name = name;
	this->t = t;
	this->type = type;
}

Module::~Module()
{
	for (Port *p : inputPorts)
	{
		delete p;
	}

	for (Port *p : outputPorts)
	{
		delete p;
	}

	for (Port *p : internalPorts)
	{
		delete p;
	}

	for (std::pair<Port *, Port *> portPair : regPorts)
	{
		delete portPair.first;
		delete portPair.second;
	}

	for (Module *M : subModules)
	{
		delete M;
	}
}

FU *Module::getFU()
{
	Module *mod = this->getParent();
	while (mod)
	{
		if (FU *fu = dynamic_cast<FU *>(mod))
		{
			return fu;
		}
		mod = mod->getParent();
	}
	return NULL;
}

std::vector<LatPort> Module::getNextPorts(LatPort currPort, HeuristicMapper *hm)
{
	std::vector<LatPort> nextPorts;

	//	if(currPort->getType() == REG) std::cout << "DDDDDDDDDEBUGG : REG PORTS : ";
	Port *currP = currPort.second;
	PE *currPE = currP->getMod()->getPE();
	int lat = currPort.first;

	for (Port *p : connectedTo[currP])
	{
		bool conflicted = false;


		//if this port belongs to a DP and has been mapped, then we should not use it.
		if(p->mapped_nodes.size() > 0 && p->getFullName().find("DP") != std::string::npos){
			// std::cout<<"-----------port name:"<<p->getFullName()<<", num:"<< p->mapped_nodes.size()<<", skip...\n";
			continue;
		}


		// if the T_ROUTE has been used, then we should not use DP0.I1 as next ports.
		bool cause_confict = false;
		if (currP->getFullName().find("DP0.T") != std::string::npos && 
				p->getFullName().find("DP0.I") != std::string::npos ){
			if (DataPath *dp = dynamic_cast<DataPath *>(p->getMod())){
				auto fu = dp->getFU();
				for(auto output_port: fu->outputPorts){
					if(output_port->getName().find("T_ROUTE") != std::string::npos && output_port->mapped_nodes.size()>0){
						cause_confict = true;
						break;
					}
				}
			}else{
				assert(false);
			}
		}

		// if there is an operation mapped, we disable routing
		if(p->getName().find("T_ROUTE") != std::string::npos ){
			auto dp_ptr = p->getMod()->getSubMod("DP0");
			assert(dp_ptr != NULL);
			if (DataPath *dp = dynamic_cast<DataPath *>(dp_ptr)){
				if(dp->getMappedNode()!= NULL){
					cause_confict = true;
				}
			}
		}

		if(cause_confict){
			continue;
		}

		if (PathFinderMapper *pfm = dynamic_cast<PathFinderMapper *>(hm))
		{

			//			if(currPort->getType() == REG){
			//				std::cout << p->getFullName() << ",";
			//			}
		}
		else
		{
			for (Port *conflict_port : getConflictPorts(p))
			{
				if (conflict_port->getNode() != NULL)
				{
					conflicted = true;
					break;
				}
			}
		}

		PE *nextPE = p->getMod()->getPE();
		int tdiff = nextPE->T - currPE->T;
		if (tdiff < 0)
		{
			tdiff += nextPE->getCGRA()->get_t_max();
			assert(tdiff > 0);
		}

		assert(tdiff <= 1);

		if (nextPE->getCGRA()->get_t_max() == 1)
		{
			//special case for ii = 1
			if (regCons[std::make_pair(currP, p)])
			{
				// std::cout << "tdiff = " << tdiff << "\n";
				// assert(false);
				if (tdiff == 0)
				{
					tdiff = 1;
					// std::cout << "tdiff with src=" << currP->getFullName() << ",dest=" << p->getFullName() << "\n";
				}
			}
		}

		if (!conflicted)
		{
			if (regCons[std::make_pair(currP, p)])
			{
				// assert(false);
				assert(tdiff == 1);
				assert(currPort.first < lat + tdiff);
			}

			nextPorts.push_back(std::make_pair(lat + tdiff, p));
		}
	}

	//	if(currPort->getType() == REG) std::cout << "\n";

	if (currP->getType() == OUT)
	{
		if (getParent())
		{
			for (Port *p : currP->getMod()->getParent()->connectedTo[currP])
			{
				//					std::cout << currPort->getMod()->getParent()->getName() << "***************\n";
				//				nextPorts.push_back(p);

				bool conflicted = false;
				if (PathFinderMapper *pfm = dynamic_cast<PathFinderMapper *>(hm))
				{
				}
				else
				{
					for (Port *conflict_port : getParent()->getConflictPorts(currP))
					{
						if (conflict_port->getNode() != NULL)
						{
							conflicted = true;
							break;
						}
					}
				}

				PE *nextPE = p->getMod()->getPE();
				int tdiff = nextPE->T - currPE->T;
				if (tdiff < 0)
				{
					tdiff += nextPE->getCGRA()->get_t_max();
					assert(tdiff > 0);
				}

				if (nextPE->getCGRA()->get_t_max() == 1)
				{
					//special case for ii = 1
					if (regCons[std::make_pair(currP, p)])
					{
						// std::cout << "tdiff = " << tdiff << "\n";
						// assert(false);
						if (tdiff == 0)
							tdiff = 1;
					}
				}

				if (!conflicted)
				{
					if (regCons[std::make_pair(currP, p)])
					{
						// assert(false);
						assert(tdiff == 1);
					}
					nextPorts.push_back(std::make_pair(lat + tdiff, p));
				}
			}
		}
	}
	return nextPorts;
}

std::vector<Port *> Module::getNextPorts(Port *currPort)
{
	std::vector<Port *> nextPorts;

	for (Port *p : connectedTo[currPort])
	{
		nextPorts.push_back(p);
	}

	if (currPort->getType() == OUT)
	{
		if (getParent())
		{
			for (Port *p : currPort->getMod()->getParent()->connectedTo[currPort])
			{
				//					std::cout << currPort->getMod()->getParent()->getName() << "***************\n";
				//				nextPorts.push_back(p);

				nextPorts.push_back(p);
			}
		}
	}
	return nextPorts;
}

std::vector<Port *> Module::getFromPorts(Port *currPort)
{
	std::vector<Port *> fromPorts;

	for (Port *p : connectedFrom[currPort])
	{
		fromPorts.push_back(p);
	}

	if (currPort->getType() == IN)
	{
		if (getParent())
		{
			for (Port *p : currPort->getMod()->getParent()->connectedFrom[currPort])
			{
				//					std::cout << currPort->getMod()->getParent()->getName() << "***************\n";
				//				nextPorts.push_back(p);

				fromPorts.push_back(p);
			}
		}
	}
	return fromPorts;
}

std::set<Port *> Module::getConflictPorts(Port *currPort)
{
	assert(this->getCGRA());
	std::set<Port *> vec = this->getCGRA()->getConflictPorts(currPort);
	return vec;
}

std::vector<Port *> Module::getFromPorts(Port *currPort, HeuristicMapper *hm)
{
	std::vector<Port *> fromPorts;

	for (Port *p : connectedFrom[currPort])
	{
		bool conflicted = false;
		for (Port *conflict_port : getConflictPorts(p))
		{
			if (conflict_port->getNode() != NULL)
			{
				conflicted = true;
				break;
			}
		}
		if (!conflicted)
		{
			fromPorts.push_back(p);
		}
	}

	if (currPort->getType() == IN)
	{
		if (getParent())
		{
			for (Port *p : currPort->getMod()->getParent()->connectedFrom[currPort])
			{
				//					std::cout << currPort->getMod()->getParent()->getName() << "***************\n";
				//				nextPorts.push_back(p);

				bool conflicted = false;
				for (Port *conflict_port : getParent()->getConflictPorts(currPort))
				{
					if (conflict_port->getNode() != NULL)
					{
						conflicted = true;
						break;
					}
				}
				if (!conflicted)
				{
					fromPorts.push_back(p);
				}
			}
		}
	}

	return fromPorts;
}

bool Module::isConflictPortsEmpty(Port *p)
{
	return getCGRA()->isConflictPortsEmpty(p);
}

std::string Module::getFullName()
{
	Module *mod = this;
	std::stack<std::string> fullNameSt;

	while (mod)
	{
		fullNameSt.push(mod->getName());
		mod = mod->getParent();
	}

	std::string fullName;
	while (!fullNameSt.empty())
	{
		fullName = fullName + fullNameSt.top() + ".";
		fullNameSt.pop();
	}
	fullName = fullName + name;
	return fullName;
}

std::pair<Port *, Port *> Module::getRegPort(std::string Pname)
{

	std::string ri = Pname + "_RI";

	for (std::pair<Port *, Port *> portpair : regPorts)
	{
		if (portpair.first->getName() == ri)
		{
			return portpair;
		}

		if (portpair.first->getName() == Pname)
		{
			return portpair;
		}

		if (portpair.second->getName() == Pname)
		{
			return portpair;
		}
	}
	assert(false);
}

Port* Module::getSingleRegPort2(std::string Pname)
{

	std::string ri = Pname + "_RI";

	for (std::pair<Port *, Port *> portpair : regPorts)
	{
		if (portpair.first->getName() == ri)
		{
			return portpair.first;
		}

		if (portpair.first->getName() == Pname)
		{
			return portpair.first;
		}

		if (portpair.second->getName() == Pname)
		{
			return portpair.second;
		}
	}
	assert(false);
}

Port* Module::getSingleRegPort(std::string Pname)
{

//	std::string ri = Pname + "_RO";

	for (std::pair<Port *, Port *> portpair : regPorts)
	{

		if (portpair.first->getName() == Pname)
		{
			return portpair.first;
		}

		if (portpair.second->getName() == Pname)
		{
			return portpair.second;
		}
	}
//	assert(false);
}

void Module::insertRegPort(std::string pName)
{
	// cout << "INSERT REGPORT = " << pName << ",to module = " << this->getFullName() << ",parent = " << getParent()->getName() << "\n";
	Port *ri = new Port(pName + "_RI", REGI, this);
	Port *ro = new Port(pName + "_RO", REGO, this);
	insertConnection(ri, ro);
	regPorts.push_back(std::make_pair(ri, ro));
	PE *pe = getPE();
	pe->insertRegConPort(std::make_pair(ri, ro));
}

void Module::insertConnection(Port *src, Port *dest)
{

	PE *src_pe = src->getMod()->getPE();
	PE *dest_pe = dest->getMod()->getPE();

	// cout << "srcp = " << src->getFullName() << ",destp = " << dest->getFullName() << "\n";

	//	if(src_pe && dest_pe){
	if (src_pe && dest_pe && src_pe->T > dest_pe->T && dest_pe->T != 0)
	{
		std::cout << "ILLEGAL CONNECTION!\n";
		std::cout << src_pe->getFullName() << "\n";
		std::cout << dest_pe->getFullName() << "\n";
		cout << "src_pe->T = " << src_pe->T << ",dest_pe->T = " << dest_pe->T << "\n";
		assert(src_pe->T <= dest_pe->T || dest_pe->T == 0);
	}
	
	//	}

	connectedTo[src].push_back(dest);
	connectedFrom[dest].push_back(src);
}

void Module::insertConnection(std::pair<Port *, Port *> regSrc, Port *dest)
{

	PE *src_pe = regSrc.first->getMod()->getPE();
	PE *dest_pe = dest->getMod()->getPE();

	//	if(src_pe && dest_pe){
	if (src_pe->T > dest_pe->T && dest_pe->T != 0)
	{
		std::cout << "ILLEGAL CONNECTION!\n";
		std::cout << src_pe->getFullName() << "\n";
		std::cout << dest_pe->getFullName() << "\n";
	}
	assert(src_pe->T <= dest_pe->T || dest_pe->T == 0);
	//	}

	connectedTo[regSrc.first].push_back(dest);
	connectedFrom[dest].push_back(regSrc.first);
}

void Module::insertConnection(Port *src, std::pair<Port *, Port *> regDest)
{

	PE *src_pe = src->getMod()->getPE();
	PE *dest_pe = regDest.first->getMod()->getPE();

	//	if(src_pe && dest_pe){
	if (src_pe->T > dest_pe->T && dest_pe->T != 0)
	{
		std::cout << "ILLEGAL CONNECTION!\n";
		std::cout << src_pe->getFullName() << "\n";
		std::cout << dest_pe->getFullName() << "\n";
	}
	assert(src_pe->T <= dest_pe->T || dest_pe->T == 0);
	//	}

	connectedTo[src].push_back(regDest.second);
	connectedFrom[regDest.second].push_back(src);
}

} /* namespace CGRAXMLCompile */

CGRAXMLCompile::Port *CGRAXMLCompile::Module::getInPort(std::string Pname)
{
	for (Port *p : inputPorts)
	{
		if (p->getName().compare(Pname) == 0)
		{
			return p;
		}
	}
	assert(false);
}

CGRAXMLCompile::Port *CGRAXMLCompile::Module::getOutPort(std::string Pname)
{
	for (Port *p : outputPorts)
	{
		if (p->getName().compare(Pname) == 0)
		{
			return p;
		}
	}
	assert(false);
}

CGRAXMLCompile::Port *CGRAXMLCompile::Module::getInternalPort(std::string Pname)
{
	for (Port *p : internalPorts)
	{
		if (p->getName().compare(Pname) == 0)
		{
			return p;
		}
	}
	assert(false);
}

CGRAXMLCompile::Port *CGRAXMLCompile::Module::getSocketPort(std::string Pname)
{
	for (Port *p : socketPorts)
	{
		if (p->getName().compare(Pname) == 0)
		{
			return p;
		}
	}
	assert(false);
}

CGRAXMLCompile::Module *CGRAXMLCompile::Module::getSubMod(std::string Mname)
{
	for (Module *m : subModules)
	{
		if (m->getName().compare(Mname) == 0)
		{
			return m;
		}
	}
	return NULL;
}

CGRAXMLCompile::CGRA *CGRAXMLCompile::Module::getCGRA()
{
	Module *mod = this;
	while (true)
	{
		if (CGRA *cgra = dynamic_cast<CGRA *>(mod))
		{
			return cgra;
		}
		mod = mod->getParent();
		if (!mod)
		{
			assert(false);
		}
	}
}

CGRAXMLCompile::PE *CGRAXMLCompile::Module::getPE()
{

	Module *mod = this;
	while (mod)
	{
		if (PE *pe = dynamic_cast<PE *>(mod))
		{
			return pe;
		}
		mod = mod->getParent();
	}
	return NULL;
}

void CGRAXMLCompile::Module::UpdateMappedConnectionsJSON(json &output_json)
{
	Module *mod = this;
	CGRA *cgra = getCGRA();
	do
	{
		int t = mod->get_t();
		for (auto it = mod->connectedTo.begin(); it != mod->connectedTo.end(); it++)
		{
			Port *src_port = it->first;
			Module *src_module = src_port->getMod();
			string src_port_name;
			if (src_module == mod)
			{
				src_port_name = "THIS." + src_port->getName();
			}
			else
			{
				src_port_name = src_module->getName() + "." + src_port->getName();
			}

			//if source port is not mapped no need to explore
			if (src_port->getNode() == NULL)
			{
				// cout << "src_port = " << src_port->getFullName() << "is not used!\n";
				continue;
			}

			for (Port *dest_port : it->second)
			{
				Module *dest_module = dest_port->getMod();
				string dest_port_name;
				if (dest_module == mod)
				{
					dest_port_name = "THIS." + dest_port->getName();
				}
				else
				{
					dest_port_name = dest_module->getName() + "." + dest_port->getName();
				}

				//the dest port is not in use
				if (dest_port->getNode() == NULL)
					continue;

				int LatencyDiff = dest_port->getLat() - src_port->getLat();
				bool isTemproalRegConnection = src_port->getType() == REGO && dest_port->getType() == REGI;
				bool isBothRegPorts = (src_port->getType() == REGO && dest_port->getType() == REGI) || (src_port->getType() == REGI && dest_port->getType() == REGO);
				bool hasSameNode = dest_port->getNode() == src_port->getNode();
				bool valid_connection = false;

				// if (isTemproalRegConnection)
				// {
				// 	if (LatencyDiff == 1 && hasSameNode)
				// 	{
				// 		output_json["CONNECTIONS"][to_string(t)][src_port_name] = dest_port_name;
				// 		output_json["MAPPED_NODES"][to_string(t)][src_port_name] = src_port->getNode()->idx;
				// 		output_json["MAPPED_NODES"][to_string(t)][dest_port_name] = dest_port->getNode()->idx;
				// 		valid_connection = true;
				// 	}
				// }
				// else
				// {
				// 	if (LatencyDiff == 0 && hasSameNode)
				// 	{
				// 		output_json["CONNECTIONS"][to_string(t)][src_port_name] = dest_port_name;
				// 		output_json["MAPPED_NODES"][to_string(t)][src_port_name] = src_port->getNode()->idx;
				// 		output_json["MAPPED_NODES"][to_string(t)][dest_port_name] = dest_port->getNode()->idx;
				// 		valid_connection = true;
				// 	}
				// }

				if (!isBothRegPorts)
				{
					if (LatencyDiff == 0 && hasSameNode)
					{
						output_json["CONNECTIONS"][to_string(t)][src_port_name].push_back(dest_port_name);
						// by Yujie
						std::cout<<src_port->getName()<<"\t"<<dest_port->getName()<<std::endl;
						output_json["MAPPED_NODES"][to_string(t)][src_port_name] = src_port->getNode()->idx;
						output_json["MAPPED_NODES"][to_string(t)][dest_port_name] = dest_port->getNode()->idx;
						valid_connection = true;
					}
				}

				// if (src_port->getName().find("WP") != string::npos)
				// {
				// 	if (!valid_connection)
				// 	{
				// 		cout << "src_port=" << src_port->getFullName() << "\n";
				// 		cout << "dest_port=" << dest_port->getFullName() << "\n";
				// 		cout << "src_node = " << src_port->getNode()->idx << "\n";
				// 		cout << "dest_node = " << dest_port->getNode()->idx << "\n";
				// 	}
				// 	// assert(false);
				// 	assert(valid_connection);
				// }
			}
		}
		mod = mod->getNextTimeIns();
	} while (mod != this);

	//If this is a funcional unit explore children for datapath submodule
	if (FU *this_fu = dynamic_cast<FU *>(this))
	{
		for (Module *sub_module : this_fu->subModules)
		{
			if (DataPath *dp = dynamic_cast<DataPath *>(sub_module))
			{
				Module *mod = dp;
				do
				{
					int t = mod->get_t();
					DataPath *mod_dp = static_cast<DataPath *>(mod);
					if (mod_dp->getMappedNode())
					{
						output_json["OPS"][to_string(t)] = mod_dp->getMappedNode()->op;
						output_json["OP_INFO"][to_string(t)]["id"] = mod_dp->getMappedNode()->idx;
						output_json["OP_INFO"][to_string(t)]["NPB"] = mod_dp->getMappedNode()->npb;

						if(mod_dp->getMappedNode()->hasConst){
							//if there are constants they are always fed to I2 port.
							output_json["CONNECTIONS"][to_string(t)]["CONST." + to_string(mod_dp->getMappedNode()->constant)] = mod_dp->getName() + ".I2";
						}

					}
					mod = mod->getNextTimeIns();
				} while (mod != dp);

				//Assumption :: only 1 datapath module is found inside FU
				break;
			}
		}
	}
}

CGRAXMLCompile::Port* CGRAXMLCompile::Module::getJSONPort(string pname, bool isSrc){
	Port* src_p;
	string src = pname;

	string src_mod_str = src.substr(0, src.find("."));
	string src_port_str = src.erase(0, src.find(".") + 1);

	Module *src_mod;
	if (src_mod_str == "THIS")
	{
		src_mod = this;
	}
	else
	{
		src_mod = this->Name2SubMod[src_mod_str];
	}
	assert(src_mod);


	if (!src_mod->Name2RegPort.empty() && src_mod->Name2RegPort.find(src_port_str) != src_mod->Name2RegPort.end())
	{
		if(isSrc){
			//in a regport first is used for outgoing connections
			src_p = src_mod->Name2RegPort[src_port_str].first;
		}
		else{
			//in a regport second is used for incoming connections
			src_p = src_mod->Name2RegPort[src_port_str].second;	
		}
	}
	else
	{
		src_p = src_mod->Name2Port[src_port_str];
	}


	// cout << "src_mod = " << src_mod->getName() << "\n";
	// cout << "src_p_name = " << src_port_str << "\n";
	assert(src_p);
	return src_p;
}

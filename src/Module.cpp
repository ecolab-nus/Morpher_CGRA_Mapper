/*
 * Module.cpp
 *
 *  Created on: 26 Feb 2018
 *      Author: manupa
 */

#include "Module.h"
#include "CGRA.h"
#include "FU.h"
#include "PathFinderMapper.h"
#include <assert.h>

namespace CGRAXMLCompile
{

Module::Module(const Module *Parent, std::string name)
{
	// TODO Auto-generated constructor stub
	this->Parent = Parent;
	this->name = name;
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

void Module::insertRegPort(std::string pName)
{
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

	//	if(src_pe && dest_pe){
	if (src_pe->T > dest_pe->T && dest_pe->T != 0)
	{
		std::cout << "ILLEGAL CONNECTION!\n";
		std::cout << src_pe->getFullName() << "\n";
		std::cout << dest_pe->getFullName() << "\n";
	}
	assert(src_pe->T <= dest_pe->T || dest_pe->T == 0);
	//	}

	connectedTo[src].push_back(dest);
	connectedFrom[dest].push_back(src);
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

CGRAXMLCompile::Module *CGRAXMLCompile::Module::getSubMod(std::string Mname)
{
	for (Module *m : subModules)
	{
		if (m->getName().compare(Mname) == 0)
		{
			return m;
		}
	}
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

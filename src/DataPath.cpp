/*
 * DataPath.cpp
 *
 *  Created on: 26 Feb 2018
 *      Author: manupa
 */

#include "DataPath.h"
#include "FU.h"
#include "Port.h"
#include "CGRA.h"
#include "DFG.h"
#include <assert.h>
#include <iostream>

namespace CGRAXMLCompile
{

} /* namespace CGRAXMLCompile */

CGRAXMLCompile::FU *CGRAXMLCompile::DataPath::getFU()
{
	Module *mod = this->getParent();
	FU *parentFU = static_cast<FU *>(mod);
	return parentFU;
}

CGRAXMLCompile::PE *CGRAXMLCompile::DataPath::getPE()
{
	Module *mod = this->getParent();
	while (true)
	{
		if (PE *pe = dynamic_cast<PE *>(mod))
		{
			return pe;
		}
		mod = mod->getParent();
		if (!mod)
		{
			assert(false);
		}
	}
}

CGRAXMLCompile::CGRA *CGRAXMLCompile::DataPath::getCGRA()
{
	Module *mod = this->getParent();
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

CGRAXMLCompile::Port *CGRAXMLCompile::DataPath::getOutputPort(int latency)
{
	PE *currPE = getPE();
	FU *currFU = getFU();
	CGRA *currCGRA = getCGRA();

	int nextPE_t = (currPE->T + latency) % currCGRA->get_t_max();

	//	std::cout << "nextPE_t = " << nextPE_t << "\n";

	// PE *outputPE = currCGRA->PEArr[nextPE_t][currPE->Y][currPE->X];
	PE *outputPE = currCGRA->getLatencyPE(currPE,latency);
	FU *outputFU = static_cast<FU *>(outputPE->getSubMod(currFU->getName()));
	DataPath *outputDP = static_cast<DataPath *>(outputFU->getSubMod(this->getName()));

	//	std::cout << "getOutputPort::outputDP=" << outputPE->getName();
	//	std::cout << "." << outputFU->getName();
	//	std::cout << "." << outputDP->getName() << "\n";

	Port *outputPort = outputDP->getOutPort("T");
	return outputPort;
}

void CGRAXMLCompile::DataPath::assignNode(DFGNode *node, int lat, DFG *dfg)
{

	this->mappedNode = node;
	this->latency = lat;

	FU *fu = getFU();
	PE *pe = getPE();
	CGRA *cgra = getCGRA();

	assert(pe->T == lat % cgra->get_t_max());

	if (fu->currOP.compare("NOP") == 0)
	{
		fu->currOP = node->op;
	}
	else
	{
		// std::cout << "new_op = " << node->op << ",old_op = " << fu->currOP << "\n";
		assert(fu->currOP.compare(node->op) == 0);
	}

	int oplatency = fu->supportedOPs[node->op];
	int next_t = (pe->T + oplatency) % cgra->get_t_max();
	// std::cout << "assigning node=" << node->idx << ",to=" << pe->getName() << ",starting t=" << next_t << "\n";

	// PE *nextPE = cgra->PEArr[next_t][pe->Y][pe->X];
	PE* nextPE = cgra->getLatencyPE(pe,oplatency);
	// cout << "op_lat = " << oplatency << "," << "nextPE = " << nextPE->getName() << "\n";

	FU *nextFU = static_cast<FU *>(nextPE->getSubMod(fu->getName()));
	DataPath *nextDP = static_cast<DataPath *>(nextFU->getSubMod(this->getName()));

	this->outputDP = nextDP;
	nextDP->getOutPort("T")->setNode(node, latency + oplatency, node->idx);
	node->routingPorts.push_back(std::make_pair(nextDP->getOutPort("T"), node->idx));

	if (fu->supportedOPs.find("LOAD") != fu->supportedOPs.end())
	{
		cgra->freeMemNodes--;
		cgra->freeMemNodeSet.erase(this);
	}

	if (node->isMemOp())
	{
		dfg->unmappedMemOps--;
		dfg->unmappedMemOpSet.erase(node);
	}
}

void CGRAXMLCompile::DataPath::clear()
{

	this->outputDP->getOutPort("T")->clear();
	mappedNode = NULL;
	outputDP = NULL;
	latency = -1;

	FU *fu = getFU();

	// if(detailed) std::cout << "PE=" << fu->getPE()->getName() << "is cleared!\n";

	bool restDPsNOP = true;
	for (Module *mod : fu->subModules)
	{
		if (DataPath *dp = dynamic_cast<DataPath *>(mod))
		{
			if (dp == this)
				continue;
			if (dp->mappedNode)
			{
				if (dp->mappedNode->op.compare("NOP") != 0)
				{
					restDPsNOP = false;
				}
			}
		}
	}

	if (restDPsNOP)
	{
		fu->currOP = "NOP";
	}
}

CGRAXMLCompile::Port *CGRAXMLCompile::DataPath::getPotOutputPort(DFGNode *node)
{

	FU *fu = getFU();
	PE *pe = getPE();
	CGRA *cgra = getCGRA();

	assert(fu->supportedOPs.find(node->op) != fu->supportedOPs.end());
	int latency = fu->supportedOPs[node->op];
	return getOutputPort(latency);
}

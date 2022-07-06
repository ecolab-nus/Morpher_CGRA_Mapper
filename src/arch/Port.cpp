/*
 * Port.cpp
 *
 *  Created on: 26 Feb 2018
 *      Author: manupa
 */

#include <morpher/arch/Port.h>
#include <morpher/arch/PE.h>
#include <morpher/arch/Module.h>
#include <morpher/arch/CGRA.h>
#include <morpher/mapper/PathFinderMapper.h>
#include <morpher/arch/CGRA.h>
#include <stack>
#include <iostream>

namespace CGRAXMLCompile
{

Port::Port(std::string name, PortType pType, Module *mod)
{
	// TODO Auto-generated constructor stub
	this->name = name;
	this->pType = pType;
	this->mod = mod;
	this->pe = mod->getPE();
}

void Port::increaseUse(HeuristicMapper *hm)
{
	assert(mapped_nodes.size() > 0);
	DFGNode * last_mapped_node = std::get<0>(*(this->mapped_nodes.rbegin()));
	if (PathFinderMapper *pfm = dynamic_cast<PathFinderMapper *>(hm))
	{
		number_signals = 1;

		for (DFGNode *node : (*pfm->getcongestedPortsPtr())[this])
		{
			if (last_mapped_node == node)
				continue;
			if (pfm->dfg->isMutexNodes(last_mapped_node, node, this))
				continue;
			number_signals++;
			//			break;
		}

		for (DFGNode *node : (*pfm->getconflictedPortsPtr())[this])
		{
			if (last_mapped_node == node)
				continue;
			if (pfm->dfg->isMutexNodes(last_mapped_node, node, this))
				continue;
			number_signals++;
			break;
		}

		int timeStep = this->getMod()->getPE()->T;
		int alreadyConflicts = pfm->getTimeStepConflicts(timeStep);
		pfm->updateConflictedTimeSteps(timeStep, number_signals - 1);
		number_signals = number_signals - alreadyConflicts;
	}
	else
	{
		number_signals++;
	}

}

void Port::decreaseUse(DFGNode *extnode, HeuristicMapper *hm)
{
	if (number_signals > 0)
	{
		if (PathFinderMapper *pfm = dynamic_cast<PathFinderMapper *>(hm))
		{
			for (DFGNode *con_node : (*pfm->getcongestedPortsPtr())[this])
			{
				if (extnode == con_node)
					continue;
				if (pfm->dfg->isMutexNodes(extnode, con_node, this))
					continue;
				number_signals--;
				break;
			}
		}
		else
		{
			number_signals--;
		}
	}
}

void Port::setNode(DFGNode *node, int latency,  int dest ,  HeuristicMapper *hm)
{
	// std::cout<<this->getFullName()<<"\n";

	// std::cout<<" before put \n";
	// for(auto it =  mapped_nodes.begin(); it != mapped_nodes.end(); it++){
		
	// 	std::cout<<std::get<0>(*it)->idx<<","<<std::get<1>(*it)<<","<<std::get<2>(*it)<<" \t";
	// }
	// std::cout<<"\n";
	
	if(std::find(mapped_nodes.begin(), mapped_nodes.end(), std::make_tuple(node, dest, latency)) == mapped_nodes.end()){
		this->mapped_nodes.push_back(std::make_tuple(node, dest, latency));
		// node_value_dests.emplace(node, std::set<int>());
	}
	// std::cout<<"\n after put \n";
	// for(auto it =  mapped_nodes.begin(); it != mapped_nodes.end(); it++){
	// 	std::cout<<std::get<0>(*it)->idx<<","<<std::get<1>(*it)<<","<<std::get<2>(*it)<<" \t";

	// }
	// std::cout<<"\n";

	bool find_node = false;
	for(auto it =  mapped_nodes.begin(); it != mapped_nodes.end(); it++){
		if(std::get<0>(*it)->idx == node->idx){
			find_node = true;
		}
	}
	assert(find_node);
	setLat(latency);

	PE *pe = getMod()->getPE();
	CGRA *cgra = getMod()->getCGRA();
	assert(pe->T == latency % cgra->get_t_max());

	if (hm == NULL)
		return;

	if (PathFinderMapper *pfm = dynamic_cast<PathFinderMapper *>(hm))
	{
		//		for(Port* p : getMod()->getConflictPorts(this)){
		//			(*pfm->getcongestedPortsPtr())[p].insert(node);
		//		}
	}
}

void Port::increaseConflictedUse(DFGNode *node, HeuristicMapper *hm)
{
	increaseUse();

	if (!getMod()->isConflictPortsEmpty(this))
	{
		for (Port *p : getMod()->getConflictPorts(this))
		{
			assert(p != NULL);

			if (PathFinderMapper *pfm = dynamic_cast<PathFinderMapper *>(hm))
			{
				(*pfm->getconflictedPortsPtr())[p].insert(node);
			}
			p->increaseUse();
		}
	}

	//	if(this->getType()==OUT){
	//		for(Port* p : getMod()->getParent()->getConflictPorts(this)){
	//			p->increaseUse();
	//		}
	//	}
}

} /* namespace CGRAXMLCompile */

CGRAXMLCompile::PE *CGRAXMLCompile::Port::findParentPE()
{

	Module *m = mod;

	while (m)
	{
		if (PE *ret = dynamic_cast<PE *>(m))
		{
			return ret;
		}
		m = m->getParent();
	}
	return NULL;
}

std::string CGRAXMLCompile::Port::getFullName()
{
	Module *mod = this->mod;
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

void CGRAXMLCompile::Port::increastCongCost()
{
	if (history_cost == 0)
	{
		history_cost = INIT_CONG_COST / 10;
	}
	else
	{
		history_cost = history_cost + history_cost / 10;
	}
}

int CGRAXMLCompile::Port::getCongCost()
{

	//	if()

	int cost = base_cost * number_signals + history_cost * (number_signals + 1);
	return cost;
}


void CGRAXMLCompile::Port::erase(DFGNode * eraseNode, int  node_value_dest){
	// std::cout<<this->getFullName()<<"\n";
	// std::cout<<" before erase \n";
	// for(auto it =  mapped_nodes.begin(); it != mapped_nodes.end(); it++){
	// 	std::cout<<std::get<0>(*it)->idx<<","<<std::get<1>(*it)<<","<<std::get<2>(*it)<<" \t";
	// }
	// std::cout<<"\n";

	assert(this->mod->getCGRA());
	if (eraseNode != NULL)
	{
		(*this->mod->getCGRA()->getCongestedPortPtr())[this].erase(eraseNode);
	}

	for (Port *p : getMod()->getConflictPorts(this))
	{
		p->decreaseUse(eraseNode);
		(*p->mod->getCGRA()->getCongestedPortPtr())[p].erase(eraseNode);
	}

	if (this->getType() == OUT)
	{
		for (Port *p : getMod()->getParent()->getConflictPorts(this))
		{
			p->decreaseUse(eraseNode);
			(*p->mod->getCGRA()->getCongestedPortPtr())[p].erase(eraseNode);
		}
	}

	bool find_node = false;
	int found_node_num = 0;

	for(auto it =  mapped_nodes.begin(); it != mapped_nodes.end(); ){
		if(std::get<0>(*it)->idx == eraseNode->idx && std::get<1>(*it) == node_value_dest){
			find_node = true;
			it = mapped_nodes.erase(it);
			found_node_num ++;
			// break;
		}else{
			it++;
		}
	}
			
	assert(found_node_num < 2);

	// std::cout<<" after erase \n";
	// for(auto it =  mapped_nodes.begin(); it != mapped_nodes.end(); it++){
	// 	std::cout<<std::get<0>(*it)->idx<<","<<std::get<1>(*it)<<","<<std::get<2>(*it)<<" \t";
	// }
	// std::cout<<"\n";	

	assert(find_node);
	if(mapped_nodes.size() == 0){
		number_signals = 0;
		latency = -1;
	}else{
		number_signals = mapped_nodes.size(); 
		latency = std::get<2>(*(mapped_nodes.rbegin())); 
	}
	
}
void CGRAXMLCompile::Port::clear()
{
	// std::cout<<"call clear\n";
	assert(this->mod->getCGRA());

	if(mapped_nodes.size() == 0 ) {
		return;
	}

	DFGNode* mappedNode = std::get<0>(*(mapped_nodes.rbegin())); 
	if (mappedNode != NULL)
	{
		(*this->mod->getCGRA()->getCongestedPortPtr())[this].erase(mappedNode);
	}

	for (Port *p : getMod()->getConflictPorts(this))
	{
		p->decreaseUse(mappedNode);
		(*p->mod->getCGRA()->getCongestedPortPtr())[p].erase(mappedNode);
	}

	if (this->getType() == OUT)
	{
		for (Port *p : getMod()->getParent()->getConflictPorts(this))
		{
			p->decreaseUse(mappedNode);
			(*p->mod->getCGRA()->getCongestedPortPtr())[p].erase(mappedNode);
		}
	}

	mapped_nodes.clear();
	// mappedNode = NULL;
	number_signals = 0;
	latency = -1;
}

void CGRAXMLCompile::Port::setLat(int lat)
{
	// std::cout<<this->getFullName()<<"set latency:"<<lat<<"\n";
	CGRA *cgra = this->getMod()->getCGRA();
	int ii = cgra->get_t_max();
	assert(lat % ii == this->getMod()->getPE()->T);
	latency = lat;
}

int CGRAXMLCompile::Port::getLat()
{
	CGRA *cgra = this->getMod()->getCGRA();
	int ii = cgra->get_t_max();
	assert(latency % ii == this->getMod()->getPE()->T);
	return latency;
}

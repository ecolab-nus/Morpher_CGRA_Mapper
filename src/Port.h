/*
 * Port.h
 *
 *  Created on: 26 Feb 2018
 *      Author: manupa
 */

#ifndef PORT_H_
#define PORT_H_

#include "DFGEdge.h"
#include "DFGNode.h"
#include <string>
#include <vector>
#include <map>
#include <assert.h>

#define INIT_CONG_COST 100000

namespace CGRAXMLCompile
{

class PE;
class Module;
class HeuristicMapper;

enum PortType
{
	IN,
	OUT,
	INT,
	REGI,
	REGO,
	SOCKET
};

class Port
{
public:
	Port(std::string name, PortType pType, Module *mod);
	std::string getName() { return name; }
	std::string getFullName();

	void clear();
	void erase(DFGNode * eraseNode);
	PE *findParentPE();
	Module *getMod() { return mod; }
	PortType getType() { return pType; }

	void setNode(DFGNode *node, int latency, HeuristicMapper *hm = NULL);

	DFGNode *getNode() { 
		if(mapped_nodes.size() == 0){
			return NULL;
		}else{
			return this->mapped_nodes.rbegin()->first; 
		}
	}

	int getCongCost();
	void increastCongCost();
	void increaseUse(HeuristicMapper *hm = NULL);
	void decreaseUse(DFGNode *extnode, HeuristicMapper *hm = NULL);
	void increaseConflictedUse(DFGNode *node, HeuristicMapper *hm = NULL);
	int getHistoryCost() { return history_cost; }
	int getLat();
	void setLat(int lat);

	PE* getPE(){return pe;}

private:
	std::string name;
	Module *mod;
	PE* pe;
	PortType pType;
	// DFGNode *mappedNode = NULL; // this is the latest node that this port stores
	std::vector<std::pair<DFGNode *, int >> mapped_nodes; // <node, lat> means latency
	std::map<DFGNode *, std::set<int>> node_to_dest_map; 
	int latency = -1;

	int base_cost = INIT_CONG_COST;
	int history_cost = 0;
	int number_signals = 0; 
	// the # of  singnals should be equal to the sum of # of congested nodes and # of conflicted nodes.

	bool operator==(const Port &other)
	{
		return (this->name.compare(other.name) == 0);
	}
};

} /* namespace CGRAXMLCompile */

#endif /* PORT_H_ */

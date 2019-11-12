/*
 * dfgnode.h
 *
 *  Created on: 20 Feb 2018
 *      Author: manupa
 */

#ifndef DFGNODE_H_
#define DFGNODE_H_

#include <string>
#include <vector>
#include <map>
#include <set>

namespace CGRAXMLCompile
{

class DataPath;
class Port;
class DFG;

class DFGNode
{
public:
	DFGNode();
	int idx;
	int ASAP;
	int ALAP;
	int constant;
	bool hasConst = false;
	bool npb;
	std::string op;
	std::string base_pointer_name;
	int gep_offset = -1;
	std::vector<DFGNode *> parents;
	std::vector<DFGNode *> phiParents;
	std::vector<DFGNode *> recParents;
	std::vector<DFGNode *> children;
	std::vector<DFGNode *> phiChildren;

	DataPath *rootDP = NULL;
	std::vector<std::pair<Port *, int>> routingPorts;
	//	std::map<Port*,int> routingPortDestMap;

	std::map<DFGNode *, std::string> childrenOPType;
	std::map<DFGNode *, int> childNextIter;

	std::string BB;

	void clear(DFG *dfg);
	std::string getOPtype(DFGNode *child);
	bool isMemOp();

	bool operator==(const DFGNode &rhs)
	{
		return this->idx == rhs.idx;
	}
	std::set<DataPath *> blacklistDest;

private:
};

} /* namespace CGRAXMLCompile */

#endif /* DFGNODE_H_ */

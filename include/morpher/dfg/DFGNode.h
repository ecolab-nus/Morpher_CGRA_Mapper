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
	bool npb=false;
	bool type_i1i2 = false; //  true if i1 and i2 both get the data from same source (to support x*x)
	bool negated=false;
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

	int align;
	void clear(DFG *dfg);
	void CarefulClear(DFG *dfg);
	// the difference is, for overuse, SA stores multiple values in one port, but pathfinder only stores the last value
	std::string getOPtype(DFGNode *child);
	bool isMemOp();

	bool operator==(const DFGNode &rhs)
	{
		return this->idx == rhs.idx;
	}
	std::set<DataPath *> blacklistDest;
	std::string getBinaryString();
	std::string get27bitConstantBinaryString();

private:
};

} /* namespace CGRAXMLCompile */

#endif /* DFGNODE_H_ */

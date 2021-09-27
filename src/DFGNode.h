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

//#define HIERARCHICAL
#define CLUSTERED_ARCH
#define NOTIMEDISTANCEFUNC

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
#ifdef HIERARCHICAL
	int TILE;
	int DFG_CLUSTER;
	std::vector<std::string> CGRA_CLUSTERS;
#endif
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
	std::map<DFGNode *, std::string> childrenEdgeType;//INTRA-intra cluster (within the cluster), INTER - inter cluster (between clusters)

	std::string BB;

	int align;
	void clear(DFG *dfg);
	std::string getOPtype(DFGNode *child);
	bool isMemOp();

	bool operator==(const DFGNode &rhs)
	{
		return this->idx == rhs.idx;
	}
	std::set<DataPath *> blacklistDest;
	std::string getBinaryString();
	std::string get27bitConstantBinaryString();

    bool in_rec_cycle = false;

private:
};

} /* namespace CGRAXMLCompile */

#endif /* DFGNODE_H_ */

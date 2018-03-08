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

namespace CGRAXMLCompile {

class DataPath;
class Port;


class DFGNode {
public:
	DFGNode();
	int idx;
	int ASAP;
	std::string op;
	std::vector<DFGNode*> parents;
	std::vector<DFGNode*> phiParents;
	std::vector<DFGNode*> children;
	std::vector<DFGNode*> phiChildren;

	DataPath* rootDP=NULL;
	std::vector<Port*> routingPorts;
	std::map<Port*,DFGNode*> routingPortDestMap;

	std::map<DFGNode*,std::string> childrenOPType;

	std::string BB;

	void clear();
	std::string getOPtype(DFGNode* child);

	bool operator==(const DFGNode& rhs){
		return this->idx == rhs.idx;
	}

};

} /* namespace CGRAXMLCompile */

#endif /* DFGNODE_H_ */

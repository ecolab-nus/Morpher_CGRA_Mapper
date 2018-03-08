/*
 * DFG.h
 *
 *  Created on: 28 Feb 2018
 *      Author: manupa
 */

#ifndef DFG_H_
#define DFG_H_

#include <vector>
#include <set>
#include <stack>
#include "DFGNode.h"

namespace CGRAXMLCompile {


class DFG {
public:
	DFG();
	std::vector<DFGNode> nodeList;

	bool parseXML(std::string fileName);
	void printDFG();
	DFGNode* findNode(int idx);
	DFGNode* getNodePtr(int it);
	std::map<std::string,std::set<std::string>> mutexBBs;

	std::vector<std::set<DFGNode*>> getSCCs();

private:
	void strongconnect(DFGNode* v,
					   std::map<DFGNode*,int>& v_idx,
					   std::map<DFGNode*,int>& v_lowlink,
					   std::map<DFGNode*,bool>& v_onstack,
					   int& idx,
					   std::stack<DFGNode*>& S,
			           std::vector<std::set<DFGNode*>>& result);


};

} /* namespace CGRAXMLCompile */

#endif /* DFG_H_ */

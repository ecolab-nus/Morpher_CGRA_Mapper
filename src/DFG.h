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
#include <unordered_set>
#include <unordered_map>
#include "DFGNode.h"

namespace CGRAXMLCompile
{

typedef std::pair<DFGNode *, DFGNode *> BackEdge;

class DFG
{
public:
	DFG();
	std::vector<DFGNode> nodeList;

	bool parseXML(std::string fileName);
	void printDFG();
	DFGNode *findNode(int idx);
	DFGNode *getNodePtr(int it);
	std::map<std::string, std::set<std::string>> mutexBBs;

	std::vector<std::set<DFGNode *>> getSCCs();
	int unmappedMemOps = 0;
	std::set<DFGNode *> unmappedMemOpSet;

	bool isMutexNodes(DFGNode *a, DFGNode *b, Port *p);
	bool getAncestoryASAPUntil(DFGNode *beParent, DFGNode *beChild, std::set<DFGNode *> &result);
	std::vector<DFGNode *> getAncestoryASAP(const DFGNode *node);
	std::vector<DFGNode *> getAncestoryALAP(const DFGNode *node);
	std::vector<DFGNode *> mergeAncestoryASAP(const std::vector<DFGNode *> &in1,
											  const std::vector<DFGNode *> &in2,
											  const std::map<BackEdge, std::set<DFGNode *>> &RecCycles);
	std::vector<DFGNode *> mergeAncestoryALAP(const std::vector<DFGNode *> &in1, const std::vector<DFGNode *> &in2);

	std::unordered_map<std::string,int> pointer_sizes;
	std::unordered_map<std::string,int> ldst_pointer_sizes;

	bool dfg_parse_lisa_training = false;  
	// this is to fix a weird bug in tinyxml parser.
	// bug happens if the name of child element is a substr of the current element.

private:
	void strongconnect(DFGNode *v,
					   std::map<DFGNode *, int> &v_idx,
					   std::map<DFGNode *, int> &v_lowlink,
					   std::map<DFGNode *, bool> &v_onstack,
					   int &idx,
					   std::stack<DFGNode *> &S,
					   std::vector<std::set<DFGNode *>> &result);
};

} /* namespace CGRAXMLCompile */

#endif /* DFG_H_ */

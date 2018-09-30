/*
 * DFG.cpp
 *
 *  Created on: 28 Feb 2018
 *      Author: manupa
 */

#include "DFG.h"
#include <assert.h>
#include <iostream>
#include <queue>

#include "tinyxml2.h"
using namespace tinyxml2;

namespace CGRAXMLCompile {

DFG::DFG() {
	// TODO Auto-generated constructor stub

}

bool DFG::parseXML(std::string fileName) {
	XMLDocument doc;
	doc.LoadFile(fileName.c_str());


	XMLElement* MutexBB = doc.FirstChildElement("MutexBB");
	XMLElement* BB1 = MutexBB->FirstChildElement("BB1");
	while(BB1){
		XMLElement* BB2 = BB1->FirstChildElement("BB2");
		while(BB2){
			const char* BB1Name;
			BB1->QueryStringAttribute("name",&BB1Name);
			std::string BB1NameStr(BB1Name);
			const char* BB2Name;
			BB2->QueryStringAttribute("name",&BB2Name);
			std::string BB2NameStr(BB2Name);
			mutexBBs[BB1NameStr].insert(BB2NameStr);

			BB2 = BB2->NextSiblingElement("BB2");
		}
		BB1 = BB1->NextSiblingElement("BB1");
	}

	XMLElement* DFG = doc.FirstChildElement("DFG");
	int node_count;
	DFG->QueryIntAttribute("count",&node_count);
//	std::map<int,DFGNode*> parsedNodes;



	//parse all nodes
	XMLElement* node=NULL;
	std::cout << "xmlParse :: node iteration...\n";
	for (int i = 0; i < node_count; ++i) {

		std::cout << "i=" << i << ",";
		if(node==NULL){
			node = DFG->FirstChildElement("Node");
		}
		else{
			node = node->NextSiblingElement("Node");
		}
		assert(node);

		DFGNode currDFGNode;
		int idx;
		node->QueryIntAttribute("idx",&idx);
		int asap;
		node->QueryIntAttribute("ASAP",&asap);
		currDFGNode.ASAP = asap;

		const char* BBName;
		node->QueryStringAttribute("BB",&BBName);
		std::string BBNameStr(BBName);
		currDFGNode.BB=BBNameStr;

		int constant;
		if(node->QueryIntAttribute("CONST",&constant) == XML_SUCCESS){
			currDFGNode.constant = constant;
			currDFGNode.hasConst = true;
		}

		std::cout << ",idx=" << idx << ",";
		currDFGNode.idx=idx;

		XMLElement* op = node->FirstChildElement("OP");
		const char* opName = op->GetText();
		std::cout << ",op=" << opName << "\n";

		currDFGNode.op = std::string(opName);
		this->nodeList.push_back(currDFGNode);

	}

	std::cout << "NODELIST SIZE = " << this->nodeList.size() << "\n";



	//parse all connections

	std::cout << "Parsing Connections...\n";
	node=NULL;
	for (int i = 0; i < node_count; ++i) {
		if(node==NULL){
			node = DFG->FirstChildElement("Node");
		}
		else{
			node = node->NextSiblingElement("Node");
		}
		assert(node);

		int idx;
		node->QueryIntAttribute("idx",&idx);
		std::cout << "idx=" << idx;


		XMLElement* Inputs = node->FirstChildElement("Inputs");
		assert(Inputs);
		std::cout << "|Inputs=";

		XMLElement* input = Inputs->FirstChildElement("Input");
		while(input){
			int input_idx;
			input->QueryIntAttribute("idx",&input_idx);
			std::cout << input_idx << ",";

			findNode(idx)->parents.push_back(findNode(input_idx));


			input = input->NextSiblingElement("Input");
		}

		XMLElement* Outputs = node->FirstChildElement("Outputs");
		assert(Outputs);
		std::cout << "|Outputs=";

		XMLElement* output = Outputs->FirstChildElement("Output");
		while(output){
			int output_idx;
			output->QueryIntAttribute("idx",&output_idx);
			std::cout << output_idx << ",";

			findNode(idx)->children.push_back(findNode(output_idx));

			int output_nextiter;
			output->QueryIntAttribute("nextiter",&output_nextiter);
			std::cout << "nextiter = " << output_nextiter;
			assert(output_nextiter == 0 || output_nextiter == 1);
			findNode(idx)->childNextIter[findNode(output_idx)]=output_nextiter;

			const char* type;
			output->QueryStringAttribute("type",&type);
			if(type){
				findNode(idx)->childrenOPType[findNode(output_idx)]=std::string(type);
			}

			output = output->NextSiblingElement("Output");
		}

		XMLElement* RecParents = node->FirstChildElement("RecParents");
		assert(RecParents);
		std::cout << "|RecParents=";

		XMLElement* RecParent = RecParents->FirstChildElement("RecParent");
		while(RecParent){
			int RecParent_idx;
			RecParent->QueryIntAttribute("idx",&RecParent_idx);
			std::cout << RecParent_idx << ",";

			findNode(idx)->recParents.push_back(findNode(RecParent_idx));
			RecParent = RecParent->NextSiblingElement("RecParent");
		}
		std::cout << "\n";
	}
}

void DFG::printDFG() {
	std::cout << "Printing DFG...\n";
	for(DFGNode& node : nodeList){
		std::cout << "idx=" << node.idx << ",OP=" << node.op;

		std::cout << "|Parents=";
		for(DFGNode* parent : node.parents){
			std::cout << parent->idx << ",";
		}

		std::cout << "|Children=";
		for(DFGNode* child : node.children){
			std::cout << child->idx << ",";
		}
		std::cout << "\n";
	}
	std::cout << "Printing DFG Done!\n";
}

DFGNode* DFG::findNode(int idx) {
	for(DFGNode& node : nodeList){
		if(node.idx==idx){
			return & node;
		}
	}
}

DFGNode* DFG::getNodePtr(int it) {
	assert(it < nodeList.size());

	std::vector<DFGNode>::iterator iter;
	iter = nodeList.begin()+it;
	return &(*iter);
}

std::vector<std::set<DFGNode*> > DFG::getSCCs() {

	  int index = 0;
	  std::stack<DFGNode*> S;
	  std::map<DFGNode*,int> v_idx;
	  std::map<DFGNode*,int> v_lowlink;
	  std::map<DFGNode*,bool> v_onstack;
	  std::vector<std::set<DFGNode*>> res;

	  for(DFGNode& V : nodeList){
		  DFGNode* v = &V;
		  if(v_idx.find(v)==v_idx.end()){
			  strongconnect(v,v_idx,v_lowlink,v_onstack,index,S,res);
		  }
	  }
	  return res;
}

bool DFG::isMutexNodes(DFGNode* a, DFGNode* b) {

	if(mutexBBs.find(a->BB)==mutexBBs.end()){
		return false;
	}

	if(mutexBBs.find(b->BB)==mutexBBs.end()){
		return false;
	}

	if(mutexBBs[a->BB].find(b->BB)!=mutexBBs[a->BB].end()){
		return true;
	}

	if(mutexBBs[b->BB].find(a->BB)!=mutexBBs[b->BB].end()){
		return true;
	}

	return false;
}

std::vector<DFGNode*> DFG::mergeAncestory(const std::vector<DFGNode*>& in1,
		const std::vector<DFGNode*>& in2) {

	std::map<int,std::vector<DFGNode*>> asapLevelNodeList;
	for(DFGNode* node : in1){
		asapLevelNodeList[node->ASAP].push_back(node);
	}
	for(DFGNode* node : in2){
		asapLevelNodeList[node->ASAP].push_back(node);
	}

	int maxASAPlevel=0;
	for(std::pair<int,std::vector<DFGNode*>> pair : asapLevelNodeList){
		if(pair.first > maxASAPlevel){
			maxASAPlevel = pair.first;
		}
	}

	std::vector<DFGNode*> res;
	for (int i = 0; i <= maxASAPlevel; ++i) {
		for(DFGNode* node : asapLevelNodeList[i]){
			res.push_back(node);
		}
	}
	return res;
}

void DFG::strongconnect(DFGNode* v,
						std::map<DFGNode*,int>& v_idx,
						std::map<DFGNode*,int>& v_lowlink,
						std::map<DFGNode*,bool>& v_onstack,
						int& idx,
						std::stack<DFGNode*>& S,
		                std::vector<std::set<DFGNode*>>& result) {


			v_idx[v]=idx;
			v_lowlink[v]=idx;

			idx++;
			S.push(v);
			v_onstack[v]=true;

			std::cout << "v=" << v->idx << ",idx=" << idx << "\n";

			for(DFGNode* child : v->children){
				if(v_idx.find(child)==v_idx.end()){
					strongconnect(child,v_idx,v_lowlink,v_onstack,idx,S,result);
					v_lowlink[v] = std::min(v_lowlink[v], v_lowlink[child]);
				}
				else if(v_onstack[child]){
					v_lowlink[v] = std::min(v_lowlink[v],v_idx[child]);
				}
			}

			std::cout << "v=" << v->idx << ",idx=" << idx  << ",v_lowlink[v]=" << v_lowlink[v] << "\n";

			if(v_lowlink[v] == v_idx[v]){
				std::cout << "found SCC::\n";
				std::set<DFGNode*> scc;
				DFGNode* w;
				do{
					w = S.top();
					S.pop();
					scc.insert(w);
					std::cout << w->idx << ",";
					v_onstack[w]=false;
				}while(w != v);
				std::cout << "\n";

				std::cout << "res.size = " << result.size() << "\n";
				result.push_back(scc);
				std::cout << "res.size = " << result.size() << "\n";
			}
}

std::vector<DFGNode*> DFG::getAncestory(const DFGNode* node) {
	std::stack<const DFGNode*> ancestors;

	std::queue<const DFGNode*> q;
	q.push(node);
	ancestors.push(node);

	while(!q.empty()){
		const DFGNode* top = q.front(); q.pop();
		for(DFGNode* parent : top->parents){
			if(parent->ASAP >= top->ASAP) continue; //ignore backedges
			std::cout << parent->idx << ",";
			ancestors.push(parent);
			q.push(parent);
		}
	}

	std::vector<DFGNode*> res;
	while(!ancestors.empty()){
		res.push_back((DFGNode*)ancestors.top()); ancestors.pop();
	}

	return res;
}


} /* namespace CGRAXMLCompile */


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
#include "Port.h"
#include <algorithm>
#include <list>
#include <fstream>
#include <sstream>

#include "tinyxml2.h"
using namespace tinyxml2;

namespace CGRAXMLCompile
{

DFG::DFG()
{
	// TODO Auto-generated constructor stub
}

bool DFG::parseXML(std::string fileName)
{

#ifdef HIERARCHICAL
	std::string clustering_outcome_file_name = "dfg_clustering_result.txt";
	std::ifstream clustering_outcome(clustering_outcome_file_name.c_str());
	std::map<int, int> nodeid_to_clusterid;

	std::string line;
//	std::cout<< "clustering:";
	while(std::getline(clustering_outcome,line)){
		std::istringstream iss2(line);

		std::string node_id;
		std::string cluster_id;

		std::getline(iss2,node_id,'\t');
		std::getline(iss2,cluster_id,',');


		nodeid_to_clusterid[atoi(node_id.c_str())]= atoi(cluster_id.c_str());
//		std::cout << node_id << "," << nodeid_to_clusterid[atoi(node_id.c_str())] << "\n";
	}
	std::string interclusteredges_file_name = "inter_cluster_edges.txt";
	std::ifstream interclusteredges(interclusteredges_file_name.c_str());
	std::set< std::pair<int,int>> intercluster_edges;


	while(std::getline(interclusteredges,line)){
		std::istringstream iss2(line);

		std::string edge_0;
		std::string edge_1;

		std::getline(iss2,edge_0,'\t');
		std::getline(iss2,edge_1,',');

		intercluster_edges.insert( std::make_pair(atoi(edge_0.c_str()),atoi(edge_1.c_str())) );
	}
	/*
	 * Read DFG cluster to CGRA cluster mapping txt file
	 * CGRA cluster is represented by row and column ID pair
	 * One DFG cluster can be mapped onto multiple CGRA clusters
	 * Therefore it needs a vector of pairs to hold CGRA cluster information
	 * */
	std::string dfg_to_cgra_cluster_mapping_outcome_file_name = "cluster_mapping_result.txt";
	std::ifstream dfg_to_cgra_cluster_mapping(dfg_to_cgra_cluster_mapping_outcome_file_name.c_str());
//	std::map<int,std::vector<std::pair<int,int>> > dfg_cluster_to_cgra_cluster;
	std::map<int,std::vector<std::string> > dfg_cluster_to_cgra_cluster;

    int number_of_clusters = 0;
	while(std::getline(dfg_to_cgra_cluster_mapping,line)){
		std::istringstream iss3(line);

		std::string dfg_cluster_id;
		std::string rows;
		std::string columns;

		std::getline(iss3,dfg_cluster_id,'\t');
		std::getline(iss3,rows,'\t');
		std::getline(iss3,columns,'.');

		std::cout << "dfg_cluster_id=" << dfg_cluster_id << ",rows=" << rows << ",columns=" << columns << "\n";
		number_of_clusters = std::stoi(dfg_cluster_id);
		//int row_size =
		std::string rows_ = rows.substr(1,(rows.size()-2));
		std::string cols_ = columns.substr(1,(columns.size()-2));
		int start = 0;
		int end = rows_.find(", ");
		while(end !=-1){
			std::string row = rows_.substr(start,end-start);
			std::string col = cols_.substr(start,end-start);
//			std::cout << row<<"\t";
//			std::cout << col<<"\n";

			//dfg_cluster_to_cgra_cluster[std::stoi(dfg_cluster_id)].push_back(std::make_pair(std::stoi(row),std::stoi(col)));
			std::string tile_name = "TILE_X"+col+"|_Y"+row;
			dfg_cluster_to_cgra_cluster[std::stoi(dfg_cluster_id)].push_back(tile_name);

			start = end + 2;
			end = rows_.find(", ",start);

		}
		std::string row = rows_.substr(start,end-start);
		std::string col = cols_.substr(start,end-start);
//		std::cout << row<<"\t";
//		std::cout << col<<"\n";
//		dfg_cluster_to_cgra_cluster[std::stoi(dfg_cluster_id)].push_back(std::make_pair(std::stoi(row),std::stoi(col)));

		std::string tile_name = "TILE_X"+col+"|_Y"+row;
		dfg_cluster_to_cgra_cluster[std::stoi(dfg_cluster_id)].push_back(tile_name);

	}
	std::cout<<"Cluster ID: (row,col)";
	for (int i =0;i<=number_of_clusters;i++){
		std::cout << i <<":";
		for(auto tile_name : dfg_cluster_to_cgra_cluster[i]){
			std::cout << tile_name <<"\t";
		}
		std::cout<<"\n";
	}
//	std::cout<< "inter cluster edges:";
//	for ( const auto& edge : intercluster_edges )
//	        std::cout << edge.first << ", " << edge.second << "\n";
#endif
	XMLDocument doc;
	doc.LoadFile(fileName.c_str());

	XMLElement *MutexBB = doc.FirstChildElement("MutexBB");
	XMLElement *BB1 = MutexBB->FirstChildElement("BB1");
	while (BB1)
	{
		XMLElement *BB2 = BB1->FirstChildElement("BB2");
		while (BB2)
		{
			const char *BB1Name;
			BB1->QueryStringAttribute("name", &BB1Name);
			std::string BB1NameStr(BB1Name);
			const char *BB2Name;
			BB2->QueryStringAttribute("name", &BB2Name);
			std::string BB2NameStr(BB2Name);
			mutexBBs[BB1NameStr].insert(BB2NameStr);

			BB2 = BB2->NextSiblingElement("BB2");
		}
		BB1 = BB1->NextSiblingElement("BB1");
	}

	XMLElement *DFG = doc.FirstChildElement("DFG");
	int node_count;
	DFG->QueryIntAttribute("count", &node_count);
	//	std::map<int,DFGNode*> parsedNodes;

	//parse all nodes
	XMLElement *node = NULL;
	std::cout << "xmlParse :: node iteration...\n";
	for (int i = 0; i < node_count; ++i)
	{

		std::cout << "i=" << i << ",";
		if (node == NULL)
		{
			node = DFG->FirstChildElement("Node");
		}
		else
		{
			node = node->NextSiblingElement("Node");
		}
		assert(node);

		DFGNode currDFGNode;
		int idx;
		node->QueryIntAttribute("idx", &idx);
		int asap = -1;
		node->QueryIntAttribute("ASAP", &asap);
		assert(asap != -1);
		currDFGNode.ASAP = asap;
		int alap = -1;
		node->QueryIntAttribute("ALAP", &alap);
		assert(alap != -1);
		currDFGNode.ALAP = alap;
#ifdef HIERARCHICAL
		//int tile = -1;
		//node->QueryIntAttribute("TILE", &tile);
		//assert(tile != -1);
		//currDFGNode.TILE = tile;
		assert(nodeid_to_clusterid.count(idx));
		currDFGNode.DFG_CLUSTER = nodeid_to_clusterid[idx];
		currDFGNode.CGRA_CLUSTERS = dfg_cluster_to_cgra_cluster[nodeid_to_clusterid[idx]];
#endif

		const char *BBName;
		node->QueryStringAttribute("BB", &BBName);
		std::string BBNameStr(BBName);
		currDFGNode.BB = BBNameStr;

		int constant;
		if (node->QueryIntAttribute("CONST", &constant) == XML_SUCCESS)
		{
			currDFGNode.constant = constant;
			currDFGNode.hasConst = true;
		}

		std::cout << ",idx=" << idx << ",";
		currDFGNode.idx = idx;

		XMLElement *op = node->FirstChildElement("OP");
		const char *opName = op->GetText();
		std::cout << ",op=" << opName << "\n";
		currDFGNode.op = std::string(opName);

//		XMLElement *base_pointer_name = node->FirstChildElement("BasePointerName");
//		if(base_pointer_name){
//			const char *base_pointer_name_str = base_pointer_name->GetText();
//			std::cout << ",base_pointer_name=" << base_pointer_name_str << "\n";
//			currDFGNode.base_pointer_name = std::string(base_pointer_name_str);

//			int base_pointer_size = -1;
//			base_pointer_name->QueryIntAttribute("size", &base_pointer_size);
//			std::cout << ",base_pointer_size=" << base_pointer_size << "\n";
//			assert(base_pointer_size != -1);
//
//			if(pointer_sizes.find(currDFGNode.base_pointer_name) == pointer_sizes.end()){
//				pointer_sizes[currDFGNode.base_pointer_name] = base_pointer_size;
//			}
//			else{
//				//if its already there it should be equal
//				assert(pointer_sizes[currDFGNode.base_pointer_name] == base_pointer_size);
//			}
//
//			if(currDFGNode.op.find("LOAD") != std::string::npos || currDFGNode.op.find("STORE") != std::string::npos){
//				ldst_pointer_sizes[currDFGNode.base_pointer_name] = base_pointer_size;
//			}

//		}

		XMLElement *GEPOffset = node->FirstChildElement("GEPOffset");
		if(GEPOffset){
			int gep_offest = atoi(GEPOffset->GetText());
			currDFGNode.gep_offset = gep_offest;
		}

		this->nodeList.push_back(currDFGNode);
	}

	std::cout << "NODELIST SIZE = " << this->nodeList.size() << "\n";

	//parse all connections

	std::cout << "Parsing Connections...\n";
	node = NULL;
	for (int i = 0; i < node_count; ++i)
	{
		if (node == NULL)
		{
			node = DFG->FirstChildElement("Node");
		}
		else
		{
			node = node->NextSiblingElement("Node");
		}
		assert(node);

		int idx;
		node->QueryIntAttribute("idx", &idx);
		std::cout << "idx=" << idx;

		XMLElement *Inputs = node->FirstChildElement("Inputs");
		assert(Inputs);
		std::cout << "|Inputs=";

		XMLElement *input = Inputs->FirstChildElement("Input");
		while (input)
		{
			int input_idx;
			input->QueryIntAttribute("idx", &input_idx);
			std::cout << input_idx << ",";

			findNode(idx)->parents.push_back(findNode(input_idx));

			input = input->NextSiblingElement("Input");
		}

		XMLElement *Outputs = node->FirstChildElement("Outputs");
		assert(Outputs);
		std::cout << "|Outputs=";

		XMLElement *output = Outputs->FirstChildElement("Output");
		while (output)
		{
			int output_idx;
			output->QueryIntAttribute("idx", &output_idx);
			std::cout << output_idx << ",";

			findNode(idx)->children.push_back(findNode(output_idx));

			int output_nextiter;
			output->QueryIntAttribute("nextiter", &output_nextiter);
			std::cout << "nextiter = " << output_nextiter;
			assert(output_nextiter == 0 || output_nextiter == 1);
			findNode(idx)->childNextIter[findNode(output_idx)] = output_nextiter;




			const char *type;
			output->QueryStringAttribute("type", &type);
			if (type)
			{
				if(std::string(type)=="I3"){
					findNode(idx)->childrenOPType[findNode(output_idx)] = std::string("I1");
					findNode(output_idx)->type_i1i2 = true;
				}else{
					findNode(idx)->childrenOPType[findNode(output_idx)] = std::string(type);
				}

				//				if(type == 0){
				if(std::string(type)=="P"){
					int npb;
					output->QueryIntAttribute("NPB", &npb);
					std::cout << "npb = " << npb;
					assert(npb == 0 || npb == 1);
					if(npb == 1){
						findNode(output_idx)->npb = true;
					}
					else{
						findNode(output_idx)->npb = false;
					}
				}

			}
#ifdef HIERARCHICAL
			if(intercluster_edges.find(std::make_pair(idx,output_idx)) != intercluster_edges.end()){
				findNode(idx)->childrenEdgeType[findNode(output_idx)] = "INTER";
				//std::cout << "Inter edge:"<< idx<<","<<output_idx <<" \n";
			}else if (output_nextiter == 1){
				findNode(idx)->childrenEdgeType[findNode(output_idx)] = "INTER"; // let backedges to use inter cluster routing
			}else{
				findNode(idx)->childrenEdgeType[findNode(output_idx)] = "INTRA";
				//std::cout << "Intra edge:"<< idx<<","<<output_idx <<" \n";
			}
#endif

			output = output->NextSiblingElement("Output");
		}

		XMLElement *RecParents = node->FirstChildElement("RecParents");
		assert(RecParents);
		std::cout << "|RecParents=";

		XMLElement *RecParent = RecParents->FirstChildElement("RecParent");
		while (RecParent)
		{
			int RecParent_idx;
			RecParent->QueryIntAttribute("idx", &RecParent_idx);
			std::cout << RecParent_idx << ",";

			findNode(idx)->recParents.push_back(findNode(RecParent_idx));
			RecParent = RecParent->NextSiblingElement("RecParent");
		}
		std::cout << "\n";
	}

}

void DFG::printDFG()
{
	std::cout << "Printing DFG...\n";
	for (DFGNode &node : nodeList)
	{
		std::cout << "idx=" << node.idx << ",OP=" << node.op;

		std::cout << "|Parents=";
		for (DFGNode *parent : node.parents)
		{
			std::cout << parent->idx << ",";
		}

		std::cout << "|Children=";
		for (DFGNode *child : node.children)
		{
			std::cout << child->idx << ",";
		}
		std::cout << "\n";
#ifdef HIERARCHICAL
		std::cout << "CGRA Clusters: ";
		for(auto tile_name: node.CGRA_CLUSTERS){
			std::cout << tile_name<< "\t";
		}
		std::cout << "\n";
#endif
	}
	std::cout << "Printing DFG Done!\n";
}

DFGNode *DFG::findNode(int idx)
{
	for (DFGNode &node : nodeList)
	{
		if (node.idx == idx)
		{
			return &node;
		}
	}
}

DFGNode *DFG::getNodePtr(int it)
{
	assert(it < nodeList.size());

	std::vector<DFGNode>::iterator iter;
	iter = nodeList.begin() + it;
	return &(*iter);
}

std::vector<std::set<DFGNode *>> DFG::getSCCs()
{

	int index = 0;
	std::stack<DFGNode *> S;
	std::map<DFGNode *, int> v_idx;
	std::map<DFGNode *, int> v_lowlink;
	std::map<DFGNode *, bool> v_onstack;
	std::vector<std::set<DFGNode *>> res;

	for (DFGNode &V : nodeList)
	{
		DFGNode *v = &V;
		if (v_idx.find(v) == v_idx.end())
		{
			strongconnect(v, v_idx, v_lowlink, v_onstack, index, S, res);
		}
	}
	return res;
}

bool DFG::isMutexNodes(DFGNode *a, DFGNode *b, Port *p)
{

	//Making the operand port mutex
	if ((p->getName() == "P") ||
			(p->getName().find("_P") != std::string::npos) ||
			(p->getName().find("I1") != std::string::npos) ||
			(p->getName().find("I2") != std::string::npos))
	{
		return true;
	}

	if ((p->getName().find("P") == std::string::npos) && (p->getName().find("I1") == std::string::npos) && (p->getName().find("I2") == std::string::npos))
	{
		return false;
	}

	if (mutexBBs.find(a->BB) == mutexBBs.end())
	{
		return false;
	}

	if (mutexBBs.find(b->BB) == mutexBBs.end())
	{
		return false;
	}

	if (mutexBBs[a->BB].find(b->BB) != mutexBBs[a->BB].end())
	{
		return true;
	}

	if (mutexBBs[b->BB].find(a->BB) != mutexBBs[b->BB].end())
	{
		return true;
	}

	return false;
}

std::vector<DFGNode *> removeDuplicates(std::vector<DFGNode *> vec_in)
		{
	std::vector<DFGNode *> res;
	for (DFGNode *n : vec_in)
	{
		if (std::find(res.begin(), res.end(), n) == res.end())
		{
			res.push_back(n);
		}
	}
	return res;
		}

std::vector<DFGNode *> DFG::mergeAncestoryASAP(const std::vector<DFGNode *> &in1,
		const std::vector<DFGNode *> &in2,
		const std::map<BackEdge, std::set<DFGNode *>> &RecCycles)
{

	std::map<DFGNode *, std::vector<DFGNode *>> beParentAncestors;
	std::map<DFGNode *, DFGNode *> beParentChildMap;
	for (std::pair<BackEdge, std::set<DFGNode *>> pair : RecCycles)
	{
		DFGNode *beParent = pair.first.first;
		DFGNode *beChild = pair.first.second;
		beParentAncestors[beParent] = removeDuplicates(getAncestoryASAP(beParent));
		beParentAncestors[beChild] = removeDuplicates(getAncestoryASAP(beChild));
		beParentChildMap[beParent] = beChild; //write somehings to order same ASAP level nodes based on the fact that they are one each others beParent ancestory
	}

	struct NodeBE
	{
		DFGNode *node;
		std::set<DFGNode *> superiors;
		bool operator<(const NodeBE &other) const
		{

			if (other.superiors.find(this->node) != other.superiors.end())
			{
				if (this->superiors.find(other.node) != this->superiors.end())
				{
					return false; // no point of swapping
				}

				return true;
			}
			return false;
		}
	};

	std::map<DFGNode *, NodeBE> NodeBEMap;

	std::map<int, std::vector<DFGNode *>> asapLevelNodeList;
	std::map<int, std::vector<NodeBE>> asapLevelNBList;
	for (DFGNode *node : in1)
	{
		asapLevelNodeList[node->ASAP].push_back(node);

		NodeBE nb;
		nb.node = node;
		for (std::pair<DFGNode *, std::vector<DFGNode *>> p : beParentAncestors)
		{
			DFGNode *beParent = p.first;
			std::vector<DFGNode *> ancestors = p.second;

			if (std::find(ancestors.begin(), ancestors.end(), node) != ancestors.end())
			{
				nb.superiors.insert(beParentChildMap[beParent]);
			}
		}
		NodeBEMap[node] = nb;
		asapLevelNBList[node->ASAP].push_back(nb);
	}
	for (DFGNode *node : in2)
	{
		asapLevelNodeList[node->ASAP].push_back(node);

		NodeBE nb;
		nb.node = node;
		for (std::pair<DFGNode *, std::vector<DFGNode *>> p : beParentAncestors)
		{
			DFGNode *beParent = p.first;
			std::vector<DFGNode *> ancestors = p.second;

			if (std::find(ancestors.begin(), ancestors.end(), node) != ancestors.end())
			{
				nb.superiors.insert(beParentChildMap[beParent]);
			}
		}
		NodeBEMap[node] = nb;
		asapLevelNBList[node->ASAP].push_back(nb);
	}

	int maxASAPlevel = 0;
	for (std::pair<int, std::vector<DFGNode *>> pair : asapLevelNodeList)
	{
		if (pair.first > maxASAPlevel)
		{
			maxASAPlevel = pair.first;
		}
	}

	for (int i = 0; i <= maxASAPlevel; ++i)
	{
		std::sort(asapLevelNBList[i].begin(), asapLevelNBList[i].end());
	}

	std::vector<DFGNode *> res;
	for (int i = 0; i <= maxASAPlevel; ++i)
	{
		for (NodeBE nb : asapLevelNBList[i])
		{
			res.push_back(nb.node);
		}
	}

	assert(res.size() == in1.size() + in2.size());
	return res;
}

std::vector<DFGNode *> DFG::mergeAncestoryALAP(const std::vector<DFGNode *> &in1,
		const std::vector<DFGNode *> &in2)
{

	std::map<int, std::vector<DFGNode *>> alapLevelNodeList;
	for (DFGNode *node : in1)
	{
		alapLevelNodeList[node->ALAP].push_back(node);
	}
	for (DFGNode *node : in2)
	{
		alapLevelNodeList[node->ALAP].push_back(node);
	}

	int maxALAPlevel = 0;
	for (std::pair<int, std::vector<DFGNode *>> pair : alapLevelNodeList)
	{
		if (pair.first > maxALAPlevel)
		{
			maxALAPlevel = pair.first;
		}
	}

	std::vector<DFGNode *> res;
	for (int i = 0; i <= maxALAPlevel; ++i)
	{
		for (DFGNode *node : alapLevelNodeList[i])
		{
			res.push_back(node);
		}
	}
	return res;
}

std::vector<DFGNode *> DFG::getAncestoryALAP(const DFGNode *node)
{
	std::stack<const DFGNode *> ancestors;
	std::set<const DFGNode *> anc_visited;

	std::queue<const DFGNode *> q;
	q.push(node);
	ancestors.push(node);

	while (!q.empty())
	{
		const DFGNode *top = q.front();
		q.pop();
		for (DFGNode *parent : top->parents)
		{
			if (parent->ALAP >= top->ALAP)
				continue; //ignore backedges
			if (anc_visited.find(parent) == anc_visited.end())
			{
				std::cout << parent->idx << ",";
			}
			ancestors.push(parent);
			anc_visited.insert(parent);
			q.push(parent);
		}
	}

	std::vector<DFGNode *> res;
	while (!ancestors.empty())
	{
		res.push_back((DFGNode *)ancestors.top());
		ancestors.pop();
	}

	return res;
}

void DFG::strongconnect(DFGNode *v,
		std::map<DFGNode *, int> &v_idx,
		std::map<DFGNode *, int> &v_lowlink,
		std::map<DFGNode *, bool> &v_onstack,
		int &idx,
		std::stack<DFGNode *> &S,
		std::vector<std::set<DFGNode *>> &result)
{

	v_idx[v] = idx;
	v_lowlink[v] = idx;

	idx++;
	S.push(v);
	v_onstack[v] = true;

	std::cout << "v=" << v->idx << ",idx=" << idx << "\n";

	for (DFGNode *child : v->children)
	{
		if (v_idx.find(child) == v_idx.end())
		{
			strongconnect(child, v_idx, v_lowlink, v_onstack, idx, S, result);
			v_lowlink[v] = std::min(v_lowlink[v], v_lowlink[child]);
		}
		else if (v_onstack[child])
		{
			v_lowlink[v] = std::min(v_lowlink[v], v_idx[child]);
		}
	}

	std::cout << "v=" << v->idx << ",idx=" << idx << ",v_lowlink[v]=" << v_lowlink[v] << "\n";

	if (v_lowlink[v] == v_idx[v])
	{
		std::cout << "found SCC::\n";
		std::set<DFGNode *> scc;
		DFGNode *w;
		do
		{
			w = S.top();
			S.pop();
			scc.insert(w);
			std::cout << w->idx << ",";
			v_onstack[w] = false;
		} while (w != v);
		std::cout << "\n";

		std::cout << "res.size = " << result.size() << "\n";
		result.push_back(scc);
		std::cout << "res.size = " << result.size() << "\n";
	}
}

std::vector<DFGNode *> DFG::getAncestoryASAP(const DFGNode *node)
{
	std::stack<const DFGNode *> ancestors;

	std::queue<const DFGNode *> q;
	q.push(node);
	ancestors.push(node);

	while (!q.empty())
	{
		const DFGNode *top = q.front();
		q.pop();
		for (DFGNode *parent : top->parents)
		{
			if (parent->ASAP >= top->ASAP)
				continue; //ignore backedges
			if (parent->childNextIter[(DFGNode *)top] == 1)
				continue; //ignore backedges
			//			std::cout << parent->idx << ",";
			ancestors.push(parent);
			q.push(parent);
		}
	}

	std::vector<DFGNode *> res;
	while (!ancestors.empty())
	{
		res.push_back((DFGNode *)ancestors.top());
		ancestors.pop();
	}

	return res;
}

bool DFG::getAncestoryASAPUntil(DFGNode *beParent, DFGNode *beChild,
		std::set<DFGNode *> &result)
{

	std::map<int, std::set<DFGNode *>> asapOrderSet;
	std::queue<DFGNode *> q;
	q.push(beParent);

	//	std::vector<DFGNode*> ancestory = getAncestoryASAP(beParent);
	//	if(std::find(ancestory.begin(),ancestory.end(),beChild) == ancestory.end()) return false;

	while (!q.empty())
	{
		DFGNode *node = q.front();
		q.pop();
		for (DFGNode *parent : node->parents)
		{
			if (parent->ASAP >= node->ASAP)
				continue; //ignore backedges
			if (parent->childNextIter[node] == 1)
				continue; //ignore backedges

			if (parent->ASAP <= beChild->ASAP)
				continue;
			asapOrderSet[parent->ASAP].insert(parent);
			q.push(parent);
		}
	}

	for (std::pair<int, std::set<DFGNode *>> pair : asapOrderSet)
	{
		for (DFGNode *node : pair.second)
		{
			result.insert(node);
		}
	}

	return true;
}

void DFG::printDOT(std::string fileName) {
	std::ofstream ofs;
	ofs.open(fileName.c_str());

	//Write the initial info
	ofs << "digraph Region_18 {\n";
	ofs << "\tgraph [ nslimit = \"1000.0\",\n";
	ofs <<	"\torientation = landscape,\n";
	ofs <<	"\t\tcenter = true,\n";
	ofs <<	"\tpage = \"8.5,11\",\n";
	ofs << "\tcompound=true,\n";
	ofs <<	"\tsize = \"10,7.5\" ] ;" << std::endl;

	assert(nodeList.size() != 0);

		for(DFGNode &node : nodeList){
			ofs << "\""; //BEGIN NODE NAME
			ofs << "Op_" << node.idx;
			ofs << "\" [ fontname = \"Helvetica\" shape = box, ";
			ofs << "color = ";

						if(node.in_rec_cycle){
							ofs << "red";
						}
						else{
							ofs << "black";
						}
						ofs << ", ";
			ofs << "label = \" ";
//			if(node->getNode() != NULL){
				ofs << node.op;
//				ofs << " " << node->getNode()->getName().str() << " ";
//
//				if(node->hasConstantVal()){
//					ofs << " C=" << "0x" << std::hex << node->getConstantVal() << std::dec;
//				}
//
//				if(node->isGEP()){
//					ofs << " C=" << "0x" << std::hex << node->getGEPbaseAddr() << std::dec;
//				}

//			}
//			else{
//				ofs << node->getNameType();
//
//				if(node->getNameType() == "OuterLoopLOAD"){
//					ofs << " " << OutLoopNodeMapReverse[node]->getName().str() << " ";
//				}
//
//				if(node->isOutLoop()){
//					ofs << " C=" << "0x" << node->getoutloopAddr() << std::dec;
//				}
//
//				if(node->hasConstantVal()){
//					ofs << " C=" << "0x" << node->getConstantVal() << std::dec;
//				}
//			}

//			ofs << "BB=" << node->BB->getName().str();

//			if(!mutexNodes[node].empty()){
//				ofs << ",mutex={";
//				for(dfgNode* m : mutexNodes[node]){
//					ofs << m->getIdx() << ",";
//				}
//				ofs << "}";
//			}

//			if(node->getFinalIns() != NOP){
//				ofs << " HyIns=" << HyCUBEInsStrings[node->getFinalIns()];
//			}
			ofs << ",\n";
			ofs << node.idx <<", ASAP=" << node.ASAP;
			ofs << ", ALAP=" << node.ALAP;

			ofs << "\"]\n"; //END NODE NAME
		}

	//The EDGES

	for(DFGNode &node : nodeList){

		for(DFGNode* child : node.children){
			bool isCondtional=false;
			bool condition=true;

//			isCondtional= node->childConditionalMap[child] != UNCOND;
//			condition = (node->childConditionalMap[child] == TRUE);

//		bool isBackEdge = node->childBackEdgeMap[child];


			ofs << "\"" << "Op_" << node.idx << "\"";
			ofs << " -> ";
			ofs << "\"" << "Op_" << child->idx << "\"";
			ofs << " [style = ";

//			if(isBackEdge){
//				ofs << "dashed";
//			}
//			else{
				ofs << "bold";
//			}
			ofs << ", ";

			ofs << "color = ";
//			if(isCondtional){
//				if(findEdge(node,child)->getType() == EDGE_TYPE_PS){
//					ofs << "cyan";
//				}
//				else if(condition==true){
//					ofs << "blue";
//				}
//				else{
//					ofs << "red";
//				}
//			}
//			else{
				ofs << "black";

//			}

//			ofs << ", headport=n, tailport=s";
			ofs << "];\n";
		}

//		for(dfgNode* recChild : node->getRecChildren()){
//			ofs << "\"" << "Op_" << node->getIdx() << "\"";
//			ofs << " -> ";
//			ofs << "\"" << "Op_" << recChild->getIdx() << "\"";
//			ofs << "[style = bold, color = green];\n";
//		}
	}

	ofs << "}" << std::endl;
	ofs.close();
}

} /* namespace CGRAXMLCompile */

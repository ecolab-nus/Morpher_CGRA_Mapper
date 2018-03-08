/*
 * DataPath.h
 *
 *  Created on: 26 Feb 2018
 *      Author: manupa
 */

#ifndef DATAPATH_H_
#define DATAPATH_H_
#include "Module.h"
#include "DFGNode.h"

namespace CGRAXMLCompile {

class FU;
class PE;
class CGRA;

class DataPath : public Module {
public:
	DataPath(const Module* Parent, std::string name) : Module(Parent,name){
		//create inputPorts
		inputPorts.push_back(Port("P",IN,this));
		inputPorts.push_back(Port("I1",IN,this));
		inputPorts.push_back(Port("I2",IN,this));

		//create outputPorts
		outputPorts.push_back(Port("T",OUT,this));

		//create output to input connection
		connections[getOutPort("T")].push_back(getInPort("P"));
		connections[getOutPort("T")].push_back(getInPort("I1"));
		connections[getOutPort("T")].push_back(getInPort("I2"));

		mappedNode=NULL;
		outputDP=NULL;
	}

	FU* getFU();
	PE* getPE();
	CGRA* getCGRA();
	Port* getOutputPort(int latency);

	void assignNode(DFGNode* node);
	DFGNode* getMappedNode(){return mappedNode;}
	DataPath* getOutputDP(){return outputDP;}
	void clear();


private:
	DFGNode* mappedNode;
	DataPath* outputDP;


};

} /* namespace CGRAXMLCompile */

#endif /* DATAPATH_H_ */

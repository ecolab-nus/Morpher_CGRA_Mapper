/*
 * cgra.h
 *
 *  Created on: 20 Feb 2018
 *      Author: manupa
 */

#ifndef CGRA_H_
#define CGRA_H_

#include "Module.h"
#include "PE.h"
#include "FU.h"
#include "DataPath.h"

namespace CGRAXMLCompile
{

class DFG;

class CGRA : public Module
{
public:
	CGRA(const Module *Parent, std::string name, int t, int y, int x, DFG *dfg, std::string peType = "GENERIC_8REGF", int numberofDPs = 1, std::map<Port *, std::set<DFGNode *>> *_congestedPortPtr = NULL) : Module(Parent, name)
	{
		createGenericCGRA(x, y, t, peType, numberofDPs);
		this->peType = peType;
		this->numberofDPs = numberofDPs;
		this->congestedPortPtr = _congestedPortPtr;

		for (int t = 0; t < this->get_t_max(); ++t)
		{
			for (int y = 0; y < this->get_y_max(); ++y)
			{
				for (int x = 0; x < this->get_x_max(); ++x)
				{
					PE *pe = this->PEArr[t][y][x];
					for (Module *submod : pe->subModules)
					{
						if (FU *fu = dynamic_cast<FU *>(submod))
						{
							if (fu->supportedOPs.find("LOAD") != fu->supportedOPs.end())
							{
								for (Module *submod2 : fu->subModules)
								{
									if (DataPath *dp = dynamic_cast<DataPath *>(submod2))
									{
										if (dp->getMappedNode() == NULL)
										{
											freeMemNodes++;
											freeMemNodeSet.insert(dp);
										}
									}
								}
							}
						}
					}
				}
			}
		}
		currDFG = dfg;
	}
	void createGenericCGRA(int x, int y, int t, std::string peType = "GENERIC_8REGF", int numberofDPs = 1);

	std::map<int, std::map<int, std::map<int, PE *>>> PEArr;
	void adjustII(int newII);
	int get_t_max() { return t_max; }
	int get_y_max() { return y_max; }
	int get_x_max() { return x_max; }

	std::map<Port *, std::set<DFGNode *>> *getCongestedPortPtr() { return congestedPortPtr; }
	void setCongestedPortPtr(std::map<Port *, std::set<DFGNode *>> *congestedPortPtr) { this->congestedPortPtr = congestedPortPtr; }

	//	bool isGenericPE=false;
	std::string peType;
	int numberofDPs = 1;
	int freeMemNodes = 0;
	std::set<DataPath *> freeMemNodeSet;

	std::set<Port *> getConflictPorts(Port *p);
	void insertConflictPort(Port *a, Port *b);
	bool isConflictPortsEmpty(Port *p) { return conflictPorts[p].empty(); }
	DFG *currDFG = NULL;

	int minLatBetweenPEs = 1;

private:
	int x_max;
	int y_max;
	int t_max;
	std::map<Port *, std::set<Port *>> conflictPorts;
	std::map<Port *, std::set<DFGNode *>> *congestedPortPtr;
};

} /* namespace CGRAXMLCompile */

#endif /* CGRA_H_ */

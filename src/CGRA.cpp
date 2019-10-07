/*
 * cgra.cpp
 *
 *  Created on: 20 Feb 2018
 *      Author: manupa
 */

#include "CGRA.h"
#include "PE.h"
#include "Port.h"
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <iomanip>

using json = nlohmann::json;
using namespace CGRAXMLCompile;

namespace CGRAXMLCompile
{

//CGRA::CGRA() {
//	// TODO Auto-generated constructor stub
//
//}

} /* namespace CGRAXMLCompile */

void CGRAXMLCompile::CGRA::createGenericCGRA(int x_max, int y_max, int t_max, std::string peType, int numberofDPs)
{

	this->x_max = x_max;
	this->y_max = y_max;
	this->t_max = t_max;

	this->minLatBetweenPEs = 1;
	if (peType.find("HyCUBE") != std::string::npos || peType.find("HYCUBE") != std::string::npos)
	{
		this->minLatBetweenPEs = 0;
	}

	for (int t = 0; t < t_max; ++t)
	{
		for (int y = 0; y < y_max; ++y)
		{
			for (int x = 0; x < x_max; ++x)
			{
				PE *newPE;
				std::string PE_name = "PE_" + std::to_string(t) + "," + std::to_string(y) + "," + std::to_string(x);

				//				bool isCorner = (x==0)&&(y==0);
				//				isCorner = isCorner | ((x==0)&&(y==y_max-1));
				//				isCorner = isCorner | ((x==x_max-1)&&(y==0));
				//				isCorner = isCorner | ((x==x_max-1)&&(y==y_max-1));

				if (x == 0)
				{
					newPE = new PE(this, PE_name, x, y, t, peType, true, numberofDPs);
				}
				else
				{
					newPE = new PE(this, PE_name, x, y, t, peType, false, numberofDPs);
				}
				subModules.push_back(newPE);
				PEArr[t][y][x] = newPE;
			}
		}
	}

	//create connections
	for (int t = 0; t < t_max; ++t)
	{
		for (int y = 0; y < y_max; ++y)
		{
			for (int x = 0; x < x_max; ++x)
			{

				PE *currPE = PEArr[t][y][x];
				if (x - 1 >= 0)
				{
					Port *currPE_WestO = currPE->getOutPort("WEST_O");
					PE *westPE = PEArr[t][y][x - 1];
					Port *westPE_EastI = westPE->getInPort("EAST_I");
					//					connectedTo[currPE_WestO].push_back(westPE_EastI);
					insertConnection(currPE_WestO, westPE_EastI);
				}

				if (x + 1 < x_max)
				{
					Port *currPE_EastO = currPE->getOutPort("EAST_O");
					PE *eastPE = PEArr[t][y][x + 1];
					Port *eastPE_WestI = eastPE->getInPort("WEST_I");
					//					connectedTo[currPE_EastO].push_back(eastPE_WestI);
					insertConnection(currPE_EastO, eastPE_WestI);
				}

				if (y - 1 >= 0)
				{
					Port *currPE_NorthO = currPE->getOutPort("NORTH_O");
					PE *northPE = PEArr[t][y - 1][x];
					Port *northPE_SouthI = northPE->getInPort("SOUTH_I");
					//					connectedTo[currPE_NorthO].push_back(northPE_SouthI);
					insertConnection(currPE_NorthO, northPE_SouthI);
				}

				if (y + 1 < y_max)
				{
					Port *currPE_SouthO = currPE->getOutPort("SOUTH_O");
					PE *southPE = PEArr[t][y + 1][x];
					Port *southPE_NorthI = southPE->getInPort("NORTH_I");
					//					connectedTo[currPE_SouthO].push_back(southPE_NorthI);
					insertConnection(currPE_SouthO, southPE_NorthI);
				}

				int t_next = (t + 1) % t_max;
				PE *nextCyclePE = PEArr[t_next][y][x];
				for (RegFile *RF : currPE->allRegs)
				{
					for (int i = 0; i < RF->get_nRegs(); ++i)
					{
						Port *reg_i = nextCyclePE->getInPort(RF->getName() + "_REG_I" + std::to_string(i));
						Port *reg_o = currPE->getOutPort(RF->getName() + "_REG_O" + std::to_string(i));

						//						connectedTo[reg_o].push_back(reg_i);
						insertConnection(reg_o, reg_i);
						regCons[std::make_pair(reg_o, reg_i)] = true;
					}
				}

				for (std::pair<Port *, Port *> curr_portpair : currPE->getRegConPorts())
				{
					std::pair<Port *, Port *> next_portpair = nextCyclePE->getRegConPort(curr_portpair.first->getName());
					Module *mod = curr_portpair.first->getMod();
					mod->insertConnection(curr_portpair.second, next_portpair.first);
					mod->regCons[std::make_pair(curr_portpair.second, next_portpair.first)] = true;
				}

				//Create Input Registers linked to next time instance : N2N_FIX
				//				if(peType == "N2N_4REGF"){
				//					for(Module* subMod : currPE->subModules){
				//						if(FU* fu = dynamic_cast<FU*>(subMod)){
				//							for(Port *ip : fu->inputPorts){
				//								std::string regRI_name = fu->getName() + "_" + ip->getName() + "RI";
				//								std::string regRO_name = fu->getName() + "_" + ip->getName() + "RO";
				//
				//								Port* ri = nextCyclePE->getInPort(regRI_name); assert(ri);
				//								Port* ro = currPE->getOutPort(regRO_name); assert(ro);
				//
				//								insertConnection(ro,ri);
				//
				//							}
				//						}
				//					}
				//				}
			}
		}
	}

	// for (int t = 0; t < t_max; ++t)
	// {

	// }
}

std::set<CGRAXMLCompile::Port *> CGRAXMLCompile::CGRA::getConflictPorts(Port *p)
{
	if (conflictPorts.find(p) == conflictPorts.end())
	{
		std::set<Port *> emptyVec;
		conflictPorts[p] = emptyVec;
	}
	//	std::cout << "size = " << conflictPorts[p].size() << "\n";

	for (Port *tp : conflictPorts[p])
	{
		assert(tp != NULL);
		//		std::cout << tp->getName() << "\n";
		//		std::cout << tp->getFullName() << "\n";
	}

	return conflictPorts[p];
}

void CGRAXMLCompile::CGRA::insertConflictPort(Port *a, Port *b)
{
	assert(a != NULL);
	assert(b != NULL);
	std::cout << "insertConflict Port b : " << b->getFullName();
	std::cout << ", a : " << a->getFullName() << "\n";
	conflictPorts[a].insert(b);
}

void CGRAXMLCompile::CGRA::ParseCGRA(json &cgra_desc, int II)
{

	//NOTE you can only have sub modules at the CGRA level.
	//You can always add functionality in terms of PEs.

	for (int t = 0; t < II; t++)
	{

		for (auto &el : cgra_desc["SUBMODS"].items())
		{

			//if not at CGRA level, a single instance suffice
			string type = el.key();
			for (auto &sm : el.value())
			{
				string name = sm["name"];
				name = name + "-T" + to_string(t);

				Module *submod;
				if (sm.find("X") != sm.end() && sm.find("Y") != sm.end())
				{
					int y = sm["Y"];
					int x = sm["X"];
					submod = ParseModule(top_desc["ARCH"][type], this, name, type, t, y, x);
				}
				else
				{
					submod = ParseModule(top_desc["ARCH"][type], this, name, type, t);
				}
				// Module *submod = ParseModule(top_desc["ARCH"][type], this, name, type, t);
				assert(submod);

				// cout << "submod name = " << name << "\n";
				// cout << "submod real name = " << submod->getFullName() << "\n";

				Name2SubMod[name] = submod;
				subModArr[t].push_back(submod);
			}
		}

		for (auto &el : cgra_desc["CONNECTIONS"].items())
		{
			string src = el.key();
			string src_mod_str = src.substr(0, src.find("."));
			string src_port_str = src.erase(0, src.find(".") + 1);

			Module *src_mod;
			if (src_mod_str == "THIS")
			{
				src_mod = this;
			}
			else
			{
				src_mod_str = src_mod_str + "-T" + to_string(t);
				src_mod = Name2SubMod[src_mod_str];
			}
			assert(src_mod);

			for (string dest : el.value())
			{
				string dest_mod_str = dest.substr(0, dest.find("."));
				string dest_port_str = dest.erase(0, dest.find(".") + 1);

				Module *dest_mod;
				if (dest_mod_str == "THIS")
				{
					dest_mod = this;
				}
				else
				{
					dest_mod_str = dest_mod_str + "-T" + to_string(t);
					dest_mod = Name2SubMod[dest_mod_str];
				}
				assert(dest_mod);

				Port *src_p;
				// cout << "src_mod_str = " << src_mod_str << "\n";
				// cout << "src_port_str = " << src_port_str << "\n";
				// cout << "src_mod->Name2RegPort.empty()  = " << src_mod->Name2RegPort.empty() << "\n";
				// cout << "src_mod real = " << src_mod->getName() << "\n";
				if (!src_mod->Name2RegPort.empty() && src_mod->Name2RegPort.find(src_port_str) != src_mod->Name2RegPort.end())
				{
					//in a regport second is used for outgoing connections
					src_p = src_mod->Name2RegPort[src_port_str].second;
				}
				else
				{
					src_p = src_mod->Name2Port[src_port_str];
				}
				assert(src_p);

				Port *dest_p;
				// cout << "dest_port_str = " << dest_port_str << "\n";
				if (!dest_mod->Name2RegPort.empty() && dest_mod->Name2RegPort.find(dest_port_str) != dest_mod->Name2RegPort.end())
				{
					//in a regport first is used for incoming connections
					dest_p = dest_mod->Name2RegPort[dest_port_str].first;
				}
				else
				{
					dest_p = dest_mod->Name2Port[dest_port_str];
				}
				assert(dest_p);
				this->insertConnection(src_p, dest_p);
			}
		}
	}

	for (int t = 0; t < II; t++)
	{
		for (int i = 0; i < subModArr[t].size(); i++)
		{
			PE *currPE = static_cast<PE *>(subModArr[t][i]);
			int next_t = (t + 1) % II;
			PE *nextCyclePE = static_cast<PE *>(subModArr[next_t][i]);

			NextCyclePEMap[currPE] = nextCyclePE;

			string curr_pe_name_prefix = currPE->getName().substr(0, currPE->getName().find("-"));
			string next_pe_name_prefix = nextCyclePE->getName().substr(0, nextCyclePE->getName().find("-"));
			assert(curr_pe_name_prefix == next_pe_name_prefix);

			for (std::pair<Port *, Port *> curr_portpair : currPE->getRegConPorts())
			{
				std::pair<Port *, Port *> next_portpair = nextCyclePE->getRegConPort(curr_portpair.first->getName());
				Module *mod = curr_portpair.first->getMod();
				mod->insertConnection(curr_portpair.second, next_portpair.first);
				mod->regCons[std::make_pair(curr_portpair.second, next_portpair.first)] = true;
			}
		}
	}
}

Module *CGRAXMLCompile::CGRA::ParseModule(json &module_desc, Module *parent, string module_name, string type, int t, int x, int y)
{
	// module_name = module_name + "-T" + to_string(t);

	assert(!top_desc.empty());
	Module *ret;

	if (type == "DP")
	{
		ret = new DataPath(parent, module_name, t);
		ret->Name2Port["I1"] = ret->getInPort("I1");
		ret->Name2Port["I2"] = ret->getInPort("I2");
		ret->Name2Port["P"] = ret->getInPort("P");
		ret->Name2Port["T"] = ret->getOutPort("T");
		//Datapath has a fixed design for now, so no need to modify just return

		FU* ret_fu = ret->getFU();
		if(ret_fu->isMEMFU()){
			freeMemNodes++;
			freeMemNodeSet.insert((DataPath*)ret);
			PE* ret_pe = ret->getPE();
			ret_pe->isMemPE = true;
		}
		return ret;
	}
	else if (type == "FU_MEM" || type == "FU")
	{
		ret = new FU(parent, module_name, t);
	}
	else if (CGRA *cgra = dynamic_cast<CGRA *>(parent))
	{
		//if parent is CGRA then child should be a PE.
		assert(type.find("PE") != string::npos);
		assert(x != -1);
		assert(y != -1);
		ret = new PE(parent, module_name, t, y, x);
	}
	else if (type == "CGRA")
	{
		ret = this;
	}
	else
	{
		// cout << "No special module for type = " << type << ", implementing geneic module.\n";
		ret = new Module(parent, module_name, t);
	}

	for (auto &p : module_desc["INPUTS"])
	{
		Port *port_ptr = new Port(p, IN, ret);
		ret->Name2Port[p] = port_ptr;
		ret->inputPorts.push_back(port_ptr);
	}

	for (auto &p : module_desc["OUTPUTS"])
	{
		Port *port_ptr = new Port(p, OUT, ret);
		// cout << "output port = " << p << "\n";
		ret->Name2Port[p] = port_ptr;
		ret->outputPorts.push_back(port_ptr);
	}

	for (auto &p : module_desc["INTERNALS"])
	{
		Port *port_ptr = new Port(p, INT, ret);
		ret->Name2Port[p] = port_ptr;
		ret->internalPorts.push_back(port_ptr);
	}

	for (auto &p : module_desc["REGS"])
	{
		assert(ret->getPE());
		ret->insertRegPort(p);
		ret->Name2RegPort[p] = ret->getRegPort(p);
	}

	for (auto &el : module_desc["OPS"].items())
	{
		//OPS should only be present in FUs
		FU *ret_fu = static_cast<FU *>(ret);
		string OP = el.key();
		int lat = el.value();
		ret_fu->supportedOPs[OP] = lat;
		insertGlobalOP(OP, lat);
	}

	for (auto &el : module_desc["SUBMODS"].items())
	{

		//if not at CGRA level, a single instance suffice
		string type = el.key();

		for (auto &sm : el.value())
		{
			string name = sm["name"];
			Module *submod;
			if (sm.find("X") != sm.end() && sm.find("Y") != sm.end())
			{
				int y = sm["Y"];
				int x = sm["X"];
				submod = ParseModule(top_desc["ARCH"][type], ret, name, type, t, y, x);
			}
			else
			{
				submod = ParseModule(top_desc["ARCH"][type], ret, name, type, t);
			}

			ret->Name2SubMod[name] = submod;
			ret->subModules.push_back(submod);
		}

		// for (string name : el.value())
		// {
		// 	Module *submod = ParseModule(top_desc["ARCH"][type], ret, name, type, t);
		// 	assert(submod);

		// 	// cout << "submod name = " << name << "\n";
		// 	// cout << "submod real name = " << submod->getFullName() << "\n";

		// 	ret->Name2SubMod[name] = submod;
		// 	ret->subModules.push_back(submod);
		// }
	}

	for (auto &el : module_desc["CONNECTIONS"].items())
	{
		string src = el.key();
		string src_mod_str = src.substr(0, src.find("."));
		string src_port_str = src.erase(0, src.find(".") + 1);

		Module *src_mod;
		if (src_mod_str == "THIS")
		{
			src_mod = ret;
		}
		else
		{
			src_mod = ret->Name2SubMod[src_mod_str];
		}
		assert(src_mod);

		for (string dest : el.value())
		{
			string dest_mod_str = dest.substr(0, dest.find("."));
			string dest_port_str = dest.erase(0, dest.find(".") + 1);

			Module *dest_mod;
			if (dest_mod_str == "THIS")
			{
				dest_mod = ret;
			}
			else
			{
				dest_mod = ret->Name2SubMod[dest_mod_str];
			}
			assert(dest_mod);

			Port *src_p;
			// cout << "src_mod_str = " << src_mod_str << "\n";
			// cout << "src_port_str = " << src_port_str << "\n";
			// cout << "src_mod->Name2RegPort.empty()  = " << src_mod->Name2RegPort.empty() << "\n";
			// cout << "src_mod real = " << src_mod->getName() << "\n";
			if (!src_mod->Name2RegPort.empty() && src_mod->Name2RegPort.find(src_port_str) != src_mod->Name2RegPort.end())
			{
				//in a regport second is used for outgoing connections
				src_p = src_mod->Name2RegPort[src_port_str].second;
			}
			else
			{
				src_p = src_mod->Name2Port[src_port_str];
			}
			assert(src_p);

			Port *dest_p;
			// cout << "dest_port_str = " << dest_port_str << "\n";
			if (!dest_mod->Name2RegPort.empty() && dest_mod->Name2RegPort.find(dest_port_str) != dest_mod->Name2RegPort.end())
			{
				//in a regport first is used for incoming connections
				dest_p = dest_mod->Name2RegPort[dest_port_str].first;
			}
			else
			{
				dest_p = dest_mod->Name2Port[dest_port_str];
			}
			assert(dest_p);
			ret->insertConnection(src_p, dest_p);
		}
	}

	if (FU *ret_fu = dynamic_cast<FU *>(ret))
	{
		assert(!ret_fu->supportedOPs.empty());
	}

	return ret;
}

bool CGRAXMLCompile::CGRA::ParseJSONArch(string fileName, int II)
{
	this->t_max = II;

	ifstream json_file(fileName.c_str());
	assert(json_file.is_open());
	json json;
	json_file >> json;

	// cout << json;

	PreprocessPattern(json["ARCH"]["CGRA"]);

	top_desc = json;

	cout << "TOP_ARCH :: begin %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";
	std::cout << setw(4) << top_desc << std::endl;
	cout << "TOP_ARCH :: end %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";

	//just creating CGRA instance for t
	// ParseModule(json["ARCH"]["CGRA"], NULL, "CGRA_Ins", "CGRA", 0);

	//TODO : remove hardcoding of II=2
	ParseCGRA(json["ARCH"]["CGRA"], II);
	cout << "Parsing JSON Success!!\n";
	// exit(EXIT_SUCCESS);

	return true;
}

bool CGRAXMLCompile::CGRA::PreprocessPattern(json &top)
{

	json submods;
	json connections;

	cout << "Before JSON Begin :: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";
	std::cout << setw(4) << top << std::endl;
	cout << "Before JSON End :: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";

	assert(top["SUBMODS"][0]["PATTERN"] == "GRID");

	int xdim = top["SUBMODS"][0]["DIMS"]["X"];
	int ydim = top["SUBMODS"][0]["DIMS"]["Y"];

	unordered_map<int, unordered_map<int, pair<string, string>>> submod_grid;

	for (auto &el : top["SUBMODS"][0]["MODS"])
	{
		int x = el["X"];
		int y = el["Y"];
		string mod_type = el["MOD"];
		string mod_name = mod_type + "_X" + to_string(x) + "|" + "_Y" + to_string(y) + "|";

		json mod;
		mod["name"] = mod_name;
		mod["X"] = x;
		mod["Y"] = y;

		submods[mod_type].push_back(mod);
		submod_grid[x][y] = make_pair(mod_type, mod_name);
	}

	for (auto &el : top["SUBMODS"][0]["CONNECTIONS"])
	{
		string fromx_str = el["FROM_X"];
		int fromx = atoi(fromx_str.substr(1, fromx_str.size() - 1).c_str());
		if (fromx_str.size() == 1)
			fromx = 0;

		string fromy_str = el["FROM_Y"];
		int fromy = atoi(fromy_str.substr(1, fromy_str.size() - 1).c_str());
		if (fromy_str.size() == 1)
			fromy = 0;

		string curr_port_str = el["FROM_PORT"];

		string tox_str = el["TO_X"];
		int tox = atoi(tox_str.substr(1, tox_str.size() - 1).c_str());
		if (tox_str.size() == 1)
			tox = 0;

		string toy_str = el["TO_Y"];
		int toy = atoi(toy_str.substr(1, toy_str.size() - 1).c_str());
		if (toy_str.size() == 1)
			toy = 0;

		string to_port_str = el["TO_PORT"];

		cout << "fromx = " << fromx << ",";
		cout << "fromy = " << fromy << ",";
		cout << "tox = " << tox << ",";
		cout << "toy = " << toy << "\n";

		for (int y = 0; y < ydim; y++)
		{
			for (int x = 0; x < xdim; x++)
			{
				int src_x = x + fromx;
				int src_y = y + fromy;
				int dest_x = x + tox;
				int dest_y = y + toy;

				if ((src_x >= 0) && (src_x < xdim) &&
					(src_y >= 0) && (src_y < ydim) &&
					(dest_x >= 0) && (dest_x < xdim) &&
					(dest_y >= 0) && (dest_y < ydim))
				{
					string conn_from_str = submod_grid[src_x][src_y].second + "." + curr_port_str;
					string conn_next_str = submod_grid[dest_x][dest_y].second + "." + to_port_str;
					connections[conn_from_str].push_back(conn_next_str);
				}
			}
		}
	}

	top.erase("SUBMODS");
	top["SUBMODS"] = submods;
	top["CONNECTIONS"] = connections;

	cout << "After JSON Begin :: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";
	std::cout << setw(4) << top << std::endl;
	cout << "After JSON End :: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";
}

unordered_set<PE *> CGRAXMLCompile::CGRA::getAllPEList()
{

	unordered_set<PE *> ret;

	if (!subModArr.empty())
	{
		for (auto it = subModArr.begin(); it != subModArr.end(); it++)
		{
			for (Module *m : it->second)
			{
				if (PE *pe = dynamic_cast<PE *>(m))
				{
					ret.insert(pe);
				}
			}
		}
	}
	else
	{
		assert(!PEArr.empty());
		for (int t = 0; t < this->get_t_max(); ++t)
		{
			for (int y = 0; y < this->y_max; ++y)
			{
				for (int x = 0; x < this->x_max; ++x)
				{
					ret.insert(PEArr[t][y][x]);
				}
			}
		}
	}

	return ret;
}

PE *CGRAXMLCompile::CGRA::getPE(int t, int y, int x)
{

	assert(!PEArr.empty());
	return PEArr[t][y][x];
}

PE *CGRAXMLCompile::CGRA::getLatencyPE(PE *currPE, int lat)
{
	if (!subModArr.empty())
	{
		PE *ret = currPE;
		while (lat > 0)
		{
			ret = NextCyclePEMap[ret];
			lat--;
		}
		return ret;
	}
	else
	{
		assert(!PEArr.empty());
		int curr_x = currPE->X;
		int curr_y = currPE->Y;
		int curr_t = currPE->T;

		int next_t = (curr_t + 1) % get_t_max();
		return PEArr[next_t][curr_y][curr_x];
	}
}

vector<PE *> CGRAXMLCompile::CGRA::getSpatialPEList(int t)
{
	vector<PE *> ret;

	if (!subModArr.empty())
	{
		auto it = subModArr[t];
		for (Module *m : it)
		{
			if (PE *pe = dynamic_cast<PE *>(m))
			{
				ret.push_back(pe);
			}
		}
	}
	else
	{
		assert(!PEArr.empty());
		for (int y = 0; y < this->y_max; ++y)
		{
			for (int x = 0; x < this->x_max; ++x)
			{
				ret.push_back(PEArr[t][y][x]);
			}
		}
	}

	return ret;
}

void CGRAXMLCompile::CGRA::insertGlobalOP(string OP, int lat)
{
	if (GlobalOPMinLatencyMap.find(OP) != GlobalOPMinLatencyMap.end())
	{
		if (lat != GlobalOPMinLatencyMap[OP])
		{
			cout << "op = " << OP << ", old lat = " << GlobalOPMinLatencyMap[OP] << ", new lat = " << lat << "\n";
			if (lat < GlobalOPMinLatencyMap[OP])
			{
				GlobalOPMinLatencyMap[OP] = lat;
			}
		}
		// assert(lat == GlobalOPMinLatencyMap[OP]);
	}
	else
	{
		GlobalOPMinLatencyMap[OP] = lat;
	}
}

string CGRAXMLCompile::CGRA::getCGRAName()
{

	if (subModArr.empty())
	{
		//old arch desc
		return peType + "_DP" + std::to_string(numberofDPs) + "_XDim=" + std::to_string(x_max) + "_YDim=" + std::to_string(y_max) + "_II=" + std::to_string(t_max);
	}
	else
	{
		std::stringstream ss(json_file);
		std::string token;
		vector<string> cont;
		while (std::getline(ss, token, '/')) {
			cont.push_back(token);
		}
		return *cont.rbegin();
	}
}

void CGRAXMLCompile::CGRA::analyzeTimeDist(){

	for(Module* m1 : subModArr[0]){
		PE* pe1 = static_cast<PE*>(m1);
		TimeDistBetweenClosestMEMPEMap[pe1] = INT32_MAX;
		for(Module* m2 : subModArr[0]){
			PE* pe2 = static_cast<PE*>(m2);
			if(pe1 == pe2) continue;
			TimeDistBetweenPEMap[pe1][pe2] = getTimeDistBetweenPEs(pe1,pe2);
		}
	}

	if(get_t_max() > 1){
		for(Module* m1 : subModArr[0]){
			PE* zeroth_pe = static_cast<PE*>(m1);
			PE* currPE = NextCyclePEMap[zeroth_pe];
			for(int i=1; i<get_t_max(); i++){
				MapTzeroPE[currPE]=zeroth_pe;
				currPE = NextCyclePEMap[currPE];
			}
		}
	}

	cout << "Timing analysis between PEs done.!\n";
	for(auto it1 = TimeDistBetweenPEMap.begin(); it1 != TimeDistBetweenPEMap.end(); it1++){
		PE* pe1 = it1->first;
		for(auto it2 = TimeDistBetweenPEMap[pe1].begin(); it2 != TimeDistBetweenPEMap[pe1].end(); it2++){
			PE* pe2 = it2->first;
			int time_dist = it2->second;
			cout << "PE1=" << pe1->getFullName() << "\tPE2=" << pe2->getFullName() << "\ttime_dist=" << time_dist << "\n";
		}
	}

	cout << "Timing analysis between PE and closest memPE :: \n";
	for(auto it = TimeDistBetweenClosestMEMPEMap.begin(); it != TimeDistBetweenClosestMEMPEMap.end(); it++){
		PE* pe = it->first;
		int time_dist = it->second;
		cout << "PE=" << pe->getFullName() << "\ttime_dist=" << time_dist << "\n";
	}

	IntraPETimeDistAnalysisDone = true;

}


void CGRAXMLCompile::CGRA::traverseUntil(PE* srcPE, PE* destPE, Port* currPort, int time_dist, unordered_map<Port*,int>& already_traversed, int& result){
	// cout << "srcPE=" << srcPE->getName() << ",destPE=" << destPE->getName() << "\tcurrPort=" << currPort->getFullName() << ",time_dist=" << time_dist << "\n";
	if(already_traversed.find(currPort) != already_traversed.end()){
		if(time_dist < already_traversed[currPort]){
			already_traversed[currPort] = time_dist;
		}
		else{
			return;
		}
	}
	else{
		already_traversed[currPort] = time_dist;
	}

	PE* currPE = currPort->getMod()->getPE();
	FU* currFU = currPort->getMod()->getFU();

	if(currFU != NULL && currFU->isMEMFU() ){
		if(time_dist < TimeDistBetweenClosestMEMPEMap[srcPE]){
			    // cout << "fu = " << currFU->getFullName() << ",";
				// cout << "srcPE=" << srcPE->getName() << ",destPE=" << destPE->getName() << "\tcurrPort=" << currPort->getFullName() << ",time_dist=" << time_dist << "\n";
			TimeDistBetweenClosestMEMPEMap[srcPE] = time_dist;
		}
	}

	if(currFU != NULL && currPE == destPE){
		result = time_dist;
		return;
	}

	vector<Port*> nextPorts = currPort->getMod()->getNextPorts(currPort);
	for(Port* p : nextPorts){
		int time_delta = 0;
		if(p->getMod()->get_t() != currPort->getMod()->get_t()){
			time_delta = 1;
		}
		if(currPort->getType() == REGI && p->getType() == REGO){
			time_delta = 1;
		}
		assert(p != currPort);
		traverseUntil(srcPE, destPE,p,time_dist+time_delta,already_traversed,result);
	}
}

int CGRAXMLCompile::CGRA::getTimeDistBetweenPEs(PE* srcPE, PE* destPE){
	int ret_val = INT32_MAX;
	for(Port* op : srcPE->outputPorts){
		unordered_map<Port*,int> already_traversed;
		int tmp = -1;
		traverseUntil(srcPE, destPE,op,0,already_traversed,tmp);
		if(tmp != -1 && tmp < ret_val) ret_val = tmp;
	}
	assert(ret_val != INT32_MAX);
	return ret_val;
}

int CGRAXMLCompile::CGRA::getTimeClosestMEMPE(PE* currPE){
	assert(IntraPETimeDistAnalysisDone);
	PE* zeroth_currPE = MapTzeroPE[currPE];
	assert(TimeDistBetweenClosestMEMPEMap.find(zeroth_currPE) != TimeDistBetweenClosestMEMPEMap.end());

	return TimeDistBetweenClosestMEMPEMap[zeroth_currPE];
}

int CGRAXMLCompile::CGRA::getQuickTimeDistBetweenPEs(PE* srcPE, PE* destPE){
	assert(IntraPETimeDistAnalysisDone);

	PE* zeroth_srcPE = MapTzeroPE[srcPE];
	PE* zeroth_destPE = MapTzeroPE[destPE];

	assert(TimeDistBetweenPEMap.find(zeroth_srcPE) != TimeDistBetweenPEMap.end());
	assert(TimeDistBetweenPEMap[zeroth_srcPE].find(zeroth_destPE) != TimeDistBetweenPEMap[zeroth_srcPE].end());

	//Hack to make the zeroth_srcPE == zeroth_destPE more closer
	if(zeroth_srcPE != zeroth_destPE){
		return TimeDistBetweenPEMap[zeroth_srcPE][zeroth_destPE];
	}
	else{
		return TimeDistBetweenPEMap[zeroth_srcPE][zeroth_destPE] + 1;
	}
}

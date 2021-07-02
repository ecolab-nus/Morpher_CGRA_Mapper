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
#include <memory>
#include <nlohmann/json.hpp>

using namespace std;
using json = nlohmann::json;


namespace CGRAXMLCompile
{

class DFG;

struct TimeDistInfo{
	unordered_map<string,unordered_map<string,int>> TimeDistBetweenPEMap;
	unordered_map<string,int> TimeDistBetweenClosestMEMPEMap;
};

class CGRA : public Module
{
public:
	CGRA(const Module *Parent, std::string name, int t, int y, int x, DFG *dfg, std::string peType = "GENERIC_8REGF", int numberofDPs = 1, std::map<Port *, std::set<DFGNode *>> *_congestedPortPtr = NULL) : Module(Parent, name, "CGRA", t)
	{
		createGenericCGRA(x, y, t, peType, numberofDPs);
		this->peType = peType;
		this->numberofDPs = numberofDPs;
		this->congestedPortPtr = _congestedPortPtr;

		for (int t = 0; t < this->get_t_max(); ++t)
		{
			for (int y = 0; y < this->y_max; ++y)
			{
				for (int x = 0; x < this->x_max; ++x)
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
		EstablishTemporalLinkage();
		currDFG = dfg;
	}

	CGRA(const Module *Parent, std::string name, int t) : Module(Parent,name,"CGRA",t){}
	CGRA(std::string json_filename, int II, int x, int y,  std::map<Port *, std::set<DFGNode *>> *_congestedPortPtr = NULL) : Module(NULL,"CGRA_Ins","CGRA",0){
		json_file = json_filename;
		this->congestedPortPtr = _congestedPortPtr;
		t_max = II;
		x_max = x;
		y_max = y;

		ParseJSONArch(json_filename,II);
		EstablishTemporalLinkage();

		unordered_set<Module *> spms;
		SearchALLSPMs(this, spms);
		
		if (!spms.empty())
		{
			this->is_spm_modelled = true;
			checkMDPVars(spms);
		}
	}

	void createGenericCGRA(int x, int y, int t, std::string peType = "GENERIC_8REGF", int numberofDPs = 1);

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
	std::map<string, int> Op2ID;
	std::map<string, string> Desc2Mux;
	std::map<string, std::map<string, int>> SourcePort;
	std::map<int, int*> ConstRecord;
	int interConnection = false;

	string getFUName(string operations, int* output_ID);
	string getMuxName(string src_port_name, string desc_port_name, int* input_ID);
	int getRegInfo(string src_port_name, string dest_port_name, int* input_ID, int* output_ID);

	std::set<Port *> getConflictPorts(Port *p);
	void insertConflictPort(Port *a, Port *b);
	bool isConflictPortsEmpty(Port *p) { return conflictPorts[p].empty(); }
	DFG *currDFG = NULL;

	int minLatBetweenPEs = 1;

	Module* ParseModule(json& module_desc, Module* parent, string module_name, string type, int t, int x = -1, int y = -1);
	void ParseCGRA(json& cgra_desc, int II);
	unordered_map<int,std::vector<Module*>> subModArr;
	unordered_map<int,std::vector<Module*>> PEModArr;


	bool ParseJSONArch(string fileName, int II);
	bool PreprocessPattern(json& top);
	bool PreprocessInterSubmodConns(json& top);
	bool PreprocessTilePattern(json& top);
	void printARCHI(std::string fileName,json &connections, json &submods);
	json top_desc;

	unordered_set<PE*> getAllPEList();
	vector<PE*> getSpatialPEList(int t);
	PE* getPE(int t, int y, int x);
	PE* getLatencyPE(PE* currPE, int lat);

	void insertGlobalOP(string OP, int lat);
	unordered_map<string,int> getGlobalOPMinLatencyMap(){return GlobalOPMinLatencyMap;}
	string getCGRAName();

	TimeDistInfo analyzeTimeDist();
	void analyzeTimeDist(TimeDistInfo tdi);
	int getTimeClosestMEMPE(PE* currPE);
	int getQuickTimeDistBetweenPEs(PE* srcPE, PE* destPE);

	void EstablishTemporalLinkage();
	void PrintMappedJSON(string fileName);
	// by Yujie
	void PrintMappingForPillars(string fileName_i, string fileName_r);    

	void InsertVariable2SPMAddrInfo(json& output_json);
	void checkMDPVars(unordered_set<Module *>& spms);

	// unordered_map<DataPath*,unordered_set<string>> datapath_accessible_vars;
	unordered_map<string,Module*> Variable2SPM;
	unordered_map<string,int> Variable2SPMAddr;

	bool is_spm_modelled = false;
	int max_hops = 4;
	void insertConstOp(int id, int t, int Y, int X){
		int* second = new int[3];
		second[0] = t;
		second[1] = Y;
		second[2] = X;
		ConstRecord.insert(pair<int, int*>(id, second));
	}
	void DataPrepare(){
		//  for stdnoc
		Op2ID.insert(pair<string, int>("NOP", 0));
		Op2ID.insert(pair<string, int>("SEXT", 1));
		Op2ID.insert(pair<string, int>("ZEXT", 2));
		Op2ID.insert(pair<string, int>("TRUNC", 3));
		Op2ID.insert(pair<string, int>("INPUT", 4));
		Op2ID.insert(pair<string, int>("OUTPUT", 5));
		Op2ID.insert(pair<string, int>("PHI", 6));
		Op2ID.insert(pair<string, int>("CONST", 7));
		Op2ID.insert(pair<string, int>("ADD", 8));
		Op2ID.insert(pair<string, int>("SUB", 9));
		Op2ID.insert(pair<string, int>("MUL", 10));
		Op2ID.insert(pair<string, int>("DIV", 11));
		Op2ID.insert(pair<string, int>("AND", 12));
		Op2ID.insert(pair<string, int>("OR", 13));
		Op2ID.insert(pair<string, int>("XOR", 14));
		Op2ID.insert(pair<string, int>("SHLL", 15));
		Op2ID.insert(pair<string, int>("LS", 15));
		Op2ID.insert(pair<string, int>("SHRA", 16));
		Op2ID.insert(pair<string, int>("CLT", 16));
		Op2ID.insert(pair<string, int>("SHRL", 17));
		Op2ID.insert(pair<string, int>("RS", 17));
		Op2ID.insert(pair<string, int>("LOAD", 18));
		Op2ID.insert(pair<string, int>("STORE", 19));
		Op2ID.insert(pair<string, int>("GEP", 20));
		Op2ID.insert(pair<string, int>("ICMP", 21));
		Op2ID.insert(pair<string, int>("CMP", 21));
		Op2ID.insert(pair<string, int>("SHR", 22));
		Op2ID.insert(pair<string, int>("MOVC", 22));
		Op2ID.insert(pair<string, int>("SLT", 23));
		Op2ID.insert(pair<string, int>("CGT", 24));
		Op2ID.insert(pair<string, int>("SLTU", 25));
		Op2ID.insert(pair<string, int>("SELECT", 25));
		Op2ID.insert(pair<string, int>("SHLA", 26));
		Op2ID.insert(pair<string, int>("CMERGE", 26));
		Op2ID.insert(pair<string, int>("LOADH", 27));
		Op2ID.insert(pair<string, int>("STOREH", 28));
		Op2ID.insert(pair<string, int>("LOADB", 29));
		Op2ID.insert(pair<string, int>("STOREB", 30));
		Op2ID.insert(pair<string, int>("LOADB_CONST", 31));
		Op2ID.insert(pair<string, int>("STOREB_CONST", 32));
		Op2ID.insert(pair<string, int>("ADD_CONST", 33));
		Op2ID.insert(pair<string, int>("LS_CONST", 34));
		Op2ID.insert(pair<string, int>("CMP_CONST", 35));
		Op2ID.insert(pair<string, int>("CMERGE_CONST", 36));
		Op2ID.insert(pair<string, int>("CMERGE_NPB", 37));

		Desc2Mux.insert(pair<string, string>("DP0_I1", "muxI1"));
		Desc2Mux.insert(pair<string, string>("I1", "muxI1"));
		Desc2Mux.insert(pair<string, string>("DP0_I2", "muxI2"));
		Desc2Mux.insert(pair<string, string>("I2", "muxI2"));
		Desc2Mux.insert(pair<string, string>("DP0_P", "muxP"));
		Desc2Mux.insert(pair<string, string>("P", "muxP"));
		Desc2Mux.insert(pair<string, string>("WP0", "muxWP0"));
		Desc2Mux.insert(pair<string, string>("WP1", "muxWP1"));
		Desc2Mux.insert(pair<string, string>("NORTH_O", "muxNO"));
		Desc2Mux.insert(pair<string, string>("EAST_O", "muxEO"));
		Desc2Mux.insert(pair<string, string>("WEST_O", "muxWO"));
		Desc2Mux.insert(pair<string, string>("SOUTH_O", "muxSO"));

		std::map<string, int> muxI1;
		muxI1.insert(pair<string, int>("WEST_I", 0));
		muxI1.insert(pair<string, int>("EAST_I", 1));
		muxI1.insert(pair<string, int>("NORTH_I", 2));
		muxI1.insert(pair<string, int>("SOUTH_I", 3));
		muxI1.insert(pair<string, int>("RP0", 4));
		muxI1.insert(pair<string, int>("RP1", 5));
		muxI1.insert(pair<string, int>("T", 6)); // may be extra
		muxI1.insert(pair<string, int>("DP0_T", 6));
		std::map<string, int> muxI2;
		muxI2.insert(pair<string, int>("WEST_I", 0));
		muxI2.insert(pair<string, int>("EAST_I", 1));
		muxI2.insert(pair<string, int>("NORTH_I", 2));
		muxI2.insert(pair<string, int>("SOUTH_I", 3));
		muxI2.insert(pair<string, int>("RP0", 4));
		muxI2.insert(pair<string, int>("RP1", 5));
		std::map<string, int> muxP;
		muxP.insert(pair<string, int>("WEST_I", 0));
		muxP.insert(pair<string, int>("EAST_I", 1));
		muxP.insert(pair<string, int>("NORTH_I", 2));
		muxP.insert(pair<string, int>("SOUTH_I", 3));
		muxP.insert(pair<string, int>("RP0", 4));
		muxP.insert(pair<string, int>("RP1", 5));
		std::map<string, int> muxWP0;
		muxWP0.insert(pair<string, int>("WEST_I", 0));
		muxWP0.insert(pair<string, int>("EAST_I", 1));
		muxWP0.insert(pair<string, int>("NORTH_I", 2));
		muxWP0.insert(pair<string, int>("SOUTH_I", 3));
		muxWP0.insert(pair<string, int>("T", 4));
		muxWP0.insert(pair<string, int>("DP0_T", 4));
		std::map<string, int> muxWP1;
		muxWP1.insert(pair<string, int>("WEST_I", 0));
		muxWP1.insert(pair<string, int>("EAST_I", 1));
		muxWP1.insert(pair<string, int>("NORTH_I", 2));
		muxWP1.insert(pair<string, int>("SOUTH_I", 3));
		muxWP1.insert(pair<string, int>("T", 4));
		muxWP1.insert(pair<string, int>("DP0_T", 4));
		std::map<string, int> muxNO;
		muxNO.insert(pair<string, int>("T", 0));
		muxNO.insert(pair<string, int>("DP0_T", 0));
		muxNO.insert(pair<string, int>("RP0", 1));
		muxNO.insert(pair<string, int>("RP1", 2));
		std::map<string, int> muxEO;
		muxEO.insert(pair<string, int>("T", 0));
		muxEO.insert(pair<string, int>("DP0_T", 0));
		muxEO.insert(pair<string, int>("RP0", 1));
		muxEO.insert(pair<string, int>("RP1", 2));
		std::map<string, int> muxWO;
		muxWO.insert(pair<string, int>("T", 0));
		muxWO.insert(pair<string, int>("DP0_T", 0));
		muxWO.insert(pair<string, int>("RP0", 1));
		muxWO.insert(pair<string, int>("RP1", 2));
		std::map<string, int> muxSO;
		muxSO.insert(pair<string, int>("T", 0));
		muxSO.insert(pair<string, int>("DP0_T", 0));
		muxSO.insert(pair<string, int>("RP0", 1));
		muxSO.insert(pair<string, int>("RP1", 2));
		std::map<string, int> muxT;
		muxT.insert(pair<string, int>("LSU_O", 0));
		muxT.insert(pair<string, int>("ALU_O", 1));
		// std::map<string, std::map<string, int>> SourcePort;
		SourcePort["muxI1"]=muxI1;
		SourcePort["muxI2"]=muxI2;
		SourcePort["muxP"]=muxP;
		SourcePort["muxWP0"]=muxWP0;
		SourcePort["muxWP1"]=muxWP1;
		SourcePort["muxNO"]=muxNO;
		SourcePort["muxEO"]=muxEO;
		SourcePort["muxWO"]=muxWO;
		SourcePort["muxSO"]=muxSO;
		SourcePort["muxT"]=muxT;
		// SourcePort.insert(pair<string, std::map<string, int>>("muxI1", muxI1));
		// SourcePort.insert(pair<string, std::map<string, int>>("muxI2", muxI2));
		// SourcePort.insert(pair<string, std::map<string, int>>("muxP", muxP));
		// SourcePort.insert(pair<string, std::map<string, int>>("muxWP0", muxWP0));
		// SourcePort.insert(pair<string, std::map<string, int>>("muxWP1", muxWP1));
		// SourcePort.insert(pair<string, std::map<string, int>>("muxNO", muxNO));
		// SourcePort.insert(pair<string, std::map<string, int>>("muxEO", muxEO));
		// SourcePort.insert(pair<string, std::map<string, int>>("muxWO", muxWO));
		// SourcePort.insert(pair<string, std::map<string, int>>("muxSO", muxSO));
		// SourcePort.insert(pair<string, std::map<string, int>>("muxT", muxT));
	}

private:
	int x_max;
	int y_max;
	int t_max;

	std::map<Port *, std::set<Port *>> conflictPorts;
	std::map<Port *, std::set<DFGNode *>> *congestedPortPtr;

	std::map<int, std::map<int, std::map<int, PE *>>> PEArr;
	unordered_map<PE*,PE*> NextCyclePEMap;
	unordered_map<string,int> GlobalOPMinLatencyMap;
	string json_file;

	// unordered_map<PE*,unordered_map<PE*,int>> TimeDistBetweenPEMap;
	// unordered_map<PE*,int> TimeDistBetweenClosestMEMPEMap;

	unordered_map<string,unordered_map<string,int>> TimeDistBetweenPEMap;
	unordered_map<string,int> TimeDistBetweenClosestMEMPEMap;

#ifdef HIERARCHICAL

	unordered_map<string,int> TimeDistBetweenClosestMEMPEMapInTile;
#endif

	unordered_map<PE*,PE*> MapTzeroPE;

	void traverseUntil(PE* srcPE, PE* destPE, Port* currPort, int time_dist, unordered_map<Port*,int>& already_traversed, int& result);
	int getTimeDistBetweenPEs(PE* srcPE, PE* destPE);
	bool IntraPETimeDistAnalysisDone = false;

	void EstablishTemporalLinkageModule(Module* curr_cycle_module, Module* next_cycle_module);
	void PrintMappedJSONModule(Module* curr_module, json& output_json);
	void PrintMappedPillarsModule(Module* curr_module, json& output_json, ofstream& outfile_i);

	void SearchALLSPMs(Module *currModule, unordered_set<Module *> &spms);


	int tile_xdim=0;
    int tile_ydim=0;

    int pe_xdim=0;
	int pe_ydim=0;

};

} /* namespace CGRAXMLCompile */

#endif /* CGRA_H_ */

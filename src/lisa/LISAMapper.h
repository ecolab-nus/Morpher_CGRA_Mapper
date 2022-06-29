/*
 * LISAMapper.h
 * Since PathFinderMapper has implemented many functions and other source file also use PathFinderMapper, better to create a mapper which bases
 * on PathFinderMapper
 */

#include "../SimulatedAnnealingMapper.h"
#include "../PathFinderMapper.h"
#include "../DataPath.h"
#include "../debug.h"
#include "../util.h"
#include "LISAController.h"

#include <string>
#include <random>
#include <algorithm> 
#include <chrono>
#include <assert.h>
#include <sys/time.h>
#ifndef LISAMAPPER_H_
#define LISAMAPPER_H_

namespace CGRAXMLCompile
{


class LISAMapper : public SAMapper
{
public:
	LISAMapper(std::string fName) : SAMapper(fName){
		mapping_method_name  = "LISA";
	};

	bool LISAMap( arguments arg, TimeDistInfo &tdi, int  & start_II);
	
	void do_training(  arguments arg, TimeDistInfo &tdi, int start_II );
	void set_lisa_controller(arguments arg, int MII);
	bool initMap();
	float inner_map();

	bool LISAMapCore(CGRA *cgra, DFG *dfg);

	

	std::map<int, pos3d> dumpMapping(){
		int max_lat; return dumpMapping(max_lat);
	};
	std::map<int, pos3d> dumpMapping(int & max_latency);
	bool optimizeMappingToMinimizeCost();
	DataPath *  getLISADPCandidate(DFGNode *dfg_node, int accepted = 1 , int total_tried =1);
	std::vector<DataPath*> getCandidateByIIConstraint( int start_II, int end_II, DFGNode * node);

	std::pair<int,int> getIntervalByScheduleOrder( std::map<int, pos3d> & dumped_mapping, DFGNode * node, int scheduler_order );
    std::map<DataPath *, int> getCostByComm( std::map<int, pos3d> & dumped_mapping, std::vector<DataPath *> candidates, DFGNode *node );
    std::map<DataPath *, int> getCostByAssociation( std::map<int, pos3d> & dumped_mapping, std::vector<DataPath *> candidates, DFGNode *node, int eraly_lat );
    std::map<DataPath *, int> getCostForSameLevelNode( std::map<int, pos3d> & dumped_mapping, std::vector<DataPath *> candidates, DFGNode *node );

	DataPath *   getRoutingNode(int  x, int  y, int t);


	DataPath *  getCloseRandomDP(DFGNode* node, DataPath * old_dp){
        return  getCloseRandomDP( node,  old_dp,  this->cgra->get_x_max(),  this->cgra->get_y_max() );
    }
    DataPath *  getCloseRandomDP(DFGNode* node, DataPath * old_dp, int max_physical_dis,  int max_temp_dis );
	
	void enableTraining(){ is_training = true;}
    void disableTraining(){ is_training = false;}
	bool pass_lisa_arg(lisa_arguments la);

	


protected:

	std::shared_ptr<LISAController> lisa_ctrl;
	bool is_training = false;
	int max_training_iteration = 5;

    bool finish_init = false;
    bool lisa_eval_routing_priority = false;
	int after_mapping_optimize_cost_steps  =  1000;
	std::string dfg_id =  "none";


	// std::map<DFGNode*, int>  node_to_id_;
    // std::map<int, DFGNode*>  id_to_node_;
	std::shared_ptr<std::map<int, node_label>> dfg_label_;

	
};

} /* namespace CGRAXMLCompile */

#endif /* LISAMAPPER_H_ */

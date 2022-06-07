/*
 * LISAMapper.h
 * Since PathFinderMapper has implemented many functions and other source file also use PathFinderMapper, better to create a mapper which bases
 * on PathFinderMapper
 */

#include "../SimulatedAnnealingMapper.h"
#include "../PathFinderMapper.h"
#include "../DataPath.h"
#include "../debug.h"
#include "LISAController.h"

#include <string>
#include <random>
#include <algorithm> 
#include <chrono>
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

	bool LISAMap(CGRA *cgra, DFG *dfg);

	void initLisa(std::map<DFGNode*, int>  node_to_id,  std::map<int, DFGNode*>  id_to_node, std::shared_ptr<std::map<int, node_label>> dfg_label){
        dfg_label_ = dfg_label;
        node_to_id_ = node_to_id;
        id_to_node_ = id_to_node;
    }

	std::map<int, pos3d> dumpMappingForGNN();
	bool optimizeMapping();
	DataPath *  getLISADPCandidate(DFGNode *op, int accepted = 1 , int total_tried =1, int num_swap = 1);
	std::vector<DataPath*> getDesiredFu( int start_II, int end_II, DFGNode * node);

	std::pair<int,int> getIntervalByScheduleOrder( std::map<int, pos3d> & dumped_mapping, DFGNode * node, int scheduler_order );
    std::map<DataPath *, int> getCostByComm( std::map<int, pos3d> & dumped_mapping, std::vector<DataPath *> candidates, DFGNode *node );
    std::map<DataPath *, int> getCostByAssociation( std::map<int, pos3d> & dumped_mapping, std::vector<DataPath *> candidates, DFGNode *node, int start_II );
    std::map<DataPath *, int> getCostForSameLevelNode( std::map<int, pos3d> & dumped_mapping, std::vector<DataPath *> candidates, DFGNode *node );

	DataPath *  getCloseRandomFU(DFGNode* node, DataPath * old_dp){
        return  getCloseRandomFU( node,  old_dp,  this->cgra->get_x_max(),  this->cgra->get_y_max() );
    }
    DataPath *  getCloseRandomFU(DFGNode* node, DataPath * old_dp, int max_physical_dis,  int max_temp_dis );
	void setLISAController( std::shared_ptr<LISAController> ctrl){ 
        lisa_ctrl = ctrl;
    }
	void enableTraining(){ is_training = true;}
    void disableTraining(){ is_training = false;}

	


protected:

	std::shared_ptr<LISAController> lisa_ctrl;
	bool is_training = false;

    bool finish_init = false;
    bool lisa_eval_routing_priority = false;


	std::map<DFGNode*, int>  node_to_id_;
    std::map<int, DFGNode*>  id_to_node_;
	std::shared_ptr<std::map<int, node_label>> dfg_label_;

	
};

} /* namespace CGRAXMLCompile */

#endif /* LISAMAPPER_H_ */


#include <ctime>
#include <set>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <unordered_set>
#include <stdlib.h>
#include <memory>


#include "LISADFG.h"
#include "LISASchedule.h"
#ifndef ___LISAController_H__
#define ___LISAController_H__


//currently, we have three types of labels:
// a) schedule order: set the max value as 2 * len(longest_path); this is to leave some space for the medium value. 
//   for example, we have a graph with three nodes: A, B, and C. We have two edges that A->B and A->C. B, C have the same initial label. 
//   So value: A:1, B:2, C:2. If B has more child nodes, thus B should be scheduled before A. However, there is no space between 1 and 2.
//  initial value: 2* asap value
//  calculate value from mapping: normalized to  2 * len(longest_path)
// b) communication: initial value: the degree value.
//    let the communication value be the total length of distance between neighbours. Do not normalize it,
// c) same level node of distance. Same level means the node have the same ASAP value, and they have common parents or children. Also, make sure the node should have same descendant.
// This is not determined by the ancestor, as the ancestor usually has been placed before this..
// d) association: association value can be positive. The lower the value, the closer the neighbours. 
//    we should have two type of association: temporal association and spatial association.
//    Time difference can be negative or postive, but physical different must be positive.  
//    initial value: longest_path - distance_between_two_nodes. distance_between_two_nodes is calculated by difference of asap value;
//    calculate from : let us calculate the average distance between neigbours. 


struct node_label{
  int schedule_order;
  int communication;
  std::map<int, int> sameLevel_node_distance;
  std::map<int, std::pair<int, int>> association; //<spatial,temporal>

  std::string toStr(){
    std::stringstream output;
    output<<" schedule_order:"<<schedule_order<<" communication:"<<communication<<"\t  sameLevel_node_distance:\t";
    for(auto dis: sameLevel_node_distance){
      output<<" <"<<dis.first<<","<<dis.second<<"> ";
    }
     output<<"\n \tassociation:\t";
    for(auto aff: association){
      output<<" <"<<aff.first<<","<<aff.second.first<<","<<aff.second.second<<"> ";
    }
    return output.str();
  }

  int calTotalPoint(){
    int point = 0;
    point += schedule_order;
    for(auto pair: sameLevel_node_distance){
      point += pair.second;
    }
    for(auto pair: association){
      point += pair.second.first;
      point += pair.second.second;
    }

    return point;

  }
};

struct perf_metric{
    int ii;
    float cost;
    int running_time;

    bool operator < (const perf_metric& other) const {
        if( ii < other.ii ) return true;
        if ( ii == other.ii && cost < other.cost ) return true;
        if ( ii == other.ii && cost && other.cost && running_time <  other.running_time) return true;
        return false;
    }
};

using DFG_label = std::map<int, node_label>;

class LISAController{
  public:
    LISAController(int fabric_x, int fabric_y, std::string dfg_id, std::set<int> nodes, std::map<int, std::string> node_op, 
                  std::vector<std::pair<int,int>> edges, std::vector<std::pair<int,int>> backedges);

    void initLabels();
    
    //for generate and update label
    void passMapping(bool isBest, std::map<int, pos3d> & mapping, int max_lat, perf_metric perf);
    std::map<int, int> genScheduleOrderLabel(std::map<int, pos3d> & mapping,  int max_lat);
    std::map<int, int> genCommunicationLabel(std::map<int, pos3d> & mapping,  int max_lat);
    std::map<int, std::pair<int, int>> genlAssocationLabel(int node_id, std::map<int, pos3d> & mapping,  int max_lat);
    std::map<int, int> genSameLevelNodeLabel(int node_id, std::map<int, pos3d> & mapping,  int max_lat);
    void updateLabel(std::shared_ptr<DFG_label> label_a, std::shared_ptr<DFG_label> label_b, std::shared_ptr<DFG_label> update_label);
    void updateNodeLabel(node_label & node_label_a, node_label & node_label_b, node_label & update_node_label);

    //generate final combined label
    int calLabelDifference(node_label & a_label, node_label & b_label);
    std::vector<std::shared_ptr<DFG_label>> filterBestLabels(std::vector<std::shared_ptr<DFG_label>> best_labels);
    void generateCombinedBestLabelHistorically(perf_metric best_perf);


    
    std::shared_ptr<DFG_label> getBestLabel(){return best_label_;}
    std::shared_ptr<DFG_label> getCurrLabel(){return label_hist.back();}

    //for gnn dataset generation and call gnn inference
    void dumpBestLabelForGNNDataSet();
    std::string labelToStrForGNNDataSet(DFG_label & dfg_label);
    void callGNNInference();
    void dumpGraphForInference(std::string graph_name);
    void getLabelFromFile(DFG_label & dfg_label, std::string graph_name);

    //utils
    void setArchandDFGFileName(std::string arch_file_name, std::string dfg_file, std::string arch_name){
      arch_file_name_ = arch_file_name; dfg_file_name_ = dfg_file;   arch_name_ = arch_name; }
      
    std::string mappingToStr(std::map<int, pos3d> & mapping, int max_lat);
    std::string bestMappingToStr(){return mappingToStr(best_mapping_.first, best_mapping_.second);};
    bool isStartNode(int node_id){if(start_nodes_.find(node_id) == start_nodes_.end()) return false; return true;}
    std::set<int> getSameLevelNodes(int node_id){return sameLevel_nodes_with_common_descendent_[node_id];};
    std::string DFGLabelToStr(DFG_label & dfg_label) const;
     std::map<int, int> getNodeASAP(){
      std::map<int, int> node_asap;
      for(auto nf: dfg_->node_feature_){
        node_asap.emplace(nf.first, nf.second.asap);
      }
      return node_asap;
    }
    std::string gen_random(const int len)
    {
      std::string tmp_s;
      static const char alphanum[] =
          "0123456789"
          "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
          "abcdefghijklmnopqrstuvwxyz";
      srand((unsigned)time(NULL) * getpid());
      tmp_s.reserve(len);
      for (int i = 0; i < len; ++i)
        tmp_s += alphanum[rand() % (sizeof(alphanum) - 1)];
      return tmp_s;
    }

    void setMII(int MII){MII_ = MII;}
  
  private:
    std::shared_ptr<DFG_label> init_label_;
    std::shared_ptr<DFG_label> best_label_;
    std::shared_ptr<DFG_label> src_of_best_label_;
    std::pair<std::map<int, pos3d>, int> best_mapping_;
    std::vector<std::shared_ptr<DFG_label> > label_hist;
    std::map<std::shared_ptr<DFG_label>, perf_metric> label_perf; // this is to combine the best mapping

    // dfg relevant labels
    LISADFG* dfg_;
    std::set<int> start_nodes_; 
    std::map<int, std::set<int>> sameLevel_nodes_with_common_descendent_; 

    int fabric_x_;
    int fabric_y_;

    int MII_ = 0; //for cgra_me


    std::string arch_file_name_;
    std::string dfg_file_name_;
    std::string arch_name_; // this is for dumping label

    double label_difference_threashold = 1;

};


#endif
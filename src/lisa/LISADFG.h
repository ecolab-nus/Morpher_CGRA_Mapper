#include <ctime>
#include <set>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <unordered_set>
#include <sstream>
#include <map>
#include <vector>

#include "../debug.h"





#ifndef ___LISADFG_H__
#define ___LISADFG_H__


struct edge{
  int src;
  int des;

  bool operator<(const edge& b) const
  { 
    if (src != b.src){
      return src < b.src;
    }else {
      return des < b.des;
    }
  } 

  bool operator==(const edge& b) const
  { 
    return src == b.src && des == b.des;
  } 
};

struct feature{
  std::string op_type;
  int asap;
  int input_degree;
  int output_degree;

  std::string toStr(){
    std::stringstream output;
    output<< "op:"<<op_type<<" asap:"<<asap<<" in_degree:"<<input_degree<<" out_degree"<<output_degree;
    return output.str();
  }
};

class LISADFG{
  public:

    friend class LISAController;
    LISADFG(std::string dfg_id, std::set<int> nodes,  std::map<int, std::string> node_op, std::vector<std::pair<int,int>> edges);
    void calASAP();
    void calDegree();
    int cal_dist(int src, int des);
    bool is_edge(int src, int des){return !(node_children_[src].find(des) == node_children_[src].end() ); }
    std::string ToString();
  private:
    std::set<int> nodes_;
    std::map<int, std::string> node_op_;
    std::map<int, std::set<int>> node_parents_;
    std::map<int, std::set<int>> node_children_;
    std::set<edge> edges_;
    std::map<int, feature> node_feature_;
    std::map<int, std::set<int>> asap_to_node_;
    int max_length_; // means max asap value
    int max_degree_;
    std::string dfg_id_;
    
};


#endif
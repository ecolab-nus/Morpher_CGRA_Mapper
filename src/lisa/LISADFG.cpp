#include "LISADFG.h"

LISADFG::LISADFG(std::string dfg_id, std::set<int> nodes,  std::map<int, std::string> node_op,std::vector<std::pair<int, int>> edges)
{
  dfg_id_ = dfg_id;
  nodes_ = nodes;
  node_op_ = node_op;

  for(auto node: nodes_){
    node_parents_[node] = std::set<int>();
    node_children_[node] = std::set<int>();
  }
  for (auto e : edges)
  {
    edges_.insert(edge{e.first, e.second});
    node_children_[e.first].insert(e.second);
    node_parents_[e.second].insert(e.first);;
  }


  // set node feature
  for(auto node: nodes){
    node_feature_[node] = feature{"none", 0 , 0 , 0};
  }
  // std::cout<<ToString();
  calASAP();
  calDegree();
  LOG(LISAGNN)<<ToString();
  for(auto nf: node_feature_){
    int node_id = nf.first;
    int asap_value = nf.second.asap;
    if(asap_to_node_.find(asap_value) == asap_to_node_.end()){
      asap_to_node_.emplace(asap_value, std::set<int>());
    }

    asap_to_node_[asap_value].insert(node_id);
  }

}


void LISADFG::calASAP(){
  std::map<int, int> asap_value;
  std::set<int> non_scheduled ;
  for(auto node: nodes_){
    if(node_parents_[node].size() == 0){
      asap_value[node] = 1;
    }else{
      non_scheduled.insert(node);
    }
  }

  assert(non_scheduled.size() != nodes_.size());

  while(non_scheduled.size() > 0){
    bool erase_node = false;

    for(auto it = non_scheduled.begin(); it != non_scheduled.end( ); ){
      bool can_start =  true;
      int max_asap = 1;
      
      for(auto parent: node_parents_[*it]){
        if (non_scheduled.find(parent) != non_scheduled.end()){
          can_start= false;
          break;
        }else{
          int temp_asap = asap_value[parent];
          max_asap = temp_asap > max_asap ? temp_asap: max_asap;
        }
      }

      if(can_start){
        asap_value[*it] = max_asap + 1;
        it =  non_scheduled.erase(it);
        erase_node = true;
      }else{
        it ++;
      }
    }

    assert(erase_node);
  }
  int max_asap = 0;
  for(auto value: asap_value){
      int temp_value = value.second;
      max_asap = temp_value > max_asap? temp_value:max_asap;
  }
  max_length_ = max_asap ; 
  for(auto node: nodes_){
    node_feature_ [node].asap = asap_value[node];
  }
}

void LISADFG::calDegree(){
  int max_degree = 0;
  for(auto node: nodes_){
    node_feature_ [node].input_degree = node_parents_[node].size();
    node_feature_ [node].output_degree = node_children_[node].size();
    int temp_d = node_parents_[node].size() + node_children_[node].size();
    max_degree = std::max(max_degree,temp_d );
  }
  max_degree_ = max_degree;
}

int LISADFG::cal_dist(int src, int des){
  std::set<int> to_visit_node;
  to_visit_node.insert(src);

  int level = 0;
  while(true){
    level ++ ;
    std::set<int> new_nodes ;
    for(auto node: to_visit_node){
      for(auto child: node_children_[node]){
        if( child  == des){
          return level;
        }
        new_nodes.insert(child);
      }
    }
    assert(new_nodes.size() != 0);
    to_visit_node.clear();
    for(auto node: new_nodes){
      to_visit_node.insert(node);
    }
  }
  assert(false);
}
std::string LISADFG::ToString(){
  std::stringstream output;
  output<<"\n##########node list:\n";
  for(auto node: nodes_){
    output<<" node:"<<node<<"\t"<<node_feature_[node].toStr();
    output<<"\tparent nodes:";
    for(auto parent:node_parents_[node]){
      output<<parent<<",";
    }
    output<<"\tchildren nodes:";
    for(auto child:node_children_[node]){
      output<<child<<",";
    }
    output<<"\n";
  }
  output<<"##########edges:\n";
  for(auto edge: edges_){
    output<<"("<<edge.src<<","<<edge.des<<")  ";
  }
  output<<"\n\n";
  return output.str();
}
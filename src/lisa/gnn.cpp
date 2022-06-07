#include "gnn.h"

GNN::GNN(){

  //let us give the dfg a name;
  dfg_name_ = gen_random(10);
  LOG(DGNN)<<"set dfg name:"<<dfg_name_;

}


void GNN::inference(){
  dump_dfg();
  dump_feature();
  // call_gnn();
}



void GNN::dump_dfg(){

  std::ofstream outfile ("./gnn_data/"+dfg_name_+"_edge.txt");

  // for(auto edge: edges){
  //   int id = node_to_id_[node];
  //   for(auto input_val: node->input){
  //     auto s_node = input_val->input;
  //     assert(node_to_id_.find(s_node) !=  node_to_id_.end());
  //     outfile << id <<" "<<node_to_id_[s_node]<< std::endl;
  //   }

  // }
  outfile.close();
}

void GNN::dump_feature(){

  std::ofstream outfile ("./gnn_data/"+dfg_name_+"_feature.txt");

  // auto & nodes = opgraph_->op_nodes;
  // for(auto node: nodes){
  
    
  //   outfile << node->opcode<< std::endl;

  // }
  outfile.close();
}
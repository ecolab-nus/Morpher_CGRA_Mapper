#include "LISAController.h"

LISAController::LISAController(int fabric_x, int fabric_y, std::string dfg_id, std::set<int> nodes, std::map<int, std::string> node_op, 
                  std::vector<std::pair<int,int>> edges, std::vector<std::pair<int,int>> backedges){
  fabric_x_ = fabric_x;
  fabric_y_ = fabric_y;
  dfg_ = new LISADFG(dfg_id, nodes, node_op, edges, backedges);
  init_label_ = std::make_shared<DFG_label> (); 
  // std::cout<<"fabric x"<<fabric_x_<<"fabric_y_"<<fabric_y_<<std::endl;
  initLabels();
  label_hist.push_back(init_label_);
  
}

void LISAController::initLabels(){

  for(auto f: dfg_->node_feature_ )  {
    if(f.second.asap == 1){
      start_nodes_.insert(f.first);
      // std::cout<<"start node"<<f.first<<" ";
    }
  }

  //constrcut transitive data dependency
  std::map<int, std::set<int>> transitive_parents;
  std::map<int, std::set<int>> transitive_children;

  for (auto node : dfg_->nodes_)
  {
      transitive_children.emplace(node, std::set<int>());
      transitive_parents.emplace(node, std::set<int>());
  }

  //
  for(int i = 0 ; i < dfg_->max_length_; i++){
    for(auto node: dfg_->nodes_){
      for(auto e: dfg_->edges_){
        if(e.des == node){
          transitive_parents[e.des].insert(e.src);
          for(auto parent: transitive_parents[e.src]){
            transitive_parents[e.des].insert(parent);
          }
        }
      }
    }
  }

  for(auto it: transitive_parents){
    int node = it.first;
    for(auto parent: it.second){
      transitive_children[parent].insert(node);
    }
  }

  // std::cout<<"transitive_parents\n";
  // for(auto i: transitive_parents){
  //   std::cout<<i.first<<", "<<i.second.size()<<"\n";
  // }
  // std::cout<<"transitive_children\n";
  // for(auto i: transitive_children){
  //   std::cout<<i.first<<", "<<i.second.size()<<"\n";
  // }
  auto asap_sort = [&](int ca, int cb){
      return dfg_->node_feature_[ca].asap < dfg_->node_feature_[cb].asap;
    };
  auto asap_reverse_sort = [&](int ca, int cb){
      return dfg_->node_feature_[ca].asap > dfg_->node_feature_[cb].asap;
    };  
  auto cal_distance = [&, this](int a, int b){
    auto& a_children = transitive_children[a];
    auto& b_children = transitive_children[b];
    std::vector<int> common_children;
    for(auto ac : a_children){
      if( b_children.find(ac)!= b_children.end()){
        common_children.push_back(ac);
      }
    }
    auto& a_parents = transitive_parents[a];
    auto& b_parents = transitive_parents[b];
    std::vector<int> common_parents;
    for(auto ac : a_parents){
      if( b_parents.find(ac)!= b_parents.end()){
        common_parents.push_back(ac);
      }
    }
    
    if(common_children.size() == 0 && common_parents.size() == 0){ return std::make_pair(false, 0);  }
    
    

    int dist = 0, child_dist = 0, parent_dist = 0;
    if(common_children.size() != 0){
      std::sort(common_children.begin(), common_children.end(), asap_sort );
      int child = common_children.front();
      child_dist = (dfg_->cal_dist(a, child) +dfg_->cal_dist(b, child))/2;
    }


    if(common_parents.size() != 0){
      int parent = common_parents.front();
      std::sort(common_parents.begin(), common_parents.end(), asap_reverse_sort );
      parent_dist = (dfg_->cal_dist (parent, a) +dfg_->cal_dist(parent, b))/2;
    }
    if(child_dist != 0 && parent_dist!=0 ){
      dist = (parent_dist + child_dist)/2;
    }else{
      dist = parent_dist + child_dist;
    }
     
    return std::make_pair(true, dist);
  };

  int max_length = dfg_->max_length_;
  for(auto f: dfg_->node_feature_){
    auto fet = f.second;
    int node_id = f.first;

    std::map<int, int> sameLevel_node_distance;
    std::map<int, std::pair<int, int>> association;

    sameLevel_nodes_with_common_descendent_[node_id] = std::set<int> ();
    for(auto other_id: dfg_->asap_to_node_[fet.asap]){
      if (other_id  == node_id) continue;
      // dist structure: pair(bool, int)  bool means whether they have child or not.
      auto dist = cal_distance(node_id, other_id);
      if(dist.first){
        sameLevel_node_distance[other_id] = dist.second; 
        sameLevel_nodes_with_common_descendent_[node_id].insert(other_id);
      }
    }
   
    for(auto parent: dfg_->node_parents_[node_id]){
      association[parent] =  std::make_pair(0,std::abs(fet.asap - dfg_->node_feature_[parent].asap));
    }
     for(auto child: dfg_->node_parents_[node_id]){
      association[child] =  std::make_pair(0,std::abs(fet.asap - dfg_->node_feature_[child].asap));  ;
    }
    init_label_->emplace(node_id, node_label{(fet.asap)*2, fet.input_degree + fet.output_degree, sameLevel_node_distance, association});
  }

  LOG(DLABEL)<<DFGLabelToStr(*init_label_);
  best_label_ = init_label_;
}


std::string LISAController::DFGLabelToStr(DFG_label  & dfg_label) const {
  std::stringstream output;
  output<<"\n ******** dump labels\n";
  for(auto nl: dfg_label){
    output<<"node id:"<<nl.first;
    output<<nl.second.toStr()<<"\n";
  }
  output<<" ******** dump labels finish\n";
  return output.str();
}

void LISAController::passMapping(bool isBest, std::map<int, pos3d> & mapping, int max_lat, perf_metric perf){

  if(isBest){
    std::shared_ptr<DFG_label> extracted_label = std::make_shared<DFG_label> (); 
    LISASchedule sched (dfg_, fabric_x_, fabric_y_, max_lat, mapping) ;
    LOG(DLABEL)<<sched.simpleSchedToString();
    //calculate label;
    auto sched_order_label = genScheduleOrderLabel(mapping, max_lat);
    auto comm_label = genCommunicationLabel(mapping, max_lat);

    for(auto node : dfg_->nodes_){
      auto ass_label = genlAssocationLabel(node, mapping, max_lat);
      auto sameLevel_node_label = genSameLevelNodeLabel(node, mapping, max_lat);
      extracted_label->emplace(node, node_label{sched_order_label[node],comm_label[node], sameLevel_node_label, ass_label }); 
    }
    LOG(DLABEL)<<DFGLabelToStr(*extracted_label);
    best_mapping_ = std::make_pair(mapping, max_lat);
    // std::shared_ptr<dfg_label> update_label = std::make_shared<dfg_label> ();
    // updateLabel(label_hist.back(), extracted_label, update_label);
    src_of_best_label_ = best_label_;
    best_label_ = extracted_label;
    label_perf.emplace(extracted_label, perf);
    label_hist.push_back(best_label_);
    
  }else{
    // std::shared_ptr<dfg_label> update_label = std::make_shared<dfg_label> ();
    // updateLabel(label_hist.back(), src_of_best_label_, update_label);
    // label_hist.push_back(update_label);

    // doing nothing
  }

}

//this function is to calculate the difference between two labels of one node. 
//It is the sum of difference for each feature.
int LISAController::calLabelDifference(node_label & a_label, node_label & b_label){
  int difference = 0;

  // check schedule order
  difference += std::abs(a_label.schedule_order - b_label.schedule_order);

  //sameLevel_node_distance
  for(auto pair: a_label.sameLevel_node_distance){
    int node_id = pair.first;
    auto & b_dis = b_label.sameLevel_node_distance;
    assert(b_dis.find(node_id)!= b_dis.end());
    difference += std::abs(pair.second - b_dis[node_id]);
  }

  //association value
  for(auto pair: a_label.association){
    int node_id = pair.first;
    auto & b_ass= b_label.association;
    assert(b_ass.find(node_id)!= b_ass.end());
    difference += std::abs(pair.second.first - b_ass[node_id].first);
    difference += std::abs(pair.second.second - b_ass[node_id].second);
  }

  return difference;
}

std::vector<std::shared_ptr<DFG_label>> LISAController::filterBestLabels(std::vector<std::shared_ptr<DFG_label>> best_labels){
  if(best_labels.size()==1) return best_labels;
  int label_number = best_labels.size();
  std::vector<int> labels_difference; //temporialy, did not use this
  std::vector<std::set<int>> similarLabel_num;
  for(int i = 0; i < label_number; i++ ){
    labels_difference.push_back(0);
    similarLabel_num.push_back(std::set<int>());
  }

  std::vector<int> labels_points;
  for(int i = 0; i < label_number; i++ ){
    int label_point = 0;
    for(auto pair: *(best_labels[i]) ){
      label_point += pair.second.calTotalPoint();
    }
    labels_points.push_back(label_point);
  }
  int point_sum = 0;
  for(auto point: labels_points){
    point_sum += point;
  }
  int average_label_point = point_sum/label_number;

  std::string result_filename = "label_filter_log.txt";
  std::ofstream result_file;
  result_file.open (result_filename, std::ios_base::app);
  result_file<<"arch:"<<arch_file_name_<<" average_label_point:"<<average_label_point<<" dfgfile:"<<dfg_file_name_<< "("<<fabric_x_<<","<<fabric_y_<<")\n";
  result_file<<"before filter: number of labels-"<<label_number<<" "<<" points(";
  for(auto p: labels_points){
    result_file<<p<<", ";
  }
  result_file<<")\n";
  result_file<<"difference:"; 


  //calculate label difference 
  for(int i = 0; i < label_number; i++ ){
    for(int j = i+1; j < label_number; j ++ ){
      int total_difference = 0;
      for(auto pair: *(best_labels[i])){
        int node_id = pair.first;
        assert(best_labels[j]->find(node_id) != best_labels[j]->end());
        total_difference += calLabelDifference(pair.second, best_labels[j]->at(node_id));
      }
      if(total_difference < label_difference_threashold * average_label_point ){
        //similar
        similarLabel_num[i].insert(j);
        similarLabel_num[j].insert(i);
      }
      result_file<<i<<","<<j<<"->"<<total_difference<<" ";
      labels_difference[i] +=  total_difference;
      labels_difference[j] +=  total_difference;
    }
  }

  std::vector<int> sorted_label;
  for(int i = 0; i < label_number; i++){
    sorted_label.push_back(i);
  }
  std::sort(sorted_label.begin(), sorted_label.end(), [&](int  a, int b){
      return similarLabel_num[a].size() > similarLabel_num[b].size();
  });


  std::vector<std::shared_ptr<DFG_label>> updated_labels;
  updated_labels.push_back( best_labels[sorted_label.front()]);
  for(auto label_id: similarLabel_num[sorted_label.front()]){
    updated_labels.push_back(best_labels[label_id]);
  }

  
  
   result_file<<"\nafter filter: number of labels-"<<updated_labels.size()<<" "<<" points("<<sorted_label.front()<<", ";
  for(auto label_id: similarLabel_num[sorted_label.front()]){
    result_file<<labels_points[label_id]<<", ";
  }
  result_file<<")\n\n"; 
  

  return updated_labels;
}

void LISAController::generateCombinedBestLabelHistorically(perf_metric best_perf){

  LOG(COMBINELABEL)<<"best label: "<<best_perf.ii<<" "<<best_perf.cost;
  if(label_perf.size() == 0){
     best_label_ = init_label_;
     return ;
  }
  std::shared_ptr<DFG_label> combined_best_label = std::make_shared<DFG_label> (); 

  std::vector<std::shared_ptr<DFG_label>> best_labels;
  for(auto lp: label_perf){
    if(lp.second.ii == best_perf.ii){
      float coef = (float(lp.second.cost)) / best_perf.cost;
      if(coef <= 1.1){
        LOG(COMBINELABEL)<<"select label: "<<lp.second.ii<<" "<<lp.second.cost;
        best_labels.push_back(lp.first);
      }
    }
  }
  // best_labels = filterBestLabels(best_labels);


  int best_label_num = best_labels.size();
  //dump for gnn label filter
  {
     if(dfg_->dfg_id_ != "none"){
        std::ofstream result_file;
        result_file.open ("../lisa_gnn/data/labels/"+arch_name_+"/"+ "label_evaluate.txt", std::ios_base::app); 
        result_file<<dfg_->dfg_id_<<" "<<MII_<<" "<<best_perf.ii<<" "<<best_label_num<<"\n";
        result_file.close();
      }
  }

  


  auto temp_label = best_labels.front();
  for(auto nl: *temp_label ){
    int node_id = nl.first;
    LOG(COMBINELABEL)<<"generate combined label for node "<<node_id;
    auto & basic_label = nl.second;
    int schedule_order = 0;
    int communication = 0;
    std::map<int, int> sameLevel_node_distance;
    std::map<int, std::pair<int, int>> association;
    for(auto ass: basic_label.association){
      association[ass.first] = std::make_pair(0, 0);
    }
    for(auto sn: basic_label.sameLevel_node_distance){
      sameLevel_node_distance[sn.first] = 0;
    }

    for(auto best_label: best_labels){
      auto & n_label = best_label->at(node_id); 
      LOG(COMBINELABEL)<<" select label: "<<n_label.toStr();
      schedule_order += n_label.schedule_order;
      communication += n_label.communication;
      for(auto ass: n_label.association){
        association[ass.first] = std::make_pair(ass.second.first + association[ass.first].first, ass.second.second +  association[ass.first].second);
      }
      for(auto sn: n_label.sameLevel_node_distance){
        sameLevel_node_distance[sn.first] = sameLevel_node_distance[sn.first] + sn.second;
      }
    }
    for(auto s : association){
      association[s.first] = std::make_pair(s.second.first/best_label_num, s.second.second/best_label_num);
    }
    for(auto s: sameLevel_node_distance){
      sameLevel_node_distance[s.first] = s.second/best_label_num;
    }
    node_label final_node_label{schedule_order/best_label_num, communication/best_label_num, sameLevel_node_distance, association};
    LOG(COMBINELABEL)<<" final label: "<<final_node_label.toStr();
    combined_best_label->emplace(node_id, final_node_label);
  }

  best_label_ = combined_best_label;

}
std::string LISAController::mappingToStr(std::map<int, pos3d> & mapping, int max_lat){

  LISASchedule sched (dfg_, fabric_x_, fabric_y_, max_lat, mapping) ;
  return sched.simpleSchedToString();
}

void LISAController::updateLabel(std::shared_ptr<DFG_label> label_a, std::shared_ptr<DFG_label> label_b, std::shared_ptr<DFG_label> update_label){
  // this function is useless
  for(auto it = label_a->begin(); it!= label_a->end(); it++){
    int node_id = it->first;
    auto & a_l  = it->second;
    auto b_l =  label_b->at(node_id);
    LOG(UPDATELABEL)<<node_id ;
    LOG(UPDATELABEL)<< "node label update source: a " + a_l.toStr();
    LOG(UPDATELABEL)<< "node label update source: b " + b_l.toStr();
    node_label update_node_label;
    updateNodeLabel(a_l, b_l, update_node_label );
    LOG(UPDATELABEL)<< "update label" + update_node_label.toStr();
    update_label->emplace(node_id, update_node_label);
  }

}

void LISAController::updateNodeLabel(node_label & node_label_a, node_label & node_label_b, node_label & update_node_label){
  update_node_label.schedule_order = (node_label_a.schedule_order + node_label_b.schedule_order) / 2;
  update_node_label.communication = (node_label_a.communication + node_label_b.communication) / 2;
  auto & a_ass = node_label_a.association;
  auto & b_ass = node_label_b.association;

  for(auto a: a_ass){
    int neighbor = a.first;
    auto info = a.second;
    auto last_info =  b_ass[neighbor];
    update_node_label.association[neighbor] = std::make_pair( (info.first + last_info.first)/2 , (info.second + last_info.second)/2  );
  }

  auto & a_start = node_label_a.sameLevel_node_distance;
  auto & b_start = node_label_b.sameLevel_node_distance;
   for(auto a: a_start){
    int neighbor = a.first;
    auto info = a.second;
    auto last_info =  b_start[neighbor];
    update_node_label.sameLevel_node_distance[neighbor] = (info + last_info)/2;
  }
}


std::map<int, int> LISAController::genSameLevelNodeLabel(int node_id, std::map<int, pos3d> & mapping,  int max_lat){
  std::map<int, int> sameLevel_node_dist;

  int node_x =  mapping[node_id].x;
  int node_y =  mapping[node_id].y;


  for(auto other_node: sameLevel_nodes_with_common_descendent_[node_id]){
    auto & m = mapping[other_node];
    int dist = std::abs(m.x- node_x)+std::abs(m.y- node_y);
    sameLevel_node_dist [other_node] = dist;
  }

  return sameLevel_node_dist;

}
std::map<int, int> LISAController::genScheduleOrderLabel(std::map<int, pos3d> & mapping,  int max_lat){
  std::map<int, int> order;

  int max_value = 2 * dfg_->max_length_;
  float factor =  (float)max_value / max_lat;
  for(auto m: mapping){
    order[m.first] =m.second.t * factor;
  }

  return order;
}

std::map<int, int> LISAController::genCommunicationLabel(std::map<int, pos3d> & mapping,  int max_lat){
  std::map<int, int> label_value;

  auto distance_lambda = [](pos3d a, pos3d b) { 
    return std::abs(b.t- a.t) + std::abs(b.x- a.x)+std::abs(b.y- a.y); 
  };

  //calculate distance
  for(auto m: mapping){
    int node_id = m.first;
    auto& pos = m.second;
    int total_distance = 0;
    for(auto e: dfg_->edges_){
      pos3d src, des;
      if(e.src == node_id ){
        src = pos;
        des = mapping[e.des];
      }else if (e.des == node_id ){
        src = mapping[e.src];
        des = pos;
      }else{
        continue;
      }
      total_distance += distance_lambda(src, des);
    }
    label_value[node_id] = total_distance; 
  }

  return label_value;
  
}
void LISAController::dumpBestLabelForGNNDataSet(){
  if(dfg_->dfg_id_ == "none"){
    return;
  }
  std::string output_str = labelToStrForGNNDataSet(*best_label_);
  std::ofstream result_file;
  result_file.open ("../lisa_gnn/data/labels/"+arch_name_+"/"+ dfg_->dfg_id_+".txt", std::ios_base::trunc); 
  result_file<<output_str;
  result_file.close();
}
std::string LISAController::labelToStrForGNNDataSet(DFG_label & dfg_label){
  std::stringstream finaoutput ;
  for(auto & node_label: dfg_label){
    finaoutput<<node_label.first<<" "<<node_label.second.schedule_order<<"\n";
  }
  finaoutput<<"###\n";
  for(auto & node_label: dfg_label){
    finaoutput<<node_label.first<<" "<<node_label.second.communication<<"\n";
  }
  finaoutput<<"###\n";
  std::map<std::pair<int, int>, int> sameLevel_node_distance;
  std::map<std::pair<int, int>, std::pair<int, int>> association_value;
  for(auto & node_label: dfg_label){
    int curr_node_id = node_label.first;
    for(auto & start_node_info: node_label.second.sameLevel_node_distance){
      int another_node_id = start_node_info.first;
      bool pair_exist = false;
      if( another_node_id > curr_node_id){
        continue;
      }
      sameLevel_node_distance.emplace(std::make_pair(curr_node_id, another_node_id), start_node_info.second);
    }

    for(auto & ass_info: node_label.second.association){
      int another_node_id = ass_info.first;
      if(dfg_->is_edge(curr_node_id, another_node_id)){
        association_value.emplace(std::make_pair(curr_node_id,another_node_id), ass_info.second);
      }
    }
  }
  for(auto & dist: sameLevel_node_distance){
    finaoutput<<dist.first.first<<" "<<dist.first.second<<" "<<dist.second<<"\n";
  }
  finaoutput<<"###\n";

  for(auto & ass: association_value){
    finaoutput<<ass.first.first<<" "<<ass.first.second<<" "<<ass.second.first<<" "<<ass.second.second<<"\n";
  }
  return finaoutput.str();

}

std::map<int, std::pair<int, int>> LISAController::genlAssocationLabel(int node_id,std::map<int, pos3d> & mapping,  int max_lat){
  std::map<int, std::pair<int, int>> ass_label;
  auto curr_pos = mapping[node_id];
  int node_t = curr_pos.t;
  int node_x = curr_pos.x;
  int node_y = curr_pos.y;

  for(auto& parent: dfg_->node_parents_[node_id]){
    auto pos = mapping[parent];
    int t_diff = node_t - pos.t;
    int x_diff = node_x - pos.x;
    int y_diff = node_y - pos.y;
    // time difference can be negative or postive, but physical different must be positive.  
    ass_label[parent] = std::make_pair(abs(x_diff)+ abs(y_diff), abs(t_diff));
  }

   for(auto& child: dfg_->node_children_[node_id]){
    auto pos = mapping[child];
    int t_diff = node_t - pos.t;
    int x_diff = node_x - pos.x;
    int y_diff = node_y - pos.y;
    // time difference can be negative or postive, but physical different must be positive.  
    ass_label[child] = std::make_pair(abs(x_diff)+ abs(y_diff), abs(t_diff));
  }
  return ass_label;
}
void LISAController::callGNNInference(){
  std::string graph_name = gen_random(10);
  dumpGraphForInference(graph_name);
  char* pPath;
  pPath = getenv ("LISA_DIR");
  std::string lisa_dir_name ="" ;
  lisa_dir_name.assign(pPath);
  
  std::string command  =  "bash  "+lisa_dir_name+"/cgra_me/call_gnn.sh " + graph_name + " " + arch_name_;
  // std::cout<<"command line"<<command<<"\n";
  system(command.c_str());
  std::shared_ptr<DFG_label> gnn_dfg_label = std::make_shared<DFG_label> (); 
  getLabelFromFile(*gnn_dfg_label, graph_name);
  std::cout<<"gnn label"<<DFGLabelToStr(*gnn_dfg_label);
  best_label_ = gnn_dfg_label;
}

void LISAController::dumpGraphForInference(std::string graph_name){
  std::stringstream edge_output;
  for (auto e: dfg_->edges_)
  {
    edge_output<<e.src<<" "<<e.des<<"\n";
  }
  std::ofstream result_file;
  result_file.open ("../lisa_gnn/data/infer/"+ graph_name+".txt", std::ios_base::trunc); 
  result_file<<edge_output.str();
  result_file.close();


  std::stringstream node_op_output;

  int max_id = 0;
  for (auto node_id: dfg_->nodes_ ){
    max_id = std::max(max_id, node_id);
  }

  for (int i = 0; i <= max_id; i++ ){
    assert(dfg_->node_op_.find(i) != dfg_->node_op_.end());
    node_op_output<<dfg_->node_op_[i]<<"\n";
  }
  result_file.open ("../lisa_gnn/data/infer/"+ graph_name+"_op.txt", std::ios_base::trunc); 
  result_file<<node_op_output.str();
  result_file.close();
}

void LISAController::getLabelFromFile(DFG_label & dfg_label, std::string graph_name){
  std::ifstream label_file("../lisa_gnn/data/infer/"+ graph_name+"_label.txt");
  std::string line;
  int label_id = 0;
  std::string singleToken;

  std::map<int, int > schedule_order;
  std::map<int, int > communication_distance;
  std::map<std::pair<int, int>, int> sameLevel_node_distance;
  std::map< std::pair<int, int>, int> neighbor_assocation_spatial;
  std::map< std::pair<int, int>, int> neighbor_assocation_temporal;
  while (std::getline(label_file, line)) {
      if (line.find("###") != std::string::npos) {
        label_id ++;
        std::getline(label_file, line);
      }
      std::stringstream ss(line);
      std::vector<int> tokens;
      while(std::getline(ss,singleToken,' ')){
        tokens.push_back(atoi(singleToken.c_str()));
      }
      if(label_id == 0){
        assert(tokens.size() == 2);
        schedule_order.emplace(tokens[0], tokens[1]);
      // }else if(label_id == 1){
      //  communication_distance.emplace(tokens[0], tokens[1]);
      // } assert(tokens.size() == 2);
      } else if(label_id == 1){
        assert(tokens.size() == 3);
        sameLevel_node_distance.emplace(std::make_pair(tokens[0], tokens[1]), tokens[2]);
      }else if(label_id == 2){
        assert(tokens.size() == 3);
        neighbor_assocation_spatial.emplace(std::make_pair(tokens[0], tokens[1]), tokens[2]);
      }
      else if(label_id == 3){
        assert(tokens.size() == 3);
        neighbor_assocation_temporal.emplace(std::make_pair(tokens[0], tokens[1]), tokens[2]);
      }else{
        assert(false);
      }
  }

  
  for(auto od: schedule_order){
    int id = od.first;
    int order = od.second;
    // assert(communication_distance.find(id) != communication_distance.end());
    int comm = 0;
    std::map<int, int> this_node_start;
    for (auto st_info: sameLevel_node_distance){
      if(st_info.first.first == id){
        this_node_start.emplace(st_info.first.second, st_info.second);
      }else if (st_info.first.second == id){
        this_node_start.emplace(st_info.first.first, st_info.second);
      }
    }
    std::map<int, std::pair<int, int>> this_neigbor_association;
    for (auto neighbor: neighbor_assocation_spatial){
      int neihgbor_id = -1;
      if(neighbor.first.first == id){
        neihgbor_id = neighbor.first.second;
      }else if (neighbor.first.second == id){
        neihgbor_id = neighbor.first.first;
      }else{
        continue;
      }
      
      assert(neighbor_assocation_temporal.find(neighbor.first)!= neighbor_assocation_temporal.end());
      this_neigbor_association.emplace(neihgbor_id, std::make_pair(neighbor.second, neighbor_assocation_temporal[neighbor.first]));

    }
   
    dfg_label.emplace(id,  node_label{order, comm, this_node_start, this_neigbor_association});
  }
}
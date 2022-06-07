#include "LISASchedule.h"

LISASchedule::LISASchedule(LISADFG* dfg, int x, int y, int t, std::map<int, pos3d> & mapping){
  // std::cout<<"fabric x2"<<x<<"fabric_y_"<<y<<std::endl;

  dfg_ = dfg;
  x_ = x;
  y_ = y;
  t_ = t;
  // ts_mapping_ =  (int*) std::malloc( x_ * y_ * t_ * sizeof(int));
  // for(int i =0; i<t_; i++){
  //   for(int j =0; j<x_; j++){
  //       for(int k = 0; k<y_;k++){
  //           ts_mapping_[getOffset(j,k,i)] = -1;
  //       }
  //   }
  // }
  // // std::cout<<"max x y  t"<<x<<" "<<y<<" "<<t<<std::endl;
  // for(auto m: mapping){
  //   auto pos = m.second;
  // //  std::cout<<pos.toStr()<<",";
  //   ts_mapping_[getOffset(pos.x,pos.y,pos.t)] = m.first;
  // }

  mapping_ = mapping;

  // LOG(LISAGNN)<<simpleSchedToString();

}

std::string LISASchedule::simpleSchedToString(){
  // std::cout<<"mapping "<<mapping_.size();
  std::stringstream output;
  output<<"\n";
  // for(int t = 0; t< t_; t++){
  //   bool print_t = false;
  //   bool has_node = false;
  //   for(int i =0; i<x_; i++){
  //     for(int j =0; j<y_; j++){
  //       int node_id = ts_mapping_[getOffset(i,j,t)];
  //       if(node_id != -1) {
  //         if( !print_t) output<<"t :"<<t<<"\n";
  //         output<<" <"<<i<<","<<j<<"> ->"<<node_id<<"\t";
  //         has_node = true;
  //         print_t = true;
  //       }
  //     }
  //   }
  //   if (has_node) output<<"\n";
  // }
   for(int t = 0; t< t_; t++){
     std::vector<std::pair<int, pos3d>> mapped_node;
     for(auto m: mapping_){
       if(m.second.t == t){
         mapped_node.push_back(std::make_pair(m.first, m.second));
       }
     }
     if(mapped_node.size()!=0){
        std::sort (mapped_node.begin(), mapped_node.end(), 
        [this](std::pair<int, pos3d> a, std::pair<int, pos3d> b) {
          return  a.second.x  + a.second.y <  b.second.x  + b.second.y ;
        });
        output<<"t :"<<t<<"\n";
        for(auto node: mapped_node){
           output<<" <"<<node.second.x<<","<<node.second.y<<"> ->"<<node.first<<"\t";
        }
        output<<"\n";
     }
    
    
   }
  return output.str();
}

std::string LISASchedule::complexSchedToString(){
  std::stringstream output;
  
  // for(int t = 0; t< t_; t++){
  //   output<<"t :"<<t<<"\n";
  //   for(int i =0; i<x_; i++){
  //     output<<"\t";
  //     for(int j =0; j<y_; j++){
  //       int node_id = ts_mapping_[getOffset(i,j,t)];
  //       if(node_id == -1){
  //         output<<"_\t";
  //       }else{
  //         output<<node_id<<"\t";
  //       }
        
  //     }
  //     output<<"\n";
  //   }
  //   output<<"\n\n";
  // }
  return output.str();
}


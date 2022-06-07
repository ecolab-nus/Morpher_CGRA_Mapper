#include <ctime>
#include <set>
#include <map>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <unordered_set>
#include <sstream>
#include <tuple> 
#include <vector> 
#include <algorithm> 
#include "../debug.h"
#include "LISADFG.h"




#ifndef ___LISASCHEDULE_H__
#define ___LISASCHEDULE_H__

struct pos3d{
  int x;
  int y;
  int t;

  std::string toStr(){
    return "("+std::to_string(x)+","+std::to_string(y)+","+std::to_string(t)+")";
  }
};
class LISASchedule{
  friend class LISAController;
  public:
    LISASchedule(LISADFG* dfg, int x, int y, int t, std::map<int, pos3d> & mapping);

    int getOffset(int x, int y, int t){
      return t*x_*y_ + x * y_ + y;
    }

    std::string simpleSchedToString();
    std::string complexSchedToString();
  private:
    int x_;
    int y_;
    int t_;
    // int *ts_mapping_ ; //temporal_spatial_mapping
    std::map<int, pos3d> mapping_;
    LISADFG* dfg_;
};
#endif
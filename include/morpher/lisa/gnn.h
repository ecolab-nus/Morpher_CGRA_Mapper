#include <ctime>
#include <set>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <unordered_set>

#include <morpher/util/debug.h>





#ifndef ___GNN_H__
#define ___GNN_H__





static std::string gen_random(const int len)
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

class GNN{
  public:
    GNN();

    void inference();

    void dump_dfg();
    void dump_feature();


    void call_gnn();


  private:
    std::string dfg_name_;
    
   
};


#endif

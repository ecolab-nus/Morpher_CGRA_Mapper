/* used for abstract mapping
abstract MRRG architecture on HyCUBE */

#ifndef AMRRG_H_
#define AMRRG_H_

#include <vector>
#include "DFG.h"
#include "assert.h"
#include <algorithm>
#include <iostream>
#include <map>
#include "CGRA.h"
#include "MRRGNode.h"
#include "sstream"
#include <queue>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace CGRAXMLCompile
{

    struct DestCost
    {
        MRRGNode *pe;
        int cost;

        DestCost(MRRGNode *pe, int cost) : pe(pe), cost(cost) {}

        bool operator<(const DestCost &rhs) const
        {
            return this->cost > rhs.cost; // min heap
        }
    };

    struct MappedPath
    {
        int commCycle;
        DFGNode *srcDFGNode;
        DFGNode *destDFGNode;

        MRRGNode *srcMRRGNode;
        MRRGNode *destMRRGNode;
        vector<pair<MRRGNode *, Direction>> path;

        MappedPath(DFGNode *sd, DFGNode *dd, MRRGNode *sm, MRRGNode *dm)
            : srcDFGNode(sd), destDFGNode(dd), srcMRRGNode(sm), destMRRGNode(dm) {}
    };

    struct abstractArgs
    {
        string CGRAType;
        int minII;
        int maxII;
        int curII;
        int x_dim;
        int y_dim;
        DFG *dfg;
        CGRA *cgra;

        abstractArgs(string CGRAType, int minII, int maxII, int x_dim, int y_dim, DFG *dfg, CGRA *cgra)
            : CGRAType(CGRAType), minII(minII), maxII(maxII), x_dim(x_dim), y_dim(y_dim), dfg(dfg), cgra(cgra) {}
    };

    class AMRRG
    {

    public:
        // parameters
        DFG *dfg;
        map<int, map<int, map<int, MRRGNode *>>> MRRG; // t x y
        vector<DFGNode *> topoOrder;
        json jmap; // output the mapping results to json file

        AMRRG(int ii_len, int x_dim, int y_dim, DFG *dfg);
        ~AMRRG();

        void getTopoOrder();
        void printTopoOrder();
        int get2DDistance(int x1, int y1, int x2, int y2);
        int get3DDistance(int t1, int x1, int y1, int t2, int x2, int y2);
        void printCandidatePE(DFGNode *node, priority_queue<DestCost> candidatePEs);
        void printRoutingInfo(vector<MappedPath> &paths);

        void mapNodesWithNoParents(DFGNode *node);
        // node is the DFG node to be mapped
        int calculateCost(DFGNode *node, MRRGNode *candidatePE);
        bool isIdleLocalLink(DFGNode *parent, MRRGNode *pe, Direction dir);
        void preDijkstra(int t);
        MRRGNode *minDistance(int t);
        void findParents(MRRGNode *dst, vector<pair<MRRGNode *, Direction>> &path);
        void updatePathState(MappedPath &mpath);
        void updatePathsState(vector<MappedPath> &paths);
        void emptyPathState(MappedPath &mpath);
        void emptyPathsState(vector<MappedPath> &paths);
        /* src --> dst, using the dijkstra algorithm
        node is the source DFG node of the packet */
        bool existPathToOnePE(DFGNode *parent, MRRGNode *src, MRRGNode *dst, int &commCycle,
                              vector<pair<MRRGNode *, Direction>> &path);
        // node is the DFG node to be mapped
        bool isReachableToAllMappedNbrs(DestCost dc, DFGNode *node, vector<MappedPath> &paths);
        // node is the DFG node to be mapped
        bool findValidPE(DFGNode *node, MRRGNode *targetPE, vector<MappedPath> &paths);
        bool abstractMap(DFG *dfg, string dfgAddr);
        
        int get_x_dim() { return x_dim; }
        int get_y_dim() { return y_dim; }
        int get_ii() { return ii_len; }

    private:
        int ii_len; // initiation interval
        int x_dim;
        int y_dim;

    }; // class AMRRG

} // namespace CGRAXMLCompile

#endif

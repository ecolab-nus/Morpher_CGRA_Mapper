/* 
used for abstract mapping
a node in MRRG
 */

#ifndef MRRGNODE_H_
#define MRRGNODE_H_

#include "DFGNode.h"

//the following are terminal color codes
#define RESET "\033[0m"
#define BLACK "\033[30m"              /* Black */
#define RED "\033[31m"                /* Red */
#define GREEN "\033[32m"              /* Green */
#define YELLOW "\033[33m"             /* Yellow */
#define BLUE "\033[34m"               /* Blue */
#define MAGENTA "\033[35m"            /* Magenta */
#define CYAN "\033[36m"               /* Cyan */
#define WHITE "\033[37m"              /* White */
#define BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED "\033[1m\033[31m"     /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"    /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"   /* Bold White */

namespace CGRAXMLCompile
{

    enum Direction
    {
        NORTH,
        SOUTH,
        EAST,
        WEST,
        LOCAL
    };

    class MRRGNode
    {
    public:
        //parameters
        int t; //cycle index in AMRRG
        int x; //row in PE array
        int y; //column in PE array

        std::vector<MRRGNode *> nextMRRGNodes;                      //logically single-cycle connection
        std::vector<std::pair<MRRGNode *, Direction>> adjMRRGNodes; //physically one-hop connection

        bool isMemPE; //false: general PE; true: memory PE
        /* the availability of current PE and the assigned DFGNode */
        std::pair<bool, DFGNode *> ALU;

        /* the availability of the input and output ports
        true: not available; false: available 
        the port is occupied by the packet from DFGNode */
        std::pair<bool, DFGNode *> westInputPort;
        std::pair<bool, DFGNode *> eastInputPort;
        std::pair<bool, DFGNode *> northInputPort;
        std::pair<bool, DFGNode *> southInputPort;

        //used for dijkstra algorithm
        bool isVisited;
        int distance;
        std::pair<MRRGNode *, Direction> parent; //previous PE + input port of current PE

        MRRGNode(int t, int x, int y);
        void setResource(std::pair<bool, DFGNode *> *res);

        MRRGNode &operator=(const MRRGNode &pe)
        {
            if (this != &pe)
            {
                t = pe.t;
                x = pe.x;
                y = pe.y;
            }
            return *this;
        }

    private:
    };

}

#endif
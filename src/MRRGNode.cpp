#include "MRRGNode.h"


void CGRAXMLCompile::MRRGNode::setResource(std::pair<bool, DFGNode *> *res)
{
    res->first = false;
    res->second = NULL;
}


CGRAXMLCompile::MRRGNode::MRRGNode(int t, int x, int y)
{
    this->t = t;
    this->x = x;
    this->y = y;

    setResource(&ALU);
    setResource(&northInputPort);
    setResource(&southInputPort);
    setResource(&westInputPort);
    setResource(&eastInputPort);
    
}



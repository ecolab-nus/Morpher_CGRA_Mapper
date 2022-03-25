#include "AMRRG.h"

int CGRAXMLCompile::AMRRG::get2DDistance(int x1, int y1, int x2, int y2)
{
    return abs(x1 - x2) + abs(y1 - y2);
}

int CGRAXMLCompile::AMRRG::get3DDistance(int t1, int x1, int y1, int t2, int x2, int y2)
{
    return abs(t1 - t2) + abs(x1 - x2) + abs(y1 - y2);
}

CGRAXMLCompile::AMRRG::AMRRG(int ii_len, int x_dim, int y_dim, DFG *dfg)
{
    this->ii_len = ii_len;
    this->x_dim = x_dim;
    this->y_dim = y_dim;
    this->dfg = dfg;

    // create MRRG nodes
    std::cout << "start creating MRRG node...\n";
    for (int t = 0; t < ii_len; ++t)
    {
        for (int x = 0; x < x_dim; ++x)
        {
            for (int y = 0; y < y_dim; ++y)
            {
                MRRG[t][x][y] = new MRRGNode(t, x, y);
                if (x == 0) // most left side
                {
                    MRRG[t][x][y]->isMemPE = true;
                }
                else
                {
                    MRRG[t][x][y]->isMemPE = false;
                }
            }
        }
    }
    std::cout << "finish creation of MRRG nodes...\n";

    // create single-cycle connections between MRRG nodes (logically)
    for (int t = 0; t < ii_len; ++t)
    {
        for (int x = 0; x < x_dim; ++x)
        {
            for (int y = 0; y < y_dim; ++y)
            {
                for (int x1 = 0; x1 < x_dim; ++x1)
                {
                    for (int y1 = 0; y1 < y_dim; ++y1)
                    {
                        MRRG[t][x][y]->nextMRRGNodes.push_back(MRRG[(t + 1) % ii_len][x1][y1]);
                    }
                }
            }
        }
    }

    // create one-hop connections between MRRG nodes (physically)
    pair<MRRGNode *, Direction> nextPE;
    for (int t = 0; t < ii_len; ++t)
    {
        for (int x = 0; x < x_dim; ++x)
        {
            for (int y = 0; y < y_dim; ++y)
            {
                if (x - 1 >= 0) // left MRRG node (x-1, y)
                {
                    // current PE + input port of current PE
                    nextPE = make_pair(MRRG[t][x - 1][y], EAST);
                    MRRG[t][x][y]->adjMRRGNodes.push_back(nextPE);
                }
                if (x + 1 < x_dim) // right MRRG node (x+1, y)
                {
                    nextPE = make_pair(MRRG[t][x + 1][y], WEST);
                    MRRG[t][x][y]->adjMRRGNodes.push_back(nextPE);
                }
                if (y - 1 >= 0) // bottom MRRG node (x, y-1)
                {
                    nextPE = make_pair(MRRG[t][x][y - 1], NORTH);
                    MRRG[t][x][y]->adjMRRGNodes.push_back(nextPE);
                }
                if (y + 1 < y_dim) // top MRRG node (x, y+1)
                {
                    nextPE = make_pair(MRRG[t][x][y + 1], SOUTH);
                    MRRG[t][x][y]->adjMRRGNodes.push_back(nextPE);
                }
            }
        }
    }

    std::cout << "finish connection creation between MRRG nodes...\n";
}

CGRAXMLCompile::AMRRG::~AMRRG()
{
    std::cout << "delete the MRRG PE nodes...\n";
    for (int t = 0; t < ii_len; ++t)
    {
        for (int x = 0; x < x_dim; ++x)
        {
            for (int y = 0; y < y_dim; ++y)
            {
                delete MRRG[t][x][y];
            }
        }
    }
}

void CGRAXMLCompile::AMRRG::getTopoOrder()
{
    topoOrder.clear();
    map<int, vector<DFGNode *>> asapLevelNodeList;
    for (DFGNode &node : dfg->nodeList)
    {
        asapLevelNodeList[node.ASAP].push_back(&node);
    }

    int maxASAPLevel = 0;
    for (auto pair : asapLevelNodeList)
    {
        if (pair.first > maxASAPLevel)
        {
            maxASAPLevel = pair.first;
        }
    }

    for (int i = 0; i <= maxASAPLevel; ++i)
    {
        for (DFGNode *node : asapLevelNodeList[i])
        {
            topoOrder.push_back(node);
        }
    }
    reverse(topoOrder.begin(), topoOrder.end());
}

// this part is from sortBackEdgePriorityASAP() in PathFinderMapper.cpp
void CGRAXMLCompile::AMRRG::getTopoOrderBackEdge()
{
    topoOrder.clear();
    struct BEDist
    {
        DFGNode *parent;
        DFGNode *child;
        int dist;
        BEDist(DFGNode *parent, DFGNode *child, int dist) : parent(parent), child(child), dist(dist) {}
        bool operator<(const BEDist &other) const
        {
            if (dist == other.dist)
            {
                return true;
            }
            return dist > other.dist;
        }
    };

    std::set<BEDist> backedges;
    for (DFGNode &node : dfg->nodeList)
    {
        if (node.idx == 97)
        {
            std::cout << "node_idx:97,node_ASAP:" << node.ASAP << "\n";
        }
        for (DFGNode *child : node.children)
        {

            if (node.idx == 97)
            {
                std::cout << "child_idx:" << child->idx << "child_ASAP:" << child->ASAP << "\n";
            }
            if (child->ASAP <= node.ASAP)
            {
                std::cout << "inserting for : node=" << node.idx << ",child:" << child->idx << "\n";
                backedges.insert(BEDist(&node, child, node.ASAP - child->ASAP));
            }
        }
    }

    // populate reccycles
    std::cout << "Populate Rec Cycles!\n";
    RecCycles.clear();
    // exit(0);
    for (BEDist be : backedges)
    {
        std::vector<DFGNode *> backedgePathVec = dfg->getAncestoryASAP(be.parent);
        std::cout << "REC_CYCLE :: BE_Parent = " << be.parent->idx << "\n";
        std::cout << "REC_CYCLE :: BE_Child = " << be.child->idx << "\n";
        std::cout << "REC_CYCLE :: BE_Parent's ancesotry : \n";
        for (DFGNode *n : backedgePathVec)
        {
            if (RecCycles[BackEdge(be.parent, be.child)].find(n) == RecCycles[BackEdge(be.parent, be.child)].end())
            {
                std::cout << n->idx << ",";
            }
            RecCycles[BackEdge(be.parent, be.child)].insert(n);
        }
        std::cout << "REC_CYCLE :: Done!\n";
    }

    RecCyclesLS.clear();
    for (DFGNode &node : dfg->nodeList)
    {
        for (DFGNode *recParent : node.recParents)
        {
            BEDist be_temp(&node, recParent, node.ASAP - recParent->ASAP);

            std::vector<DFGNode *> backedgePathVec = dfg->getAncestoryASAP(be_temp.parent);

            std::cout << "REC_CYCLELS :: BE_Parent = " << be_temp.parent->idx << "\n";
            std::cout << "REC_CYCLELS :: BE_Child = " << be_temp.child->idx << "\n";
            std::cout << "REC_CYCLELS :: BE_Parent's ancesotry : \n";
            for (DFGNode *n : backedgePathVec)
            {
                if (n == be_temp.parent)
                    continue;
                if (RecCyclesLS[BackEdge(be_temp.parent, be_temp.child)].find(n) == RecCyclesLS[BackEdge(be_temp.parent, be_temp.child)].end())
                {
                    std::cout << n->idx << ",";
                }
                RecCyclesLS[BackEdge(be_temp.parent, be_temp.child)].insert(n);
            }
            std::cout << "REC_CYCLELS :: Done!\n";

            backedges.insert(be_temp);
        }
    }

    std::map<DFGNode *, std::vector<DFGNode *>> beparentAncestors;
    std::map<DFGNode *, std::vector<DFGNode *>> bechildAncestors;
    std::map<std::pair<DFGNode *, DFGNode *>, bool> trueBackedges;

    for (BEDist be : backedges)
    {
        std::cout << "BE PARENT = " << be.parent->idx << ",dist=" << be.dist << "\n";
        std::cout << "BE CHILD = " << be.child->idx << "\n";

        std::cout << "Ancestory : \n";
        beparentAncestors[be.parent] = dfg->getAncestoryASAP(be.parent);
        bechildAncestors[be.child] = dfg->getAncestoryASAP(be.child);
        std::cout << "\n";

        if (std::find(beparentAncestors[be.parent].begin(),
                      beparentAncestors[be.parent].end(),
                      be.child) == beparentAncestors[be.parent].end())
        {
            std::cout << "BE CHILD does not belong BE Parent's Ancestory\n";

            // Hack to force all backedges to be true backedges
            trueBackedges[std::make_pair(be.parent, be.child)] = false;
        }
        else
        {
            // change this be.parent if PHI nodes are not removed
            std::cout << "RecPHI inserted : " << be.child->idx << "\n";
            trueBackedges[std::make_pair(be.parent, be.child)] = false;
        }
    }

    std::map<DFGNode *, std::set<DFGNode *>> superiorChildren;

    //	std::vector<DFGNode*> mergedAncestory;
    std::map<DFGNode *, std::vector<DFGNode *>> mergedAncestories;
    mergedAncestories.clear();
    std::map<DFGNode *, DFGNode *> mergedKeys;
    for (BEDist be : backedges)
    {
        // write a logic to merge ancestories where if one be's child is present in some other be's parent's ancesotory'
        bool merged = false;
        for (std::pair<DFGNode *, std::vector<DFGNode *>> pair : mergedAncestories)
        {
            DFGNode *key = pair.first;
            if (std::find(mergedAncestories[key].begin(), mergedAncestories[key].end(), be.child) != mergedAncestories[key].end())
            {

                if (trueBackedges[std::make_pair(be.parent, be.child)] == true)
                {
                    superiorChildren[key].insert(be.child);
                }

                std::cout << "Merging :: " << key->idx << ", " << be.parent->idx << "\n";
                mergedAncestories[key] = dfg->mergeAncestoryASAP(mergedAncestories[key], beparentAncestors[be.parent], RecCycles);
                merged = true;
                std::cout << "Merging Done :: " << key->idx << ", " << be.parent->idx << "\n";
                mergedKeys[be.parent] = key;
                //				break;
            }
        }
        if (!merged)
        {
            mergedAncestories[be.parent] = dfg->getAncestoryASAP(be.parent);
            mergedKeys[be.parent] = be.parent;

            if (trueBackedges[std::make_pair(be.parent, be.child)] == true)
            {
                superiorChildren[be.parent].insert(be.child);
            }
        }
    }

    for (BEDist be : backedges)
    {
        std::vector<DFGNode *> mergedSuperiorChildren;
        for (DFGNode *sChild : superiorChildren[mergedKeys[be.parent]])
        {
            mergedSuperiorChildren = dfg->mergeAncestoryASAP(mergedSuperiorChildren, bechildAncestors[sChild], RecCycles);
        }

        for (DFGNode *ancestorNode : mergedSuperiorChildren)
        {
            if (std::find(topoOrder.begin(), topoOrder.end(), ancestorNode) == topoOrder.end())
            {
                topoOrder.push_back(ancestorNode);
            }
        }

        for (DFGNode *ancestorNode : mergedAncestories[mergedKeys[be.parent]])
        {
            if (std::find(topoOrder.begin(), topoOrder.end(), ancestorNode) == topoOrder.end())
            {
                topoOrder.push_back(ancestorNode);
            }
        }
    }

    for (BEDist be : backedges)
    {
        assert(mergedKeys.find(be.parent) != mergedKeys.end());
        std::vector<DFGNode *> ancestoryNodes = mergedAncestories[mergedKeys[be.parent]];
        for (DFGNode *ancestorNode : ancestoryNodes)
        {
            if (std::find(topoOrder.begin(), topoOrder.end(), ancestorNode) == topoOrder.end())
            {
                topoOrder.push_back(ancestorNode);
            }
        }
        std::cout << "BE PARENT = " << be.parent->idx << ",dist=" << be.dist << "\n";
        if (std::find(topoOrder.begin(), topoOrder.end(), be.parent) == topoOrder.end())
        {
            topoOrder.push_back(be.parent);
        }

        ancestoryNodes = dfg->getAncestoryASAP(be.child);
        for (DFGNode *ancestorNode : ancestoryNodes)
        {
            if (std::find(topoOrder.begin(), topoOrder.end(), ancestorNode) == topoOrder.end())
            {
                topoOrder.push_back(ancestorNode);
            }
        }
        std::cout << "BE CHILD = " << be.child->idx << "\n";
        if (std::find(topoOrder.begin(), topoOrder.end(), be.child) == topoOrder.end())
        {
            topoOrder.push_back(be.child);
        }
    }

    std::map<int, std::vector<DFGNode *>> asapLevelNodeList;
    for (DFGNode &node : dfg->nodeList)
    {
        asapLevelNodeList[node.ASAP].push_back(&node);
    }

    int maxASAPlevel = 0;
    for (std::pair<int, std::vector<DFGNode *>> pair : asapLevelNodeList)
    {
        if (pair.first > maxASAPlevel)
        {
            maxASAPlevel = pair.first;
        }
    }

    for (int i = 0; i <= maxASAPlevel; ++i)
    {
        for (DFGNode *node : asapLevelNodeList[i])
        {
            if (std::find(topoOrder.begin(), topoOrder.end(), node) == topoOrder.end())
            {
                topoOrder.push_back(node);
            }
        }
    }

    std::cout << "***********SORTED LIST*******************\n";
    for (DFGNode *node : topoOrder)
    {
        std::cout << "Node=" << node->idx << ",ASAP=" << node->ASAP << "\n";
    }
    std::reverse(topoOrder.begin(), topoOrder.end());
}

void CGRAXMLCompile::AMRRG::printTopoOrder()
{
    std::cout << BOLDRED << "Reversal topological order is:" << RESET << endl;
    for (DFGNode *node : topoOrder)
    {
        std::cout << node->idx << ", |Parents=";
        for (DFGNode *parent : node->parents)
        {
            std::cout << parent->idx << ", ";
        }
        std::cout << ", |Children=";
        for (DFGNode *child : node->children)
        {
            std::cout << child->idx << ", ";
        }
        std::cout << "\n";
    }
    std::cout << "\n";
}

void CGRAXMLCompile::AMRRG::mapNodesWithNoParents(DFGNode *node)
{
    if (node->isMemOp() == true) // memory DFG node, mapped on the leftmost side
    {
        for (int t = 0; t < ii_len; ++t)
        {
            for (int y = 0; y < y_dim; ++y)
            {
                if (MRRG[t][0][y]->ALU.first == false) // available
                {
                    MRRG[t][0][y]->ALU = make_pair(true, node); // reset the PE allocation state
                    node->setMappedState(true, t, 0, y);        // reset the DFG node state
                    std::cout << BOLDMAGENTA << "Map the DFG node ->" << node->idx
                              << " on MRRG(" << t << ",0," << y << ")..." << RESET << endl;

                    // jmap["DFG_node_" + to_string(node->idx)]["mapped_PE"] =
                    //     "PE(" + to_string(t) + ",0," + to_string(y) + ")";
                    jmap["DFG_node_" + to_string(node->idx)]["mapped_PE"]["t"] = t;
                    jmap["DFG_node_" + to_string(node->idx)]["mapped_PE"]["x"] = 0;
                    jmap["DFG_node_" + to_string(node->idx)]["mapped_PE"]["y"] = y;
                    jmap["DFG_node_" + to_string(node->idx)]["is_MemOp"] = "true";
                    return;
                }
            }
        }
    }
    else // non-memory DFG node, mapped from the rightmost first
    {
        for (int t = 0; t < ii_len; ++t)
        {
            for (int x = x_dim - 1; x >= 0; --x)
            {
                for (int y = 0; y < y_dim; ++y)
                {
                    if (MRRG[t][x][y]->ALU.first == false) // available
                    {
                        MRRG[t][x][y]->ALU = make_pair(true, node); // reset the PE allocation state
                        node->setMappedState(true, t, x, y);        // reset the DFG node state
                        std::cout << BOLDMAGENTA << "Map the DFG node ->" << node->idx
                                  << " on MRRG(" << t << ", " << x << ", " << y << ")..." << RESET << endl;

                        // jmap["DFG_node_" + to_string(node->idx)]["mapped_PE"] =
                        //     "PE(" + to_string(t) + "," + to_string(x) + "," + to_string(y) + ")";
                        jmap["DFG_node_" + to_string(node->idx)]["mapped_PE"]["t"] = t;
                        jmap["DFG_node_" + to_string(node->idx)]["mapped_PE"]["x"] = x;
                        jmap["DFG_node_" + to_string(node->idx)]["mapped_PE"]["y"] = y;

                        jmap["DFG_node_" + to_string(node->idx)]["is_MemOp"] = "false";
                        return;
                    }
                }
            }
        }
    }
}

int CGRAXMLCompile::AMRRG::calculateCost(DFGNode *node, MRRGNode *candidatePE)
{
    int totalCost = 0;

    // for mapped parents
    for (DFGNode *parent : node->parents)
    {
        if (parent->isMapped)
        {
            int timeCost;
            if (candidatePE->t == parent->t)
            {
                // timeCost = (node->ASAP - parent->ASAP) * ii_len;
                timeCost = ii_len;
                // timeCost = 0;
            }
            else if (candidatePE->t < parent->t)
            {
                timeCost = candidatePE->t + ii_len - parent->t;
            }
            else // candidatePE->t > parent->t
            {
                timeCost = candidatePE->t - parent->t;
            }
            timeCost *= (x_dim + y_dim - 2);

            int distCost = abs(candidatePE->x - parent->x) + abs(candidatePE->y - parent->y);
            if (!node->isMemOp() && candidatePE->x == 0)
            {
                distCost += x_dim;
            }
            totalCost += (timeCost + distCost);
        }
    }

    // for mapped children, exist back edge and thus some children can be previously mapped
    for (DFGNode *child : node->children)
    {
        if (child->isMapped)
        {
            int timeCost;
            if (candidatePE->t == child->t)
            {
                // timeCost = (node->ASAP - child->ASAP) * ii_len;
                timeCost = ii_len;
                // timeCost = 0;
            }
            else if (candidatePE->t < child->t)
            {
                timeCost = child->t - candidatePE->t;
            }
            else // candidatePE->t > child->t
            {
                timeCost = child->t + ii_len - candidatePE->t;
            }
            timeCost *= (x_dim + y_dim - 2);

            int distCost = abs(candidatePE->x - child->x) + abs(candidatePE->y - child->y);
            if (!node->isMemOp() && candidatePE->x == 0)
            {
                distCost += x_dim;
            }
            totalCost += (4 * timeCost + distCost);
        }
    }
    return totalCost;
}

// parent of current DFG node + current PE + input port of current PE
bool CGRAXMLCompile::AMRRG::isIdleLocalLink(DFGNode *parent, MRRGNode *pe, Direction dir)
{
    bool isIdle = false;
    switch (dir)
    {
    case NORTH:
        if (pe->northInputPort.first == false) // available
        {
            isIdle = true;
            std::cout << "[" << pe->t << "][" << pe->x << "][" << pe->y << "]"
                      << " NORTH link is idle ...\n";
        }
        else
        {
            if (parent->isMapped && parent->idx == pe->northInputPort.second->idx) // with the same source
            {
                isIdle = true;
                std::cout << "[" << pe->t << "][" << pe->x << "][" << pe->y << "]"
                          << " NORTH link can be shared with the same source = " << parent->idx << "\n";
            }
            else
            {
                isIdle = false;
                std::cout << "[" << pe->t << "][" << pe->x << "][" << pe->y << "]"
                          << " NORTH link is not idle ...\n";
            }
        }
        break;

    case SOUTH:
        if (pe->southInputPort.first == false)
        {
            isIdle = true;
            std::cout << "[" << pe->t << "][" << pe->x << "][" << pe->y << "]"
                      << " SOUTH link is idle ...\n";
        }
        else
        {
            if (parent->isMapped && parent->idx == pe->southInputPort.second->idx) // with the same source
            {
                isIdle = true;
                std::cout << "[" << pe->t << "][" << pe->x << "][" << pe->y << "]"
                          << " SOUTH link can be shared with the same source = " << parent->idx << "\n";
            }
            else
            {
                isIdle = false;
                std::cout << "[" << pe->t << "][" << pe->x << "][" << pe->y << "]"
                          << " SOUTH link is not idle ...\n";
            }
        }
        break;

    case EAST:
        if (pe->eastInputPort.first == false)
        {
            isIdle = true;
            std::cout << "[" << pe->t << "][" << pe->x << "][" << pe->y << "]"
                      << " EAST link is idle ...\n";
        }
        else
        {
            if (parent->isMapped && parent->idx == pe->eastInputPort.second->idx) // with the same source
            {
                isIdle = true;
                std::cout << "[" << pe->t << "][" << pe->x << "][" << pe->y << "]"
                          << " EAST link can be shared with the same source = " << parent->idx << "\n";
            }
            else
            {
                isIdle = false;
                std::cout << "[" << pe->t << "][" << pe->x << "][" << pe->y << "]"
                          << " EAST link is not idle ...\n";
            }
        }
        break;

    case WEST:
        if (pe->westInputPort.first == false)
        {
            isIdle = true;
            std::cout << "[" << pe->t << "][" << pe->x << "][" << pe->y << "]"
                      << " WEST link is idle ...\n";
        }
        else
        {
            if (parent->isMapped && parent->idx == pe->westInputPort.second->idx) // with the same source
            {
                isIdle = true;
                std::cout << "[" << pe->t << "][" << pe->x << "][" << pe->y << "]"
                          << " WEST link can be shared with the same source = " << parent->idx << "\n";
            }
            else
            {
                isIdle = false;
                std::cout << "[" << pe->t << "][" << pe->x << "][" << pe->y << "]"
                          << " WEST link is not idle ...\n";
            }
        }
        break;

    default:
        break;
    }
    return isIdle;
}

void CGRAXMLCompile::AMRRG::preDijkstra(int t)
{
    for (int x = 0; x < x_dim; ++x)
    {
        for (int y = 0; y < y_dim; ++y)
        {
            MRRG[t][x][y]->isVisited = false;
            MRRG[t][x][y]->distance = INT32_MAX;
            MRRG[t][x][y]->parent.first = NULL;
        }
    }
}

// get the PEs from the second last PE to src PE (not including dest PE)
void CGRAXMLCompile::AMRRG::findParents(MRRGNode *dst, vector<pair<MRRGNode *, Direction>> &path)
{
    if (dst->parent.first == NULL)
    {
        return;
    }
    path.push_back(dst->parent);
    findParents(dst->parent.first, path);
}

void CGRAXMLCompile::AMRRG::printRoutingInfo(vector<MappedPath> &paths)
{
    for (auto it = paths.begin(); it != paths.end(); ++it)
    {
        reverse(it->path.begin(), it->path.end());
        string src = "DFG_node_" + to_string(it->srcDFGNode->idx);
        string dst = "to DFG_node_" + to_string(it->destDFGNode->idx);
        string route = "";
        for (auto pe : it->path)
        {
            int t = pe.first->t, x = pe.first->x, y = pe.first->y;
            route += "(" + to_string(t) + "," + to_string(x) + "," + to_string(y) + ",";
            switch (pe.second)
            {
            case NORTH:
                route += "SOUTH";
                break;
            case SOUTH:
                route += "NORTH";
                break;
            case WEST:
                route += "EAST";
                break;
            case EAST:
                route += "WEST";
                break;
            default:
                break;
            }
            route += ")->";
        }
        if (!it->path.empty())
        {
            int t = it->commCycle, x = it->destMRRGNode->x, y = it->destMRRGNode->y;
            route += "(" + to_string(t) + "," + to_string(x) + "," + to_string(y) + ")";
        }
        else
        {
            route = "no route information: mapped to the PE of parent!";
        }
        jmap[src][dst]["routing"] = route;
    }
}

void CGRAXMLCompile::AMRRG::emptyPathState(MappedPath &mpath)
{
    MRRGNode *next = MRRG[mpath.commCycle][mpath.destMRRGNode->x][mpath.destMRRGNode->y];
    for (auto pe : mpath.path)
    {
        switch (pe.second)
        {
        case NORTH:
            next->northInputPort = make_pair(false, nullptr);
            break;
        case SOUTH:
            next->southInputPort = make_pair(false, nullptr);
            break;
        case WEST:
            next->westInputPort = make_pair(false, nullptr);
            break;
        case EAST:
            next->eastInputPort = make_pair(false, nullptr);
            break;
        default:
            break;
        }
        next = pe.first;
    }
}

void CGRAXMLCompile::AMRRG::emptyPathsState(vector<MappedPath> &paths)
{
    for (MappedPath mp : paths)
    {
        if (mp.path.empty())
        {
            continue;
        }
        emptyPathState(mp);
    }
}

void CGRAXMLCompile::AMRRG::updatePathState(MappedPath &mpath)
{
    if (mpath.path.empty())
    {
        return;
    }

    DFGNode *parent = mpath.srcDFGNode;
    MRRGNode *next = MRRG[mpath.commCycle][mpath.destMRRGNode->x][mpath.destMRRGNode->y];

    for (auto pe : mpath.path)
    {
        switch (pe.second)
        {
        case NORTH:
            next->northInputPort = make_pair(true, parent);
            break;
        case SOUTH:
            next->southInputPort = make_pair(true, parent);
            break;
        case WEST:
            next->westInputPort = make_pair(true, parent);
            break;
        case EAST:
            next->eastInputPort = make_pair(true, parent);
            break;
        default:
            break;
        }
        next = pe.first;
    }
}

void CGRAXMLCompile::AMRRG::updatePathsState(vector<MappedPath> &paths)
{
    for (MappedPath mp : paths)
    {
        if (mp.path.empty())
        {
            continue;
        }
        updatePathState(mp);
    }
}

CGRAXMLCompile::MRRGNode *CGRAXMLCompile::AMRRG::minDistance(int t)
{
    int minDist = INT32_MAX;
    MRRGNode *minPE = NULL;
    for (int x = 0; x < x_dim; ++x)
    {
        for (int y = 0; y < y_dim; ++y)
        {
            if (MRRG[t][x][y]->isVisited == false && MRRG[t][x][y]->distance <= minDist)
            {
                minPE = MRRG[t][x][y];
                minDist = minPE->distance;
            }
        }
    }
    return minPE;
}

// Dijkstra algorith, find a single-cycle src-dst path at the cycle between src and dst
bool CGRAXMLCompile::AMRRG::existPathToOnePE(DFGNode *parent, MRRGNode *src, MRRGNode *dst, int &commCycle,
                                             vector<pair<MRRGNode *, Direction>> &path)
{
    int unavailable = 100;
    int t1 = src->t, x1 = src->x, y1 = src->y;
    int t2 = dst->t, x2 = dst->x, y2 = dst->y;
    if (get2DDistance(x1, y1, x2, y2) == 0) // same physical PE in different cycle
    {
        return true;
    }

    for (int delta = 0; delta <= abs(t2 - t1); ++delta)
    {
        int cycle = (t1 + delta) % ii_len;
        MRRGNode *midDst = MRRG[cycle][x2][y2];
        preDijkstra(cycle);
        MRRG[cycle][x1][y1]->distance = 0;

        for (int count = 0; count < x_dim * y_dim - 1; ++count)
        {
            MRRGNode *minPE = minDistance(cycle);
            minPE->isVisited = true; // only the PE with finally minimal distance will be marked

            for (auto it = minPE->adjMRRGNodes.begin(); it != minPE->adjMRRGNodes.end(); ++it)
            {
                // link + input port as a whole, parent of current DFG node + current PE + input port of current PE
                int edgeWgt = isIdleLocalLink(parent, it->first, it->second) == true ? 1 : unavailable;
                if (!it->first->isVisited && minPE->distance + edgeWgt < it->first->distance)
                {
                    it->first->distance = minPE->distance + edgeWgt;
                    // reset the parent, including previous PE + input port of current PE
                    it->first->parent = make_pair(minPE, it->second);
                }
            }
        }
        if (midDst->distance < unavailable)
        {
            findParents(midDst, path);
            commCycle = cycle;
            std::cout << "In function existPathToOnePE: path size = " << path.size() << "\n";
            return true;
        }
    }
    return false;
}

bool CGRAXMLCompile::AMRRG::isReachableToAllMappedNbrs(DestCost dc, DFGNode *node, vector<MappedPath> &paths)
{
    // for mapped parents
    for (DFGNode *parent : node->parents)
    {
        if (parent->isMapped)
        {
            MRRGNode *src = MRRG[parent->t][parent->x][parent->y];
            int commCycle = -1;
            MappedPath route(parent, node, src, dc.pe); // src DFG node + dest DFG node + src PE + dest PE
            if (!existPathToOnePE(parent, src, dc.pe, commCycle, route.path))
            {
                emptyPathsState(paths);
                paths.clear();
                return false;
            }
            else
            {
                route.commCycle = commCycle;
                updatePathState(route);
                paths.push_back(route);
            }
        }
    }

    // for mapped children
    for (DFGNode *child : node->children)
    {
        if (child->isMapped == true)
        {
            MRRGNode *dst = MRRG[child->t][child->x][child->y];
            int commCycle = -1;
            MappedPath route(node, child, dc.pe, dst); // src DFG node + dest DFG node + src PE + dest PE
            if (!existPathToOnePE(node, dc.pe, dst, commCycle, route.path))
            {
                emptyPathsState(paths);
                paths.clear();
                return false;
            }
            else
            {
                route.commCycle = commCycle;
                updatePathState(route);
                paths.push_back(route);
            }
        }
    }
    return true;
}

void CGRAXMLCompile::AMRRG::printCandidatePE(DFGNode *node, priority_queue<DestCost> candidatePEs)
{
    if (candidatePEs.empty())
    {
        std::cout << "The candidate PEs are empty!\n";
        return;
    }

    std::cout << "For DFG node " << node->idx << ", the candidate PEs are as follows: \n";
    while (!candidatePEs.empty())
    {
        DestCost dc(candidatePEs.top().pe, candidatePEs.top().cost);
        std::cout << "PE: (" << dc.pe->t << "," << dc.pe->x << "," << dc.pe->y << ")\t"
                  << "cost: " << candidatePEs.top().cost << "\n";
        candidatePEs.pop();
    }
}

bool CGRAXMLCompile::AMRRG::findValidPE(DFGNode *node, MRRGNode *targetPE, vector<MappedPath> &paths)
{
    priority_queue<DestCost> candidatePEs;

    // get the PE candidates and cost
    if (node->isMemOp()) // for the memory operation nodes
    {
        for (int t = 0; t < ii_len; ++t)
        {
            for (int y = 0; y < y_dim; ++y)
            {
                if (!MRRG[t][0][y]->ALU.first) // available
                {
                    int cost = calculateCost(node, MRRG[t][0][y]);
                    std::cout << "PE: (" << t << ",0," << y << "): "
                              << "calculate cost for Mem Node, cost is " << cost << "\n";
                    if (cost != INT32_MAX)
                    {
                        DestCost dc(MRRG[t][0][y], cost);
                        candidatePEs.push(dc);
                    }
                }
            }
        }
    }
    else // for the non-memory operation nodes
    {
        for (int t = 0; t < ii_len; ++t)
        {
            for (int x = 0; x < x_dim; ++x)
            {
                for (int y = 0; y < y_dim; ++y)
                {
                    if (!MRRG[t][x][y]->ALU.first) // available
                    {
                        int cost = calculateCost(node, MRRG[t][x][y]);
                        std::cout << "PE: (" << t << "," << x << "," << y << "): "
                                  << "calculate cost for Non-Mem Node, cost is " << cost << "\n";
                        if (cost != INT32_MAX)
                        {
                            DestCost dc(MRRG[t][x][y], cost);
                            candidatePEs.push(dc);
                        }
                    }
                }
            }
        }
    }
    printCandidatePE(node, candidatePEs);

    bool found = false;
    // verify the effectiveness of the PE candidate
    while (!candidatePEs.empty())
    {
        DestCost head = candidatePEs.top();
        candidatePEs.pop();
        if (isReachableToAllMappedNbrs(head, node, paths))
        {
            *targetPE = *head.pe;
            found = true;
            break;
        }
    }
    return found;
}

bool CGRAXMLCompile::AMRRG::abstractMap(DFG *dfg, string dfgAddr)
{
    stack<DFGNode *> mappedNodes;
    stack<DFGNode *> unmappedNodes;

    this->dfg = dfg;
    getTopoOrder();
    // getTopoOrderBackEdge();
    for (DFGNode *node : topoOrder)
    {
        unmappedNodes.push(node);
    }

    std::cout << BOLDRED << "Map begin..." << RESET << endl;
    while (!unmappedNodes.empty())
    {
        DFGNode *node = unmappedNodes.top();
        unmappedNodes.pop();

        stringstream output;
        output << "II = " << get_ii();
        output << ", current node = " << node->idx;
        output << ", mapped nodes = " << mappedNodes.size();
        output << ", unmapped nodes = " << unmappedNodes.size();
        output << "\n";
        std::cout << output.str();

        if (node->parents.size() == 0) // has no parents
        {
            mapNodesWithNoParents(node);
            mappedNodes.push(node);
        }
        else // has parents
        {
            vector<MappedPath> paths;
            MRRGNode *validPE = new MRRGNode(0, 0, 0);
            bool foundPE = findValidPE(node, validPE, paths);

            if (foundPE) // find an available PE
            {
                int t = validPE->t, x = validPE->x, y = validPE->y;
                std::cout << BOLDMAGENTA << "Map the DFG node ->" << node->idx
                          << " on MRRG(" << t << ", " << x << ", " << y << ")..." << RESET << endl;

                jmap["DFG_node_" + to_string(node->idx)]["mapped_PE"]["t"] = t;
                jmap["DFG_node_" + to_string(node->idx)]["mapped_PE"]["x"] = x;
                jmap["DFG_node_" + to_string(node->idx)]["mapped_PE"]["y"] = y;
                //    "PE(" + to_string(t) + "," + to_string(x) + "," + to_string(y) + ")";
                jmap["DFG_node_" + to_string(node->idx)]["is_MemOp"] = node->isMemOp() ? "true" : "false";

                node->setMappedState(true, t, x, y);        // set the mapping state of DFG node
                MRRG[t][x][y]->ALU = make_pair(true, node); // set the PE allocation state
                updatePathsState(paths);                    // set the routing state
                printRoutingInfo(paths);                    // print the routing information
                mappedNodes.push(node);
            }
            else
            {
                std::cout << "Map failed...\n";
                return false;
            }
            delete validPE;
        }
    }
    std::cout << BOLDRED << "Map success, final II = " << get_ii() << RESET << endl;
    jmap["II"] = get_ii();

    // string outFile = "../build/src/placement_" + dfgAddr + ".json";
    string outFile = "./placement_" + dfgAddr + ".json";
    ofstream out(outFile);
    assert(out.is_open());
    out << setw(4) << jmap << "\n";

    return true;
}
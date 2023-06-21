/*
 * IVFG.cpp
 *
 *  Created on: Aug 1, 2020
 *      Author: Yuxiang Lei
 */

#include "Graphs/IVFG.h"
#include "Util/SVFUtil.h"
#include <fstream>
#include <iostream>

using namespace SVF;
using namespace SVFUtil;

IVFG::IVFG()
{
    // Specify direct edge kinds
    CFLNode::directEdgeKinds.clear();
    CFLNode::directEdgeKinds.insert(DirectVF);
}

/*!
 * Build VFG for memory objects
 */
void IVFG::build(SVFG* svfg)
{
    // init nodes
    for (SVFG::iterator it = svfg->begin(); it != svfg->end(); ++it)
        addVFGNode(it->first);

    // init edges
    WorkList trivialTree;
    for (iterator it = begin(); it != end(); ++it)
    {
        SVFGNode* svfgNode = svfg->getSVFGNode(it->first);

        // allocate
        if (ActualOUTSVFGNode* retNode =
                SVFUtil::dyn_cast<ActualOUTSVFGNode>(svfgNode))
        {
            const SVFInstruction* fun = retNode->getCallSite()->getCallSite();
            if (SaberCheckerAPI::getCheckerAPI()->isMemAlloc(fun))
            {
                it->second->setSrc();
            }
        }
        if (ActualRetSVFGNode* retNode =
                SVFUtil::dyn_cast<ActualRetSVFGNode>(svfgNode))
        {
            const SVFInstruction* fun = retNode->getCallSite()->getCallSite();
            if (SaberCheckerAPI::getCheckerAPI()->isMemAlloc(fun))
            {
                it->second->setSrc();
            }
        }
        // deallocate
        if (SVFUtil::isa<NullPtrSVFGNode>(svfgNode))
            it->second->setSnk();

        // trivial trees
        if (!svfgNode->hasOutgoingEdge() && !svfgNode->hasIncomingEdge())
            trivialTree.push(it->first);

        for (SVFGEdge* outEdge : svfgNode->getOutEdges())
        {
            NodeID dstId = outEdge->getDstID();
            if (SVFUtil::isa<IntraDirSVFGEdge>(outEdge))
                addEdge(it->first, dstId, DirectVF);
            if (auto callDirEdge = SVFUtil::dyn_cast<CallDirSVFGEdge>(outEdge))
                addEdge(it->first, dstId, CallVF, callDirEdge->getCallSiteId());
            if (auto retDirEdge = SVFUtil::dyn_cast<RetDirSVFGEdge>(outEdge))
                addEdge(it->first, dstId, RetVF, retDirEdge->getCallSiteId());
            if (SVFUtil::isa<IntraIndSVFGEdge>(outEdge))
                addEdge(it->first, dstId, DirectVF);
            if (auto callIndEdge = SVFUtil::dyn_cast<CallIndSVFGEdge>(outEdge))
                addEdge(it->first, dstId, CallVF, callIndEdge->getCallSiteId());
            if (auto retIndEdge = SVFUtil::dyn_cast<RetIndSVFGEdge>(outEdge))
                addEdge(it->first, dstId, RetVF, retIndEdge->getCallSiteId());
            if (SVFUtil::isa<ThreadMHPIndSVFGEdge>(outEdge))
                addEdge(it->first, dstId, DirectVF);
        }
    }

    cleanNodes(trivialTree);
}

void IVFG::copyBuild(const IVFG& rhs)
{
    /// initialize nodes
    for (auto it = rhs.begin(), eit = rhs.end(); it != eit; ++it)
    {
        if (!it->second->hasIncomingEdge() && !it->second->hasOutgoingEdge())
            continue;
        addVFGNode(it->first);
    }

    /// initialize edges
    for (auto edge : rhs.getVFGEdges())
    {
        addEdge(edge->getSrcID(), edge->getDstID(), edge->getEdgeKind(),
                edge->getEdgeIdx());
    }
}

/*!
 * Read from file
 */
void IVFG::readGraph(std::string fname)
{
    std::ifstream gFile;
    gFile.open(fname, std::ios::in);
    if (!gFile.is_open())
    {
        std::cout << "error opening " << fname << std::endl;
        return;
    }

    NodeSet nodeSet;
    std::string line;
    while (getline(gFile, line))
    {
        std::vector<std::string> vec = split(line, '\t');
        if (vec.empty())
            continue;

        NodeID src = stoi(vec[0]);
        NodeID dst = stoi(vec[1]);
        std::string lbl = vec[2];
        addVFGNode(src);
        addVFGNode(dst);

        if (vec[2] == "a")
        {
            addEdge(src, dst, DirectVF);
        }
        if (vec[2] == "call_i")
        {
            addEdge(src, dst, CallVF, std::stoi(vec[3]));
        }
        if (vec[2] == "ret_i")
        {
            addEdge(src, dst, RetVF, std::stoi(vec[3]));
        }

        if (vec[2] == "src")
        {
            CFLNode* node = getVFGNode(src);
            node->setSrc();
        }
    }

    gFile.close();
}

/*!
 *
 */
void IVFG::cleanNodes(WorkList& tree)
{
    // remove trivial trees
    while (!tree.empty())
    {
        NodeID nodeId = tree.pop();
        if (!hasVFGNode(nodeId))
            continue;

        CFLNode* node = getVFGNode(nodeId);
        if (node->hasIncomingEdge() && node->hasOutgoingEdge())
            continue;

        removeVFGNode(node);
    }
}

//@{
bool IVFG::addEdge(NodeID srcId, NodeID dstId, CFLEdge::GEdgeKind k,
                   CallsiteID idx)
{
    CFLNode* src = getVFGNode(srcId);
    CFLNode* dst = getVFGNode(dstId);
    return addEdge(src, dst, k, idx);
}

bool IVFG::addEdge(CFLNode* src, CFLNode* dst, CFLEdge::GEdgeKind k,
                   CallsiteID idx)
{
    if (hasEdge(src, dst, k, idx))
        return false;

    if (k == DirectVF && src == dst)
        return false;

    CFLEdge* edge = new CFLEdge(src, dst, k, idx);
    bool added = addEdge(edge);
    src->addOutEdgeWithKind(edge, k);
    dst->addInEdgeWithKind(edge, k);
    return added;
}
//@}

/*!
 * Re-target dst node of an edge
 *
 * (1) Remove edge from old dst target,
 * (2) Change edge dst id and
 * (3) Add modifed edge into new dst
 */
void IVFG::reTargetDstOfEdge(CFLEdge* edge, CFLNode* newDstNode)
{
    NodeID newDstNodeID = newDstNode->getId();
    NodeID srcId = edge->getSrcID();

    addEdge(srcId, newDstNodeID, edge->getEdgeKind(), edge->getEdgeIdx());

    removeVFGEdge(edge);
}

/*!
 * Re-target src node of an edge
 * (1) Remove edge from old src target,
 * (2) Change edge src id and
 * (3) Add modified edge into new src
 */
void IVFG::reTargetSrcOfEdge(CFLEdge* edge, CFLNode* newSrcNode)
{
    NodeID newSrcNodeID = newSrcNode->getId();
    NodeID dstId = edge->getDstID();

    addEdge(newSrcNodeID, dstId, edge->getEdgeKind(), edge->getEdgeIdx());

    removeVFGEdge(edge);
}

/*!
 * Remove edge from their src and dst edge sets
 */
void IVFG::removeVFGEdge(CFLEdge* edge)
{
    getVFGNode(edge->getSrcID())->removeCFLOutEdge(edge);
    getVFGNode(edge->getDstID())->removeCFLInEdge(edge);
    u32_t num1 = pegEdgeSet.erase(edge);
    //    u32_t num2 = idToEdgeMap.erase(eId);
    delete edge;

    assert(num1 && "edge not in the set, can not remove!!!");
}

/*!
 * Move incoming direct edges of a sub node which is outside SCC to its rep node
 * Remove incoming direct edges of a sub node which is inside SCC from its rep
 * node
 */
bool IVFG::moveInEdgesToRepNode(CFLNode* node, CFLNode* rep)
{
    std::vector<CFLEdge*> sccEdges;
    std::vector<CFLEdge*> nonSccEdges;
    for (CFLNode::const_iterator it = node->InEdgeBegin(),
                                 eit = node->InEdgeEnd();
         it != eit; ++it)
    {
        CFLEdge* subInEdge = *it;
        if (repNodeID(subInEdge->getSrcID()) != rep->getId())
            nonSccEdges.push_back(subInEdge);
        else
        {
            sccEdges.push_back(subInEdge);
        }
    }
    // if this edge is outside scc, then re-target edge dst to rep
    while (!nonSccEdges.empty())
    {
        CFLEdge* edge = nonSccEdges.back();
        nonSccEdges.pop_back();
        reTargetDstOfEdge(edge, rep);
    }

    bool criticalGepInsideSCC = false;
    // if this edge is inside scc, then remove this edge and two end nodes
    while (!sccEdges.empty())
    {
        CFLEdge* edge = sccEdges.back();
        sccEdges.pop_back();
        /// only copy and gep edge can be removed
        reTargetDstOfEdge(edge, rep);
    }
    return criticalGepInsideSCC;
}

/*!
 * Move outgoing direct edges of a sub node which is outside SCC to its rep node
 * Remove outgoing direct edges of a sub node which is inside SCC from its rep
 * node
 */
bool IVFG::moveOutEdgesToRepNode(CFLNode* node, CFLNode* rep)
{
    std::vector<CFLEdge*> sccEdges;
    std::vector<CFLEdge*> nonSccEdges;

    for (CFLNode::const_iterator it = node->OutEdgeBegin(),
                                 eit = node->OutEdgeEnd();
         it != eit; ++it)
    {
        CFLEdge* subOutEdge = *it;
        if (repNodeID(subOutEdge->getDstID()) != rep->getId())
            nonSccEdges.push_back(subOutEdge);
        else
        {
            sccEdges.push_back(subOutEdge);
        }
    }
    // if this edge is outside scc, then re-target edge src to rep
    while (!nonSccEdges.empty())
    {
        CFLEdge* edge = nonSccEdges.back();
        nonSccEdges.pop_back();
        reTargetSrcOfEdge(edge, rep);
    }
    bool criticalGepInsideSCC = false;
    // if this edge is inside scc, then remove this edge and two end nodes
    while (!sccEdges.empty())
    {
        CFLEdge* edge = sccEdges.back();
        sccEdges.pop_back();
        /// only copy and gep edge can be removed
        reTargetSrcOfEdge(edge, rep);
    }
    return criticalGepInsideSCC;
}

void IVFG::mergeNodeToRep(NodeID nodeId, NodeID newRepId)
{
    if (nodeId == newRepId || repNodeID(nodeId) != nodeId)
        return;

    CFLNode* node = getVFGNode(nodeId);
    /// move the edges from node to rep, and remove the node
    moveEdgesToRepNode(node, getVFGNode(newRepId));
    /// set rep and sub relations
    updateNodeRepAndSubs(node->getId(), newRepId);
    removeVFGNode(node);
}

void IVFG::updateNodeRepAndSubs(NodeID nodeId, NodeID newRepId)
{
    setRep(nodeId, newRepId);
    NodeBS repSubs;
    repSubs.set(nodeId);
    /// update nodeToRepMap, for each subs of current node updates its rep to
    /// newRepId
    //  update nodeToSubsMap, union its subs with its rep Subs
    NodeBS& nodeSubs = subNodeIds(nodeId);
    for (NodeID subId : nodeSubs)
        setRep(subId, newRepId);
    repSubs |= nodeSubs;
    setSubs(newRepId, repSubs);
    resetSubs(nodeId);
}

/*!
 *
 */
void IVFG::writeGraph(std::string name)
{
    std::ofstream outFile(name, std::ios::out);
    if (!outFile)
    {
        std::cout << "error opening file!!";
        return;
    }

    std::map<NodeID, NodeID> nodeIdMap;
    std::map<NodeID, NodeID> callsiteIdMap;

    for (auto nIt = begin(); nIt != end(); ++nIt)
    {
        CFLNode* node = nIt->second;
        NodeID src = node->getId();

        for (auto edge : node->getOutEdgeWithTy(DirectVF))
        {
            NodeID dst = edge->getDstID();
            outFile << src << "\t" << dst << "\ta" << std::endl;
        }
        for (auto edge : node->getOutEdgeWithTy(CallVF))
        {
            NodeID dst = edge->getDstID();
            outFile << src << "\t" << dst << "\tcall_i"
                    << "\t" << edge->getEdgeIdx() << std::endl;
        }
        for (auto edge : node->getOutEdgeWithTy(RetVF))
        {
            NodeID dst = edge->getDstID();
            outFile << src << "\t" << dst << "\tret_i"
                    << "\t" << edge->getEdgeIdx() << std::endl;
        }

        if (node->isSrc())
        {
            outFile << src << "\t" << src << "\tsrc" << std::endl;
        }
    }

    outFile.close();
}

/*!
 * GraphTraits specialization for constraint cflData
 */
namespace SVF
{
template <> struct DOTGraphTraits<IVFG*> : public DOTGraphTraits<SVFG*>
{

    typedef CFLNode NodeType;

    DOTGraphTraits(bool isSimple = false) : DOTGraphTraits<SVFG*>(isSimple) {}

    /// Return name of the cflData
    static std::string getGraphName(IVFG* graph)
    {
        return "VFG";
    }

    std::string getNodeLabel(NodeType* node, IVFG* graph)
    {
        return "";
    }

    static std::string getNodeAttributes(NodeType* n, IVFG* graph)
    {
        std::string str;
        std::stringstream rawstr(str);
        rawstr << "";

        return rawstr.str();
    }

    template <class EdgeIter>
    static std::string getEdgeAttributes(NodeType* node, EdgeIter EI,
                                         IVFG* graph)
    {
        std::string str;
        std::stringstream rawstr(str);
        rawstr << "";

        CFLEdge* edge = *(EI.getCurrent());
        if (edge->getEdgeKind() == IVFG::CallVF)
            rawstr << "color=red,label=\"{" << edge->getEdgeIdx() << "}\"";
        else if (edge->getEdgeKind() == IVFG::RetVF)
            rawstr << "color=blue,label=\"{" << edge->getEdgeIdx() << "}\"";
        else
            rawstr << "color=black";
        return rawstr.str();
    }

    template <class EdgeIter>
    static std::string getEdgeLabel(NodeType* node, EdgeIter EI)
    {
        std::string str;
        std::stringstream rawstr(str);
        rawstr << "";

        return rawstr.str();
    }
};
} // namespace SVF

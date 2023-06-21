//===- PEG.cpp -- Program Expression Graph-----------------------------//

/*
 * PEG.cpp
 *
 *  Created on: Nov 22, 2019
 *      Author: Yuxiang Lei
 */

#include "Graphs/PEG.h"
#include <Util/SVFUtil.h>
// #include "CFLBase/CFLOpt.h"
#include <fstream>
#include <iostream>

using namespace SVF;
using namespace SVFUtil;

std::set<CFLEdge::GEdgeKind> CFLNode::directEdgeKinds;

PEG::PEG()
{
    // Specify direct edge kinds
    CFLNode::directEdgeKinds.clear();
    CFLNode::directEdgeKinds.insert(Asgn);
}

/*!
 * Start building constraint cflData
 */
void PEG::build(PAG* p)
{
    // initialize nodes
    for (PAG::iterator it = p->begin(), eit = p->end(); it != eit; ++it)
    {
        if (!it->second->hasIncomingEdge() && !it->second->hasOutgoingEdge())
            continue;
        addPEGNode(it->first);
    }

    // initialize edges
    for (PAGEdge* edge : p->getSVFStmtSet(PAGEdge::Addr))
    {
        auto dEdges = getPEGNode(edge->getSrcID())->getInEdgeWithTy(Deref);
        if (dEdges.empty())
        {
            NodeID addrId = p->addDummyValNode();
            addPEGNode(addrId);
            addEdge(addrId, edge->getSrcID(), Deref);
            addEdge(addrId, edge->getDstID(), Asgn);
        }
        else
            for (CFLEdge* dEdge : dEdges)
                addEdge(dEdge->getSrcID(), edge->getDstID(), Asgn);
    }

    for (PAGEdge* edge : p->getSVFStmtSet(PAGEdge::Copy))
        addEdge(edge->getSrcID(), edge->getDstID(), Asgn);

    for (PAGEdge* edge : p->getSVFStmtSet(PAGEdge::Phi))
    {
        const PhiStmt* phi = SVFUtil::cast<PhiStmt>(edge);
        for (const auto opVar : phi->getOpndVars())
            addEdge(opVar->getId(), phi->getResID(), Asgn);
    }

    for (PAGEdge* edge : p->getSVFStmtSet(PAGEdge::Select))
    {
        const SelectStmt* sel = SVFUtil::cast<SelectStmt>(edge);
        for (const auto opVar : sel->getOpndVars())
            addEdge(opVar->getId(), sel->getResID(), Asgn);
    }

    for (PAGEdge* edge : p->getSVFStmtSet(PAGEdge::Call))
        addEdge(edge->getSrcID(), edge->getDstID(), Asgn);

    for (PAGEdge* edge : p->getSVFStmtSet(PAGEdge::Ret))
        addEdge(edge->getSrcID(), edge->getDstID(), Asgn);

    for (PAGEdge* edge : p->getSVFStmtSet(PAGEdge::ThreadFork))
        addEdge(edge->getSrcID(), edge->getDstID(), Asgn);

    for (PAGEdge* edge : p->getSVFStmtSet(PAGEdge::ThreadJoin))
        addEdge(edge->getSrcID(), edge->getDstID(), Asgn);

    for (PAGEdge* edge : p->getSVFStmtSet(PAGEdge::Gep))
    {
        GepStmt* gep = SVFUtil::cast<GepStmt>(edge);
        if (gep->isVariantFieldGep())
            addEdge(gep->getRHSVarID(), gep->getLHSVarID(), Asgn);
        else
            addEdge(gep->getRHSVarID(), gep->getLHSVarID(), Gep,
                    gep->getConstantFieldIdx());
    }

    // opt load and store
    for (PAGEdge* edge : p->getSVFStmtSet(PAGEdge::Store))
    {
        auto dEdges = getPEGNode(edge->getDstID())->getOutEdgeWithTy(Deref);
        if (dEdges.empty())
        {
            NodeID derefId = p->addDummyObjNode(0);
            addPEGNode(derefId);
            addEdge(edge->getDstID(), derefId, Deref);
            addEdge(edge->getSrcID(), derefId, Asgn);
        }
        else
            for (CFLEdge* dEdge : dEdges)
                addEdge(edge->getSrcID(), dEdge->getDstID(), Asgn);
    }
    for (PAGEdge* edge : p->getSVFStmtSet(PAGEdge::Load))
    {
        auto dEdges = getPEGNode(edge->getSrcID())->getOutEdgeWithTy(Deref);
        if (dEdges.empty())
        {
            NodeID derefId = p->addDummyObjNode(0);
            addPEGNode(derefId);
            addEdge(edge->getSrcID(), derefId, Deref);
            addEdge(derefId, edge->getDstID(), Asgn);
        }
        else
            for (CFLEdge* dEdge : dEdges)
                addEdge(dEdge->getDstID(), edge->getDstID(), Asgn);
    }

    cleanGraph();
}

/*!
 *
 */
void PEG::readGraph(std::string fname)
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
        addPEGNode(src);
        addPEGNode(dst);

        if (vec[2] == "a")
        {
            addEdge(src, dst, Asgn);
        }
        if (vec[2] == "d")
        {
            addEdge(src, dst, Deref);
        }
        if (vec[2] == "f_i")
        {
            addEdge(src, dst, Gep, std::stoi(vec[3]));
        }
    }

    gFile.close();
}

/*!
 *
 */
/*!
 * Start building constraint cflData
 */
void PEG::copyBuild(const PEG& rhs)
{
    /// initialize nodes
    for (auto it = rhs.begin(), eit = rhs.end(); it != eit; ++it)
    {
        if (!it->second->hasIncomingEdge() && !it->second->hasOutgoingEdge())
            continue;
        addPEGNode(it->first);
    }

    /// initialize edges
    for (auto edge : rhs.getPEGEdges())
    {
        addEdge(edge->getSrcID(), edge->getDstID(), edge->getEdgeKind(),
                edge->getEdgeIdx());
    }
}

/*!
 *
 */
void PEG::cleanGraph()
{
    // remove trivial deref edges
    std::stack<CFLEdge*> edgesToRemove;
    for (CFLEdge* edge : getPEGEdges())
    {
        if (edge->getEdgeKind() == Deref &&
            !edge->getSrcNode()->hasIncomingEdge() &&
            !edge->getDstNode()->hasOutgoingEdge() &&
            edge->getSrcNode()->getOutEdges().size() == 1 &&
            edge->getDstNode()->getInEdges().size() == 1)
            edgesToRemove.push(edge);
    }

    while (!edgesToRemove.empty())
    {
        CFLEdge* edge = edgesToRemove.top();
        edgesToRemove.pop();
        removePEGEdge(edge);
    }

    // clear singular nodes
    std::stack<CFLNode*> nodesToRemove;
    for (auto nIt = begin(); nIt != end(); ++nIt)
    {
        if (!nIt->second->hasIncomingEdge() && !nIt->second->hasOutgoingEdge())
            nodesToRemove.push(nIt->second);
    }
    while (!nodesToRemove.empty())
    {
        CFLNode* node = nodesToRemove.top();
        nodesToRemove.pop();
        removePEGNode(node);
    }
}

/*!
 *
 */
//@{
bool PEG::addEdge(NodeID srcId, NodeID dstId, CFLEdge::GEdgeKind k, u32_t idx)
{
    CFLNode* src = getPEGNode(srcId);
    CFLNode* dst = getPEGNode(dstId);
    return addEdge(src, dst, k, idx);
}

bool PEG::addEdge(CFLNode* src, CFLNode* dst, CFLEdge::GEdgeKind k, u32_t idx)
{
    if (hasEdge(src, dst, k, idx))
        return false;

    if (k == Asgn && src == dst)
        return false;

    CFLEdge* edge = new CFLEdge(src, dst, k, idx);
    bool added = pegEdgeSet.insert(edge).second;
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
void PEG::reTargetDstOfEdge(CFLEdge* edge, CFLNode* newDstNode)
{
    NodeID newDstNodeID = newDstNode->getId();
    NodeID srcId = edge->getSrcID();

    addEdge(srcId, newDstNodeID, edge->getEdgeKind(), edge->getEdgeIdx());
    removePEGEdge(edge);
}

/*!
 * Re-target src node of an edge
 * (1) Remove edge from old src target,
 * (2) Change edge src id and
 * (3) Add modified edge into new src
 */
void PEG::reTargetSrcOfEdge(CFLEdge* edge, CFLNode* newSrcNode)
{
    NodeID newSrcNodeID = newSrcNode->getId();
    NodeID dstId = edge->getDstID();

    addEdge(newSrcNodeID, dstId, edge->getEdgeKind(), edge->getEdgeIdx());
    removePEGEdge(edge);
}

/*!
 * Remove edge from their src and dst edge sets
 */
void PEG::removePEGEdge(CFLEdge* edge)
{
    getPEGNode(edge->getSrcID())->removeCFLOutEdge(edge);
    getPEGNode(edge->getDstID())->removeCFLInEdge(edge);
    u32_t num1 = pegEdgeSet.erase(edge);
    delete edge;

    assert(num1 && "edge not in the set, can not remove!!!");
}

/*!
 * Move incoming direct edges of a sub node which is outside SCC to its rep node
 * Remove incoming direct edges of a sub node which is inside SCC from its rep
 * node
 */
bool PEG::moveInEdgesToRepNode(CFLNode* node, CFLNode* rep)
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
        if (isPositiveGepEdge(edge))
            criticalGepInsideSCC = true;

        reTargetDstOfEdge(edge, rep);
    }
    return criticalGepInsideSCC;
}

/*!
 * Move outgoing direct edges of a sub node which is outside SCC to its rep node
 * Remove outgoing direct edges of a sub node which is inside SCC from its rep
 * node
 */
bool PEG::moveOutEdgesToRepNode(CFLNode* node, CFLNode* rep)
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
        if (isPositiveGepEdge(edge))
            criticalGepInsideSCC = true;

        reTargetSrcOfEdge(edge, rep);
    }
    return criticalGepInsideSCC;
}

/*!
 *
 */
void PEG::mergeNodeToRep(NodeID nodeId, NodeID newRepId)
{
    if (nodeId == newRepId || repNodeID(nodeId) != nodeId)
        return;

    CFLNode* node = getPEGNode(nodeId);
    /// move the edges from node to rep, and remove the node
    moveEdgesToRepNode(node, getPEGNode(newRepId));
    /// set rep and sub relations
    updateNodeRepAndSubs(node->getId(), newRepId);
    removePEGNode(node);

    /// vertical merges
    // @{
    //    /// merge deref
    //    auto& derefEdgeSet =
    //    getPEGNode(newRepId)->getOutEdgeWithTy(PEG::Deref); if
    //    (!derefEdgeSet.empty()) {
    //        NodeID dRep = (*derefEdgeSet.begin())->getDstID();
    //        NodeSet subSet;
    //        for (auto edge: derefEdgeSet) {
    //            subSet.insert(edge->getDstID());
    //        }
    //        for (NodeID sub: subSet)
    //            mergeNodeToRep(sub, dRep);
    //    }
    //
    //    /// merge gep
    //    auto& gepEdgeSet = getPEGNode(newRepId)->getOutEdgeWithTy(PEG::Gep);
    //    if (!gepEdgeSet.empty()) {
    //        std::unordered_map<NodeID, NodeBS> indMap;
    //        for (auto edge: gepEdgeSet) {
    //            if (edge->getDstNode()->getInEdges().size() > 1)
    //                continue;
    //            indMap[edge->getEdgeIdx()].set(edge->getDstID());
    //        }
    //        for (auto& indIter: indMap) {
    //            NodeID fRep = indIter.second.find_first();
    //            for (NodeID sub: indIter.second)
    //                mergeNodeToRep(sub, fRep);
    //        }
    //    }
    //@}
}

/*!
 *
 */
void PEG::updateNodeRepAndSubs(NodeID nodeId, NodeID newRepId)
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
void PEG::writeGraph(std::string name)
{
    std::ofstream outFile(name + ".g", std::ios::out);
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

        for (auto edge : node->getOutEdgeWithTy(Asgn))
        {
            NodeID dst = edge->getDstID();
            outFile << src << "\t" << dst << "\ta" << std::endl;
            outFile << dst << "\t" << src << "\tabar" << std::endl;
        }
        for (auto edge : node->getOutEdgeWithTy(Deref))
        {
            NodeID dst = edge->getDstID();
            outFile << src << "\t" << dst << "\td" << std::endl;
            outFile << dst << "\t" << src << "\tdbar" << std::endl;
        }
        for (auto edge : node->getOutEdgeWithTy(Gep))
        {
            NodeID dst = edge->getDstID();
            outFile << src << "\t" << dst << "\tf_i"
                    << "\t" << edge->getEdgeIdx() << std::endl;
            outFile << dst << "\t" << src << "\tfbar_i"
                    << "\t" << edge->getEdgeIdx() << std::endl;
        }
    }

    outFile.close();
}

/*!
 * GraphTraits specialization for constraint cflData
 */
namespace SVF
{
template <> struct DOTGraphTraits<PEG*> : public DOTGraphTraits<PAG*>
{

    typedef CFLNode NodeType;

    DOTGraphTraits(bool isSimple = false) : DOTGraphTraits<PAG*>(isSimple) {}

    /// Return name of the cflData
    static std::string getGraphName(PEG* graph)
    {
        return "PEG";
    }

    /// Return label of a VFG node with two display mode
    /// Either you can choose to display the name of the value or the whole
    /// instruction
    static std::string getNodeLabel(NodeType* n, PEG* graph)
    {
        PAGNode* node = PAG::getPAG()->getGNode(n->getId());
        bool briefDisplay = true;
        std::string str;
        std::stringstream rawstr(str);

        if (briefDisplay)
        {
            if (SVFUtil::isa<ValVar>(node))
            {
                rawstr << node->getId();
            }
            else
                rawstr << node->getId();
        }
        else
        {
            // print the whole value
            if (!SVFUtil::isa<DummyValVar>(node) &&
                !SVFUtil::isa<DummyObjVar>(node))
                rawstr << *node->getValue();
            else
                rawstr << "";
        }

        return rawstr.str();
    }

    static std::string getNodeAttributes(NodeType* n, PEG* graph)
    {
        return "";
    }

    template <class EdgeIter>
    static std::string getEdgeAttributes(NodeType* node, EdgeIter EI, PEG* p)
    {
        std::string str;
        std::stringstream rawstr(str);
        rawstr << "";

        CFLEdge* edge = *(EI.getCurrent());
        assert(edge && "No edge found!!");
        if (edge->getEdgeKind() == PEG::Asgn)
        {
            rawstr << "color=black";
        }
        else if (edge->getEdgeKind() == PEG::Gep)
        {
            rawstr << "color=purple,label=\"{" << edge->getEdgeIdx() << "}\"";
        }
        else if (edge->getEdgeKind() == PEG::Deref)
        {
            rawstr << "color=red";
        }
        else
        {
            rawstr << "color=grey";
        }
        return rawstr.str();
    }

    template <class EdgeIter>
    static std::string getEdgeSourceLabel(NodeType* node, EdgeIter EI)
    {
        return "";
    }
};
} // namespace llvm

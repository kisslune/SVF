/*
 * PEG.h
 *
 *  Created on: Nov 22, 2019
 *      Author: Yuxiang Lei
 */

#ifndef PEG_H_
#define PEG_H_

#include "Graphs/CFLEdge.h"
#include "Graphs/CFLNode.h"

namespace SVF
{
/*!
 * Program expression graph
 */
class PEG : public GenericGraph<CFLNode, CFLEdge>
{
public:
    enum PEGEdgeTy
    {
        Asgn,
        Deref,
        Gep
    };

    typedef Map<NodeID, NodeID> NodeToRepMap;
    typedef Map<NodeID, NodeBS> NodeToSubsMap;

protected:
    NodeToRepMap nodeToRepMap;
    NodeToSubsMap nodeToSubsMap;
    CFLEdge::PEGEdgeSetTy pegEdgeSet;

public:
    /// Constructor
    PEG();

    /// copy constructor
    PEG(const PEG& rhs)
    {
        copyBuild(rhs);
    }

    void build(PAG* p);                /// build from pag
    void readGraph(std::string fname); /// build from graph file
    void copyBuild(const PEG& rhs);    /// copy builder

    void destroy(){};

    /// Destructor
    virtual ~PEG()
    {
        destroy();
    }

    void cleanGraph();

    /// Get/add/remove constraint node
    //@{
    inline const CFLEdge::PEGEdgeSetTy& getPEGEdges() const
    {
        return pegEdgeSet;
    }

    inline CFLNode* getPEGNode(NodeID id) const
    {
        id = repNodeID(id);
        return getGNode(id);
    }

    inline void addPEGNode(NodeID id)
    {
        if (hasPEGNode(id))
            return;

        addGNode(id, new CFLNode(id));
    }

    inline bool hasPEGNode(NodeID id) const
    {
        return hasGNode(id);
    }

    inline void removePEGNode(CFLNode* node)
    {
        removeGNode(node);
    }
    //@}

    // Find and get edges
    //@{
    inline bool hasEdge(CFLNode* src, CFLNode* dst, CFLEdge::GEdgeKind kind,
                        u32_t idx)
    {
        CFLEdge edge(src, dst, kind, idx);
        return hasEdge(&edge);
    }

    inline CFLEdge* getEdge(CFLNode* src, CFLNode* dst, CFLEdge::GEdgeKind kind)
    {
        CFLEdge edge(src, dst, kind);
        auto eit = pegEdgeSet.find(&edge);
        assert(eit != pegEdgeSet.end() && "no such edge!!");
        return *eit;
    }

    inline bool hasEdge(CFLEdge* edge)
    {
        return pegEdgeSet.find(edge) != pegEdgeSet.end();
    }
    //@}

    /// Add a PAG edge into Edge map
    //@{
    /// Add terminal edge with specified type
    bool addEdge(NodeID srcId, NodeID dstId, CFLEdge::GEdgeKind k,
                 u32_t idx = 0);
    bool addEdge(CFLNode* src, CFLNode* dst, CFLEdge::GEdgeKind k,
                 u32_t idx = 0);
    //@}

    /// Used for cycle elimination
    ///@{
    /// Remove edge from old dst target, change edge dst id and add modifed edge
    /// into new dst
    void reTargetDstOfEdge(CFLEdge* edge, CFLNode* newDstNode);

    /// Remove edge from old src target, change edge dst id and add modifed edge
    /// into new src
    void reTargetSrcOfEdge(CFLEdge* edge, CFLNode* newSrcNode);

    /// Remove direct edge from their src and dst edge sets
    void removePEGEdge(CFLEdge* edge);
    /// @}

    /// SCC rep/sub nodes methods
    //@{
    inline NodeID repNodeID(NodeID id) const
    {
        NodeToRepMap::const_iterator it = nodeToRepMap.find(id);
        if (it == nodeToRepMap.end())
            return id;
        else
            return it->second;
    }

    inline NodeBS& subNodeIds(NodeID id)
    {
        nodeToSubsMap[id].set(id);
        return nodeToSubsMap[id];
    }

    inline void setRep(NodeID node, NodeID rep)
    {
        nodeToRepMap[node] = rep;
    }

    inline void setSubs(NodeID node, NodeBS& subs)
    {
        nodeToSubsMap[node] |= subs;
    }

    inline void resetSubs(NodeID node)
    {
        nodeToSubsMap.erase(node);
    }
    //@}

    /// Move incoming direct edges of a sub node which is outside the SCC to its
    /// rep node Remove incoming direct edges of a sub node which is inside the
    /// SCC from its rep node Return TRUE if there's a gep edge inside this SCC
    /// (PWC).
    bool moveInEdgesToRepNode(CFLNode* node, CFLNode* rep);

    /// Move outgoing direct edges of a sub node which is outside the SCC to its
    /// rep node Remove outgoing direct edges of sub node which is inside the
    /// SCC from its rep node Return TRUE if there's a gep edge inside this SCC
    /// (PWC).
    bool moveOutEdgesToRepNode(CFLNode* node, CFLNode* rep);

    /// Move incoming/outgoing direct edges of a sub node to its rep node
    /// Return TRUE if there's a gep edge inside this SCC (PWC).
    inline bool moveEdgesToRepNode(CFLNode* node, CFLNode* rep)
    {
        bool gepIn = moveInEdgesToRepNode(node, rep);
        bool gepOut = moveOutEdgesToRepNode(node, rep);
        return (gepIn || gepOut);
    }

    /// merge nodes
    void mergeNodeToRep(NodeID nodeId, NodeID newRepId);

    /// update reps and subs
    void updateNodeRepAndSubs(NodeID nodeId, NodeID newRepId);

    /// Check if a given edge is a NormalGepCGEdge with 0 offset.
    inline bool isPositiveGepEdge(CFLEdge* edge) const
    {
        return (edge->getEdgeKind() == Gep && edge->getEdgeIdx() > 0);
    }

    void writeGraph(std::string name);
};

} // namespace SVF

namespace SVF
{
/* !
 * GraphTraits specializations for the generic graph algorithms.
 * Provide graph traits for traversing from a constraint node using standard
 * cflData traversals.
 */
template <>
struct GenericGraphTraits<CFLNode*>
    : public GenericGraphTraits<GenericNode<CFLNode, CFLEdge>*>
{
};

/// Inverse GraphTraits specializations for Value flow node, it is used for
/// inverse traversal.
template <>
struct GenericGraphTraits<Inverse<CFLNode*>>
    : public GenericGraphTraits<Inverse<GenericNode<CFLNode, CFLEdge>*>>
{
};

template <>
struct GenericGraphTraits<PEG*>
    : public GenericGraphTraits<GenericGraph<CFLNode, CFLEdge>*>
{
    typedef CFLNode* NodeRef;
};

} // namespace SVF

#endif
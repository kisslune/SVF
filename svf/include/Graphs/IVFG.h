/*
 * IVFG.h
 *
 *  Created on: Aug 1, 2020
 *      Author: Yuxiang Lei
 */

#ifndef IVFG_H_
#define IVFG_H_

#include "Graphs/CFLEdge.h"
#include "Graphs/CFLNode.h"
#include "Graphs/SVFG.h"
#include "SABER/SaberSVFGBuilder.h"
#include "SABER/SaberCheckerAPI.h"

namespace SVF
{
/*!
 * Interprocedural valueflow analysis
 */
class IVFG : public GenericGraph<CFLNode, CFLEdge>
{
public:
    enum EdgeTy
    {
        DirectVF,
        CallVF,
        RetVF
    };

    typedef Map<NodeID, NodeID> NodeToRepMap;
    typedef Map<NodeID, NodeBS> NodeToSubsMap;
    typedef FIFOWorkList<NodeID> WorkList;
    typedef u32_t CallsiteID;

protected:
    NodeToRepMap nodeToRepMap;
    NodeToSubsMap nodeToSubsMap;
    CFLEdge::PEGEdgeSetTy pegEdgeSet;

    void cleanNodes(WorkList& tree);

    bool addEdge(CFLEdge* edge)
    {
        return pegEdgeSet.insert(edge).second;
    }

public:
    /// Constructor
    IVFG();

    void build(SVFG* svfg);            // focus on objs
    void readGraph(std::string fname); /// build from graph file
    void copyBuild(const IVFG& rhs);   /// copy builder

    /// copy constructor
    IVFG(const IVFG& rhs)
    {
        copyBuild(rhs);
    }

    /// Get/add/remove constraint node
    //@{
    inline CFLNode* getVFGNode(NodeID id) const
    {
        id = repNodeID(id);
        return getGNode(id);
    }

    inline void addVFGNode(NodeID id)
    {
        if (hasVFGNode(id))
            return;

        addGNode(id, new CFLNode(id));
    }

    inline bool hasVFGNode(NodeID id) const
    {
        return hasGNode(id);
    }

    inline void removeVFGNode(CFLNode* node)
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
    bool addEdge(NodeID srcId, NodeID dstId, CFLEdge::GEdgeKind k,
                 CallsiteID idx = 0);
    bool addEdge(CFLNode* src, CFLNode* dst, CFLEdge::GEdgeKind k,
                 CallsiteID idx = 0);
    //@}

    /// Get VFG edges
    inline const CFLEdge::PEGEdgeSetTy& getVFGEdges() const
    {
        return pegEdgeSet;
    }

    /// Used for cycle elimination
    //@{
    /// Remove edge from old dst target, change edge dst id and add modifed edge
    /// into new dst
    void reTargetDstOfEdge(CFLEdge* edge, CFLNode* newDstNode);

    /// Remove edge from old src target, change edge dst id and add modifed edge
    /// into new src
    void reTargetSrcOfEdge(CFLEdge* edge, CFLNode* newSrcNode);

    /// Remove direct edge from their src and dst edge sets
    void removeVFGEdge(CFLEdge* edge);
    //@}

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

    /// write graph to file
    void writeGraph(std::string name);
};
} // namespace SVF

/* !
 * GraphTraits specializations for the generic graph algorithms.
 * Provide graph traits for traversing from a constraint node using standard
 * cflData traversals.
 */
namespace SVF
{
template <>
struct GenericGraphTraits<IVFG*>
    : public GenericGraphTraits<GenericGraph<CFLNode, CFLEdge>*>
{
    typedef CFLNode* NodeRef;
};

} // namespace SVF

#endif

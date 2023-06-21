//===- CFLNode.h -- PEG node-----------------------------------//

/*
 * CFLNode.h
 *
 *  Created on: Nov 22, 2019
 *      Author: Yuxiang Lei
 */

#ifndef CFLNODE_H_
#define CFLNODE_H_

namespace SVF
{
/*!
 * Constraint node
 */
typedef GenericNode<CFLNode, CFLEdge> GenericPEGNodeTy;

class CFLNode : public GenericPEGNodeTy
{

public:
    typedef CFLEdge::PEGEdgeSetTy::iterator iterator;
    typedef CFLEdge::PEGEdgeSetTy::const_iterator const_iterator;

    // key: different edge kinds, val: a set holding edges with corresponding kind(s)
    typedef std::map<CFLEdge::GEdgeKind, CFLEdge::PEGEdgeSetTy> PEGEdgeDataTy;

private:
    PEGEdgeDataTy inPEGEdges;  /// holding all the incoming edges of a node
    PEGEdgeDataTy outPEGEdges;  /// holding all the outgoing edges of a node

    CFLEdge::PEGEdgeSetTy directInEdges;    /// direct inedges for SCC
    CFLEdge::PEGEdgeSetTy directOutEdges;   /// direct outedges for SCC

    bool _isSrc, _isSnk;

public:
    static std::set<CFLEdge::GEdgeKind> directEdgeKinds;

    CFLNode(NodeID i) : GenericPEGNodeTy(i, 0), _isSrc(false), _isSnk(false)
    {
    }

    /// Whether a node involves in PWC, if so, all its points-to elements should become field-insensitive.
    //@{
    inline bool isSrc() const
    {
        return _isSrc;
    }
    inline void setSrc()
    {
        _isSrc = true;
    }
    inline bool isSnk() const
    {
        return _isSnk;
    }
    inline void setSnk()
    {
        _isSnk = true;
    }
    //@}

    /// Return constraint edges
    //@{
    inline const CFLEdge::PEGEdgeSetTy& getInEdgeWithTy(CFLEdge::GEdgeKind k)
    {
        return inPEGEdges[k];
    }

    inline const CFLEdge::PEGEdgeSetTy& getOutEdgeWithTy(CFLEdge::GEdgeKind k)
    {
        return outPEGEdges[k];
    }

    inline const CFLEdge::PEGEdgeSetTy& getDirectInEdges() const
    {
        return directInEdges;
    }

    inline const CFLEdge::PEGEdgeSetTy& getDirectOutEdges() const
    {
        return directOutEdges;
    }
    //@}

    ///  Iterators
    //@{
    inline iterator directOutEdgeBegin()
    {
        return getDirectOutEdges().begin();
    }

    inline iterator directOutEdgeEnd()
    {
        return getDirectOutEdges().end();
    }

    inline iterator directInEdgeBegin()
    {
        return getDirectInEdges().begin();
    }

    inline iterator directInEdgeEnd()
    {
        return getDirectInEdges().end();
    }

    inline const_iterator directOutEdgeBegin() const
    {
        return getDirectOutEdges().begin();
    }

    inline const_iterator directOutEdgeEnd() const
    {
        return getDirectOutEdges().end();
    }

    inline const_iterator directInEdgeBegin() const
    {
        return getDirectInEdges().begin();
    }

    inline const_iterator directInEdgeEnd() const
    {
        return getDirectInEdges().end();
    }
    //@}

    ///  Add constraint cflData edges
    //@{
    inline bool addInEdgeWithKind(CFLEdge* inEdge, CFLEdge::GEdgeKind k)
    {
        assert(inEdge->getDstID() == this->getId());
        bool added1 = addIncomingEdge(inEdge);
        bool added2 = inPEGEdges[k].insert(inEdge).second;

        if (directEdgeKinds.find(k) != directEdgeKinds.end())
            directInEdges.insert(inEdge);
        return added1 && added2;
    }

    inline bool addOutEdgeWithKind(CFLEdge* outEdge, CFLEdge::GEdgeKind k)
    {
        assert(outEdge->getSrcID() == this->getId());
        bool added1 = addOutgoingEdge(outEdge);
        bool added2 = outPEGEdges[k].insert(outEdge).second;

        if (directEdgeKinds.find(k) != directEdgeKinds.end())
            directOutEdges.insert(outEdge);
        return added1 && added2;
    }

    inline bool removeCFLInEdge(CFLEdge* inEdge)
    {
        u32_t num1 = removeIncomingEdge(inEdge);

        CFLEdge::GEdgeKind k = inEdge->getEdgeKind();
        u32_t num2 = inPEGEdges[k].erase(inEdge);

        directInEdges.erase(inEdge);
        return num1 && num2;
    }

    inline bool removeCFLOutEdge(CFLEdge* outEdge)
    {
        u32_t num1 = removeOutgoingEdge(outEdge);

        CFLEdge::GEdgeKind k = outEdge->getEdgeKind();
        u32_t num2 = outPEGEdges[k].erase(outEdge);

        directOutEdges.erase(outEdge);
        return num1 && num2;
    }
    //@}

};

}
#endif /* CFLNODE_H_ */

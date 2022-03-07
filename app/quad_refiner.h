#ifndef QUAD_REFINER
#define QUAD_REFINER

#include <vcg/complex/algorithms/polygonal_algorithms.h>
#include <vcg/complex/algorithms/dual_meshing.h>
#include <vcg/complex/algorithms/polygonal_algorithms.h>

template <class CoordType>
void RefineFace(std::vector<CoordType> &Poly,
                std::vector<bool> &SplitE,
                std::vector<bool> &SplitF,
                std::vector<bool> &SplitV,
                std::vector<CoordType> &NewEdgeCoord,
                std::vector<std::vector<CoordType> > &NewPoly)
{

    assert(SplitV.size()==Poly.size());
    assert(SplitE.size()==Poly.size());
    assert(SplitF.size()==Poly.size());
    assert(NewEdgeCoord.size()==Poly.size());

    NewPoly.clear();

    std::vector<CoordType> BorderV;
    std::vector<bool> ToSplitVert;
    //add the new vertices
    for (size_t i=0;i<Poly.size();i++)
    {
        BorderV.push_back(Poly[i]);
        ToSplitVert.push_back(SplitV[i]);

        if (!SplitE[i])continue;
        BorderV.push_back(NewEdgeCoord[i]);
        ToSplitVert.push_back(SplitF[i]);
    }

    //then count the number of vertices from which a split edge depart
    size_t numSplit=0;
    for (size_t i=0;i<ToSplitVert.size();i++)
        if (ToSplitVert[i])numSplit++;

    //if a single vertex has been inserted then nothing happens
    if (numSplit==0)
    {
        NewPoly.push_back(BorderV);
        return;
    }

    //otherwise insert a new vertex in the middle
    CoordType bary(0,0,0);
    for (size_t i=0;i<Poly.size();i++)
        bary+=Poly[i];
    bary/=Poly.size();

    //then start going around the polygon and get the first one to split
    int startingV=-1;
    for (size_t i=0;i<ToSplitVert.size();i++)
        if (ToSplitVert[i]){startingV=i;break;}
    assert(startingV!=-1);

    //then create the faces
    for (size_t i=0;i<BorderV.size();i++)
    {
        size_t curr_i=(startingV+i)%BorderV.size();

        //create a new polygon if needed
        if (ToSplitVert[curr_i])
        {
            NewPoly.resize(NewPoly.size()+1);
            //add the middle vertex
            NewPoly.back().push_back(bary);
        }

        //then the other one
        CoordType CurrPos=BorderV[curr_i];
        NewPoly.back().push_back(CurrPos);

        //replicate the last one if needed
        size_t next_i=(curr_i+1)%BorderV.size();
        if (ToSplitVert[next_i])
            NewPoly.back().push_back(BorderV[next_i]);
    }
}



template <class PolyMeshType>
void RefineFaces(PolyMeshType &polyM,
                 const std::vector<size_t> &IndexF,
                 std::vector<std::vector<bool> > &SplitE,
                 std::vector<std::vector<bool> > &SplitF,
                 std::vector<std::vector<bool> > &SplitV,
                 std::vector<std::vector<typename PolyMeshType::CoordType> > &NewEdgeCoord)
{
    typedef typename PolyMeshType::VertexType PVert;
    typedef typename PolyMeshType::CoordType CoordType;

    for (size_t i=0;i<IndexF.size();i++)
    {
        size_t currF=IndexF[i];
        assert(polyM.face[currF].VN()==(int)SplitE[i].size());
        assert(polyM.face[currF].VN()==(int)SplitF[i].size());
        assert(polyM.face[currF].VN()==(int)SplitV[i].size());
        assert(polyM.face[currF].VN()==(int)NewEdgeCoord[i].size());

        std::vector<CoordType> Poly;
        for (size_t i=0;i<(int)polyM.face[currF].VN();i++)
            Poly.push_back(polyM.face[currF].V(i)->P());

        //then rene the face
        std::vector<std::vector<CoordType> > NewPoly;
        RefineFace<CoordType>(Poly,SplitE[i],SplitF[i],SplitV[i],NewEdgeCoord[i],NewPoly);

        //add all faces
        for (size_t j=0;j<NewPoly.size();j++)
        {
            vcg::tri::Allocator<PolyMeshType>::AddFaces(polyM,1);
            std::vector<size_t> FaceV;
            for (size_t k=0;k<NewPoly[j].size();k++)
            {
                vcg::tri::Allocator<PolyMeshType>::AddVertex(polyM,NewPoly[j][k]);
                FaceV.push_back(polyM.vert.size()-1);
            }
            polyM.face.back().Alloc(FaceV.size());
            for (size_t k=0;k<FaceV.size();k++)
            {
                PVert *v=&polyM.vert[FaceV[k]];
                assert(k<polyM.face.back().VN());
                polyM.face.back().V(k)=v;
            }
        }
    }

    //finally delete the refined faces
    for (size_t i=0;i<IndexF.size();i++)
        vcg::tri::Allocator<PolyMeshType>::DeleteFace(polyM,polyM.face[IndexF[i]]);

    //remove unreferenced vertices
    vcg::tri::Clean<PolyMeshType>::RemoveUnreferencedVertex(polyM);
    vcg::tri::Clean<PolyMeshType>::RemoveDuplicateVertex(polyM);
    vcg::tri::Allocator<PolyMeshType>::CompactEveryVector(polyM);

    //then remove valence 2 vertices
    vcg::PolygonalAlgorithm<PolyMeshType>::RemoveValence2Vertices(polyM);
}

template <class CoordType>
bool CollapseFace(std::vector<CoordType> &PolyV0,
                  std::vector<CoordType> &PolyV1,
                  std::vector<CoordType> &NewPoly)
{
    //create a map of the edges
    std::map<std::pair<CoordType,CoordType>,size_t> EdgeCoords0;//,EdgeCoords1;

    for (size_t i=0;i<PolyV0.size();i++)
    {
        CoordType P0=PolyV0[i];
        CoordType P1=PolyV0[(i+1) % PolyV0.size()];
        std::pair<CoordType,CoordType> EdgeKey(std::min(P0,P1),std::max(P0,P1));
        EdgeCoords0[EdgeKey]=i;
    }

    //retrieve the shared edges
    int IndexE0=-1;
    int IndexE1=-1;
    for (size_t i=0;i<PolyV1.size();i++)
    {
        CoordType P0=PolyV1[i];
        CoordType P1=PolyV1[(i+1) % PolyV1.size()];
        std::pair<CoordType,CoordType> EdgeKey(std::min(P0,P1),std::max(P0,P1));
        if (EdgeCoords0.count(EdgeKey)==0)continue;
        if (IndexE0!=-1)return false;//multiple shared edges
        IndexE0=EdgeCoords0[EdgeKey];
        IndexE1=i;
    }
    if (IndexE0==-1)return false;//no shared edge
    assert(IndexE0>=0);
    assert(IndexE0<(int)PolyV0.size());
    assert(IndexE1>=0);
    assert(IndexE1<(int)PolyV1.size());

    //then merge along
    NewPoly.clear();

    size_t StartI0=(IndexE0+1)%PolyV0.size();
    size_t EndI0=IndexE0;
    for (size_t i=StartI0;i!=EndI0;i=(i+1)%PolyV0.size())
        NewPoly.push_back(PolyV0[i]);

    size_t StartI1=(IndexE1+1)%PolyV1.size();
    size_t EndI1=IndexE1;
    for (size_t i=StartI1;i!=EndI1;i=(i+1)%PolyV1.size())
        NewPoly.push_back(PolyV1[i]);

    return true;
}

template <class PolyFaceType>
void FaceCoords(const PolyFaceType &polyF,
                std::vector<typename PolyFaceType::CoordType> &FacePos)
{
    FacePos.clear();
    for (int i=0;i<polyF.VN();i++)
        FacePos.push_back(polyF.cV(i)->cP());

}

template <class PolyMeshType>
bool CollapseFaces(PolyMeshType &PolyM,
                   const std::vector<size_t> &IndexF,
                   const std::vector<size_t> &IndexE)
{
    typedef typename PolyMeshType::FaceType PFace;
    typedef typename PolyMeshType::VertexType PVert;
    typedef typename PolyMeshType::CoordType CoordType;

    //check one face should be collapsed only once
    vcg::tri::UpdateFlags<PolyMeshType>::FaceClearV(PolyM);

    std::vector<std::vector<CoordType> > PolyV0,PolyV1;

    std::vector<size_t> To_EraseF;
    for (size_t i=0;i<IndexF.size();i++)
    {
        size_t currF=IndexF[i];
        size_t currE=IndexE[i];
        //same face modified twice
        if (PolyM.face[currF].IsV())
            return false;

        PolyM.face[currF].SetV();

        //cannot collapse a border vertex
        if (vcg::face::IsBorder(PolyM.face[currF],currE))
            return false;

        PFace *FOpp=PolyM.face[currF].FFp(currE);
        //size_t EOpp=polyM.face[currF].FFi(currE);

        if (FOpp->IsV())
            return false;

        PolyV0.resize(PolyV0.size()+1);
        FaceCoords(PolyM.face[currF],PolyV0.back());

        PolyV1.resize(PolyV1.size()+1);
        FaceCoords((*FOpp),PolyV1.back());

        To_EraseF.push_back(currF);
        To_EraseF.push_back(vcg::tri::Index(PolyM,(*FOpp)));

    }

    //do the collapse
    assert(PolyV0.size()==PolyV1.size());
    std::vector<std::vector<CoordType> > NewPolys;
    for (size_t i=0;i<PolyV0.size();i++)
    {
        NewPolys.resize(NewPolys.size()+1);
        bool collapsed=CollapseFace(PolyV0[i],PolyV1[i],NewPolys.back());
        if (!collapsed)return false;
    }

    //add all faces and vertices
    for (size_t j=0;j<NewPolys.size();j++)
    {
        vcg::tri::Allocator<PolyMeshType>::AddFaces(PolyM,1);
        std::vector<size_t> FaceV;
        for (size_t k=0;k<NewPolys[j].size();k++)
        {
            vcg::tri::Allocator<PolyMeshType>::AddVertex(PolyM,NewPolys[j][k]);
            FaceV.push_back(PolyM.vert.size()-1);
        }
        PolyM.face.back().Alloc(FaceV.size());
        for (size_t k=0;k<FaceV.size();k++)
        {
            PVert *v=&PolyM.vert[FaceV[k]];
            assert(k<PolyM.face.back().VN());
            PolyM.face.back().V(k)=v;
        }
    }

    //finally delete the refined faces
    for (size_t i=0;i<To_EraseF.size();i++)
        vcg::tri::Allocator<PolyMeshType>::DeleteFace(PolyM,PolyM.face[To_EraseF[i]]);

    //remove unreferenced vertices
    vcg::tri::Clean<PolyMeshType>::RemoveUnreferencedVertex(PolyM);
    vcg::tri::Clean<PolyMeshType>::RemoveDuplicateVertex(PolyM);
    vcg::tri::Allocator<PolyMeshType>::CompactEveryVector(PolyM);

    //then remove valence 2 vertices
    vcg::PolygonalAlgorithm<PolyMeshType>::RemoveValence2Vertices(PolyM);

    return true;
}

void FaceOppositeEdges(const size_t &EdgeNum,
                       const size_t &IndexE,
                       std::vector<size_t> &OppE)
{
    OppE.clear();
    assert(EdgeNum>2);

    //if even then put the oppoite only
    if ((EdgeNum%2)==0)
    {
        size_t OppEIndex=(IndexE+EdgeNum/2)%EdgeNum;
        assert(OppEIndex!=IndexE);
        OppE.push_back(OppEIndex);
    }else
    {
        assert((EdgeNum%2)==1);
        size_t OppVIndex=(1+IndexE+EdgeNum/2)%EdgeNum;
        size_t OppEIndex0=OppVIndex;
        size_t OppEIndex1=(OppVIndex+EdgeNum-1)%EdgeNum;
        assert(OppEIndex0!=IndexE);
        assert(OppEIndex1!=IndexE);
        OppE.push_back(OppEIndex0);
        OppE.push_back(OppEIndex1);
    }

    //propagate for size bigget than 5
    if (EdgeNum>=6)
    {
        std::vector<size_t> AddE;
        for (size_t i=0;i<OppE.size();i++)
        {
            size_t currE=OppE[i];
            size_t currE0=(currE+1)%EdgeNum;
            size_t currE1=(currE+EdgeNum-1)%EdgeNum;
            AddE.push_back(currE0);
            AddE.push_back(currE1);
        }
        OppE.insert(OppE.end(),AddE.begin(),AddE.end());
        std::sort(OppE.begin(),OppE.end());
        auto last = std::unique(OppE.begin(), OppE.end());
        OppE.erase(last, OppE.end());
    }
}

void FaceOppositeVertices(const size_t &EdgeNum,
                          const size_t &IndexE,
                          std::vector<size_t> &OppV)
{
    OppV.clear();
    assert(EdgeNum>2);

    //if even then put the oppoite only
    if ((EdgeNum%2)==0)
    {
        size_t OppEIndex=(IndexE+EdgeNum/2)%EdgeNum;
        assert(OppEIndex!=IndexE);
        size_t OppVIndex0=OppEIndex;
        size_t OppVIndex1=(OppVIndex0+1)%EdgeNum;
        OppV.push_back(OppVIndex0);
        OppV.push_back(OppVIndex1);
    }else
    {
        assert((EdgeNum%2)==1);
        size_t OppVIndex=(1+IndexE+EdgeNum/2)%EdgeNum;
        OppV.push_back(OppVIndex);
    }
}

template <class PolyMeshType>
class QuadRefiner
{
    typedef typename PolyMeshType::FaceType PFace;
    typedef typename PolyMeshType::VertexType PVert;
    typedef typename PolyMeshType::CoordType CoordType;
    typedef typename PolyMeshType::ScalarType ScalarType;

    PolyMeshType &PolyM;
    ScalarType TargetEdge;
    //ScalarType Tolerance;


    typedef std::pair<CoordType,CoordType> EdgeKey;
    std::map<EdgeKey,CoordType> SplitPos;
    std::set<EdgeKey> To_Split;
    std::set<EdgeKey> To_Split_Original;
    std::set<CoordType> OriginalPos;



    void GetEdgeKey(const PFace &f,const size_t &IndexE,EdgeKey &Ekey)
    {
        assert(IndexE<f.VN());

        size_t IndexV0=IndexE;
        size_t IndexV1=(IndexE+1)%f.VN();

        CoordType P0= f.cV(IndexV0)->cP();
        CoordType P1= f.cV(IndexV1)->cP();
        Ekey=EdgeKey(std::min(P0,P1),std::max(P0,P1));
    }

    bool HasOneOppositeSplit(const PFace &f,const size_t &IndexE)
    {
        std::vector<size_t> OppE;
        FaceOppositeEdges(f.VN(),IndexE,OppE);
        for (size_t k=0;k<OppE.size();k++)
        {
            EdgeKey KOpp;
            size_t OppEIndex=OppE[k];
            assert((int)OppEIndex!=IndexE);
            GetEdgeKey(f,OppEIndex,KOpp);
            if (To_Split.count(KOpp)==0)continue;
            return true;
        }
        return false;
    }

    int GetBestSplitVert(const PFace &f,size_t &IndexE)
    {
        std::vector<size_t> OppV;
        FaceOppositeVertices(f.VN(),IndexE,OppV);
        EdgeKey K;
        GetEdgeKey(f,IndexE,K);
        assert(To_Split.count(K)>0);
        assert(SplitPos.count(K)>0);
        CoordType P0=SplitPos[K];
        ScalarType minD=std::numeric_limits<ScalarType>::max();
        int closestV=-1;
        for (size_t i=0;i<OppV.size();i++)
        {
            size_t IndexV=OppV[i];
            CoordType P1=f.cV(IndexV)->cP();
            if ((P0-P1).Norm()>minD)continue;
            minD=(P0-P1).Norm();
            closestV=IndexV;
        }
        assert(closestV!=-1);
        return (closestV);
    }

    void OptimizeForPrimal(const PFace &f,
                           std::vector<bool> &SplitE,
                           std::vector<bool> &SplitF,
                           std::vector<bool> &SplitV)
    {
        //first count the number of splits
        size_t numSplit=0;
        for (size_t i=0;i<SplitE.size();i++)
            if (SplitE[i])numSplit++;

        //in this case is already ok
        if ((numSplit%2)==0)return;

        for (size_t i=0;i<SplitE.size();i++)
        {
            if (!SplitE[i])continue;

            //must be necessarily split for primal
            assert(SplitF[i]);

            //check if it has opposite split
            std::vector<size_t> OppE;
            FaceOppositeEdges(f.VN(),i,OppE);

            //in this case is ok
            if (HasOneOppositeSplit(f,i))continue;

            //otherwise get best opposite vert
            int BestV=GetBestSplitVert(f,i);

            //then split that vertex
            SplitV[BestV]=true;
        }
    }

    void OptimizeForDual(const PFace &f,
                         std::vector<bool> &SplitE,
                         std::vector<bool> &SplitF)
    {

        for (size_t i=0;i<SplitE.size();i++)
        {
            if (!SplitE[i])continue;


            //check if it has opposite split
            std::vector<size_t> OppE;
            FaceOppositeEdges(f.VN(),i,OppE);

            //in this case is ok
            if (HasOneOppositeSplit(f,i))continue;


            //then avoid splitting it
            SplitF[i]=false;
        }
    }

public:

    bool optimize_for_dual;
    bool erode_dilate;
    bool dualize_final;
    bool final_optimize;
    ScalarType tolerance;

    void RemoveSingleSplitS(bool OnlyBorder=false)
    {
        std::vector<EdgeKey> ToRemove;
        for (size_t i=0;i<PolyM.face.size();i++)
            for (int j=0;j<PolyM.face[i].VN();j++)
            {
                EdgeKey K;
                GetEdgeKey(PolyM.face[i],j,K);
                if (OnlyBorder && (!vcg::face::IsBorder(PolyM.face[i],j)))continue;
                //in this case no need doing anything
                if (To_Split.count(K)==0)continue;

                //in this case should not been removed
                if (HasOneOppositeSplit(PolyM.face[i],j))continue;

                if (vcg::face::IsBorder(PolyM.face[i],j))
                {
                    ToRemove.push_back(K);
                    continue;
                }

                //otherwise check the other side
                PFace *Fopp=PolyM.face[i].FFp(j);
                size_t Iopp=PolyM.face[i].FFi(j);
                if (HasOneOppositeSplit((*Fopp),Iopp))continue;
                ToRemove.push_back(K);
            }

        for (size_t i=0;i<ToRemove.size();i++)
            To_Split.erase(ToRemove[i]);
    }


    void InitTargetEdge()
    {
        TargetEdge=0;
        size_t Num=0;
        for (size_t i=0;i<PolyM.face.size();i++)
            for (int j=0;j<PolyM.face[i].VN();j++)
            {
                if (vcg::face::IsBorder(PolyM.face[i],j))continue;
                size_t numV=PolyM.face[i].VN();
                PVert *v0=PolyM.face[i].V(j);
                PVert *v1=PolyM.face[i].V((j+1)%numV);

                if (v0->IsB())continue;
                if (v1->IsB())continue;

                TargetEdge+=(v0->P()-v1->P()).Norm();
                Num++;
            }
        assert(Num>0);
        TargetEdge/=Num;
    }

    void InitInternalSplitAsMidPoint()
    {
        SplitPos.clear();
        for (size_t i=0;i<PolyM.face.size();i++)
            for (int j=0;j<PolyM.face[i].VN();j++)
            {
                EdgeKey K;
                GetEdgeKey(PolyM.face[i],j,K);
                CoordType MidP=(K.first+K.second)/2;
                SplitPos[K]=MidP;
            }
    }

    void DilateToSplitStep(bool check_initial=true)
    {
        std::vector<EdgeKey> ToAdd;
        for (size_t i=0;i<PolyM.face.size();i++)
            for (int j=0;j<PolyM.face[i].VN();j++)
            {
                EdgeKey K;
                GetEdgeKey(PolyM.face[i],j,K);
                if (To_Split.count(K)==0)continue;

                std::vector<size_t> OppE;
                FaceOppositeEdges(PolyM.face[i].VN(),j,OppE);

                for (size_t k=0;k<OppE.size();k++)
                {
                    EdgeKey KOpp;
                    size_t OppEIndex=OppE[k];
                    assert((int)OppEIndex!=j);
                    GetEdgeKey(PolyM.face[i],OppEIndex,KOpp);
                    ToAdd.push_back(KOpp);
                }
            }

        for (size_t i=0;i<ToAdd.size();i++)
        {
            if (check_initial && (To_Split_Original.count(ToAdd[i])==0))continue;
            To_Split.insert(ToAdd[i]);
        }
    }

    void ErodeToSplitStep()
    {
        std::vector<EdgeKey> ToRemove;
        for (size_t i=0;i<PolyM.face.size();i++)
            for (int j=0;j<PolyM.face[i].VN();j++)
            {
                EdgeKey K;
                GetEdgeKey(PolyM.face[i],j,K);
                if (To_Split.count(K)==0)continue;

                bool keepIt=HasOneOppositeSplit(PolyM.face[i],j);

                 if (!keepIt)
                    ToRemove.push_back(K);
            }

        for (size_t i=0;i<ToRemove.size();i++)
            To_Split.erase(ToRemove[i]);

        //RemoveSingleSplitS(true);
    }

    void InitToSplit()
    {
        //InitTargetEdge();
        To_Split.clear();
        for (size_t i=0;i<PolyM.face.size();i++)
            for (int j=0;j<PolyM.face[i].VN();j++)
            {
                EdgeKey K;
                GetEdgeKey(PolyM.face[i],j,K);
                ScalarType L0=(K.first-K.second).Norm();
                //                ScalarType L1=L0/2;

                //                if (fabs(L0-TargetEdge)<(fabs(L1-TargetEdge)))continue;
                //EdgeKey K(std::min(P0,P1),std::max(P0,P1));
                if (L0<=(TargetEdge*(1+tolerance)))
                    continue;

                To_Split.insert(K);
            }
        To_Split_Original=To_Split;
    }

    void RetrieveSplitEdgeData(const PFace &f,
                               std::vector<bool> &SplitE,
                               std::vector<bool> &SplitV,
                               std::vector<bool> &SplitF,
                               std::vector<CoordType> &NewEdgeCoord)
    {
        SplitE=std::vector<bool>(f.VN(),false);
        SplitV=std::vector<bool>(f.VN(),false);
        SplitF=std::vector<bool>(f.VN(),true);
        NewEdgeCoord=std::vector<CoordType>(f.VN(),CoordType(0,0,0));
        for (size_t i=0;i<f.VN();i++)
        {
            EdgeKey K;
            GetEdgeKey(f,i,K);
            if (To_Split.count(K)==0)continue;
            SplitE[i]=true;
            NewEdgeCoord[i]=SplitPos[K];
        }

        if (!optimize_for_dual)
            OptimizeForPrimal(f,SplitE,SplitF,SplitV);
        else
            OptimizeForDual(f,SplitE,SplitF);
    }

    void RetrieveSplitEdgeData(std::vector<size_t> &IndexF,
                               std::vector<std::vector<bool> > &SplitE,
                               std::vector<std::vector<bool> > &SplitF,
                               std::vector<std::vector<bool> > &SplitV,
                               std::vector<std::vector<CoordType> > &NewEdgeCoord)
    {
        IndexF.clear();
        SplitE.resize(PolyM.face.size());
        SplitF.resize(PolyM.face.size());
        SplitV.resize(PolyM.face.size());
        NewEdgeCoord.resize(PolyM.face.size());
        for (size_t i=0;i<PolyM.face.size();i++)
        {
            IndexF.push_back(i);
            RetrieveSplitEdgeData(PolyM.face[i],SplitE[i],SplitV[i],SplitF[i],NewEdgeCoord[i]);
        }
    }

    void InitOriginalPos()
    {
        OriginalPos.clear();
        for (size_t i=0;i<PolyM.vert.size();i++)
            OriginalPos.insert(PolyM.vert[i].P());
    }

    void SelectOriginalV()
    {
        vcg::tri::UpdateSelection<PolyMeshType>::VertexClear(PolyM);
        for (size_t i=0;i<PolyM.vert.size();i++)
            if (OriginalPos.count(PolyM.vert[i].P())>0)PolyM.vert[i].SetS();
    }

    void UpdateMesh()
    {
        vcg::PolygonalAlgorithm<PolyMeshType>::UpdateFaceNormalByFitting(PolyM);
        vcg::tri::UpdateNormal<PolyMeshType>::PerVertexNormalized(PolyM);

        vcg::tri::UpdateBounding<PolyMeshType>::Box(PolyM);
        vcg::tri::UpdateTopology<PolyMeshType>::FaceFace(PolyM);
        vcg::tri::UpdateFlags<PolyMeshType>::VertexBorderFromFaceAdj(PolyM);
        vcg::PolygonalAlgorithm<PolyMeshType>::UpdateFaceNormalByFitting(PolyM);
    }

    void SplitStep()
    {

        std::vector<size_t> IndexF;
        std::vector<std::vector<bool> > SplitE,SplitF,SplitV;
        std::vector<std::vector<CoordType> > NewEdgeCoord;

        //retrieve info
        RetrieveSplitEdgeData(IndexF,SplitE,SplitF,SplitV,NewEdgeCoord);
        RefineFaces(PolyM,IndexF,SplitE,SplitF,SplitV,NewEdgeCoord);

        SplitPos.clear();
        To_Split.clear();

        UpdateMesh();
    }

    void Refine(bool write_debug=true)
    {
//        InitOriginalPos();
        InitTargetEdge();
        size_t num_refine=0;
        do
        {
            InitInternalSplitAsMidPoint();
            InitToSplit();
            if (erode_dilate)
            {
                ErodeToSplitStep();
                RemoveSingleSplitS(true);
                DilateToSplitStep();
            }
            RemoveSingleSplitS();
            num_refine=To_Split.size();

            if(write_debug)
                std::cout<<"Refining Edges:"<<num_refine<<std::endl;

            if (num_refine>0)
                SplitStep();
        }while (num_refine>0);

        //first optimize the mesh
//        SelectOriginalV();
//        vcg::PolygonalAlgorithm<PolyMeshType>::SmoothPCA(PolyM,10,0.5,true,false,0.1,true,false);
        if (dualize_final)
        {
            PolyMeshType dualM;
            vcg::tri::DualMeshing<PolyMeshType>::MakeDual(PolyM,dualM);
            vcg::PolygonalAlgorithm<PolyMeshType>::RemoveValence2Faces(dualM);
            PolyM.Clear();
            vcg::tri::Append<PolyMeshType,PolyMeshType>::Mesh(PolyM,dualM);
            UpdateMesh();
        }

        if (final_optimize)
            vcg::PolygonalAlgorithm<PolyMeshType>::SmoothPCA(PolyM,10,0.5,false,false);
    }

    //    void InitInternalSplitFromHighResMesh(PolyMeshType &HresPoly)
    //    {
    //        assert(0);
    //    }

    void GLDrawSplitPos()
    {

        glPushAttrib(GL_ALL_ATTRIB_BITS);

        glDepthRange(0,0.9999);

        glDisable(GL_LIGHTING);

        glPointSize(10);

        glBegin(GL_POINTS);
        for (size_t i=0;i<PolyM.face.size();i++)
            for (int j=0;j<PolyM.face[i].VN();j++)
            {
                EdgeKey K;
                GetEdgeKey(PolyM.face[i],j,K);

                if(SplitPos.count(K)==0)continue;
                CoordType Pos=SplitPos[K];

                if (To_Split.count(K)==0)
                    vcg::glColor(vcg::Color4b(0,255,0,255));
                else
                    vcg::glColor(vcg::Color4b(255,0,0,255));

                vcg::glVertex(Pos);
            }
        glEnd();
        glPopAttrib();
    }



    QuadRefiner(PolyMeshType &_PolyM):PolyM(_PolyM)
    {
        optimize_for_dual=false;
        dualize_final=false;
        final_optimize=true;
        erode_dilate=true;
        tolerance=0.25;
    }
};

#endif

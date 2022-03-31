#ifndef MY_TRI_MESH_TYPE
#define MY_TRI_MESH_TYPE

#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/simplex/face/topology.h>
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export.h>
#include <wrap/gl/trimesh.h>
//
#include <wrap/igl/smooth_field.h>
#include <wrap/io_trimesh/export_field.h>
#include <wrap/io_trimesh/import_field.h>
#include <iostream>
#include <fstream>
#include <vcg/complex/algorithms/polygonal_algorithms.h>

class PolyFace;
class PolyVertex;

struct PUsedTypes: public vcg::UsedTypes<vcg::Use<PolyVertex>  ::AsVertexType,
        vcg::Use<PolyFace>	::AsFaceType>{};

class PolyVertex:public vcg::Vertex<	PUsedTypes,
        vcg::vertex::Coord3d,
        vcg::vertex::Normal3d,
        vcg::vertex::Mark,
        vcg::vertex::BitFlags,
        vcg::vertex::Qualityd,
        vcg::vertex::Color4b,
        vcg::vertex::TexCoord2d>
{
public:
    bool IsFix;
    bool IsBound;
} ;

class PolyFace:public vcg::Face<
        PUsedTypes
        ,vcg::face::PolyInfo
        ,vcg::face::PFVAdj
        ,vcg::face::PFFAdj
        ,vcg::face::BitFlags
        ,vcg::face::Normal3d
        ,vcg::face::Color4b
        ,vcg::face::Qualityd      // face quality.
        ,vcg::face::BitFlags
        ,vcg::face::Mark
        ,vcg::face::CurvatureDird> {
};

class PMesh: public
        vcg::tri::TriMesh<
        std::vector<PolyVertex>,	// the vector of vertices
        std::vector<PolyFace >     // the vector of faces
        >
{
public:

    void TriangulateQuadBySplit(size_t IndexF)
    {

        size_t sizeV=face[IndexF].VN();
        assert(sizeV==4);

        //then reupdate the faces
        VertexType * v0=face[IndexF].V(0);
        VertexType * v1=face[IndexF].V(1);
        VertexType * v2=face[IndexF].V(2);
        VertexType * v3=face[IndexF].V(3);

        face[IndexF].Dealloc();
        face[IndexF].Alloc(3);
        face[IndexF].V(0)=v0;
        face[IndexF].V(1)=v1;
        face[IndexF].V(2)=v3;

        vcg::tri::Allocator<PMesh>::AddFaces(*this,1);

        face.back().Alloc(3);
        face.back().V(0)=v1;
        face.back().V(1)=v2;
        face.back().V(2)=v3;
    }

    void GoToNextSeqPos(vcg::face::Pos<PolyFace> &currPos)
    {
        //        PolyFace *F=currPos.F();
        //        size_t E=currPos.E();
        //        //first parametric line
        //        do{
        //            currPos.FlipE();
        //            currPos.FlipF();
        //        }while (!currPos.F()->OnParametricLine[currPos.E()]);
        //        //second para line
        //        do{
        //            currPos.FlipE();
        //            currPos.FlipF();
        //        }while (!currPos.F()->OnParametricLine[currPos.E()]);
        //come back on the other side
        currPos.FlipE();
        currPos.FlipF();
        currPos.FlipE();
        //and on the other vertex
        currPos.FlipV();
    }

    void GetEdgeSequenceFrom(vcg::face::Pos<PolyFace> &FirstEdge,
                             std::vector<vcg::face::Pos<PolyFace> > &EdgeSeq,
                             bool &IsCircular,
                             bool OnlyB)
    {
        if (!OnlyB)assert(!FirstEdge.IsBorder());
        if (OnlyB)assert(FirstEdge.IsBorder());

        EdgeSeq.clear();

        //set the pos
        vcg::face::Pos<PolyFace> currPos=FirstEdge;
        if (!OnlyB)assert(!currPos.IsBorder());
        if (OnlyB)assert(currPos.IsBorder());

        bool HasTerminated=false;
        IsCircular=false;
        do{
            EdgeSeq.push_back(currPos);

            //check if it is a singularity or a border
            //(in this case it has terminated)
            if (!OnlyB)
                HasTerminated=(currPos.V()->IsS() ||
                               currPos.V()->IsB() );
            else
                HasTerminated=(currPos.V()->IsS());

            if ((!HasTerminated)&&(!OnlyB))
            {
                GoToNextSeqPos(currPos);
            }
            else
                if ((!HasTerminated)&&(OnlyB))
                {
                    if (!OnlyB)assert(!currPos.IsBorder());
                    if (OnlyB)assert(currPos.IsBorder());
                    currPos.NextB();
                    if (!OnlyB)assert(!currPos.IsBorder());
                    if (OnlyB)assert(currPos.IsBorder());
                }
            IsCircular=(!HasTerminated)&&(currPos==FirstEdge);
            HasTerminated|=IsCircular;
        }while (!HasTerminated);

        //reverse the order of the first one
        std::reverse(EdgeSeq.begin(),EdgeSeq.end());
        for (size_t i=0;i<EdgeSeq.size();i++)
            EdgeSeq[i].FlipV();

        if (IsCircular)return;

        //then go on the other side
        currPos=FirstEdge;
        currPos.FlipV();
        if (!OnlyB)assert(!currPos.IsBorder());
        if (OnlyB)assert(currPos.IsBorder());

        if (!OnlyB)
            HasTerminated=(currPos.V()->IsS() || currPos.V()->IsB());
        else
            HasTerminated=(currPos.V()->IsS());

        while (!HasTerminated)
        {
            if ((!HasTerminated)&&(!OnlyB))
            {

                GoToNextSeqPos(currPos);
            }
            else
                if ((!HasTerminated)&&(OnlyB))
                {
                    if (!OnlyB)assert(!currPos.IsBorder());
                    if (OnlyB)assert(currPos.IsBorder());
                    currPos.NextB();
                    if (!OnlyB)assert(!currPos.IsBorder());
                    if (OnlyB)assert(currPos.IsBorder());
                }

            EdgeSeq.push_back(currPos);

            //HasTerminated=(currPos.V()->IsS() || currPos.V()->IsB());
            if (!OnlyB)
                HasTerminated=(currPos.V()->IsS() ||
                               currPos.V()->IsB() );
            else
                HasTerminated=(currPos.V()->IsS());
        }
    }

    std::vector<std::vector<vcg::face::Pos<PolyFace> > > EdgeSeq;
    std::vector<bool > Circular;

    //    std::vector<std::vector<vcg::face::Pos<QuadFaceC> > > EdgeSeq;
    //    std::vector<vcg::face::Pos<QuadFaceC> > FirstPos;

    //    std::vector<ScalarType> AvgEdgeEnergy;

    //    std::vector<vcg::Color4b> ColorSeq;
    //    std::vector<ScalarType> EnergySeq;

    //    void SelectIrregular(bool CheckVolume)
    //    {
    //        UpdateAttributes();

    //        std::vector<size_t> Valence(vert.size(),0);
    //        std::vector<size_t> ValenceVol(vert.size(),0);
    //        std::set<std::pair<size_t,size_t> > InsertedEdges;
    //        for (size_t i=0;i<face.size();i++)
    //            for (int j=0;j<face[i].VN();j++)
    //            {
    //                if (!face[i].OnParametricLine[j])continue;

    //                size_t VIndex0=vcg::tri::Index(*this,face[i].V0(j));
    //                size_t VIndex1=vcg::tri::Index(*this,face[i].V1(j));
    //                std::pair<size_t,size_t> EdgeI(std::min(VIndex0,VIndex1),std::max(VIndex0,VIndex1));
    //                if (InsertedEdges.count(EdgeI)>0)continue;

    //                InsertedEdges.insert(EdgeI);
    //                Valence[VIndex0]++;
    //                Valence[VIndex1]++;
    //                if (!CheckVolume)continue;
    //                if (!HasVolume(&face[i],j))continue;
    //                ValenceVol[VIndex0]++;
    //                ValenceVol[VIndex1]++;
    //            }

    //        vcg::tri::UpdateFlags<QuadMeshC>::VertexClearS(*this);
    //        for (size_t i=0;i<vert.size();i++)
    //        {
    //            vert[i].Irregular=false;
    //            if (vert[i].IsB())continue;
    //            if (Valence[i]==4)continue;
    //            if (Valence[i]==2)continue;
    //            if (CheckVolume && ValenceVol[i]==4)continue;
    //            if (CheckVolume && ValenceVol[i]==2)continue;
    //            if (CheckVolume && ValenceVol[i]==1)continue;
    //            if (CheckVolume && ValenceVol[i]==0)continue;
    //            vert[i].SetS();
    //            vert[i].Irregular=true;
    //        }

    //    }

    void SetEdgeSequences(bool OnlyB)
    {

        if (!OnlyB)
            vcg::PolygonalAlgorithm<PMesh>::SelectIrregularInternal(*this);
        //SelectIrregular();
        else
        {
            //TO BE IMPLEMENTED
            assert(0);
            //            vcg::tri::UpdateFlags<QuadMeshC>::VertexClearS(*this);
            //            vcg::tri::UpdateSelection<MeshType>::VertexCornerBorder(*this,math::ToRad(150.0));
        }
        EdgeSeq.clear();
        Circular.clear();
        //        FirstPos.clear();


        //the sequence of edges kept as  pair of vertices
        std::set<std::pair<size_t,size_t> > VisitedEdges;
        for (size_t i=0;i<face.size();i++)
        {
            int sizeV=face[i].VN();
            for (int j=0;j<sizeV;j++)
            {
                if ((face[i].FFp(j)==&face[i])&&(!OnlyB))continue;
                if ((face[i].FFp(j)!=&face[i])&&(OnlyB))continue;

                size_t IndexV0=vcg::tri::Index(*this,face[i].V(j));
                size_t IndexV1=vcg::tri::Index(*this,face[i].V((j+1)%sizeV));
                std::pair<size_t,size_t> entry(std::min(IndexV0,IndexV1),std::max(IndexV0,IndexV1));

                if (VisitedEdges.count(entry)>0)continue;

                vcg::face::Pos<PolyFace> FirstEdge(&face[i],j);
                //FirstPos.push_back(FirstEdge);

                std::vector<vcg::face::Pos<PolyFace> > CurrSeq;
                bool IsCircular;
                GetEdgeSequenceFrom(FirstEdge,CurrSeq,IsCircular,OnlyB);

                //std::cout<<"SizeSeq "<<CurrSeq.size()<<std::endl;
                Circular.push_back(IsCircular);
                EdgeSeq.push_back(CurrSeq);

                //then insert the edges
                for (size_t k=0;k<CurrSeq.size();k++)
                {
                    FaceType *currF=CurrSeq[k].F();
                    int IndexE=CurrSeq[k].E();
                    int sizeV=currF->VN();
                    VertexType *v0=currF->V(IndexE);
                    VertexType *v1=currF->V((IndexE+1)%sizeV);

                    size_t IndexV0=vcg::tri::Index(*this,v0);
                    size_t IndexV1=vcg::tri::Index(*this,v1);
                    std::pair<size_t,size_t> entry(std::min(IndexV0,IndexV1),std::max(IndexV0,IndexV1));
                    VisitedEdges.insert(entry);
                }
            }
        }
        //std::cout<<"3"<<std::endl;
        //        //check correctness
        //        for (size_t i=0;i<EdgeSeq.size();i++)
        //            for (size_t j=0;j<EdgeSeq[i].size();j++)
        //            {

        //            }


    }

    void TriangulateQuadBySplit()
    {
        for (size_t i=0;i<face.size();i++)
        {
            if(face[i].VN()!=4)continue;
            TriangulateQuadBySplit(i);
        }
    }

    void SplitToQuad(const int IndexF)
    {
        assert(face[IndexF].VN()>4);
        int BestStart=-1;
        ScalarType MaxDev=std::numeric_limits<ScalarType>::max();
        for (int i=0;i<face[IndexF].VN();i++)
        {
            PolyFace TestF;
            TestF.Alloc(4);
            size_t sizeF=face[IndexF].VN();
            for (size_t j=0;j<4;j++)
            {
                size_t curr_index=(i+j)%sizeF;
                TestF.V(j)=face[IndexF].V(curr_index);
            }
            TestF.N()=vcg::PolygonNormal(TestF);
            ScalarType currDev=0;
            for (size_t j=0;j<4;j++)
                currDev+=pow((TestF.V(j)->N()-TestF.N()).Norm(),2);

            //            ScalarType currDev=vcg::PolyAspectRatio(TestF,true);
            if (currDev>MaxDev)continue;
            MaxDev=currDev;
            BestStart=i;
        }
        assert(BestStart!=-1);

        size_t sizeF=face[IndexF].VN();
        //then create the faces
        vcg::tri::Allocator<PMesh>::AddFaces(*this,1);

        size_t size0=4;
        face.back().Alloc(size0);
        for (size_t i=0;i<size0;i++)
        {
            size_t curr_index=(BestStart+i)%sizeF;
            face.back().V(i)=face[IndexF].V(curr_index);
        }

        //then modify the old faced
        int size1=face[IndexF].VN()-2;
        assert(size1>=3);
        std::vector<PolyVertex*> NewV;
        for (int i=0;i<size1;i++)
        {
            size_t curr_index=(BestStart+i+3)%sizeF;
            NewV.push_back(face[IndexF].V(curr_index));
        }
        face[IndexF].Dealloc();
        face[IndexF].Alloc(size1);
        for (int i=0;i<size1;i++)
            face[IndexF].V(i)=NewV[i];
    }

    void SplitToQuad()
    {
        bool AllQuads=true;
        do
        {
            AllQuads=true;
            int sizeF0=face.size();
            for (int i=0;i<sizeF0;i++)
            {
                if (face[i].VN()<=4)continue;
                SplitToQuad(i);
                AllQuads=false;
            }
            //UpdateParametricLines();
            UpdateAttributes();
        }while (!AllQuads);
    }

    void GLDrawEdgeSeq()
    {

        glPushAttrib(GL_ALL_ATTRIB_BITS);

        glDepthRange(0,0.9999);

        glDisable(GL_LIGHTING);
        glDisable(GL_CULL_FACE);
        glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);


        for(unsigned int i=0; i<EdgeSeq.size(); i++)
        {
            vcg::Color4b currC=vcg::Color4b::Scatter(EdgeSeq.size(),i);
            for(unsigned int j=0; j<EdgeSeq[i].size(); j++)
            {

                vcg::glColor(currC);
                glLineWidth(10);
                vcg::face::Pos<FaceType> currP=EdgeSeq[i][j];
                CoordType pos0=currP.V()->P();
                CoordType pos1=currP.VFlip()->P();
                glBegin(GL_LINES);
                vcg::glVertex(pos0);
                vcg::glVertex(pos1);
                glEnd();
            }
        }
        //        for (size_t i=0;i<FirstPos.size();i++)
        //        {
        //            vcg::face::Pos<QuadFaceC> currP=FirstPos[i];
        //            FaceType *f=currP.F();
        //            CoordType pos0=currP.V()->P();
        //            CoordType pos1=currP.VFlip()->P();
        //            pos1=pos0*0.7+pos1*0.3;
        //            CoordType N=f->N();
        //            CoordType DirOtho=(pos0-pos1)^N;
        //            CoordType pos2=pos1+DirOtho;
        //            CoordType faceBary=vcg::PolyBarycenter(*f);
        //            if (((pos2-pos1)*(faceBary-pos1))<0)
        //                pos2=pos1-DirOtho;

        //            CoordType posBary=(pos0+pos1+pos2)/3;
        //            pos0=pos0*0.8+posBary*0.2;
        //            pos1=pos1*0.8+posBary*0.2;
        //            pos2=pos2*0.8+posBary*0.2;

        //            vcg::glColor(vcg::Color4b(255,0,0,255));
        //            glLineWidth(10);
        //            glBegin(GL_LINE_LOOP);
        //            vcg::glVertex(pos0);
        //            vcg::glVertex(pos1);
        //            vcg::glVertex(pos2);
        //            glEnd();
        //        }
        glPopAttrib();

    }

    void SaveEdgeSeq(const char *path)
    {
        FILE *f=fopen(path,"wt");
        assert(f!=NULL);
        fprintf(f,"Num Sequences: %d\n",(int)EdgeSeq.size());
        for(unsigned int i=0; i<EdgeSeq.size(); i++)
        {
            fprintf(f,"Edge Size Seq: %d\n",(int)EdgeSeq[i].size());
            if (Circular[i])
                fprintf(f,"Circular 1\n");
            else
                fprintf(f,"Circular 0\n");
            size_t Vind0=vcg::tri::Index(*this,EdgeSeq[i][0].VFlip());
            fprintf(f,"V %d\n",(int)Vind0);
            for(unsigned int j=0; j<EdgeSeq[i].size(); j++)
            {
                size_t Vind=vcg::tri::Index(*this,EdgeSeq[i][j].V());
                fprintf(f,"V %d\n",(int)Vind);
            }
        }
        fclose(f);
    }

    void GLDraw(bool DrawEdges=true)
    {

        glPushAttrib(GL_ALL_ATTRIB_BITS);

        glDepthRange(0.000001,1);

        glEnable(GL_LIGHTING);

        glDisable(GL_CULL_FACE);
        glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);

        for(unsigned int i=0; i<face.size(); i++)
        {
            vcg::glColor(face[i].C());

            if(face[i].IsD())  continue;
            //if(face[i].Filtered) continue;

            vcg::glNormal(face[i].N());

            glBegin(GL_POLYGON);
            for(int j=0; j<face[i].VN(); j++)
                vcg::glVertex( face[i].V(j)->P() );

            glEnd();

        }

        //DrawEdgeSmoothPairs();

        if (DrawEdges)
        {
            glDepthRange(0.0,0.999999);
            glDisable(GL_LIGHTING);
            for(unsigned int i=0; i<face.size(); i++)
            {
                if(face[i].IsD())  continue;
                int size=face[i].VN();
                for(int j=0; j<face[i].VN(); j++)
                {

                    glLineWidth(4);
                    vcg::glColor(vcg::Color4b(0,0,0,255));

                    CoordType pos0=face[i].V(j)->P();
                    CoordType pos1=face[i].V((j+1)%size)->P();

                    glBegin(GL_LINES);
                    vcg::glVertex( pos0);
                    vcg::glVertex( pos1);
                    glEnd();
                }
            }
        }

        glDepthRange(0.0,0.9999);
        glPointSize(20);
        glBegin(GL_POINTS);
        for (size_t i=0;i<vert.size();i++)
        {
            if (!(vert[i].IsFix || vert[i].IsBound))continue;
            if (vert[i].IsFix)
                vcg::glColor(vcg::Color4b(255,0,255,255));
            if (vert[i].IsBound)
                vcg::glColor(vcg::Color4b(255,255,0,255));

            vcg::glVertex( vert[i].P());
        }
        glEnd();

        glPopAttrib();

    }

    void UpdateNormal()
    {
        vcg::PolygonalAlgorithm<PMesh>::UpdateFaceNormalByFitting(*this);
        vcg::tri::UpdateNormal<PMesh>::PerVertexNormalized(*this);
    }

    void UpdateAttributes()
    {
        UpdateNormal();
        vcg::tri::UpdateBounding<PMesh>::Box(*this);
        vcg::tri::UpdateTopology<PMesh>::FaceFace(*this);
        //vcg::PolygonalAlgorithm<QuadMeshC>::UpdateBorderVertexFromPFFAdj(*this);
        vcg::tri::UpdateFlags<PMesh>::VertexBorderFromFaceAdj(*this);

    }

    void SetFixedConstrainedVertFromBoxes(const std::vector<vcg::Box3<ScalarType> > &FixedBox,
                                          const std::vector<vcg::Box3<ScalarType> > &BoundaryBox)
    {
        for (size_t i=0;i<vert.size();i++)
        {
            vert[i].IsFix=false;
            vert[i].IsBound=false;
            for (size_t j=0;j<FixedBox.size();j++)
            {
                if (FixedBox[j].IsIn(vert[i].P()))
                    vert[i].IsFix=true;
            }
            for (size_t j=0;j<BoundaryBox.size();j++)
            {
                if (BoundaryBox[j].IsIn(vert[i].P()))
                    vert[i].IsBound=true;
            }
        }
    }

    bool ExportToAbaqus(std::string &path)
    {
        std::ofstream myfile;
        myfile.open(path.c_str());
        myfile << "*Heading\n";
        myfile << "** Job name: Job-1 Model name: Model-1\n";
        myfile << "** Generated by: Nico :)\n";
        myfile << "*Preprint, echo=NO, model=NO, history=NO, contact=NO\n";
        myfile << "**\n";
        myfile << "*Part, name=Part-1\n";
        myfile << "*Node\n";
        for (size_t i=0;i<vert.size();i++)
        {
            myfile<<std::setw(7)<<(i+1)<<
                    ","<<std::setw(13)<<vert[i].P().X()<<
                    ","<<std::setw(13)<<vert[i].P().Y()<<
                    ","<<std::setw(13)<<vert[i].P().Z()<<std::endl;
        }

        //write valence 3 faces
        myfile << "*Element, type=S3"<<std::endl;
        int num=0;
        for (size_t i=0;i<face.size();i++)
        {
            if (face[i].VN()!=3)continue;
            num++;
            size_t IndexV0=vcg::tri::Index(*this,face[i].V(0));
            size_t IndexV1=vcg::tri::Index(*this,face[i].V(1));
            size_t IndexV2=vcg::tri::Index(*this,face[i].V(2));
            myfile<<std::setw(4)<<(num)<<
                    ","<<std::setw(4)<<IndexV0+1<<
                    ","<<std::setw(4)<<IndexV1+1<<
                    ","<<std::setw(4)<<IndexV2+1<<std::endl;

        }
        myfile << "*Element, type=S4"<<std::endl;
        for (size_t i=0;i<face.size();i++)
        {
            if (face[i].VN()!=4)continue;
            num++;
            size_t IndexV0=vcg::tri::Index(*this,face[i].V(0));
            size_t IndexV1=vcg::tri::Index(*this,face[i].V(1));
            size_t IndexV2=vcg::tri::Index(*this,face[i].V(2));
            size_t IndexV3=vcg::tri::Index(*this,face[i].V(3));
            myfile<<std::setw(4)<<(num)<<
                    ","<<std::setw(4)<<IndexV0+1<<
                    ","<<std::setw(4)<<IndexV1+1<<
                    ","<<std::setw(4)<<IndexV2+1<<
                    ","<<std::setw(4)<<IndexV3+1<<std::endl;
        }
        myfile << "*Elset, elset=ss, generate"<<std::endl;
        myfile << "   1,"<<std::setw(5)<<num<<",     1"<<std::endl;
        myfile << "** Section: sldSec"<<std::endl;
        myfile << "*Shell Section, elset=ss, material=Material01"<<std::endl;
        myfile << "0.1, 5"<<std::endl;
        myfile << "*End Part"<<std::endl;
        myfile << "**"<<std::endl;
        myfile << "**"<<std::endl;
        myfile << "** ASSEMBLY"<<std::endl;
        myfile << "**"<<std::endl;
        myfile << "*Assembly, name=Assembly"<<std::endl;
        myfile << "**"<<std::endl;
        myfile << "*Instance, name=Part-1-1, part=Part-1"<<std::endl;
        myfile << "*End Instance"<<std::endl;
        myfile << "**"<<std::endl;
        myfile << "*Nset, nset=Set-1, instance=Part-1-1"<<std::endl;
        for (size_t i=0;i<vert.size();i++)
        {
            if (!vert[i].IsFix)continue;
            myfile <<std::setw(3)<<(i+1)<<std::endl;//",";
        }
        //myfile <<std::endl;

        //        *Elset, elset=Set-1, instance=Part-1-1
        //         178, 180, 182, 184, 186, 188, 190, 192, 370, 372, 374, 376, 378, 380, 382, 384
        //        *Nset, nset=Set-2, instance=Part-1-1

        myfile << "*Nset, nset=Set-2, instance=Part-1-1"<<std::endl;
        for (size_t i=0;i<vert.size();i++)
        {
            if (!vert[i].IsBound)continue;
            myfile <<std::setw(3)<<(i+1)<<std::endl;//",";
        }
        //myfile <<std::endl;
        myfile <<"*End Assembly"<<std::endl;
        myfile <<"**"<<std::endl;
        myfile <<"** MATERIALS"<<std::endl;
        myfile <<"**"<<std::endl;
        myfile <<"*Material, name=Material00"<<std::endl;
        myfile <<"*Density"<<std::endl;
        myfile <<"0.,"<<std::endl;
        myfile <<"*Elastic"<<std::endl;
        myfile <<" 1e-09, 0.3"<<std::endl;
        myfile <<"*Material, name=Material01"<<std::endl;
        myfile <<"*Density"<<std::endl;
        myfile <<" 7.8e-11,"<<std::endl;
        myfile <<"*Elastic"<<std::endl;
        myfile <<"1., 0.3"<<std::endl;
        myfile <<"** ----------------------------------------------------------------"<<std::endl;
        myfile <<"**"<<std::endl;
        myfile <<"** STEP: Step-1"<<std::endl;
        myfile <<"**"<<std::endl;
        myfile <<"*Step, name=Step-1, nlgeom=NO"<<std::endl;
        myfile <<"*Static"<<std::endl;
        myfile <<"1., 1., 1e-05, 1."<<std::endl;
        myfile <<"**"<<std::endl;
        myfile <<"** BOUNDARY CONDITIONS"<<std::endl;
        myfile <<"**"<<std::endl;
        myfile <<"** Name: BC-1 Type: Displacement/Rotation"<<std::endl;
        myfile <<"*Boundary"<<std::endl;
        myfile <<"Set-1, 1, 1"<<std::endl;
        myfile <<"Set-1, 2, 2"<<std::endl;
        myfile <<"Set-1, 3, 3"<<std::endl;
        myfile <<"Set-1, 4, 4"<<std::endl;
        myfile <<"Set-1, 5, 5"<<std::endl;
        myfile <<"Set-1, 6, 6"<<std::endl;
        myfile <<"**"<<std::endl;
        myfile <<"** LOADS"<<std::endl;
        myfile <<"**"<<std::endl;
        myfile <<"** Name: Load-1   Type: Concentrated force"<<std::endl;
        myfile <<"*Cload"<<std::endl;
        myfile <<"Set-2, 2, -1."<<std::endl;
        myfile <<"**"<<std::endl;
        myfile <<"** OUTPUT REQUESTS"<<std::endl;
        myfile <<"**"<<std::endl;
        myfile <<"*Restart, write, frequency=0"<<std::endl;
        myfile <<"**"<<std::endl;
        myfile <<"** FIELD OUTPUT: SEDensity"<<std::endl;
        myfile <<"**"<<std::endl;
        myfile <<"*Output, field"<<std::endl;
        myfile <<"*Node Output"<<std::endl;
        myfile <<"U,"<<std::endl;
        myfile <<"*Element Output, directions=YES"<<std::endl;
        myfile <<"ELEDEN, EVOL, S"<<std::endl;
        myfile <<"**"<<std::endl;
        myfile <<"** FIELD OUTPUT: F-Output-1"<<std::endl;
        myfile <<"**"<<std::endl;
        myfile <<"*Output, field, variable=PRESELECT"<<std::endl;
        myfile <<"**"<<std::endl;
        myfile <<"** HISTORY OUTPUT: ExtWork"<<std::endl;
        myfile <<"**"<<std::endl;
        myfile <<"*Output, history"<<std::endl;
        myfile <<"*Energy Output"<<std::endl;
        myfile <<"ALLWK,"<<std::endl;
        myfile <<"**"<<std::endl;
        myfile <<"** HISTORY OUTPUT: H-Output-1"<<std::endl;
        myfile <<"**"<<std::endl;
        myfile <<"*Output, history, variable=PRESELECT"<<std::endl;
        myfile <<"*End Step"<<std::endl;
        myfile.close();
    }
};

class MyTriFace;
class MyTriEdge;
class MyTriVertex;

enum FeatureKind{ETConcave,ETConvex,ETNone};

struct TriUsedTypes: public vcg::UsedTypes<vcg::Use<MyTriVertex>::AsVertexType,
        vcg::Use<MyTriFace>::AsFaceType,
        vcg::Use<MyTriEdge>::AsEdgeType>{};


class MyTriVertex:public vcg::Vertex<TriUsedTypes,
        vcg::vertex::Coord3d,
        vcg::vertex::Color4b,
        vcg::vertex::Normal3d,
        vcg::vertex::VFAdj,
        vcg::vertex::BitFlags,
        vcg::vertex::CurvatureDird,
        vcg::vertex::Qualityd,
        vcg::vertex::TexCoord2d>
{
public:

    bool IsFix;
    bool IsBound;
};

class MyTriEdge:public vcg::Edge<TriUsedTypes,
        vcg::edge::VertexRef,
        vcg::edge::BitFlags>
{};

class MyTriFace:public vcg::Face<TriUsedTypes,
        vcg::face::VertexRef,
        vcg::face::VFAdj,
        vcg::face::FFAdj,
        vcg::face::BitFlags,
        vcg::face::Normal3d,
        vcg::face::CurvatureDird,
        vcg::face::Color4b,
        vcg::face::Qualityd,
        vcg::face::WedgeTexCoord2d,
        vcg::face::Mark>
{
public:
    FeatureKind FKind[3];
};

enum GoemPrecondition{NOVertManifold,NOFaceManifold,
                      DegenerateFace,DegenerateVertex,
                      UnreferencedVert,
                      IsOk};


class MyTriMesh: public vcg::tri::TriMesh< std::vector<MyTriVertex>,
        std::vector<MyTriEdge>,
        std::vector<MyTriFace > >
{
    typedef std::pair<CoordType,CoordType> CoordPair;
    std::set< CoordPair > FeaturesCoord;
public:

    //ScalarType LimitConcave;
    std::vector<vcg::Box3<ScalarType> > FixedBox,BoundaryBox;

private:


    // Basic subdivision class
    struct SplitLev : public   std::unary_function<vcg::face::Pos<FaceType> ,CoordType >
    {
        std::map<CoordPair,CoordType> *SplitOps;

        void operator()(VertexType &nv,vcg::face::Pos<FaceType>  ep)
        {
            VertexType* v0=ep.f->V0(ep.z);
            VertexType* v1=ep.f->V1(ep.z);

            assert(v0!=v1);

            CoordType Pos0=v0->P();
            CoordType Pos1=v1->P();

            CoordPair CoordK(std::min(Pos0,Pos1),std::max(Pos0,Pos1));
            assert(SplitOps->count(CoordK)>0);
            nv.P()=(*SplitOps)[CoordK];
        }

        vcg::TexCoord2<ScalarType> WedgeInterp(vcg::TexCoord2<ScalarType> &t0,
                                               vcg::TexCoord2<ScalarType> &t1)
        {
            return (vcg::TexCoord2<ScalarType>(0,0));
        }

        SplitLev(std::map<CoordPair,CoordType> *_SplitOps){SplitOps=_SplitOps;}
    };

    class EdgePred
    {

        std::map<CoordPair,CoordType> *SplitOps;

    public:

        bool operator()(vcg::face::Pos<FaceType> ep) const
        {
            VertexType* v0=ep.f->V0(ep.z);
            VertexType* v1=ep.f->V1(ep.z);

            assert(v0!=v1);

            CoordType Pos0=v0->P();
            CoordType Pos1=v1->P();

            CoordPair CoordK(std::min(Pos0,Pos1),std::max(Pos0,Pos1));

            return (SplitOps->count(CoordK)>0);
        }

        EdgePred(std::map<CoordPair,CoordType> *_SplitOps){SplitOps=_SplitOps;}
    };

//    void InitEdgeType()
//    {
//        for (size_t i=0;i<face.size();i++)
//            for (size_t j=0;j<3;j++)
//            {
//                if (IsConcaveEdge(face[i],j))
//                    face[i].FKind[j]=ETConcave;
//                else
//                    face[i].FKind[j]=ETConvex;
//            }
//    }

    void InitFeatureCoordsTable()
    {
        FeaturesCoord.clear();
        for (size_t i=0;i<face.size();i++)
        {
            for (size_t j=0;j<3;j++)
            {
                if (!face[i].IsFaceEdgeS(j))continue;
                CoordPair PosEdge(std::min(face[i].P0(j),face[i].P1(j)),
                                  std::max(face[i].P0(j),face[i].P1(j)));
                FeaturesCoord.insert(PosEdge);
            }
        }
    }

    void SetFeatureFromTable()
    {
        for (size_t i=0;i<face.size();i++)
        {
            for (size_t j=0;j<3;j++)
            {
                face[i].ClearFaceEdgeS(j);
                CoordPair PosEdge(std::min(face[i].P0(j),face[i].P1(j)),
                                  std::max(face[i].P0(j),face[i].P1(j)));
                if(FeaturesCoord.count(PosEdge)==0)continue;
                face[i].SetFaceEdgeS(j);
            }
        }
    }

    bool RefineInternalFacesStepFromEdgeSel()
    {
        InitFeatureCoordsTable();
        std::vector<int> to_refine_face;
        for (size_t i=0;i<face.size();i++)
        {
            //find the number of edges
            int Num=0;
            for (size_t j=0;j<3;j++)
            {
                if (!face[i].IsFaceEdgeS(j))continue;
                Num++;
            }
            if (Num==3)
                to_refine_face.push_back(i);
        }
        if (to_refine_face.size()==0)return false;

        std::cout<<"Performing "<<to_refine_face.size()<< " face refinement ops"<<std::endl;
        for (size_t j=0;j<to_refine_face.size();j++)
        {
            int IndexF=to_refine_face[j];
            CoordType PD1=face[IndexF].PD1();
            CoordType PD2=face[IndexF].PD2();
            CoordType NewPos=(face[IndexF].P(0)+
                              face[IndexF].P(1)+
                              face[IndexF].P(2))/3;
            vcg::tri::Allocator<MeshType>::AddVertex(*this,NewPos);
            VertexType *V0=face[IndexF].V(0);
            VertexType *V1=face[IndexF].V(1);
            VertexType *V2=face[IndexF].V(2);
            VertexType *V3=&vert.back();
            face[IndexF].V(2)=V3;
            vcg::tri::Allocator<MeshType>::AddFace(*this,V1,V2,V3);
            face.back().PD1()=PD1;
            face.back().PD2()=PD2;
            vcg::tri::Allocator<MeshType>::AddFace(*this,V2,V0,V3);
            face.back().PD1()=PD1;
            face.back().PD2()=PD2;
        }
        UpdateDataStructures();
        SetFeatureFromTable();
        return true;
    }

//    bool IsConcaveEdge(const FaceType &f0,int IndexE)
//    {
//        FaceType *f1=f0.cFFp(IndexE);
//        if (f1==&f0)return false;
//        CoordType N0=f0.cN();
//        CoordType N1=f1->cN();
//        CoordType EdgeDir=f0.cP1(IndexE)-f0.cP0(IndexE);
//        EdgeDir.Normalize();
//        CoordType Cross=N0^N1;
//        return ((Cross*EdgeDir)<LimitConcave);
//    }


//    bool SplitAdjacentTerminalVertices()
//    {
//        InitFeatureCoordsTable();
//        std::vector<size_t> PerVertConcaveEdge(vert.size(),0);
//        std::vector<size_t> PerVertConvexEdge(vert.size(),0);
//        //count concave vs convex
//        for (size_t i=0;i<face.size();i++)
//        {
//            for (size_t j=0;j<3;j++)
//            {
//                if (!face[i].IsFaceEdgeS(j))continue;
//                size_t IndexV0=vcg::tri::Index(*this,face[i].V0(j));
//                size_t IndexV1=vcg::tri::Index(*this,face[i].V1(j));
//                //only on one side
//                if (IndexV0>IndexV1)continue;
//                if (IsConcaveEdge(face[i],j))
//                {
//                    PerVertConcaveEdge[IndexV0]++;
//                    PerVertConcaveEdge[IndexV1]++;
//                }
//                else
//                {
//                    PerVertConvexEdge[IndexV0]++;
//                    PerVertConvexEdge[IndexV1]++;
//                }
//            }
//        }

//        //count concave vs convex
//        std::map<CoordPair,CoordType> ToBeSplitted;
//        std::set<CoordPair> NewFeatureEdges;
//        for (size_t i=0;i<face.size();i++)
//        {
//            for (size_t j=0;j<3;j++)
//            {
//                if (!face[i].IsFaceEdgeS(j))continue;
//                size_t IndexV0=vcg::tri::Index(*this,face[i].V0(j));
//                size_t IndexV1=vcg::tri::Index(*this,face[i].V1(j));
//                size_t ConcaveEV0=PerVertConcaveEdge[IndexV0];
//                size_t ConcaveEV1=PerVertConcaveEdge[IndexV1];
//                size_t ConvexEV0=PerVertConvexEdge[IndexV0];
//                size_t ConvexEV1=PerVertConvexEdge[IndexV1];
//                size_t NumEV0=ConcaveEV0+ConvexEV0;
//                size_t NumEV1=ConcaveEV1+ConvexEV1;
//                bool IsCornerV0=false;
//                bool IsCornerV1=false;

//                if (NumEV0==1)IsCornerV0=true;
//                if (NumEV0>2)IsCornerV0=true;
//                if ((ConcaveEV0>0)&&(ConvexEV0>0))IsCornerV0=true;

//                if (NumEV1==1)IsCornerV1=true;
//                if (NumEV1>2)IsCornerV1=true;
//                if ((ConcaveEV1>0)&&(ConvexEV1>0))IsCornerV1=true;

//                //                std::cout<<"ConcaveEV0 "<<ConcaveEV0<<std::endl;
//                //                std::cout<<"ConcaveEV1 "<<ConcaveEV1<<std::endl;
//                //                std::cout<<"ConvexEV0 "<<ConvexEV0<<std::endl;
//                //                std::cout<<"ConvexEV1 "<<ConvexEV1<<std::endl;
//                if (IsCornerV0 && IsCornerV1)
//                {
//                    CoordType P0=face[i].P0(j);
//                    CoordType P1=face[i].P1(j);
//                    CoordPair Key(std::min(P0,P1),std::max(P0,P1));
//                    CoordType Mid=(P0+P1)/2;
//                    ToBeSplitted[Key]=Mid;
//                    CoordPair newEdge0(std::min(P0,Mid),std::max(P0,Mid));
//                    CoordPair newEdge1(std::min(P1,Mid),std::max(P1,Mid));
//                    NewFeatureEdges.insert(newEdge0);
//                    NewFeatureEdges.insert(newEdge1);
//                }
//            }
//        }


//        std::cout<<"Performing "<<ToBeSplitted.size()<< " split ops"<<std::endl;
//        if (ToBeSplitted.size()==0)return false;

//        SplitLev splMd(&ToBeSplitted);
//        EdgePred eP(&ToBeSplitted);

//        //do the final split
//        bool done=vcg::tri::RefineE<MeshType,SplitLev,EdgePred>(*this,splMd,eP);

//        //set old features
//        SetFeatureFromTable();

//        //and the new ones
//        for (size_t i=0;i<face.size();i++)
//        {
//            for (size_t j=0;j<3;j++)
//            {
//                CoordType P0=face[i].P0(j);
//                CoordType P1=face[i].P1(j);
//                CoordPair Key(std::min(P0,P1),std::max(P0,P1));
//                if (NewFeatureEdges.count(Key)==0)continue;
//                face[i].SetFaceEdgeS(j);
//            }
//        }
//        return done;
//    }

//    bool SplitAdjacentEdgeSharpFromEdgeSel()
//    {
//        InitFeatureCoordsTable();
//        vcg::tri::UpdateSelection<MeshType>::VertexClear(*this);
//        //InitFaceEdgeSelFromFeatureSeq();

//        std::set<std::pair<CoordType,CoordType> > EdgePos;

//        for (size_t i=0;i<face.size();i++)
//        {
//            for (size_t j=0;j<3;j++)
//            {
//                if (!face[i].IsFaceEdgeS(j))continue;
//                int VIndex0=vcg::tri::Index(*this,face[i].V0(j));
//                int VIndex1=vcg::tri::Index(*this,face[i].V1(j));
//                CoordType P0=vert[VIndex0].P();
//                CoordType P1=vert[VIndex1].P();
//                vert[VIndex0].SetS();
//                vert[VIndex1].SetS();
//                EdgePos.insert(std::pair<CoordType,CoordType>(std::min(P0,P1),std::max(P0,P1)));
//            }
//        }

//        //then save the edges to be splitted
//        std::map<CoordPair,CoordType> ToBeSplitted;
//        for (size_t i=0;i<face.size();i++)
//        {
//            //find the number of edges
//            int Num=0;
//            for (size_t j=0;j<3;j++)
//            {
//                int VIndex0=vcg::tri::Index(*this,face[i].V0(j));
//                int VIndex1=vcg::tri::Index(*this,face[i].V1(j));
//                if ((!vert[VIndex0].IsS())||(!vert[VIndex1].IsS()))continue;
//                CoordType P0=vert[VIndex0].P();
//                CoordType P1=vert[VIndex1].P();
//                std::pair<CoordType,CoordType> Key(std::min(P0,P1),std::max(P0,P1));
//                if (EdgePos.count(Key)==1){Num++;continue;}

//                ToBeSplitted[Key]=(P0+P1)/2;
//            }
//            assert(Num<=2);//this should be already solved
//        }
//        std::cout<<"Performing "<<ToBeSplitted.size()<< " split ops"<<std::endl;

//        SplitLev splMd(&ToBeSplitted);
//        EdgePred eP(&ToBeSplitted);

//        //do the final split
//        bool done=vcg::tri::RefineE<MeshType,SplitLev,EdgePred>(*this,splMd,eP);

//        UpdateDataStructures();
//        SetFeatureFromTable();
//        return done;
//    }

    typedef vcg::tri::FieldSmoother<MyTriMesh> FieldSmootherType;

public:

    void SmoothField(FieldSmootherType::SmoothParam FieldParam)
    {
        InitFeatureCoordsTable();

        FieldParam.sharp_thr=0.0;
        FieldParam.curv_thr=0.0;
        FieldParam.AddConstr.clear();
        for (size_t i=0;i<face.size();i++)
        {
            for (size_t j=0;j<3;j++)
            {
                if (!face[i].IsFaceEdgeS(j))continue;
                size_t IndexV0=vcg::tri::Index(*this,face[i].V0(j));
                size_t IndexV1=vcg::tri::Index(*this,face[i].V1(j));
                //only on one side
                if (IndexV0>IndexV1)continue;
                CoordType P0=face[i].V0(j)->P();
                CoordType P1=face[i].V1(j)->P();
                CoordType Dir=P1-P0;
                Dir.Normalize();
                FieldParam.AddConstr.push_back(std::pair<int,CoordType>(i,Dir));

                typename MeshType::FaceType *FOpp=face[i].FFp(j);
                if (FOpp==&face[i])continue;

                int IndexF=vcg::tri::Index(*this,*FOpp);
                FieldParam.AddConstr.push_back(std::pair<int,CoordType>(IndexF,Dir));
            }
        }

        FieldSmootherType::SmoothDirections(*this,FieldParam);
        vcg::tri::CrossField<MeshType>::OrientDirectionFaceCoherently(*this);
        vcg::tri::CrossField<MeshType>::UpdateSingularByCross(*this);

        UpdateDataStructures();
        SetFeatureFromTable();
    }

//    void RefineIfNeeded()
//    {
//        bool has_refined=false;
//        do
//        {
//            has_refined=false;
//            has_refined|=RefineInternalFacesStepFromEdgeSel();
//            has_refined|=SplitAdjacentEdgeSharpFromEdgeSel();
//            //has_refined|=SplitAdjacentTerminalVertices();
//            //has_refined|=SplitEdgeSharpSharingVerticesFromEdgeSel();
//        }while (has_refined);
//        InitEdgeType();
//    }

    void InitBound()
    {
        for (size_t i=0;i<vert.size();i++)
        {
            vert[i].IsFix=false;
            vert[i].IsBound=false;
        }
    }

    bool LoadTriMesh(const std::string &filename,bool &allQuad)
    {
        allQuad=false;
        Clear();
        if(filename.empty()) return false;
        int position0=filename.find(".ply");
        int position1=filename.find(".obj");
        int position2=filename.find(".off");

        if (position0!=-1)
        {
            int err=vcg::tri::io::ImporterPLY<MyTriMesh>::Open(*this,filename.c_str());
            if (err!=vcg::ply::E_NOERROR)return false;
            InitBound();

            return true;
        }
        if (position1!=-1)
        {
            //            PMesh pmesh;
            //            int Mask;
            //            //std::cout<<"1"<<std::endl;
            //            vcg::tri::io::ImporterOBJ<PMesh>::LoadMask(filename.c_str(), Mask);
            //            //std::cout<<"2"<<std::endl;
            //            int err=vcg::tri::io::ImporterOBJ<PMesh>::Open(pmesh,filename.c_str(),Mask);
            //            if ((err!=0)&&(err!=5))return false;
            //            //check if all quad
            //            allQuad=true;
            //            for (size_t i=0;i<pmesh.face.size();i++)
            //            {
            //                if (pmesh.face[i].VN()==4)continue;
            //                allQuad=false;
            //            }
            //            if (allQuad)
            //            {
            //                for (size_t i=0;i<pmesh.face.size();i++)
            //                {
            //                    CoordType PD1[2];
            //                    PD1[0]=(pmesh.face[i].V(0)->P()-pmesh.face[i].V(1)->P());
            //                    PD1[1]=(pmesh.face[i].V(3)->P()-pmesh.face[i].V(2)->P());
            //                    PD1[0].Normalize();
            //                    PD1[1].Normalize();
            //                    CoordType PD2[2];
            //                    PD2[0]=(pmesh.face[i].V(2)->P()-pmesh.face[i].V(1)->P());
            //                    PD2[1]=(pmesh.face[i].V(3)->P()-pmesh.face[i].V(0)->P());
            //                    PD2[0].Normalize();
            //                    PD2[1].Normalize();
            //                    pmesh.face[i].PD1()=PD1[0]+PD1[1];
            //                    pmesh.face[i].PD2()=PD2[0]+PD2[1];
            //                    pmesh.face[i].PD1().Normalize();
            //                    pmesh.face[i].PD2().Normalize();
            //                }
            //                size_t size=pmesh.fn;
            //                //vcg::PolygonalAlgorithm<PMesh>::Triangulate(pmesh,false);

            //                pmesh.TriangulateQuadBySplit();
            //                for (size_t i=0;i<size;i++)
            //                {
            //                    pmesh.face[i+size].PD1()=pmesh.face[i].PD1();
            //                    pmesh.face[i+size].PD2()=pmesh.face[i].PD2();
            //                }
            //                //then copy the field
            //                Clear();
            //                vcg::tri::Allocator<MyTriMesh>::AddVertices(*this,pmesh.vn);
            //                vcg::tri::Allocator<MyTriMesh>::AddFaces(*this,pmesh.fn);

            //                for (size_t i=0;i<pmesh.vert.size();i++)
            //                    vert[i].P()=pmesh.vert[i].P();

            //                for (size_t i=0;i<pmesh.face.size();i++)
            //                {
            //                    size_t IndexV0=vcg::tri::Index(pmesh,pmesh.face[i].V(0));
            //                    size_t IndexV1=vcg::tri::Index(pmesh,pmesh.face[i].V(1));
            //                    size_t IndexV2=vcg::tri::Index(pmesh,pmesh.face[i].V(2));
            //                    face[i].V(0)=&vert[IndexV0];
            //                    face[i].V(1)=&vert[IndexV1];
            //                    face[i].V(2)=&vert[IndexV2];
            //                    face[i].PD1()=pmesh.face[i].PD1();
            //                    face[i].PD2()=pmesh.face[i].PD2();
            //                }
            //                UpdateDataStructures();
            //                for (size_t i=0;i<face.size();i++)
            //                {
            //                    face[i].PD1()-=face[i].N()*(face[i].PD1()*face[i].N());
            //                    face[i].PD2()-=face[i].N()*(face[i].PD2()*face[i].N());
            //                    face[i].PD1().Normalize();
            //                    face[i].PD2().Normalize();
            //                    CoordType Avg=face[i].PD1()+face[i].PD2();
            //                    Avg.Normalize();
            //                    CoordType Avg1=face[i].N()^Avg;
            //                    Avg1.Normalize();
            //                    face[i].PD1()=Avg+Avg1;
            //                    face[i].PD1().Normalize();
            //                    face[i].PD2()=face[i].N()^face[i].PD1();
            //                }
            //                vcg::tri::CrossField<MyTriMesh>::OrientDirectionFaceCoherently(*this);
            //                vcg::tri::CrossField<MyTriMesh>::UpdateSingularByCross(*this);
            //                InitBound();

            //                return true;
            //            }
            //            else
            //            {
            int mask;
            vcg::tri::io::ImporterOBJ<MyTriMesh>::LoadMask(filename.c_str(),mask);
            int err=vcg::tri::io::ImporterOBJ<MyTriMesh>::Open(*this,filename.c_str(),mask);
            InitBound();
            //                edge.clear();
            //                en=0;
            if ((err!=0)&&(err!=5))return false;
            return true;
            //            }
        }
        if (position2!=-1)
        {
            int err=vcg::tri::io::ImporterOFF<MyTriMesh>::Open(*this,filename.c_str());
            InitBound();

            if (err!=0)return false;
            return true;
        }
        return false;
    }

    void NormalizeMagnitudo()
    {
        for (size_t i=0;i<face.size();i++)
        {
            face[i].PD1().Normalize();
            face[i].PD2().Normalize();
        }
        for (size_t i=0;i<vert.size();i++)
        {
            vert[i].PD1().Normalize();
            vert[i].PD2().Normalize();
        }
    }

    void ScaleMagnitudo()
    {
        for (size_t i=0;i<face.size();i++)
        {
            face[i].PD1()*=face[i].K1();
            face[i].PD2()*=face[i].K2();
        }
        for (size_t i=0;i<vert.size();i++)
        {
            vert[i].PD1()*=vert[i].K1();
            vert[i].PD2()*=vert[i].K2();
        }
    }

    bool LoadBLK(const std::string &field_filename)
    {
        MeshType vertMesh;

        std::cout<<"Loading "<<field_filename.c_str()<<std::endl;
        FILE *f=NULL;
        f=fopen(field_filename.c_str(),"rt");
        if (f==NULL){std::cout<<"Not Loaded"<<std::endl;return false;}
        char header[200];
        fgets(header, 200, f);
        //std::cout<<"Header "<<header<<std::endl;
        vcg::tri::UpdateSelection<MyTriMesh>::VertexClear(*this);
        //for (size_t i=0;i<vert.size();i++)
        while(!feof(f))
        {
            float Px,Py,Pz,Mag1,V1x,V1y,V1z,Mag2,V2x,V2y,V2z;
            //            std::cout<<"Dir1 "<<V1x<<","<<V1y<<","<<V1z<<std::endl;
            //            std::cout<<"Dir2 "<<V2x<<","<<V2y<<","<<V2z<<std::endl;

            fscanf(f,"%f %f %f %f %f %f %f %f %f %f %f \n",
                   &Px,&Py,&Pz,&Mag1,&V1x,&V1y,&V1z,&Mag2,&V2x,&V2y,&V2z);
            CoordType VPos(Px,Py,Pz);
            vcg::tri::Allocator<MeshType>::AddVertex(vertMesh,VPos);

            vertMesh.vert.back().K1()=(Mag1);
            vertMesh.vert.back().K2()=(Mag2);

            vertMesh.vert.back().PD1()=CoordType(V1x,V1y,V1z);
            vertMesh.vert.back().PD1().Normalize();
            vertMesh.vert.back().PD2()=CoordType(V2x,V2y,V2z);
            vertMesh.vert.back().PD2().Normalize();
            //vertMesh.vert.back().SetS();
        }
        fclose(f);
        std::cout<<"Assigning to Vertices "<<std::endl;
        vcg::GridStaticPtr<VertexType,ScalarType> Grid;
        Grid.Set(vertMesh.vert.begin(),vertMesh.vert.end());

        for (size_t i=0;i<vert.size();i++)
        {
            ScalarType currD;
            VertexType *foundV=vcg::tri::GetClosestVertex(vertMesh,Grid,vert[i].P(),this->bbox.Diag(),currD);
            assert(foundV!=NULL);
            vert[i].K1()=foundV->K1();
            vert[i].K2()=foundV->K2();
            vert[i].PD1()=foundV->PD1();
            vert[i].PD2()=foundV->PD2();
        }

        vcg::tri::CrossField<MyTriMesh>::SetFaceCrossVectorFromVert(*this);
        //vcg::tri::UpdateSelection<MyTriMesh>::VertexClear(*this);


        ScaleMagnitudo();
        //exit(0);
        return true;
    }

    void SelectToFixVertices()
    {
        vcg::tri::UpdateSelection<MyTriMesh>::VertexClear(*this);
        std::vector<size_t> NumSel(vert.size(),0);

        for (size_t i=0;i<face.size();i++)
            for (size_t j=0;j<3;j++)
            {
                size_t IndexV0=vcg::tri::Index(*this,face[i].V0(j));
                size_t IndexV1=vcg::tri::Index(*this,face[i].V1(j));

                bool IsConstr0=(vert[IndexV0].IsFix)||(vert[IndexV0].IsBound);
                bool IsConstr1=(vert[IndexV1].IsFix)||(vert[IndexV1].IsBound);

                if (!(IsConstr0 && IsConstr1))
                    continue;

                if (!vcg::face::IsBorder(face[i],j)&&(IndexV0>IndexV1))
                    continue;
                NumSel[IndexV0]++;
                NumSel[IndexV1]++;
            }

        for (size_t i=0;i<vert.size();i++)
        {
            bool IsConstr=(vert[i].IsFix)||(vert[i].IsBound);
            if (!IsConstr)continue;
            if (NumSel[i]>=2)continue;
            std::cout<<"Selected"<<std::endl;
            vert[i].SetS();
        }
    }

    size_t GetClosestV(const CoordType &currP)
    {
        ScalarType minD=std::numeric_limits<ScalarType>::max();
        int IndexV=-1;
        for (size_t i=0;i<vert.size();i++)
        {
            ScalarType currD=(vert[i].P()-currP).Norm();
            if (currD>minD)continue;
            minD=currD;
            IndexV=(int)i;
        }
        assert(IndexV>=0);
        return IndexV;
    }

    void GetInBoxV(const vcg::Box3<ScalarType> &BBox,
                   std::vector<size_t> &IndexV)
    {
        for (size_t i=0;i<vert.size();i++)
        {
            if (BBox.IsIn(vert[i].P()))
                IndexV.push_back(i);
        }
    }

    void LoadBoundaryConditions(const std::string &load_filename)
    {
        FILE *f=fopen(load_filename.c_str(),"rt");
        int numFix;
        fscanf(f, "%d\n",&numFix);
        for (int i=0;i<numFix;i++)
        {
            float x,y,z;
            fscanf(f, "%f,%f,%f\n",&x,&y,&z);
            size_t fixV=GetClosestV(CoordType(x,y,z));
            assert(fixV>=0);
            assert(fixV<vert.size());
            vert[fixV].IsFix=true;
            std::cout<<"Fix:"<<fixV<<std::endl;
        }
        int numBound;
        fscanf(f, "%d\n",&numBound);
        for (int i=0;i<numBound;i++)
        {
            float x,y,z;
            fscanf(f, "%f,%f,%f\n",&x,&y,&z);
            size_t boundV=GetClosestV(CoordType(x,y,z));
            assert(boundV>=0);
            assert(boundV<vert.size());
            vert[boundV].IsBound=true;
            std::cout<<"Bound:"<<boundV<<std::endl;
        }
    }

    void LoadBoxBoundaryConditions(const std::string &load_filename)
    {
        FixedBox.clear();
        BoundaryBox.clear();

        FILE *f=fopen(load_filename.c_str(),"rt");
        int numFix;
        fscanf(f, "%d\n",&numFix);
        for (int i=0;i<numFix;i++)
        {
            float xMin,yMin,zMin;
            float xMax,yMax,zMax;
            fscanf(f, "%f,%f,%f,%f,%f,%f\n",&xMin,&yMin,&zMin,&xMax,&yMax,&zMax);
            vcg::Box3<ScalarType> BB;
            BB.min=CoordType(xMin,yMin,zMin);
            BB.max=CoordType(xMax,yMax,zMax);
            FixedBox.push_back(BB);
            std::vector<size_t> IndexV;
            GetInBoxV(BB,IndexV);
            for (size_t i=0;i<IndexV.size();i++)
                vert[IndexV[i]].IsFix=true;

            //std::cout<<"Fix:"<<fixV<<std::endl;
        }
        int numBound;
        fscanf(f, "%d\n",&numBound);
        for (int i=0;i<numBound;i++)
        {
            float xMin,yMin,zMin;
            float xMax,yMax,zMax;
            fscanf(f, "%f,%f,%f,%f,%f,%f\n",&xMin,&yMin,&zMin,&xMax,&yMax,&zMax);
            vcg::Box3<ScalarType> BB;
            BB.min=CoordType(xMin,yMin,zMin);
            BB.max=CoordType(xMax,yMax,zMax);
            BoundaryBox.push_back(BB);
            std::vector<size_t> IndexV;
            GetInBoxV(BB,IndexV);
            for (size_t i=0;i<IndexV.size();i++)
                vert[IndexV[i]].IsBound=true;
        }
    }


    ScalarType GetFieldNormPercentile(typename MeshType::ScalarType percentile,
                                      bool Abs=false)
    {
        std::vector<ScalarType> FieldVal;
        for (size_t i=0;i<face.size();i++)
        {
            //            ScalarType Norm1=face[i].PD1().Norm();
            //            ScalarType Norm2=face[i].PD2().Norm();
            ScalarType Norm1=face[i].K1();
            if (Abs)Norm1=fabs(Norm1);
            ScalarType Norm2=face[i].K2();
            if (Abs)Norm2=fabs(Norm2);
            FieldVal.push_back(Norm1);
            FieldVal.push_back(Norm2);
        }
        std::sort(FieldVal.begin(),FieldVal.end());
        int IndexF=floor(FieldVal.size()*percentile+0.5);
        return(FieldVal[IndexF]);
    }

    bool LoadField(std::string field_filename,bool &IsDimensioned)
    {
        int position0=field_filename.find(".ffield");
        int position1=field_filename.find(".rosy");
        int position2=field_filename.find(".blk");
        IsDimensioned=false;

        if (position0!=-1)
        {
            bool loaded=vcg::tri::io::ImporterFIELD<MyTriMesh>::LoadFFIELD(*this,field_filename.c_str());
            if (!loaded)return false;
            vcg::tri::CrossField<MyTriMesh>::OrientDirectionFaceCoherently(*this);
            vcg::tri::CrossField<MyTriMesh>::UpdateSingularByCross(*this);
            return true;
        }
        if (position1!=-1)
        {
            std::cout<<"Importing ROSY field"<<std::endl;
            bool loaded=vcg::tri::io::ImporterFIELD<MyTriMesh>::Load4ROSY(*this,field_filename.c_str());
            std::cout<<"Imported ROSY field"<<std::endl;
            if (!loaded)return false;
            vcg::tri::CrossField<MyTriMesh>::OrientDirectionFaceCoherently(*this);
            vcg::tri::CrossField<MyTriMesh>::UpdateSingularByCross(*this);
            return true;
        }
        if (position2!=-1)
        {
            IsDimensioned=true;
            std::cout<<"Importing Laccone field"<<std::endl;
            bool loaded=LoadBLK(field_filename);
            //            bool loaded=vcg::tri::io::ImporterFIELD<CMesh>::Load4ROSY(*this,field_filename.c_str());
            std::cout<<"Imported Laccone field"<<std::endl;
            if (!loaded)return false;

            std::cout<<"Normalizign Magnitudo"<<std::endl;

            NormalizeMagnitudo();
            vcg::tri::CrossField<MyTriMesh>::OrientDirectionFaceCoherently(*this);
            vcg::tri::CrossField<MyTriMesh>::UpdateSingularByCross(*this);
            ScaleMagnitudo();
            return true;
        }
        //        if (position3!=-1)
        //        {
        //            IsDimensioned=true;
        //            std::cout<<"Importing Laccone interpolated field"<<std::endl;
        //            bool loaded=LoadBLI(field_filename);
        //            std::cout<<"Imported Laccone Interpolated field"<<std::endl;
        //            if (!loaded)return false;
        //            NormalizeMagnitudo();
        //            vcg::tri::CrossField<MyTriMesh>::OrientDirectionFaceCoherently(*this);
        //            vcg::tri::CrossField<MyTriMesh>::UpdateSingularByCross(*this);
        //            ScaleMagnitudo();
        //            return true;
        //        }
        return false;
    }

    bool SaveSharpFeatures(const std::string &filename)
    {
        if(filename.empty()) return false;
        std::ofstream myfile;
        myfile.open (filename.c_str());
        size_t num=0;
        for (size_t i=0;i<face.size();i++)
            for (size_t j=0;j<3;j++)
            {
                if (!face[i].IsFaceEdgeS(j))continue;
                num++;
            }
        myfile <<num<<std::endl;
        for (size_t i=0;i<face.size();i++)
            for (size_t j=0;j<3;j++)
            {
                if (!face[i].IsFaceEdgeS(j))continue;
                if (face[i].FKind[j]==ETConcave)
                    myfile <<"0,"<< i <<","<<j<<std::endl;
                else
                    myfile <<"1,"<< i <<","<<j<<std::endl;
            }
        myfile.close();
        return true;
    }

    bool SaveTriMesh(const std::string &filename)
    {
        if(filename.empty()) return false;
        int position0=filename.find(".ply");
        int position1=filename.find(".obj");
        int position2=filename.find(".off");

        if (position0!=-1)
        {
            int err=vcg::tri::io::ExporterPLY<MyTriMesh>::Save(*this,filename.c_str());
            if (err!=vcg::ply::E_NOERROR)return false;
            return true;
        }
        if (position1!=-1)
        {
            int mask=0;
            int err=vcg::tri::io::ExporterOBJ<MyTriMesh>::Save(*this,filename.c_str(),mask);
            if ((err!=0)&&(err!=5))return false;
            return true;
        }
        if (position2!=-1)
        {
            int err=vcg::tri::io::ExporterOFF<MyTriMesh>::Save(*this,filename.c_str());
            if (err!=0)return false;
            return true;
        }
        return false;
    }

    bool SaveField(const std::string &filename)
    {
        if(filename.empty()) return false;
        vcg::tri::io::ExporterFIELD<MyTriMesh>::Save4ROSY(*this,filename.c_str());
        return true;
    }

    GoemPrecondition CheckPreconditions()
    {
        int Num=vcg::tri::Clean<MyTriMesh>::CountNonManifoldVertexFF(*this);
        if (Num>0)return NOVertManifold;
        Num=vcg::tri::Clean<MyTriMesh>::CountNonManifoldEdgeFF(*this);
        if (Num>0)return NOVertManifold;
        Num=vcg::tri::Clean<MyTriMesh>::RemoveDegenerateFace(*this);
        if (Num>0)return DegenerateFace;
        Num=vcg::tri::Clean<MyTriMesh>::RemoveDegenerateVertex(*this);
        if (Num>0)return DegenerateVertex;
        Num=vcg::tri::Clean<MyTriMesh>::RemoveUnreferencedVertex(*this);
        if (Num>0)return UnreferencedVert;
        return IsOk;
    }

    //VCG UPDATING STRUCTURES
    void UpdateDataStructures()
    {
        vcg::tri::UpdateBounding<MyTriMesh>::Box(*this);
        vcg::tri::UpdateNormal<MyTriMesh>::PerVertexNormalizedPerFace(*this);
        vcg::tri::UpdateNormal<MyTriMesh>::PerFaceNormalized(*this);
        vcg::tri::UpdateTopology<MyTriMesh>::FaceFace(*this);
        vcg::tri::UpdateTopology<MyTriMesh>::VertexFace(*this);
        vcg::tri::UpdateFlags<MyTriMesh>::FaceBorderFromFF(*this);
        vcg::tri::UpdateFlags<MyTriMesh>::VertexBorderFromFaceBorder(*this);
    }

    void InitSharpFeatures(ScalarType SharpAngleDegree)
    {
        vcg::tri::UpdateFlags<MeshType>::FaceEdgeSelCrease(*this,vcg::math::ToRad(SharpAngleDegree));
        //InitEdgeType();
        for (size_t i=0;i<face.size();i++)
            for (size_t j=0;j<3;j++)
            {
                if (vcg::face::IsBorder(face[i],j))
                {
                    face[i].SetFaceEdgeS(j);
                    face[i].FKind[j]=ETConvex;
                }
            }
    }

    //    void InitAnisotropyOnQ(const ScalarType MinABSFieldVal,
    //                           const ScalarType MaxABSFieldVal,
    //                           const ScalarType MinFieldVal,
    //                           const ScalarType MaxFieldVal,
    //                           const ScalarType ClampVal)
    //    {
    //        assert(MinABSFieldVal>0);
    //        assert(MaxABSFieldVal>0);
    //        for (size_t i=0;i<face.size();i++)
    //        {
    ////            ScalarType Norm1=face[i].PD1().Norm();
    ////            ScalarType Norm2=face[i].PD2().Norm();
    //            ScalarType Norm1=face[i].K1();
    //            ScalarType Norm2=face[i].K2();

    //            //clamp values
    //            Norm1=std::max(Norm1,MinFieldVal);
    //            Norm1=std::min(Norm1,MaxFieldVal);
    //            Norm2=std::max(Norm2,MinFieldVal);
    //            Norm2=std::min(Norm2,MaxFieldVal);

    //            Norm1-=MinFieldVal;
    //            Norm2-=MinFieldVal;

    //            ScalarType MinV=std::min(Norm1,Norm2);
    //            ScalarType MaxV=std::max(Norm1,Norm2);

    //            ScalarType MaxABS=std::max(fabs(Norm1),fabs(Norm2));
    //            //ScalarType minABS=std::min(fabs(Norm1),fabs(Norm2));

    //            if (MaxABS<MinABSFieldVal)
    //                face[i].Q()=0;
    //            else
    //            {
    //                face[i].Q()=MaxV/MinV;
    //            }
    //            //clamp it
    //            face[i].Q()=std::min(face[i].Q(),ClampVal);
    //        }
    //        //normalize it
    //        for (size_t i=0;i<face.size();i++)
    //            face[i].Q()/=ClampVal;

    //        vcg::tri::UpdateQuality<MyTriMesh>::VertexFromFace(*this);


    ////        for (size_t i=0;i<vert.size();i++)
    ////            vert[i].Q()/=ClampVal;
    //    }

    void InitAnisotropyOnQ(const ScalarType MinABSFieldVal,
                           const ScalarType MaxABSFieldVal,
                           const ScalarType MinFieldVal,
                           const ScalarType MaxFieldVal,
                           const ScalarType ClampVal)
    {
        assert(MinABSFieldVal>0);
        assert(MaxABSFieldVal>0);
        for (size_t i=0;i<face.size();i++)
        {
            ScalarType Norm1=face[i].K1();
            ScalarType Norm2=face[i].K2();

            //clamp values
            Norm1=std::max(Norm1,MinFieldVal);
            Norm1=std::min(Norm1,MaxFieldVal);
            Norm2=std::max(Norm2,MinFieldVal);
            Norm2=std::min(Norm2,MaxFieldVal);


            ScalarType MaxABS=std::max(fabs(Norm1),fabs(Norm2));
            ScalarType MinABS=std::min(fabs(Norm1),fabs(Norm2));

            if (MaxABS<MinABSFieldVal)
                face[i].Q()=0;
            else
            {
                if ((Norm1*Norm2)<0)
                    MinABS=MinABSFieldVal;

                MinABS=std::max(MinABS,MinABSFieldVal);

                face[i].Q()=MaxABS/MinABS;
            }
            //clamp it
            face[i].Q()=std::min(face[i].Q(),ClampVal);
        }
        //normalize it
        for (size_t i=0;i<face.size();i++)
            face[i].Q()/=ClampVal;

        vcg::tri::UpdateQuality<MyTriMesh>::VertexFromFace(*this);

    }

    void SetEdgeDirection(FaceType *f,int edge)
    {
        CoordType dir=f->P0(edge)-f->P1(edge);
        dir.Normalize();
        ScalarType prod1=fabs(dir*f->PD1());
        ScalarType prod2=fabs(dir*f->PD2());
        if (prod1>prod2)
        {
            f->PD1()=dir;
            f->PD2()=f->N()^dir;
        }else
        {
            f->PD2()=dir;
            f->PD1()=f->N()^dir;
        }
    }

    void InitFieldBoundaryConstraint()
    {
        vcg::tri::UpdateSelection<MyTriMesh>::FaceClear(*this);
        for (size_t i=0;i<face.size();i++)
            for (int j=0;j<face[i].VN();j++)
            {
                FaceType *f0=&face[i];
                FaceType *f1=f0->FFp(j);
                assert(f1!=NULL);
                if (f0!=f1)continue;
                SetEdgeDirection(f0,j);
                f0->SetS();
            }
    }

    void GLDrawSharpEdges()
    {
        glPushAttrib(GL_ALL_ATTRIB_BITS);
        glDisable(GL_LIGHTING);
        glDepthRange(0,0.9999);
        glLineWidth(5);
        glBegin(GL_LINES);
        for (size_t i=0;i<face.size();i++)
            for (size_t j=0;j<3;j++)
            {
                if (!face[i].IsFaceEdgeS(j))continue;

                if (face[i].FKind[j]==ETConcave)
                    vcg::glColor(vcg::Color4b(255,0,255,255));
                else
                    vcg::glColor(vcg::Color4b(255,255,0,255));

                CoordType Pos0=face[i].P0(j);
                CoordType Pos1=face[i].P1(j);
                vcg::glVertex(Pos0);
                vcg::glVertex(Pos1);
            }
        glEnd();
        glPopAttrib();
    }

    void GLDrawBoundaryConditions()
    {
        glPushAttrib(GL_ALL_ATTRIB_BITS);
        glDisable(GL_LIGHTING);

        glDepthRange(0,0.9999);
        glPointSize(20);

        glBegin(GL_POINTS);
        for (size_t i=0;i<vert.size();i++)
        {
            if (!(vert[i].IsFix || vert[i].IsBound))continue;
            if (vert[i].IsFix)
                vcg::glColor(vcg::Color4b(255,0,255,255));
            else
                vcg::glColor(vcg::Color4b(255,255,0,255));

            vcg::glVertex( vert[i].P());
        }
        glEnd();
        glPopAttrib();
    }


};


#endif

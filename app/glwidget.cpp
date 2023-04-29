/****************************************************************************
**
** Copyright (C) 2011 Nokia Corporation and/or its subsidiary(-ies).
** All rights reserved.
** Contact: Nokia Corporation (qt-info@nokia.com)
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of Nokia Corporation and its Subsidiary(-ies) nor
**     the names of its contributors may be used to endorse or promote
**     products derived from this software without specific prior written
**     permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
** $QT_END_LICENSE$
**
****************************************************************************/

#include <GL/glew.h>
#include <QMouseEvent>

#include <math.h>
#include "glwidget.h"


#include <wrap/igl/smooth_field.h>
#include <triangle_mesh_type.h>
#include <wrap/qt/trackball.h>
#include <wrap/gl/picking.h>
#include <wrap/qt/anttweakbarMapper.h>
#include <wrap/io_trimesh/import_field.h>
#include <wrap/io_trimesh/export_field.h>
#include <wrap/io_trimesh/export.h>
#include <wrap/gl/trimesh.h>
//#include <vcg/complex/algorithms/parametrization/tangent_field_operators.h>
#include <wrap/gl/gl_field.h>
//#include <AutoRemesher.h>
#include <wrap/igl/miq_parametrization.h>
#include <vcg/complex/algorithms/quadrangulator.h>
#include <vcg/complex/algorithms/dual_meshing.h>
#include <vcg/complex/algorithms/polygonal_algorithms.h>
#include "quad_refiner.h"

std::string pathM="";
std::string pathF="";
std::string pathL="";

vcg::Trackball track;//the active manipulator

bool drawfield=false;
bool drawmagnitudo=true;
bool drawsingularities=false;

MyTriMesh tri_mesh;
PMesh quad_mesh;
bool snapBorder=false;
bool alignFieldBorder=false;

//MyTriMesh remeshed_mesh;
//MyTriMesh original_mesh;

TwBar *barQuad;

vcg::GlTrimesh<MyTriMesh> glWrap;
vcg::GLField<MyTriMesh> glField;

vcg::GLW::DrawMode drawmode=vcg::GLW::DMFlatWire;     /// the current drawmode

typedef typename MyTriMesh::ScalarType ScalarType;
typedef typename MyTriMesh::CoordType CoordType;

//int Iterations;
//ScalarType EdgeStep;
//ScalarType Multiplier=2;

//ScalarType SharpDegree=45;
//ScalarType CornerDegree=45;

ScalarType MinFieldVal=0;
ScalarType MaxFieldVal=0;

ScalarType MinABSFieldVal=0;
ScalarType MaxABSFieldVal=0;

int xMouse,yMouse;

ScalarType DefaultGradient=50;

vcg::GridStaticPtr<MyTriMesh::FaceType,MyTriMesh::ScalarType> Gr;
QuadRefiner<PMesh> QRef(quad_mesh);

typedef vcg::tri::FieldSmoother<MyTriMesh> FieldSmootherType;
FieldSmootherType::SmoothParam FieldParam;

//AutoRemesher<MyTriMesh>::Params RemPar;
vcg::tri::MiQParametrizer<MyTriMesh>::MIQParameters MiqP;

bool quadrangulated=false;
double smallVal=0;
double MinF,MaxF=0;
double SmoothGamma=0.1;
bool do_batch=false;
bool draw_seq=false;
ScalarType miqAnisotropy=0.2;

std::set<std::pair<size_t,size_t> > SelEdges;

void DoCollapseSmall()
{
    vcg::PolygonalAlgorithm<PMesh>::CollapseBorderSmallEdges(quad_mesh,smallVal);
    quadrangulated=true;
    quad_mesh.UpdateAttributes();
    quad_mesh.SetEdgeSequences(false);
    quad_mesh.SetFixedConstrainedVertFromBoxes(tri_mesh.FixedBox,tri_mesh.BoundaryBox);
}

void TW_CALL CollapseSmallEdges(void *)
{
    DoCollapseSmall();
}

void DoRefineFaces()
{
    QRef.Refine();
    PMesh quad_swap;
    vcg::tri::Append<PMesh,PMesh>::Mesh(quad_swap,quad_mesh);
    //DualMeshing<PMesh>::MakeDual(quad_swap,quad_mesh);
    quad_mesh.UpdateAttributes();
    quad_mesh.SetEdgeSequences(false);
    quad_mesh.SetFixedConstrainedVertFromBoxes(tri_mesh.FixedBox,tri_mesh.BoundaryBox);
}

void TW_CALL RefineBigFaces(void *)
{
    DoRefineFaces();
}


void DoMiqQuadrangulate()
{
    tri_mesh.NormalizeMagnitudo();

    tri_mesh.SelectToFixVertices();
    MiqP.round_selected=snapBorder;
    MiqP.crease_as_feature=snapBorder;
    MiqP.miqAnisotropy=miqAnisotropy;
    if (snapBorder)
    {
        for (size_t i=0;i<tri_mesh.face.size();i++)
            for (size_t j=0;j<3;j++)
            {
                tri_mesh.face[i].ClearFaceEdgeS(j);
                if (!vcg::face::IsBorder(tri_mesh.face[i],j))continue;
                tri_mesh.face[i].SetFaceEdgeS(j);
            }
    }

    //MiqP.crease_as_feature
    vcg::tri::MiQParametrizer<MyTriMesh>::MIQParametrize(tri_mesh,MiqP);//,MaxFeature);
    vcg::tri::Quadrangulator<MyTriMesh,PMesh> Quadr;

    std::cout<<"Quadrangulating"<<std::endl;
    MyTriMesh splitted_mesh;
    vcg::tri::Append<MyTriMesh,MyTriMesh>::Mesh(splitted_mesh,tri_mesh);
    std::vector< std::vector< short int> > AxisUV;
    Quadr.Quadrangulate(splitted_mesh,quad_mesh,AxisUV,true);

    vcg::PolygonalAlgorithm<PMesh>::RemoveValence2Faces(quad_mesh);
    vcg::PolygonalAlgorithm<PMesh>::RemoveValence2Vertices(quad_mesh,25);
    //vcg::PolygonalAlgorithm<PMesh>::RemoveDuplicatedFVertices(quad_mesh);
    vcg::tri::Clean<PMesh>::RemoveUnreferencedVertex(quad_mesh);
    vcg::tri::Allocator<PMesh>::CompactEveryVector(quad_mesh);
    quad_mesh.UpdateAttributes();

    quadrangulated=true;
    quad_mesh.UpdateAttributes();

    std::cout<<"Initializing Sequences"<<std::endl;


    quad_mesh.SetEdgeSequences(false);

    tri_mesh.ScaleMagnitudo();

    quad_mesh.SetFixedConstrainedVertFromBoxes(tri_mesh.FixedBox,tri_mesh.BoundaryBox);
    //vcg::tri::io::ExporterOBJ<PMesh>::Save(quad_mesh,"quadrangulated.obj",vcg::tri::io::Mask::IOM_BITPOLYGONAL);
}

void TW_CALL MiqQuadrangulate(void *)
{
    DoMiqQuadrangulate();
}

//void TW_CALL SplitQuad(void *)
//{
//    quad_mesh.SplitToQuad();
//    quad_mesh.UpdateAttributes();
//    quad_mesh.SetFixedConstrainedVertFromBoxes(tri_mesh.FixedBox,tri_mesh.BoundaryBox);
//}

//void TW_CALL AutoRemesh(void *)
//{
//    int remesher_iterations=15;
//    ScalarType remesher_aspect_ratio=0.3;
//    int remesher_termination_delta=10000;

//    //    ScalarType sharp_feature_thr=35;
//    //    int feature_erode_dilate=4;

//    tri_mesh.UpdateDataStructures();
//    AutoRemesher<MyTriMesh>::Params RemPar;
//    RemPar.iterations   = remesher_iterations;
//    RemPar.targetAspect = remesher_aspect_ratio;
//    RemPar.targetDeltaFN= remesher_termination_delta;
//    RemPar.userSelectedCreases = true;
//    RemPar.surfDistCheck = true;

//    std::shared_ptr<MyTriMesh> clean = AutoRemesher<MyTriMesh>::CleanMesh(tri_mesh,true);

//    std::shared_ptr<MyTriMesh> ret=AutoRemesher<MyTriMesh>::Remesh(*clean,RemPar);
//    tri_mesh.Clear();
//    vcg::tri::Append<MyTriMesh,MyTriMesh>::Mesh(tri_mesh,(*ret));
//    vcg::tri::Clean<MyTriMesh>::RemoveUnreferencedVertex(tri_mesh);
//    vcg::tri::Allocator<MyTriMesh>::CompactEveryVector(tri_mesh);
//    tri_mesh.UpdateDataStructures();
//    //    tri_mesh.UpdateDataStructures();
//    //    vcg::tri::IsotropicRemeshing<MyTriMesh>::Params par;
//    //    par.minLength=tri_mesh.bbox.Diag()*0.005;
//    //    par.maxLength=tri_mesh.bbox.Diag()*0.01;
//    //    vcg::tri::IsotropicRemeshing<MyTriMesh>::Do(tri_mesh,par);
//    //std::shared_ptr<MyTriMesh> ret=AutoRemesher<MyTriMesh>::Remesh(tri_mesh,RemPar);
//    //tri_mesh.Clear();
//    //vcg::tri::Append<MyTriMesh,MyTriMesh>::Mesh(tri_mesh,(*ret));
//    tri_mesh.UpdateDataStructures();
//}

//void TW_CALL InitSharpFeatures(void *)
//{
//    //tri_mesh.Clear();
//    //vcg::tri::Append<MyTriMesh,MyTriMesh>::Mesh(tri_mesh,remeshed_mesh);
//    tri_mesh.UpdateDataStructures();
//    tri_mesh.InitSharpFeatures(SharpDegree);
//}

//void TW_CALL RefineIfNeeded(void *)
//{
//    tri_mesh.RefineIfNeeded();
//    Gr.Set(tri_mesh.face.begin(),tri_mesh.face.end());
//}


//void TW_CALL SelectCorners(void *)
//{
//    vcg::tri::UpdateSelection<MyTriMesh>::VertexCornerBorder(tri_mesh,fabs(180-CornerDegree) * M_PI/180);
//    vcg::tri::UpdateColor<MyTriMesh>::PerVertexConstant(tri_mesh);
//    for (size_t i=0;i<tri_mesh.vert.size();i++)
//    {
//        if (!tri_mesh.vert[i].IsS())continue;
//        tri_mesh.vert[i].C()=vcg::Color4b::Red;
//    }
//    //Gr.Set(tri_mesh.face.begin(),tri_mesh.face.end());
//}

void TW_CALL SmoothCurvatureField(void *)
{
    FieldParam.align_borders=alignFieldBorder;
    tri_mesh.SmoothField(FieldParam);
    drawfield=true;
    drawsingularities=true;
    tri_mesh.SampleFacesV();
    //Gr.Set(tri_mesh.face.begin(),tri_mesh.face.end());
}

#define MAX_ANI 8

void TW_CALL InitAnisotropy(void *)
{
    tri_mesh.InitAnisotropyOnQ(MinABSFieldVal,MaxABSFieldVal,MinFieldVal,MaxFieldVal,MAX_ANI);
    vcg::tri::UpdateColor<MyTriMesh>::PerFaceQualityGray(tri_mesh);
}

void TW_CALL SmoothFieldByAnisotropy(void *)
{
    //save the norm
    tri_mesh.NormalizeMagnitudo();

    //    //set quality as importance
    //    for (size_t i=0;i<tri_mesh.face.size();i++)
    //        tri_mesh.face[i].Q()=1-tri_mesh.face[i].MaskVal;

    if (alignFieldBorder)
    {
        tri_mesh.InitFieldBoundaryConstraint();
    }
    vcg::tri::FieldSmoother<MyTriMesh>::SmoothDirectionsIGL(tri_mesh,4,vcg::tri::SMMiq,alignFieldBorder,SmoothGamma);
    vcg::tri::CrossField<MyTriMesh>::UpdateSingularByCross(tri_mesh);

    tri_mesh.ScaleMagnitudo();
}

std::string GetProjectName()
{
    std::string projM=pathM;
    size_t indexExt=projM.find_last_of(".");
    projM=projM.substr(0,indexExt);

    std::string FieldDetails=std::string("curv");
    if (pathF!=std::string(""))
    {
        //FieldDetails=pathF;
        QFileInfo fi(pathF.c_str());
        QString name = fi.fileName();                // name = "archive.tar.gz"
        FieldDetails=name.toStdString();
    }
    std::string gradientName=std::to_string((int)MiqP.gradient);
    return(projM+std::string("_")+FieldDetails+
            std::string("_")+gradientName);
}


void DoSaveFieldData()
{
//    std::string projM=pathM;
//    size_t indexExt=projM.find_last_of(".");
//    projM=projM.substr(0,indexExt);

//    std::string FieldDetails=std::string("_curv");
//    if (pathF!=std::string(""))
//    {
//        std::string FieldDetails=pathF;
//        size_t indexExt=FieldDetails.find_last_of(".");
//        FieldDetails=FieldDetails.substr(0,indexExt);
//    }

    //std::string meshName=projM+FieldDetails+std::string("_rem.obj");
    //std::string fieldName=projM+FieldDetails+std::string("_rem.rosy");
    //std::string fieldName=projM+FieldDetails+std::string(".rosy");
    std::string fieldName=GetProjectName()+std::string(".rosy");

    //std::string sharpName=projM+FieldDetails+std::string("_rem.sharp");
    //std::cout<<"Saving Mesh TO:"<<meshName.c_str()<<std::endl;
    std::cout<<"Saving Field TO:"<<fieldName.c_str()<<std::endl;
    //std::cout<<"Saving Sharp TO:"<<sharpName.c_str()<<std::endl;
    //tri_mesh.SaveTriMesh(meshName.c_str());
    tri_mesh.SaveField(fieldName.c_str());
    //tri_mesh.SaveSharpFeatures(sharpName.c_str());
}

void TW_CALL SaveFieldData(void *)
{

    DoSaveFieldData();
}


void DoSaveSetup()
{
//    std::string projM=pathM;
//    size_t indexExt=projM.find_last_of(".");
//    projM=projM.substr(0,indexExt);

//    std::string FieldDetails=std::string("_curv");
//    if (pathF!=std::string(""))
//    {
//        std::string FieldDetails=pathF;
//        size_t indexExt=FieldDetails.find_last_of(".");
//        FieldDetails=FieldDetails.substr(0,indexExt);
//    }
//    std::string setupName=projM+FieldDetails+std::string("_setup.txt");
    std::string setupName=GetProjectName()+std::string("_setup.txt");
    FILE *f=fopen(setupName.c_str(),"wt");
    assert(f!=NULL);
    if (FieldParam.align_borders)
        fprintf(f,"Align Borders:1\n");
    else
        fprintf(f,"Align Borders:0\n");
    fprintf(f,"Alpha Curv:%5.5f\n",FieldParam.alpha_curv);
    fprintf(f,"Hard Curv:%5.5f\n",FieldParam.curv_thr);
    fprintf(f,"CurvRing:%d\n",FieldParam.curvRing);

    if (FieldParam.SmoothM==vcg::tri::SMMiq)
        fprintf(f,"SmoothMethod: MIQ \n");
    else
        fprintf(f,"SmoothMethod: NPoly \n");

    fprintf(f,"AniGlobal:%5.5f\n",SmoothGamma);
    fprintf(f,"Gradient:%5.5f\n",MiqP.gradient);
    fprintf(f,"Ratio Small:%5.5f\n",smallVal);
    fclose(f);
}

void DoSaveQuad()
{
    std::string quadName=GetProjectName()+std::string("_quad.obj");
    std::string ropeName=GetProjectName()+std::string("_rope.txt");
    std::cout<<"Saving Mesh TO:"<<quadName.c_str()<<std::endl;
    std::cout<<"Saving Tope TO:"<<ropeName.c_str()<<std::endl;
    quad_mesh.SaveEdgeSeq(ropeName.c_str());
    vcg::tri::io::ExporterOBJ<PMesh>::Save(quad_mesh,quadName.c_str(),vcg::tri::io::Mask::IOM_BITPOLYGONAL);
}

void TW_CALL SaveAbaqus(void *)
{
    std::string projM=pathM;
    size_t indexExt=projM.find_last_of(".");
    projM=projM.substr(0,indexExt);
    std::string inpName=projM+std::string("_quad.inp");
    quad_mesh.ExportToAbaqus(inpName);
}

void SaveAll()
{
    //save field
    DoSaveFieldData();
    //save quad
    DoSaveQuad();
    //save setup
    DoSaveSetup();
    //close the execution
}

void TW_CALL SaveAllData(void *)
{
    SaveAll();
    //DoSaveQuad();
}

void TW_CALL SaveSingularities(void *)
{
    // query if an attribute is present or not
    bool hasSingular = vcg::tri::HasPerVertexAttribute(tri_mesh,std::string("Singular"));
    bool hasSingularIndex = vcg::tri::HasPerVertexAttribute(tri_mesh,std::string("SingularIndex"));

    if (!hasSingular)return;
    if(!hasSingularIndex)return;

    typename MyTriMesh::template PerVertexAttributeHandle<bool> Handle_Singular;
    Handle_Singular=vcg::tri::Allocator<MyTriMesh>::template GetPerVertexAttribute<bool>(tri_mesh,std::string("Singular"));
    typename MyTriMesh::template PerVertexAttributeHandle<int> Handle_SingularIndex;
    Handle_SingularIndex =vcg::tri::Allocator<MyTriMesh>::template GetPerVertexAttribute<int>(tri_mesh,std::string("SingularIndex"));

    FILE *f=fopen("sing_index.txt","wt");
    for (size_t i=0;i<tri_mesh.vert.size();i++)
    {
        if (tri_mesh.vert[i].IsD())continue;
        if (!Handle_Singular[i])continue;
        fprintf(f,"%d,%f,%f,%f\n",Handle_SingularIndex[i],
                tri_mesh.vert[i].P().X(),
                tri_mesh.vert[i].P().Y(),
                tri_mesh.vert[i].P().Z());
    }
    fclose(f);
}

void SetFieldBarSizePosition(QWidget *w)
{
    int params[2];
    params[0] = QTDeviceWidth(w) / 3;
    params[1] = QTDeviceHeight(w) / 1.5;
    TwSetParam(barQuad, NULL, "size", TW_PARAM_INT32, 2, params);
    params[0] = QTLogicalToDevice(w, 10);
    params[1] = 30;//QTDeviceHeight(w) - params[1] - QTLogicalToDevice(w, 10);
    TwSetParam(barQuad, NULL, "position", TW_PARAM_INT32, 2, params);
}

void DoBatchComputation()
{
    if (drawfield)
    {
        std::cout<<"*** USING FIELD ****"<<std::endl;
        tri_mesh.InitAnisotropyOnQ(MinABSFieldVal,MaxABSFieldVal,MinFieldVal,MaxFieldVal,MAX_ANI);
        vcg::tri::UpdateColor<MyTriMesh>::PerFaceQualityGray(tri_mesh);

        //save the norm
        tri_mesh.NormalizeMagnitudo();
        if (alignFieldBorder)
        {
            FieldParam.align_borders=alignFieldBorder;
            tri_mesh.InitFieldBoundaryConstraint();
        }
        vcg::tri::FieldSmoother<MyTriMesh>::SmoothDirectionsIGL(tri_mesh,4,vcg::tri::SMMiq,alignFieldBorder,SmoothGamma);
        vcg::tri::CrossField<MyTriMesh>::UpdateSingularByCross(tri_mesh);

        tri_mesh.ScaleMagnitudo();

    }
    else
    {
        std::cout<<"*** USING CURVATURE ****"<<std::endl;
        tri_mesh.SmoothField(FieldParam);
    }
    drawfield=true;
    drawsingularities=true;
    //quadrangulate
    DoMiqQuadrangulate();
    if (smallVal>0)
        DoCollapseSmall();
//    //save field
//    DoSaveFieldData();
//    //save quad
//    DoSaveQuad();
//    //save setup
//    DoSaveSetup();
//    //close the execution
//    exit(0);
}

void TW_CALL ColorUniformly(void *)
{
   vcg::tri::UpdateColor<MyTriMesh>::PerFaceConstant(tri_mesh);
}

void TW_CALL BatchComputation(void *)
{
   DoBatchComputation();
}

void InitFieldBar(QWidget *w)
{
    barQuad = TwNewBar("MiqQuadrangulator");

    SetFieldBarSizePosition(w);
    TwEnumVal drawmodes[4] = { {vcg::GLW::DMSmooth, "Smooth"},
                               {vcg::GLW::DMPoints, "Per Points"},
                               {vcg::GLW::DMFlatWire, "FlatWire"},
                               {vcg::GLW::DMFlat, "Flat"}};

    // Create a type for the enum shapeEV
    TwType drawMode = TwDefineEnum("DrawMode", drawmodes, 4);
    TwAddVarRW(barQuad, "Draw Mode", drawMode, &drawmode, " keyIncr='<' keyDecr='>' help='Change draw mode.' ");

    TwAddVarRW(barQuad,"Draw Field",TW_TYPE_BOOLCPP, &drawfield," label='Draw Field'");
    TwAddVarRW(barQuad,"Draw Magnitudo",TW_TYPE_BOOLCPP, &drawmagnitudo," label='Draw Magnitudo'");

    TwAddVarRW(barQuad,"Draw Sing",TW_TYPE_BOOLCPP, &drawsingularities," label='Draw Singularities'");
    TwAddVarRW(barQuad,"Draw Seq",TW_TYPE_BOOLCPP, &draw_seq," label='Draw Sequences'");


    //TwAddVarRW(barQuad,"SharpDegree",TW_TYPE_DOUBLE, &SharpDegree," label='Sharp Degree'");
    //TwAddVarRW(barQuad,"LimitConcave",TW_TYPE_DOUBLE, &tri_mesh.LimitConcave," label='Limit Concave'");

    //TwAddButton(barQuad,"SetSharp",InitSharpFeatures,0,"label='InitSharp'");
    //TwAddSeparator(barQuad,NULL,NULL);

//    TwAddButton(barQuad,"AutoRemesh",AutoRemesh,0,"label='AutoRemesh'");
//    TwAddButton(barQuad,"Refine",RefineIfNeeded,0,"label='Refine if needed'");

    TwAddSeparator(barQuad,NULL,NULL);
    TwAddVarRW(barQuad,"Alpha",TW_TYPE_DOUBLE, &FieldParam.alpha_curv," label='Alpha Curvature'");
    TwAddVarRW(barQuad,"HardCT",TW_TYPE_DOUBLE, &FieldParam.curv_thr," label='Hard Curv Thr'");
    TwAddVarRW(barQuad,"CurvRing",TW_TYPE_INT32,&FieldParam.curvRing,"label='Curvature Ring'");

    TwEnumVal smoothmodes[2] = {
        {vcg::tri::SMMiq,"MIQ"},
        {vcg::tri::SMNPoly,"NPoly"}
    };
    TwType smoothMode = TwDefineEnum("SmoothMode", smoothmodes, 2);
    TwAddVarRW(barQuad, "Smooth Mode", smoothMode, &FieldParam.SmoothM," label='Smooth Mode' ");

    TwAddButton(barQuad,"ComputeField",SmoothCurvatureField,0,"label='Compute Curvature Field'");

    TwAddSeparator(barQuad,NULL,NULL);
    TwAddButton(barQuad,"Init Anisotropy",InitAnisotropy,0,"label='Init Anisotropy'");
    TwAddButton(barQuad,"ColorUniformly",ColorUniformly,0,"label='Color Uniformly'");

    TwAddVarRW(barQuad,"AlignB",TW_TYPE_BOOLCPP, &alignFieldBorder," label='Align Borders'");
    TwAddVarRW(barQuad,"GlobSmooth",TW_TYPE_DOUBLE, &SmoothGamma," label='Global Smooth Factor'");
    TwAddButton(barQuad,"Smooth By Anisotropy",SmoothFieldByAnisotropy,0,"label='Smooth By Anisotropy'");

    TwAddSeparator(barQuad,NULL,NULL);
    TwAddButton(barQuad,"SaveFieldData",SaveFieldData,0,"label='Save Field Sharp Remesh Data'");

    TwAddSeparator(barQuad,NULL,NULL);

//    TwAddVarRW(barQuad,"Corner Degrees",TW_TYPE_DOUBLE, &CornerDegree," label='Corner Degree'");
//    TwAddButton(barQuad,"Select Corner",SelectCorners,0,"label='Select Corners'");

    TwAddVarRW(barQuad,"Gradient",TW_TYPE_DOUBLE, &MiqP.gradient," label='Gradient'");
    TwAddVarRW(barQuad,"Direct Round",TW_TYPE_BOOLCPP, &MiqP.directRound," label='Direct Round'");
    TwAddVarRW(barQuad,"Round Singularities",TW_TYPE_BOOLCPP, &MiqP.round_singularities," label='Round Singularities'");
    TwAddVarRW(barQuad,"IsotropyVsAlign",TW_TYPE_DOUBLE, &miqAnisotropy," label='Isotropy Vs Align'");
//    TwAddVarRW(barQuad,"Align Sharp",TW_TYPE_BOOLCPP, & MiqP.crease_as_feature," label='Align Sharp'");

    TwAddVarRW(barQuad,"Snap Border",TW_TYPE_BOOLCPP, &snapBorder," label='Snap Border'");
    TwAddButton(barQuad,"Quadrangulate",MiqQuadrangulate,0,"label='Miq Quadrangulate'");
//    TwAddButton(barQuad,"SplitNonQuad",SplitQuad,0,"label='Split non Quad'");


    TwAddVarRW(barQuad,"Ratio",TW_TYPE_DOUBLE, &smallVal," label='Ratio'");
    TwAddButton(barQuad,"CollapseSmall",CollapseSmallEdges,0,"label='Collapse Small Edges'");

    //    TwAddVarRW(barRem,"DrawTris",TW_TYPE_BOOL8, &drawTris," label='Draw Tris'");
    //    TwAddVarRW(barRem,"DrawHexa",TW_TYPE_BOOL8, &drawHex," label='Draw Hexa'");

//    TwAddVarRW(barQuad,"Optimize Dual",TW_TYPE_BOOL8, &QRef.optimize_for_dual," label='dual Optimize'");
//    TwAddVarRW(barQuad,"Erode Dilate",TW_TYPE_BOOL8, &QRef.erode_dilate," label='erode Dilate'");
//    TwAddVarRW(barQuad,"Dualize",TW_TYPE_BOOL8, &QRef.dualize_final," label='dualize'");
//    TwAddVarRW(barQuad,"Optimize",TW_TYPE_BOOL8, &QRef.final_optimize," label='optimize'");

//    TwAddButton(barQuad,"RefineBig",RefineBigFaces,0,"label='Refine Big Faces'");

    TwAddSeparator(barQuad,NULL,NULL);
    TwAddButton(barQuad,"BatchProcess",BatchComputation,0,"label='BatchProcess'");
    TwAddButton(barQuad,"SaveAll",SaveAllData,0,"label='Save All Data'");

    //TwAddButton(barQuad,"SaveSing",SaveSingularities,0,"label='Save Singolarities'");
 //   TwAddButton(barQuad,"SaveAbaqus",SaveAbaqus,0,"label='Save Abaqus Data'");
    //TwAddButton(barQuad,"SaveSing",SaveSingularities,0,"label='Save Singolarities'");

}


GLWidget::GLWidget(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{
    FieldParam.alpha_curv=0.3;
    FieldParam.align_borders=true;
    drawmode=vcg::GLW::DMFlat;
    hasToPick=false;
    bool AllQuad=false;
    bool Loaded=tri_mesh.LoadTriMesh(pathM,AllQuad);
    if (!Loaded)
    {
        std::cout<<"Error Loading Mesh"<<std::endl;
        exit(0);
    }
    if (AllQuad)
    {
        drawfield=true;
        drawsingularities=true;
    }
    std::cout<<"Loaded "<<tri_mesh.face.size()<<" faces "<<std::endl;
    std::cout<<"Loaded "<<tri_mesh.vert.size()<<" vertices "<<std::endl;

    glWrap.m=&tri_mesh;
    //    original_mesh.Clear();
    //    vcg::tri::Append<MyTriMesh,MyTriMesh>::Mesh(original_mesh,tri_mesh);
    //    remeshed_mesh.Clear();
    //    vcg::tri::Append<MyTriMesh,MyTriMesh>::Mesh(remeshed_mesh,tri_mesh);

    tri_mesh.UpdateDataStructures();
    tri_mesh.SampleFacesV();
    //tri_mesh.LimitConcave=0;
    //remeshed_mesh.UpdateDataStructures();
    Gr.Set(tri_mesh.face.begin(),tri_mesh.face.end());

    if (pathF!=std::string(""))
    {
        bool IsDimensioned=false;
        bool loaded=tri_mesh.LoadField(pathF,IsDimensioned);
        assert(loaded);
        drawfield=true;
        drawsingularities=true;
        if (IsDimensioned)
        {
            MinFieldVal=tri_mesh.GetFieldNormPercentile(0.1);
            MaxFieldVal=tri_mesh.GetFieldNormPercentile(0.9);
            std::cout<<"Min Field Magnitudo "<<MinFieldVal<<std::endl;
            std::cout<<"Max Field Magnitudo "<<MaxFieldVal<<std::endl;
            MinABSFieldVal=tri_mesh.GetFieldNormPercentile(0.1,true);
            MaxABSFieldVal=tri_mesh.GetFieldNormPercentile(0.9,true);
            std::cout<<"Min ABS Field Magnitudo "<<MinFieldVal<<std::endl;
            std::cout<<"Max ABS Field Magnitudo "<<MaxFieldVal<<std::endl;
        }
    }
//    if (pathL!=std::string(""))
//    {
//       std::cout<<"Loading Boundary Conditions"<<std::endl;
//       tri_mesh.LoadBoxBoundaryConditions(pathL);//LoadBoundaryConditions(pathL);
//       std::cout<<"Done Loading Boundary Conditions"<<std::endl;
//    }

    MiqP.gradient=DefaultGradient;
    MiqP.directRound=false;
    MiqP.round_singularities=false;
    MiqP.local_iter=10;

    if (do_batch)
    {
        DoBatchComputation();
        SaveAll();
        exit(0);
    }
}



void GLWidget::initializeGL ()
{
    //initialize Glew
    glewInit();
    //CaptInt.GLInit( GLWidget::width(),GLWidget::height());
    glClearColor(0, 0, 0, 0);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
}


void GLWidget::resizeGL (int w, int h)
{
    glViewport (0, 0, (GLsizei) w, (GLsizei) h);
    TwWindowSize(w, h);
    InitFieldBar(this);
    initializeGL();
}


void GLWidget::paintGL ()
{

    //    if (RType!=OldRType)
    //    {
    //        PatchDeco.ColorPatches(RType);
    //        OldRType=RType;
    //    }

    glClearColor(255,255,255,255);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(40, GLWidget::width()/(float)GLWidget::height(), 0.1, 100);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0,0,3.5f,   0,0,0,   0,1,0);
    track.center=vcg::Point3f(0, 0, 0);
    track.radius= 1;
    track.GetView();

    glPushMatrix();
    track.Apply();
    glPushMatrix();

    glDisable(GL_CULL_FACE);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);


    if(tri_mesh.vert.size()>0)
    {
        vcg::glScale(2.0f/tri_mesh.bbox.Diag());
        glTranslate(-tri_mesh.bbox.Center());
        tri_mesh.GLDrawSharpEdges();
        if (!quadrangulated)
        {
            glWrap.Draw(drawmode,vcg::GLW::CMPerFace,vcg::GLW::TMNone);
            tri_mesh.GLDrawBoundaryConditions();
        }
    }
    if (quadrangulated)
    {
        quad_mesh.GLDraw(true);

        if (draw_seq)
            quad_mesh.GLDrawEdgeSeq();
    }

    if (drawfield)
    {
        if (MinFieldVal==MaxFieldVal)
            vcg::GLField<MyTriMesh>::GLDrawFaceField(tri_mesh,false,false,0.007,0,0,false,true);
        else
        {
            //            std::cout<<"Min "<<MinFieldVal<<std::endl;
            //            std::cout<<"Max "<<MaxFieldVal<<std::endl;
            if (drawmagnitudo)
                vcg::GLField<MyTriMesh>::GLDrawFaceField(tri_mesh,false,false,0.007,MaxFieldVal,MinFieldVal,true,true);
            else
                vcg::GLField<MyTriMesh>::GLDrawFaceField(tri_mesh,false,false,0.007,0,0,false,true);
        }
    }
    if (drawsingularities)
        vcg::GLField<MyTriMesh>::GLDrawSingularity(tri_mesh);
    ////        if (!quadrangulated)
    ////        {
    //            vcg::GLField<MyTriMesh>::GLDrawFaceField(tri_mesh,false,false,0.007);
    //            //vcg::GLField<MyTriMesh>::GLDrawVertField(tri_mesh,0.007);
    //            vcg::GLField<MyTriMesh>::GLDrawSingularity(tri_mesh);
    //        }
    //    }

    if(hasToPick)
    {
        hasToPick=false;
        typename MyTriMesh::CoordType pp;
        if(vcg::Pick<typename MyTriMesh::CoordType>(xMouse,yMouse,pp))
        {
            typename MyTriMesh::CoordType closPt,bary;
            typename MyTriMesh::ScalarType minD;
            typename MyTriMesh::FaceType *f=vcg::tri::GetClosestFaceBase(tri_mesh,Gr,pp,tri_mesh.bbox.Diag(),minD,closPt);
            vcg::InterpolationParameters(*f,closPt,bary);
            size_t EdgeI=1;
            if ((bary.Y()<bary.X())&&(bary.Y()<bary.Z()))EdgeI=2;
            if ((bary.Z()<bary.X())&&(bary.Z()<bary.Y()))EdgeI=0;

            MyTriMesh::FaceType *fOpp=f->FFp(EdgeI);
            int eOpp=f->FFi(EdgeI);

            if (f->IsFaceEdgeS(EdgeI))
            {
                f->ClearFaceEdgeS(EdgeI);
                if (fOpp!=f)
                    fOpp->ClearFaceEdgeS(eOpp);
            }else
            {
                f->SetFaceEdgeS(EdgeI);
                if (fOpp!=f)
                    fOpp->SetFaceEdgeS(eOpp);
            }
        }
    }



    glPopMatrix();
    glPopMatrix();


    TwDraw();

}

void GLWidget::keyReleaseEvent (QKeyEvent * e)
{
    e->ignore ();
    if (e->key () == Qt::Key_Control)  track.ButtonUp (QT2VCG (Qt::NoButton, Qt::ControlModifier));
    if (e->key () == Qt::Key_Shift)  track.ButtonUp (QT2VCG (Qt::NoButton, Qt::ShiftModifier));
    if (e->key () == Qt::Key_Alt) track.ButtonUp (QT2VCG (Qt::NoButton, Qt::AltModifier));
    updateGL ();
}


void GLWidget::keyPressEvent (QKeyEvent * e)
{
    e->ignore ();
    if (e->key () == Qt::Key_Control) track.ButtonDown (QT2VCG (Qt::NoButton, Qt::ControlModifier));
    if (e->key () == Qt::Key_Shift)  track.ButtonDown (QT2VCG (Qt::NoButton, Qt::ShiftModifier));
    if (e->key () == Qt::Key_Alt)  track.ButtonDown (QT2VCG (Qt::NoButton, Qt::AltModifier));

    TwKeyPressQt(e);
    updateGL ();
}

void GLWidget::mousePressEvent (QMouseEvent * e)
{
    if(!TwMousePressQt(this,e))
    {
        e->accept ();
        setFocus ();
        track.MouseDown(QT2VCG_X(this, e), QT2VCG_Y(this, e), QT2VCG (e->button (), e->modifiers ()));
    }
    updateGL ();
}

void GLWidget::mouseMoveEvent (QMouseEvent * e)
{
    if (e->buttons ()) {
        track.MouseMove(QT2VCG_X(this, e), QT2VCG_Y(this, e));
        updateGL ();
    }
    TwMouseMotion(QTLogicalToDevice(this, e->x()), QTLogicalToDevice(this, e->y()));
}

void GLWidget::mouseDoubleClickEvent (QMouseEvent * e)
{
    if (e->buttons ())
    {
        xMouse=QT2VCG_X(this, e);
        yMouse=QT2VCG_Y(this, e);
        //pointToPick=Point2i(e->x(),height()-e->y());
        //pointToPick=Point2i(xMouse,yMouse);
        hasToPick=true;
        updateGL ();
    }
    updateGL();
}

void GLWidget::mouseReleaseEvent (QMouseEvent * e)
{
    track.MouseUp(QT2VCG_X(this, e), QT2VCG_Y(this, e), QT2VCG(e->button (), e->modifiers ()));
    TwMouseReleaseQt(this,e);
    updateGL ();
}

void GLWidget::wheelEvent (QWheelEvent * e)
{
    const int WHEEL_STEP = 120;
    track.MouseWheel (e->delta () / float (WHEEL_STEP), QTWheel2VCG (e->modifiers ()));
    updateGL ();
}

#ifndef VORONOI_MESHER
#define VORONOI_MESHER

#include <vcg/complex/algorithms/voronoi_remesher.h>
#include <vcg/complex/algorithms/dual_meshing.h>
#include <vcg/complex/algorithms/polygon_support.h>
#include <vcg/complex/algorithms/polygonal_algorithms.h>


template <class TriMesh,class PolyMesh>
void VoroRemesh(TriMesh &tri_mesh,
                const typename TriMesh::ScalarType &SampleSize,
                const bool UseDeformation,
                PolyMesh &poly_mesh,
                int MaxRelaxIte=50,
                int refinementRatio=6)
{
    typedef typename TriMesh::FaceType TriFaceType;
    typedef typename TriMesh::VertexType TriVertexType;
    typedef typename TriMesh::CoordType CoordType;
    typedef typename TriMesh::ScalarType ScalarType;

    TriMesh baseMesh;
    vcg::tri::Append<TriMesh,TriMesh>::Mesh(baseMesh,tri_mesh);
    baseMesh.UpdateDataStructures();

    vcg::tri::VoronoiProcessingParameter vpp;
    vpp.refinementRatio=refinementRatio;

    typedef typename vcg::tri::SurfaceSampling<TriMesh,vcg::tri::TrivialSampler<TriMesh> > SurfaceSampler;
    typename SurfaceSampler::PoissonDiskParam pp;

    float radius = 2*baseMesh.bbox.Diag()/SampleSize;
    vcg::tri::VoronoiProcessing<TriMesh>::PreprocessForVoronoi(baseMesh,radius,vpp);
    baseMesh.UpdateDataStructures();

    if (UseDeformation)
    {
        vcg::GridStaticPtr<TriFaceType,ScalarType> Gr;
        Gr.Set(tri_mesh.face.begin(),tri_mesh.face.end());
        for (size_t i=0;i<baseMesh.vert.size();i++)
        {
            ScalarType minD;
            CoordType closP,normF,IP;
            TriFaceType *f=vcg::tri::GetClosestFaceBase(tri_mesh,Gr,baseMesh.vert[i].P(),
                                                        tri_mesh.bbox.Diag(),minD,closP,
                                                        normF,IP);
            assert(f!=NULL);
            CoordType RPos0=f->V(0)->RPos;
            CoordType RPos1=f->V(1)->RPos;
            CoordType RPos2=f->V(2)->RPos;
            CoordType InterpRPos=RPos0*IP.X()+RPos1*IP.Y()+RPos2*IP.Z();
            baseMesh.vert[i].RPos=InterpRPos;
        }
    }

    //vcg::tri::io::ExporterPLY<MyTriMesh>::Save(baseMesh,"remeshed_voro.ply");
    // -- Build a sampling with just corners (Poisson filtered)
    TriMesh poissonCornerMesh;
    std::vector<CoordType> sampleVec;
    vcg::tri::TrivialSampler<TriMesh> mps(sampleVec);
    vcg::tri::SurfaceSampling<TriMesh,vcg::tri::TrivialSampler<TriMesh> >::VertexBorderCorner(baseMesh,mps,vcg::math::ToRad(150.f));
    vcg::tri::BuildMeshFromCoordVector(poissonCornerMesh,sampleVec);


    // sampleVec.clear();
    TriMesh borderMesh,poissonBorderMesh;

    //    float radius = tri_mesh.bbox.Diag()/MiqP.gradient;
    //    vcg::tri::SurfaceSampling<MyTriMesh,vcg::tri::TrivialSampler<MyTriMesh> >::PoissonDiskPruning(mps, poissonCornerMesh, radius, pp);
    //    vcg::tri::BuildMeshFromCoordVector(poissonCornerMesh,sampleVec);
    //    vcg::tri::io::ExporterPLY<MyTriMesh>::Save(poissonCornerMesh,"poissonCornerMesh.ply");

    // Now save the corner as Fixed Seeds for later...
    std::vector<TriVertexType*> fixedSeedVec;
    vcg::tri::VoronoiProcessing<TriMesh>::SeedToVertexConversion(baseMesh,sampleVec,fixedSeedVec);
    vcg::tri::VoronoiProcessing<TriMesh, vcg::tri::EuclideanDistance<TriMesh> >::MarkVertexVectorAsFixed(baseMesh,fixedSeedVec);
    vpp.preserveFixedSeed=true;
    //}

    // -- Build a sampling with points on the border
    sampleVec.clear();
    vcg::tri::SurfaceSampling<TriMesh,vcg::tri::TrivialSampler<TriMesh> >::VertexBorder(baseMesh,mps);
    vcg::tri::BuildMeshFromCoordVector(borderMesh,sampleVec);
    //vcg::tri::io::ExporterPLY<MyTriMesh>::Save(borderMesh,"borderMesh.ply");

    // -- and then prune the border sampling with poisson strategy using the precomputed corner vertexes.
    pp.preGenMesh = &poissonCornerMesh;
    pp.preGenFlag=true;
    sampleVec.clear();
    vcg::tri::SurfaceSampling<TriMesh,vcg::tri::TrivialSampler<TriMesh> >::PoissonDiskPruning(mps, borderMesh, radius*0.8f, pp);
    vcg::tri::BuildMeshFromCoordVector(poissonBorderMesh,sampleVec);
    //vcg::tri::io::ExporterPLY<MyTriMesh>::Save(poissonBorderMesh,"PoissonEdgeMesh.ply");

    // -- Build the montercarlo sampling of the surface
    TriMesh MontecarloSurfaceMesh;
    sampleVec.clear();
    vcg::tri::SurfaceSampling<TriMesh,vcg::tri::TrivialSampler<TriMesh> >::Montecarlo(baseMesh,mps,50000);
    vcg::tri::BuildMeshFromCoordVector(MontecarloSurfaceMesh,sampleVec);
    //vcg::tri::io::ExporterPLY<MyTriMesh>::Save(MontecarloSurfaceMesh,"MontecarloSurfaceMesh.ply");

    // -- Prune the montecarlo sampling with poisson strategy using the precomputed vertexes on the border.
    pp.preGenMesh = &poissonBorderMesh;
    pp.preGenFlag=true;
    sampleVec.clear();
    vcg::tri::SurfaceSampling<TriMesh,vcg::tri::TrivialSampler<TriMesh> >::PoissonDiskPruning(mps, MontecarloSurfaceMesh, radius, pp);
    TriMesh PoissonMesh;
    //    vcg::tri::BuildMeshFromCoordVector(PoissonMesh,sampleVec);
    //    vcg::tri::io::ExporterPLY<MyTriMesh>::Save(PoissonMesh,"PoissonMesh.ply");

    std::vector<TriVertexType *> seedVec;
    vcg::tri::VoronoiProcessing<TriMesh>::SeedToVertexConversion(baseMesh,sampleVec,seedVec);

    // Select all the vertexes on the border to define a constrained domain.
    // In our case we select the border vertexes to make sure that the seeds on the border
    // relax themselves remaining on the border
    for(size_t i=0;i<baseMesh.vert.size();++i){
        if(baseMesh.vert[i].IsB())
            baseMesh.vert[i].SetS();
    }

    //  vpp.deleteUnreachedRegionFlag=true;
    vpp.deleteUnreachedRegionFlag=false;
    vpp.triangulateRegion = false;
    vpp.geodesicRelaxFlag=false;
    vpp.constrainSelectedSeed=true;

    vcg::tri::EuclideanDistance<TriMesh> dd;
    int iterNum=MaxRelaxIte;
    vpp.collapseShortEdge=true;
    vpp.collapseShortEdgePerc=0.05;

    // And now, at last, the relaxing procedure!
    int actualIter = vcg::tri::VoronoiProcessing<TriMesh, vcg::tri::EuclideanDistance<TriMesh> >::VoronoiRelaxing(baseMesh, seedVec, iterNum, dd, vpp);
    std::cout<<"Performed "<<actualIter<<"iterations"<<std::endl;

    //        MyTriMesh PoissonRelaxed;
    //        std::vector<MyTriMesh::CoordType> relaxedSample;
    //        for (size_t i=0;i<seedVec.size();i++)
    //            relaxedSample.push_back(seedVec[i]->P());

    //        vcg::tri::BuildMeshFromCoordVector(PoissonRelaxed,relaxedSample);
    //        vcg::tri::io::ExporterPLY<MyTriMesh>::Save(PoissonRelaxed,"RelaxedSamples.ply");
    //        //MyTriMesh PoissonMesh;

    //        //    int t1=clock();

    TriMesh voroMesh, voroPoly, delaMesh;
    // Get the result in some pleasant form converting it to a real voronoi diagram.
    if(vcg::tri::VoronoiProcessing<TriMesh>::CheckVoronoiTopology(baseMesh,seedVec))
    {
        baseMesh.MoveToRestPos();
        vcg::tri::VoronoiProcessing<TriMesh>::ConvertVoronoiDiagramToMesh(baseMesh,voroMesh,voroPoly,seedVec, vpp);
        //            vcg::tri::io::ExporterPLY<MyTriMesh>::Save(voroMesh,"voroPoly.ply",vcg::tri::io::Mask::IOM_ALL);
        //            vcg::tri::io::ExporterPLY<MyTriMesh>::Save(voroPoly,"voroEdge.ply",vcg::tri::io::Mask::IOM_ALL);
        vcg::tri::Clean<TriMesh>::RemoveUnreferencedVertex(voroMesh);
        vcg::tri::Allocator<TriMesh>::CompactEveryVector(voroMesh);
        voroMesh.UpdateDataStructures();
        vcg::tri::PolygonSupport<TriMesh,PolyMesh>::ImportFromTriMesh( poly_mesh,voroMesh);
        poly_mesh.UpdateAttributes();

    }

    vcg::PolygonalAlgorithm<PolyMesh>::SmoothReprojectPCA(poly_mesh);
}

#endif

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


#include <QApplication>
#include <QDesktopWidget>
#include <GL/glew.h>
#include <triangle_mesh_type.h>
#include "glwidget.h"
#include <wrap/qt/anttweakbarMapper.h>
#include <QWindow>
#include <QFileInfo>
//#include <triangle_mesh_type.h>
//#include <wrap/io_trimesh/import_field.h>
//#include <wrap/io_trimesh/import.h>
#include <wrap/igl/miq_parametrization.h>
#include <vcg/complex/algorithms/hole.h>
#include <stdio.h>      /* printf */
#include <stdlib.h>     /* strtol */

extern std::string pathM;
extern std::string pathF;
extern std::string pathL;
extern typename MyTriMesh::ScalarType DefaultGradient;
extern bool do_batch;
extern bool snapBorder;
extern bool alignFieldBorder;
extern typename MyTriMesh::ScalarType miqAnisotropy;
extern typename MyTriMesh::ScalarType SmoothGamma;
extern MyTriMesh::ScalarType smallVal;

bool CheckExist(const std::string &path)
{
    QString pathQ=QString(path.c_str());
    QFileInfo f_infoF(pathQ);
    if (!f_infoF.exists())
    {
        printf("error: fileneme wrong\n");
        fflush(stdout);
        return false;
    }
    return true;
}

std::string GetExtension(const std::string &path)
{
    QFileInfo fi(path.c_str());
    QString ext = fi.suffix();
    return (ext.toStdString());
}

bool isNumber(const std::string &str)
{
    //char* p;
    //std::strtol(str.c_str(), &p, 10);
    //return *p == 0;
    char* p=NULL;
    float ret=std::strtof(str.c_str(), &p);
    return (ret != 0);
}

void ParseArgument(const std::string &str)
{
    if (str.compare(std::string("alignb"))==0)
    {
        alignFieldBorder=true;
        std::cout<<"**** ALIGN BORDERS **** "<<std::endl;
        return;
    }
    if (str.compare(std::string("snap"))==0)
    {
        snapBorder=true;
        std::cout<<"**** SNAP BORDERS **** "<<std::endl;
        return;
    }
    if (str.compare(std::string("batch"))==0)
    {
        do_batch=true;
        std::cout<<"**** BATCH COMPUTATION **** "<<std::endl;
        return;
    }

    size_t indexExt=str.find("-G");
    if (indexExt!=string::npos)
    {
        std::string valGrad=str.substr(2,str.size());
        DefaultGradient=atoi(valGrad.c_str());
        std::cout<<"Gradient Value: "<<DefaultGradient<<std::endl;
        return;
    }

    indexExt=str.find("-A");
    if (indexExt!=string::npos)
    {
        std::string valAni=str.substr(2,str.size());
        miqAnisotropy=atof(valAni.c_str());
        std::cout<<"Anisotropy Value: "<<miqAnisotropy<<std::endl;
        return;
    }

    indexExt=str.find("-S");
    if (indexExt!=string::npos)
    {
        std::string valSm=str.substr(2,str.size());
        SmoothGamma=atof(valSm.c_str());
        std::cout<<"Smooth Gamma Value: "<<SmoothGamma<<std::endl;
        return;
    }

    indexExt=str.find("-C");
    if (indexExt!=string::npos)
    {
        std::string valSm=str.substr(2,str.size());
        smallVal=atof(valSm.c_str());
        std::cout<<"Small collapse: "<<smallVal<<std::endl;
        return;
    }

    std::string ext=GetExtension(str);
    if (((ext.compare(std::string("ply"))==0)||
         (ext.compare(std::string("obj"))==0)||
         (ext.compare(std::string("off"))==0)))
    {
        pathM=str;
        std::cout<<"Mesh Filename:"<<pathM.c_str()<<std::endl;
        bool existM=CheckExist(pathM);
        if (!existM)
        {
            std::cout<<"Wrong Mesh Filename"<<std::endl;
            exit(0);
        }
        return;
    }

    if ((ext.compare(std::string("ffield"))==0)||
        (ext.compare(std::string("rosy"))==0)||
        (ext.compare(std::string("blk"))==0))
    {
        pathF=std::string(str);
        std::cout<<"Field Filename:"<<pathF.c_str()<<std::endl;
        bool existF=CheckExist(pathF);
        if (!existF)
        {
            std::cout<<"Wrong Field Filename"<<std::endl;
            exit(0);
        }
        return;
    }

}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QWindow dummy;
    QString def_string = QString("GLOBAL fontscaling=%1").arg((int)dummy.devicePixelRatio());
    TwDefine(def_string.toStdString().c_str());
    printf("%s\n",qPrintable(def_string));
    fflush(stdout);

    // Set functions to handle string copy
    TwCopyCDStringToClientFunc(CopyCDStringToClient);
    TwCopyStdStringToClientFunc(CopyStdStringToClient);

    if( !TwInit(TW_OPENGL, NULL) )
    {
        fprintf(stderr, "AntTweakBar initialization failed: %s\n", TwGetLastError());
        return 1;
    }

    assert(argc>1);
//    pathM=std::string(argv[1]);
//    bool existM=CheckExist(pathM);
//    if (!existM)
//    {
//        std::cout<<"Wrong Mesh Filename"<<std::endl;
//        exit(0);
//    }

//    std::string ext=GetExtension(pathM);
//    std::cout<<"Lading Mesh Extension:"<<ext.c_str()<<std::endl;
//    if (!((ext.compare(std::string("ply"))==0)||
//          (ext.compare(std::string("obj"))==0)||
//          (ext.compare(std::string("off"))==0)))
//    {
//        std::cout<<"Wrong Mesh Extension"<<std::endl;
//        exit(0);
//    }

//    if (argc>2)
//    {
//        std::cout<<"**Checking Parameters**"<<std::endl;
//        for (size_t i=2;i<argc;i++)
//        {
//            std::cout<<"Checking Parameter "<<argv[i]<<std::endl;
//            //in this case is a gradient of the tessellation
//            if (isNumber(argv[i]))
//            {
//                float testN=atof(argv[i]);
//                assert(testN>0);
//                if (testN>1)
//                {
//                    DefaultGradient=atoi( argv[i] );
//                    std::cout<<"Gradient Value: "<<DefaultGradient<<std::endl;
//                }
//                else
//                {
//                    miqAnisotropy=(MyTriMesh::ScalarType)testN;
//                    std::cout<<"Anisotropy Value: "<<miqAnisotropy<<std::endl;
//                }
//            }
//            else{
//                if (std::string(argv[i]).compare(std::string("alignb"))==0)
//                {
//                    alignFieldBorder=true;
//                    std::cout<<"**** ALIGN BORDERS **** "<<std::endl;
//                }
//                else
//                {
//                    if (std::string(argv[i]).compare(std::string("snap"))==0)
//                    {
//                        snapBorder=true;
//                        std::cout<<"**** SNAP BORDERS **** "<<std::endl;
//                    }
//                    else
//                    {
//                        if (std::string(argv[i]).compare(std::string("batch"))==0)
//                        {
//                            do_batch=true;
//                            std::cout<<"**** BATCH COMPUTATION **** "<<std::endl;
//                        }
//                        else
//                        {
//                            std::string ext=GetExtension(argv[i]);
//                            if ((ext.compare(std::string("ffield"))==0)||
//                                    (ext.compare(std::string("rosy"))==0)||
//                                    (ext.compare(std::string("blk"))==0))
//                            {
//                                pathF=std::string(argv[i]);
//                                std::cout<<"Field Filename:"<<pathF.c_str()<<std::endl;
//                                bool existF=CheckExist(pathF);
//                                if (!existF)
//                                {
//                                    std::cout<<"Wrong Field Filename"<<std::endl;
//                                    exit(0);
//                                }
//                            }
//                            else
////                                if (ext.compare(std::string("box"))==0)
////                                {
////                                    pathL=std::string(argv[i]);
////                                    std::cout<<"Load Filename:"<<pathF.c_str()<<std::endl;
////                                    bool existL=CheckExist(pathL);
////                                    if (!existL)
////                                    {
////                                        std::cout<<"Wrong Load Filename"<<std::endl;
////                                        exit(0);
////                                    }
////                                }
////                                else
//                                {
//                                    std::cout<<"Wrong Parameter"<<std::endl;
//                                    exit(0);
//                                }
//                        }
//                    }
//                }
//            }
//        }
//    }

    for (size_t i=0;i<argc;i++)
        ParseArgument(argv[i]);

    if (pathM == std::string(""))
    {
       std::cout<<"You should pass a Mesh"<<std::endl;
       exit(0);
    }
    GLWidget window;

    window.show();
    return app.exec();
}

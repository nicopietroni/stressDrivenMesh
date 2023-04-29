CONFIG += c++11

VCGLIBDIR = $$PWD/../vcglib
GLEWDIR   = $$PWD/../code/lib/glew
ANTDIR    = $$PWD/../code/lib/AntTweakBar1.16
EIGENLIB  = $$PWD/lib/libigl/external/nanogui/ext/eigen
IGLLIB    = $$PWD/lib/libigl/include/
COMISODIR = $$PWD/lib/CoMISo

INCLUDEPATH += $$VCGLIBDIR
INCLUDEPATH += $$GLEWDIR/include
INCLUDEPATH += $$ANTDIR/include
INCLUDEPATH += $$EIGENLIB
INCLUDEPATH += $$IGLLIB

HEADERS       = ./app/glwidget.h \
                ./app/triangle_mesh_type.h \
                ./app/quad_refiner.h \
                $$IGLLIB/igl/principal_curvature.h

SOURCES       = ./app/glwidget.cpp \
                ./app/main.cpp \
    		
QT += opengl
QT += widgets

DEFINES += GLEW_STATIC
DEFINES += INCLUDE_TEMPLATES
DEFINES += COMISO_FIELD

SOURCES += $$GLEWDIR/src/glew.c


INCLUDEPATH += $$COMISODIR/gmm/include
INCLUDEPATH += $$COMISODIR/Solver
INCLUDEPATH += $$COMISODIR/..
INCLUDEPATH += ./app/

SOURCES += $$VCGLIBDIR/wrap/ply/plylib.cpp
SOURCES += $$VCGLIBDIR/wrap/gui/trackball.cpp
SOURCES += $$VCGLIBDIR/wrap/gui/trackmode.cpp
SOURCES += $$VCGLIBDIR/wrap/qt/anttweakbarMapperNew.cpp
SOURCES += $$IGLLIB/igl/principal_curvature.cpp
SOURCES += $$IGLLIB/igl/copyleft/comiso/nrosy.cpp

# Awful problem with windows..
win32{
  DEFINES += NOMINMAX
  LIBS +=$$ANTDIR/lib/AntTweakBar.lib
}

macx{
#CONFIG+=sdk_no_version_check
QMAKE_CXXFLAGS_WARN_ON += -Wno-extra
QMAKE_CXXFLAGS_WARN_ON += -Wno-int-in-bool-context
QMAKE_CXXFLAGS_WARN_ON += -Wgnu-inline-cpp-without-extern

}

mac{
# Mac specific Config required to avoid to make application bundles
  CONFIG -= app_bundle
  LIBS +=$$ANTDIR/lib/libAntTweakBar.dylib
  QMAKE_POST_LINK +="cp -P $$ANTDIR/lib/libAntTweakBar.dylib . ; "
  QMAKE_POST_LINK +="install_name_tool -change ../lib/libAntTweakBar.dylib ./libAntTweakBar.dylib $$TARGET ; "
  #QMAKE_POST_LINK +="install_name_tool -change libCoMISo.dylib $$COMISODIR/build/Build/lib/CoMISo/libCoMISo.dylib $$TARGET ;"
  #LIBS += -L $$COMISODIR/build/Build/lib/CoMISo/ -lCoMISo
  #INCLUDEPATH += $$COMISODIR/build/Build/lib/CoMISo
  #DEPENDPATH += $$COMISODIR/build/Build/lib/CoMISo

  #comiso
  LIBS += -L$$COMISODIR/build/Build/lib/CoMISo/ -lCoMISo
  INCLUDEPATH += $$COMISODIR/..

  DEPENDPATH += .
}


#-------------------------------------------------
#
# Project created by QtCreator 2016-07-19T14:47:30
#
#-------------------------------------------------

QT       += opengl

TARGET = QMeshLib
TEMPLATE = lib
CONFIG += staticlib

SOURCES += \
    QMeshEdge.cpp \
    QMeshFace.cpp \
    QMeshNode.cpp \
    QMeshPatch.cpp \
    QMeshTetra.cpp \
    PolygenMesh.cpp

SOURCES += .\PQPLib\*.cpp
SOURCES += .\PQPLib\*.c

HEADERS += \
    QMeshEdge.h \
    QMeshFace.h \
    QMeshNode.h \
    QMeshPatch.h \
    PolygenMesh.h \
    QMeshTetra.h


HEADERS += .\PQPLib\*.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}


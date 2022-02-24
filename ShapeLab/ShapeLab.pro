#-------------------------------------------------
#
# Project created by QtCreator 2016-07-19T14:43:13
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = ShapeLab
TEMPLATE = app


#SOURCES += main.cpp\
#        MainWindow.cpp\
#		MainWindow_SoftRobotKinematics.cpp\
#		soroPneumaticKinematics.cpp

SOURCES += *.cpp
SOURCES += *.cxx

#HEADERS  += MainWindow.h\
#		soroPneumaticKinematics.h

HEADERS += *.h

FORMS    += MainWindow.ui

RESOURCES += \
    ShapeLab.qrc

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../QMeshLib/release/ -lQMeshLib
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../QMeshLib/debug/ -lQMeshLib
else:unix: LIBS += -L$$OUT_PWD/../QMeshLib/ -lQMeshLib

INCLUDEPATH += $$PWD/../QMeshLib/PQPLib
INCLUDEPATH += $$PWD/../QMeshLib
DEPENDPATH += $$PWD/../QMeshLib

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../QMeshLib/release/libQMeshLib.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../QMeshLib/debug/libQMeshLib.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../QMeshLib/release/QMeshLib.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../QMeshLib/debug/QMeshLib.lib
else:unix: PRE_TARGETDEPS += $$OUT_PWD/../QMeshLib/libQMeshLib.a

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../GLKLib/release/ -lGLKLib
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../GLKLib/debug/ -lGLKLib
else:unix: LIBS += -L$$OUT_PWD/../GLKLib/ -lGLKLib

LIBS += -lopengl32
LIBS += -lglu32


INCLUDEPATH += $$PWD/../GLKLib
DEPENDPATH += $$PWD/../GLKLib

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../GLKLib/release/libGLKLib.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../GLKLib/debug/libGLKLib.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../GLKLib/release/GLKLib.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../GLKLib/debug/GLKLib.lib
else:unix: PRE_TARGETDEPS += $$OUT_PWD/../GLKLib/libGLKLib.a


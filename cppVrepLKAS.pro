QT -= core
QT -= gui

TARGET = imacsLKAS
TEMPLATE = app

IMACSROOT = $(PWD)
OPENCV_PATH = /home/sajid/apps/opencv
VREP_PATH   = $$IMACSROOT/externalApps/vrep
EIGEN_PATH  = $$IMACSROOT/externalApps/eigen
PKG_CONFIG_PATH = /usr/lib/pkgconfig

DEFINES -= UNICODE
CONFIG   += console
CONFIG   -= app_bundle

DEFINES += NON_MATLAB_PARSING
DEFINES += MAX_EXT_API_CONNECTIONS=255
DEFINES += DO_NOT_USE_SHARED_MEMORY
DEFINES += HALIDE_NO_JPEG
DEFINES += _GLIBCXX_USE_CXX11_ABI=0



*-msvc* {
    QMAKE_CXXFLAGS += -W3
}
*-g++* {
    QMAKE_CXXFLAGS += -Wall
    QMAKE_CXXFLAGS += -Wno-unused-parameter
    QMAKE_CXXFLAGS += -Wno-strict-aliasing
    QMAKE_CXXFLAGS += -Wno-empty-body
    QMAKE_CXXFLAGS += -Wno-write-strings
    QMAKE_CXXFLAGS += -g
    QMAKE_CXXFLAGS += -Wno-unused-but-set-variable
    QMAKE_CXXFLAGS += -Wno-unused-local-typedefs
    QMAKE_CXXFLAGS += -Wno-narrowing
    QMAKE_CXXFLAGS += -std=c++11 

    QMAKE_CFLAGS += -Wall
    QMAKE_CFLAGS += -Wno-strict-aliasing
    QMAKE_CFLAGS += -Wno-unused-parameter
    QMAKE_CFLAGS += -Wno-unused-but-set-variable
    QMAKE_CFLAGS += -Wno-unused-local-typedefs  
	 
}


win32 {
    LIBS += -lwinmm
    LIBS += -lWs2_32
}

macx {
}


unix:!macx {
    LIBS += -lrt
    LIBS += -ldl
    LIBS += -lm
    LIBS += `pkg-config opencv --cflags --libs`
    LIBS += -L/usr/lib
    LIBS += -L$$PKG_CONFIG_PATH
    LIBS += -L$$OPENCV_PATH -ljpeg
    LIBS += `libpng-config --cflags --ldflags`
    LIBS += -lpthread
    OBJECTS_DIR = $$IMACSROOT/obj
}


INCLUDEPATH += "$$VREP_PATH/programming/include"
INCLUDEPATH += "$$VREP_PATH/programming/remoteApi"
INCLUDEPATH += "$$EIGEN_PATH"
INCLUDEPATH += "$$IMACSROOT/src/cpp_vrep_api"
INCLUDEPATH += "$$IMACSROOT/include"
INCLUDEPATH += "$$PKG_CONFIG_PATH"
INCLUDEPATH += "$$IMACSROOT/src/LaneDetection_and_Control"

SOURCES += \
	$$IMACSROOT/src/cpp_vrep_api/cpp_vrep_framework.cpp \
	$$IMACSROOT/src/cpp_vrep_api/my_vrep_api.cpp \
	$$IMACSROOT/src/LaneDetection_and_Control/lane_detection.cpp \
	$$IMACSROOT/src/LaneDetection_and_Control/lateralcontrol_multiple.cpp \  
	$$IMACSROOT/src/cpp_vrep_api/utils.cpp \
    $$VREP_PATH/programming/remoteApi/extApi.c \
    $$VREP_PATH/programming/remoteApi/extApiPlatform.c \
    $$VREP_PATH/programming/common/shared_memory.c

HEADERS +=\
    $$VREP_PATH/programming/remoteApi/extApi.h \
    $$VREP_PATH/programming/remoteApi/extApiPlatform.h \
    $$VREP_PATH/programming/include/shared_memory.h \

unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /usr/lib
    }
    INSTALLS += target
}

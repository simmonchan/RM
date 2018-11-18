QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += /usr/local/include \
                /usr/local/include/opencv \
                /usr/local/include/opencv2

LIBS += /usr/local/lib/libopencv_highgui.so.3.3 \
        /usr/local/lib/libopencv_core.so.3.3    \
        /usr/local/lib/libopencv_imgproc.so.3.3 \
        /usr/local/lib/libopencv_imgcodecs.so.3.3\
        /usr/local/lib/libopencv_videoio.so.3.3\
        /usr/local/lib/libopencv_calib3d.so.3.3\
        /usr/local/lib/libopencv_ml.so\
        /usr/local/lib/libopencv_dnn.so.3.3

SOURCES += main.cpp \
    CRC_Check.cpp \
    serialport.cpp \
    RM_v4l2.cpp \
    solvepnp.cpp \
    camera_calibration.cpp \
    sudoku.cpp \
    color_detect.cpp

HEADERS += \
    CRC_Check.h \
    serialport.h \
    RM_v4l2.h \
    solvepnp.h \
    camera_calibration.h \
    sudoku.h \
    color_detect.h

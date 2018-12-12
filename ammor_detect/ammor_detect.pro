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
    src/ArmorFind/ammor_find.cpp\
    src/Camera/camera_calibration.cpp\
    src/Camera/RMVideoCapture.cpp\
    src/Serial/CRC_Check.cpp\
    src/Serial/serialport.cpp\
    src/Stereo_vision/solvepnp.cpp\
    src/Stereo_vision/stereo_vision.cpp \
    src/ArmorFind/armorpredict.cpp




HEADERS += include/Header.h\
    include/ArmorFind/ammor_find.h\
    include/Camera/camera_calibration.h\
    include/Camera/RMVideoCapture.h\
    include/Serial/CRC_Check.h\
    include/Serial/serialport.h\
    include/Stereo_vision/solvepnp.h\
    include/Stereo_vision/stereo_vision.h \
    include/ArmorFind/armorpredict.h

SUBDIRS += \
    ammor_detect.pro



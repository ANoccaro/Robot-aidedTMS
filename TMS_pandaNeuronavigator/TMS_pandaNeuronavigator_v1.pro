QT += core gui
QT += network
QT += serialport

CONFIG += c++14

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += main.cpp\
        UDPmanager.cpp \
    #../../include/RoboticLibrary.cpp \
    pandaclass_3.cpp \
    mainwindow_ee.cpp

INCLUDEPATH += ../include/eigen-3.3.7 \
    /home/panda/libfranka/include \
    ../include

HEADERS  += mainwindow.h \
   pandaclass_3.h \
    ../../include/examples_common.h \
        UDPmanager.h \


LIBS +=  -lm \
-lpthread \
/home/panda/libfranka/build/libfranka.so \
/home/panda/libfranka/build/libfranka.so.0.5.0 \
/usr/lib/libPocoNet.so.9 \
/usr/lib/libPocoFoundation.so.9 \
-lX11

FORMS    += mainwindow.ui


# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

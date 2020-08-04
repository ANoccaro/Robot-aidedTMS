#include "mainwindow.h"
#include <QApplication>
#include "UDPmanager.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    MyUDP UDPclass;
    QObject::connect( &UDPclass, SIGNAL(ReceivedData()), &w, SLOT(UpdateCameraData) );

    int r=a.exec();

    UDPclass.MyUDPclosing();

    return r;
}

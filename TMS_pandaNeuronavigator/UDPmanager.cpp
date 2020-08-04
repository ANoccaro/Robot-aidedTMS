// myudp.cpp

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include "UDPmanager.h"




MyUDP::MyUDP(QObject *parent) :
    QObject(parent)
{

    socket = new QUdpSocket();

    // The most common way to use QUdpSocket class is
    // to bind to an address and port using bind()
    // bool QAbstractSocket::bind(const QHostAddress & address,
    //     quint16 port = 0, BindMode mode = DefaultForPlatform)
    //socket->bind(QHostAddress::LocalHost, 65002);

    socket->bind(QHostAddress::AnyIPv4 , 1237);

    connect(socket, SIGNAL(readyRead()), this, SLOT(readPendingDatagrams()));


    qDebug()<<"udp connesso";

}


void MyUDP::readPendingDatagrams()
{
    while (socket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(socket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;

        socket->readDatagram(datagram.data(), datagram.size(),
                                &sender, &senderPort);


        //processTheDatagram(datagram);
        qDebug() << "Message from: " << sender.toString();
        qDebug() << "Message port: " << senderPort;
        qDebug() << "Message: " << datagram;
    }
}


void MyUDP::Create(){

    // create a QUDP socket
    socket = new QUdpSocket();

    // The most common way to use QUdpSocket class is
    // to bind to an address and port using bind()
    // bool QAbstractSocket::bind(const QHostAddress & address,
    //     quint16 port = 0, BindMode mode = DefaultForPlatform)
    socket->bind(QHostAddress::AnyIPv4 , 1238);

    connect(socket, SIGNAL(readyRead()), this, SLOT(readPendingDatagrams()));


    //qDebug()<<"udp connesso";

}

void MyUDP::HelloUDP()
{
    QByteArray Data;
    Data.append("Hello from UDP");

    // Sends the datagram datagram
    // to the host address and at port.
    // qint64 QUdpSocket::writeDatagram(const QByteArray & datagram,
    //                      const QHostAddress & host, quint16 port)
    socket->writeDatagram(Data, QHostAddress::LocalHost, 1235);

    //qDebug()<<"udp inviato";



}

void MyUDP::readyRead()
{
    // when data comes in
    QByteArray buffer;
    buffer.resize(socket->pendingDatagramSize());

    QHostAddress sender;
    quint16 senderPort;

    // qint64 QUdpSocket::readDatagram(char * data, qint64 maxSize,
    //                 QHostAddress * address = 0, quint16 * port = 0)
    // Receives a datagram no larger than maxSize bytes and stores it in data.
    // The sender's host address and port is stored in *address and *port
    // (unless the pointers are 0).

    socket->readDatagram(buffer.data(), buffer.size(),
                         &sender, &senderPort);


    qDebug() << "Message from: " << sender.toString();
    qDebug() << "Message port: " << senderPort;
    qDebug() << "Message: " << buffer;

     emit ReceivedData(buffer);
}

void MyUDP::SendData(QByteArray data){
    // Sends the datagram datagram
    // to the host address and at port.
    // qint64 QUdpSocket::writeDatagram(const QByteArray & datagram,
    //                      const QHostAddress & host, quint16 port)

    //socket->writeDatagram(*data, QHostAddress::LocalHost, 1235);
    QByteArray d=data;

    socket->writeDatagram(d, QHostAddress::LocalHost, 1237);

    QString st=d;
    //qDebug()<<st;
    //qDebug()<<QString(*data);


}

void MyUDP::MyUDPclosing(){

    socket->close();
}

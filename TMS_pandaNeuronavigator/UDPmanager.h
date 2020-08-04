
#ifndef MYUDP_H
#define MYUDP_H

#include <QObject>
#include <QUdpSocket>

Q_DECLARE_METATYPE(QByteArray);

class MyUDP : public QObject
{
    Q_OBJECT
public:
    explicit MyUDP(QObject *parent = 0);
    void HelloUDP();
    void SendData(QByteArray data);
signals:
    void ReceivedData(QByteArray data);

public slots:
    void readyRead();
    void MyUDPclosing();
    void readPendingDatagrams();
    void Create();

private:
    QUdpSocket *socket;

};

#endif // MYUDP_H

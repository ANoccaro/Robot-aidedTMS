/*
 *	AUTHOR: Alessia Noccaro, PhD student
 *	Research Unit of Neurophysiology and Neuroengineering of Human-Technology Interaction
 *	Università Campus Bio-Medico di Roma
 *	Via Alvaro del Portillo, 21
 *	00128 Roma, Italy
 *	email: a.noccaro@unicampus.it
*/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
//#include <testremote2.h>
#include <QTimer>
#include <pandaclass_3.h>
#include <QThread>


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();


  //  TestRemote2 testremote;
    QTimer *timer;
    PandaClass panda;
    QThread *pandathread;

signals:
    void goTracking();
    void goResting();
    void goReArrangeRobot();


public slots:
    void onLoadClicked();
    void onConnectClicked();
    void onDisconnectClicked();
    void onCameraConnected();
    void onCameraDisconnected();
    void onRobotConnected();
    void onRobotDisconnected();
    void onRobotStarted();
    void onRobotStopped();
    void onRobotFinished();
    void onRobotError();
    void onAcquirePointClicked();
    void onLoadPointClicked();
    void onSofTaxicPointClicked();
    void onSTARTClicked();
    void onSTOPClicked();
    void onRestClicked();
    void onNewForceValue();             //slot to update forces value on gui
    void onHeadCompensationStarted();   //slot to display Force and Error value on gui
    void onHeadCompensationStarted_RE();
    void onPointSelected();
    void onEmergencyStop();

    void onStartStimulationClicked();
    void onStopStimulationClicked();
    void on10StimulationClicked();
    void onManualStimulationClicked();
    void onConnectSerialPortClicked();
    void onDisconnectSerialPortClicked();
    void onSerialDeviceSelection();

    void SendTriggerTMS();

    void onPrintData(QVector<double> data);     //slot to print debug data in separate function
    void UpdateCameraData(QByteArray data);





private slots:
    void on_pushButtonSavePoint_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H

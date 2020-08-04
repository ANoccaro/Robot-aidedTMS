/*
 *	AUTHOR: Alessia Noccaro, PhD student
 *	Research Unit of Neurophysiology and Neuroengineering of Human-Technology Interaction
 *	Università Campus Bio-Medico di Roma
 *	Via Alvaro del Portillo, 21
 *	00128 Roma, Italy
 *	email: a.noccaro@unicampus.it
*/

#ifndef PANDACLASS_H
#define PANDACLASS_H

#include <QObject>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <franka/robot.h>
#include <QVector>

Q_DECLARE_METATYPE(QVector<double>)

class PandaClass : public QObject
{
    Q_OBJECT
public:
    explicit PandaClass(QObject *parent = 0);
//    franka::Robot robot;


signals:
    void connected();   //robot connection enstablished
    void started();     //robot starts to move
    void stopped();     //robot stops to move
    void finished();
    void error();       //error or exception
    void HeadCompensation();    //signal to start the head motion compensation
    void HeadCompensation_RE(); //signal to restart the head compensation after rearrangment;
    void EmergencySTOP();
    void newForceValue();       //signal to update the forces value on gui
    void printData(QVector<double> d);

public slots:
    void run();
    void initialize();  //robot connection initialization
    void goTracking();  //go to stimulation point and keep it
    void goResting();   //go to resting pose
    void goHeadCompensation();//to start the head motion compensation
    void onEmergencySTOP();
    void onReArrangeRobot();    //re-arrange the robot on the target pose

    //debug
    void goHeadCompensation_0();//to start the head motion compensation
    void goHeadCompensation_1();//to start the head motion compensation





};

#endif // PANDACLASS_H

/*
 *  This code creates a GUI for the TMS robot-aided using the Panda robots and
 *  a generic neuronavigator software able to online stream the coil and head pose, together with the hotspot. 
 *
 *  The robot moves to the stimulation point with a torque control and to the resting pose with a cartesian velocity control.
 *  The head motion compensation is done with an Impedance Control (Torque Control).
 *  
 *
 *  The robot is moved setting the coil frame as EE frame (transformation between the flange and the coil.
 *  Also targets and errors are computed in the EE (coil) frame.
 *
 *  An EMERGENCY control avoid the robot following the head if the head is moving too fast,
 *  both during tracking phase and initial moving to hot-spot phase.
 *
 *  When the coil is on the Hot Spot you can select the maximum acceptable error.
 *  You can start the stimulation selecting the insterstimuli time.
 *
 *  If the error is too high for long time, the robot moves behind and then
 *  goes again on the target point.
 *
 *  The stimulation will send a trigger to the TMS only if the error is acceptable.
 *
 *  The target could be acquired from the camera data. The robot control uses a target more close to the
 *  head to compensate the residual control error in the z axis.
 *
 *
 *
 *	AUTHOR: Alessia Noccaro, PhD student
 *	Research Unit of Neurophysiology and Neuroengineering of Human-Technology Interaction
 *	Università Campus Bio-Medico di Roma
 *	Via Alvaro del Portillo, 21
 *	00128 Roma, Italy
 *	email: a.noccaro@unicampus.it
*/




#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <pandaclass_3.h>
#include <QFile>
#include <QTextStream>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/duration.h>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <thread>
#include <QObject>
#include <QElapsedTimer>
//#include <QDateTime>
#include <QDir>
#include <error.h>
//#include <RoboticLibrary.h>

#include <examples_common.h>
#include "UDPmanager.h"


#define DEBUG

#ifdef DEBUG
#include <QDebug>
#endif


#define BA2QSAssign(qsDest, baSrc)	qsDest = QString::fromUtf8((const char *)&baSrc)
#define QS2BAAssign(baDest, qsSrc)	strncpy((char *)&baDest, qsSrc.toUtf8().data(), sizeof baDest)



//----------------------------------
//Define constant
//----------------------------------
const int NUM_TARGET=100;   //Define max number of target points
const double pos_error=0.01; //0.01 ;   //max position head movement   [m]
const double ori_error=0.17453292519943/2; //0.17453292519943/2;  //(5 deg)   //max orientation head movement   [rad]
const double DeltaSafe=0.1;             //Safety Delta along EE z axis (i.e. coil normal axis)  [m]
const double DeltaControlTarget=0.003;//0.004;  //Delta to move the target "inside" the head (to compensate control error) along EE z axis (i.e. coil normal axis)  [m]
const double DeltaArrangement=0.01;    //[m] Delta along the coil z axis to rearrange the coil if the error is still high

const double pos_error_secondary= 0.1;              //max head movement before coil touching the head, at camera rate 20Hz [m]
const double ori_error_secondary=0.17453292519943*2;//max head movement before coil touching the head, at camera rate 20Hz [rad] (20 deg)

const double maxErrorTime=5.0; //[s] max time with an error higher than the threshold

//----------------------------------
//Define rom ID:
//----------------------------------
QString headTool="8700338";
QString coilTool="8700339";

//-----------------------------------------------
//Define Minimm Jerk functions
//-----------------------------------------------
double FifthOrderPoly(double p0, double pf, double c);
void LogSO3(Eigen::Matrix3d R, Eigen::Vector3d &w, double &theta);
Eigen::Matrix3d ExpSO3(Eigen::Vector3d r_c);
Eigen::Matrix3d SKEW(Eigen::Vector3d a);


//-----------------------------------------------
//Input/Output txt files
//-----------------------------------------------
QFile outputRobot("robot.txt");
QFile outputCamera("camera.txt");
QFile calibrationFile("calibration.txt");   //txt file with calibration matrices
QFile targetPoint("target.txt");            //txt file with target point (with respect to head referance frame)
QFile LogFile("LogFile.txt");
QFile errorRobot("errorByRobot.txt");       //txt file with pose error computed as delta robot pose
QFile errorCamera("errorByCamera.txt");     //txt file with pose error computed as delta coil pose read by camera
QFile interactionForces("forces.txt");      //txt file with Forces and Torques at the K frame during head motion compensation
QFile cmdRobot("cmdRobot.txt");
QFile stimulation("stimulation.txt");

//-----------------------------------------------
//Data saving frequency
//-----------------------------------------------
const unsigned int saving_frequency=1000; //[Hz]
const unsigned int display_frequency=5;    //[Hz]
const int sampling_count=1000/saving_frequency; //number of samples for donwsampling
const int sampling_count_gui=1000/display_frequency;


//-----------------------------------------------
//Help variables
//-----------------------------------------------
bool pose_end=false;	//true if the final pose is reached
bool one=true;          //first loop iteration: initialize final time
bool rest_time=false;   //true during a rest time (1 s) between different points
bool START_FLAG=false;  //true if the camera streaming is started
bool STOP=false;        //true if the robot is stopped (by the user or by the emergency control)
bool onHEAD=false;      //true if the robot is on the head (Safety point reached or HeadCompensation)
bool TRACKING=false;    //true if the robot is on the hot-spot compensating the head motion
bool CoilNotVisible=false;  //true if the coil is not visible from the camera
bool GOstimulation=false;   //true if the coil is on the hot spot and error is acceptable: you can stimulate
bool stimuli_10=false;          //true if you have to provide 10 stimuli
bool ONstimulation=false;       //true if the stimulation is started
unsigned int Session=0;     //number of the current tracking session (updated when START is pressed
unsigned int GlobalSession=0;   //increase as the start button is pressed
double errorTimer=0;        //Count the time the error is higher than the set threshold to re-arrange the robot
double noErrorTimer=0;      //count the time the error is lower than the set threshold


//robot trajectories
unsigned int Point=0;	//Point counter
bool safe=false;    //true if safe point is reached
std::mutex TargetMutex;
std::mutex TrackingMutex;
std::mutex ForceMutex;  //mutex to read robot forces and torques
std::mutex ErrorMutex;  //mutex to set error on gui
std::mutex SofTaxicMutex; //mutex to read target from SofTaxic Optic

//-----------------------------------------------
//Time variables
//-----------------------------------------------
double time_first_point=10.0;//20.0;   //time to reach the first point
double time_far=5.0;//10.0            //time to reach far points on the sphere
double time_near=3.0;           //time to reach near points on the sphere
double time_rest=60.0;           //time for impedance control in rest pose
double time_first_interpolation=2.0;//2.5;    //time to interpolate head motion compensation at the first loop
double time_interpolation=0.1; //0.1;  //time to interpolate head motion compensation
double GLOBAL_TIME=0.0;         //global time of the calibration
double time_arrangement=0.5;  //time to re-arrange the coil
QElapsedTimer ElapsTime;    //time elapsed from the start of the code
QElapsedTimer RearrangeElapsTime;
double old_elapsed_time=0;  //time elapsed from the old loop (in Error and ReArrangement functions)

//-----------------------------------------------
//Cartesian Impedance values
//-----------------------------------------------
double kxy=3000;//5000;//2000;//2800;        //K on xy plane (wrt end-effector frame)
double kr=100;//100;//40;//100;         //K on xy plane (wrt end-effector frame)
double damp_xy= 50.0;
double damp_r= 10.0;

//-----------------------------------------------
//Global variables
//-----------------------------------------------
Eigen::Matrix4d T_cam_head;
Eigen::Matrix4d T_cam_head_filt_old, T_cam_head_filt;

Eigen::Matrix4d T_head_target;  //T_head_EE(target)
Eigen::Matrix4d T_head_coil;
Eigen::Matrix4d T_head_coil_target; //T_head_coil(target)
Eigen::Matrix4d T_cam_coil;
Eigen::Matrix4d T_EE_coil;
Eigen::Matrix4d T_bR_cam;
Eigen::Matrix4d T_EE_delta;
Eigen::Matrix4d *Target= new Eigen::Matrix4d[NUM_TARGET];
Eigen::Matrix4d *SafeTarget= new Eigen::Matrix4d[NUM_TARGET];   //Safety points array (Safe Delta over the head and the actual target)
Eigen::Matrix4d *TargetControl= new Eigen::Matrix4d[NUM_TARGET];
Eigen::Matrix4d SofTaxicTarget;

Eigen::Matrix4d T_cam_head_old; //old head reference pose
Eigen::Matrix4d T_ctrl;         //control pose for the robot
Eigen::Matrix4d T_ctrl_old;     //control pose at the time i-1
Eigen::Matrix4d T_rest;         //rest pose

Eigen::Matrix4d T0, T_old, Ti, Tf;
Eigen::Vector3d p0, p_old, pi, pf;
Eigen::Matrix3d R0, R_old, Ri, Rf;
Eigen::Vector3d w_t;
double theta_t, qc_old;

double FT_interaction[6]={0.0,0.0,0.0,0.0,0.0,0.0};   //interaction forces and torque at the robot stiffness frame (defined as coil)
double FT_interaction_MAX[6]={7.0,7.0,7.0,7.0,7.0,7.0};//max values for interaction forces nd torques
double error_position_max=5.0;  //max error position from target[mm]
double error_orientation_max=2.0; //max error orientation from target [deg]
double PositionError[3];
double OrientationError;

double ErrorThreshold_position;         //acceptable coil position error
double ErrorThreshold_orientation;      //acceptable coil orientation error
double ErrorThreshold_z;                //acceptable coil distance from the head

int stimCount=0;    //counter for TMS stimulation

QTimer *stimTimer=new QTimer(0);    //Timer for the stimulation protocol
QSerialPort serial;                 //serial port to send trigger


//Minimum Jerk trajectory phases
typedef enum{
    ONE,
    REST,
    MINJERK,
    TRACK,
    END
}Phase;

//Camera and Robot data
struct {
   std::mutex mutex;
   bool has_data;
   Eigen::Matrix4d T_camera;
   Eigen::Matrix4d T_robot;
   QString romID;
   double camera_time;
   double global_time;
 } print_data{};
std::atomic_bool running{true};



//-----------------------------------------------
// Cameras variables
//-----------------------------------------------
typedef struct CameraData{
    Eigen::Quaterniond q;
    Eigen::Vector3d p;
    Eigen::Matrix3d R;
    Eigen::Matrix4d T;
    float timeStamp;
};

CameraData coil;
CameraData head;
CameraData item;

//-----------------------------------------------
// Cameras variables
//-----------------------------------------------
int ID_COIL=0;
int ID_HEAD=1;

//-----------------------------------------------
// Mutex
//-----------------------------------------------
std::mutex coilMutex;


//-----------------------------------------------
//Camera Read Function
//-----------------------------------------------
void MainWindow::UpdateCameraData(QByteArray data){


    //Read data received from UDP
    qDebug() << "Message: " << data;

    //unpack Optitrack data
    QList<QByteArray> list=data.split(' ');

    int id= list[0].toInt();
    float x= list[1].toFloat();
    float y= list[2].toFloat();
    float z= list[3].toFloat();
    float qx= list[4].toFloat();
    float qy= list[5].toFloat();
    float qz= list[6].toFloat();
    float qw= list[7].toFloat();
    float tmp= list[8].toFloat();



    //copy data into global variables
    if(coilMutex.try_lock()){

        item.p= Eigen::Vector3d(x, y,z);
        item.q= Eigen::Quaterniond(qw,qx,qy,qz);
        item.R=item.q.toRotationMatrix();
        item.T=Eigen::Matrix4d::Identity();
        item.T.block(0,0,3,3)=item.R;
        item.T.block(0,3,3,1)=item.p;

        item.timeStamp=tmp;

        qDebug()<<"p " << item.p.x() << ","<<item.p.y() << ","<<item.p.z();
        qDebug()<<"R " << item.R(0,0) << ","<<item.R(0,1) << ","<<item.R(0,2)<< "\n"<<
                        item.R(1,0)<< ","<<item.R(1,1) << ","<<item.R(1,2) <<"\n"<<
                        item.R(2,0) << ","<<item.R(2,1) << ","<<item.R(2,2)<< "\n";

        coilMutex.unlock();
    }



    bool HeadVisible=false;  //true if the head is visible and its measure is valid

    //check the id --> coil / head
    if(id==ID_COIL){

        if(TargetMutex.try_lock()){
             T_cam_coil=item.T;
             TargetMutex.unlock();
        }


    }
    else if(id==ID_HEAD){
        HeadVisible=true;



       //old pose
       T_cam_head_old=T_cam_head;

       if(TargetMutex.try_lock()){
            T_cam_head=item.T;
            TargetMutex.unlock();
       }
       if(!START_FLAG){
          T_cam_head_old=item.T;
          T_cam_head_filt=item.T;
          T_cam_head_filt_old=item.T;
          START_FLAG=true;
      }
    }


    // read coil pose with respect to the head
    //?
    //T_head_coil

    if(TRACKING){
        //-----------------------------------------------
        //Compute Error Data
        //-----------------------------------------------
        Eigen::Matrix3d Rc1,Rc2,Re;
        Eigen::Vector3d pc1,pc2,pe,vec;
        double alpha;

        if(TargetMutex.try_lock()){
            for(int i=0;i<3;i++){
                for(int j=0;j<3;j++){
                    Rc1(i,j)=T_head_coil(i,j);
                    Rc2(i,j)=T_head_coil_target(i,j);
                }
                pc1(i)=T_head_coil(i,3);
                pc2(i)=T_head_coil_target(i,3);
            }
            TargetMutex.unlock();
        }

        //verificare che l'ordine sia giusto ??
        Re=Rc1*Rc2.transpose();
        LogSO3(Re,vec,alpha);
        //error wrt COIL frame
        pe=Rc1.transpose()*(pc1-pc2);


        if(ErrorMutex.try_lock()){
            //update global variables
            for(int i=0;i<3;i++)
                PositionError[i]=pe(i);
            OrientationError=fabs(alpha);

            ErrorMutex.unlock();
        }






    qDebug() << "id: " << id << "; x: "<<x<< "; y: "<<y<< "; z: "<<z;

}
}





//-----------------------------------------------
//Robot Control Function
//-----------------------------------------------

franka::Robot robot("192.168.2.130");
//franka::Robot robot("192.168.1.131");


void PandaClass::initialize(){

    try{

        franka::RobotState state=robot.readOnce();

#ifdef DEBUG
        for(int i=0;i<16;i++)
        qDebug()<<state.O_T_EE[i];
#endif

        Eigen::Matrix4d T_F_EE;

        T_F_EE(0,0)=1;   T_F_EE(0,1)=0;    T_F_EE(0,2)=0;   T_F_EE(0,3)=0;
        T_F_EE(1,0)=0;   T_F_EE(1,1)=1;    T_F_EE(1,2)=0;   T_F_EE(1,3)=0;
        T_F_EE(2,0)=0;   T_F_EE(2,1)=0;   T_F_EE(2,2)=1;  T_F_EE(2,3)=0;

        T_F_EE(3,0)=0;   T_F_EE(3,1)=0;    T_F_EE(3,2)=0;  T_F_EE(3,3)=1;


        std::array<double,16> TFEE;
        Eigen::Matrix4d::Map(&TFEE[0])=T_F_EE;

         robot.setEE(TFEE);




        emit connected();

    } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    emit error();
    }

}



//Impedance Control
void PandaClass::goTracking(){
    try{

        #ifdef DEBUG
            //debug
            std::cout<<"panda run \n";
        #endif

         // Compliance parameters
         const double translational_stiffness=kxy;

         const double rotational_stiffness=kr;

         Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
         stiffness.setZero();
         stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);



         stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
         damping.setZero();
         damping.topLeftCorner(3, 3) << damp_xy * Eigen::MatrixXd::Identity(3, 3);


         damping.bottomRightCorner(3, 3) << damp_r * Eigen::MatrixXd::Identity(3, 3);



         //-----------------------------------------------
         // Set Stiffness Frame
         //-----------------------------------------------
         std::array<double,16> T_EE_K;
         for(int i=0;i<16;i++){
             T_EE_K[i]=T_EE_coil(i);
         }

         robot.setK(T_EE_K);



             //-----------------------------------------------
             //Load Kinematic and Dynamic Model
             //-----------------------------------------------

            //setDefaultBehavior(robot);
            // load the kinematics and dynamics model
            franka::Model model = robot.loadModel();

            franka::RobotState initial_state = robot.readOnce();

            // equilibrium point is the initial position
            Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
            Eigen::Vector3d position_d(initial_transform.translation());
            Eigen::Quaterniond orientation_d(initial_transform.linear());

            // set collision behavior
            robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                       {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                       {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                       {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

            // define callback for the torque control loop

            //-----------------------------------------------
            // Set initial values
            //-----------------------------------------------
            R_old=R0;
            T_old=T0;
            p_old=p0;


            double time=0.0;
            double time_tot= time_first_point;


            Phase phase;
            phase=ONE;
            safe=false;

            GLOBAL_TIME=0.0;

            while (!START_FLAG) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
#ifdef DEBUG
                std::cout<<"wait for camera \n";
                std::cout<<START_FLAG<<"\n";
#endif
            }
#ifdef DEBUG
            std::cout<<"START FLAG = true \n";
            std::cout<<"Start Panda control Loop \n";
#endif

            emit started();

            //debug
             unsigned int count=0;


             Eigen::Matrix4d T_bR_head, T_bR_target;

            //--------------------------------------------
            // Butterworth low-pass filter for dq at 1 kHz cutoff=10Hz (0.02)
            //--------------------------------------------
            double dq_old[7];
            double dq_filt_old[7];
            double b_=0.0305;
            double a_=-0.9391;

            for(int i=0;i<7;i++){
                dq_old[i]=0.0;
                dq_filt_old[i]=0.0;
            }
            //---------------------------------------------


            TRACKING=true;


            //-----------------------------------------------
            //Control Loop
            //-----------------------------------------------
            std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
                            impedance_control_callback = [ &model,&count,&b_, &a_,&dq_old,&dq_filt_old, &time, &S,&time_tot, &position_d, &stiffness, &damping,
                    &orientation_d, &T_bR_head, &T_bR_target, this](const franka::RobotState& robot_state,
                                                             franka::Duration period /*duration*/) -> franka::Torques {



            bool DONE=false;

            count++;
            //-----------------------------------------------
            //Print Thread
            //-----------------------------------------------
            GLOBAL_TIME += period.toSec();
            // Update data to print.
            if (print_data.mutex.try_lock()) {
                for(int i=0;i<16;i++){
                    print_data.T_robot(i)=robot_state.O_T_EE_d[i];
                    print_data.global_time=GLOBAL_TIME;
                }

              print_data.has_data = true;
              print_data.mutex.unlock();
            }

            //-----------------------------------------------
            //SECONDARY EMERGENCY STOP: if the head is moving too fast
            //-----------------------------------------------
            if(TargetMutex.try_lock()){
                double px,py,pz;
                Eigen::Matrix3d R_help_old,R_help,R_;
                Eigen::Vector3d t_;
                double mov_th;

                px= fabs(T_cam_head_old(0,3)-T_cam_head(0,3));
                py= fabs(T_cam_head_old(1,3)-T_cam_head(1,3));
                pz= fabs(T_cam_head_old(2,3)-T_cam_head(2,3));
                for(int i=0;i<3;i++){
                    for(int j=0;j<3;j++){
                        R_help_old(i,j)=T_cam_head_old(i,j);
                        R_help(i,j)=T_cam_head(i,j);
                    }
                }

                TargetMutex.unlock();

                R_=R_help_old.transpose()*R_help;

                LogSO3(R_,t_,mov_th);

                //check if the head is moving too fast
                if(px >= pos_error_secondary || py >= pos_error_secondary || pz >= pos_error_secondary || fabs(mov_th)>=ori_error_secondary ){

                    qDebug()<<"== SECONDARY EMERGENCY STOP! ==";
                    emit EmergencySTOP();

                }
            }





            //-----------------------------------------------
            // Get state variables
            //-----------------------------------------------
          std::array<double, 7> coriolis_array = model.coriolis(robot_state);
          std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

          // convert to Eigen
          Eigen::Map<const Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
          Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
          Eigen::Map<const Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
          Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
          Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
          Eigen::Vector3d position(transform.translation());
          Eigen::Quaterniond orientation(transform.linear());


          Eigen::Matrix4d T_c_EE;

            //-----------------------------------------------
            //Robot Control: Compute Trajectories
            //-----------------------------------------------

            if(time==0.0)
            {
                while(!DONE){
                  if(TargetMutex.try_lock()){

                      T_bR_head=T_bR_cam*T_cam_head_filt;
                      TargetMutex.unlock();

                      if(!safe){
                          for(int i=0;i<16;i++)
                              Ti(i)=robot_state.O_T_EE[i];
                          T_head_target=SafeTarget[0];    //select safe point
                          time_tot=time_first_point;
                          //set false the flag when the robot is NOT on subject's head
                          onHEAD=false;
                      }
                      else{
                          for(int i=0;i<16;i++)
                              Ti(i)=robot_state.O_T_EE[i];
                          T_head_target=TargetControl[0];     //select stimulation point (deepest one)
                          time_tot=time_far;
                      }

                      T_bR_target=T_bR_head*T_head_target;

                      Tf=T_bR_target;

                      DONE=true;
                  }
                }


#ifdef DEBUG
                    qDebug()<<"Initial Pose:";
                    for(int i=0;i<16;i++)
                        qDebug()<<Ti(i);
                    qDebug()<<"Target base referred:";
                    for(int i=0;i<16;i++)
                        qDebug()<<Tf(i);
                     qDebug()<<"== Motion Started ==\n";
#endif


                //extract position and rotation
                for(int i=0;i<3;i++){
                    pi(i)=Ti(i,3);
                    pf(i)=Tf(i,3);
                    for(int j=0;j<3;j++){
                        Ri(i,j)=Ti(i,j);
                        Rf(i,j)=Tf(i,j);
                    }
                }
                // Interpolate the Orientation
                // Compute rotation from initial to final pose
                Eigen::Matrix3d dR;
                dR=Ri.transpose()*Rf;

                //Log SO3
                Eigen::Vector3d w_t_;
                LogSO3(dR, w_t_, theta_t);
                w_t=w_t_;


                //update Desired values
                for(int i=0;i<3;i++){
                    position_d(i)=Tf(i,3);
                    for(int j=0;j<3;j++){
                        Rf(i,j)=Tf(i,j);
                    }
                }
                orientation_d=Rf;
             }

                //Min jerk first iteration:
                //-----------------------------------------------
                //Robot Control: Compute Trajectories
                //-----------------------------------------------
                time += period.toSec();

                //update final pose
              //-------------------
              if((count%5)==0){
              if(TargetMutex.try_lock()){

                  T_bR_head=T_bR_cam*T_cam_head_filt;
                  TargetMutex.unlock();
                }

                  T_bR_target=T_bR_head*T_head_target;

                  Tf=T_bR_target;

              //extract position and rotation
              for(int i=0;i<3;i++){
                  pf(i)=Tf(i,3);
                  for(int j=0;j<3;j++){
                      Rf(i,j)=Tf(i,j);
                  }
              }
              // Interpolate the Orientation
              // Compute rotation from initial to final pose
              Eigen::Matrix3d dR;
              dR=Ri.transpose()*Rf;

              //Log SO3
              Eigen::Vector3d w_t_;
              LogSO3(dR, w_t_, theta_t);
              w_t=w_t_;


              //update Desired values
              for(int i=0;i<3;i++){
                  position_d(i)=Tf(i,3);
                  for(int j=0;j<3;j++){
                      Rf(i,j)=Tf(i,j);
                  }
              }
              orientation_d=Rf;
               }

              //----------------------



                double c;
                if(time<time_tot){
                    c=time/time_tot;

                    double xc=FifthOrderPoly(pi(0),pf(0),c);
                    double yc=FifthOrderPoly(pi(1),pf(1),c);
                    double zc=FifthOrderPoly(pi(2),pf(2),c);
                    double qc=FifthOrderPoly(0.0,theta_t,c);

                    Eigen::Matrix3d Rc;
                    Rc=Ri* ExpSO3(qc*w_t);


                    //update Desired values
                    position_d(0)=xc;
                    position_d(1)=yc;
                    position_d(2)=zc;
                    orientation_d=Rc;

               }

                // compute error to desired equilibrium pose
                // position error
                Eigen::Matrix<double, 6, 1> error;
                error.head(3) << position - position_d;

                // orientation error
                // "difference" quaternion
                if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
                  orientation.coeffs() << -orientation.coeffs();
                }
                Eigen::Quaterniond error_quaternion(orientation * orientation_d.inverse());
                // convert to axis angle
                Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
                // compute "orientation error"
                error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

                // compute control
                Eigen::VectorXd tau_task(7), tau_d(7);

              //-------------------------------------------
              // Butterworth low-pass filter for dq
              //-------------------------------------------
              Eigen::Matrix<double, 7, 1> dq_filt;
              for(int i=0;i<7;i++){
              dq_filt(i)=b_*dq(i) + b_*dq_old[i] - a_*dq_filt_old[i];
              }
              for(int i=0;i<7;i++){
              dq_filt_old[i]=dq_filt(i);
              dq_old[i]=dq(i);
              }


              //-------------------------------------------
              //Print Log Data
              //-------------------------------------------

              // 0 robot dt
              // 1 GLOBAL TIME
              // 2:8  q robot
              // 9:15 q_dot robot
              // 16:22 tau_J
              // 23:38 T coil
              // 39:54 T head
              // 55:70 T target
              // 71:76 Error
              // 77:82 F coil

              if(count%sampling_count==0)
              {  //downsampling to 200 Hz


                QVector<double> printdata(77);
                printdata[0]=period.toSec();
                printdata[1]=ElapsTime.elapsed();  //elapsed time [s]
                //                printdata[1]=GLOBAL_TIME;

//                for(int i=0;i<7;i++){
//                    printdata[2+i]=q(i);
//                    printdata[9+i]=dq(i);
//                    printdata[16+i]=dq_filt(i);
//                }

                for(int i=0;i<16;i++){
                    printdata[2+i]=robot_state.O_T_EE[i];
                }
                printdata[18]= print_data.camera_time;

                if(TargetMutex.try_lock()){
                    for(int i=0;i<16;i++){
                        printdata[19+i]=T_cam_coil(i);
                        printdata[35+i]=T_cam_head_filt(i);
                        printdata[51+i]=Target[0](i);
                    }
                    TargetMutex.unlock();
                }
                if(ErrorMutex.try_lock()){
                    printdata[67]=PositionError[0];
                    printdata[68]=PositionError[1];
                    printdata[69]=PositionError[2];
                    printdata[70]=OrientationError;
                ErrorMutex.unlock();
                }

                for(int i=0;i<6;i++){
                    printdata[71+i]=robot_state.K_F_ext_hat_K[i];
                }

                emit printData(printdata);

            }


                tau_task << jacobian.transpose() * ( -stiffness * error - damping * (jacobian * dq));

                //           tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq_filt));

                tau_d << tau_task + coriolis;

                std::array<double, 7> tau_d_array{};
                Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

                  //---------------------

           if(time>=time_tot){
                if(!safe){
#ifdef DEBUG
                    qDebug()<<"Safe point Reached";
#endif
                    safe=true;
                    time=0.0;

                }
                else{
#ifdef DEBUG
                    qDebug()<<"Target point Reached";
#endif
                    time=0.0;
                    franka::Torques torques_ctrl={tau_d_array[0],tau_d_array[1],tau_d_array[2],tau_d_array[3],tau_d_array[4],tau_d_array[5],tau_d_array[6]};
                    return franka::MotionFinished( torques_ctrl);
                }
            }

           //debug
           for(int i=0;i<7;i++)
           qDebug()<<"Torque error:"<<i<<tau_d_array[i]-robot_state.tau_J_d[i];

           //debug
           qDebug()<<"P error:"<<position_d[0]-robot_state.O_T_EE[12]<<position_d[1]-robot_state.O_T_EE[13]<<position_d[2]-robot_state.O_T_EE[14];

//            return limitRate(kMaxTorqeRate, tau_d_array, robot_state.tau_J_d);
           return tau_d_array;
       };


       robot.control(impedance_control_callback);


      } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;

      }

    emit finished();

    //-----------------------------------------------
    //Call the Head Motion compensation function
    //-----------------------------------------------
    //check if the robot is in the target pose and has been not stopped!
    if(!STOP){
        emit HeadCompensation();
        STOP=false;
    }



}






void PandaClass::goHeadCompensation(){
    try{

        #ifdef DEBUG
            //debug
            std::cout<<"panda run \n";
        #endif

         // Compliance parameters
         const double translational_stiffness=kxy;

         const double rotational_stiffness=kr;

         Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
         stiffness.setZero();
         stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);


         stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
         damping.setZero();
         damping.topLeftCorner(3, 3) << damp_xy * Eigen::MatrixXd::Identity(3, 3);


         damping.bottomRightCorner(3, 3) << damp_r * Eigen::MatrixXd::Identity(3, 3);



            //setDefaultBehavior(robot);
            // load the kinematics and dynamics model
            franka::Model model = robot.loadModel();

            franka::RobotState initial_state = robot.readOnce();

            // equilibrium point is the initial position
            Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
            Eigen::Vector3d position_d(initial_transform.translation());
            Eigen::Quaterniond orientation_d(initial_transform.linear());

 //            set collision behavior
            robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                       {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                       {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                       {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

//            robot.setFilters(50,50,50,50,50);

            // define callback for the torque control loop

            //-----------------------------------------------
            // Set initial values
            //-----------------------------------------------
            R_old=R0;
            T_old=T0;
            p_old=p0;

            double time=0.0;
            double time_tot= 1.0;

            Phase phase;
            phase=ONE;
            safe=false;

            GLOBAL_TIME=0.0;



            while (!START_FLAG) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
#ifdef DEBUG
                std::cout<<"wait for camera \n";
                std::cout<<START_FLAG<<"\n";
#endif
            }
#ifdef DEBUG
            std::cout<<"START FLAG = true \n";
            std::cout<<"Start Panda control Loop \n";
#endif

            emit started();

            //debug
            unsigned int count=0;

            //set true the flags when the robot is on subject's head
            onHEAD=true;
            TRACKING=true;

            //--------------------------------------------
            // Start the timer for the rearrangement
            //--------------------------------------------
            RearrangeElapsTime.start();


            Eigen::Matrix4d T_bR_head, T_bR_target;

            //--------------------------------------------
            // Butterworth low-pass filter for dq at 1 kHz
            //--------------------------------------------
            //cutoff= 10 Hz (0.02)
            double dq_old[7];
            double dq_filt_old[7];
            double b_=0.0305;
            double a_=-0.9391;

            //1 order, cutoff=5 Hz (0.01)
//            double b_=0.0155;
//            double a_=-0.9691;


            for(int i=0;i<7;i++){
                dq_old[i]=0.0;
                dq_filt_old[i]=0.0;
            }
            //---------------------------------------------



            std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
                impedance_control_callback = [ this, &count,&b_, &a_,&dq_old,&dq_filt_old, &time, &time_tot, &S, &stiffness, &damping,
                    &position_d,  &model,&orientation_d, &T_bR_head, &T_bR_target](const franka::RobotState& robot_state,
                                                 franka::Duration period /*duration*/) -> franka::Torques {

                bool DONE=false;

                //-----------------------------------------------
                //Print Thread
                //-----------------------------------------------
                GLOBAL_TIME += period.toSec();
                // Update data to print.
                if (print_data.mutex.try_lock()) {
                    for(int i=0;i<16;i++){
//                        print_data.T_robot(i)=robot_state.O_T_EE_d[i];
                        print_data.global_time=GLOBAL_TIME;
                    }

                  print_data.has_data = true;
                  print_data.mutex.unlock();
                }


                //-----------------------------------------------
                //Interaction Force and Torque
                //-----------------------------------------------
                //update dispaly info each 50 ms
                if(count%sampling_count_gui==0){

                    if(ForceMutex.try_lock()){

                        for(int i=0;i<6;i++)
                            FT_interaction[i]=robot_state.K_F_ext_hat_K[i];

                        ForceMutex.unlock();

                        emit newForceValue();
                    }
                }


               //-----------------------------------------------
               //EMERGENCY STOP: if the head is moving too fast
               //-----------------------------------------------
               if(TargetMutex.try_lock()){
                   double px,py,pz;
                   Eigen::Matrix3d R_help_old,R_help,R_;
                   Eigen::Vector3d t_;
                   double mov_th;

                   px= fabs(T_cam_head_old(0,3)-T_cam_head(0,3));
                   py= fabs(T_cam_head_old(1,3)-T_cam_head(1,3));
                   pz= fabs(T_cam_head_old(2,3)-T_cam_head(2,3));
                   for(int i=0;i<3;i++){
                       for(int j=0;j<3;j++){
                           R_help_old(i,j)=T_cam_head_old(i,j);
                           R_help(i,j)=T_cam_head(i,j);
                       }
                   }

                   TargetMutex.unlock();

                   R_=R_help_old.transpose()*R_help;

                   LogSO3(R_,t_,mov_th);

                   //check if the head is moving too fast
                   if(px >= pos_error || py >= pos_error || pz >= pos_error || fabs(mov_th)>=ori_error ){

                       qDebug()<<"== EMERGENCY STOP! ==";
                       emit EmergencySTOP();

                   }
               }



                //-----------------------------------------------
                // Get state variables
                //-----------------------------------------------
              std::array<double, 7> coriolis_array = model.coriolis(robot_state);
              std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

              // convert to Eigen
              Eigen::Map<const Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
              Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
              Eigen::Map<const Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
              Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
              Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
              Eigen::Vector3d position(transform.translation());
              Eigen::Quaterniond orientation(transform.linear());


              //-----------------------------------------------
              //Define Target Values: Desired Pose
              //-----------------------------------------------
              if(time==0.0){
                  while(!DONE){
                      if(TargetMutex.try_lock()){

                          T_bR_head=T_bR_cam*T_cam_head_filt;
                          TargetMutex.unlock();

                          if(!safe){
                              for(int i=0;i<16;i++)
                                Ti(i)=robot_state.O_T_EE[i];
                              T_head_target=TargetControl[0];    //select deepest point for better control
                              time_tot=time_first_interpolation;
                          }
                          else{
                              //modifica smooth ale
                              for(int i=0;i<16;i++)
                                Ti(i)=robot_state.O_T_EE[i];

                              T_head_target=TargetControl[0];     //select stimulation point
                              time_tot=time_interpolation;
                          }

                          T_bR_target=T_bR_head*T_head_target;

                          Tf=T_bR_target;

                          DONE=true;
                      }
                  }

                //modifica smooth ale
//                  if(!safe){
                      //extract position and rotation
                      for(int i=0;i<3;i++){
                          pi(i)=Ti(i,3);
                          pf(i)=Tf(i,3);
                          for(int j=0;j<3;j++){
                              Ri(i,j)=Ti(i,j);
                              Rf(i,j)=Tf(i,j);
                          }
                      }
                      // Interpolate the Orientation
                      // Compute rotation from initial to final pose
                      Eigen::Matrix3d dR;
                      dR=Ri.transpose()*Rf;

                      //Log SO3
                      Eigen::Vector3d w_t_;
                      LogSO3(dR, w_t_, theta_t);
                      w_t=w_t_;

                //modifica smooth ale
//                }

                  //update Desired values
                  for(int i=0;i<3;i++){
                      position_d(i)=Tf(i,3);
                      for(int j=0;j<3;j++){
                          Rf(i,j)=Tf(i,j);
                      }
                  }
                  orientation_d=Rf;
               }


              //-----------------------------------------------
              //Robot Control: Compute Trajectories
              //-----------------------------------------------
              time += period.toSec();

              if(!safe){

                  //update final pose
                  //-------------------
                  if(TargetMutex.try_lock()){

                      T_bR_head=T_bR_cam*T_cam_head_filt;
                      TargetMutex.unlock();
                    }

                      T_bR_target=T_bR_head*T_head_target;

                      Tf=T_bR_target;

                  //extract position and rotation
                  for(int i=0;i<3;i++){
                      pf(i)=Tf(i,3);
                      for(int j=0;j<3;j++){
                          Rf(i,j)=Tf(i,j);
                      }
                  }
                  // Interpolate the Orientation
                  // Compute rotation from initial to final pose
                  Eigen::Matrix3d dR;
                  dR=Ri.transpose()*Rf;

                  //Log SO3
                  Eigen::Vector3d w_t_;
                  LogSO3(dR, w_t_, theta_t);
                  w_t=w_t_;


              //update Desired values
              for(int i=0;i<3;i++){
                  position_d(i)=Tf(i,3);
                  for(int j=0;j<3;j++){
                      Rf(i,j)=Tf(i,j);
                  }
              }
              orientation_d=Rf;


                  //----------------------

                  double c;
                  if(time<time_tot){
                      c=time/time_tot;

                      double xc=FifthOrderPoly(pi(0),pf(0),c);
                      double yc=FifthOrderPoly(pi(1),pf(1),c);
                      double zc=FifthOrderPoly(pi(2),pf(2),c);
                      double qc=FifthOrderPoly(0.0,theta_t,c);

                      Eigen::Matrix3d Rc;
                      Rc=Ri* ExpSO3(qc*w_t);


                      //update Desired values
                      position_d(0)=xc;
                      position_d(1)=yc;
                      position_d(2)=zc;
                      orientation_d=Rc;

                 }
              }



              // compute error to desired equilibrium pose
              // position error
              Eigen::Matrix<double, 6, 1> error;
              error.head(3) << position - position_d;

              // orientation error
              // "difference" quaternion
              if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
                orientation.coeffs() << -orientation.coeffs();
              }
              Eigen::Quaterniond error_quaternion(orientation * orientation_d.inverse());
              // convert to axis angle
              Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
              // compute "orientation error"
              error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

              // compute control
              Eigen::VectorXd tau_task(7), tau_d(7);

            //-------------------------------------------
            // Butterworth low-pass filter for dq
            //-------------------------------------------
            Eigen::Matrix<double, 7, 1> dq_filt;
            for(int i=0;i<7;i++){
            dq_filt(i)=b_*dq(i) + b_*dq_old[i] - a_*dq_filt_old[i];
            }
            for(int i=0;i<7;i++){
            dq_filt_old[i]=dq_filt(i);
            dq_old[i]=dq(i);
            }



            tau_task << jacobian.transpose() * ( -stiffness * error - damping * (jacobian * dq));

//           tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq_filt));

              tau_d << tau_task + coriolis;

              std::array<double, 7> tau_d_array{};
              Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

               // if(count<=5)
//              qDebug()<<"errore:"<<error(0)<<error(1)<<error(2)<<error(3)<<error(4)<<error(5);

              if(time>=time_tot){
                  if(!safe){
                      safe=true;
                      time=0.0;

                  }
                  else{
                      time=0.0;
                  }
              }



              //-------------------------------------------
              //Print Log Data
              //-------------------------------------------

              // 0 robot dt
              // 1 GLOBAL TIME
              // 2:8  q robot
              // 9:15 q_dot robot
              // 16:22 tau_J
              // 23:38 T coil
              // 39:54 T head
              // 55:70 T target
              // 71:76 Error
              // 77:82 F coil

              if(count%sampling_count==0)
              {  //downsampling to 200 Hz


                QVector<double> printdata(77);
                printdata[0]=period.toSec();
                printdata[1]=ElapsTime.elapsed();  //elapsed time [ms]
                //                printdata[1]=GLOBAL_TIME;

//                for(int i=0;i<7;i++){
//                    printdata[2+i]=q(i);
//                    printdata[9+i]=dq(i);
//                    printdata[16+i]=dq_filt(i);
//                }

                for(int i=0;i<16;i++){
                    printdata[2+i]=robot_state.O_T_EE[i];
                }
                printdata[18]= print_data.camera_time;

                if(TargetMutex.try_lock()){
                    for(int i=0;i<16;i++){
                        printdata[19+i]=T_cam_coil(i);
                        printdata[35+i]=T_cam_head_filt(i);
                        printdata[51+i]=Target[0](i);
                    }
                    TargetMutex.unlock();
                }
                if(ErrorMutex.try_lock()){
                    printdata[67]=PositionError[0];
                    printdata[68]=PositionError[1];
                    printdata[69]=PositionError[2];
                    printdata[70]=OrientationError;
                ErrorMutex.unlock();
                }

                for(int i=0;i<6;i++){
                    printdata[71+i]=robot_state.K_F_ext_hat_K[i];
                }

                emit printData(printdata);

            }

              count++;

              //debug
//              qDebug()<<"P error:"<<position_d[0]-robot_state.O_T_EE[12]<<position_d[1]-robot_state.O_T_EE[13]<<position_d[2]-robot_state.O_T_EE[14];


//              return limitRate(kMaxTorqueRate, tau_d_array, robot_state.tau_J_d);
              return tau_d_array;
            };


            robot.control(impedance_control_callback);

          } catch (const franka::Exception& ex) {
            // print exception
            std::cout << ex.what() << std::endl;
          }


        emit finished();

        //set the tracking flag false
        TRACKING=false;


}


void PandaClass::onReArrangeRobot(){


#ifdef DEBUG
    qDebug()<<"rearrange";
#endif

    try{

        #ifdef DEBUG
            //debug
            std::cout<<"panda run \n";
        #endif

        //-----------------------------------------------
        //Setting
        //-----------------------------------------------

         // Compliance parameters
         const double translational_stiffness=kxy;

         const double rotational_stiffness=kr;

         Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
         stiffness.setZero();
         stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);

         stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
         damping.setZero();
         damping.topLeftCorner(3, 3) << damp_xy * Eigen::MatrixXd::Identity(3, 3);

         damping.bottomRightCorner(3, 3) << damp_r * Eigen::MatrixXd::Identity(3, 3);


             //-----------------------------------------------
             //Load Kinematic and Dynamic Model
             //-----------------------------------------------

            //setDefaultBehavior(robot);
            // load the kinematics and dynamics model
            franka::Model model = robot.loadModel();

            franka::RobotState initial_state = robot.readOnce();

            // equilibrium point is the initial position
            Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
            Eigen::Vector3d position_d(initial_transform.translation());
            Eigen::Quaterniond orientation_d(initial_transform.linear());

            // set collision behavior
            robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                       {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                       {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                       {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

            // define callback for the torque control loop

            //-----------------------------------------------
            // Set initial values
            //-----------------------------------------------
            R_old=R0;
            T_old=T0;
            p_old=p0;


            double time=0.0;
            double time_tot= time_arrangement;


            Phase phase;
            phase=ONE;
            safe=false;

            GLOBAL_TIME=0.0;


            emit started();

            //debug
             unsigned int count=0;

             TRACKING=true;

             //-----------------------------------------------
             // Define new target
             //-----------------------------------------------
             Eigen::Matrix4d T_EE_delta, TargetArrange;
             Eigen::Matrix4d T_EE_delta_target;
              for(int i=0;i<4;i++){
                   for(int j=0;j<4;j++){
                     if(i==j){
                         T_EE_delta(i,j)=1;
                         T_EE_delta_target(i,j)=1;
                     }
                     else{
                         T_EE_delta(i,j)=0;
                         T_EE_delta_target(i,j)=0;
                         }
                     }
               }
               T_EE_delta(2,3)=-DeltaArrangement;

             //Define Safe Points (Target point + Safety Delta (along the axis normal to the head (i.e. EE z axis)))
             TargetArrange=Target[0]*T_EE_delta;


             Eigen::Matrix4d T_bR_head, T_bR_target;


            //-----------------------------------------------
            //Control Loop
            //-----------------------------------------------
            std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
                            impedance_control_callback = [ &count,&TargetArrange, &time, &time_tot, &stiffness, &model, &damping, &position_d, &orientation_d, &T_bR_head, &T_bR_target, this, &S](const franka::RobotState& robot_state,
                                                             franka::Duration period /*duration*/) -> franka::Torques {


            bool DONE=false;

            count++;
            //-----------------------------------------------
            //Print Thread
            //-----------------------------------------------
            GLOBAL_TIME += period.toSec();
            // Update data to print.
            if (print_data.mutex.try_lock()) {
                for(int i=0;i<16;i++){
                    print_data.T_robot(i)=robot_state.O_T_EE_d[i];
                    print_data.global_time=GLOBAL_TIME;
                }

              print_data.has_data = true;
              print_data.mutex.unlock();
            }

            //-----------------------------------------------
            //SECONDARY EMERGENCY STOP: if the head is moving too fast
            //-----------------------------------------------
            if(TargetMutex.try_lock()){
                double px,py,pz;
                Eigen::Matrix3d R_help_old,R_help,R_;
                Eigen::Vector3d t_;
                double mov_th;

                px= fabs(T_cam_head_old(0,3)-T_cam_head(0,3));
                py= fabs(T_cam_head_old(1,3)-T_cam_head(1,3));
                pz= fabs(T_cam_head_old(2,3)-T_cam_head(2,3));
                for(int i=0;i<3;i++){
                    for(int j=0;j<3;j++){
                        R_help_old(i,j)=T_cam_head_old(i,j);
                        R_help(i,j)=T_cam_head(i,j);
                    }
                }

                TargetMutex.unlock();

                R_=R_help_old.transpose()*R_help;

                LogSO3(R_,t_,mov_th);

                //check if the head is moving too fast
                if(px >= pos_error_secondary || py >= pos_error_secondary || pz >= pos_error_secondary || fabs(mov_th)>=ori_error_secondary ){

                    qDebug()<<"== SECONDARY EMERGENCY STOP! ==";
                    emit EmergencySTOP();

                }
            }


            //-----------------------------------------------
            // Get state variables
            //-----------------------------------------------
          std::array<double, 7> coriolis_array = model.coriolis(robot_state);
          std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

          // convert to Eigen
          Eigen::Map<const Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
          Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
          Eigen::Map<const Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
          Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
          Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
          Eigen::Vector3d position(transform.translation());
          Eigen::Quaterniond orientation(transform.linear());


            //-----------------------------------------------
            //Robot Control: Compute Trajectories
            //-----------------------------------------------

            if(time==0.0)
            {
                while(!DONE){
                  if(TargetMutex.try_lock()){

                      T_bR_head=T_bR_cam*T_cam_head_filt;
                      TargetMutex.unlock();

                      if(!safe){
                          for(int i=0;i<16;i++)
                              Ti(i)=robot_state.O_T_EE[i];
                          T_head_target=TargetArrange;    //select safe point
                          time_tot=time_arrangement;
                          //set false the flag when the robot is NOT on subject's head
                          onHEAD=false;
                      }
                      else{
                          for(int i=0;i<16;i++)
                              Ti(i)=robot_state.O_T_EE[i];
                          T_head_target=TargetControl[0];     //select stimulation point (deepest one)
                          time_tot=time_arrangement;
                      }

                      T_bR_target=T_bR_head*T_head_target;

                      Tf=T_bR_target;

                      DONE=true;
                  }
                }


#ifdef DEBUG
                    qDebug()<<"Initial Pose:";
                    for(int i=0;i<16;i++)
                        qDebug()<<Ti(i);
                    qDebug()<<"Target base referred:";
                    for(int i=0;i<16;i++)
                        qDebug()<<Tf(i);
                     qDebug()<<"== Motion Started ==\n";
#endif


                //extract position and rotation
                for(int i=0;i<3;i++){
                    pi(i)=Ti(i,3);
                    pf(i)=Tf(i,3);
                    for(int j=0;j<3;j++){
                        Ri(i,j)=Ti(i,j);
                        Rf(i,j)=Tf(i,j);
                    }
                }
                // Interpolate the Orientation
                // Compute rotation from initial to final pose
                Eigen::Matrix3d dR;
                dR=Ri.transpose()*Rf;

                //Log SO3
                Eigen::Vector3d w_t_;
                LogSO3(dR, w_t_, theta_t);
                w_t=w_t_;


                //update Desired values
                for(int i=0;i<3;i++){
                    position_d(i)=Tf(i,3);
                    for(int j=0;j<3;j++){
                        Rf(i,j)=Tf(i,j);
                    }
                }
                orientation_d=Rf;
             }

                //Min jerk first iteration:
                //-----------------------------------------------
                //Robot Control: Compute Trajectories
                //-----------------------------------------------
                time += period.toSec();

                //update final pose
              //-------------------
              if((count%5)==0){
              if(TargetMutex.try_lock()){

                  T_bR_head=T_bR_cam*T_cam_head_filt;
                  TargetMutex.unlock();
                }

                  T_bR_target=T_bR_head*T_head_target;

                  Tf=T_bR_target;

              //extract position and rotation
              for(int i=0;i<3;i++){
                  pf(i)=Tf(i,3);
                  for(int j=0;j<3;j++){
                      Rf(i,j)=Tf(i,j);
                  }
              }
              // Interpolate the Orientation
              // Compute rotation from initial to final pose
              Eigen::Matrix3d dR;
              dR=Ri.transpose()*Rf;

              //Log SO3
              Eigen::Vector3d w_t_;
              LogSO3(dR, w_t_, theta_t);
              w_t=w_t_;


              //update Desired values
              for(int i=0;i<3;i++){
                  position_d(i)=Tf(i,3);
                  for(int j=0;j<3;j++){
                      Rf(i,j)=Tf(i,j);
                  }
              }
              orientation_d=Rf;
               }

              //----------------------



                double c;
                if(time<time_tot){
                    c=time/time_tot;

                    double xc=FifthOrderPoly(pi(0),pf(0),c);
                    double yc=FifthOrderPoly(pi(1),pf(1),c);
                    double zc=FifthOrderPoly(pi(2),pf(2),c);
                    double qc=FifthOrderPoly(0.0,theta_t,c);

                    Eigen::Matrix3d Rc;
                    Rc=Ri* ExpSO3(qc*w_t);


                    //update Desired values
                    position_d(0)=xc;
                    position_d(1)=yc;
                    position_d(2)=zc;
                    orientation_d=Rc;

               }

                // compute error to desired equilibrium pose
                // position error
                Eigen::Matrix<double, 6, 1> error;
                error.head(3) << position - position_d;

                // orientation error
                // "difference" quaternion
                if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
                  orientation.coeffs() << -orientation.coeffs();
                }
                Eigen::Quaterniond error_quaternion(orientation * orientation_d.inverse());
                // convert to axis angle
                Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
                // compute "orientation error"
                error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

                // compute control
                Eigen::VectorXd tau_task(7), tau_d(7);



              //-------------------------------------------
              //Print Log Data
              //-------------------------------------------

              // 0 robot dt
              // 1 GLOBAL TIME
              // 2:8  q robot
              // 9:15 q_dot robot
              // 16:22 tau_J
              // 23:38 T coil
              // 39:54 T head
              // 55:70 T target
              // 71:76 Error
              // 77:82 F coil

              if(count%sampling_count==0)
              {  //downsampling to 200 Hz


                QVector<double> printdata(77);
                printdata[0]=period.toSec();
                printdata[1]=ElapsTime.elapsed();  //elapsed time [s]
                //                printdata[1]=GLOBAL_TIME;

//                for(int i=0;i<7;i++){
//                    printdata[2+i]=q(i);
//                    printdata[9+i]=dq(i);
//                    printdata[16+i]=dq_filt(i);
//                }

                for(int i=0;i<16;i++){
                    printdata[2+i]=robot_state.O_T_EE[i];
                }
                printdata[18]= print_data.camera_time;

                if(TargetMutex.try_lock()){
                    for(int i=0;i<16;i++){
                        printdata[19+i]=T_cam_coil(i);
                        printdata[35+i]=T_cam_head_filt(i);
                        printdata[51+i]=Target[0](i);
                    }
                    TargetMutex.unlock();
                }
                if(ErrorMutex.try_lock()){
                    printdata[67]=PositionError[0];
                    printdata[68]=PositionError[1];
                    printdata[69]=PositionError[2];
                    printdata[70]=OrientationError;
                ErrorMutex.unlock();
                }

                for(int i=0;i<6;i++){
                    printdata[71+i]=robot_state.K_F_ext_hat_K[i];
                }

                emit printData(printdata);

            }




                tau_task << jacobian.transpose() * ( -stiffness * error - damping * (jacobian * dq));

                //           tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq_filt));

                tau_d << tau_task + coriolis;

                std::array<double, 7> tau_d_array{};
                Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

                  //---------------------

           if(time>=time_tot){
                if(!safe){
#ifdef DEBUG
                    qDebug()<<"Safe point Reached";
#endif
                    safe=true;
                    time=0.0;

                }
                else{
#ifdef DEBUG
                    qDebug()<<"Target point Reached";
#endif
                    time=0.0;
                    franka::Torques torques_ctrl={tau_d_array[0],tau_d_array[1],tau_d_array[2],tau_d_array[3],tau_d_array[4],tau_d_array[5],tau_d_array[6]};
                    return franka::MotionFinished( torques_ctrl);
                }
            }


//            return limitRate(kMaxTorqueRate, tau_d_array, robot_state.tau_J_d);
           return tau_d_array;
       };


       robot.control(impedance_control_callback);


      } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;

      }

//    emit finished();

    //-----------------------------------------------
    //Call the Head Motion compensation function
    //-----------------------------------------------
    //check if the robot is in the target pose and has been not stopped!
    if(!STOP){
        emit HeadCompensation();
        STOP=false;
    }


}


void PandaClass::goResting(){
    try{

    #ifdef DEBUG
            //debug
            std::cout<<"panda run \n";
    #endif
            //-----------------------------------------------
            //Read where is the robot
            //-----------------------------------------------
            //read initial pose
            std::array<double,16> initial_pose=robot.readOnce().O_T_EE_d;

            //extract initial homogeneus matrix
            for(int i=0;i<16;i++){
                T0(i)=initial_pose[i];
            }

        #ifdef DEBUG
            //Debug
            std::cout<<"intial homogeneus matrix \n";
            std::cout<< T0<<"\n";
        #endif

            //extract initial position
            for(int i=0;i<3;i++){
                p0(i)=T0(i,3);
            }

        #ifdef DEBUG
            //Debug
            std::cout<<"intial position vector \n";
            std::cout<< p0<<"\n";
        #endif

            //extract initial rotation
            for(int i=0;i<3;i++){
                for(int j=0;j<3;j++){
                    R0(i,j)=T0(i,j);
                }
            }

        #ifdef DEBUG
            //Debug
            std::cout<<"intial Rotation matrix \n";
            std::cout<< R0<<"\n";
        #endif


            //-----------------------------------------------
            // Set Stiffness Frame
            //-----------------------------------------------
            std::array<double,16> T_EE_K;
            for(int i=0;i<16;i++){
                T_EE_K[i]=T_EE_coil(i);
            }

            robot.setK(T_EE_K);

            //-----------------------------------------------
            // Set Impedance
            //-----------------------------------------------
            std::array<double,6> K;
            K[0]=kxy;
            K[1]=kxy;
            K[2]=kxy;
            K[3]=kr;
            K[4]=kr;
            K[5]=kr;

            robot.setCartesianImpedance(K);


            //-----------------------------------------------
            // Set initial values
            //-----------------------------------------------
            R_old=R0;
            T_old=T0;
            p_old=p0;

            double time=0.0;
            double time_tot= time_far;

            Phase phase;
            phase=ONE;
            safe=false;

            GLOBAL_TIME=0.0;

            while (!START_FLAG) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
#ifdef DEBUG
                std::cout<<"wait for camera \n";
                std::cout<<START_FLAG<<"\n";
#endif
            }
#ifdef DEBUG
            std::cout<<"START FLAG = true \n";
            std::cout<<"Start Panda control Loop \n";
#endif

            emit started();

            //-----------------------------------------------
            //Control Loop
            //-----------------------------------------------
            robot.control([=, &time, &time_tot , &phase, this](const franka::RobotState& state,
                                 franka::Duration period) -> franka::CartesianVelocities {


            //-----------------------------------------------
            //Print Thread
            //-----------------------------------------------
            GLOBAL_TIME += period.toSec();
            // Update data to print.
            if (print_data.mutex.try_lock()) {
                for(int i=0;i<16;i++){
                    print_data.T_robot(i)=state.O_T_EE[i];
                    print_data.global_time=GLOBAL_TIME;
                }

              print_data.has_data = true;
              print_data.mutex.unlock();
            }


            //-----------------------------------------------
            //Robot Control: Compute Trajectories
            //-----------------------------------------------
            time += period.toSec();

            //Control Variables: velocities and time
            double c;
            Eigen::Vector3d vel, omega;
            Eigen::Matrix4d T_bR_head, T_bR_target;

//            //Check if there are still points
//            if(Point>= N)
//                phase=END;  // ? controllare le condizioni di start e stop nel caso della gui


            switch (phase) {
            case ONE:
            {
//                First loop iteration
                time=-period.toSec();

                if(TargetMutex.try_lock()){
//                    T_cam_head_old=T_cam_head;

                    T_bR_head=T_bR_cam*T_cam_head;
                 TargetMutex.unlock();
                }


                    //--------------------------------------------
                    //Check if the robot is on the Head: 2 trajectories= Safe + Rest
                    //else go directly to the Rest pose
                    //----------------------------------------------
                    if(onHEAD){
                        if(!safe){
                            Ti=T0;
                            T_head_target=SafeTarget[0];    //select safe point
                            T_bR_target=T_bR_head*T_head_target;
                            time_tot=time_far;
                        }
                        else{
                            for(int i=0;i<16;i++)
                                Ti(i)=state.O_T_EE[i];
                            T_bR_target=T_rest;     //select stimulation point
                            time_tot=time_first_point;

                            onHEAD=false;
                        }
                    }
                    else{
                        for(int i=0;i<16;i++)
                            Ti(i)=state.O_T_EE[i];
                        T_bR_target=T_rest;     //select stimulation point
                        time_tot=time_first_point;
                    }


                    Tf=T_bR_target;

#ifdef DEBUG
                    qDebug()<<"Initial Pose:";
                    for(int i=0;i<16;i++)
                        qDebug()<<Ti(i);
                    qDebug()<<"Target base referred:";
                    for(int i=0;i<16;i++)
                        qDebug()<<Tf(i);
                     qDebug()<<"== Motion Started ==\n";
#endif


                //extract position and rotation
                for(int i=0;i<3;i++){
                    pi(i)=Ti(i,3);
                    pf(i)=Tf(i,3);
                    for(int j=0;j<3;j++){
                        Ri(i,j)=Ti(i,j);
                        Rf(i,j)=Tf(i,j);
                    }
                }
                p_old=pi;
                R_old=Ri;
                qc_old=0.0;

                // Interpolate the Orientation
                // Compute rotation from initial to final pose
                Eigen::Matrix3d dR;
                dR=Ri.transpose()*Rf;

                //Log SO3
                Eigen::Vector3d w_t_;
                LogSO3(dR, w_t_, theta_t);
                w_t=Ri*w_t_;

                //set initial velocities null
                for(int i=0;i<3;i++){
                    vel(i)=0.0;
                    omega(i)=0.0;
                }
                phase=MINJERK;
            }
                break;

            case REST:
            {
                for(int i=0;i<3;i++){
                    vel(i)=0.0;
                    omega(i)=0.0;
                }
                if(time>=time_rest){
                     #ifdef DEBUG
                    //Debug
                    std::cout<<"Rest time finished \n";
                    #endif
                    std::cout<<"Pause finished \n\n";
                    phase=END;
                    time=0.0;
                }
            }
                break;

            case MINJERK:
            {

                if(time<0.0)
                    time=0.0;

                if(time<time_tot){
                    c=time/time_tot;

                    double xc=FifthOrderPoly(pi(0),pf(0),c);
                    vel(0)=(xc-p_old(0))/period.toSec();
                    p_old(0)=xc;

                    double yc=FifthOrderPoly(pi(1),pf(1),c);
                    vel(1)=(yc-p_old(1))/period.toSec();
                    p_old(1)=yc;

                    double zc=FifthOrderPoly(pi(2),pf(2),c);
                    vel(2)=(zc-p_old(2))/period.toSec();
                    p_old(2)=zc;

                    double qc=FifthOrderPoly(0.0,theta_t,c);
                    omega(0)=(qc-qc_old)/period.toSec()*w_t[0];
                    omega(1)=(qc-qc_old)/period.toSec()*w_t[1];
                    omega(2)=(qc-qc_old)/period.toSec()*w_t[2];
                    qc_old=qc;

                }

                else{

                    if(!safe){
                        safe=true;
                        phase=ONE;
                        time=0.0;
                        for(int i=0;i<3;i++){
                            vel(i)=0.0;
                            omega(i)=0.0;
                        }
#ifdef DEBUG
                    qDebug()<<"Safe Point reached\n";
#endif
                    }
                    else{

                        phase=REST;
                        time=0.0;

                        for(int i=0;i<3;i++){
                            vel(i)=0.0;
                            omega(i)=0.0;
                        }
                    }
                }

            }
                break;

            case END:
            {
                //set final velocities null
                for(int i=0;i<3;i++){
                    vel(i)=0.0;
                    omega(i)=0.0;
                }
                franka::CartesianVelocities output = {{vel(0),vel(1),vel(2),omega(0),omega(1),omega(2)}};
                return franka::MotionFinished(output);
             }
                break;

            }


            franka::CartesianVelocities output = {{vel(0),vel(1),vel(2),omega(0),omega(1),omega(2)}};

            return output;

          }, franka::ControllerMode::kCartesianImpedance);

      } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;

      }

    emit finished();

}


void PandaClass::onEmergencySTOP(){

    //-----------------------------------------------
    //STOP the robot
    //-----------------------------------------------
    robot.stop();

    //set the onHEAD to false;
    TRACKING=false;
    // ? deve però spostarsi!!
    onHEAD=false;


}


void PandaClass::run(){

}

void PandaClass::goHeadCompensation_0(){

}

void PandaClass::goHeadCompensation_1(){

}



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

     qRegisterMetaType<QVector<double> >();

    ui->setupUi(this);

    //-----------------------------------------------
    //Set initial window state
    //-----------------------------------------------
    ui->groupBoxCalibration->setEnabled(true);
    ui->groupBoxConnection->setEnabled(false);
    ui->groupBoxTMS->setEnabled(false);
    ui->labelCameraConnected->setEnabled(false);
    ui->labelRobotConnected->setEnabled(false);
    ui->labelRobotError->setEnabled(false);
    ui->groupBoxError->setEnabled(false);
    ui->groupBoxForce->setEnabled(false);
    ui->groupBoxAcceptableError->setEnabled(false);
    ui->groupBoxStimulationProtocol->setEnabled(true);
    ui->groupBoxAutomaticStimulation->setEnabled(false);
    ui->groupBoxManualStimulation->setEnabled(false);

    //-----------------------------------------------
    //Initialize Serial port Device List
    //-----------------------------------------------
    onSerialDeviceSelection();
    ui->pushButtonDisconnectSerialPort->setEnabled(false);


    //-----------------------------------------------
    //Variables Definition
    //-----------------------------------------------
    timer = new QTimer(0);
    timer->setInterval(1000);
    pandathread = new QThread;

    panda.moveToThread(pandathread);




    //------------------------------------
    //Create UDP class
    //------------------------------------
    // UDPsocket = new QUdpSocket();
    QThread *udpt=new QThread();
//    MyUDP UDPclass;
    //UDPclass.moveToThread(udpt);



//    QObject::connect(  UDPclass, &MyUDP::ReceivedData, UpdateCameraData );

 //  QObject::connect(&udpt,SIGNAL(quit()),UDPclass, SLOT(UDPClass.MyUDPclosing()));



    //-----------------------------------------------
    //Set SIGNAL-SLOT connection
    //-----------------------------------------------
//    QObject::connect(timer,SIGNAL(timeout()),&testremote,SLOT(run()));
    QObject::connect(pandathread,SIGNAL(started()),&panda,SLOT(initialize()));
//    QObject::connect(&testremote,SIGNAL(started()),this,SLOT(onCameraConnected()));
//    QObject::connect(&testremote,SIGNAL(finished()),this,SLOT(onCameraDisconnected()));
    QObject::connect(&panda,SIGNAL(connected()),this,SLOT(onRobotConnected()));
    QObject::connect(&panda,SIGNAL(started()),this,SLOT(onRobotStarted()));
    QObject::connect(&panda,SIGNAL(stopped()),this,SLOT(onRobotStopped()));
    QObject::connect(&panda,SIGNAL(error()),this,SLOT(onRobotError()));
    QObject::connect(&panda,SIGNAL(finished()),this,SLOT(onRobotFinished()));

    //provo a muovere sul punto premendo il bottone
//    QObject::connect(ui->pushButtonPoint1,SIGNAL(clicked(bool)),&panda,SLOT(goTracking()));

    QObject::connect(this,SIGNAL(goTracking()),&panda,SLOT(goTracking()));
    QObject::connect(this,SIGNAL(goResting()),&panda,SLOT(goResting()));
    QObject::connect(&panda,SIGNAL(HeadCompensation()),this,SLOT(onHeadCompensationStarted()));
    QObject::connect(&panda,SIGNAL(HeadCompensation()),&panda,SLOT(goHeadCompensation()));
    QObject::connect(&panda,SIGNAL(HeadCompensation_RE()),&panda,SLOT(goHeadCompensation()));
    QObject::connect(&panda,SIGNAL(HeadCompensation_RE()),this,SLOT(onHeadCompensationStarted_RE()));

    QObject::connect(this,SIGNAL(goReArrangeRobot()),&panda,SLOT(onReArrangeRobot()));

    QObject::connect(&panda,SIGNAL(EmergencySTOP()),this,SLOT(onSTOPClicked()));
    QObject::connect(&panda,SIGNAL(EmergencySTOP()),this,SLOT(onEmergencyStop()));
    QObject::connect(&panda,SIGNAL(newForceValue()),this,SLOT(onNewForceValue()));

    QObject::connect(&panda,SIGNAL(printData(QVector<double>)),this, SLOT(onPrintData(QVector<double>)));

    QObject::connect(stimTimer,SIGNAL(timeout()),this,SLOT(SendTriggerTMS()));

    //-----------------------------------------------
    //Define Robot Resting Pose
    //-----------------------------------------------
    T_rest(0,0)=1.0 ;  T_rest(0,1)=0.0 ;  T_rest(0,2)=0.0 ;  T_rest(0,3)=0.4 ;
    T_rest(1,0)=0.0 ;  T_rest(1,1)=-1.0;  T_rest(1,2)=0.0 ;  T_rest(1,3)=-0.1;
    T_rest(2,0)=0.0 ;  T_rest(2,1)=0.0 ;  T_rest(2,2)=-1.0;  T_rest(2,3)=0.6 ;
    T_rest(3,0)=0.0 ;  T_rest(3,1)=0.0 ;  T_rest(3,2)=0.0 ;  T_rest(3,3)=1.0 ;


    //-----------------------------------------------
    //Set Forces and Error values null
    //-----------------------------------------------
    ui->progressBarForceX->setValue(0);
    ui->progressBarForceY->setValue(0);
    ui->progressBarForceZ->setValue(0);
    ui->progressBarTorqueX->setValue(0);
    ui->progressBarTorqueY->setValue(0);
    ui->progressBarTorqueZ->setValue(0);
    ui->progressBarErrorOrientation->setValue(0);
    ui->progressBarErrorPosition->setValue(0);

    //-----------------------------------------------
    //Inizialize Error Thresholds
    //-----------------------------------------------
    ErrorThreshold_position = ui->spinBoxPositionThreshold->value();
    ErrorThreshold_orientation= ui->spinBoxOrientationThreshold->value();
    ErrorThreshold_z=ui->doubleSpinBoxThresholdZ->value();

    //-----------------------------------------------
    //Initialize Targets array
    //-----------------------------------------------
    for(int i=1;i<NUM_TARGET;i++){
        Target[i](0)=NAN;
    }






}

MainWindow::~MainWindow()
{
    //testremote.end();

    //-----------------------------------------------
    //Close Serial Communication
    //-----------------------------------------------
    if (serial.isOpen())
        {
            serial.close();
        }

    delete ui;
}


void MainWindow::onLoadClicked(){
    //-----------------------------------------------
    //Read Calibration Data
    //-----------------------------------------------
    unsigned int N=0;	//Number of points
    double *Predef_T=new double[1000];

    if(calibrationFile.open(QIODevice::ReadOnly | QIODevice::Text)){
        QTextStream in_traj(& calibrationFile);
        int s=0;
        while ( !in_traj.atEnd() )
        {
           double a;
           in_traj >> a;
            Predef_T[s]=a;
            s++;
        }
        //Number of points
        N=s/16;
        calibrationFile.close();
     }
    //-----------------------------------------------
    //Store Calibration Matrices (wrt panda base frame)
    //-----------------------------------------------
    //First matrix (16 elements): T_EE_coil
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++)
            T_EE_coil(i,j)=Predef_T[i*4+j];
    }
    //Second matrix (16 elements): T_bR_cam
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++)
            T_bR_cam(i,j)=Predef_T[16+i*4+j];
    }

#ifdef DEBUG
    qDebug("\n T_EE_coil \n");
    for(int i=0;i<16;i++){
            qDebug()<< T_EE_coil(i);
        }
    qDebug("\n T_bR_cam \n");
    for(int i=0;i<16;i++){
            qDebug()<< T_bR_cam(i);
    }

#endif

    //-----------------------------------------------
    //Define Transformation matrix referred to EE frame, with identity rotation matrix
    //and DeltaSafe translation along EE z axis
    //-----------------------------------------------
    Eigen::Matrix4d T_EE_delta;
     for(int i=0;i<4;i++){
          for(int j=0;j<4;j++){
            if(i==j)
                T_EE_delta(i,j)=1;
            else
                T_EE_delta(i,j)=0;
          }
      }
      T_EE_delta(2,3)=-DeltaSafe;




      //-----------------------------------------------
      //Create data folder and initialize txt files
      //-----------------------------------------------

      //read the folder name from the gui (default value: "Data")
        QString path1= ui->lineEditFOLDERname->text();
        QString path="../"+path1+"/";

        //set file name in the selected path
        outputRobot.setFileName(path+"robot.txt");
        outputCamera.setFileName(path+"camera.txt");
        calibrationFile.setFileName(path+"calibration.txt");   //txt file with calibration matrices
        targetPoint.setFileName(path+"target.txt");            //txt file with target point (with respect to head referance frame)
        LogFile.setFileName(path+"LogFile.txt");
        errorRobot.setFileName(path+"errorByRobot.txt");       //txt file with pose error computed as delta robot pose
        errorCamera.setFileName(path+"errorByCamera.txt");     //txt file with pose error computed as delta coil pose read by camera
        interactionForces.setFileName(path+"forces.txt");
        cmdRobot.setFileName(path+"data.txt");
        stimulation.setFileName(path+"stimulation.txt");

        QDir dir;
        if(!dir.exists(path)){
            dir.mkpath(path);
        }
        //-----------------------------------------------
        //initialize output text file
        //-----------------------------------------------
        outputRobot.open(QIODevice::WriteOnly | QIODevice::Text);
        outputRobot.close();
        outputCamera.open(QIODevice::WriteOnly | QIODevice::Text);
        outputCamera.close();
        errorRobot.open(QIODevice::WriteOnly | QIODevice::Text);
        errorRobot.close();
        errorCamera.open(QIODevice::WriteOnly | QIODevice::Text);
        errorCamera.close();
        interactionForces.open(QIODevice::WriteOnly | QIODevice::Text);
        interactionForces.close();
        cmdRobot.open(QIODevice::WriteOnly | QIODevice::Text);
        cmdRobot.close();
        stimulation.open(QIODevice::WriteOnly | QIODevice::Text);
        stimulation.close();

        //-----------------------------------------------
        //write Log text file
        //-----------------------------------------------
        if( LogFile.open(QIODevice::WriteOnly | QIODevice::Text)){
          QTextStream out_t(& LogFile);


          out_t<<"Torque control \n";

          out_t<<"Kxy"<<"\t"<<kxy<<"\n";
          out_t<<"Kz"<<"\t"<<kz<<"\n";
          out_t<<"Kori"<<"\t"<<kr<<"\n";
          out_t<<"Dxy"<<"\t"<<damp_xy<<"\n";
          out_t<<"Dz"<<"\t"<<damp_z<<"\n";
          out_t<<"Dori"<<"\t"<<damp_r<<"\n";
          out_t<<"Delta target position"<<DeltaControlTarget<<"\n";

          out_t<<"t_interpol"<<"\t"<<time_interpolation<<"\n";

          LogFile.close();
        }



        //-----------------------------------------------
        //Start the code timer
        //-----------------------------------------------
        ElapsTime.start();


    //-----------------------------------------------
    //Make features accesible after calibration loading
    //-----------------------------------------------
    ui->groupBoxConnection->setEnabled(true);
    ui->groupBoxDATAfolder->setEnabled(false);
    ui->groupBoxCalibration->setEnabled(false);


}





void MainWindow::onConnectClicked(){

    //-----------------------------------------------
    //Start Camera and Robot Connection
    //-----------------------------------------------
    timer->start();
    pandathread->start(QThread::TimeCriticalPriority);

    ui->pushButtonConnect ->setEnabled(false);
    ui->pushButtonDisconnect ->setEnabled(true);



}


void MainWindow::onDisconnectClicked(){

    //-----------------------------------------------
    //Start Camera and Robot Connection
    //-----------------------------------------------
    //testremote.end();

    ui->pushButtonDisconnect ->setEnabled(false);
    ui->pushButtonConnect->setEnabled(true);

    ui->groupBoxTMS->setEnabled(false);
}


void MainWindow::onCameraConnected(){


    QPalette sample_palette;
    sample_palette.setColor(QPalette::Window, Qt::green);
    sample_palette.setColor(QPalette::WindowText, Qt::white);

    ui->labelCameraConnected->setAutoFillBackground(true);
    ui->labelCameraConnected->setPalette(sample_palette);


    //-----------------------------------------------
    //Make TMS features accesible after connections ON
    //-----------------------------------------------
    ui->groupBoxTMS->setEnabled(true);
    ui->pushButtonSTART->setEnabled(false);
    ui->pushButtonStop->setEnabled(false);
    ui->groupBoxAcceptableError->setEnabled(true);


}
void MainWindow::onCameraDisconnected(){

    QPalette sample_palette;
    sample_palette.setColor(QPalette::Window, Qt::gray);
    sample_palette.setColor(QPalette::WindowText, Qt::gray);

    ui->labelCameraConnected->setAutoFillBackground(false);
    ui->labelCameraConnected->setPalette(sample_palette);

    ui->labelCameraConnected->setEnabled(false);

}

void MainWindow::onRobotConnected(){
    QPalette sample_palette;
    sample_palette.setColor(QPalette::Window, Qt::yellow);
    sample_palette.setColor(QPalette::WindowText, Qt::white);

    ui->labelRobotConnected->setAutoFillBackground(true);
    ui->labelRobotConnected->setPalette(sample_palette);
}

void MainWindow::onRobotDisconnected(){

}

void MainWindow::onRobotStarted(){
    QPalette sample_palette;
    sample_palette.setColor(QPalette::Window, Qt::green);
    sample_palette.setColor(QPalette::WindowText, Qt::white);

    ui->labelRobotConnected->setAutoFillBackground(true);
    ui->labelRobotConnected->setPalette(sample_palette);


    //-----------------------------------------------
    //Turn off the EMERGENCY STOP label
    //-----------------------------------------------
    QPalette palette_gr;
    palette_gr.setColor(QPalette::WindowText, Qt::gray);

    ui->labelEmergencyStop->setAutoFillBackground(true);
    ui->labelEmergencyStop->setPalette(palette_gr);


    //-----------------------------------------------
    //Make START and REST unaccesible while robot is moving
    //-----------------------------------------------
    ui->pushButtonSTART->setEnabled(false);
    ui->pushButtonRest->setEnabled(false);
    ui->pushButtonStop->setEnabled(true);
    //-----------------------------------------------
    //Make ACQUIRE POINT unaccesible while robot is moving
    //-----------------------------------------------
    ui->pushButtonAcquirePoint->setEnabled(false);
    ui->pushButtonLoadPoint->setEnabled(false);
    ui->pushButtonSofTaxicPoint->setEnabled(false);
    ui->groupBoxP1P2P3->setEnabled(false);

}

void MainWindow::onRobotStopped(){
    QPalette sample_palette;
    sample_palette.setColor(QPalette::Window, Qt::red);
    sample_palette.setColor(QPalette::WindowText, Qt::white);

    ui->labelRobotConnected->setAutoFillBackground(true);
    ui->labelRobotConnected->setPalette(sample_palette);
}

void MainWindow::onRobotFinished(){

    QPalette sample_palette;
    sample_palette.setColor(QPalette::Window, Qt::yellow);
    sample_palette.setColor(QPalette::WindowText, Qt::white);

    ui->labelRobotConnected->setAutoFillBackground(true);
    ui->labelRobotConnected->setPalette(sample_palette);

    //-----------------------------------------------
    //Make START, REST and STOP accesible after point acquisition
    //-----------------------------------------------
    ui->pushButtonSTART->setEnabled(true);
    ui->pushButtonRest->setEnabled(true);
    ui->pushButtonStop->setEnabled(true);
    //-----------------------------------------------
    //Make ACQUIRE POINT accesible while robot is moving
    //-----------------------------------------------
    ui->pushButtonAcquirePoint->setEnabled(true);
    ui->pushButtonLoadPoint->setEnabled(true);
    ui->pushButtonSofTaxicPoint->setEnabled(true);
    ui->groupBoxP1P2P3->setEnabled(true);


    //-----------------------------------------------
    //Make Display Box not visible while robot is not moving
    //-----------------------------------------------
    ui->groupBoxError->setEnabled(false);
    ui->groupBoxForce->setEnabled(false);

    //-----------------------------------------------
    //Set Stimulation buttons not enabled
    //-----------------------------------------------
    ui->groupBoxAcceptableError->setEnabled(true);

    //-----------------------------------------------
    //Stop Stimulation if it is running and it is not re-arrangement
    //-----------------------------------------------
    if(!ONstimulation){
        stimTimer->stop();

    }


#ifdef DEBUG
    qDebug()<<"== Motion Finished ==";
#endif


}

void MainWindow::onRobotError(){
    QPalette sample_palette;
    sample_palette.setColor(QPalette::Window, Qt::red);
    sample_palette.setColor(QPalette::WindowText, Qt::white);

    ui->labelRobotError->setAutoFillBackground(true);
    ui->labelRobotError->setPalette(sample_palette);
}



void MainWindow::onHeadCompensationStarted(){
    //-----------------------------------------------
    //Make Display Box visible while robot is moving
    //-----------------------------------------------
    ui->groupBoxError->setEnabled(true);
    ui->groupBoxForce->setEnabled(true);

    //-----------------------------------------------
    //Set Error Thresholds
    //-----------------------------------------------
    ErrorThreshold_position = ui->spinBoxPositionThreshold->value();
    ErrorThreshold_orientation= ui->spinBoxOrientationThreshold->value();
    ErrorThreshold_z=ui->doubleSpinBoxThresholdZ->value();

    ui->groupBoxAcceptableError->setEnabled(false);

    //check if the stimulation is ON
    if(!ONstimulation){

        //-----------------------------------------------
        //Set Stimulation buttons enabled
        //-----------------------------------------------
        ui->groupBoxAutomaticStimulation->setEnabled(true);
        ui->spinBoxInterstimuliTime->setEnabled(true);

        //stimulation buttons became enabled only if the serial port is open
        if(serial.isOpen()){
            ui->pushButton10Stimulations->setEnabled(true);
            ui->pushButtonStartStimulation->setEnabled(true);
            ui->pushButtonStopStimulation->setEnabled(false);
        }
        else{
            ui->pushButton10Stimulations->setEnabled(false);
            ui->pushButtonStartStimulation->setEnabled(false);
            ui->pushButtonStopStimulation->setEnabled(false);

            MainWindow::onSerialDeviceSelection();
        }

        GOstimulation=false;

        //update session number
        Session++;
    }


}


void MainWindow::onHeadCompensationStarted_RE(){
    //-----------------------------------------------
    //Make Display Box visible while robot is moving
    //-----------------------------------------------
    ui->groupBoxError->setEnabled(true);
    ui->groupBoxForce->setEnabled(true);

    //-----------------------------------------------
    //Set Error Thresholds
    //-----------------------------------------------
    ErrorThreshold_position = ui->spinBoxPositionThreshold->value();
    ErrorThreshold_orientation= ui->spinBoxOrientationThreshold->value();
    ErrorThreshold_z=ui->doubleSpinBoxThresholdZ->value();

    //-----------------------------------------------
    //Set Stimulation buttons enabled
    //-----------------------------------------------
    ui->groupBoxAutomaticStimulation->setEnabled(true);
    ui->groupBoxAcceptableError->setEnabled(false);
    ui->spinBoxInterstimuliTime->setEnabled(true);

    //stimulation buttons became enabled only if the serial port is open
    if(serial.isOpen()){
        ui->pushButton10Stimulations->setEnabled(true);
        ui->pushButtonStartStimulation->setEnabled(true);
        ui->pushButtonStopStimulation->setEnabled(false);
    }
    else{
        ui->pushButton10Stimulations->setEnabled(false);
        ui->pushButtonStartStimulation->setEnabled(false);
        ui->pushButtonStopStimulation->setEnabled(false);

        MainWindow::onSerialDeviceSelection();
    }

    GOstimulation=false;


}




void MainWindow::onEmergencyStop(){
    //-----------------------------------------------
    //Turn on the EMERGENCY STOP label
    //-----------------------------------------------
    QPalette palette_red;
    palette_red.setColor(QPalette::Window, Qt::red);
    palette_red.setColor(QPalette::WindowText, Qt::black);

    ui->labelEmergencyStop->setAutoFillBackground(true);
    ui->labelEmergencyStop->setPalette(palette_red);

}


void MainWindow::onAcquirePointClicked(){

    //Acquire Target Point as coil pose wrt head frame
    if(TargetMutex.try_lock()){
        T_head_coil_target=T_head_coil;
        Target[0]=T_head_coil_target*T_EE_coil.inverse();

        TargetMutex.unlock();
    }


    //-----------------------------------------------
    //Update Point Info on GUI
    //-----------------------------------------------
    ui->lcdNumberX->display(Target[0](0,3));
    ui->lcdNumberY->display(Target[0](1,3));
    ui->lcdNumberZ->display(Target[0](2,3));


//    //-----------------------------------------------
//    //Save point on file (row major)
//    //-----------------------------------------------
//    if( targetPoint.open( QIODevice::ReadWrite | QIODevice::Truncate | QIODevice::Text)){
//         QTextStream out_t(& targetPoint);
//         for(int i=0;i<4;i++){
//             for(int j=0;j<4;j++){
//                 out_t<<Target[0](i,j)<<"\t";
//             }
//             out_t<<"\n";
//         }

//         targetPoint.close();
//       }


//    //Express target with respect to head reference, as EE pose.
//    // T_coil(target)= T_head_coil(target) * T_coil_EE
    Eigen::Matrix4d T_help, T_help_controlTarget;
//    T_help=Target[0]*T_EE_coil.inverse();
//    Target[0]=T_help;


    //-----------------------------------------------
    //Define Transformation matrix referred to EE frame, with identity rotation matrix
    //and DeltaSafe translation along EE z axis
    //-----------------------------------------------
    Eigen::Matrix4d T_EE_delta;
    Eigen::Matrix4d T_EE_delta_target;
     for(int i=0;i<4;i++){
          for(int j=0;j<4;j++){
            if(i==j){
                T_EE_delta(i,j)=1;
                T_EE_delta_target(i,j)=1;
            }
            else{
                T_EE_delta(i,j)=0;
                T_EE_delta_target(i,j)=0;
                }
            }
      }
      T_EE_delta(2,3)=-DeltaSafe;
      T_EE_delta_target(2,3)=DeltaControlTarget;    //towards the head


    //Define Safe Points (Target point + Safety Delta (along the axis normal to the head (i.e. EE z axis)))
    SafeTarget[0]=Target[0]*T_EE_delta;

    //Define the Target more close to the head in order to compensate the control residual error on the z axis
    //(Target point - Delta (along the axis normal to the head (i.e. EE z axis))
    T_help_controlTarget=Target[0]*T_EE_delta_target;
    TargetControl[0]=T_help_controlTarget;


#ifdef DEBUG
    QDebug dbg(QtDebugMsg);
    dbg<<"Stim Point Acquired: \n";
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            dbg<<Target[0](i,j)<<"\t";
        }
        dbg<<"\n";
    }
//    qDebug()<<"Safe Point: ";
//    for(int i=0;i<16;i++)
//        qDebug()<<SafeTarget[0](i);
#endif


    //-----------------------------------------------
    //Save target info in the selected P array
    //-----------------------------------------------
    if(ui->radioButtonP1->isChecked()){
        Target[1]=Target[0];
        SafeTarget[1]=SafeTarget[0];
        TargetControl[1]=TargetControl[0];
    }
    else if(ui->radioButtonP2->isChecked()){
        Target[2]=Target[0];
        SafeTarget[2]=SafeTarget[0];
        TargetControl[2]=TargetControl[0];
    }
    else if(ui->radioButtonP3->isChecked()){
        Target[3]=Target[0];
        SafeTarget[3]=SafeTarget[0];
        TargetControl[3]=TargetControl[0];
    }




    //-----------------------------------------------
    //Make START and STOP accesible after point acquisition
    //-----------------------------------------------
    ui->pushButtonSTART->setEnabled(true);
    ui->pushButtonStop->setEnabled(true);


}


void MainWindow::onLoadPointClicked(){
    //-----------------------------------------------
    //Load point from file
    //-----------------------------------------------
    //read Target Points
    unsigned int NPoint;
    double *CMatrices=new double[1000];

    if(targetPoint.open(QIODevice::ReadOnly | QIODevice::Text)){
        QTextStream in_p(& targetPoint);
        int s=0;
        while ( !in_p.atEnd() )
        {
           double a;
           in_p >> a;
            CMatrices[s]=a;
            s++;
        }
        NPoint=s/16;
        targetPoint.close();

        //-----------------------------------------------
        //Store Target Point Matrices (wrt head reference frame)
        //-----------------------------------------------
//        for(int z=0;z<NPoint;z++){
            for(int i=0;i<4;i++){
                for(int j=0;j<4;j++)
                    Target[0](i,j)=CMatrices[(i*4+j)];
            }
//        }


        //-----------------------------------------------
        //Update Point Info on GUI
        //-----------------------------------------------
        ui->lcdNumberX->display(Target[0](0,3));
        ui->lcdNumberY->display(Target[0](1,3));
        ui->lcdNumberZ->display(Target[0](2,3));

        T_head_coil_target=Target[0];

//        //Express target with respect to head reference, as EE pose.
//        // T_coil(target)= T_head_coil(target) * T_coil_EE
        Eigen::Matrix4d T_help, T_help_controlTarget;
        T_help=Target[0]*T_EE_coil.inverse();
        Target[0]=T_help;


        //-----------------------------------------------
        //Define Transformation matrix referred to EE frame, with identity rotation matrix
        //and DeltaSafe translation along EE z axis
        //-----------------------------------------------
        Eigen::Matrix4d T_EE_delta;
        Eigen::Matrix4d T_EE_delta_target;
         for(int i=0;i<4;i++){
              for(int j=0;j<4;j++){
                if(i==j){
                    T_EE_delta(i,j)=1;
                    T_EE_delta_target(i,j)=1;
                }
                else{
                    T_EE_delta(i,j)=0;
                    T_EE_delta_target(i,j)=0;
                    }
                }
          }
          T_EE_delta(2,3)=-DeltaSafe;
          T_EE_delta_target(2,3)=DeltaControlTarget;    //towards the head


        //Define Safe Points (Target point + Safety Delta (along the axis normal to the head (i.e. EE z axis)))
        SafeTarget[0]=Target[0]*T_EE_delta;

        //Define the Target more close to the head in order to compensate the control residual error on the z axis
        //(Target point - Delta (along the axis normal to the head (i.e. EE z axis))
        T_help_controlTarget=Target[0]*T_EE_delta_target;
        TargetControl[0]=T_help_controlTarget;

     }

    //-----------------------------------------------
    //Save target info in the selected P array
    //-----------------------------------------------
    if(ui->radioButtonP1->isChecked()){
        Target[1]=Target[0];
        SafeTarget[1]=SafeTarget[0];
        TargetControl[1]=TargetControl[0];
    }
    else if(ui->radioButtonP2->isChecked()){
        Target[2]=Target[0];
        SafeTarget[2]=SafeTarget[0];
        TargetControl[2]=TargetControl[0];
    }
    else if(ui->radioButtonP3->isChecked()){
        Target[3]=Target[0];
        SafeTarget[3]=SafeTarget[0];
        TargetControl[3]=TargetControl[0];
    }



    //-----------------------------------------------
    //Make START and STOP accesible after point acquisition
    //-----------------------------------------------
    ui->pushButtonSTART->setEnabled(true);
    ui->pushButtonStop->setEnabled(true);


}


void MainWindow::onSofTaxicPointClicked(){

    //Acquire Target Point from SofTaxic Optic
    //as coil pose wrt head frame

    if(SofTaxicMutex.try_lock()){

        Target[0]=SofTaxicTarget;
        T_head_coil_target=Target[0];

        SofTaxicMutex.unlock();
    }


    //-----------------------------------------------
    //Update Point Info on GUI
    //-----------------------------------------------
    ui->lcdNumberX->display(Target[0](0,3));
    ui->lcdNumberY->display(Target[0](1,3));
    ui->lcdNumberZ->display(Target[0](2,3));


//    //-----------------------------------------------
//    //Save point on file (row major)
//    //-----------------------------------------------
//    if( targetPoint.open( QIODevice::ReadWrite | QIODevice::Truncate | QIODevice::Text)){
//         QTextStream out_t(& targetPoint);
//         for(int i=0;i<4;i++){
//             for(int j=0;j<4;j++){
//                 out_t<<Target[0](i,j)<<"\t";
//             }
//             out_t<<"\n";
//         }

//         targetPoint.close();
//       }


//    //Express target with respect to head reference, as EE pose.
//    // T_coil(target)= T_head_coil(target) * T_coil_EE
    Eigen::Matrix4d T_help, T_help_controlTarget;
    T_help=Target[0]*T_EE_coil.inverse();
    Target[0]=T_help;


    //-----------------------------------------------
    //Define Transformation matrix referred to EE frame, with identity rotation matrix
    //and DeltaSafe translation along EE z axis
    //-----------------------------------------------
    Eigen::Matrix4d T_EE_delta;
    Eigen::Matrix4d T_EE_delta_target;
     for(int i=0;i<4;i++){
          for(int j=0;j<4;j++){
            if(i==j){
                T_EE_delta(i,j)=1;
                T_EE_delta_target(i,j)=1;
            }
            else{
                T_EE_delta(i,j)=0;
                T_EE_delta_target(i,j)=0;
                }
            }
      }
      T_EE_delta(2,3)=-DeltaSafe;
      T_EE_delta_target(2,3)=DeltaControlTarget;    //towards the head


    //Define Safe Points (Target point + Safety Delta (along the axis normal to the head (i.e. EE z axis)))
    SafeTarget[0]=Target[0]*T_EE_delta;

    //Define the Target more close to the head in order to compensate the control residual error on the z axis
    //(Target point - Delta (along the axis normal to the head (i.e. EE z axis))
    T_help_controlTarget=Target[0]*T_EE_delta_target;
    TargetControl[0]=T_help_controlTarget;


#ifdef DEBUG
    QDebug dbg(QtDebugMsg);
    dbg<<"Stim Point Softaxic Acquired: \n";
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            dbg<<Target[0](i,j)<<"\t";
        }
        dbg<<"\n";
    }

//    qDebug()<<"Safe Point: ";
//    for(int i=0;i<16;i++)
//        qDebug()<<SafeTarget[0](i);
#endif


    //-----------------------------------------------
    //Save target info in the selected P array
    //-----------------------------------------------
    if(ui->radioButtonP1->isChecked()){
        Target[1]=Target[0];
        SafeTarget[1]=SafeTarget[0];
        TargetControl[1]=TargetControl[0];
    }
    else if(ui->radioButtonP2->isChecked()){
        Target[2]=Target[0];
        SafeTarget[2]=SafeTarget[0];
        TargetControl[2]=TargetControl[0];
    }
    else if(ui->radioButtonP3->isChecked()){
        Target[3]=Target[0];
        SafeTarget[3]=SafeTarget[0];
        TargetControl[3]=TargetControl[0];
    }




    //-----------------------------------------------
    //Make START and STOP accesible after point acquisition
    //-----------------------------------------------
    ui->pushButtonSTART->setEnabled(true);
    ui->pushButtonStop->setEnabled(true);


}



void MainWindow::onPointSelected(){
#ifdef DEBUG
    qDebug()<<"P checked";
#endif

    Eigen::Matrix4d T_help;
    //-----------------------------------------------
    //Move the selected point info into the Target[0]
    //-----------------------------------------------
    if(ui->radioButtonP1->isChecked()){

        T_help=Target[1];
#ifdef DEBUG
            qDebug()<<"Target 1= "<<Target[1](0);
#endif
        //check if the point has been acquired
        if(std::isfinite(Target[1](0))){
#ifdef DEBUG
            qDebug()<<"not NAN= ";
#endif
            Target[0]=Target[1];
            SafeTarget[0]=SafeTarget[1];
            TargetControl[0]=TargetControl[1];

            ui->pushButtonSTART->setEnabled(true);
            ui->pushButtonStop->setEnabled(true);
        }
        else{
#ifdef DEBUG
            qDebug()<<"NAN= ";
#endif

            ui->pushButtonSTART->setEnabled(false);
            ui->pushButtonStop->setEnabled(false);
        }

    }
    else if(ui->radioButtonP2->isChecked()){

        T_help=Target[2];

#ifdef DEBUG
            qDebug()<<"Target 2= "<<Target[2](0);
#endif

        //check if the point has been acquired
        if(std::isfinite(Target[2](0))){
#ifdef DEBUG
            qDebug()<<"not NAN= ";
#endif
            Target[0]=Target[2];
            SafeTarget[0]=SafeTarget[2];
            TargetControl[0]=TargetControl[2];
            ui->pushButtonSTART->setEnabled(true);
            ui->pushButtonStop->setEnabled(true);
        }
        else{
#ifdef DEBUG
            qDebug()<<"NAN= ";
#endif

            ui->pushButtonSTART->setEnabled(false);
            ui->pushButtonStop->setEnabled(false);
        }
    }
    else if(ui->radioButtonP3->isChecked()){
#ifdef DEBUG
            qDebug()<<"Target 3= "<<Target[3](0);
#endif

        T_help=Target[3];

        //check if the point has been acquired
        if(std::isfinite(Target[3](0))){
#ifdef DEBUG
            qDebug()<<"not NAN= ";
#endif

            Target[0]=Target[3];
            SafeTarget[0]=SafeTarget[3];
            TargetControl[0]=TargetControl[3];
            ui->pushButtonSTART->setEnabled(true);
            ui->pushButtonStop->setEnabled(true);
        }
        else{

#ifdef DEBUG
            qDebug()<<"NAN= ";
#endif
            ui->pushButtonSTART->setEnabled(false);
            ui->pushButtonStop->setEnabled(false);
        }
    }

    //-----------------------------------------------
    //Update target info for error calculation
    //-----------------------------------------------
    T_head_coil_target=T_help;

    //-----------------------------------------------
    //Update Point Info on GUI
    //-----------------------------------------------
    ui->lcdNumberX->display(T_help(0,3));
    ui->lcdNumberY->display(T_help(1,3));
    ui->lcdNumberZ->display(T_help(2,3));


}


void MainWindow::onSTARTClicked(){

    //-----------------------------------------------
    //Call Robot Moving Function
    //-----------------------------------------------
    emit goTracking();

    GlobalSession++;

    STOP=false;
}

void MainWindow::onSTOPClicked(){

    //-----------------------------------------------
    //Stop the Robot
    //-----------------------------------------------
    STOP=true;
    TRACKING=false;
    ONstimulation=false;

    robot.stop();



}


void MainWindow::onRestClicked(){

    STOP=false;
    //-----------------------------------------------
    //Call Robot Moving Function
    //-----------------------------------------------
    emit goResting();

    //-----------------------------------------------
    //Make REST button unaccesible until the robot has reached the rest pose
    //-----------------------------------------------
    ui->pushButtonRest->setEnabled(false);
    ui->pushButtonStop->setEnabled(true);


}


void MainWindow::onNewForceValue(){


//    qDebug()<<"on new ForceValue"<<PositionError[0]<<PositionError[1]<<PositionError[2];

    int err[3];

    double counter=RearrangeElapsTime.elapsed()/1000.0;   //[s]

    //-----------------------------------------------
    //Display Coil Error values
    //-----------------------------------------------

    if(ErrorMutex.try_lock()){
        //error in the xy plane
        err[0]=sqrt(PositionError[0]*PositionError[0]+PositionError[1]*PositionError[1])*1000;    //express as mm in the progress bar
        //error in the z distance
        err[1]=PositionError[2]*1000;   //mm
        err[2]=OrientationError*180.0/M_PI; //[deg]

        ErrorMutex.unlock();

        ui->progressBarErrorPosition->setValue(err[0]);
        ui->progressBarErrorPosizionZ->setValue(err[1]);
        ui->progressBarErrorOrientation->setValue(err[2]);


        //-----------------------------------------------
        //Check if the Error is Acceptable (GREEN yes, RED not)
        //-----------------------------------------------
        bool check_xy,check_z,check_ori;


        QPalette gr ,re;
        gr.setColor(QPalette::Highlight, QColor(Qt::green));
        re.setColor(QPalette::Highlight, QColor(Qt::red));

        if(err[0]> ErrorThreshold_position){
            ui->progressBarErrorPosition->setPalette(re);
            check_xy=false;
        }
        else{
            ui->progressBarErrorPosition->setPalette(gr);
            check_xy=true;
        }
        if(err[1]>ErrorThreshold_z){
            ui->progressBarErrorPosizionZ->setPalette(re);
            check_z=false;
        }
        else{
            ui->progressBarErrorPosizionZ->setPalette(gr);
            check_z=true;
        }
        if(err[2]> ErrorThreshold_orientation){
            ui->progressBarErrorOrientation->setPalette(re);
            check_ori=false;
        }
        else{
            ui->progressBarErrorOrientation->setPalette(gr);
            check_ori=true;
        }

//        if(check_ori && check_xy && check_z){
        if(check_ori && check_xy ){
            GOstimulation=true;
            noErrorTimer+=counter;
            if(noErrorTimer>=0.003)
                errorTimer=0;
        }
        else{
            GOstimulation=false;
            errorTimer+=counter;

            noErrorTimer=0;
            //-----------------------------------------------
            //Check how much time the error was high
            //-----------------------------------------------
            if(errorTimer>=maxErrorTime){
                errorTimer=0;
                robot.stop();
                //-----------------------------------------------
                //Callback to re-arrange the coil
                //-----------------------------------------------
                emit goReArrangeRobot();

            }

        }

        ui->progressBarErrorPosition->setAutoFillBackground(true);
        ui->progressBarErrorPosizionZ->setAutoFillBackground(true);
        ui->progressBarErrorOrientation->setAutoFillBackground(true);


        //debug check
        QPalette grB ,reB;
        grB.setColor(QPalette::Background, QColor(Qt::green));
        reB.setColor(QPalette::Background, QColor(Qt::red));
        if(GOstimulation)
            ui->ledGOstimulation->setPalette(gr);
        else
            ui->ledGOstimulation->setPalette(re);
        ui->ledGOstimulation->setAutoFillBackground(true);

    }

//    qDebug()<<"elapsed:"<<RearrangeElapsTime.elapsed() <<"counter: "<<counter<<"error no timer: "<<noErrorTimer<<"error timer: "<<errorTimer;
    RearrangeElapsTime.restart();


}



void MainWindow::onSerialDeviceSelection(){
    ui->fontComboBoxSerialPort->clear();
    static const char blankString[] = QT_TRANSLATE_NOOP("SettingsDialog", "N/A");
    QString description;
    QString manufacturer;
    QString serialNumber;
    const auto infos = QSerialPortInfo::availablePorts();
        for (const QSerialPortInfo &info : infos) {
            QStringList list;

            description = info.description();
            manufacturer = info.manufacturer();
            serialNumber = info.serialNumber();

            //check if the device is not the NDI camera
            if(manufacturer.compare("NDI")){
                list << info.portName()
                     << (!description.isEmpty() ? description : blankString)
                     << (!manufacturer.isEmpty() ? manufacturer : blankString)
                     << (!serialNumber.isEmpty() ? serialNumber : blankString)
                     << info.systemLocation()
                     << (info.vendorIdentifier() ? QString::number(info.vendorIdentifier(), 16) : blankString)
                     << (info.productIdentifier() ? QString::number(info.productIdentifier(), 16) : blankString);
                ui->fontComboBoxSerialPort->addItem(list.first(), list);
            }

            //debug
#ifdef DEBUG
            qDebug()<<list;
#endif
        }

}

void MainWindow::onConnectSerialPortClicked(){


    QString serialName=ui->fontComboBoxSerialPort->currentText();

    //-----------------------------------------------
    //Open Serial Communication to Send TMS trigger
    //-----------------------------------------------
    serial.setPortName(serialName);
    serial.setBaudRate(QSerialPort::Baud115200);
    serial.setDataBits(QSerialPort::Data8);
    serial.setParity(QSerialPort::NoParity);
    serial.setStopBits(QSerialPort::OneStop);
    serial.setFlowControl(QSerialPort::NoFlowControl);

    serial.open(QIODevice::ReadWrite);


    if(serial.isOpen()){
        ui->groupBoxManualStimulation->setEnabled(true);
        ui->groupBoxAutomaticStimulation->setEnabled(true);
        //-----------------------------------------------
        //Make START button accesible
        //-----------------------------------------------
        ui->pushButtonStartStimulation->setEnabled(false);
        ui->pushButton10Stimulations->setEnabled(false);
        ui->pushButtonStopStimulation->setEnabled(false);
        ui->spinBoxInterstimuliTime->setEnabled(true);
        ui->pushButtonConnectSerialPort->setEnabled(false);
        ui->pushButtonDisconnectSerialPort->setEnabled(true);
        ui->pushButtonManualStimulation->setEnabled(true);
    }
}


void MainWindow::onDisconnectSerialPortClicked(){
    //-----------------------------------------------
    //Close Serial Communication
    //-----------------------------------------------
    if (serial.isOpen())
        {
            serial.close();
            ui->pushButtonConnectSerialPort->setEnabled(true);
            ui->pushButtonDisconnectSerialPort->setEnabled(false);
            //-----------------------------------------------
            //Make START button accesible
            //-----------------------------------------------
//            ui->groupBoxAutomaticStimulation->setEnabled(false);
            ui->pushButtonStartStimulation->setEnabled(false);
            ui->pushButton10Stimulations->setEnabled(false);
            ui->pushButtonStopStimulation->setEnabled(false);
            ui->pushButtonManualStimulation->setEnabled(false);
        }

}


void MainWindow::onStartStimulationClicked(){

    //-----------------------------------------------
    //Set Timer for Stimulation Protocol to send TRIGGER to TMS
    //-----------------------------------------------
    int timeStep=ui->spinBoxInterstimuliTime->value(); //[s]
    stimTimer->setInterval(timeStep*1000);

    //signal slot connection
    // ? posso metterlo anche all'inizio tra tutte le connsessioni


    //flag stimulation ON
    ONstimulation=true;


    //set 10 stim not active
    stimuli_10=false;

    //debug check
    ui->lcdNumberStimulation->display(0);

    stimTimer->start();

    //-----------------------------------------------
    //Make START button unaccesible
    //-----------------------------------------------
    ui->pushButtonStartStimulation->setEnabled(false);
    ui->pushButton10Stimulations->setEnabled(false);
    ui->pushButtonStopStimulation->setEnabled(true);
    ui->groupBoxManualStimulation->setEnabled(false);

    ui->spinBoxInterstimuliTime->setEnabled(false);


}


void MainWindow::onStopStimulationClicked(){

    stimTimer->stop();
    //-----------------------------------------------
    //Make Stimulation buttons accesible
    //-----------------------------------------------
    ui->pushButtonStartStimulation->setEnabled(true);
    ui->pushButton10Stimulations->setEnabled(true);
    ui->pushButtonStopStimulation->setEnabled(false);
    ui->groupBoxManualStimulation->setEnabled(true);

    ui->spinBoxInterstimuliTime->setEnabled(true);


}


void MainWindow::on10StimulationClicked(){

    //-----------------------------------------------
    //Set Timer for Stimulation Protocol to send TRIGGER to TMS
    //-----------------------------------------------
    int timeStep=ui->spinBoxInterstimuliTime->value(); //[s]
    stimTimer->setInterval(timeStep*1000);

    //set 10 stim active
    stimuli_10=true;

    //debug check
    ui->lcdNumberStimulation->display(0);

    stimTimer->start();

    //-----------------------------------------------
    //Make Stimulation buttons accesible
    //-----------------------------------------------
    ui->pushButtonStartStimulation->setEnabled(false);
    ui->pushButton10Stimulations->setEnabled(false);
    ui->pushButtonStopStimulation->setEnabled(true);
    ui->groupBoxManualStimulation->setEnabled(false);

    ui->spinBoxInterstimuliTime->setEnabled(false);

}

void MainWindow::onManualStimulationClicked(){

    //Send trigger to TMS if the user manually press the button

    // setup serial communication variables
    QByteArray toSerial;
    toSerial[0]='T';

    // send trigger
    if (serial.isOpen() && serial.isWritable())
    {
       serial.write(toSerial);
       serial.flush();

    }


}



void MainWindow::SendTriggerTMS(){

    // setup serial communication variables
    QByteArray toSerial;
    QByteArray toSerial10;
    toSerial[0]='T';
    toSerial10[0]='S';

    if(GOstimulation){

        // send trigger
        if (serial.isOpen() && serial.isWritable())
        {

            if(!stimuli_10)
                serial.write(toSerial);
            else
                serial.write(toSerial10);
          serial.flush();

          //Write stimulation time
          //-----------------------------------------------
          //Save Data to file
          //-----------------------------------------------
          if( stimulation.open(QIODevice::Append | QIODevice::Text)){
               QTextStream out_t(& stimulation);
//                if(print_data.mutex.try_lock()){
//                   out_t<<Session<<"\t"<<print_data.global_time<<"\t";

//                   print_data.mutex.unlock();
//                }
               if(print_data.mutex.try_lock()){
                  out_t<<Session<<"\t"<<ElapsTime.elapsed()<<"\t";

                  print_data.mutex.unlock();
               }
                if(ErrorMutex.try_lock()){
                    out_t<<PositionError[0]<<"\t";
                    out_t<<PositionError[1]<<"\t";
                    out_t<<PositionError[2]<<"\t";
                    out_t<<OrientationError<<"\t";
                    ErrorMutex.unlock();
                }
                out_t<<"\n";
               stimulation.close();
             }

        }

        //debug control
        double n=ui->lcdNumberStimulation->value();
        ui->lcdNumberStimulation->display(n+1);

    }
    else{

//        stimTimer->stop();

//        double timeStep=(double)ui->spinBoxInterstimuliTime->value()*1000; //[s]
//        QElapsedTimer et;
//        et.start();
//        double t_elapsed=et.elapsed();
//        while(!GOstimulation || t_elapsed<timeStep/2){
//            // do nothing
//            t_elapsed=et.elapsed();
//        }

//        if(GOstimulation){
//            // send trigger
//            if (serial.isOpen() && serial.isWritable())
//            {

//              serial.write(toSerial);
//              serial.flush();

//            }
//            //debug control
//            double n=ui->lcdNumberStimulation->value();
//            ui->lcdNumberStimulation->display(n+1);

//        }

//        //restart the timer
//        stimTimer->start();

    }



    //Check if only 10 stimuli have to be provided
    if(stimuli_10){
        stimCount++;
        if(stimCount>=10){
            stimTimer->stop();
            //-----------------------------------------------
            //Make Stimulation buttons accesible
            //-----------------------------------------------
            ui->pushButtonStartStimulation->setEnabled(true);
            ui->pushButton10Stimulations->setEnabled(true);
            ui->pushButtonStopStimulation->setEnabled(false);

            ui->groupBoxManualStimulation->setEnabled(true);

            ui->spinBoxInterstimuliTime->setEnabled(true);
        }
    }


}



void MainWindow::onPrintData(QVector<double> data){


    //-----------------------------------------------
    //Save Data to file
    //-----------------------------------------------
    if( cmdRobot.open(QIODevice::Append | QIODevice::Text)){
         QTextStream out_t(& cmdRobot);
//#ifdef DEBUG
//    qDebug()<<"printData";
//#endif

         for(int i=0;i<data.size();i++)
             out_t<<data[i]<<"\t";
         out_t<<Session<<"\t";    //on Head session number
         out_t<<GlobalSession<<"\t";
         out_t<<onHEAD;
         out_t<<"\n";

         cmdRobot.close();
       }


}



//-----------------------------------------------
//Minimum Jerk Functions
//-----------------------------------------------


double FifthOrderPoly(double p0, double pf, double c){

   double pc;
   pc=6.0*(pf-p0)*c*c*c*c*c -15.0*(pf-p0)*c*c*c*c + 10.0*(pf-p0)*c*c*c + p0;
   return pc;

}

void LogSO3(Eigen::Matrix3d R, Eigen::Vector3d &w, double &theta){

   theta= acos( (R(0,0)+R(1,1)+R(2,2) -1.0)/2.0 );

   double arg=(R(0,0)+R(1,1)+R(2,2) -1.0)/2.0 ;
   //debug
   //check if the acos argument is >=1 because of numeric computation
   if(arg >=1.0 || arg<=-1.0)
       theta=0.0;



   Eigen::Matrix3d r= R-R.transpose();
   if(theta==0.0){
           w(0)= -r(1,2);
           w(1)= r(0,2);
           w(2)= -r(0,1);
   }
   else{
           w(0)= -r(1,2)/sin(theta)/2.0;
           w(1)= r(0,2)/sin(theta)/2.0;
           w(2)= -r(0,1)/sin(theta)/2.0;
   }

//   //debug
//   qDebug()<<"LogSO3---theta: "<<theta<<sin(theta);
//   qDebug()<<"LogSO3---w: "<<w(0)<<w(1)<<w(2);
//   qDebug()<<"LogSO3---R: "<<R(0,0)<<R(1,1)<<R(2,2)<<acos( (R(0,0)+R(1,1)+R(2,2) -1.0)/2.0 );
//   qDebug()<<"LogSO3---r: "<<r(1,2)<<r(0,2)<<r(0,1);

}


Eigen::Matrix3d ExpSO3(Eigen::Vector3d r_c){

   //Identity matrix
   Eigen::Matrix3d RI, Rout;
   RI=Eigen::MatrixXd::Identity(3,3);

   double norm= r_c.norm();

   if(norm==0.0){
        return RI;
   }
   else{
       Eigen::Matrix3d sw;
        sw=SKEW(r_c)*(sin(norm)/norm) + SKEW(r_c)*SKEW(r_c)*(( 1-cos(norm) )/ (norm*norm));
       Rout=RI+sw;
       return Rout;
   }

}

Eigen::Matrix3d SKEW(Eigen::Vector3d a){
   Eigen::Matrix3d out;
   out(0,0)= 0.0;     out(0,1)= -a(2);   out(0,2)= a(1);
   out(1,0)= a(2);  out(1,1)= 0.0;       out(1,2)= -a(0);
   out(2,0)= -a(1);  out(2,1)= a(0);    out(2,2)= 0.0;

   return out;
}





void MainWindow::on_pushButtonSavePoint_clicked()
{
    //-----------------------------------------------
    //Save point on file (row major)
    //-----------------------------------------------
    if( targetPoint.open( QIODevice::ReadWrite | QIODevice::Truncate | QIODevice::Text)){
         QTextStream out_t(& targetPoint);
         for(int i=0;i<4;i++){
             for(int j=0;j<4;j++){
                 out_t<<Target[0](i,j)<<"\t";
             }
             out_t<<"\n";
         }

         targetPoint.close();
       }
}

#include <ros.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

//#include <Braccio.h>
#include <Servo.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

//Posicion actual del robot
int curPositions[6] = {90 , 45, 180 , 180 ,90 ,10};
//Nombre sde los servos para la validacion
String servoNames[6] = {"base","shoulder","elbow","wrist_rot","wrist_ver","gripper"};

ros::NodeHandle  nh;


std_msgs::String str_msg;


void updateCurPositions(int positions[]){
  for(int i = 0 ; i < 6 ; i++){
    curPositions[i] = positions[i];
  }
}

//Nodo de movimiento simple.

boolean checkPositions( const int names[] , const trajectory_msgs::JointTrajectoryPoint& point){

  boolean res = true;
  if(point.positions_length != 6){
    res = false;
  }
  else{
    int i = 0;
    float * pos = point.positions;
    
    while(i < point.positions_length && res){
      float * curPos = pos + i;    
      float val = *curPos;
      if(names[i] == 1){
        if(val > 165 || val < 0){
          res = false;
        }
      }
      else if(names[i] == 5){
        if(val > 73 || val < 10){
          res = false;
        }
      }
      else{
        if(val > 180 || val < 0){
          res = false;
        }
      }
      i++;
    }
  }
  return res;
}


boolean checkPoint(const int names[] , const trajectory_msgs::JointTrajectoryPoint& point ){

  boolean res = true;
  res = checkPositions(names , point);
  if(res){
    
      int spd = point.time_from_start.sec;
      if(spd > 30 || spd < 10){
        
        res = false;
        
      }
  }
  return res;
}

void getPos(const int names[] , const trajectory_msgs::JointTrajectoryPoint& point , int res[]){
  for(int i = 0 ; i < 6; i++){
    float* curPos = point.positions +i;
    res[names[i]] = (int)*curPos;
  }
  
}

void cbSM(const trajectory_msgs::JointTrajectoryPoint& message){

        
    int names[6] = {0,1,2,3,4,5};
    if(checkPoint(names , message)){
      int pos[6];
      getPos(names, message , pos);
      int spd = message.time_from_start.sec;

      
      Serial3.print(spd);
      Serial3.print(pos[0]);
      Serial3.print(pos[1]);
      Serial3.print(pos[2]);
      Serial3.print(pos[3]);
      Serial3.print(pos[4]);
      Serial3.println(pos[5]);

      
      //Braccio.ServoMovement(spd, pos[0] , pos[1] , pos[2] , pos[3] , pos[4] , pos[5]);         

      updateCurPositions(pos);

      
    }

    else{
      Serial3.println("ERROR -> Incorrect message sent. Try to send the same message to 'simpleMovement' for help.");
    }
}

//////////////////////////////////////////////////////////////////////
////////////////////// Trajectory Movement Node //////////////////////
//////////////////////////////////////////////////////////////////////


boolean checkNames(const trajectory_msgs::JointTrajectory& message , int names[]){

    boolean res = true;
    boolean checked[6] = {false, false , false , false ,false , false};
    if(message.joint_names_length != 6){
      return false;
    }
    else{
      int i = 0;
      char** mNames = message.joint_names;
      while(i < 6 && res){
        char* curName = *mNames + i;
        if(curName == "base"){
          if(checked[0]){
            res = false;
          }
          else{
            names[i] = 0;
            checked[0] = true;
          }
        }
        else if(curName == "shoulder"){
          if(checked[1]){
            res = false;
          }
          else{
            names[i] = 1;
            checked[1] = true;
          }
        }
        else if(curName == "elbow"){
          if(checked[2]){
            res = false;
          }
          else{
            names[i] = 2;
            checked[2] = true;
          }
        } 
        else if(curName == "wrist_rot"){
          if(checked[3]){
            res = false;
          }
          else{
            names[i] = 3;
            checked[3] = true;
          }
        }
        else if(curName == "wrist_ver"){
          if(checked[4]){
            res = false;
          }
          else{
            names[i] = 4;
            checked[4] = true;
          }
        }
        else if(curName == "gripper"){
          if(checked[5]){
            res = false;
          }
          else{
            names[i] = 5;
            checked[5] = true;
          }
        }
        else{
          res = false;
        }
        i++;
      }
    }
}

boolean checkPoints( const int names[] , const trajectory_msgs::JointTrajectory& message ){
  bool res = true;
  if(message.points_length < 1){
    res = false;
  }
  int i = 0;
  while (i < message.points_length && res){
    trajectory_msgs::JointTrajectoryPoint* curPoint = message.points + i;
    res = checkPoint(names , *curPoint);
    i++;
  }
  return res;
}


void cbTM(const trajectory_msgs::JointTrajectory& message){
  int names[6];

  if(checkNames(message , names)){
    if(checkPoints(names , message)){
      int pos[6];
      int spd;

      Serial3.println("Starting trajectory");
      
      for(int i = 0 ; i < message.points_length; i++){
        
        trajectory_msgs::JointTrajectoryPoint* curPointP = message.points + i;
        trajectory_msgs::JointTrajectoryPoint curPoint = *curPointP;
        getPos(names, curPoint , pos);
        int spd = *curPoint.velocities;

        Serial3.println("--------------------");

        Serial3.print(spd);
        Serial3.print(pos[0]);
        Serial3.print(pos[1]);
        Serial3.print(pos[2]);
        Serial3.print(pos[3]);
        Serial3.print(pos[4]);
        Serial3.println(pos[5]);
        
        //Braccio.ServoMovement(spd, pos[0] , pos[1] , pos[2] , pos[3] , pos[4] , pos[5]);

        updateCurPositions(pos);

        
      }
      Serial3.println("Trajectory ended successfuly");
    }
  }
  
}

//Subscribers.

ros::Subscriber<trajectory_msgs::JointTrajectory> subTM("trajectoryMovementArduino", &cbTM);
ros::Subscriber<trajectory_msgs::JointTrajectoryPoint> subSM("simpleMovementArduino", &cbSM);


void setup()
{

  Serial3.begin(9600);

  //Braccio.begin();
  
  nh.initNode();  
  
  nh.subscribe(subSM);
  nh.subscribe(subTM);
  
}

void loop(){

  nh.spinOnce();
  delay(10);
 
}

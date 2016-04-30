/*This node performs the motion planning using a BFS "wavefront" algorithm
*/
#define HAUTEUR 2000 //en mm
#define LARGEUR 3000//en mm
#define DEC 100//en mm

//C++ Libraries
#include <unistd.h>
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include "motion_planning/wavefrontROS.h"


//ROS
#include "ros/ros.h"

//ROS msgs
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point32.h>

//Namespaces

using namespace std;

//Global Variables
ros::Publisher pub_point;
geometry_msgs::Point32 PointCons;

wavefront W;
vector<node> liste;
double x;
double y;
double theta;
float Hres,Lres;

//Interpolation parameters
double f;//fréquence d'envoi du générateur de trajectoires
double tf;//temps de mouvement pour avancer de 14mm (sqrt(2)*resolution)
int n=20;//nombre de points de passage
float xcprev,ycprev;


void coord2rc(float x,float y,int &i,int &j){
	i=(int)(x/Hres);
	j=(int)(y/Lres);
        if(i<0) i=0;
        if(i>=W.size_h-1) i=W.size_h-1;
        if(j<0) j=0;
        if(j>=W.size_l-1) j=W.size_l-1;
}

void rc2coord(int i,int j,float &x,float &y){
	x=i*Hres+Hres/2;
	y=j*Lres+Lres/2;
}

void CallBack_RefreshPosition(geometry_msgs::Pose2D Pose){
  theta=Pose.theta;
  x=Pose.x+DEC*cos(theta);
	y=Pose.y+DEC*sin(theta);
}

void CallBack_Compute(geometry_msgs::Point32 ConsPosition){
    liste.clear();
    int i,j;
    xcprev=x;
    ycprev=y;
    coord2rc(ConsPosition.x,ConsPosition.y,i,j);
    W.setgoal(i,j);
    coord2rc(x,y,i,j);
    W.setstart(i,j);
    W.propagate_wave();
    liste=W.find_path();
    W.affichersol("/home/pi/catkin_ws/src/motion_planning/src/result.txt");
    W.clearmatrix();
}

int main(int argc,char** argv){

  //ROS Initialization
  ros::init(argc,argv,"motion_planning_node");
  ROS_INFO("Motion Planning enabled\n");
  ros::NodeHandle nh;//ROS Handler 
  
  //Parameters
  nh.param("tf",tf,0.05);

  //Subscribing
  ros::Subscriber pose;
  ros::Subscriber comp_cmd;
  ROS_INFO("Suscribing to topic\n");
  pose=nh.subscribe<geometry_msgs::Pose2D>("/Pose",1,CallBack_RefreshPosition);
  comp_cmd=nh.subscribe<geometry_msgs::Point32>("/WavConsPosition",1,CallBack_Compute);
  
  //Service configuration

  //Publishing
  pub_point= nh.advertise<geometry_msgs::Point32>("/PointCons",1);

  //Configuration
    char* pPath;
    pPath = getenv("TEAM");
    if(strcmp(pPath,"purple")==0){
     ROS_INFO("Team Purple");
     W.readfile("/home/pi/catkin_ws/src/motion_planning/src/map_purple.txt");
   }else{
     ROS_INFO("Team Green");
     W.readfile("/home/pi/catkin_ws/src/motion_planning/src/map_green.txt"); 
  }
  
  Hres=HAUTEUR/((float)W.size_h);
  Lres=LARGEUR/((float)W.size_l);
  float ti=tf/n;
  f=1/ti;

  sleep(10);
 
  ros::Rate rate(f);
  while (nh.ok()){
    if(!liste.empty()){
		float xc,yc;
		rc2coord(liste[0].ligne,liste[0].colonne,xc,yc);
		
		//Polynomial interpolation
		float D[2]={xc-xcprev,yc-ycprev};
		int i=0;
		while(i<=n){
			float r=3.0*pow((i*ti/tf),2.0)-2.0*pow((i*ti/tf),3.0);
			xc=xcprev+r*D[0];
			yc=ycprev+r*D[1];
			PointCons.x=xc;
		  PointCons.y=yc;
		  PointCons.z=0;
			pub_point.publish(PointCons);
			rate.sleep();
			i++;
		}
		xcprev=xc;
    ycprev=yc;
		//Delete first element of "liste"
		for(unsigned it=0;it<liste.size()-1;it++){
			liste[it]=liste[it+1];
		}
		liste.pop_back();
	}
    //Listen for topics
	ros::spinOnce();
	rate.sleep();
  }

  ROS_INFO("ROS-Node Terminated\n");
 
 }

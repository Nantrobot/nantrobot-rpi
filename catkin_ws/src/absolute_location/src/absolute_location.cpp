//********************************************************************
//                      ABSOLUTE LOCATION NODE
// Author : Alexis MARTIN
// Mail : alexis.francois.martin@gmail.com
// Behaviour : This node perform an absolute localisation of beacon in
//             the robot frame and a localisation of obstacle in the map
//             frame
//********************************************************************

//********************************************************************
// INCLUDES
//********************************************************************
// Precompilater variable DisplayON unlock the publication of 4 LaserScan topic which describe
// the localisation of the beacon and the localisation of the obstacle
// PS : Erase to make the code faster
#define DisplayOn

// Precompilater variable OutputON unlock the display of data which describe
// the localisation of the beacon and the running process of the node
// PS : Erase to make the code faster
#define OutputON

// ROS management includes
#include "ros/ros.h"
#include <std_msgs/Empty.h>              // used for the estimation topic request "Request_Estimation"
#include <std_msgs/Float32.h>            // used to change the mahalanobis threshold
#include <std_msgs/Float32MultiArray.h>  // used to hold all the obstacles position
#include <geometry_msgs/Pose2D.h>        // used to hold the pose of the robot in the map
#include <geometry_msgs/Point32.h>       // used to hold the beacon position in the robot frame "Mesure_Kalman"
#include <sensor_msgs/LaserScan.h>       // used to hold all the beacon valid point in laserScan and the laserScan of the Hokuyo

#include <sstream>

// other management library used
#include <stdlib.h>           // malloc
#include <stdio.h>            //printf, fflush
#include <vector>             // vector
#include <sys/time.h>         // gettimeofday
#include <iostream>           // cout
#include <math.h>             // pow, sqrt
#include <eigen3/Eigen/Dense> // eigen matrix vector

// personal include of global const
#define Pi 3.14

//********************************************************************
// NAMESPACE
//********************************************************************
using namespace std;   // used for monitor output
using namespace Eigen; // used for matrix calculation

//********************************************************************
// STRUCTURE
//********************************************************************
// TBeacon structure holds all the interesting data for a beacon
typedef struct {
  Vector2f y_hat;        // position of the beacon in the robot frame
  Matrix2f sigma_maha;   // covariance matrix for the beacon
}Tbeacon, *Pbeacon;

// TScan structure hold all the interesting data for a scan of the hokuyo
typedef struct{
    struct timeval timestamp;       // timestamp of the scan
    std::vector<Vector2f> scan_vec; // vector of all the point in the robot frame
}TScan, *PScan;

//********************************************************************
// Definition of all the application wide variable
//********************************************************************
const float longueur=3000;              // longueur du terrain en mm
const float largeur=2000;               // largeur du terrain en mm
const float Rayon=30;                   // rayon de la beacon en mm
const float x_hokuyo = -30;             // déportation selon x de l'hokuyo dans le repère robot en mm
const float y_hokuyo = -5;               // déportation selon y de l'hokuyo dans le repère robot en mm
const float deportation_angle = 0.0267;
const int beacon_nb = 3;                // nombre de balise sur le terrain
Tbeacon beacon[beacon_nb];              // liste des balises selon la strucure TBeacon
float mahalanobis_threshold = 6.0;      // critère de distance de mahalanobis (mm)
TScan scan;                             // buffer d'un scan d'hokuyo
float pose_x;                           // buffer de la position du robot selon x (mm)
float pose_y;                           // buffer de la position du robot selon y (mm)
float pose_theta;                       // buffer de l'orientation du robot theta (rad)
bool fresh_scan = false;                // indication de MAJ du scan de l'hokuyo
bool fresh_estimation = false;          // indication de MAJ de réception des informations sur les balises

//********************************************************************
// Definition Gauss-Algorithm parameter
//********************************************************************
// Les paramètres
const float TolX = 0.0000000000001;     // critère d'arret sur la variation du point
const float TolG = 0.0000000000001;     // critère d'arret sur la variation du gradient
const int MaxIter = 100;               // critère d'arret sur le nombred'itération
float beta = 0.75;                      // paramètre de décroissance sur la condition d'Armijo
float Armijo_const = 0.0001;            // critère de décroissance sur la condition d'Armijo

//intermediare de calcul (just buffer variable create as global to avoid the definition of new variable every loop)
Vector2f center_init;                   // point d'initialisation de l'algorithme
float dist_maha;                        // hold the mahalanobis distance for a point
Vector2f diff;                          // hold the position difference between current point and the center
MatrixXf diff_transpose(1,2);           // hold the transposition of diff
MatrixXf inv(2,2);                      // hold the inv of the covariance matrix
Vector2f temp;                          // hold product of matrix
Vector2f cluster;

#ifdef OutputON
    int main_cnt = 0;                      // hold the number of main loop done
    int useful_main_cnt = 0;               // hold the number of main loop done with a NEW scan
    int callback_estimation = 0;           // hold the number of callback for "Loc_Estimation" done
    int useful_callback_estimation = 0;    // hold the number of callback for "Loc_Estimation" done when the data is not MAJ
    int callback_hokuyo = 0;               // hold the number of callback for "scan" done
    int useful_callback_hokuyo = 0;        // hold the number of callback for "scan" done when the data is not MAJ
#endif /* OutputON */

//********************************************************************
// Predefinition of topic variable
//********************************************************************
#ifdef DisplayOn
    sensor_msgs::LaserScan beacon1_laserscan;  // laserScan contenant les points relatifs à la beacon 1
    sensor_msgs::LaserScan beacon2_laserscan;  // laserScan contenant les points relatifs à la beacon 2
    sensor_msgs::LaserScan beacon3_laserscan;  // laserScan contenant les points relatifs à la beacon 3
    sensor_msgs::LaserScan obstacle_laserscan; // laserScan contenant les points relatifs aux obstacles
#endif /* DisplayOn */

//*************************************************************
// Definition of ROS CallBack functions
//*************************************************************
// EstimationCallback : Update the TBeacon datas if the data is not MAJ
void EstimationCallback(std_msgs::Float32MultiArray msg){
    if(fresh_estimation == false){
        int j=0;
        for(int i = 0 ; i < beacon_nb ; i++ , j+=6){
            beacon[i].y_hat << msg.data[j], msg.data[j+1];
            beacon[i].sigma_maha << msg.data[j+2], msg.data[j+3],
                                    msg.data[j+4], msg.data[j+5];
        }
        fresh_estimation = true;
        #ifdef OutputON
	 printf("\033[8;0HFrom EstimationCallback : yhat0 = [%f,%f] \033[K\n", beacon[0].y_hat(0,0), beacon[0].y_hat(1,0));
            printf("\033[9;0HFrom EstimationCallback : yhat0 = [%f,%f] sigma = [%f,%f,%f,%f]033[K\n", beacon[1].y_hat(0,0), beacon[1].y_hat(1,0), beacon[1].sigma_maha(0,0), beacon[1].sigma_maha(1,0), beacon[1].sigma_maha(0,1), beacon[1].sigma_maha(1,1));
            printf("\033[10;0HFrom EstimationCallback : yhat0 = [%f,%f] \033[K\n", beacon[2].y_hat(0,0), beacon[2].y_hat(1,0));
        #endif /* OutputON */
    }
}

// ParameterCallback : Allow user to change the mahalanobis threshold
void ParameterCallback(std_msgs::Float32 msg){
    mahalanobis_threshold = msg.data;
    #ifdef OutputON
        printf("\033[6;0HFrom ParameterCallback : mahalonobis_threshold = %f \033[K\n", mahalanobis_threshold);
    #endif // OutputON
}

// ParameterCallback : Update the TScan datas if the data is not MAJ
void HokuyoCallback(sensor_msgs::LaserScan msg){
    if(fresh_scan == false){
        float angle_increment = msg.angle_increment;
        float angle_min = msg.angle_min;
        float angle_max = msg.angle_max;
        float angle_temp = angle_min;

        // fulfill the scan.scan_vec with the position of the point in the hokuyo frame in mm
        Vector2f temp_point;
        scan.scan_vec.clear();
        int i = 0;
        std::vector<Vector2f>::iterator it;
        while(angle_temp < angle_max){
            temp_point << (msg.ranges[i])*1000.0 * cosf(angle_temp-deportation_angle), (msg.ranges[i])*1000.0 * sinf(angle_temp-deportation_angle);
            it = scan.scan_vec.begin();
            it = scan.scan_vec.insert ( it , temp_point );
            angle_temp = angle_temp + angle_increment;
            i++;
        }
        gettimeofday(&(scan.timestamp), NULL);

        #ifdef DisplayOn
            beacon1_laserscan.header = msg.header;
            beacon1_laserscan.angle_min = msg.angle_min;
            beacon1_laserscan.angle_max = msg.angle_max;
            beacon1_laserscan.angle_increment = msg.angle_increment;
            beacon1_laserscan.time_increment = msg.time_increment;
            beacon1_laserscan.scan_time = msg.scan_time;
            beacon1_laserscan.range_min = msg.range_min;
            beacon1_laserscan.range_max = msg.range_max;

            beacon2_laserscan.header = msg.header;
            beacon2_laserscan.angle_min = msg.angle_min;
            beacon2_laserscan.angle_max = msg.angle_max;
            beacon2_laserscan.angle_increment = msg.angle_increment;
            beacon2_laserscan.time_increment = msg.time_increment;
            beacon2_laserscan.scan_time = msg.scan_time;
            beacon2_laserscan.range_min = msg.range_min;
            beacon2_laserscan.range_max = msg.range_max;

            beacon3_laserscan.header = msg.header;
            beacon3_laserscan.angle_min = msg.angle_min;
            beacon3_laserscan.angle_max = msg.angle_max;
            beacon3_laserscan.angle_increment = msg.angle_increment;
            beacon3_laserscan.time_increment = msg.time_increment;
            beacon3_laserscan.scan_time = msg.scan_time;
            beacon3_laserscan.range_min = msg.range_min;
            beacon3_laserscan.range_max = msg.range_max;

            obstacle_laserscan.header = msg.header;
            obstacle_laserscan.angle_min = msg.angle_min;;
            obstacle_laserscan.angle_max = msg.angle_max;
            obstacle_laserscan.angle_increment = msg.angle_increment;
            obstacle_laserscan.time_increment = msg.time_increment;
            obstacle_laserscan.scan_time = msg.scan_time;
            obstacle_laserscan.range_min = msg.range_min;
            obstacle_laserscan.range_max = msg.range_max;
        #endif /* DisplayOn */

        fresh_scan = true;  // scan is update
    }
}

// ParameterCallback : Update the position datas of the robot in the map (use to detect obstacle)
void PoseCallback(geometry_msgs::Pose2D msg){
    pose_x = (float)msg.x;
    pose_y = (float)msg.y;
    pose_theta = (float)msg.theta;
    #ifdef OutputON
        printf("\033[12;0HFrom PoseCallback : Pose = [x = %f, y = %f, theta = %f] \033[K\n", pose_x, pose_y, pose_theta);
    #endif /* OutputON */
}

//*************************************************************
// Definition of personal functions
//*************************************************************
    //*************************************************************
    // Definition of personal functions for time output management
    //*************************************************************
struct timeval diffTimeCalculate(struct timeval tic ,struct timeval toc){
    struct timeval diff_t;
    diff_t.tv_sec  = (toc.tv_usec-tic.tv_usec < 0) ? toc.tv_sec-tic.tv_sec-1 : toc.tv_sec-tic.tv_sec;
    diff_t.tv_usec = (toc.tv_usec-tic.tv_usec < 0) ? toc.tv_usec-tic.tv_usec+1000000 : toc.tv_usec-tic.tv_usec;
    return diff_t;
}

void diffTimeDisplay(struct timeval time, char * instruction_name, int line){
    printf("\033[%i;0HThe time to perform the instruction _%s_ : %ld seconds and %ld microseconds.\033[K\n", line, instruction_name, time.tv_sec,time.tv_usec);
    return;
}

void diffTimeCndD(struct timeval tic ,struct timeval toc, char * instruction_name, int line){
    struct timeval diff_t;
    diff_t = diffTimeCalculate(tic, toc);
    diffTimeDisplay(diff_t, instruction_name, line);
    return;
}

    //*************************************************************
    // Definition of personal functions for GaussNewton algorithm
    //*************************************************************
float CalculateCriteria(Vector2f X, std::vector<Vector2f> data){
    float criteria = 0;
    for(int i = 0 ; i < data.size() ; i++){
        criteria += powf(powf(X(0,0)-data[i](0,0),2) + powf(X(1,0)-data[i](1,0),2) - powf(Rayon,2),2);
    }
    criteria /= 2.00;
    return criteria;
}

VectorXf CalculateR(Vector2f X, std::vector<Vector2f> data){
    VectorXf R(data.size(),1);
    for(int i = 0 ; i < data.size() ; i++){
        R(i,0) = powf(data[i](0,0)-X(0,0),2) + powf(data[i](1,0)-X(1,0),2) - powf(Rayon,2);
    }
    return R;
}

MatrixXf CalculateJacobian(Vector2f X, std::vector<Vector2f> data){
    MatrixXf jacobian(data.size(),2);
    for(int i = 0 ; i < data.size() ; i++){
        jacobian(i,0) = -2.0*(data[i](0,0)-X(0,0));
        jacobian(i,1) = -2.0*(data[i](1,0)-X(1,0));
    }
    return jacobian;
}

float CalculateDiffX(Vector2f X_current, Vector2f X_prec){
    float DiffX;
    DiffX = powf(X_current(0,0)-X_prec(0,0),2) + powf(X_current(1,0)-X_prec(1,0),2);
    return DiffX;
}

Vector2f CalculateGradient(Vector2f X, std::vector<Vector2f> data){
    Vector2f gradient;
    MatrixXf jacobian(data.size(),2);
    jacobian = CalculateJacobian(X,data);
    MatrixXf jacobian_t(2,data.size());
    jacobian_t = jacobian.transpose();
    MatrixXf R(data.size(),1);
    R = CalculateR(X,data);
    gradient = jacobian_t*R;
    return gradient;
}

float CalculateDiffG(Vector2f X_current, Vector2f X_prec, std::vector<Vector2f> data){
    float DiffG;
    Vector2f gradient_X_current; Vector2f gradient_X_prec;
    gradient_X_current = CalculateGradient(X_current, data);
    gradient_X_prec = CalculateGradient(X_prec, data);
    DiffG = CalculateDiffX(gradient_X_current, gradient_X_prec);
    return DiffG;
}

int StopTest(float diffG, float diffX, int iter){
    if(iter > MaxIter){return 1;}
    else if(diffX < TolX){return 2;}
    else if (diffG < TolG){return 3;}
    else{return 0;}
}


//*************************************************************
// MAIN
//*************************************************************
int main(int argc, char **argv)
{
	//*************************************************************
    // Init ROS
    //*************************************************************
	ros::init(argc, argv, "absolute_location"); // init the node
    ros::NodeHandle n;                          // usefull object to create publisher and subcriber :

    //********************************************************************
    // Definition the publisher object
    //********************************************************************
    ros::Publisher beacon_mesure_pub = n.advertise<geometry_msgs::Point32>("Mesure_Kalman", 1);              // beacons localisation in sensor frame
    ros::Publisher estimation_request_pub = n.advertise<std_msgs::Empty>("Request_Estimation",1);            // request estimation
    ros::Publisher obstacle_position_pub = n.advertise<std_msgs::Float32MultiArray>("Obstacle_estimation",1);// obstacle localisation in map frame
    ros::Publisher reset_localisation_pub = n.advertise<geometry_msgs::Point32>("PointCons", 1);                  // reset_robot localisation

    #ifdef DisplayOn
        ros::Publisher beacon1_laserscan_pub = n.advertise<sensor_msgs::LaserScan>("scan_beacon1", 1);       // to display on RVIZ
        ros::Publisher beacon2_laserscan_pub = n.advertise<sensor_msgs::LaserScan>("scan_beacon2", 1);       // to display on RVIZ
        ros::Publisher beacon3_laserscan_pub = n.advertise<sensor_msgs::LaserScan>("scan_beacon3", 1);       // to display on RVIZ
        ros::Publisher obstacle_laserscan_pub = n.advertise<sensor_msgs::LaserScan>("scan_obstacle", 1);     // to display on RVIZ
    #endif /* DisplayOn */

    //********************************************************************
    // Definition the subscriber object
    //********************************************************************
    ros::Subscriber hokuyo_sub = n.subscribe("scan",1,&HokuyoCallback);                    // Scan of the hokuyo
    ros::Subscriber estimation_sub = n.subscribe("Loc_Estimation",1,&EstimationCallback);  // estimation of beacon localisation in sensor frame
    ros::Subscriber parameter_sub = n.subscribe("Loc_parameter",1,&ParameterCallback);     // mahalanobis threshold
    ros::Subscriber pose_sub = n.subscribe("Pose",1,&PoseCallback);                        // position of the robot in map frame

    //********************************************************************
    // Definition of topic variable
    //********************************************************************
    geometry_msgs::Point32 beacon_mesure;         // publish the beacon localisation in sensor frame (x,y) last argument give the beacon identifier
    std_msgs::Empty empty_msg;                    // publish a request estimation
    std_msgs::Float32MultiArray obstacle_position;// publish the position of the obstacle in map frame
    geometry_msgs::Point32 reset_localisation;

    //********************************************************************
    // Rate of the main loop
    //********************************************************************
    ros::Rate loop_rate(40);  // full loop speed (Hz)

    //********************************************************************
    // Clean all screen before starting the loop
    //********************************************************************
    #ifdef OutputON
        printf("\033[2J");
    #endif /* OutputON */

    sleep(5);

    //********************************************************************
    // reset_localisation
    //********************************************************************
   char* pPath;
    pPath = getenv("TEAM");
    if(strcmp(pPath,"purple")==0){
        reset_localisation.x = 850.0;
        reset_localisation.y = 60.0;
        reset_localisation.z = 2.0;
    }else{
        reset_localisation.x = 1150.0;
        reset_localisation.y = 60.0;
        reset_localisation.z = 2.0;
    }

    reset_localisation_pub.publish(reset_localisation);

    //********************************************************************
    // LOOP
    //********************************************************************
	while(ros::ok()){
        //********************************************************************
        // update of the topic variable
        //********************************************************************
        #ifdef OutputON
            struct timeval tic, toc;  // handler of time start and end of a loop period
            gettimeofday(&tic, NULL); // start of a loop period
        #endif /* OutputON */

        if(fresh_scan == true){
            if(fresh_estimation == false){
                estimation_request_pub.publish(empty_msg);
            }
            if(fresh_estimation == true){
                #ifdef DisplayOn
                    beacon1_laserscan.ranges.clear();
                    beacon2_laserscan.ranges.clear();
                    beacon3_laserscan.ranges.clear();
                    obstacle_laserscan.ranges.clear();
		    beacon1_laserscan.intensities.clear();
                    beacon2_laserscan.intensities.clear();
                    beacon3_laserscan.intensities.clear();
                    obstacle_laserscan.intensities.clear();
                #endif /* DisplayOn */

                // ---------------For each beacon-----------------
                for(int i = 0 ; i < beacon_nb ; i++){
                    std::vector<Vector2f> valid_points;
                    valid_points.clear();

                    // fulfill valid points according to the mahalanobis threshold
                    for(int j = scan.scan_vec.size()-1 ; j>0  ; j--){
                        diff = scan.scan_vec[j] - beacon[i].y_hat;
                        diff_transpose = diff.transpose();
                        temp = beacon[i].sigma_maha*diff;
                        dist_maha = (diff_transpose*temp)(0,0);
                        if(sqrtf(dist_maha) < mahalanobis_threshold){
                            valid_points.push_back(scan.scan_vec[j]);
                            #ifdef DisplayOn
                                switch(i){
                                    case 0:
                                        beacon1_laserscan.ranges.push_back(sqrtf(powf(scan.scan_vec[j](1,0),2.0)+powf(scan.scan_vec[j](0,0),2.0))/1000.0);
                                        beacon1_laserscan.intensities.push_back(10);
                                        break;
                                    case 1:
                                        beacon2_laserscan.ranges.push_back(sqrtf(powf(scan.scan_vec[j](1,0),2.0)+powf(scan.scan_vec[j](0,0),2.0))/1000.0);
                                        beacon2_laserscan.intensities.push_back(20);
                                        break;
                                    case 2:
                                        beacon3_laserscan.ranges.push_back(sqrtf(powf(scan.scan_vec[j](1,0),2.0)+powf(scan.scan_vec[j](0,0),2.0))/1000.0);
                                        beacon3_laserscan.intensities.push_back(30);
                                        break;
                                    default:
                                        break;
                                }
                            #endif /* DisplayOn */
                        }else{
                            #ifdef DisplayOn
                                switch(i){
                                    case 0:
                                        beacon1_laserscan.ranges.push_back((float)100.0);
                                        beacon1_laserscan.intensities.push_back(0);
                                        break;
                                    case 1:
                                        beacon2_laserscan.ranges.push_back((float)100.0);
                                        beacon2_laserscan.intensities.push_back(0);
                                        break;
                                    case 2:
                                        beacon3_laserscan.ranges.push_back((float)100.0);
                                        beacon3_laserscan.intensities.push_back(0);
                                        break;
                                    default:
                                        break;
                                }
                            #endif /* DisplayOn */
                        }
                    }


                    #ifdef OutputON
                        int l = i + 14;
                        printf("\033[%i;0HFrom main : number of valid points for beacon %i = %i\033[K\n", l, i, (int)valid_points.size());
                    #endif /* OutputON */

                    //********************************************************************
                    // Algorithme de moindre carrée non linéaire (au moins trois points de calculs)
                    //********************************************************************
                    if(valid_points.size() > 2){
                        // init point (par point centrale + rayon)
                        float x_central = valid_points[valid_points.size()/2](0,0);
                        float y_central = valid_points[valid_points.size()/2](1,0);
                        float L_centre = sqrtf(powf(x_central,2.0) + powf(y_central,2.0)) + Rayon ;
                        float theta_centre = atanf(y_central/x_central);
                        Vector2f X_current; Vector2f X_prec;
                        X_current << L_centre*cosf(theta_centre), L_centre*sinf(theta_centre);

                        // init gradient
                        Vector2f G_current; Vector2f G_prec;
                        G_current = CalculateGradient(X_current, valid_points);

                        // definition direction et jacobien
                        Vector2f direction;
                        MatrixXf jacobian(valid_points.size(),2);

                        //definition de variable
                        float criteria = 100.0;
                        float diffG = 100.0;
                        float diffX = 100.0;
                        int iter = 0;
                        int result = 8;

                        // LOOP
                        while(result = StopTest(diffG,diffX,iter) == 0){
                            X_prec(0,0) = X_current(0,0); X_prec(1,0) = X_current(1,0);
                            G_prec(0,0) = G_current(0,0); G_prec(1,0) = G_current(1,0);
                            //calcul de la direction de descente
                            jacobian = CalculateJacobian(X_current, valid_points);
                            MatrixXf jacobian_t(2,valid_points.size());  jacobian_t = jacobian.transpose();
                            MatrixXf JTJ(2,2);                           JTJ = jacobian_t*jacobian;
                            MatrixXf JTJI(2,2);                          JTJI = JTJ.inverse();
                            G_current = CalculateGradient(X_current, valid_points);
                            direction = JTJI*G_current;
                            direction = (-1.0)*direction;
                            //calcul du pas
                            float alpha = 1;
                            while(CalculateCriteria((X_current+(alpha*direction)),valid_points) > CalculateCriteria(X_current,valid_points) + ((Armijo_const*alpha)*(G_current.transpose()*direction))(0,0)){
                                alpha *= beta;
                            }
                            // Maj X_current
                            X_current = X_current+(alpha*direction);
                            diffG = CalculateDiffG(X_current, X_prec, valid_points);
                            diffX = CalculateDiffX(X_current, X_prec);
                            iter++; criteria = CalculateCriteria(X_current, valid_points);
                        }

                        diffG = CalculateDiffG(X_current, X_prec, valid_points);
                        diffX = CalculateDiffX(X_current, X_prec);
                        result = StopTest(diffG,diffX,iter);

                        beacon_mesure.x = X_current(0,0);
                        beacon_mesure.y = X_current(1,0);
                        beacon_mesure.z = (float)i;

                        beacon_mesure_pub.publish(beacon_mesure);

                        #ifdef OutputON
                            int line = i + 18;
                            printf("\033[%i;0HFrom main : cluster value of beacon %i = [ %f, %f ]\033[K\n", line, i, beacon_mesure.x, beacon_mesure.y);
                        #endif /* OutputON */
                    }

                    valid_points.clear();
                }

                // ---------------Find obstacles-----------------
                std::vector<Vector2f> obstacle_points;
                obstacle_points.clear();

                for(int j =  scan.scan_vec.size()-1 ; j>0 ; j--){
                    float pose_x_R0 = pose_x + cosf(pose_theta)*(scan.scan_vec[j](0,0)-x_hokuyo) - sinf(pose_theta)*(scan.scan_vec[j](1,0)-y_hokuyo);
                    float pose_y_R0 = pose_y + sinf(pose_theta)*(scan.scan_vec[j](0,0)-x_hokuyo) + cosf(pose_theta)*(scan.scan_vec[j](1,0)-y_hokuyo);
                    if((pose_x_R0 < largeur)&&(pose_y_R0 < longueur)){
                        Vector2f point;
                        point(0,0) = pose_x_R0;
                        point(1,0) = pose_y_R0;
                        obstacle_points.push_back(point);
                        #ifdef DisplayOn
                            obstacle_laserscan.ranges.push_back(sqrtf(powf(scan.scan_vec[j](1,0),2.0)+powf(scan.scan_vec[j](0,0),2.0))/1000.0);
                            obstacle_laserscan.intensities.push_back(50);
                        #endif // DisplayOn
                    }else{
                        #ifdef DisplayOn
                            obstacle_laserscan.ranges.push_back((float)100.0);
                            obstacle_laserscan.intensities.push_back(0);
                        #endif // DisplayOn
                    }
                }

                #ifdef OutputON
                    printf("\033[%5;0HFrom main : number of obstacle = %i\033[K\n", (int)obstacle_points.size());
                #endif /* OutputON */

                /*if(!obstacle_points.empty()){
                    obstacle_position.layout.data_offset = 0;
                    obstacle_position.layout.dim[0].label = "point";
                    obstacle_position.layout.dim[0].size = int(obstacle_points.size());
                    obstacle_position.layout.dim[0].stride = 2*int(obstacle_points.size());
                    obstacle_position.layout.dim[1].label = "channel";
                    obstacle_position.layout.dim[1].size = 2;
                    obstacle_position.layout.dim[1].stride = 2;

                    for(int k = 0 ; k < obstacle_points.size() ; k++){
                        obstacle_position.data[k,0] = obstacle_points[k](0,0);
                        obstacle_position.data[k,1] = obstacle_points[k](1,0);
                    }
                    obstacle_position_pub.publish(obstacle_position);
                }*/

                #ifdef DisplayOn
                    beacon1_laserscan_pub.publish(beacon1_laserscan);
                    beacon2_laserscan_pub.publish(beacon2_laserscan);
                    beacon3_laserscan_pub.publish(beacon3_laserscan);
                    obstacle_laserscan_pub.publish(obstacle_laserscan);
                #endif /* DisplayOn */

                fresh_estimation = false; fresh_scan = false;
            }
        }

        ros::spinOnce();
        loop_rate.sleep();

        #ifdef DisplayOn
            main_cnt++;
            gettimeofday(&toc, NULL);
            diffTimeCndD(tic, toc, "MAIN", 1);
            printf("\033[2;0HFrom main : main : %i\033[K\n", main_cnt); //
            printf("\033[3;0HFrom main : bool estimation_state : %i\033[K\n", fresh_estimation); //
            printf("\033[4;0HFrom main : bool scan_state : %i\033[K\n", fresh_scan);
        #endif /* DisplayOn */
	}
	return 0;
}

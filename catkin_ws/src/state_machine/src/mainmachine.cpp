//C++ Libraries
#include<iostream>
#include<stdlib.h>
#include<vector>
#include<string>
#include<cmath>

#include "StateMachine.h"

using namespace std;

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#define Periode 0.01
#define PasDeplacement 10
#define TFinPoisson 75 //en sec
#define nbPhotos 5

/*class Listener {
public : geometry_msgs::Pose2D last_msg_Pose2D;
public :
    // ?? _msgs:: ?? last_msg_ ?? ?;
    bool test_POSE2d;
    // bool test_ ??;
    void Callback_LaserScan(const geometry_msgs::Pose2D& msg) {last_msg_LaserScan = msg; test_Pose2D = true;};
    void Callback_Pose(const nav_msgs::Odometry& msg) {last_msg_Pose = msg; test_Pose = true;};
};


Listener listener_Depart;
Listener listener_Pose;
Listener listener_Poisson;
Listener listener_motion;
*/
;
geometry_msg::Point32 pos;

void CapteurFront_Callback (const std_msgs::Int16& cmd_msg){
    frontWarning=cmd_msg.data;
}

void CapteurBack_Callback (const std_msgs::Int16& cmd_msg){
    backWarning=cmd_msg.data;
}

void Pose_Callback (const geometry_msg::Point32& cmd_msg){
    pos=cmd_msg.data;
}

void Poisson_Callback (const geometry_msg::Point32& cmd_msg){
    //A remplir
    poisson=1;
}


//Config ROS
ros::init(argc, argv, "display_node");
ros::NodeHandle nh;
ros::Subscriber sub_CapteurFront = nh.subscribe("capteurFront", 1, CapteurFront_Callback);
ros::Subscriber sub_CapteurBack = nh.subscribe("capteurBack", 1, CapteurBack_Callback);
ros::Subscriber sub_Pose = nh.subscribe("Pose", 1, Pose_Callback);
ros::Subscriber sub_Poisson = nh.subscribe("Poisson", 1, Poisson_Callback);

ros::Publisher pubphoto = nh.advertise<std_msgs::Empty>("Photo", 1000);
ros::Publisher pubcons = nh.advertise<geometry_msg::Point32>("PointCons", 1000);
ros::Publisher pubwavcons = nh.advertise<std_msgs::Point32>("WavConsPosition", 1000);
ros::Publisher pubbras = nh.advertise<geometry_msgs::Twist>("UarmCommand", 1000);
ros::Publisher pubpince = nh.advertise<std_msgs::String>("FrontPlierCommand", 1000);
ros::Publisher pubpincebras = nh.advertise<std_msgs::Int16>("GripperCommand", 1000);
ros::Publisher pubparasol = nh.advertise<std_msgs::Empty>("ParasolCommand", 1000);

geometry_msg::Point32 cmd_Pose;
geometry_msg::Twist cmd_Bras;


ros::Rate loop_rate(100);

//Configuration environment
char* pPath;
pPath = getenv("TEAM");
if(strcmp(pPath,"purple")==0){


// Machine déterministe (plan A)
void State100(State* S) {

    cout << "state100" << endl;
    cout << "pret a partir!" << endl; // Etat initial on attend le signal de depart
    if (Depart) {
         {
            ros::Time begin = ros::Time::now(); // On initialise le temps de depart
            ros::Time last_state = begin; // A chaque noeud on enregistre le temps
            S->setTransition("depart", true); // On modifie les conditions des transitions içi
        }
    }
}

void State150(State* S){
    cout << "state150" << endl;
    cout << "Sortie de la zone d'obstacle!" << endl;
    
    pubpince.publish("depart");
    //Avancée du robot en L, Thêta
    cmd_Pose.x = PasDeplacement;
    cmd_Pose.y = 10;
    cmd_Pose.z = 1;
    pubcons.publish(cmd_Pose);
    ros::Duration(Periode).sleep();
    cmd_Pose.x = PasDeplacement;
    cmd_Pose.y = 0;
    cmd_Pose.z = 1;
    for (int i=0; i<14; i++)
    {
        pubcons.publish(cmd_Pose);
        ros::Duration(Periode).sleep(); // Attend 10ms avant la nouvelle boucle
    }
    S->setTransition("reach150", true); // On modifie les conditions des transitions içi
}

void State200(State* S) {
    cout << "state200" << endl;
    cout << "Poussée premiers cubes de sable!" << endl;

    // On commande le robot vers les cubes
    cmd_Pose.x = PasDeplacement;
    cmd_Pose.y = -10;
    cmd_Pose.z = 1;
    pubcom.publish(cmd_Pose);
    ros::Duration(Periode).sleep();
    
    cmd_Pose.x = PasDeplacement;
    cmd_Pose.y = 0;
    cmd_Pose.z = 1;
    for (int i=0; i<60; i++)
    {
        pubcons.publish(cmd_Pose);
        ros::Duration(Periode).sleep(); // Attend 10ms avant la nouvelle boucle
    }
    
    //sortie de la zone d'obstacle
    cmd_Pose.x = -PasDeplacement;
    cmd_Pose.y = 0;
    cmd_Pose.z = 1;
    for (int i=0; i<60; i++)
    {
        pubcons.publish(cmd_Pose);
        ros::Duration(Periode).sleep(); // Attend 10ms avant la nouvelle boucle
    }
    S->setTransition("reach200");
}

void State250(State* S) {
    cout << "state250" << endl;
    cout << "ruée vers le sable!" << endl;

    // On commande le robot vers la dune
    cmd_Pose.x = 400;
    cmd_Pose.y = 1000;
    cmd_Pose.z = 0;
    pubcwavcons.publish(cmd_Pose);
    
    if (350<pos.x<450 && 950<pos.y<1050){
        S->setTransition("reach250", true); // On modifie les conditions des transitions içi
    }
}

void State300(State* S) {

    cout << "state300" << endl;
    cout << "approche lente et ensuite brutale pour la chute des cubes" << endl;
    // Approche lente en L theta pour se mettre en position
    cmd_Pose.x = PasDeplacement ;
    cmd_Pose.y = (3.14-pose.theta)*180/3.14;
    cmd_Pose.z = 1;
    pubcons.publish(cmd_Pose);
    ros::Duration(Periode).sleep();
    /
    cmd_Pose.x = PasDeplacement ;
    cmd_Pose.y = 0;
    cmd_Pose.z = 1;
    for (int i=0; i<19; i++)
    {
        pubcons.publish(cmd_Pose);
        ros::Duration(Periode).sleep(); // Attend Periode en s avant la nouvelle boucle
    }
    //Fermeture max des pinces pour éviter des dégradations
    pubpince.publish("closeMax");
    //Placement face au cube (vue de coté) et déplacement pour les faire tomber
    cmd_Pose.x = PasDeplacement ;
    cmd_Pose.y = (2*3.14/3-pose.theta)*180/3.14;
    cmd_Pose.z = 1;
    pubcons.publish(cmd_Pose);
    ros::Duration(Periode).sleep();
    //déplacement une fois le positionnement terminé
    cmd_Pose.x = PasDeplacement ;
    cmd_Pose.y = 0;
    cmd_Pose.z = 1;
    for (int i=0; i<19; i++)
    {
        pubcons.publish(cmd_Posee);
        ros::Duration(Periode).sleep(); // Attend Periode en s avant la nouvelle boucle
    }
    S->setTransition("reach300", true); // On modifie les conditions des transitions içi
}
void State400(State* S) {
    cout << "state400" << endl;
    cout << "ramassage" << endl;
    // On recule après avoir fait tomber les blocs avec le bon angle
    cmd_Pose.x = -PasDeplacement ;
    cmd_Pose.y = (2*3.14/3-pose.theta)*180/3.14;
    cmd_Pose.z = 1;
    pubcons.publish(cmd_Pose);
    ros::Duration(Periode).sleep();
    
    cmd_Pose.x = -PasDeplacement ;
    cmd_Pose.y = 0;
    cmd_Pose.z = 1;
    
    int fin = (pos.x-1100)/PasDeplacement;
    for (int i=0; i<fin; i++)
    {
        pubcons.publish(cmd_Pose);
        ros::Duration(Periode).sleep(); // Attend Periode en s avant la nouvelle boucle
    }
    //On ouvre les pinces
    pubpince.publish("open");
    
    // On avance le robot jusqu'à y=1500
    cmd_Pose.x = PasDeplacement ;
    cmd_Pose.y = 0;
    cmd_Pose.z = 1;
    fin = (1500-pos.y)/PasDeplacement
    for (int i=0; i<fin; i++)
    {
        pubcons.publish(cmd_Pose);
        ros::Duration(Periode).sleep(); // Attend Periode en s avant la nouvelle boucle
    }
    //Fermeture des pinces
    pubpince.publish("close");
    S->setTransition("reach400", true); // On modifie les conditions des transitions içi
}
void State500(State* S) {
    cout << "state500" << endl;
    cout << "depart zone d'obstacle" << endl;
    // recule jusqu'à que x soit égale à 450
    cmd_Pose.x = -PasDeplacement ;
    cmd_Pose.y = (3.14-pose.theta)*180/3.14;
    cmd_Pose.z = 1;
    pubcom.publish(cmd_Pose);// On publie la commande
    ros::Duration(Periode).sleep();
    
    cmd_Pose.x = -PasDeplacement ;
    cmd_Pose.y = 0;
    cmd_Pose.z = 1;
    
    ifin = (pos.x-450)/PasDeplacement;
    for (int i=0; i<fin; i++)
    {
        pubcons.publish(cmd_Pose);
        ros::Duration(Periode).sleep(); // Attend Periode en s avant la nouvelle boucle
    }

    S->setTransition("reach500", true); // On modifie les conditions des transitions içi
    }
}
void State600(State* S) {
    cout << "state600" << endl;
    cout << "direction les portes" << endl;
    //Utilisation du wavcons pour aller en direction des portes
    cmd_Pose.x = 450 ;
    cmd_Pose.y = 450;
    cmd_Pose.z = 0;
    pubwavcons.publish(cmd_Pose);// On publie la commande

    if (425<pos.x<475 && 425<pos.y<475){
        S->setTransition("reach600", true); // On modifie les conditions des transitions içi
    }
}
void State700(State* S) {
    cout << "state700" << endl;
    cout << "fermeture des portes" << endl;
    // On s'oriente afin de fermer les portes avec le dos du robot
    cmd_Pose.x = 0 ;
    cmd_Pose.y = -pose.theta*180/3.14;
    cmd_Pose.z = 1;
    pubcom.publish(cmd_Pose);// On publie la commande
    ros::Duration(Periode).sleep();
    //On recule
    cmd_Pose.x = -PasDeplacement ;
    cmd_Pose.y = 0;
    cmd_Pose.z = 1;
    
    fin = (pos.x)/PasDeplacement;
    for (int i=0; i<fin; i++)
    {
        pubcons.publish(cmd_Pose);
        ros::Duration(Periode).sleep(); // Attend Periode en s avant la nouvelle boucle
    }
    
    //départ zone d'obstacle
    cmd_Pose.x = PasDeplacement ;
    cmd_Pose.y = 0;
    cmd_Pose.z = 1;
    
    fin = (pos.x-300)/PasDeplacement;
    for (int i=0; i<fin; i++)
    {
        pubcons.publish(cmd_Pose);
        ros::Duration(Periode).sleep(); // Attend Periode en s avant la nouvelle boucle
    }
    
    S->setTransition("reach700", true); // On modifie les conditions des transitions içi
}

void State800(State* S) {
    cout << "state800" << endl;
    cout << "direction la zone de depot des blocs" << endl;
    cmd_Pose.x = 1100;
    cmd_Pose.y = 900;
    cmd_Pose.z = 0;
    pubwavcons.publish(cmd_Pose);

    if (1075<pos.x<1125 && 875<pos.y<925){
        S->setTransition("reach800", true); // On modifie les conditions des transitions içi
    }
}

void State900(State* S) {
    cout << "state900" << endl;
    cout << "tourne, avance puis dépose blocs" << endl;
    cmd_Pose.x = PasDeplacement ;
    cmd_Pose.y = (3.14/2-pose.theta)*180/3.14;
    cmd_Pose.z = 1;
    pubcom.publish(cmd);// On publie la commande
    ros::Duration(Periode).sleep();
    //On avance jusqu'à y=1300
    cmd_Pose.x = PasDeplacement ;
    cmd_Pose.y = 0;
    cmd_Pose.z = 1;
    
    fin = (1300-pos.y)/PasDeplacement;
    for (int i=0; i<fin; i++)
    {
        pubcons.publish(cmd_Pose);
        ros::Duration(Periode).sleep(); // Attend Periode en s avant la nouvelle boucle
    }

    //On ouvre les pinces
    pubpince.publish("open");
    
    //On recule jusqu'à y=900
    cmd_Pose.x = -PasDeplacement ;
    cmd_Pose.y = 0;
    cmd_Pose.z = 1;
    
    fin = (pos.y-900)/PasDeplacement;
    for (int i=0; i<fin; i++)
    {
        pubcons.publish(cmd_Pose);
        ros::Duration(Periode).sleep(); // Attend Periode en s avant la nouvelle boucle
    }
    //Fermeture des pinces
    pubpince.publish("close");
    
    S->setTransition("reach900", true); // On modifie les conditions des transitions içi
    }
}
void State1000(State* S) {
    cout << "state1000" << endl;
    cout << "Récupération coquillage et déplacement vers les poissons" << endl;
    cmd_Pose.x = 0 ;
    cmd_Pose.y = (-3.14/6-pose.theta)*180/3.14;
    cmd_Pose.z = 1;
    pubcons.publish(cmd_Pose);
    ros::Duration(Periode).sleep(); // Attend Periode en s avant la nouvelle boucle

    //Ouverture des pinces
    pubpince.publish("open");
    
    //1er coquillage
    cmd_Pose.x = 1250;
    cmd_Pose.y = 700;
    cmd_Pose.z = 0;
    pubwavcons.publish(cmd_Pose);
    if (1225<pos.x<1275 && 675<pos.y<725){
        //2e coquillage
        cmd_Pose.x = 1250;
        cmd_Pose.y = 300;
        cmd_Pose.z = 0;
        pubwavcons.publish(cmd_Pose);
        if (1225<pos.x<1275 && 275<pos.y<325){
            cmd_Pose.x = PasDeplacement ;
            cmd_Pose.y = 0;
            cmd_Pose.z = 1;
            fin = (pos.y-200)/PasDeplacement;
            for (int i=0; i<fin; i++)
            {
                pubcons.publish(cmd_Pose);
                ros::Duration(Periode).sleep(); // Attend Periode en s avant la nouvelle boucle
            }
            cmd_Pose.x = 0 ;
            cmd_Pose.y = -pose.theta*180/3.14;
            cmd_Pose.z = 1;
            pubcons.publish(cmd_Pose);
            ros::Duration(Periode).sleep(); // Attend Periode en s avant la nouvelle boucle
            //3e coquillage
            cmd_Pose.x = PasDeplacement ;
            cmd_Pose.y = 0;
            cmd_Pose.z = 1;
            fin = (1650-pos.x)/PasDeplacement;
            for (int i=0; i<fin; i++)
            {
                pubcons.publish(cmd_Pose);
                ros::Duration(Periode).sleep(); // Attend Periode en s avant la nouvelle boucle
            }
            //Fermeture des pinces
            pubpince.publish("close");
            
            S->setTransition("reach1000", true); // On modifie les conditions des transitions içi
        }
    }
}

void State1100(State* S) {
    cout << "state1100" << endl;
    cout << "approche de l'aquarium" << endl;
    
    //Tourne à pi/5 et avance
    cmd_Pose.x = 0 ;
    cmd_Pose.y = (3.14/5-pose.theta)*180/3.14;
    cmd_Pose.z = 1;
    pubcons.publish(cmd_Pose);
    ros::Duration(Periode).sleep(); // Attend Periode en s avant la nouvelle boucle

    cmd_Pose.x = PasDeplacement ;
    cmd_Pose.y = 0;
    cmd_Pose.z = 1;
    fin = (1850-pos.x)/PasDeplacement;
    for (int i=0; i<fin; i++)
    {
        pubcons.publish(cmd_Pose);
        ros::Duration(Periode).sleep(); // Attend Periode en s avant la nouvelle boucle
    }
    
    //Tourne à 4pi/5 et avance
    cmd_Pose.x = 0 ;
    cmd_Pose.y = (4*3.14/5-pose.theta)*180/3.14;
    cmd_Pose.z = 1;
    pubcons.publish(cmd_Pose);
    ros::Duration(Periode).sleep(); // Attend Periode en s avant la nouvelle boucle

    cmd_Pose.x = PasDeplacement ;
    cmd_Pose.y = 0;
    cmd_Pose.z = 1;
    fin = (500-pos.y)/PasDeplacement;
    for (int i=0; i<fin; i++)
    {
        pubcons.publish(cmd_Pose);
        ros::Duration(Periode).sleep(); // Attend Periode en s avant la nouvelle boucle
    }
    
     //Tourne à pi/2 et en position pour la recherche de poissons
    cmd_Pose.x = 0 ;
    cmd_Pose.y = (3.14/2-pose.theta)*180/3.14;
    cmd_Pose.z = 1;
    pubcons.publish(cmd_Pose);
    ros::Duration(Periode).sleep(); // Attend Periode en s avant la nouvelle boucle

    cmd_Bras.linear.x = 150 ;
    cmd_Bras.linear.y = 0;
    cmd_Bras.linear.z = 130;
    cmd_Bras.angular.x = 90;
    cmd_Bras.angular.y = 0;
    cmd_Bras.angular.z = 0;
    pubbras.publish(cmd_Bras);
    
    S->setTransition("reach1100", true); // On modifie les conditions des transitions içi
}

void State1200(State* S) {
    int sens = 1;
    cout << "state1200" << endl;
    cout << "cherche poissons" << endl;
    
    while (!poisson && ros::Time::now()-begin<TFinPoisson){
        for (int i=0; i<nbPhotos; i++)
        {
            cmd_Pose.x = (-1)*sens*PasDeplacement ;
            cmd_Pose.y = 0;
            cmd_Pose.z = 1;
            fin = (100)/PasDeplacement; //avancé à définir selon les photos que l'on veut
            for (int i=0; i<fin; i++)
            {
                pubcons.publish(cmd_Pose);
                ros::Duration(Periode).sleep(); // Attend Periode en s avant la nouvelle boucle
            }
            pubphoto.publish(1);
        }
        sens=-sens;
    }
    if (poisson){
        S->setTransition("reachpoisson", true); // On modifie les conditions des transitions içi
    }
    else {
        S->setTransition("reach1200", true);
    }
    
}
void State1210(State* S) {
    cout << "state1110" << endl;
    cout << "ramasse poissons" << endl;
    // Programme de ramassage poisson
    //ouverture pince
    pubpincebras.publish(0);
    //descente bras
    cmd_Bras.linear.x = xBras ;
    cmd_Bras.linear.y = yBras;
    cmd_Bras.linear.z = 0; //à déterminer par test
    cmd_Bras.angular.x = orientation;
    cmd_Bras.angular.y = 0;
    cmd_Bras.angular.z = 0;
    pubbras.publish(cmd_Bras);
    ros::Duration(3).sleep();
    //fermeture pince
    pubpincebras.publish(156);
    //remonté bras
    cmd_Bras.linear.x = xBras ;
    cmd_Bras.linear.y = yBras;
    cmd_Bras.linear.z = 130; 
    cmd_Bras.angular.x = orientation;
    cmd_Bras.angular.y = 0;
    cmd_Bras.angular.z = 0;
    pubbras.publish(cmd_Bras);
    ros::Duration(3).sleep();
    
    S->setTransition("getpoisson", true); // On modifie les conditions des transitions içi
}
void State1220(State* S) {
    cout << "state1220" << endl;
    cout << "apporte poisson" << endl;
    // On va vers le filet
    cmd_Pose.x = PasDeplacement ;
    cmd_Pose.y = 0;
    cmd_Pose.z = 1;
    fin = (1200-pos.y)/PasDeplacement;
    for (int i=0; i<fin; i++)
    {
        pubcons.publish(cmd_Pose);
        ros::Duration(Periode).sleep(); // Attend Periode en s avant la nouvelle boucle
    }
    S->setTransition("reachfilet ", true); // On modifie les conditions des transitions içi
}
void State1230(State* S) {

    cout << "state1230" << endl;
    cout << "depose poisson" << endl;
    // Programme depose poisson
    //descente bras
    cmd_Bras.linear.x = xBras ;
    cmd_Bras.linear.y = yBras;
    cmd_Bras.linear.z = 0; //à déterminer par test
    cmd_Bras.angular.x = orientation;
    cmd_Bras.angular.y = 0;
    cmd_Bras.angular.z = 0;
    pubbras.publish(cmd_Bras);
    ros::Duration(3).sleep();
    //fouverture pince
    pubpincebras.publish(0);
    //remonté bras
    cmd_Bras.linear.x = xBras ;
    cmd_Bras.linear.y = yBras;
    cmd_Bras.linear.z = 130; 
    cmd_Bras.angular.x = orientation;
    cmd_Bras.angular.y = 0;
    cmd_Bras.angular.z = 0;
    pubbras.publish(cmd_Bras);
    ros::Duration(3).sleep();
    
    nb_Poissons++;
    if (nb_Poissons==4){
        S->setTransition("finilapeche", true);
    }
    else {
        S->setTransition("poissondepose", true);
    }
}
void State1240(State* S) {
    cout << "state1200" << endl;
    cout << "retour a l'aquarium" << endl;
    cmd_Pose.x = -PasDeplacement ;
    cmd_Pose.y = 0;
    cmd_Pose.z = 1;
    fin = (pos.y-500)/PasDeplacement;
    for (int i=0; i<fin; i++)
    {
        pubcons.publish(cmd_Pose);
        ros::Duration(Periode).sleep(); // Attend Periode en s avant la nouvelle boucle
    }
    
    S->setTransition("pretapecher", true); // On modifie les conditions des transitions içi
}
void State1300(State* S) {
    cout << "state1300" << endl;
    cout << "sortie zone obstacle" << endl; // A voir si on ne peut pas le faire ne 1commande
    //Rotation pour s'écarter du bord
    cmd_Pose.x = 0;
    cmd_Pose.y = (3.14/2-pose.theta)*180/3.14;
    cmd_Pose.z = 1;
    pubcons.publish(cmd_Pose);
    ros::Duration(Periode).sleep(); // Attend Periode en s avant la nouvelle boucle
    //On recule
    cmd_Pose.x = -PasDeplacement ;
    cmd_Pose.y = 0;
    cmd_Pose.z = 1;
    fin = (pos.x-1700)/PasDeplacement;
    for (int i=0; i<fin; i++)
    {
        pubcons.publish(cmd_Pose);
        ros::Duration(Periode).sleep(); // Attend Periode en s avant la nouvelle boucle
    }
    //Rotation pour être en face du coquillage
    cmd_Pose.x = 0;
    cmd_Pose.z = 1;
    
    double theta = atan(double(pos.x-1450)/(pos.y-900))
    if (pos.y<=900){
        cmd.Pose.y = (theta + 3.14/2 - pos.theta)*180/3.14
    }
    else {
        cmd_Pose.y = (theta - 3.14/2 - pos.theta)*180/3.14;
    }

    pubcons.publish(cmd_Pose);
    ros::Duration(Periode).sleep(); // Attend Periode en s avant la nouvelle boucle
    
    //Ouverture des pinces
    pubpince.publish("open");
    
    //deplacement vers le coquillage (1450,900)
    cmd_Pose.x = 1450;
    cmd_Pose.y = 900;
    cmd_Pose.z = 0;
    pubwavcons.publish(cmd_Pose);
    if (1425<pos.x<1475 && 875<pos.y<925){
        //Fermeture des pinces
        pubpince.publish("close");
        S->setTransition("reach1300", true); // On modifie les conditions des transitions içi
    }
}
void State1400(State* S) {
    cout << "state1400" << endl;
    cout << "direction la serviette" << endl; // Voir commentaire state precedent
    cmd_Pose.x = 825;
    cmd_Pose.y = 275;
    cmd_Pose.z = 0;
    pubwavcons.publish(cmd_Pose);

    if (800<pos.x<850 && 250<pos.y<300){
        S->setTransition("reach1400", true); // On modifie les conditions des transitions içi
    }
}

void State1500(State* S) {
    cout << "state1500" << endl;
    cout << "vers l'intérieur de la serviette" << endl; // On va vers une position intermediare pour ne pas pousser les coquillages deposs
    //Rotation pour s'écarter du bord
    cmd_Pose.x = 0;
    cmd_Pose.y = (-3.14/2-pose.theta)*180/3.14;
    cmd_Pose.z = 1;
    pubcons.publish(cmd_Pose);
    ros::Duration(Periode).sleep(); // Attend Periode en s avant la nouvelle boucle
    //On avance
    cmd_Pose.x = PasDeplacement ;
    cmd_Pose.y = 0;
    cmd_Pose.z = 1;
    fin = (pos.y-25)/PasDeplacement;
    for (int i=0; i<fin; i++)
    {
        pubcons.publish(cmd_Pose);
        ros::Duration(Periode).sleep(); // Attend Periode en s avant la nouvelle boucle
    }
    
    S->setTransition("reach1500", true); // On modifie les conditions des transitions içi
}

void State1600(State* S) {
    cout << "state1600" << endl;
    cout << "ouverture Parasol" << endl;
    
    pubparasol.publish();
    
    cout << "C'est fini ;)" << endl;
}


int main() {
    int frontWarning, backWarning;
    int poisson = 0;
    int orientationPince, xBras, yBras;    
    int nb_Poissons = 0;
    // //BUILD STATE MACHINE// /// /// /// /

    depart du robot!
    // Les états deterministes
    State* S100 = new State(&State100);
    State* S150 = new State(&State150);
    State* S200 = new State(&State200);
    State* S250 = new State(&State250);
    State* S300 = new State(&State300);
    State* S400 = new State(&State400);
    State* S500 = new State(&State500);
    State* S600 = new State(&State600);
    State* S700 = new State(&State700);
    State* S800 = new State(&State800);
    State* S900 = new State(&State900);
    State* S1000 = new State(&State1000);
    State* S1100 = new State(&State1100);
    State* S1200 = new State(&State1200);
    State* S1210 = new State(&State1210);
    State* S1220 = new State(&State1220);
    State* S1230 = new State(&State1230);
    State* S1240 = new State(&State1240);
    State* S1300 = new State(&State1300);
    State* S1400 = new State(&State1400);
    State* S1500 = new State(&State1500);
    State* S1600 = new State(&State1600);
    State* S1700 = new State(&State1700);

    // On rajoute les transitions vers les états// /// //

    // Transition de depart du robot
    S100->addTransition("depart", S150);
    S150->addTransition("reach150", S200);
    S200->addTransition("reach200", S250);
    S250->addTransition("reach250", S300);
    S300->addTransition("reach300", S400);
    S400->addTransition("reach400", S500);
    S500->addTransition("reach500", S600);
    S600->addTransition("reach600", S700);
    S700->addTransition("reach700", S800);
    S800->addTransition("reach800", S900);
    S900->addTransition("reach900", S1000);
    S1000->addTransition("reach1000", S1100);
    S1100->addTransition("reach1000", S1200);
    S1200->addTransition("reach1000", S1300);

    S1200->addTransition("reachpoisson", S1210);
    S1210->addTransition("getpoisson", S1220);
    S1220->addTransition("reachfilet", S1230);
    S1230->addTransition("finilapeche", S1300);
    S1230->addTransition("poissondepose", S1240);
    S1240->addTransition("pretapecher", S1200);

    S1300->addTransition("reach1300", S1400);
    S1400->addTransition("reach1400", S1500);
    S1500->addTransition("reach1400", S1600);
    S1600->addTransition("reach1400", S1700);


    // Robot parti



    // On spécifie un nom pour la transition et un pointeur vers un état
    /*one->addTransition("one2three",three);
    two->addTransition("two2three",three);
    three->addTransition("three2one",one);*/
    // On instancie la machine d'état finis// //
    SM maMachine(one);

    // /// /// /// RUN STATE MACHINE// /// /// /// /// 
    for (int cnt = 0; cnt < 10; cnt++) {
        maMachine.run();
        // Ne pas oublier que les conditions associées aux transitions sont modifiées dans les fonctions pointées (au début)
        // C'est impératif pour changer d'état...
        // Ainsi la fonction de l'état courant est exécutée autant de fois que maMachine.run() avant passage à TRUE d'une condition
        // La classe SM se charge de vérifier les transitions et d'effectuer tous les changements nécessaires à chaque appel de maMachine.run()
        // Avant l'exécution de la fonction cible
    }
    delete one; delete two; delete three; // On désalloue tout à la fin !
    return (0);
}

//C++ Libraries
#include<iostream>
#include<stdlib.h>
#include<vector>
#include<string>

#include "StateMachine.h"

using namespace std;


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

//Config ROS
ros::init(argc, argv, "display_node");
ros::NodeHandle nh;
ros::Subscriber sub_CapteurFront = nh.subscribe("capteurFront", 1, CapteurFront_Callback);
ros::Subscriber sub_CapteurBack = nh.subscribe("capteurBack", 1, CapteurBack_Callback);
ros::Subscriber sub_Pose = nh.subscribe("Pose", 1, Pose_Callback);

ros::Publisher pubcons = nh.advertise<geometry_msg::Point32>("PointCons", 1000);
ros::Publisher pubwavcons = nh.advertise<std_msgs::Point32>("WavConsPosition", 1000);
ros::Publisher pubbras = nh.advertise<geometry_msgs::Twist>("UarmCommand", 1000);
ros::Publisher pubpince = nh.advertise<std_msgs::String>("FrontPlierCommand", 1000);
ros::Publisher pubpincebras = nh.advertise<std_msgs::Int16>("GripperCommand", 1000);
ros::Publisher pubparasol = nh.advertise<std_msgs::Empty>("ParasolCommand", 1000);

geometry_msg::Point32 cmd_Pos;
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
    cmd_Pos.x = 10;
    cmd_Pos.y = 10;
    cmd_Pos.z = 1;
    pubcons.publish(cmd_Pos);
    cmd_Pos.x = 10;
    cmd_Pos.y = 0;
    cmd_Pos.z = 1;
    for (int i=0; i<14; i++)
    {
        pubcons.publish(cmd_Pos);
        ros::Duration(0.01).sleep(); // Attend 10ms avant la nouvelle boucle
    }
    S->setTransition("reach150", true); // On modifie les conditions des transitions içi
}

void State200(State* S) {
    cout << "state200" << endl;
    cout << "Poussée premiers cubes de sable!" << endl;

    // On commande le robot vers les cubes
    cmd_Pos.x = 10;
    cmd_Pos.y = -10;
    cmd_Pos.z = 1;
    pubcom.publish(cmd_Pos);
    cmd_Pos.x = 10;
    cmd_Pos.y = 0;
    cmd_Pos.z = 1;
    for (int i=0; i<60; i++)
    {
        pubcons.publish(cmd_Pos);
        ros::Duration(0.01).sleep(); // Attend 10ms avant la nouvelle boucle
    }
    
    //sortie de la zone d'obstacle
    cmd_Pos.x = -10;
    cmd_Pos.y = 0;
    cmd_Pos.z = 1;
    for (int i=0; i<60; i++)
    {
        pubcons.publish(cmd_Pos);
        ros::Duration(0.01).sleep(); // Attend 10ms avant la nouvelle boucle
    }
    S->setTransition("reach200");
}

void State250(State* S) {
    cout << "state250" << endl;
    cout << "ruée vers le sable!" << endl;

    // On commande le robot vers la dune
    cmd_Pose.x = 450;
    cmd.y = 130;
    cmd.z = 0
    pubcom.publish(cmd);

    poseatteinte = fasle
    while (!poseatteinte && (ros::Time::now() - last_state) < 10) { // Tant qu'on est pas arrivé et qu'on a pas fait 10s
        if (listener_motion.last_msg ?? ? ) { // Si le topic motion indique su'on est arrivé
            poseatteinte = true;
        }
        // On indique qu'on est arrivé
        if (listener_motion.last_msg ?? ? ) { // Si robot detecté
            ros::Time tempsetat = (ros::Time::now() - last_state); // Temps parcouru par l'état 200
            if (tempetat < 5 ) {S->setTransition("planC", true); } // Si on voit le robot avant les 5premières secondes (à modifier selon test)
            if (tempetat < 10 ) {S->setTransition("planB", true); } // Pareil mais 10
            // Peut etre faire un 4ème cas
        }
        ros::Duration(0.1).sleep(); // Attend 100ms avant la nouvelle boucle

    }
    S->setTransition("planA", true); // On modifie les conditions des transitions içi

}

void State300(State* S) {

    cout << "state300" << endl;
    cout << "approche lente" << endl;
    // Approche lente en L theta pour se rapprocher des blocs

    while (!poseatteinte && (ros::Time::now() - last_state) < 3) { // Tant que l'on est pas arrivé et durant au maximum 3s
        cmd.x = 2 ;
        cmd.y = 0.2;
        cmd.z = 1;
        pubcom.publish(cmd);
        poseatteinte = true // A remplir si on veut une condition d'arret non temporelle
        if (robot detecte) {
            last_state = ros::Time::now()
                         S->setTransition("robotdetecte300")
        }
    }

    S->setTransition("reach300", true); // On modifie les conditions des transitions içi

}
void State400(State* S) {
    cout << "state400" << endl;
    cout << "ramassage" << endl;
// Approche lente en L theta pour attrapper des blocs
    cmd.x = 400 ;
    cmd.y = 1.7;
    cmd.z = 1;
    pubcom.publish(cmd);// On publie la commande
    if "a completer" {


    // On commande le robot vers le blocsdeblocs
    cmd.x = 900;
    cmd.y = 2450;
    cmd.z = 0;
    pubcom.publish(cmd);
    }
    if "a completer" {
    S->setTransition("reach400", true); // On modifie les conditions des transitions içi
    }
}
void State500(State* S) {
    cout << "state500" << endl;
    cout << "tourne blocbloc" << endl;
    // Tourne sur nous meme avant de pousser le blocsde blocs
    cmd.x = 0 ;
    cmd.y = -1.7;
    cmd.z = 1;
    pubcom.publish(cmd);// On publie la commande

    if "a completer" {

    S->setTransition("reach500", true); // On modifie les conditions des transitions içi
    }
}
void State600(State* S) {
    cout << "state600" << endl;
    cout << "pousse blocbloc" << endl;
    // On pousse le bloc de blocs
    cmd.x = 400 ;
    cmd.y = -1.7;
    cmd.z = 1;
    pubcom.publish(cmd);// On publie la commande

    if "a completer" {
    S->setTransition("reach600", true); // On modifie les conditions des transitions içi
    }
}
void State700(State* S) {
    cout << "state700" << endl;
    cout << "reculage laisse blocs" << endl;
    // On recule afin de deposer les blocs dans la zone
    cmd.x = -300 ;
    cmd.y = -1.7;
    cmd.z = 1;
    pubcom.publish(cmd);// On publie la commande

    if "a completer" {

    S->setTransition("reach700", true); // On modifie les conditions des transitions içi
    }
}
void State800(State* S) {
    cout << "state800" << endl;
    cout << "vers coquillage" << endl;
    // On va vers coquillage
    cmd.x = 1150;
    cmd.y = 2450;
    cmd.z = 0;
    pubcom.publish(cmd);

    if "a completer" {

    S->setTransition("reach800", true); // On modifie les conditions des transitions içi
    }
}
void State900(State* S) {
    cout << "state900" << endl;
    cout << "tourne puis pousse coquillage" << endl;
    // On se met dans la bonne orientation avant de pousser le coquillage
    // On peut surement le faire en 1seule commande
    cmd.x = 0 ;
    cmd.y = -0.85;
    cmd.z = 1;
    pubcom.publish(cmd);// On publie la commande

    if "a completer" {
    cmd.x = 400;
    cmd.y = -0.85;
    cmd.z = 0;
    pubcom.publish(cmd);
    }
    if "a completer" {
    cmd.z = 1;
    pubcom.publish(cmd);


        S->setTransition("reach900", true); // On modifie les conditions des transitions içi
    }
}
void State1000(State* S) {
    cout << "state1000" << endl;
    cout << "tourne poisson" << endl;
    // On tourne pour se mettre le long des poissons
    cmd.x = 0 ;
    cmd.y = 1, 7;
    cmd.z = 1;
    pubcom.publish(cmd);// On publie la commande

    if "a completer" {

    S->setTransition("reach1000", true); // On modifie les conditions des transitions içi
    }
}
void State1100(State* S) {


    cout << "state1100" << endl;
    cout << "cherche poissons marche avant" << endl;
    // On cherche les poissons en marche arrière
    // Il faura peut etre activer le bras pour positionnner la caméra
    while (i < 150) { // On envoie des commandes de 2mm par 2mm en marche avant
        cmd.x = 2 ;
        cmd.y = 1.7;
        cmd.z = 1;
        pubcom.publish(cmd);
        i = i + 1;
        if (listener_Poisson) {
            S->setTransition("reachpoisson", true);
        }// On publie la commande
    }
    if "a completer" {
    S->setTransition("paspoisson", true); // On modifie les conditions des transitions içi
    }
}
void State1100bis(State* S) {
    cout << "state1100bis" << endl;
    cout << "cherche poissons marche arriere" << endl;
// Pareil en marche arrière
    while (i < 150) { // On envoie des commandes de 2mm par 2mm en marche arrière
        cmd.x = -2 ;
        cmd.y = 1.7;
        cmd.z = 1;
        pubcom.publish(cmd);
        i = i + 1;
        if (listener_Poisson) {
            S->setTransition("reachpoisson", true);
        }// On publie la commande
    }
    if "a completer" {
    S->setTransition("paspoisson", true); // On modifie les conditions des transitions içi
    }
}
void State1110(State* S) {
    cout << "state1110" << endl;
    cout << "ramasse poissons" << endl;
    // Programme de ramassage poisson
    cmdbras = "descendre";
    pubbras.publish(cmdbras);
    cmdbras = "serrrer";
    pubbras.publish(cmdbras);
    cmdbras = "monter";
    pubbras.publish(cmdbras);
    // Voir comment fonctionne le bras etc

    if "a completer" {
    S->setTransition("getpoisson", true); // On modifie les conditions des transitions içi
    }
}
void State1120(State* S) {
    cout << "state1120" << endl;
    cout << "apporte poisson" << endl;
    // On va vers le filet
    float32 y = listener_Pose.last_msg_Pose2D.y;
    cmd.x = 2000 - y ;
    cmd.y = 1.7;
    cmd.z = 1;
    pubcom.publish(cmd);
    if "a completer" {
    S->setTransition("reachfilet ", true); // On modifie les conditions des transitions içi
    }
}
void State1130(State* S) {

    cout << "state1130" << endl;
    cout << "depose poisson" << endl;
    // Programme depose poisson
    cmdbras = "descendre";
    pubbras.publish(cmdbras);
    cmdbras = "serrrer";
    pubbras.publish(cmdbras);
    cmdbras = "monter";
    pubbras.publish(cmdbras);
    nb_poissons ++;
    if "a completer" {
    S->setTransition("poissondepose ", true); // On modifie les conditions des transitions içi
    }
    if (nb_poissons == 4) {
        S->setTransition("finilapeche ", true);
    }
}
void State1200(State* S) {
    cout << "state1200" << endl;
    cout << "vers coquillage2" << endl;
    // On va vers coquillage2
    cmd.x = 1700;
    cmd.y = 2800;
    cmd.z = 0;
    pubcom.publish(cmd);

    if "a completer" {

    S->setTransition("reach1200", true); // On modifie les conditions des transitions içi
    }
}
void State1300(State* S) {
    cout << "state1300" << endl;
    cout << "tourne puis pousse coquillage2" << endl; // A voir si on ne peut pas le faire ne 1commande
    cmd.x = 0 ;
    cmd.y = 3.14;
    cmd.z = 1;

    if "a completer" {

    cmd.x = 500;
    cmd.y = 3.14;
    cmd.z = 0;
    pubcom.publish(cmd);
    }
    if "a completer" {
    S->setTransition("reach1300", true); // On modifie les conditions des transitions içi
    }
}
void State1400(State* S) {
    cout << "state1400" << endl;
    cout << "tourne puis recule" << endl; // Voir commentaire state precedent
    cmd.x = 0 ;
    cmd.y = -1.7;
    cmd.z = 1;

    if "a completer" {
    cmd.x = -300;
    cmd.y = -1.7;
    cmd.z = 0;
    pubcom.publish(cmd);
    }
    if "a completer" {
    S->setTransition("reach1400", true); // On modifie les conditions des transitions içi
    }
}
void State1500(State* S) {
    cout << "state1500" << endl;
    cout << "vers position intermediare" << endl; // On va vers une position intermediare pour ne pas pousser les coquillages deposs
    cmd.x = 900;
    cmd.y = 2450;
    cmd.z = 0;
    pubcom.publish(cmd);

    if ("a completer") {

        S->setTransition("reach1500", true); // On modifie les conditions des transitions içi
    }
}
void State1600(State* S) {
    cout << "state1600" << endl;
    cout << "vers les portes" << endl;
    // On se rapproche des portes
    cmd.x = 200;
    cmd.y = 2850;
    cmd.z = 0;
    pubcom.publish(cmd);

    if "a completer" {

    S->setTransition("reach1600", true); // On modifie les conditions des transitions içi
    }
}
void State1700(State* S) {
    cout << "state1700" << endl;
    cout << "tourne puis pousse portes" << endl; // A voir si on ne peut pas le faire ne 1commande
    cmd.x = 0 ;
    cmd.y = -1.7;
    cmd.z = 1;

    if "a completer" {

    cmd.x = 500;
    cmd.y = -1.7;
    cmd.z = 0;
    pubcom.publish(cmd);
    }
    cout << "c'est la fin" << endl;
    // Il reste à gérer le temps et les pauses!
}

// Plan B
void State301(State* S) {

}


int main() {
    int nb_poisson = 0;
    // //BUILD STATE MACHINE// /// /// /// /

    depart du robot!
    // Les états deterministes
    State* S100 = new State(&State100);
    State* S200 = new State(&State200);
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
    State* S1300 = new State(&State1300);
    State* S1400 = new State(&State1400);
    State* S1500 = new State(&State1500);
    State* S1600 = new State(&State1600);
    State* S1110 = new State(&State1110);
    State* S1120 = new State(&State1120);
    State* S1100bis = new State(&State1100bis);
    State* S1130 = new State(&State1130);
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

    S1100->addTransition("reachpoisson", S1110);
    S1100->addTransition("paspoisson", S1100bis);
    S1100bis->addTransition("reachpoisson", S1110);
    S1100bis->addTransition("paspoisson", S1100);
    S1110->addTransition("getpoisson", S1120);
    S1120->addTransition("reachfilet", S1130);
    S1130->addTransition("finilapeche", S1200);
    S1130->addTransition("poissondepose", S1100);

    S1200->addTransition("reach1200", S1300);
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

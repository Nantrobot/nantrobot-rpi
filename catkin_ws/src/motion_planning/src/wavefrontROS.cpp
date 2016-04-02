#include"motion_planning/wavefrontROS.h"

#define STSCORE 1000000000

using namespace std;

node::node(unsigned int i,unsigned int j,int s):ligne(i),colonne(j),score(s){
}

node::node(void):ligne(0),colonne(0),score(0){
}

wavefront::wavefront(void):size_h(0),size_l(0),start(),goal(),wayfound(false),matrix(NULL),file(){
}

wavefront::~wavefront(void){
    if(matrix != NULL){
        for (unsigned i=0; i < size_h; i++){
            delete[] matrix[i];
        }
        delete[] matrix;
    }
}


void wavefront::setgoal(unsigned i,unsigned j){
    goal.ligne=i;
    goal.colonne=j;
    goal.score=2;
    matrix[i][j]=goal.score;
}

void wavefront::setstart(unsigned i,unsigned j){
    start.ligne=i;
    start.colonne=j;
    start.score=STSCORE;
    matrix[i][j]=start.score;
}


void wavefront::readfile(const char* path){
    fstream fichier;
    fichier.open(path,ios::in);
    fichier >> size_h;
    fichier >> size_l;
    matrix= new int* [size_h];
    for(unsigned i=0;i<size_h;i++){
            matrix[i]=new int[size_l];
    }
    for(unsigned i=0;i<size_h;i++){
            for(unsigned j=0;j<size_l;j++){
                    if(!fichier.eof()){
                        char e;
                        fichier>>e;
                        matrix[i][j]=e-'0';
                        }
            }
    }
    fichier.close();
}


void wavefront::afficher_matrix(void){
    for(unsigned i=0;i<size_h;i++){
            for(unsigned j=0;j<size_l;j++){
                    switch(matrix[i][j]){
                    case STSCORE:
                        cout<<"S"<<" ";
                        break;
                    case 2:
                        cout<<"G"<<" ";
                        break;
                    default:
                        cout<<matrix[i][j]<<" ";
                    }
            }
            cout<<"\n";
    }
}

void wavefront::propagate_wave(void){
    file.push_back(goal);
    while((!file.empty()) && (!wayfound)){
        parse();
        pull();
    }
    if((wayfound)&&(!file.empty())){
            file.clear();
    }
}

void wavefront::pull(void){
    for(unsigned it=0;it<file.size();it++){
        file[it]=file[it+1];
    }
    file.pop_back();
}

void wavefront::parse(void){
    unsigned int i=file[0].ligne;
    unsigned int j=file[0].colonne;
    int currentscore=file[0].score;
    int tab[8][2]={{-1,-1},{-1,0},{-1,1},{0,-1},{0,1},{1,-1},{1,0},{1,1}};
    for(int k=0;k<8;k++){
        unsigned int ligne=i+tab[k][0];
        unsigned int colonne=j+tab[k][1];
        if((ligne>=0)&&(ligne<size_h)&&(colonne>=0)&&(colonne<size_l)){
            if(matrix[ligne][colonne]==0){
                    matrix[ligne][colonne] = currentscore + 1 ;
                    node new_node(ligne,colonne,matrix[ligne][colonne]);
                    file.push_back(new_node);
            }
            if((ligne==start.ligne)&&(colonne==start.colonne)&&(matrix[ligne][colonne]!=1)){
                wayfound=true;
            }
        }
    }
}


void wavefront::getmatrix(unsigned int i,unsigned int j){
    cout<<matrix[i][j]<<endl;
}
std::vector<node> wavefront::find_path(void){
    std::vector<node> liste;
    if(wayfound){
        node mini=start;
        bool pathfound=false;
        int tab[8][2]={{-1,-1},{-1,0},{-1,1},{0,-1},{0,1},{1,-1},{1,0},{1,1}};
        while(!pathfound){
            unsigned int i=mini.ligne;
            unsigned int j=mini.colonne;
            liste.push_back(mini);
            for(int k=0;k<8;k++){
                unsigned int ligne=i+tab[k][0];
                unsigned int colonne=j+tab[k][1];
                if((ligne>=0)&&(ligne<size_h)&&(colonne>=0)&&(colonne<size_l)){
                    int dist =((ligne-i)*(ligne-i))+((colonne-j)*(colonne-j));
                    if((matrix[ligne][colonne]<=mini.score)&&(matrix[ligne][colonne]!=1)&&(matrix[ligne][colonne]!=-1)&&(matrix[ligne][colonne]!=0)){
                        if(matrix[ligne][colonne]<mini.score){
                            mini.ligne=ligne;
                            mini.colonne=colonne;
                            mini.score=matrix[ligne][colonne];
                        }else if((matrix[ligne][colonne]==mini.score)&&(dist<2)){
                            mini.ligne=ligne;
                            mini.colonne=colonne;
                            mini.score=matrix[ligne][colonne];
                        }
                    }
                    if((ligne==goal.ligne)&&(colonne==goal.colonne)){
                        pathfound=true;
                        liste.push_back(goal);
                    }
                }
            }
            matrix[mini.ligne][mini.colonne]=-1;
        }
        matrix[goal.ligne][goal.colonne]=2;
    }else{
        cout<<"\n"<<"Pas de solutions"<<endl;
    }
    return(liste);
}

void wavefront::affichersol(const char * path){
    fstream fichier;
    fichier.open(path,ios::out);
    for(unsigned i=0;i<size_h;i++){
            for(unsigned j=0;j<size_l;j++){
                    switch(matrix[i][j]){
                    case STSCORE:
                        fichier<<"S"<<" ";
                        break;
                    case 2:
                        fichier<<"G"<<" ";
                        break;
                    case -1:
                        fichier<<"+"<<" ";
                        break;
                    default:
                        if(matrix[i][j]==1){
                            fichier<<"x"<<" ";
                        }else{
                            fichier<<"."<<" ";
                    }
                }
            }
            fichier<<"\n";
    }
    fichier.close();
}

void wavefront::clearmatrix(void){
    wayfound=false;
    for(unsigned i=0;i<size_h;i++){
            for(unsigned j=0;j<size_l;j++){
                    if(matrix[i][j]!=1){
                            matrix[i][j]=0;
                    }
            }
    }
}

void wavefront::addobstacle(unsigned int i,unsigned int j){
    matrix[i][j]=1;
}


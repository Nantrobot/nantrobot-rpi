#include<iostream>
#include<vector>
#include<string>

#include "StateMachine.h"

using namespace std;

State::State(void(*ptfoncaddress)(State*)):names(),conditions(),followState(),l(0),ptfonc(ptfoncaddress){
}

State::~State(void){
}

void State::addTransition(string name,State* pt){
    bool isInVect=false;
    for(int it=0;it<l;it++){
        if(names[it]==name){
            isInVect=true;
        }
    }
    if(!isInVect){
        names.push_back(name);
        bool b=false;
        conditions.push_back(b);
        followState.push_back(pt);
        l++;
    }else{
        cout<<"Error : the desired transition already exists"<<endl;
    }
}

void State::setTransition(string name,bool b){
    int i=-1;
    l=names.size();
    int it=0;
   while (i==(-1)&&(it<l)){
        if(name==names[it]){
                i=it;
               }it++;
    }
    if(i!=-1){
        conditions[i]=b;
    }else{
        cout<<"Error : no transition with name "<<name<<endl;
    }
}


SM::SM(State* init):CurrentStatePointer(init){
}

SM::~SM(void){
}

void SM::run(void){
    ////////////CHECK IF THERE IS A TRANSITION//////////////////////////
    int s=CurrentStatePointer->l;
    int it=0;
    bool noTransition=true;
    while((it<s)&&(noTransition)){
            if((CurrentStatePointer->conditions)[it]){
                //////////////////IF YES REFRESH STATE///////////////////
                (CurrentStatePointer->conditions[it])=false;//reset de la condition
                CurrentStatePointer=(CurrentStatePointer->followState)[it];
                noTransition=false;//Achtung ! Les transitions doivent êtres classées par ordre de priorité lors de la construction
            }
            it++;
    }
    /////////////////////EXECUTE STATE USER-DEFINED FONCTION//////////////////////
    (CurrentStatePointer->ptfonc)(CurrentStatePointer);//On prend un pointeur sur l'état courant en argument pour
    //pouvoir modifier les attributs des états dans les procédures définies par l'utilisateur (les conditions notamment)
}


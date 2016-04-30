#ifndef STATEMACHINE_H_INCLUDED
#define STATEMACHINE_H_INCLUDED

#include<iostream>
#include<vector>
#include<string>

class State{
private:
    std::vector<std::string> names; //le vecteur stockant le nom des transitions
    std::vector<bool> conditions; //vecteur de booléeens pour passerd'un état à un autre
    std::vector<State*> followState; //les liaisons avec les autres états
    int l; //longueur des vecteurs
    State(const State &); //verrouillage du constructeur de recopie
    State & operator = (const State &); //verrouillage de l'opérateur d'affectation
public:
    State(void(*ptfoncaddress)(State*)); //Constructeur.il prend en argument l'adresse d'une fonction définie par l'utilisateur
    ~State(void); //Destructeur
    void(*ptfonc)(State*); //Un pointeur sur une fonction définie ultérieurement par l'utilisateur
    void addTransition(std::string name,State* pt); //méthode de construction des liaisons
    void setTransition(std::string name,bool b); //méthode pour activer une transition (mettre à "true" le booléen)

friend class SM;

};

class SM{
private:
    State* CurrentStatePointer; //Un pointeur sur l'état courant
    SM(const SM &); //verrouillage du constructeur de recopie
    SM & operator = (const SM &); //verrouillage de l'opérateur d'affectation

public:
    SM(State* init); //Constructeur initialisant la SM sur un état donné
    ~SM(void); //Destructeur
    void run(); //Il suffit d'appeler cette fonction après construction de la SM, dans une boucle, et tout fonctionne tout seul !

};

#endif // STATEMACHINE_H_INCLUDED

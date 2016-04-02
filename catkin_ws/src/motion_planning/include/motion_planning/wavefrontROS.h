#ifndef WAVEFRONT_H_INCLUDED
#define WAVEFRONT_H_INCLUDED

#include<iostream>
#include<fstream>
#include<vector>


typedef struct node{
    unsigned int ligne;
    unsigned int colonne;
    int score;
    node(unsigned int i,unsigned int j,int s);
    node(void);
}node;



class wavefront{
    private:
        node start;
        node goal;
        bool wayfound;
        int **matrix;
        std::vector<node> file;
        void pull(void);
        void parse(void);
    public:
	     unsigned int size_h;
         unsigned int size_l;
         wavefront(void);
         ~wavefront(void);
         void readfile(const char* path);
         void setgoal(unsigned i,unsigned j);
         void setstart(unsigned i,unsigned j);
         void afficher_matrix(void);
         void propagate_wave(void);
         std::vector<node> find_path(void);
         void affichersol(const char* path);
         void clearmatrix(void);
         void addobstacle(unsigned int i,unsigned int j);
         void getmatrix(unsigned int i,unsigned int j);
    private:wavefront(const wavefront &);
            wavefront & operator = (const wavefront &);
};


#endif // WAVEFRONT_H_INCLUDED

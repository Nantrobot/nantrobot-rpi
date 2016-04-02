#ifndef MATRIX_H
#define MATRIX_H

#include<math.h>

template<int rows,int columns> class matrix {

  private: float array[rows][columns]; //the array of rows*columns elements of type "float"

           static const int nr=rows;
           static const int nc=columns;
           int r;//helper assignment variables
           int c;
           bool assign;

  public:
          bool invalid_op; //this static flag indicates wether an incorrect assignment has been performed
          matrix(void);//first constructor with zero initialisation

          matrix<rows,columns> add(const matrix<rows,columns> & inOperand); // addition
          matrix<rows,columns> oppose(void); //get the opposite of each element
          matrix<rows,columns> scalar_multiply(const float  & inOperand); //scalar multiplication
          float det(void); // compute the determinant (for square matrix up to 3*3)
          matrix<rows,columns> inverse(void); //compute the inverse (for square matrix up to 3*3)
          matrix<columns,rows> transpose(void); //compute the transpose matrix

          matrix<rows,columns> operator + (const matrix<rows,columns> & inOperand); //addition operator as a member function
          matrix<rows,columns> operator - (const matrix<rows,columns> & inOperand); //subsraction operator as a member function

         matrix<rows,columns> & operator << (float first); //assignment operator
         //A  <<  1,2,3,
         //      4,5,6,
         //      7,8,9;
         matrix<rows,columns> & operator , (float follow); //assignment operator (next part)


          float & operator () (int i,int j); //get the element at row r and column c.Return a reference to modify the element
          //a=A(5,1);
          //A(5,1)=2;
          void  operator = (const matrix<rows,columns> & inOperand);//assignment operator overloading

          template<int RO,int CO,int CO_rhs> matrix<RO,CO_rhs> friend multiply(const matrix<RO,CO> & lhs,const matrix<CO,CO_rhs> & rhs);
          template<int RO,int CO,int CO_rhs> matrix<RO,CO_rhs> friend operator * (const matrix<RO,CO> & lhs,const matrix<CO,CO_rhs> & rhs);
          template<int R,int C> friend class matrix;
};

template<int rows,int columns> matrix<rows,columns>::matrix(void):array(),r(0),c(0),assign(false),invalid_op(false){
 for(int i=0;i<nr;i++){
   for(int j=0;j<nc;j++){
     array[i][j]=0;
   }
 }
}

template<int rows,int columns> void matrix<rows,columns>::operator = (const matrix<rows,columns> & inOperand){
  for(int i=0;i<nr;i++){
   for(int j=0;j<nc;j++){
     array[i][j]=inOperand.array[i][j];
   }
 }
}

template<int rows,int columns> float & matrix<rows,columns>::operator ()(int i,int j){
  return(array[i][j]);
}


template<int rows,int columns> matrix<rows,columns> & matrix<rows,columns>::operator << (float first){
  if(assign==false){
    assign=true;
    array[r][c]=first;
  }else{
    assign=true;
    r=0;
    c=0;
    array[r][c]=first;
    invalid_op=1;
  }
  return(*this);
}


template<int rows,int columns> matrix<rows,columns> & matrix<rows,columns>::operator , (float follow){
  if(assign==true){
    c++;
    if(c==nc){
      c=0;
      r++;
    }
    if(r==nr){
      assign=false;
      c=0;
      r=0;
    }else{
      array[r][c]=follow;
    }
  }else{
    invalid_op=1;
  }
 return(*this);
}


template<int rows,int columns> matrix<rows,columns> matrix<rows,columns>::add(const matrix<rows,columns> & inOperand){
  matrix<rows,columns> resultat;
  for(int i=0;i<nr;i++){
    for(int j=0;j<nc;j++){
      resultat.array[i][j]=(this->array[i][j])+ inOperand.array[i][j];
    }
  }
  return(resultat);
}

template<int rows,int columns> matrix<rows,columns> matrix<rows,columns>::oppose(void){
  matrix<rows,columns> resultat;
  for(int i=0;i<nr;i++){
    for(int j=0;j<nc;j++){
      resultat.array[i][j]=-array[i][j];
    }
  }
  return(resultat);
}




template<int rows,int columns> matrix<rows,columns> matrix<rows,columns>::scalar_multiply(const float  & inOperand){
   matrix<rows,columns> resultat;
  for(int i=0;i<nr;i++){
    for(int j=0;j<nc;j++){
      resultat.array[i][j]=array[i][j]*inOperand;
    }
  }
  return(resultat);
}

template<int rows,int columns> float matrix<rows,columns>::det(void){
  if(nc==nr){//matrice carrée
    float resultat;
    switch (nc){
      case 1:
        resultat=array[0][0];
        break;
      case 2:
        resultat=array[0][0]*array[1][1]-array[1][0]*array[0][1];
        break;
      case 3:{//la portée des variables est celle de tout le switch, alors qu'à l'exécution le code ne passera peut-être pas par le case qui contient sa déclaration:
      //entourer tout case contenant une déclaration par une paire d'accolade.
        float temp1=array[0][0]*array[1][1]*array[2][2];
        float temp2=array[1][0]*array[2][1]*array[0][2];
        float temp3=array[2][0]*array[0][1]*array[1][2];
        float temp4=array[2][0]*array[1][1]*array[0][2];
        float temp5=array[0][0]*array[2][1]*array[1][2];
        float temp6=array[1][0]*array[0][1]*array[2][2];
        resultat=temp1+temp2+temp3-temp4-temp5-temp6;
        break;
      }
      default:
        return(0);
        break;
    }
   return(resultat);
  }else{
    return(0);
  }
}


template<int rows,int columns> matrix<rows,columns> matrix<rows,columns>::inverse(void){
  if(nc==nr){
    matrix<rows,columns> resultat;
    switch(nc) {
      case 1:
        resultat.array[0][0]=1/(array[0][0]);
        break;
      case 2:
        resultat.array[0][0]=array[1][1];
        resultat.array[0][1]=-array[0][1];
        resultat.array[1][0]=-array[1][0];
        resultat.array[1][1]=array[0][0];
        resultat=resultat.scalar_multiply(1/(this->det()));
        break;
      case 3:
        float a,b,c,d,e,f,g,h,i;
        a=array[0][0];b=array[0][1];c=array[0][2];
        d=array[1][0];e=array[1][1];f=array[1][2];
        g=array[2][0];h=array[2][1];i=array[2][2];
        resultat.array[0][0]=e*i-f*h;resultat.array[0][1]=c*h-b*i;resultat.array[0][2]=b*f-c*e;
        resultat.array[1][0]=f*g-d*i;resultat.array[1][1]=a*i-c*g;resultat.array[1][2]=c*d-a*f;
        resultat.array[2][0]=d*h-e*g;resultat.array[2][1]=b*g-a*h;resultat.array[2][2]=a*e-b*d;
        resultat=resultat.scalar_multiply(1/(this->det()));
        break;
      default:
        invalid_op=1;
        resultat=(*this);
        break;
    }
    return(resultat);
  }else{
    invalid_op=1;
    return(*this);
  }
}


template<int rows,int columns> matrix<columns,rows> matrix<rows,columns>::transpose(void){
  matrix<columns,rows> resultat;
  for(int j=0;j<resultat.nc;j++){
    for(int i=0;i<resultat.nr;i++){
      resultat.array[i][j]=array[j][i];
    }
  }
  return(resultat);
}


template<int rows,int columns> matrix<rows,columns> matrix<rows,columns>::operator + (const matrix<rows,columns> & inOperand){
  return(this->add(inOperand));
}

template<int rows,int columns> matrix<rows,columns> matrix<rows,columns>::operator - (const matrix<rows,columns> & inOperand){
  matrix<rows,columns> temp;
  temp=inOperand;
  return(this->add(temp.oppose()));
}





////////////External function for commutativity or template issues)///

template<int RO,int CO,int CO_rhs> matrix<RO,CO_rhs> multiply(const matrix<RO,CO> & lhs,const matrix<CO,CO_rhs> & rhs){
   matrix<RO,CO_rhs> resultat;
   for(int i=0;i<resultat.nr;i++){
     for(int j=0;j<resultat.nc;j++){
       float sum=0;
       for(int k=0;k<lhs.nc;k++){
         sum+=lhs.array[i][k]*rhs.array[k][j];
       }
       resultat.array[i][j]=sum;
     }
   }
  return(resultat);
}

template<int RO,int CO,int CO_rhs> matrix<RO,CO_rhs> operator * (const matrix<RO,CO> & lhs,const matrix<CO,CO_rhs> & rhs){
  return(multiply(lhs,rhs));
}



template<int rows,int columns> matrix<rows,columns> operator * (const matrix<rows,columns> & inOperandMatrix,const float  & inOperandScalar){
  matrix<rows,columns> temp;
  temp=inOperandMatrix;
  return(temp.scalar_multiply(inOperandScalar));
}


template<int rows,int columns> matrix<rows,columns> operator * (const float  & inOperandScalar,const matrix<rows,columns> & inOperandMatrix){
  matrix<rows,columns> temp;
  temp=inOperandMatrix;
  return(temp.scalar_multiply(inOperandScalar));
}


#endif

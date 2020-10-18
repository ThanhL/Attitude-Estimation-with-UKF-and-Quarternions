/*  
Math helpers header file
*/
#include "Arduino.h"
#include "BasicLinearAlgebra.h"


float constrain_angle(float x);
BLA::Matrix<4,4> skew_matrix_4d(BLA::Matrix<4> x); 
BLA::Matrix<3,3> skew_matrix_3d(BLA::Matrix<3> x);
BLA::Matrix<3> cross_product(BLA::Matrix<3> a, BLA::Matrix<3> b);





template <int dim>
void cholesky_decomposition(const BLA::Matrix<dim,dim> &A);



// template<int dim, class MemT>
// BLA::Matrix<dim,dim,MemT> &cholesky_decomposition(BLA::Matrix<dim,dim,MemT> &A, int *res = NULL);



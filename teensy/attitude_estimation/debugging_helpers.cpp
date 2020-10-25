#include "debugging_helpers.h"


void print_mtxf(const Eigen::MatrixXf& X) 
{
	/***
	Prints the matrix/vector of type float to Arduino Serial
	***/
	int nrow = X.rows();
	int ncol = X.cols();

	Serial.print("nrow: "); Serial.println(nrow);
	Serial.print("ncol: "); Serial.println(ncol);     
	for (int i = 0; i < nrow; i++)
	{
		for (int j = 0; j < ncol; j++)
		{
			Serial.print(X(i,j), 6);   // print 6 decimal places
			Serial.print(", ");
		}
		Serial.println();
	}
	Serial.println();
}

void print_mtxd(const Eigen::MatrixXd& X) 
{
	/***
	Prints the matrix/vector of type double to Arduino Serial
	***/
	int nrow = X.rows();
	int ncol = X.cols();

	Serial.print("nrow: "); Serial.println(nrow);
	Serial.print("ncol: "); Serial.println(ncol);     
	for (int i = 0; i < nrow; i++)
	{
		for (int j = 0; j < ncol; j++)
		{
			Serial.print(X(i,j), 6);   // print 6 decimal places
			Serial.print(", ");
		}
		Serial.println();
	}
	Serial.println();
}
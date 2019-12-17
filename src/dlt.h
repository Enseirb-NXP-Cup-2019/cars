#ifndef DLT_H
#define DLT_H

double** dlt(double p[][2], double p_ligne[][2]);
double** apply_dlt(double p[][2], double** H);

void __affiche_matrix(double** p, int dim_x, int dim_y);
double** __create_matrix_dynamically(int dim_x, int dim_y);
double** __singular_value_decomposition(double** A);
double* __mat_mul(double** H, double* point);

#endif

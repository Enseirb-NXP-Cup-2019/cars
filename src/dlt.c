#include <stdio.h>
#include <stdlib.h>

#include "dlt.h"

int main() {
  double points_initals[4][2] = {{50, 15}, {55, 0}, {70, 15}, {64, 0}};
  double points_equiv[4][2]   = {{5, 65}, {5, 0}, {60, 65}, {60, 0}};
  double** H                  = dlt(points_initals, points_equiv);

  double points_curve[4][2]  = {{55, 15}, {55, 0}, {70, 15}, {60, 0}};
  double** curve_unprojected = apply_dlt(points_curve, H);
  __affiche_matrix(curve_unprojected, 4, 2);
}

double** dlt(double p[][2], double p_ligne[][2]) {

  double** A = __create_matrix_dynamically(8, 9);

  for (int i = 0; i < 4; i++) {

    double x = p[i][0];
    double y = p[i][1];
    double u = p_ligne[i][0];
    double v = p_ligne[i][1];

    A[2 * i][0] = -x;
    A[2 * i][1] = -y;
    A[2 * i][2] = -1;
    A[2 * i][3] = 0;
    A[2 * i][4] = 0;
    A[2 * i][5] = 0;
    A[2 * i][6] = u * x;
    A[2 * i][7] = u * y;
    A[2 * i][8] = u;

    A[2 * i + 1][0] = 0;
    A[2 * i + 1][1] = 0;
    A[2 * i + 1][2] = 0;
    A[2 * i + 1][3] = -x;
    A[2 * i + 1][4] = -y;
    A[2 * i + 1][5] = -1;
    A[2 * i + 1][6] = v * x;
    A[2 * i + 1][7] = v * y;
    A[2 * i + 1][8] = v;
  }

  return __singular_value_decomposition(A);
}

double** apply_dlt(double p[][2], double** H) {

  double** output = __create_matrix_dynamically(4, 2);

  for (int i = 0; i < 4; i++) {
    double point_hom_coord[3] = {p[i][0], p[i][1], 1};
    double* r                 = __mat_mul(H, point_hom_coord);
    output[i][0]              = r[0] / r[2];
    output[i][1]              = r[1] / r[2];
    printf("%f\n", point_hom_coord[0]);
    free(r);
  }
  return output;
}

// TODO: this could be way more generic. i'm just a bit lazy :P
double* __mat_mul(double** H, double* point) {
  double* result = (double*)malloc(3 * sizeof(double));
  result[0]      = H[0][0] * point[0] + H[0][1] * point[1] + H[0][2] * point[2];
  result[1]      = H[1][0] * point[0] + H[1][1] * point[1] + H[1][2] * point[2];
  result[2]      = H[2][0] * point[0] + H[2][1] * point[1] + H[2][2] * point[2];
  return result;
}

// TODO: This is the missing part in C (python impl: numpy.linalg.svd)
double** __singular_value_decomposition(double** A) {
  __affiche_matrix(A, 8, 9);
  double** H = __create_matrix_dynamically(3, 3);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      H[i][j] = 1;

  return H;
}

double** __create_matrix_dynamically(int dim_x, int dim_y) {
  double** m = (double**)malloc(dim_x * sizeof(double*));
  for (int i = 0; i < dim_x; i++)
    m[i] = (double*)malloc(dim_y * sizeof(double));

  for (int i = 0; i < dim_x; i++)
    for (int j = 0; j < dim_y; j++)
      m[i][j] = 0.0;

  return m;
}

// TODO: memory leaks are just a way for bits to experiment true freedom
void __free_matrix(int dim_x, int dim_y) {}

void __affiche_matrix(double** p, int dim_x, int dim_y) {
  for (int i = 0; i < dim_x; i++) {
    for (int j = 0; j < dim_y; j++) {
      printf("%.2f ", p[i][j]);
    }
    printf("\n");
  }
}

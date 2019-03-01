
#include "KalmanFilter.h"

#include <Arduino.h>

int choldc1(double * a, double * p, int n);
int choldcsl(double * A, double * a, double * p, int n);
int cholsl(double * A, double * a, double * p, int n);
void mulmat(double * a, double * b, double * c, int arows, int acols, int bcols);
void transpose(double * a, double * at, int m, int n);
void accum(double * a, double * b, int m, int n);
void sub(double * a, double * b, double * c, int n);
void add(double * a, double * b, double * c, int n);
void mulvec(double * a, double * x, double * y, int m, int n);
void negate(double * a, int m, int n);
void mat_addeye(double * a, int n);

KalmanFilter::KalmanFilter()
{
  memset(P[0], 0, sizeof(P[0]));
  memset(Q[0], 0, sizeof(Q[0]));
  memset(R[0], 0, sizeof(R[0]));
  memset(G[0], 0, sizeof(G[0]));
  memset(F[0], 0, sizeof(F[0]));
  memset(H[0], 0, sizeof(H[0]));
}

bool KalmanFilter::step(double * z)
{
  // model(this->fx, this->F[0], this->hx, this->H);

  /* P_k = F_{k-1} P_{k-1} F^T_{k-1} + Q_{k-1} */
  mulmat(F[0], P[0], tmp0[0], n, n, n);
  transpose(F[0], Ft[0], n, n);
  mulmat(tmp0[0], Ft[0], Pp[0], n, n, n);
  accum(Pp[0], Q[0], n, n);

  /* G_k = P_k H^T_k (H_k P_k H^T_k + R)^{-1} */
  transpose(H[0], Ht[0], m, n);
  mulmat(Pp[0], Ht[0], tmp1[0], n, n, m);
  mulmat(H[0], Pp[0], tmp2[0], m, n, n);
  mulmat(tmp2[0], Ht[0], tmp3[0], m, n, m);
  accum(tmp3[0], R[0], m, m);
  if (cholsl(tmp3[0], tmp4[0], tmp5, m))
    return false;

  mulmat(tmp1[0], tmp4[0], G[0], n, m, m);

  /* \hat{x}_k = \hat{x_k} + G_k(z_k - h(\hat{x}_k)) */
  sub(z, hx, tmp5, m);
  mulvec(G[0], tmp5, tmp2[0], n, m);
  add(fx, tmp2[0], x, n);

  /* P_k = (I - G_k H_k) P_k */
  mulmat(G[0], H[0], tmp0[0], n, m, n);
  negate(tmp0[0], n, n);
  mat_addeye(tmp0[0], n);
  mulmat(tmp0[0], Pp[0], P[0], n, n, n);

  /* success */
  return true;
}

void mulmat(double * a, double * b, double * c, int arows, int acols, int bcols)
{
    int i, j,l;
    for(i=0; i<arows; ++i)
        for(j=0; j<bcols; ++j) {
            c[i*bcols+j] = 0;
            for(l=0; l<acols; ++l)
                c[i*bcols+j] += a[i*acols+l] * b[l*bcols+j];
        }
}

int choldc1(double * a, double * p, int n)
{
    int i,j,k;
    double sum;
    for (i = 0; i < n; i++) {
        for (j = i; j < n; j++) {
            sum = a[i*n+j];
            for (k = i - 1; k >= 0; k--) {
                sum -= a[i*n+k] * a[j*n+k];
            }
            if (i == j) {
                if (sum <= 0) {
                    return 1; /* error */
                }
                p[i] = sqrt(sum);
            }
            else {
                a[j*n+i] = sum / p[i];
            }
        }
    }

    return 0; /* success */
}

int choldcsl(double * A, double * a, double * p, int n)
{
    int i,j,k; double sum;
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            a[i*n+j] = A[i*n+j];
    if (choldc1(a, p, n)) return 1;
    for (i = 0; i < n; i++) {
        a[i*n+i] = 1 / p[i];
        for (j = i + 1; j < n; j++) {
            sum = 0;
            for (k = i; k < j; k++) {
                sum -= a[j*n+k] * a[k*n+i];
            }
            a[j*n+i] = sum / p[j];
        }
    }

    return 0; /* success */
}


int cholsl(double * A, double * a, double * p, int n)
{
    int i,j,k;
    if (choldcsl(A,a,p,n)) return 1;
    for (i = 0; i < n; i++) {
        for (j = i + 1; j < n; j++) {
            a[i*n+j] = 0.0;
        }
    }
    for (i = 0; i < n; i++) {
        a[i*n+i] *= a[i*n+i];
        for (k = i + 1; k < n; k++) {
            a[i*n+i] += a[k*n+i] * a[k*n+i];
        }
        for (j = i + 1; j < n; j++) {
            for (k = j; k < n; k++) {
                a[i*n+j] += a[k*n+i] * a[k*n+j];
            }
        }
    }
    for (i = 0; i < n; i++) {
        for (j = 0; j < i; j++) {
            a[i*n+j] = a[j*n+i];
        }
    }

    return 0; /* success */
}

void mulvec(double * a, double * x, double * y, int m, int n)
{
    int i, j;
    for(i=0; i<m; ++i) {
        y[i] = 0;
        for(j=0; j<n; ++j)
            y[i] += x[j] * a[i*n+j];
    }
}

void transpose(double * a, double * at, int m, int n)
{
    int i,j;
    for(i=0; i<m; ++i)
        for(j=0; j<n; ++j) {
            at[j*m+i] = a[i*n+j];
        }
}

/* A <- A + B */
void accum(double * a, double * b, int m, int n)
{
    int i,j;
    for(i=0; i<m; ++i)
        for(j=0; j<n; ++j)
            a[i*n+j] += b[i*n+j];
}

/* C <- A + B */
void add(double * a, double * b, double * c, int n)
{
    int j;
    for(j=0; j<n; ++j)
        c[j] = a[j] + b[j];
}


/* C <- A - B */
void sub(double * a, double * b, double * c, int n)
{
    int j;
    for(j=0; j<n; ++j)
        c[j] = a[j] - b[j];
}

void negate(double * a, int m, int n)
{
    int i, j;
    for(i=0; i<m; ++i)
        for(j=0; j<n; ++j)
            a[i*n+j] = -a[i*n+j];
}

void mat_addeye(double * a, int n)
{
    int i;
    for (i=0; i<n; ++i)
        a[i*n+i] += 1;
}

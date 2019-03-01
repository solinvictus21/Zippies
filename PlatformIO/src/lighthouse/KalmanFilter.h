
#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_

#define Nsta 1
#define Mobs 2

class KalmanFilter
{

private:
  int n = Nsta;          /* number of state values */
  int m = Mobs;          /* number of observables */

  double x[Nsta];        /* state vector */

  double P[Nsta][Nsta];  /* prediction error covariance */
  double Q[Nsta][Nsta];  /* process noise covariance */
  double R[Mobs][Mobs];  /* measurement error covariance */

  double G[Nsta][Mobs];  /* Kalman gain; a.k.a. K */

  double Ht[Nsta][Mobs]; /* transpose of measurement Jacobian */
  double Ft[Nsta][Nsta]; /* transpose of process Jacobian */
  double Pp[Nsta][Nsta]; /* P, post-prediction, pre-update */

  double fx[Nsta];       /* output of user defined f() state-transition function */
  double F[Nsta][Nsta];  /* Jacobian of process model */
  double hx[Mobs];       /* output of user defined h() measurement function */
  double H[Mobs][Nsta];  /* Jacobian of measurement model */

  /* temporary storage */
  double tmp0[Nsta][Nsta];
  double tmp1[Nsta][Mobs];
  double tmp2[Mobs][Nsta];
  double tmp3[Mobs][Mobs];
  double tmp4[Mobs][Mobs];
  double tmp5[Mobs];

public:
  KalmanFilter();

  /**
   * Implement this function for your EKF model.
   * @param fx gets output of state-transition function <i>f(x<sub>0 .. n-1</sub>)</i>
   * @param F gets <i>n &times; n</i> Jacobian of <i>f(x)</i>
   * @param hx gets output of observation function <i>h(x<sub>0 .. n-1</sub>)</i>
   * @param H gets <i>m &times; n</i> Jacobian of <i>h(x)</i>
   */
  virtual void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]) = 0;

  /**
   * Sets the specified value of the prediction error covariance. <i>P<sub>i,j</sub> = value</i>
   * @param i row index
   * @param j column index
   * @param value value to set
   */
  void setP(int i, int j, double value)
  {
      this->P[i][j] = value;
  }

  /**
   * Sets the specified value of the process noise covariance. <i>Q<sub>i,j</sub> = value</i>
   * @param i row index
   * @param j column index
   * @param value value to set
   */
  void setQ(int i, int j, double value)
  {
      this->Q[i][j] = value;
  }

  /**
   * Sets the specified value of the observation noise covariance. <i>R<sub>i,j</sub> = value</i>
   * @param i row index
   * @param j column index
   * @param value value to set
   */
  void setR(int i, int j, double value)
  {
      this->R[i][j] = value;
  }

public:

  /**
   * Returns the state element at a given index.
   * @param i the index (at least 0 and less than <i>n</i>
   * @return state value at index
   */
  double getX(int i)
  {
      return this->x[i];
  }

  /**
   * Sets the state element at a given index.
   * @param i the index (at least 0 and less than <i>n</i>
   * @param value value to set
   */
  void setX(int i, double value)
  {
      this->x[i] = value;
  }

  /**
    Performs one step of the prediction and update.
   * @param z observation vector, length <i>m</i>
   * @return true on success, false on failure caused by non-positive-definite matrix.
   */
  bool step(double * z);

};

#endif

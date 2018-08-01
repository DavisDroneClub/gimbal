#include "math.h"



double quat_magnitude(double *input_q) {
    double sum = 0;
    for(int i = 0; i < 4; i++) {
        sum += pow(input_q[i], 2);
    }

    return sqrt(sum);
}

void quat_normalize(double *q) {
    double mag = quat_magnitude(q);
    for(int i = 0; i < 4; i++) {
        q[i] /= mag;
    }
}

double vect_magnitude(double *input_v) {
    double sum = 0;
    for(int i = 0; i < 3; i++) {
        sum += pow(input_v[i], 2);
    }
    return sqrt(sum);
}

void vect_normalize(double *v) {
    double mag = vect_magnitude(v);
    for(int i = 0; i < 3; i++) {
        v[i] /= mag;

    }
}

void conjugate(double *input_q, double *output_q) {
    output_q[0] = input_q[0];
    output_q[1] = -input_q[1];
    output_q[2] = -input_q[2];
    output_q[3] = -input_q[3];
}

void inverse(double *input_q, double *output_q) {
    double denominator = pow(quat_magnitude(input_q),2);
    double conj_q[4];

    conjugate(input_q, conj_q);
    output_q[0] = conj_q[0]/denominator;
    output_q[1] = -conj_q[1]/denominator;
    output_q[2] = -conj_q[2]/denominator;
    output_q[3] = -conj_q[3]/denominator;
}

/*
  Multiply two quaternions
*/
void multiply(double *p, double *q, double *output_q) {
    output_q[0] = (p[0]*q[0]) - (p[1]*q[1]) -
                  (p[2]*q[2]) - (p[3]*q[3]);
    output_q[1] = (p[0]*q[1]) + (p[1]*q[0]) +
                  (p[2]*q[3]) - (p[3]*q[2]);
    output_q[2] = (p[0]*q[2]) - (p[1]*q[3]) +
                  (p[2]*q[0]) + (p[3]*q[1]);
    output_q[3] = (p[0]*q[3]) + (p[1]*q[2]) -
                  (p[2]*q[1]) + (p[3]*q[0]);
}

/*
  Print the components of a quaternion
*/
void print_q(double *input_q) {
    for (int i = 0; i < 4; i++) {
        Serial.print(input_q[i],4);
        Serial.print('\t');
        Serial.print('\t');
    }
    Serial.println();
}

/*
  Multiply three quaternions
*/
void multiply_three(double *input_q1, double *input_q2,
                    double *input_q3, double *output_q) {
    double temp[4];
    multiply(input_q1, input_q2, temp);
    multiply(temp, input_q3, output_q);
}

/*
  Rotate quaternion by another quaternion
*/
void rotate(double *input_q1, double *input_q2,
            double *output_q) {
    double temp[4];
    inverse(input_q1, temp);
    multiply_three(input_q1, input_q2, temp, output_q);
}

/*
  Transform a quaternion, q, to a 3x3 matrix, R.
*/
void quat_to_inverse_matrix(double *q, double R[][3]) {
    R[0][0] = pow(q[0],2)+pow(q[1],2)-pow(q[2],2)-pow(q[3],2);
    R[1][0] = 2*(q[1]*q[2]-q[0]*q[3]);
    R[2][0] = 2*(q[1]*q[3]+q[0]*q[2]);

    R[0][1] = 2*(q[1]*q[2]+q[0]*q[3]);
    R[1][1] = pow(q[0],2)-pow(q[1],2)+pow(q[2],2)-pow(q[3],2);
    R[2][1] = 2*(q[2]*q[3]-q[0]*q[1]);

    R[0][2] = 2*(q[1]*q[3]-q[0]*q[2]);
    R[1][2] = 2*(q[2]*q[3]+q[0]*q[1]);
    R[2][2] = pow(q[0],2)-pow(q[1],2)-pow(q[2],2)+pow(q[3],2);
}

/*
  MAtrix multiplication in the form of Ax = B where x and B is a 3x1 vector
  and A is a 3x3 matrix
*/
void matrix_multiply(double A[][3], double *x, double *B) {
    for(int i = 0; i < 3; i++) {
        B[i] = A[i][0]*x[0] + A[i][1]*x[1] + A[i][2]*x[2];
    }
}


/*
  Calculate the error gain factor
*/
double err_gain_factor(double em, double low, double high) {
    if (em < low) {
        return 1;
    }
    else if (em > high) {
        return 0;
    }
    else
    {
        return (-1/(high-low))*(em)+((high)/(high-low));
    }
}

void slerp(double alpha, double* d_q_acc, double* d_acc_interp) {
  //SLERP
  omega = acos(d_q_acc[0]);

  for(int i = 0; i < 4; i++) {
      d_acc_interp[i] = (sin((1-alpha)*omega)/sin(omega))*identity[i] +
                        (sin(alpha*omega)/sin(omega))*d_q_acc[i];
  }
}


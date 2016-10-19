//
//  vector.c
//  Raycasting
//
//  Created by jr2339 on 9/30/16.
//  Copyright © 2016 jr2339. All rights reserved.
//

#include "vector.h"

double sqr(double v){
    return v*v;
}


void Vector_zero(Vector vector){
    vector[0] = 0;
    vector[1] = 0;
    vector[2] = 0;
}

void Vector_copy(Vector in, Vector out){
    out[0] = in[0];
    out[1] = in[1];
    out[2] = in[2];
}

void normalize(double *v) {
    double len = sqr(v[0]) + sqr(v[1]) + sqr(v[2]);
    len = sqrt(len);
    v[0] /= len;
    v[1] /= len;
    v[2] /= len;
}


double Vector_len(Vector a){
    return sqrt(sqr(a[0])+sqr(a[1])+sqr(a[2]));
}


void Vector_add(Vector a, Vector b, Vector c){
    c[0] = a[0] + b[0];
    c[1] = a[1] + b[1];
    c[2] = a[2] + b[2];
}


void Vector_sub(Vector a, Vector b, Vector c){
    c[0] = a[0] - b[0];
    c[1] = a[1] - b[1];
    c[2] = a[2] - b[2];
}


void Vector_scale(Vector a, double s, Vector b){
    b[0] = s * a[0];
    b[1] = s * a[1];
    b[2] = s * a[2];
}


double Vector_dot(Vector a, Vector b){
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}


void Vector_corss(Vector a, Vector b, Vector c){
    c[0] = a[1]*b[2] - a[2]*b[1];
    c[1] = a[2]*b[0] - a[0]*b[2];
    c[2] = a[0]*b[1] - a[1]*b[0];
}



void Vector_reflect(Vector v, Vector n, Vector v_r){
    normalize(n);
    double scalar = 2.0 * Vector_dot(n, v);
    Vector tmp_vector;
    Vector_scale(n, scalar, tmp_vector);
    Vector_sub(v, tmp_vector, v_r);
}



/*
void Vector_Unit(Vector in, Vector out){
    double len = Vector_len(in);
    out[0] = in[0]/len;
    out[1] = in[1]/len;
    out[2] = in[2]/len;
    
}
*/



















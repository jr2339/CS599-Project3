//
//  raycast.h
//  Raycasting
//
//  Created by jr2339 on 9/30/16.
//  Copyright © 2016 jr2339. All rights reserved.
//

#ifndef raycast_h
#define raycast_h

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "json.h"
#include "vector.h"
#include "raycast.h"
#include "ppm.h"
#include "base.h"
#include "illumination.h"

typedef struct Ray{
    double origin[3];
    double direction[3];
}Ray;

int get_camera(OBJECT *objects);
void shade_pixel(double *color, int row, int col,Image *image);
double plane_intersection(Ray *ray, double *Pos, double *Norm);
double sphere_intersection(Ray *ray, double *C, double r);
double quadric_intersection(Ray *ray, double *Co,double* Position);
void raycast_scene(Image *image, double cam_width, double cam_height, OBJECT *object);
void get_quadric_normal(double* normal,double *Coefficient,double *Position);
#endif /* raycast_h */

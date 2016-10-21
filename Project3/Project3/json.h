//
//  json.h
//  Raycasting
//
//  Created by jr2339 on 9/22/16.
//  Copyright © 2016 jr2339. All rights reserved.
//


#ifndef json_h
#define json_h

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "ppm.h"
#include "base.h"

#define MAX_OBJECTS 256
#define MAX_COLOR_VAL 255       // maximum value to use for colors 0-255
#define CAM 1
#define SPH 2
#define PLAN 3
#define QUAD 4
#define LIG 5
#define SPOTLIG 6
#define POINTLIG 7

/*Each Structure is a type of Objects in Jason File*/
typedef struct CAMERA {
    double width;
    double height;
} CAMERA;


typedef struct SPHERE {
    double *diff_color;
    double *spec_color;
    double *position;
    double radius;
} SPHERE;



typedef struct PLANE {
    double *diff_color;     // diffuse color
    double *spec_color;     // specular color
    double *position;
    double *normal;
} PLANE;


typedef struct QUADRIC {
    double *diff_color;
    double *spec_color;
    double *position;
    double *coefficient;
} QUADRIC;


typedef struct LIGHT{
    int type;
    double *color;
    double *position;
    double *direction;
    double theta_deg;
    double rad_att0;
    double rad_att1;
    double rad_att2;
    double ang_att0;
} LIGHT;

/*DEFINE STRUCTURE  FOR JASON FILE WHICH STROE OUR OBJECT*/
typedef struct OBJECT{
    int type;
    union {
        CAMERA camera;
        SPHERE sphere;
        PLANE plane;
        QUADRIC quadric;
    };
} OBJECT;


extern int line;  // global variable, it will tells us which line is not correct

extern OBJECT objects[MAX_OBJECTS];
extern LIGHT lights[MAX_OBJECTS];
extern int num_lights;
extern int num_objects;

void read_scene(const char* filename);
void get_objects(OBJECT *object);





#endif /* json_h */

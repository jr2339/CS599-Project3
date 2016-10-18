//
//  illumination.h
//  Project3
//
//  Created by jr2339 on 10/16/16.
//  Copyright © 2016 jr2339. All rights reserved.
//

#ifndef illumination_h
#define illumination_h

#include <stdio.h>
#include <math.h>
#include "vector.h"

//double calculate_angular_att(LIGHT *light, double direction_to_object[3]);
void calculate_diffuse(double *normal_vector,
                       double *light_vector,
                       double *light_color,
                       double *obj_color,
                       double *out_color);

void calculate_specular(double ns,
                        double *L,
                        double *R,
                        double *N,
                        double *V,
                        double *KS,
                        double *IL,
                        double *out_color);
double check_value(double color_val);


#endif /* illumination_h */

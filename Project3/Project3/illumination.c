//
//  illumination.c
//  Project3
//
//  Created by jr2339 on 10/16/16.
//  Copyright © 2016 jr2339. All rights reserved.
//

#include "illumination.h"

double check_value(double color_val){
    if (color_val < 0)
        return 0;
    else if (color_val > 1)
        return 1;
    else
        return color_val;
}
/*=======================================================================================================================*/
void get_diffuse(double *normal_vector, double *light_vector, double *light_color, double *object_color, double *out_color) {

    double n_dot_l = Vector_dot(normal_vector, light_vector);
    if (n_dot_l > 0) {
        Vector diffuse_product;
        diffuse_product[0] = object_color[0] * light_color[0];
        diffuse_product[1] = object_color[1] * light_color[1];
        diffuse_product[2] = object_color[2] * light_color[2];

        Vector_scale(diffuse_product, n_dot_l, out_color);
    }
    else {
        // would normally return K_a*I_a here...
        out_color[0] = 0;
        out_color[1] = 0;
        out_color[2] = 0;
    }
}
/*=======================================================================================================================*/

void get_specular(double ns, double *L, double *R, double *N, double *V, double *KS, double *IL, double *out_color) {
    double v_dot_r = Vector_dot(V, R);
    double n_dot_l = Vector_dot(N, L);
    if (v_dot_r > 0 && n_dot_l > 0) {
        double vr_to_the_ns = pow(v_dot_r, ns);
        Vector spec_product;
        spec_product[0] = KS[0] * IL[0];
        spec_product[1] = KS[1] * IL[1];
        spec_product[2] = KS[2] * IL[2];
        Vector_scale(spec_product, vr_to_the_ns, out_color);
    }
    else {
        Vector_zero(out_color);
    }
}
/*=======================================================================================================================*/
double calculate_angular_att(LIGHT *light, Vector direction_to_object){
    if (light->type != SPOTLIG){
        return 1.0;
    }
    if (light->direction == NULL) {
        fprintf(stderr, "Error: calculate_angular_att: Can't have spotlight with no direction\n");
        exit(1);
    }
    double theta_rad = light->theta_deg * (M_PI / 180);
    double cos_theta = cos(theta_rad);
    //double cos_theta = cos(M_PI / 2.0);
    double vo_dot_vl = Vector_dot(light->direction, direction_to_object);
    if (vo_dot_vl < cos_theta){
        return 0.0;
    }
    return pow(vo_dot_vl, light->ang_att0);
}
/*==================================================================================================*/
double calculate_radial_att(LIGHT *light, double distance_to_light) {
    if (light->rad_att0 == 0 && light->rad_att1 == 0 && light->rad_att2 == 0) {
        fprintf(stdout, "WARNING: calculate_radial_att: Found all 0s for attenuation. Assuming default values of radial attenuation\n");
        light->rad_att2 = 1.0;
    }
    // if d_l == infinity, return 1
    if (distance_to_light > 99999999999999) {
        return 1.0;
    }
    
    double dl_sqr = sqr(distance_to_light);
    double denom = light->rad_att2 * dl_sqr + light->rad_att1 * distance_to_light + light->ang_att0;
    return 1.0 / denom;
}


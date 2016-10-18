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






void calculate_diffuse(double *N, double *L, double *IL, double *KD, double *out_color) {
    // K_a*I_a should be added to the beginning of this whole thing, which is a constant and ambient light
    double n_dot_l = Vector_dot(N, L);
    if (n_dot_l > 0) {
        double diffuse_product[3];
        diffuse_product[0] = KD[0] * IL[0];
        diffuse_product[1] = KD[1] * IL[1];
        diffuse_product[2] = KD[2] * IL[2];
        // multiply by n_dot_l and store in out_color
        Vector_scale(diffuse_product, n_dot_l, out_color);
    }
    else {
        // would normally return K_a*I_a here...
        out_color[0] = 0;
        out_color[1] = 0;
        out_color[2] = 0;
    }
}

void calculate_specular(double ns, double *L, double *R, double *N, double *V, double *KS, double *IL, double *out_color) {
    double v_dot_r = Vector_dot(V, R);
    double n_dot_l = Vector_dot(N, L);
    if (v_dot_r > 0 && n_dot_l > 0) {
        double vr_to_the_ns = pow(v_dot_r, ns);
        double spec_product[3];
        spec_product[0] = KS[0] * IL[0];
        spec_product[1] = KS[1] * IL[1];
        spec_product[2] = KS[2] * IL[2];
        Vector_scale(spec_product, vr_to_the_ns, out_color);
    }
    else {
        Vector_zero(out_color);
    }
}


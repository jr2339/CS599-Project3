//
//  main.c
//  Raycasting
//
//  Created by jr2339 on 9/22/16.
//  Copyright © 2016 jr2339. All rights reserved.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "raycast.h"

int main(int argc, const char * argv[]) {
    if (argc != 5) {
        fprintf(stderr, "Error: main: You must have 4 arguments\n");
        exit(1);
    }
    if (atoi(argv[1]) <= 0 || atoi(argv[2]) <= 0) {
        fprintf(stderr, "Error: main: width and height parameters must be > 0\n");
        exit(1);
    }
    
    /*open the input json file*/
    const char *input = argv[3];
    //Error Check
    if (input == NULL) {
        fprintf(stderr, "Error: main: Failed to open input file '%s'\n", argv[3]);
        exit(1);
    }
    read_scene(input);

    Image *image =(Image *)malloc(sizeof(Image));

   
    image->width = atoi(argv[1]);

    image->height = atoi(argv[2]);

    image->maxval = 255;
    image->data =(unsigned char*) malloc(sizeof(unsigned char) * image->width * image->height*4);

    int pos = get_camera(objects);
    if (pos == -1) {
        fprintf(stderr, "Error: main: No camera object found in data\n");
        exit(1);
    }
    
    
    
    
  
    raycast_scene(image, objects[pos].camera.width, objects[pos].camera.height, objects);
    
    const char *output = argv[4];

    ImageWrite(image, output,6);
    
    return 0;
}

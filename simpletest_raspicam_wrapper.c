/*
 * simpletest_raspicam.cpp
 *
 *  Created on: Feb 18, 2017
 *      Author: steveb
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "raspicam_wrapper.h"

int main( int argc, char **argv )
{
  struct raspicam_wrapper_handle *  Camera;       /* Camera handle */
  int                               return_value; /* the return value of this program */

    /* Open camera */
    printf( "Opening Camera...\n" );

    Camera = raspicam_wrapper_create();
    if (Camera != NULL)
    {
      if (raspicam_wrapper_open( Camera ))
      {
        // wait a while until camera stabilizes
        printf( "Sleeping for 3 secs\n" );
        sleep( 3 );

        // capture
        raspicam_wrapper_grab( Camera );

        // allocate memory
        unsigned char *data = (unsigned char *)malloc( raspicam_wrapper_getImageTypeSize( Camera, RASPICAM_WRAPPER_FORMAT_RGB ) );

        // extract the image in rgb format
        raspicam_wrapper_retrieve( Camera, data, RASPICAM_WRAPPER_FORMAT_RGB ); // get camera image

        // save
        FILE * outFile = fopen( "raspicam_image.ppm", "wb" );
        if (outFile != NULL)
        {
          fprintf( outFile, "P6\n" );
          fprintf( outFile, "%d %d 255\n", raspicam_wrapper_getWidth( Camera ), raspicam_wrapper_getHeight( Camera ) );
          fwrite( data, 1, raspicam_wrapper_getImageTypeSize( Camera, RASPICAM_WRAPPER_FORMAT_RGB ), outFile );
          printf( "Image saved at raspicam_image.ppm\n" );

          fclose( outFile );

          // save gray-scale
          FILE * outFile = fopen( "raspicam_image_gray.ppm", "wb" );
          if (outFile != NULL)
          {
            // convert to gray-scale
            struct RGB_pixel
            {
                unsigned char R;
                unsigned char G;
                unsigned char B;
            };
            struct RGB_pixel* pixel;
            unsigned int      pixel_count;
            unsigned int      pixel_index;
            unsigned char     pixel_value;

            pixel = (struct RGB_pixel *)data;
            pixel_count = raspicam_wrapper_getHeight( Camera ) * raspicam_wrapper_getWidth( Camera );
            for (pixel_index = 0; pixel_index < pixel_count; pixel_index++)
            {
              pixel_value = (((unsigned int)(pixel[pixel_index].R)) +
                             ((unsigned int)(pixel[pixel_index].G)) +
                             ((unsigned int)(pixel[pixel_index].B))) / 3; // do not worry about rounding
              pixel[pixel_index].R = pixel_value;
              pixel[pixel_index].G = pixel_value;
              pixel[pixel_index].B = pixel_value;
            }
            fprintf( outFile, "P6\n" );
            fprintf( outFile, "%d %d 255\n", raspicam_wrapper_getWidth( Camera ), raspicam_wrapper_getHeight( Camera ) );
            fwrite( data, 1, raspicam_wrapper_getImageTypeSize( Camera, RASPICAM_WRAPPER_FORMAT_RGB ), outFile );
            printf( "Image saved at raspicam_image_gray.ppm\n" );

            fclose( outFile );

            return_value = 0;
          }
          else
          {
            printf( "unable to save gray-scale image\n" );
            return_value = -1;
          }

          //free resources
          raspicam_wrapper_destroy( Camera );
        }
        else
        {
          printf( "unable to open output file\n" );
          return_value = -1;
        }

      }
      else
      {
        printf( "Error opening camera\n" );
        return_value = -1;
      }
    }
    else
    {
      printf( "Unable to allocate camera handle\n" );
      return_value = -1;
    }

    return return_value;
}

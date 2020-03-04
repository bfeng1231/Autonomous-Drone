/*
 * simpletest_raspicam.cpp
 *
 *  Created on: Feb 18, 2017
 *      Author: steveb
 */

#include <ctime>
#include <fstream>
#include <iostream>
#include <raspicam/raspicam.h>

int main( int argc, char **argv )
{
    raspicam::RaspiCam  Camera;       // Camera object
    int                 return_value; // the return value of this program

    // Open camera
    std::cout << "Opening Camera..." << std::endl;

    if (Camera.open())
    {
      // wait a while until camera stabilizes
      std::cout << "Sleeping for 3 secs" << std::endl;
      std::sleep( 3 );

      // capture
      Camera.grab();

      // allocate memory
      unsigned char *data = new unsigned char[Camera.getImageTypeSize( raspicam::RASPICAM_FORMAT_RGB )];

      // extract the image in rgb format
      Camera.retrieve( data, raspicam::RASPICAM_FORMAT_RGB ); // get camera image

      // save
      std::ofstream outFile( "raspicam_image.ppm",std::ios::binary );
      outFile << "P6\n" << Camera.getWidth() << " " << Camera.getHeight() << " 255\n";
      outFile.write( (char *)data, Camera.getImageTypeSize( raspicam::RASPICAM_FORMAT_RGB ) );
      std::cout << "Image saved at raspicam_image.ppm" << std::endl;

      //free resources
      delete data;

      return_value = 0;
    }
    else
    {
      std::cerr << "Error opening camera" << std::endl;
      return_value = -1;
    }

    return return_value;
}

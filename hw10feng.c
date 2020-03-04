/**************************************************
* CMPEN 473, Spring 2018, Penn State University
* 
* Homework 10
* Revision V1.1
* On 4/19/2019
* By Boris Feng
* 
***************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include "import_registers.h"
#include "gpio.h"
#include "cm.h"
#include "pwm.h"
#include "spi.h"
#include "io_peripherals.h"
#include "enable_pwm_clock.h"
#include "LSM9DS1.h"
#include "wait_key.h"
#include "transact_SPI.h"
#include "raspicam_wrapper.h"
#include "wait_period.h"
#include "time_difference.h"
#include "pixel_format_RGB.h"


/* setting the RPi3 hardware PWM range */
#define PWM_RANGE 32
#define APB_CLOCK 250000000

#define DEN_PIN   18
#define CS_M_PIN  19
#define CS_AG_PIN 20

#define ROUND_DIVISION(x,y) (((x) + (y)/2)/(y))

int array[4] = {0, 0, 0, 0}; //w, x, a, d
int mode = 1;
int cam = 0;
int fin = 0;
char picture[128][96];
char map_display[34][26];
int rx_1, rx_2, ry_1, ry_2;
int bx_1, bx_2, by_1, by_2;
int mid_x1, mid_y1, mid_x2, mid_y2;
int pos_x, pos_y; 
int prev_x = -1;
int prev_y = -1;
int compass;
int direction;
int quit = 0;
//(7,9) (18,24) half size of 24x32
//upper box left corner
int box_y1 = 5;
int box_x1 = 7;
//lower box right corner
int box_y2 = 20;
int box_x2 = 26;

struct done_flag_t
{
  pthread_mutex_t                 lock;                 /* used to lock the contents to prevent race conditions */
  bool                            done;                 /* set to true to indicate that it is time to shut down */
};

struct image_data_t
{
  pthread_mutex_t                 lock;                 /* used to lock the contents to prevent race conditions */
  bool                            data_ready;           /* set to true when data is valid */
  unsigned char *                 data;                 /* cast to struct pixel_format_RGB to access the pixel data, index via RGB_data[x+y*image_width], total array length is image_height*image_width */
  unsigned int                    image_height;         /* the height of the RGB data */
  unsigned int                    image_width;          /* the width of the RGB data */
};

struct camera_thread_parameter_t
{
  long                            last_execution_time;  /* the execution time consumed in the last period */
  struct image_data_t *           image_data;           /* output from ThreadCamera */
  struct done_flag_t *            done;                 /* input to ThreadCamera */
};

void genMap() {
	for (int i = 0; i < 26; i++) {
		for (int j = 0; j < 34; j++) {
			map_display[j][i] = '+';
		}
	}

	for (int i = 1; i < 25; i++) {
		for (int j = 1; j < 33; j++) {
			map_display[j][i] = ' ';
		}
	}
	
	for (int i = box_y1; i < box_y2 + 1; i++){
			map_display[box_x1][i] = '*';
			map_display[box_x2][i] = '*';
	}
	
	for (int i = box_x1; i < box_x2 + 1; i++){
			map_display[i][box_y1] = '*';
			map_display[i][box_y2] = '*';
	}		
}

void searchMapFoward(char color) {
	for (int i = 0; i < 96; i++) {
		for (int j = 0; j < 128; j++) {
			if (picture[j][i] == color) { 
				if (color == 'R') {
					rx_1 = j;
					ry_1 = i;
				}
				else {
					bx_1 = j;
					by_1 = i;	
				}
				int upper_x, upper_y, lower_x, lower_y;
				upper_x = j + 8;
				upper_y = i + 8;
				if (upper_x > 128)
					upper_x = 127;
				if (upper_y > 96)
					upper_y = 95;
				//printf("upper(%d, %d)\n", upper_x, upper_y);	
				lower_x = j - 8;
				lower_y = i - 8;
				if (lower_x < 0)
					lower_x = 0;
				if (lower_y < 0)
					lower_y = 0;
				//printf("lower(%d, %d)\n", lower_x, lower_y);
				for (int i = lower_y; i < upper_y; i++) {
					for(int j = lower_x; j < upper_x; j++) {
						picture[j][i] = ' ';
					}
				}
				return;
			}
		}				
	}
	printf("%c IS OUT OF BOUNDS!\n", color);
	return;
}

void searchMapBack(char color) {
	for (int i = 95; i >= 0; i--) {
		for (int j = 127; j >= 0; j--) {
			if (picture[j][i] == color) { 
				if (color == 'R') {
					rx_2 = j;
					ry_2 = i;
				}
				else {
					bx_2 = j;
					by_2 = i;	
				}			
				return;
			}
		}
					
	}
	return;
}

void printDirection() {
	if (direction == 0)
		printf("Moving Direction: NE\n");
	else if (direction == 1)
		printf("Moving Direction: NW\n");	
	else if (direction == 2)
		printf("Moving Direction: SE\n");	
	else if (direction == 3)
		printf("Moving Direction: SW\n");
	else if (direction == 4)
		printf("Moving Direction: S\n");
	else if (direction == 5)
		printf("Moving Direction: N\n");
	else if (direction == 6)
		printf("Moving Direction: E\n");
	else if (direction == 7)
		printf("Moving Direction: W\n");
}

int get_pressed_key(void)
{
  struct termios  original_attributes;
  struct termios  modified_attributes;
  int             ch;

  tcgetattr( STDIN_FILENO, &original_attributes );
  modified_attributes = original_attributes;
  modified_attributes.c_lflag &= ~(ICANON | ECHO);
  modified_attributes.c_cc[VMIN] = 1;
  modified_attributes.c_cc[VTIME] = 0;
  tcsetattr( STDIN_FILENO, TCSANOW, &modified_attributes );

  ch = getchar();

  tcsetattr( STDIN_FILENO, TCSANOW, &original_attributes );

  return ch;
}

int searchArray(int elem, int check){
	if (array[elem] == check)
		return 1;
	else
		return 0;
}

void *ThreadCamera(void* arg) {
	struct raspicam_wrapper_handle *  camera_handle;  /* Camera handle */
	struct timespec                   timer_state;    /* used to keep the thread running at the correct rate */
	struct camera_thread_parameter_t *parameter = (struct camera_thread_parameter_t *)arg;
	struct timespec                   execution_start;/* used to determine the execution time of the loop */
	struct timespec                   execution_end;  /* used to determine the execution time of the loop */
	
    /* Open camera */
    printf("Opening Camera\n");
    while(quit == 0){
		if (quit == 1)
			break;
		while (cam == 1 && quit == 0) {
			camera_handle = raspicam_wrapper_create();
			if (camera_handle != NULL) {
				if (raspicam_wrapper_open(camera_handle)) {  

					// allocate memory
					pthread_mutex_lock( &(parameter->image_data->lock) );
					parameter->image_data->data = (unsigned char *)malloc( raspicam_wrapper_getImageTypeSize( camera_handle, RASPICAM_WRAPPER_FORMAT_RGB ) );
					pthread_mutex_unlock( &(parameter->image_data->lock) );

					// set the start time of the time period
					wait_period_initialize( &timer_state );
					if (parameter->image_data->data != NULL)
					{
						pthread_mutex_lock( &(parameter->done->lock) );
						while (!(parameter->done->done))
						{
							pthread_mutex_unlock( &(parameter->done->lock) );

							// start measuring the execution time
							clock_gettime( CLOCK_REALTIME, &execution_start );

							// capture the image
							raspicam_wrapper_grab( camera_handle );
							//printf("Take photo\n");

							// extract the image in rgb format
							pthread_mutex_lock( &(parameter->image_data->lock) );
							raspicam_wrapper_retrieve( camera_handle, parameter->image_data->data, RASPICAM_WRAPPER_FORMAT_IGNORE );
							parameter->image_data->image_height  = raspicam_wrapper_getHeight( camera_handle );
							parameter->image_data->image_width   = raspicam_wrapper_getWidth( camera_handle );
							parameter->image_data->data_ready    = true;
							pthread_mutex_unlock( &(parameter->image_data->lock) );
							
							struct RGB_pixel {
								unsigned char R;
								unsigned char G;
								unsigned char B;
							};
							struct RGB_pixel* pixel;
							unsigned int      pixel_count;
							unsigned int      pixel_index;
							unsigned char     pixel_value;

							pixel = (struct RGB_pixel *)parameter->image_data->data;
							pixel_count = parameter->image_data->image_height * parameter->image_data->image_width;
							for (pixel_index = 0; pixel_index < pixel_count; pixel_index++) {
								if (pixel[pixel_index].R < 245) { //245 for home, 254 for lab
									pixel[pixel_index].R = 0;
								}
								pixel[pixel_index].G = 0;
								if (pixel[pixel_index].B < 245) {
									pixel[pixel_index].B = 0;
								}
							}
							
							unsigned int scaled_height;
							unsigned int scaled_width;
							unsigned int height_index;
							unsigned int width_index;
							unsigned int horizontal_reduction = 10;
							unsigned int vertical_reduction = 10;
							struct RGB_pixel* scaled_data;
								
							scaled_data = pixel;
							scaled_height = parameter->image_data->image_height/vertical_reduction;
							scaled_width  = parameter->image_data->image_width/horizontal_reduction;
								
							for (height_index = 0; height_index < scaled_height; height_index++) {
								for (width_index = 0; width_index < scaled_width; width_index++) {
									unsigned int temp = height_index*scaled_width + width_index;
									scaled_data[temp] = pixel[height_index*scaled_width*vertical_reduction*horizontal_reduction + width_index*horizontal_reduction];
									if (scaled_data[temp].R >= 245) {
										int row = floor(temp / scaled_width);
										int column = temp - (row * scaled_width);
										picture[column][row] = 'R';
									}
									if (scaled_data[temp].B >= 245) {
										int row = floor(temp / scaled_width);
										int column = temp - (row * scaled_width);
										picture[column][row] = 'B';
									}							
								}
							}
							fin = 1;
							// record the execution time
							clock_gettime( CLOCK_REALTIME, &execution_end );
							parameter->last_execution_time = time_difference_us( &execution_start, &execution_end );

							// wait for the next 100ms period
							wait_period( &timer_state, 100*1000 );

							pthread_mutex_lock( &(parameter->done->lock) );
						}
						pthread_mutex_unlock( &(parameter->done->lock) );
					}
					else
					{
					printf( "unable to allocate image data\n" );
					}
				}
				else
				{
				printf( "Error opening camera\n" );
				}
			}
			else
			{
			printf( "Unable to allocate camera handle\n" );
			}		
			if (cam == 0)
				break;
		}
	}
}

void *Actions() {
	while (1) {
      switch (get_pressed_key())
      {
        case 'w':
          if (array[0] == 0)
          {
				printf("Move Foward\n");
				array[0] = 1;				
          }
          break;

		case 'x':
          if (array[1] == 0)
          {
				printf("Move Back\n");
				array[1] = 1;
          }
          break;

		case 'a':
          if (array[2] == 0)
          {
				printf("Move Left\n");
				array[2] = 1;
          }
          break;
          
		case 'd':
          if (array[3] == 0)
          {
				printf("Move Right\n");
				array[3] = 1;
          }
          break;
          
        case 'm':
		  switch (get_pressed_key()) {
			  case '1':
				mode = 1;
				cam = 0;
				printf("Mode 1\n");
				break;
			  case '2':
				mode = 2;
				cam = 1;
				printf("Mode 2\n");
				break;
		  }
		  break;
		  
		case 'q':
		  quit = 1;
		  break;
		}            
	}
}    

int main( void )
{
  volatile struct io_peripherals *io;
  pthread_t	action_handle;
  pthread_t thread_camera_handle;

  struct done_flag_t done = {PTHREAD_MUTEX_INITIALIZER, false};
  struct image_data_t image_data = {PTHREAD_MUTEX_INITIALIZER, false, NULL, 0U, 0U};
  struct camera_thread_parameter_t thread_camera_parameter;
  
  io = import_registers();
  if (io != NULL)
  {
    /* print where the I/O memory was actually mapped to */
    printf( "mem at 0x%8.8X\n", (unsigned int)io );

    enable_pwm_clock( io );

    io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_INPUT; //left & right
    io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_INPUT; //foward & reverse


    /* configure the PWM channels */
    io->pwm.RNG1 = PWM_RANGE;     /* the range value, 32 level steps */
    io->pwm.RNG2 = PWM_RANGE;     /* the range value, 32 level steps */
    io->pwm.CTL.field.MODE1 = 0;  /* PWM mode */
    io->pwm.CTL.field.MODE2 = 0;  /* PWM mode */
    io->pwm.CTL.field.RPTL1 = 1;  /* not using FIFO, but repeat the last byte anyway */
    io->pwm.CTL.field.RPTL2 = 1;  /* not using FIFO, but repeat the last byte anyway */
    io->pwm.CTL.field.SBIT1 = 0;  /* idle low */
    io->pwm.CTL.field.SBIT2 = 0;  /* idle low */
    io->pwm.CTL.field.POLA1 = 0;  /* non-inverted polarity */
    io->pwm.CTL.field.POLA2 = 0;  /* non-inverted polarity */
    io->pwm.CTL.field.USEF1 = 0;  /* do not use FIFO */
    io->pwm.CTL.field.USEF2 = 0;  /* do not use FIFO */
    io->pwm.CTL.field.MSEN1 = 1;  /* use M/S algorithm */
    io->pwm.CTL.field.MSEN2 = 1;  /* use M/S algorithm */
    io->pwm.CTL.field.CLRF1 = 1;  /* clear the FIFO, even though it is not used */
    io->pwm.CTL.field.PWEN1 = 1;  /* enable the PWM channel */
    io->pwm.CTL.field.PWEN2 = 1;  /* enable the PWM channel */
	
	/* set up the SPI parameters */
    io->spi.CLK.field.CDIV = ((ROUND_DIVISION(250000000,4000000))>>1)<<1; /* this number must be even, so shift the LSb into oblivion */
    io->spi.CS.field.CS       = 0;
    io->spi.CS.field.CPHA     = 1;  /* clock needs to idle high and clock in data on the rising edge */
    io->spi.CS.field.CPOL     = 1;
    io->spi.CS.field.CLEAR    = 0;
    io->spi.CS.field.CSPOL    = 0;
    io->spi.CS.field.TA       = 0;
    io->spi.CS.field.DMAEN    = 0;
    io->spi.CS.field.INTD     = 0;
    io->spi.CS.field.INTR     = 0;
    io->spi.CS.field.ADCS     = 0;
    io->spi.CS.field.REN      = 0;
    io->spi.CS.field.LEN      = 0;
	io->spi.CS.field.CSPOL0   = 0;
    io->spi.CS.field.CSPOL1   = 0;
    io->spi.CS.field.CSPOL2   = 0;
    io->spi.CS.field.DMA_LEN  = 0;
    io->spi.CS.field.LEN_LONG = 0;  
	
	thread_camera_parameter.last_execution_time         = 0;
    thread_camera_parameter.image_data                  = &image_data;
    thread_camera_parameter.done                        = &done;
	
	printf("m1: Manual Mode\n");
	printf("m2: Autonomous Mode\n");
    printf("w: Move Foward\n");			
    printf("a: Move Left\n");
    printf("x: Move Back\n");
    printf("d: Move Right\n");
    printf("q: Quit\n");
	genMap();
	
	pthread_create( &action_handle, 0, Actions, NULL);
	pthread_create( &thread_camera_handle, 0, ThreadCamera, (void *)&thread_camera_parameter);
	
    while(1){
		if (quit == 1)
			break;		
		//MODE 1
		do {
			//foward
			while (searchArray(0,1)) {
				io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_OUTPUT;
				GPIO_CLR( &(io->gpio), 25);			
				usleep(300000); //0.3s
				io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_INPUT;
				array[0] = 0;
				break;
			}
			//reverse
			while (searchArray(1,1)) {
				io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_OUTPUT;
				GPIO_SET( &(io->gpio), 25);			
				usleep(300000); //0.3s
				io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_INPUT;
				array[1] = 0;
				break;
			}
			//left
			while (searchArray(2,1)) {
				io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_OUTPUT;
				GPIO_CLR( &(io->gpio), 24);		
				usleep(300000); //0.3s
				io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_INPUT;
				array[2] = 0;
				break;
			}
			//right
			while (searchArray(3,1)) {
				io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_OUTPUT;
				GPIO_SET( &(io->gpio), 24);			
				usleep(300000); //0.3s
				io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_INPUT;
				array[3] = 0;
				break;
			}
		} while (mode == 1 && quit == 0);
		
		//MODE 2
		do {
			while (fin == 1) {
				/*
				for (int i = 0; i < 96; i++) {
					for (int j = 0; j < 128; j++) {
							printf("%c", picture[j][i]);
						}
					printf("\n");
				}
				*/
				
				searchMapFoward('R');
				searchMapBack('R');
				
				//printf("red1: %d, %d\n", rx_1, ry_1);
				//printf("red2: %d, %d\n", rx_2, ry_2);
				//red midpoint
				mid_x1 = (rx_1 + rx_2)/2;
				mid_y1 = (ry_1 + ry_2)/2;			
				//printf("red mid: %d, %d\n", mid_x1, mid_y1);
				
				searchMapFoward('B');
				searchMapBack('B');
				
				//printf("blue1: %d, %d\n", bx_1, by_1);
				//printf("blue2: %d, %d\n", bx_2, by_2);
				//blue mid point
				mid_x2 = (bx_1 + bx_2)/2;
				mid_y2 = (by_1 + by_2)/2;			
				//printf("blue mid: %d, %d\n", mid_x2, mid_y2);
				
				double rads = atan2(mid_y1 - mid_y2, mid_x2 - mid_x1) * 180 / M_PI;		
				//printf("Angle: %lf\n", rads);
				
				if(-15 <= rads && rads <= 15) {
					printf("Orientation: E\n");
					compass = 3;
				}
				else if (15 < rads && rads < 75) {
					printf("Orientation: SE\n");
					compass = 4;
				}
				else if (75 <= rads && rads <= 105) {
					printf("Orientation: S\n");
					compass = 5;
				}
				else if (105 < rads && rads < 165) {
					printf("Orientation: SW\n");
					compass = 6;
				}
				else if (165 <= abs(rads)) {
					printf("Orientation: W\n");
					compass = 7;
				}
				else if (-105 > rads && rads > -165) {
					printf("Orientation: NW\n");
					compass = 8;
				}
				else if (-75 >= rads && rads >= -105) {
					printf("Orientation: N\n");
					compass = 1;
				}
				else if (-15 > rads && rads > -75) {
					printf("Orientation: NE\n");
					compass = 2;
				}
					
				pos_x = floor(((mid_x1 + mid_x2)/2)/4);
				pos_y = floor(((mid_y1 + mid_y2)/2)/4);
				if (prev_x != -1 && prev_y != -1) {				
					//printf("Prev: %d, %d\n", prev_x, prev_y);
					if (pos_y < prev_y) {
						if (pos_x < prev_x)
							direction = 0; //NE
						else if (pos_x > prev_x)
							direction = 1; //NW 
						else
							direction = 5; //N	
					}
					else if (pos_y > prev_y) {
						if (pos_x < prev_x)
							direction = 2; //SE
						else if (pos_x > prev_x)
							direction = 3; //SW
						else
							direction = 4; //S	
					}
					else if (pos_x < prev_x)
						direction = 6; //E	
					else if (pos_x > prev_x)
						direction = 7; //W							
					printDirection();
					float distance = floor(sqrt(pow(pos_x - prev_x, 2) + pow(pos_y - prev_y, 2)));
					printf("Speed: %.2f pixels/sec\n", (distance * 4 * 10)/10);
				}
				
				prev_x = pos_x;
				prev_y = pos_y;						
				printf("Position: %d, %d\n", abs(32 - pos_x), pos_y + 1);
				map_display[32 - pos_x][pos_y + 1] = 'o';	
							
				for (int i = 0; i < 26; i++) {
					for (int j = 0; j < 34; j++) {
							printf("%c ", map_display[j][i]);
						}
					printf("\n");
				}	
				genMap();
				
				for (int i = 0; i < 96; i++) {
					for (int j = 0; j < 128; j++) {
						picture[j][i] = ' ';
					}
				}
				
				//Check if outside box
				//North Border
				if (pos_y < box_y1) {
					printf("Out of box: North Border\n");
					if (compass == 1 || compass == 8 || compass == 2) { //North
						io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_OUTPUT;
						GPIO_SET( &(io->gpio), 25); //reverse
						printf("Move Back\n");			
						usleep(300000); //0.3s
						io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_INPUT;
					}
					else if (compass == 5 || compass == 4 || compass == 6) { //South
						io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_OUTPUT;
						GPIO_CLR( &(io->gpio), 25);	//foward	
						printf("Move Foward\n");	
						usleep(300000); //0.3s
						io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_INPUT;
					}
					else if (compass == 3) { //East
						io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_OUTPUT;
						GPIO_SET( &(io->gpio), 24);	//right	
						printf("Move Right\n");	
						usleep(300000); //0.3s
						io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_INPUT;	
					}
					else if (compass == 7) { //West
						io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_OUTPUT;
						GPIO_CLR( &(io->gpio), 24);	//left
						printf("Move Left\n");		
						usleep(300000); //0.3s
						io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_INPUT;	
					}
				}
				//South Border
				if (pos_y > box_y2) {
					printf("Out of box: South Border\n");
					if (compass == 5 || compass == 4 || compass == 6) { //South
						io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_OUTPUT;
						GPIO_SET( &(io->gpio), 25);	//reverse
						printf("Move Back\n");			
						usleep(300000); //0.3s
						io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_INPUT;
					}
					else if (compass == 1 || compass == 8 || compass == 2) { //North
						io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_OUTPUT;
						GPIO_CLR( &(io->gpio), 25);	//foward
						printf("Move Foward\n");			
						usleep(300000); //0.3s
						io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_INPUT;
					}
					else if (compass == 3) { //East
						io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_OUTPUT;
						GPIO_CLR( &(io->gpio), 24);	//left	
						printf("Move Left\n");		
						usleep(300000); //0.3s
						io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_INPUT;
					}
					else if (compass == 7) { //West
						io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_OUTPUT;
						GPIO_SET( &(io->gpio), 24);	//right	
						printf("Move Right\n");		
						usleep(300000); //0.3s
						io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_INPUT;	
					}	
				}
				//East Border
				if (pos_x > box_x2) {
					printf("Out of box: East Border\n");
					if (compass == 3 || compass == 2 || compass == 4) { //East
						io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_OUTPUT;
						GPIO_SET( &(io->gpio), 25);	//reverse
						printf("Move Back\n");			
						usleep(300000); //0.3s
						io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_INPUT;
					}
					else if (compass == 7 || compass == 6 || compass == 8) { //West
						io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_OUTPUT;
						GPIO_CLR( &(io->gpio), 25);	//foward
						printf("Move Foward\n");			
						usleep(300000); //0.3s
						io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_INPUT;
					}
					else if (compass == 5) { //North
						io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_OUTPUT;
						GPIO_CLR( &(io->gpio), 24);	//left
						printf("Move Left\n");			
						usleep(300000); //0.3s
						io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_INPUT;
					}
					else if (compass == 1) { //South
						io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_OUTPUT;
						GPIO_SET( &(io->gpio), 24);	//right
						printf("Move Right\n");			
						usleep(300000); //0.3s
						io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_INPUT;
					}
				}
				//West Border
				if (pos_x < box_x1) {
					printf("Out of box: West Border\n");
					if (compass == 7 || compass == 6 || compass == 8) { //West
						io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_OUTPUT;
						GPIO_SET( &(io->gpio), 25);	//reverse
						printf("Move Back\n");			
						usleep(300000); //0.3s
						io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_INPUT;
					}
					else if (compass == 3 || compass == 2 || compass == 4) { //East
						io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_OUTPUT;
						GPIO_CLR( &(io->gpio), 25); //foward
						printf("Move Foward\n");				
						usleep(300000); //0.3s
						io->gpio.GPFSEL2.field.FSEL5 = GPFSEL_INPUT;
					}
					else if (compass == 5) { //North
						io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_OUTPUT;
						GPIO_SET( &(io->gpio), 24);	//right
						printf("Move Right\n");			
						usleep(300000); //0.3s
						io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_INPUT;
					}
					else if (compass == 1) { //South
						io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_OUTPUT;
						GPIO_CLR( &(io->gpio), 24);	//left
						printf("Move Left\n");			
						usleep(300000); //0.3s
						io->gpio.GPFSEL2.field.FSEL4 = GPFSEL_INPUT;
					}
				}
				
				fin = 0;			
			}

		} while (mode == 2 && quit == 0);
	}
  }
  else
  {
    ; /* warning message already issued */
  }

  return 0;
}

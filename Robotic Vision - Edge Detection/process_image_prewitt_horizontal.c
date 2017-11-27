#include <stdio.h>
#include <math.h>
#include <X11/Xlib.h>

#define DIM 512


/******************************************************************/
/* This structure contains the coordinates of a box drawn with    */
/* the left mouse button on the image window.                     */
/* roi.x , roi.y  - left upper corner's coordinates               */
/* roi.width , roi.height - width and height of the box           */
/******************************************************************/
extern XRectangle roi;


/******************************************************************/
/* Main processing routine. This is called upon pressing the      */
/* Process button of the interface.                               */
/* image  - the original greyscale image                          */
/* size   - the actual size of the image                          */
/* proc_image - the image representation resulting from the       */
/*              processing. This will be displayed upon return    */
/*              from this function.                               */
/******************************************************************/
void process_image(image, size, proc_img)
unsigned char image[DIM][DIM];
int size[2];
unsigned char proc_img[DIM][DIM];
{
	

	double prewitt_horizontal[3][3] = {{-1.0,-1.0,-1.0},
									   {0.0,0.0,0.0},
									   {1.0,1.0,1.0}};

	double image_temp[DIM][DIM];
	for (int x=0;x<DIM;x++){
		for (int y=0;y<DIM;y++){
			image_temp[x][y] = (double)image[x][y];
		}
	}


	//Mean Gray Scale Value of Image
	double mean_value = 0.0;
	for (int x=0;x<DIM;x++){
		for (int y=0;y<DIM;y++){
			mean_value = mean_value + image_temp[x][y];
		}
	}
	mean_value = mean_value / ((double) (DIM * DIM));
	for (int x=0;x<DIM;x++){
		for (int y=0;y<DIM;y++){
			image_temp[x][y] = image_temp[x][y] - mean_value;
		}
	}

	double smoothing[3][3] = {{1.0,1.0,1.0},
						   {1.0,1.0,1.0},
						   {1.0,1.0,1.0}};
	double image_temp_smoothed[DIM][DIM];
	for (int x=0;x<DIM-2;x++){
		for (int y=0;y<DIM-2;y++){
			int x_s = 0;
			double sum = 0.0;
			for (int i=x;i<x+3;i++){
				int y_s = 0;
				for (int j=y;j<y+3;j++){
					sum = sum + smoothing[x_s][y_s] * image_temp[i][j];
					y_s++;				
				}
				x_s++;
			}
			image_temp_smoothed[x+1][y+1] = sum/9.0;
		}
	}



	// Convolution for Vertical Prewitt Operator
	double maxsum = 0.0;
	for (int x=0;x<DIM-2;x++){
		for (int y=0;y<DIM-2;y++){
			int o_x = 0;
			double sum = 0.0;
			for (int i=x;i<x+3;i++){
				int o_y = 0;
				for (int j=y;j<y+3;j++){
					sum = sum + (image_temp_smoothed[i][j]) * (prewitt_horizontal[o_y][o_x]) ; //9 position
					if(maxsum<sum){
						maxsum = sum;
					}
					o_y++;
				}
				o_x++;
			}

			if (sum>255.0){
				proc_img[x+1][y+1] = 255.0;
			}else if (sum<0.0){
				proc_img[x+1][y+1] = 0.0;
			}
			else{
				proc_img[x+1][y+1] = sum;
			}
		}
	}

	// for (int x=0;x<DIM;x++){
	// 	for (int y=0;y<DIM;y++){
	// 		proc_img[x][y] = image_temp_smoothed[x][y];
	// 	}
	// }

}



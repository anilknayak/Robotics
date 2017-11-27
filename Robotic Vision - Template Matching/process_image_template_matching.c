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
	int template_x = roi.x;
	int template_y = roi.y;
	int template_w = roi.width;
	int template_h = roi.height;
	double template[template_w][template_h];

	double image_temp[DIM][DIM];
	double proc_img_temp[DIM][DIM];
	double SSD_I[DIM][DIM];
	double Cross_Corr_I[DIM][DIM];
	

	//Image [image_temp]
	for (int x=0;x<DIM;x++){
		for (int y=0;y<DIM;y++){
			image_temp[x][y] = (double)image[x][y];
		}
	}

	//Template [template]
	if(template_w>0 && template_h>0){
		for (int x=template_x;x<(template_w+template_x);x++){
			for (int y=template_y;y<(template_h+template_y);y++){
				template[x-template_x][y-template_y] = image_temp[x][y];
			}
		}
	}else{
		printf("Please Select Templete by dragging a box in Image before clicking the process Button. \n");
	}

	// Result Image [proc_img_temp]
	for (int x=0;x<DIM;x++){
		for (int y=0;y<DIM;y++){
			proc_img_temp[x][y] = 255.0;
			SSD_I[x][y] = 255.0;
			Cross_Corr_I[x][y] = 255.0;
		}
	}

	//######################################################
	//########## Mean of Image and Template ################
	//######################################################
	//Mean Gray Scale Value of Image
	double mean_value = 0.0;
	for (int x=0;x<DIM;x++){
		for (int y=0;y<DIM;y++){
			mean_value = mean_value + image_temp[x][y];
		}
	}
	mean_value = mean_value / ((double) (DIM * DIM));
	
	//Mean Gray Scale Value of Template
	double mean_value_t = 0.0;
	for (int x=0;x<(template_w);x++){
		for (int y=0;y<(template_h);y++){
			mean_value_t = mean_value_t + template[x][y];
		}
	}
	mean_value_t = mean_value_t / ((double) (template_w * template_h));
	//######################################################
	//########## Mean of Image and Template ################
	//######################################################

	//######################################################################
	//########## STD, Substraction of Mean of Image and Template ################
	//######################################################################
	//Substracting Mean Graysclae Value from Image
	double standard_deviation_image = 0.0;
	double standard_deviation_image_sum = 0.0;
	for (int x=0;x<DIM;x++){
		for (int y=0;y<DIM;y++){
			double pixel_sub = image_temp[x][y] - mean_value;
			// image_temp[x][y] = pixel_sub;
			standard_deviation_image_sum = standard_deviation_image_sum + (pixel_sub*pixel_sub);
		}
	}
	standard_deviation_image = sqrt(standard_deviation_image_sum);
	
	//Substracting Mean Graysclae Value from Template
	double standard_deviation_image_t = 0.0;
	double standard_deviation_image_sum_t = 0.0;
	for (int x=0;x<template_w;x++){
		for (int y=0;y<template_h;y++){
			double pixel_val = template[x][y] - mean_value_t;
			// template[x][y] = pixel_val;
			standard_deviation_image_sum_t = standard_deviation_image_sum_t + (pixel_val*pixel_val);
		}
	}
	standard_deviation_image_t = sqrt(standard_deviation_image_sum_t);
	//######################################################################
	//########## STD, Substraction of Mean of Image and Template ################
	//######################################################################


	//######################################################################
	//########## Itensity Normalization of Image and Template ################
	//######################################################################
	//Intensity Normalization on whole Image and whole Template
	//till here I have mean_value of image and mean_value_t of tempplate

	//Intensity Normalization Image
	for (int x=0;x<DIM;x++){
		for (int y=0;y<DIM;y++){
			image_temp[x][y] = (image_temp[x][y]-mean_value) / standard_deviation_image;
		}
	}

	//Intensity Normalization Template
	for (int x=0;x<template_w;x++){
		for (int y=0;y<template_h;y++){
			template[x][y] = (template[x][y]-mean_value_t) / standard_deviation_image_t;
		}
	}

	//######################################################################
	//########## Itensity Normalization of Image and Template ################
	//######################################################################

	

	double max_sum = 0.0;
	int x_center = (int) template_w/2;
	int y_center = (int) template_h/2;
	int number_of_pixel = template_w*template_h;
	int threshold = (int)number_of_pixel*0.5;
	
	//Convolution for Horizontal Template 
	double max_cc = 0.0;
	double max_ssd = 0.0;
	int count = 0;


	for (int x=0;x<DIM-template_w;x++){
		for (int y=0;y<DIM-template_h;y++){
			
			// Prepare Image Portion
			double image_portion[template_w][template_h];
			for (int n=x;n<(x+template_w);n++){
				for (int m=y;m<(y+template_h);m++){
					image_portion[n-x][m-y] = image_temp[n][m];
				}
			}

			//######################################################################
			//########## Local Mean and Standard Deviation of Image#############
			//######################################################################
			// double mean_value = 0.0;
			// for (int x=0;x<template_h;x++){
			// 	for (int y=0;y<DIM;y++){
			// 		mean_value = mean_value + image_portion[x][y];
			// 	}
			// }
			// mean_value = mean_value / ((double) (DIM * DIM));
			// //Substracting Mean Graysclae Value from Image
			// double standard_deviation_image = 0.0;
			// double standard_deviation_image_sum = 0.0;
			// for (int x=0;x<template_w;x++){
			// 	for (int y=0;y<template_h;y++){
			// 		double pixel_sub = image_portion[x][y] - mean_value;
			// 		// image_portion[x][y] = pixel_sub;
			// 		standard_deviation_image_sum = standard_deviation_image_sum + (pixel_sub*pixel_sub);
			// 	}
			// }
			// for (int x=0;x<template_w;x++){
			// 	for (int y=0;y<template_h;y++){
			// 		image_portion[x][y] = (image_portion[x][y]-mean_value) / standard_deviation_image;
			// 	}
			// }
			//######################################################################
			//######################################################################
			//######################################################################


			// Prepare Templete Portion
			// Already Prepared
			// Then Find Cross Correlation
			double SSD = 0.0; //Highest Score is the Correct Match
			double CC = 0.0;
			double MAX_PIXEL = 0.0;
			double SUM_ALL_ABS = 0.0;

			for (int i=0;i<template_w;i++){
				for (int j=0;j<template_h;j++){
					SSD = SSD + ((image_portion[i][j]-template[i][j]) * (image_portion[i][j]-template[i][j]));
					CC = CC + (image_portion[i][j]*template[i][j]);
					double substraction_abs = fabs(image_portion[i][j]-template[i][j]);
					if (MAX_PIXEL<substraction_abs){
						MAX_PIXEL = substraction_abs;
					}
					SUM_ALL_ABS = SUM_ALL_ABS + substraction_abs;


				}
			}
			double CC_V = CC ; 
			if(max_cc < CC_V){
				max_cc = CC_V;
			}
			if(max_ssd < SSD){
				max_ssd = SSD;
			}

			Cross_Corr_I[x+x_center][y+y_center] = CC_V;
			SSD_I[x+x_center][y+y_center] = SSD;



		}
	}

	double threshold_cc = (int)((max_cc * 0.95)*100)/200.0;
	double threshold_ssd = 1.0;
	printf("MAX CC %lf MAX SSD %lf threshold_cc %lf \n",max_cc,max_ssd,threshold_cc);
	
	for (int x=0;x<DIM;x++){
		for (int y=0;y<DIM;y++){

			if(Cross_Corr_I[x][y] >= threshold_cc){ //
				proc_img[x][y] = 0.0;
			}else{
				proc_img[x][y] = 255.0;
			}

			// if(SSD_I[x][y] > threshold_ssd){ //
			// 	proc_img[x][y] = 0.0;
			// }else{
			// 	proc_img[x][y] = 255.0;
			// }

			
		}
	}
	
}



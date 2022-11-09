////////////////////////////////////////////////////////////////
// Image management utilities for Time-Lapse Fusion
//
// Copyright (C) 2012, Francisco J. Estrada
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// This module provides:
//
// Image storage data structures
// Pyramid data structures
// .ppm reading/writing
// Simple filtering (separable kernels)
// Feature maps: Contrast, Saturation, well-exposedness
//
////////////////////////////////////////////////////////////////

#ifndef __imageProc_header

#define __imageProc_header

#include<stdio.h>
#include<stdlib.h>
#include<malloc.h>
#include<math.h>
#include<string.h>	// Seriously! needed by memcpy()!
#include<errno.h>

#define PI 3.14159265354

// Simple data structure to contain image data. All image data
// will be stored as double-precision floating point, and only
// during image output will it get converted to uint8.
// RGB input is automatically split into 3 layers and
// each layer's data is a simple chunk of memory where data
// for the layer is stored in row-major order.
struct image{
 double *layers[3];
 int sx,sy;
 int nlayers;
};

// Simple data structure for image pyramids. Contains an array
// of pointers to image structures and the number of levels
// in the pyramid.
struct pyramid{
 struct image **images;
 int levels;
};

// Simple filter kernel structure. Contains a pointer to a
// 1D filter's entries, and the size and half-size of
// the kernel. Kernels are always odd length, and the
// half size is rounded down.
struct kernel{
 double *taps;
 int size;
 int halfsize;
};


// Function declarations

// Filter kernels and simple filtering
struct kernel *GaussKernel(double sigma);				// Create a 1D Gaussian Kernel
struct kernel *doGKernel(double sigma);					// Create a derivative of Gaussian kernel
void deleteKernel(struct kernel *k);					// Free memory allocated to a kernel
struct image *convolve_x(struct image *im, struct kernel *k);		// Filter image along the x direction
struct image *convolve_y(struct image *im, struct kernel *k);		// Filter image along the y direction

// Image feature computations
struct image *gradient(struct image *im, double sigma);				// Compute the derivatives Ix and Iy using
										// a doG filter with the specified sigma.
void nonMaxSuppression(struct image *grad);					// Perform non-max suppresion on gradient map
void thresholdGradient(struct image *grad, double thresh);			// Gradient thresholding
struct image *opticalFlow(struct image *im1, struct image *im2, double sig);    // Computes and returns the optical flow components ux, uy
  										// for the two frame im1 at time t, and im2 at time t+1,
										// using a Gaussian kernel with specified sigma for smoothing. 
struct image *contrast(struct image *im, double alpha);				// Compute a contrast map
struct image *saturation(struct image *im, double alpha);			// Compute a saturation map
struct image *exposedness(struct image *im, double alpha);			// Compute a well-exposedness map
struct image *timediff(struct image *im, struct image *old, double alpha);	// Temporal derivative map
struct image *computeWeightMap(struct image *im, struct image *old, double alphaC, double alphaS, double alphaE, double alphaT);
										// Compute the image's weight map
										// for contrast, saturation,
										// well-exposedness, and temporal derivative

// Image operations
void rgb2hsv(double R, double G, double B, double *Hu, double *S, double *V);
void hsv2rgb(double H, double S, double V, double *R, double *G, double *B);
struct image *newImage(int sx, int sy, int layers);		// Create a new empty image (sx x sy x layers)
struct image *imageFromBuffer(unsigned char *buf, int sx, int sy, int nlayers); // Create an image structure from a frame buffer
unsigned char *bufferFromIm(struct image *im);			// Create a frame buffer from an image structure
struct image *copyImage(struct image *im);			// Make a copy of an image
void deleteImage(struct image *im);				// Free an image's data
double imMax(struct image *im);					// Max value in an image
double imMin(struct image *im);					// Min value over the image
struct image *desaturate(struct image *im);			// Return a grayscale version of the input colour image
void pointwise_add(struct image *im1, struct image *im2);	// Add 2 images, im1=im1+im2
void pointwise_sub(struct image *im1, struct image *im2);	// Sub 2 images, im1=im1-im2
void pointwise_mul(struct image *im1, struct image *im2);	// Mult. 2 images, im1=im1.*im2
void pointwise_div(struct image *im1, struct image *im2);	// Divide 2 images, im1=im1./im2
void pointwise_pow(struct image *im1, double p);		// Elemen-wise im=im.^p
void image_scale(struct image *im, double k);			// Multiply image by k
void normalize(struct image *im);				// Normalize image to [0,1]
struct image *resize(struct image *im, int sx, int sy);		// Resize with bilinear interp.

// Image pyramid management
struct pyramid *LaplacianPyr(struct image *im, int levels);	// Make a Laplacian pyramid
struct pyramid *GaussianPyr(struct image *im, int levels);	// Make a Gaussian pyramid
struct pyramid *weightedPyr(struct pyramid *lPyr, struct pyramid *gPyr);	// Input Laplacian
								// pyramid is weights using the
								// input Gaussian pyramid
struct image *collapsePyr(struct pyramid *pyr);			// Collapse pyramid
void deletePyramid(struct pyramid *pyr);			// De-allocate pyramid data

// Image I/O  functions
struct image *readPPM(const char *name);		// Read a PPM image from file
int writePPM(const char *name, struct image *im);	// Write PPM image to file

#endif


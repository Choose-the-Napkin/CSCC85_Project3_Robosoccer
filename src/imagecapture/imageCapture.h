/***************************************************************
 CSC C85 - UTSC RoboSoccer image processing core

 This file contains the headers for all functions in the image
 processing code. This part of the robo-soccer code handles
 camera input, blob detection/tracking, and user input via
 the keyboard as well as OpenGL display.

 You *DO NOT* have to look at the code here or in imageCapture.c,
 however, you can (if you think it is useful) access the data
 stored for each blob, or even the images obtained by this
 code to use within your AI.

 If you wish to do that, look at the definition of the blob
 data structure, and drop by my office to discuss your plans.

 Image processing code - F. Estrada, Summer 2013
 Updated by F. Estrada, Jul 2022
****************************************************************/

#ifndef __imageCapture_header

#define __imageCapture_header

// Image processing library from time-lapse fusion code
#include"imageProc.h"

// Open GL libs
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <time.h>

// libraries from luvcview
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <string.h>
#include <pthread.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <X11/Xlib.h>
#include "v4l2uvc.h"
/* Fixed point arithmetic */
#define FIXED Sint32
#define FIXED_BITS 16
#define TO_FIXED(X) (((Sint32)(X))<<(FIXED_BITS))
#define FROM_FIXED(X) (((Sint32)(X))>>(FIXED_BITS))

#define INCPANTILT 64 // 1°

static char version[] = "RoboSoccerEV3 V2.0.2022";

struct blob{
        int label;		        // Label in the labels image
        int blobId;		        // Unique blobId
        double cx;		        // Current location
        double cy;		
        double vx;		        // [vx, vy] motion vector for the current frame
        double vy;
        double mx,my;		    // Heading direction - Last non-zero motion direction - unit vector!
        double dx,dy;		    // Direction vector from blob shape - points in the direction of long axis
        int size;		        // Size in pixels
        int x1,y1,x2,y2;	    // Bounding box (top-left, bottom-right)
        double R,G,B;		    // Average colour of pixels in the blob
        double H,S,V;           // HSV components of blob color
        int idtype;		        // Integer set to the id of this blob: 0->ball, 1->blue bot, 2->red bot
        struct blob *next;	    // If needed for linked lists of blobs
        int updated;		    // Update flag - used by the image processing loop - don't change it!
        double adj_Y[2][2];	    // Y offset adjustment from image capture calibration process
};

struct displayList{
        int type;
        int x1,y1;
        int x2,y2;
        double R,G,B;
        struct displayList *next;
};

// Startup
int imageCaptureStartup(char *devName, int rx, int ry, int own_col, int ai_mode);

// OpenGL, GLUT, and main loop
void initGlut(char* winName);
void kbHandler(unsigned char key, int x, int y);
void kbUpHandler(unsigned char key, int x, int y);
void WindowReshape(int w, int h);
void FrameGrabLoop(void);

// Webcam setup and frame capture
void yuyv_to_rgb (struct vdIn *vd, int sx, int sy);
struct vdIn *initCam(const char *videodevice, int width, int height);
void getFrame(struct vdIn *videoIn, int sx, int sy);
void closeCam(struct vdIn *videoIn);

// Frame processing
double *getH(void);
void fieldUnwarp2();
void bgSubtract3();
void releaseBlobs(struct blob *blobList);
struct image *blobDetect2(struct blob **blob_list, int *nblobs);
struct image *renderBlobs(struct image *labels, struct blob *list);
void drawLine(int x1, int y1, double vx, double vy, double scale, double R, double G, double B, struct image *dst);
void drawBox(int x1, int y1, int x2, int y2, double R, double G, double B, struct image *dst);
void drawCross(int mcx, int mcy, double R, double G, double B, int len, struct image *dst);
void drawCross_buf(int mcx, int mcy, double R, double G, double B, int len, unsigned char *dst);
#endif


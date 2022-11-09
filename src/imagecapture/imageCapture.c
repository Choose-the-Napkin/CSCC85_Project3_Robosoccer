/************************************************************************************
 CSC C85 - UTSC RoboSoccer image processing core

 Implementation of the image processing functionality for robo-soccer.
 This code initializes the web-cam, obtains a rectified playfield from
 the corners specified by the user, detects the bots and ball, and
 processes the user's keyboard input as well as the OpenGL display.

 You *DO NOT* have to look at the code here.

 However, you do want to read imageCapture.h to see what data is stored
 in the blob data structure. You will need to access it (at the very least
 you may want to use the direction vector stored in the bot's and opponent's
 blobs).

 * DO NOT SPEND TIME ON THIS CODE UNTIL YOU HAVE A MOSTLY WORKING
   AI CORE that at the very least can drive the bot toward the ball
   to kick it.

 For the curious - You can learn the following from studying code in this
  file:
 
  - Setting up, initializing, and using a webcam to get video frames
    (courtesy of the folks who created uvccapture)
  - Computing a mapping from points on a plane (the field of play) to
    points on a different plane (the image of the rectified field).
  - Processing an image frame to detect coloured blobs
  - Estimating basic blob properties
  - Updating an OpenGL interactive display

 Image processing code - F. Estrada, Summer 2013
 Updated - F. Estrada, Jul. 2022.

 Code for initializing the camera and grabbing a frame adapted                    
                    from luvcview and uvccapture.                                
                    luvcview and uvccapture by Laurent Pinchart & Michel Xhaard  
		    This code is distributed under GPL V2.0. Therefore:
                                                                                  
 As per the GLP V2.0 license terms. This code is hereby distributed freely. You   
  are free to modify and redistribute this code under the provisions of the      
  GPL V2 or newer. A copy of the license should be included with this code,      
  but should this not be the case, please consult                                 
  http://www.gnu.org/licenses/gpl-2.0.html                                       
************************************************************************************/

#include "imageCapture.h"
#include "svdDynamic.h"
#include "../roboAI.h"
#include <time.h>

#define MIN_BLOB_SIZE 500         // Minimum blob size allowed by blobDetect2()

// ** DO NOT MODIFY THESE GLOBAL VARIABLE DEFINITIONS **
// OpenGL data
GLuint texture;                   // GL texture identifier 
int windowID;                     // Glut window ID (for display)
int Win[2];                       // window (x,y) size
int sx,sy;		                  // Image size - set by the webcam init function
char line[1024];
int heightAdj=1;                  // Perspective adjustment, toggle on'off with '1' in the U.I.

// Webcam image data and processed image components
struct vdIn *webcam;                  // The input video device
struct image *proc_im;                // Image structure for processing
unsigned char bigIm[1024*1024*3];     // Big texture to hold our image for OpenGL
unsigned char *fieldIm=NULL;          // Unwarped field 
unsigned char *bgIm=NULL;             // Background image
unsigned char *frame_buffer=NULL;     // Frame Buffer - camera frames are stored here

// Global image processing parameters
int gotbg=0;				          // Background acquired flag
int gotCol=0;                         // Colour calibration acquired flag
int toggleProc;				          // Toggles processing on/off
double Mcorners[4][2];			      // Manually specified corners for the field
double Mhues[4];                      // Reference hue value for agent id
double Mrgb[4][3];                    // Corresponding reference RGB values
double mcx,mcy;				          // Crosshair x and y
int cornerIdx;				          // Index for manual corner selection
int colourIdx;                        // Index for blob colour calibration
double bgThresh=2500;			      // Background subtract threshold
double colThresh=.5;			      // Saturation threshold
double colAngThresh=.975;		      // Colour angle threshold
struct blob *blobs=NULL;		      // Blob list for the current frame * DO NOT USE THIS LIST *
double *H = NULL;			          // Homography matrix for field rectification
double *Hinv = NULL;                  // Inverse Homography matrix
int frameNo=0;				          // Frame id
time_t time1,time2;		    	      // timing variables
int printFPS=0;				          // Flag that controls FPS printout

// Robot-control data
struct RoboAI skynet;			                // Bot's AI structure
int DIR_L=0, DIR_R=0, DIR_FWD=0, DIR_BACK=0;	// Toggle flags for manual robot motion
int doAI=0;				                        // Toggle switch for AI processing
int AIMode = 0;				                    // AI mode, as per command line
int botCol = 0;				                    // Own bot's colour as per command line
double adj_Y[2][2];			                    // Y coordinate adjustment for robot position
double ref_Y[2];                                // Y reference coorditanes from ball for perspective adjustment
int got_Y=0;				                    // Got Y adjustment
// ** END OF GLOBAL DATA DECLARATIONS ** //

/*********************************************************************
Image processing setup and frame processing loop
**********************************************************************/
int imageCaptureStartup(char *devName, int rx, int ry, int own_col, int AI_mode)
{
 ///////////////////////////////////////////////////////////////////////////////////
 //
 // This is the entry point for the image processing code. It is called by 
 // roboSoccer.c
 //
 // Input args:
 //	- devName: Linux device name for the webcam, usually /dev/video0, or /dev/video1
 //	- rx, ry: Requested image resolution, typically 720, 1280
 //	- own_col: Color of the bot controlled by this program (passed on from command line)
 //	- AI_mode: Penalties, Follow the Ball, or Robo-Soccer (passed on from command line)
 //
 // This function performs the following tasks:
 //  - Initializes the webcam and opens the video input device
 //  - Sets up and opens an OpenGL window for image display
 //  - Initializes the AI data structure
 //  - Calls the image processing main loop
 //  
 // Returns:
 //     - Hopefully it doesn't! (glutMainLoop() exits without returning here)
 //	    - But, -1 if there is a problem initializing the video capture device
 //	      or setting up OpenGL
 //
 ///////////////////////////////////////////////////////////////////////////////////
 toggleProc=0;
 AIMode=AI_mode;
 botCol=own_col;

 fprintf(stderr,"Camera initialization!\n");
 // Initialize the webcam
 webcam=initCam(devName,rx,ry);
// webcam=initCam(devName,640,480);
 if (webcam==NULL)
 {
  fprintf(stderr,"Unable to initialize webcam!\n");
  return -1;
 }
 sx=webcam->width;
 sy=webcam->height;
 fprintf(stderr,"Camera initialized! grabbing frames at %d x %d\n",sx,sy);
 
 // Done, set up OpenGL and call particle filter loop
 fprintf(stderr,"Entering main loop...\n");
 Win[0]=800;
 Win[1]=800;

 // Get initial time for FPS
 time(&time1);
 
 // Initialize the AI data structure with the requested mode
 setupAI(AIMode,botCol, &skynet);
 adj_Y[0][0]=-1e6;          
 adj_Y[0][1]=-1e6;
 adj_Y[1][0]=-1e6;
 adj_Y[1][1]=-1e6;
 ref_Y[0]=-1e6;
 ref_Y[1]=-1e6;
 
 // Allocate memory for the frame buffer
 fprintf(stderr,"Buffer allocation for images\n");
 frame_buffer = (unsigned char *)calloc (webcam->height*webcam->width * 3, sizeof(unsigned char));
 fieldIm = (unsigned char *)calloc (webcam->height*webcam->width * 3, sizeof(unsigned char));
 bgIm = (unsigned char *)calloc (webcam->height*webcam->width * 3, sizeof(unsigned char));
 if (!frame_buffer||!fieldIm||!bgIm)
 {
  fprintf(stderr,"imageCaptureStartup(): Can not allocate memory for image buffers.\n");
  return 0;
 }
 
 initGlut(version);
 glutMainLoop();

 return 0;
}

void FrameGrabLoop(void)
{
 ///////////////////////////////////////////////////////////////////
 //
 // This is the frame processing loop. It is called once per OpenGL
 // frame update, and does all the heavy lifting:
 //
 // - Obtains a video frame from the web cam
 // - Allows the user to capture calibration data: Corners of the
 //     playfield, bot height calibration, thresholds and colour values.
 // - Processes the video frame to extract the play field and 
 //   any blobs therein (see blobDetect2()).
 // - Calls the main AI processing function to allow your bot to
 //   plan and execute its actions based on the video data.
 // - Refreshes the video display - what gets displayed depends
 //   on whether the game is on, or whether the image processing
 //   setup is being carried out.
 //
 ////////////////////////////////////////////////////////////////////
  // OpenGL variables. Do not remove
  static int frame=0;
  char line[1024];
  int ox,oy,i,j;
  double *tmpH;
  unsigned char *big;
  unsigned char *tmp;
  struct displayList *dp;
  struct image *t1, *t2, *t3;
  struct image *labIm, *blobIm;
  static int nblobs=0;
  double *U, *s, *V, *rv1;
  FILE *f;
  double R,G,B,Hu,Sa,Va;

  /***************************************************
   Grab the current frame from the webcam
  ***************************************************/
  big=&bigIm[0];
  getFrame(webcam,sx,sy);
  ox=420;
  oy=1;

  if (toggleProc==4)        // Colour calibration step
  {
   toggleProc=1;
   R=(double)(*(frame_buffer+(((int)mcx+((int)(mcy)*sx))*3)+0));
   G=(double)(*(frame_buffer+(((int)mcx+((int)(mcy)*sx))*3)+1));
   B=(double)(*(frame_buffer+(((int)mcx+((int)(mcy)*sx))*3)+2));
    
   rgb2hsv(R/255.0,G/255.0,B/255.0,&Hu,&Sa,&Va);
   Mhues[colourIdx]=Hu;
     
   // Obtain reference RGB value - a pure colour with the given HUE, S=1, V=1;
   hsv2rgb(Hu,1.0,1.0,&R,&G,&B);     // Disable to keep original RGB
   Mrgb[colourIdx][0]=R;
   Mrgb[colourIdx][1]=G;
   Mrgb[colourIdx][2]=B;

   colourIdx++;
   fprintf(stderr,"Recording colour value #%d at %f,%f has RGB=%f,%f,%f, hue=%f\n",colourIdx,mcx,mcy,R,G,B,Hu);
   if (colourIdx==4) 
   {
     f=fopen("colours.dat","w");
     fwrite(&Mhues[0],4*sizeof(double),1,f);
     fwrite(&Mrgb[0][0],4*3*sizeof(double),1,f);
     fwrite(&bgThresh,sizeof(double),1,f);
     fwrite(&colAngThresh,sizeof(double),1,f);
     fwrite(&colThresh,sizeof(double),1,f);
     fclose(f);
     fprintf(stderr,"Saved colour calibration data.\n");
     toggleProc=0;	// Done!
     gotCol=1;
   }
  }
  
  /////////////////////////////////////////////////////////////////////////
  // What happens in this loop depends on a global variable that changes in
  // response to user keyboard commands. The variable 'toggleproc' has the
  // following values:
  // - 0 : Normal frame processing loop - Used during the game to get
  //                                      and update blob data.
  // - 1 : Used for colour calibration ('c' on the GUI)
  // - 2 : Used for corner capture ('m' on the GUI)
  /////////////////////////////////////////////////////////////////////////
  if (toggleProc==2||toggleProc==1)				
  {
    // Display crosshairs for corner capture or for colour calibration
    drawCross_buf(mcx,mcy,0,255,64,15,frame_buffer);
    drawCross_buf(mcx-1,mcy,0,255,64,15,frame_buffer);
    drawCross_buf(mcx+1,mcy,0,255,64,15,frame_buffer);
    drawCross_buf(mcx,mcy+1,0,255,64,15,frame_buffer);
    drawCross_buf(mcx,mcy+1,0,255,64,15,frame_buffer);
  }
  else
  {
   // toggleProc is zero - check if we should compute H and background image
   if (cornerIdx==4&&H==NULL)
   {
    // We have 4 corners but have not yet obtained the H matrix. Compute H and
    // obtain the background image. Cache the background along with the H
    // matrix for future use.
    fprintf(stderr,"Computing homography and acquiring background image\n");
    H=getH();
    
    // Compute Hinv
    U=NULL;
    s=NULL;
    V=NULL;
    rv1=NULL;
    Hinv=(double *)calloc(9,sizeof(double));
    SVD(H,3,3,&U,&s,&V,&rv1);
    InvertMatrix(U,s,V,3,Hinv);
    free(U);
    free(s);
    free(V);
    
    // Get background image - Here we are using image data structures that store pixel data as dloating point
    // values - it's necessary overhead because we are computing an average over frames, can't be done in
    // unsigned char. 
    t2=newImage(sx,sy,3);
    for (i=0;i<25;i++)
    {
     getFrame(webcam,sx,sy);
     t1=imageFromBuffer(frame_buffer,sx,sy,3);
     pointwise_add(t2,t1);
     deleteImage(t1);
    }
    image_scale(t2,1.0/25.0);

    for (int jj=0;jj<t2->sy;jj++)
     for (int ii=0;ii<t2->sx;ii++)
     {
      *(bgIm+((ii+(jj*t2->sx))*t2->nlayers)+0)=(unsigned char)((*(t2->layers[0]+(ii+(jj*t2->sx)))));
      *(bgIm+((ii+(jj*t2->sx))*t2->nlayers)+1)=(unsigned char)((*(t2->layers[1]+(ii+(jj*t2->sx)))));
      *(bgIm+((ii+(jj*t2->sx))*t2->nlayers)+2)=(unsigned char)((*(t2->layers[2]+(ii+(jj*t2->sx)))));
     }

    deleteImage(t2);
    gotbg=1;
    time(&time1);
    frameNo=0;

    // Cache calibration data - Homography + background image
    f=fopen("Homography.dat","w");
    fwrite(H,9*sizeof(double),1,f);
    fwrite(Hinv,9*sizeof(double),1,f);
    fwrite(bgIm,sx*sy*3*sizeof(unsigned char),1,f);
    fclose(f);
        
   }
  }

  ////////////////////////////////////////////////////////////////////
  // If we have a homography, detect blobs and call the AI routine
  ////////////////////////////////////////////////////////////////////
  labIm=blobIm=NULL;
  if (H!=NULL) 
  {
   ///////////////////////////////////////////////////////////////////
   //
   // If we have the H matrix, we must do the following:
   // - Rectify the playfield into a rectangle
   // - Perform background subtraction
   // - Detect colour blobs
   // - Call the AI main function to do its work
   // - Display the blobs along with information passed back from
   //   the AI processing code.
   //////////////////////////////////////////////////////////////////

   bgSubtract3();        
   fieldUnwarp2();
   labIm=blobDetect2(&blobs,&nblobs);
      
   // At this point we have a bunch of blobs which should have fairly uniform hue, connected, and not background.
   // What happens next depends on the status of the doAI variable - initially set to zero. If it's still zero,
   // nothing more happens, blobs are rendered 'as is', and we move on to the next frame.
   // doAI=1 means we're using the AI's main processing loop - which will identify and track blobs (setting their
   //  id values so the rendering code knows what to do with them) and carry out whatever actions the soccer
   //  code determines are needed.
   // doAI=2 is the loop used for bot-height calibration, it still calls the function that identifies blobs, so
   //  the blob ids are also set.   
   if (blobs)
   {
    if (doAI==1) skynet.runAI(&skynet,blobs,NULL);
    else if (doAI==2) skynet.calibrate(&skynet,blobs);
    blobIm=renderBlobs(labIm,blobs);
    // Render anything in the display list
    dp=skynet.DPhead;
    while (dp)
    {
      if (dp->type==0)
      {
        drawBox(dp->x1-2,dp->y1-2,dp->x1+2,dp->y1+2,dp->R,dp->G,dp->B,blobIm);
        drawBox(dp->x1-1,dp->y1-1,dp->x1+1,dp->y1+1,dp->R,dp->G,dp->B,blobIm);
        drawBox(dp->x1-0,dp->y1-0,dp->x1+0,dp->y1+0,dp->R,dp->G,dp->B,blobIm);
      }
      else
      {
        drawLine(dp->x1,dp->y1,dp->x2-dp->x1,dp->y2-dp->y1,1,dp->R,dp->G,dp->B,blobIm);
      }
      dp=dp->next;
    }
   }
   deleteImage(labIm);
  }
  
  ////////////////////////////////////////////////////////////////////
  // Render whatever we are going to display onto the texture image
  // buffer used by OpenGL
  //////////////////////////////////////////////////////////////////// 
  if (H==NULL||toggleProc>0||gotCol==0)
  {
   // We don't have corners, or colour reference values for blobs, display input image.
   double ii,jj,dx,dy;
   dx=(double)sx/1024.0;
   dy=(double)sy/768.0;
#pragma omp parallel for schedule(dynamic,16) private(ii,jj,i,j)
   for (j=0;j<768;j++)
    for (i=0;i<1024;i++)
    {
     ii=i*dx;
     jj=j*dy;
     *(big+((i+((j+128)*1024))*3)+0)=*(frame_buffer+(((int)ii+(((int)jj)*sx))*3)+0);
     *(big+((i+((j+128)*1024))*3)+1)=*(frame_buffer+(((int)ii+(((int)jj)*sx))*3)+1);
     *(big+((i+((j+128)*1024))*3)+2)=*(frame_buffer+(((int)ii+(((int)jj)*sx))*3)+2);
    }
  }
  else if (blobIm==NULL)
  {
   // We have calibration from H but no blob image (possible if no
   // agents are on the field at the moment or the image processing
   // thresholds are improperly set.
   // Copy the rectified, background subtracted field image for display
   double ii,jj,dx,dy;
   dx=(double)sx/1024.0;
   dy=(double)sy/768.0;
#pragma omp parallel for schedule(dynamic,16) private(ii,jj,i,j)
    for (j=0;j<768;j++)
     for (i=0;i<1024;i++)
     {
      ii=i*dx;
      jj=j*dy;
      *(big+(((i)+((j+128)*1024))*3)+0)=*(fieldIm+(((int)ii+(((int)jj)*sx))*3)+0);
      *(big+(((i)+((j+128)*1024))*3)+1)=*(fieldIm+(((int)ii+(((int)jj)*sx))*3)+1);
      *(big+(((i)+((j+128)*1024))*3)+2)=*(fieldIm+(((int)ii+(((int)jj)*sx))*3)+2);
     }
  }
  else
  {
   // We have the H matrix and also detected blobs. Display the blob image
   double ii,jj,dx,dy;
   dx=(double)sx/1024.0;
   dy=(double)sy/768.0;
#pragma omp parallel for schedule(dynamic,16) private(ii,jj,i,j)
    for (j=0;j<768;j++)
     for (i=0;i<1024;i++)
     {
      ii=i*dx;
      jj=j*dy;
      *(big+(((i)+((j+128)*1024))*3)+0)=(unsigned char)((*(blobIm->layers[0]+(int)ii+((int)jj*blobIm->sx))));
      *(big+(((i)+((j+128)*1024))*3)+1)=(unsigned char)((*(blobIm->layers[1]+(int)ii+((int)jj*blobIm->sx))));
      *(big+(((i)+((j+128)*1024))*3)+2)=(unsigned char)((*(blobIm->layers[2]+(int)ii+((int)jj*blobIm->sx))));
     }
    deleteImage(blobIm);
  }

  ///////////////////////////////////////////////////////////////////////////
  // Have OpenGL display our image for this frame
  ///////////////////////////////////////////////////////////////////////////
  // Clear the screen and depth buffers
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glEnable(GL_TEXTURE_2D);
  glDisable(GL_LIGHTING);

  if (frame==0)
  {
   glGenTextures( 1, &texture);
   glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
   glBindTexture( GL_TEXTURE_2D, texture);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
   glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
   glTexImage2D (GL_TEXTURE_2D, 0, GL_RGB, 1024, 1024, 0, GL_RGB, GL_UNSIGNED_BYTE, big);
  }
  else
  {
   glBindTexture(GL_TEXTURE_2D, texture);
   glTexSubImage2D(GL_TEXTURE_2D,0,0,0,1024,1024,GL_RGB,GL_UNSIGNED_BYTE,big);
  }
  // Draw box bounding the viewing area
  glBegin (GL_QUADS);
  glTexCoord2f (0.0, 0.0);
  glVertex3f (0.0, 60.0, 0.0);
  glTexCoord2f (1.0, 0.0);
  glVertex3f (800.0, 60.0, 0.0);
  glTexCoord2f (1.0, 1.0);
  glVertex3f (800.0, 740.0, 0.0);
  glTexCoord2f (0.0, 1.0);
  glVertex3f (0.0, 740.0, 0.0);
  glEnd ();

  // Make sure all OpenGL commands are executed
  glFlush();
  // Swap buffers to enable smooth animation
  glutSwapBuffers();

  // Clean Up - Do all the image processing, AI, and planning before this code
  frame++;

  // Tell glut window to update ls itself
  glutSetWindow(windowID);
  glutPostRedisplay();
}

/////////////////////////////////////////////////////////////////////////////////////
// Field processing functions:
//   - Field un-warping
//   - Homography computation
//   - Background subtraction
/////////////////////////////////////////////////////////////////////////////////////
void fieldUnwarp2()
{
 ////////////////////////////////////////////////////////////////////////////
 //
 // Same as fieldUnwarp() but loops over the input image, not the target.
 // This should allow us to capture any blob pixels that should map to the
 // image after correcting for height displacement.
 //
 // It requires the homography matrix H 
 //
 // Direct mapping - may leave some pixel holes here and there...
 ////////////////////////////////////////////////////////////////////////////
 int i,j,id;
 double px,py,pw;
 double dx,dy;
 double vx,vy,mx,my;
 unsigned char *fi;
 double r1,g1,b1,r2,g2,b2,r3,g3,b3,r4,g4,b4;
 double R,G,B,Hu,S,V,dmax,dmin,pink,adj;
 double dp0,dp1,dp2,dp3;
 double mx0,mx1,mx2,mx3,my0,my1,my2,my3;
 int huIdx;
 int chubby=1;      // Fill in center pixel +/- chubby pixels on either side, above, and below.
 
 if (Hinv==NULL) {fprintf(stderr,"fieldUnwarp2(): No homography matix data - something is wrong!\n"); return;}

 fi=fieldIm;
 memset(fi,0,sx*sy*3*sizeof(unsigned char));           // Needed here as we may not update most pixels!

 mx0=cos(Mhues[0]);                               // Reference colour directions, compute once only
 my0=sin(Mhues[0]);
 mx1=cos(Mhues[1]);
 my1=sin(Mhues[1]);
 mx2=cos(Mhues[2]);
 my2=sin(Mhues[2]);
 mx3=cos(Mhues[3]);
 my3=sin(Mhues[3]);
  
#pragma omp parallel for schedule(dynamic,32) private(i,j,px,py,pw,dx,dy,R,G,B,r1,g1,b1,r2,g2,b2,r3,g3,b3,r4,g4,b4)
 for (j=0;j<sy;j++)
  for (i=0;i<sx;i++)
  {
   R=(double)*(frame_buffer+((i+(j*sx))*3)+0);
   G=(double)*(frame_buffer+((i+(j*sx))*3)+1);
   B=(double)*(frame_buffer+((i+(j*sx))*3)+2);
   
   if (R+G+B>0)             // Only process pixels left on by bgsubtract3() - saves time!
   {       
    huIdx=-1;
    rgb2hsv(R/255.0,G/255.0,B/255.0,&Hu,&S,&V);
    vx=cos(Hu);
    vy=sin(Hu);     
        
    // Greedily map this pixel to one of the reference hues, if it's close enough
    dp0=(mx0*vx)+(my0*vy);
    dp1=(mx1*vx)+(my1*vy);
    dp2=(mx2*vx)+(my2*vy);
    dp3=(mx3*vx)+(my3*vy);

    huIdx=-1;
    if (dp0>colAngThresh)
     if (dp0>dp1&&dp0>dp2&&dp0>dp3) huIdx=0;
    if (dp1>colAngThresh)
     if (dp1>dp0&&dp1>dp2&&dp1>dp3) huIdx=1;
    if (dp2>colAngThresh)
     if (dp2>dp0&&dp2>dp1&&dp2>dp3) huIdx=2;

    // NOTE: This ignores any pixels that are not close to the reference hues - colAngThresh is the key
    //   control for this. Expect holes in the mapped image! blobDetect will have to clean that up.
          
    if (huIdx>=0)   // Convert only pixels whose hue matches a reference hue
    {
     // Re-map colour to pure colour based on reference hue
     R=Mrgb[huIdx][0];
     G=Mrgb[huIdx][1];
     B=Mrgb[huIdx][2];
                
     // Obtain coordinates for this pixel in the unwarped image
     px=((*(Hinv+0))*i) + ((*(Hinv+1))*j) + (*(Hinv+2));
     py=((*(Hinv+3))*i) + ((*(Hinv+4))*j) + (*(Hinv+5));
     pw=((*(Hinv+6))*i) + ((*(Hinv+7))*j) + (*(Hinv+8));
     px=px/pw;
     py=py/pw;
     dx=px-(int)px;
     dy=py-(int)py;
     adj=0;
   
     // If calibration data exists for bot height, use it to adjust y location for the transformed pixel coordinate
     if (ref_Y[1]>-1e5&&got_Y==3&&gotCol==1&&heightAdj==1)
     {
      // This here is the tricky bit - if we have all the calibration data (bot height), this function will re-map the pixels
      // on the robot uniforms adjusted for robot height. To do this it uses the reference hues provided by the user, and
      // based on the matched hue it uses the corresponding bot's height adjustment data. This leaves the ball alone.
      // The adjustment is toggle-able via the U.I. use the '1' key to toggle this adjustment on/off

      // Adjustment based on alignment with the ball - first sample is close to the top of the field, second sample is at the bottom,
      // bots should be fully within the field!
      if (huIdx==0)
      {
       dmax=ref_Y[0]-adj_Y[0][0];
       dmin=ref_Y[1]-adj_Y[1][0];
       pink=(dmax-dmin)/(ref_Y[1]-ref_Y[0]);
       adj=dmin+((ref_Y[1]-py)*pink);
      }
      else if (huIdx==1)
      {
       dmax=ref_Y[0]-adj_Y[0][1];
       dmin=ref_Y[1]-adj_Y[1][1];
       pink=(dmax-dmin)/(ref_Y[1]-ref_Y[0]);
       adj=dmin+((ref_Y[1]-py)*pink);
      }      
         
    } // End if (fabs...
     
     py=py+adj;
     if (py>sy-1) py=sy-1;
     if (py<0) py=0;
     
     // Fill in a fat pixel - to account for forward mapping gaps as well as noise in hue quantization
     // NOTE: This is likely a race condition with #pragma omp enabled. The order in which some pixels
     // are overwritten with a RGB value will depend on thread scheduling. This is also likely not
     // relevant... so let it be. 
     for (int py_i=((int)py)-chubby; py_i<=((int)py)+chubby; py_i++)
      for (int px_i=((int)px)-chubby; px_i<=((int)px)+chubby; px_i++)
       if (px_i>0&&px_i<sx&&py_i>0&&py_i<sy)
       {
        *(fi+((px_i+(py_i*sx))*3)+0)=(unsigned char)R;
        *(fi+((px_i+(py_i*sx))*3)+1)=(unsigned char)G;
        *(fi+((px_i+(py_i*sx))*3)+2)=(unsigned char)B;  
       }
        
    }       // End if (huIdx>=0)    
   }    // End if (R+G+B>0)
  }

}

double *getH(void)
{
 //////////////////////////////////////////////////////////////////////
 //
 // This function estimates the homography matrix H from the 4 corner
 // coordinates input by the user. These 4 corners are made to 
 // correspond to the 4 corners of the displayed rectangular image.
 //
 //////////////////////////////////////////////////////////////////////
 double cD[4],tcorners[4][2];
 double corners2[4][2];
 double A[8][9],B[8][8],b[8],Binv[8][8];
 double *U,*s,*V,*rv1;
 double *H;
 int i,j;

 corners2[0][0]=1.0;
 corners2[0][1]=1.0;
 corners2[1][0]=sx-1;
 corners2[1][1]=1.0;
 corners2[2][0]=sx-1;
 corners2[2][1]=sy-1;
 corners2[3][0]=1.0;
 corners2[3][1]=sy-1;
 
 for (i=0;i<4;i++)
 {
  A[(2*i)+0][0]=0;
  A[(2*i)+0][1]=0;
  A[(2*i)+0][2]=0;
  A[(2*i)+0][3]=-corners2[i][0];
  A[(2*i)+0][4]=-corners2[i][1];
  A[(2*i)+0][5]=-1;
  A[(2*i)+0][6]=corners2[i][0]*Mcorners[i][1];
  A[(2*i)+0][7]=corners2[i][1]*Mcorners[i][1];
  A[(2*i)+0][8]=Mcorners[i][1];
  A[(2*i)+1][0]=corners2[i][0];
  A[(2*i)+1][1]=corners2[i][1];
  A[(2*i)+1][2]=1;
  A[(2*i)+1][3]=0;
  A[(2*i)+1][4]=0;
  A[(2*i)+1][5]=0;
  A[(2*i)+1][6]=-corners2[i][0]*Mcorners[i][0];
  A[(2*i)+1][7]=-corners2[i][1]*Mcorners[i][0];
  A[(2*i)+1][8]=-Mcorners[i][0];
 }

 // Allocate memory for the homography!
 H=(double *)calloc(9,sizeof(double));	// H should be the last row of U

 U=NULL;
 s=NULL;
 V=NULL;
 rv1=NULL;

 SVD(&A[0][0],8,9,&U,&s,&V,&rv1);
 // A note on sizes: A is 8x9, SVD should return U is 8x8, S is 8x9 
 // (or in this case a vector with 9 values), and V is 9x9. However,
 // Tom's code returns an 8x8 U, and a 9x8 V! missing last column

 // Now, because V is not all there, we have to solve for the eigenvector in V
 // that lies along the null-space of A. It has to be orthogonal to the 8
 // eigenvectors we just got in V. It also has 9 entries but fortunately we
 // can fix the last one to 1. That leaves an 8x8 non-homogeneous system we
 // can solve directly for.

 for (i=0;i<8;i++)
  for (j=0; j<8; j++)
   B[i][j]=(*(V+i+(j*8)));
 for (j=0; j<8; j++)
  b[j]=(-(*(V+j+64)));

 free(U);
 free(s);
 free(V);

 // Compute B^-1
 U=NULL;
 s=NULL;
 V=NULL;
 rv1=NULL;
 SVD(&B[0][0],8,8,&U,&s,&V,&rv1);
 InvertMatrix(U,s,V,8,&Binv[0][0]);
 free(U);
 free(s);
 free(V);

 // Finally, compute the eigenvector's first 8 entries
 for (i=0;i<8;i++)
 {
  *(H+i)=0;
  for (j=0; j<8;j++) (*(H+i))+=Binv[i][j]*b[j];
 }
 *(H+8)=1.0;
 
 fprintf(stderr,"Homography martrix:\n");
 fprintf(stderr,"hh=[%f %f %f\n",*(H),*(H+1),*(H+2));
 fprintf(stderr,"%f %f %f\n",*(H+3),*(H+4),*(H+5));
 fprintf(stderr,"%f %f %f\n];\n",*(H+6),*(H+7),*(H+8));
 
 for (i=0;i<4;i++)
 {
  memset(&cD[0],0,4*sizeof(double));
  cD[0]=((*(H+0))*corners2[i][0])+((*(H+1))*corners2[i][1])+((*(H+2)));
  cD[1]=((*(H+3))*corners2[i][0])+((*(H+4))*corners2[i][1])+((*(H+5)));
  cD[2]=((*(H+6))*corners2[i][0])+((*(H+7))*corners2[i][1])+((*(H+8)));
  fprintf(stderr,"Converts (%f,%f) to (%f,%f)\n",corners2[i][0],corners2[i][1],\
          cD[0]/cD[2],cD[1]/cD[2]);
 }
 return(H);
}

void bgSubtract3()
{
 ///////////////////////////////////////////////////////////////////////////////
 //
 // This function performs background subtraction. This changes the contents of
 //  the frame buffer!
 //
 // Pixels that are zeroed out include:
 //   - Any whose difference w.r.t. background is less than the specified bgThress (which is
 //     user controlled through the GUI)
 //   - Any whose saturation value is less than a the specified threshold (colThresh, also
 //     controlled via the GUI)
 //
 ///////////////////////////////////////////////////////////////////////////////

 int j,i;
 double r,g,b,R,G,B,dd;
 double Hu,S,V;
 double scl;
 char line[1024];
 FILE *f;
  
 if (!gotbg) return;
#pragma omp parallel for schedule(dynamic,32) private(i,j,r,g,b,R,G,B,dd,scl,Hu,S,V)
 for (j=0;j<sy; j++)
  for (i=0; i<sx; i++)
  {   
   r=(double)*(frame_buffer+((i+(j*sx))*3)+0);
   g=(double)*(frame_buffer+((i+(j*sx))*3)+1);
   b=(double)*(frame_buffer+((i+(j*sx))*3)+2);

   R=(double)bgIm[((i+(j*sx))*3)+0];
   G=(double)bgIm[((i+(j*sx))*3)+1];
   B=(double)bgIm[((i+(j*sx))*3)+2];
   
   // HSV conversion
   if (r>g&&r>b) V=r; else if (g>b) V=g; else V=b;
   if (V==0) S=0; else if (r<g&&r<b) S=(V-r)/V; else if (g<b) S=(V-g)/V; else S=(V-b)/V;
      
   // Attempt to adjust for shadows by computing a uniform brightness adjustment factor
   // shadows are free, said David
   scl=G/g;
//   if (scl<1) scl=1;
   r*=scl;
   g*=scl;
   b*=scl;
   
   // Compute magnitude of difference w.r.t. background image
   dd=(r-R)*(r-R);
   dd+=(g-G)*(g-G);
   dd+=(b-B)*(b-B);

   // Zero out background pixels and pixels that are not saturated (everything except uniforms/ball)
   if (dd<bgThresh||S<colThresh)
   {
    *(frame_buffer+((i+(j*sx))*3)+0)=0;
    *(frame_buffer+((i+(j*sx))*3)+1)=0;
    *(frame_buffer+((i+(j*sx))*3)+2)=0;
   }
  }
  
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//
// Blob detection and rendering
//
///////////////////////////////////////////////////////////////////////////////////////////////////
void releaseBlobs(struct blob *blobList)
{
 // Deletes a linked list of blobs
 struct blob *p,*q;
 p=blobList;
 while(p)
 {
  q=p->next;
  free(p);
  p=q;
 }
}

struct image *blobDetect2(struct blob **blob_list, int *nblobs)
{
 /////////////////////////////////////////////////////////////////////////////////////////////////
 //
 // This function does blob detection on the rectified field image (after background subtraction)
 // and generates:
 // - A label image, with a unique label for pixels in each blob (sx * sy * 1 layer)
 // - A list of blob data structures with suitably estimated values (though note that some of
 //   the blob data values are filled-in by the AI code later on)
 // - The number of blobs found
 // 
 // NOTE 1: This function will ignore tiny blobs
 // NOTE 2: The list of blobs is created from scratch for each frame - blobs are not persistent, nor 
 //                  will they be at the same list position each frame.
 /////////////////////////////////////////////////////////////////////////////////////////////////
 int *pixStack;
 int i,j,x,y;
 int mix,miy,mx,my;
 int lab;
 double R,G,B;
 double Hu,S,V,C;
 double Hx,Hy;
 double tH,tS,tV,tC;
 double tHx,tHy;
 double Hacc,Sacc,Vacc;
 double Ra,Ga,Ba;
 double xc,yc, oy1, oy2;
 int pixcnt;
 int *stack;
 int stackPtr;
 struct image *labIm, *tmpIm, *tmpIm2, *huIm, *huIm2;
 struct blob *bl;
 unsigned char *upFld;
 double cov[2][2],T,D,L1,L2;
 double refH[4];
 char line[1024];
 struct kernel *kern;
 double hx0,hx1,hx2,hx3,hy0,hy1,hy2,hy3;
 double dp0,dp1,dp2,dp3;
 int chubby=0;
 int huIdx;
 
 // Get reference vectors for calibrated hue values
 hx0=cos(Mhues[0]);
 hy0=sin(Mhues[0]);
 hx1=cos(Mhues[1]);
 hy1=sin(Mhues[1]);
 hx2=cos(Mhues[2]);
 hy2=sin(Mhues[2]);
 hx3=cos(Mhues[3]);
 hy3=sin(Mhues[3]);

 kern=GaussKernel(1.5);       // Mind the sigma here - 2 seemed to be too much!
 
 // Clear any previous list of blobs, initialize pixel stack
 if (*(blob_list)!=NULL)
 {
  releaseBlobs(*(blob_list));
  *(blob_list)=NULL;
 }
 pixStack=(int *)calloc(sx*sy*2,sizeof(int));
 if (pixStack==NULL) 
 {
     fprintf(stderr,"blobDetect2(): Out of Memory!\n");
 }

 // Assumed: Pixels in the input fieldIm that have non-zero RGB values are foreground
 labIm=newImage(sx,sy,1);				       // 1-layer labels image
 tmpIm=imageFromBuffer(fieldIm,sx,sy,3);

 // Filter background subtracted, saturation thresholded map to make smoother blobs
 tmpIm2=convolve_x(tmpIm,kern);
 deleteImage(tmpIm);
 tmpIm=convolve_y(tmpIm2,kern);
 deleteImage(tmpIm2);

 // **DEBUG** Update fieldIm so we can see what this thing is doing.
// upFld=bufferFromIm(tmpIm);
// for (i=0; i<sx*sy; i++)
//  *(fieldIm+i)=*(upFld+i);
// free(upFld);

 stack=&pixStack[0];
 lab=1;		

 // NOTE ****  The pixel colour/HSV accumulators may be unnecessary - we can replace them with the reference hue
 //            from calibration, S=1, V=1, and the corresponding RGB values -> perfect blob display regardless of
 //            field illumination/camera settings!
  
 // Visit each pixel and try to grow a blob from it if it has a non-zero value - this uses simple floodfill
 for (j=0;j<sy;j++)
  for (i=0;i<sx;i++)
  {
   R=*(tmpIm->layers[0]+i+(j*sx));
   G=*(tmpIm->layers[1]+i+(j*sx));
   B=*(tmpIm->layers[2]+i+(j*sx));
   Hacc=0;
   Sacc=0;
   Vacc=0;
      
   if (R+G+B>0)     // Found unlabeled pixel - possible concern is the pixel is at a region edge, hue may not be stable.
   {                // a better approach would do a bit of hill-climbing to get to a stable hue with high saturation.
    // Obtainn HSV components and color vector
    rgb2hsv(R/255.0,G/255.0,B/255.0,&Hu,&S,&V);
    Hx=cos(Hu);
    Hy=sin(Hu);
    dp0=(Hx*hx0)+(Hy*hy0);
    dp1=(Hx*hx1)+(Hy*hy1);
    dp2=(Hx*hx2)+(Hy*hy2);
    dp3=(Hx*hx3)+(Hy*hy3);

    // Ignore any pixels that are not close enough to a reference hue, and whose similarity to the reference hue is not greater than similarity w.r.t. background hue
    // threshold is high because fieldUnwarp remaps pixels to reference hues - the hue should be in fact identical...
    if ((dp0>.99&&dp0>dp3)||\
        (dp1>.99&&dp1>dp3)||\
        (dp2>.99&&dp2>dp3))
    {    
     stackPtr=1;
     *(stack+(2*stackPtr))=i;
     *(stack+(2*stackPtr)+1)=j;
     *(tmpIm->layers[0]+i+(j*sx)) = -(*(tmpIm->layers[0]+i+(j*sx)));
     *(tmpIm->layers[1]+i+(j*sx)) = -(*(tmpIm->layers[1]+i+(j*sx)));
     *(tmpIm->layers[2]+i+(j*sx)) = -(*(tmpIm->layers[2]+i+(j*sx)));
     Ra=0;
     Ga=0;
     Ba=0;
     xc=0;
     yc=0;
     mix=10000;
     miy=10000;
     mx=-10000;
     my=-10000;
     pixcnt=0;
     while (stackPtr>0)
     {
      if (stackPtr>=sx*sy) {fprintf(stderr,"blobDetect2(): **** Busted the stack! label image is NOT valid ****\n"); return labIm;};
      x=*(stack+(2*stackPtr));
      y=*(stack+(2*stackPtr)+1);
      stackPtr--;
      *(labIm->layers[0]+x+(y*labIm->sx))=lab;
      R=-(*(tmpIm->layers[0]+x+(y*tmpIm->sx)));
      G=-(*(tmpIm->layers[1]+x+(y*tmpIm->sx)));
      B=-(*(tmpIm->layers[2]+x+(y*tmpIm->sx)));
      Ra+=R;
      Ga+=G;
      Ba+=B;
      xc+=x;
      yc+=y;
      rgb2hsv(R/255.0,G/255.0,B/255.0,&Hu,&S,&V);
      Hacc+=Hu;
      Sacc+=S;
      Vacc+=V;
      pixcnt++;
      *(tmpIm->layers[0]+x+(y*sx)) = 0;
      *(tmpIm->layers[1]+x+(y*sx)) = 0;
      *(tmpIm->layers[2]+x+(y*sx)) = 0;
      if (mix>x) mix=x;
      if (miy>y) miy=y;
      if (mx<x) mx=x;
      if (my<y) my=y;

      // Check neighbours
      if (y>0)
      {
       R=*(tmpIm->layers[0]+x+((y-1)*sx));
       G=*(tmpIm->layers[1]+x+((y-1)*sx));
       B=*(tmpIm->layers[2]+x+((y-1)*sx));
       if (R+G+B>0)
       {
        rgb2hsv(R/255.0,G/255.0,B/255.0,&tH,&tS,&tV);
        tHx=cos(tH);
        tHy=sin(tH);
        if (fabs((tHx*Hx)+(tHy*Hy))>colAngThresh)
        {
         stackPtr++;
         *(stack+(2*stackPtr))=x;
         *(stack+(2*stackPtr)+1)=y-1;
         *(tmpIm->layers[0]+x+((y-1)*sx)) *= -1.0;
         *(tmpIm->layers[1]+x+((y-1)*sx)) *= -1.0;
         *(tmpIm->layers[2]+x+((y-1)*sx)) *= -1.0;
        }
       }
      }
      if (x<sx-1)
      {
       R=*(tmpIm->layers[0]+x+1+(y*sx));
       G=*(tmpIm->layers[1]+x+1+(y*sx));
       B=*(tmpIm->layers[2]+x+1+(y*sx));
       if (R+G+B>0)
       {
        rgb2hsv(R/255.0,G/255.0,B/255.0,&tH,&tS,&tV);
        tHx=cos(tH);
        tHy=sin(tH);
        if (fabs((tHx*Hx)+(tHy*Hy))>colAngThresh)
        {
         stackPtr++;
         *(stack+(2*stackPtr))=x+1;
         *(stack+(2*stackPtr)+1)=y;
         *(tmpIm->layers[0]+x+1+(y*sx)) *= -1.0;
         *(tmpIm->layers[1]+x+1+(y*sx)) *= -1.0;
         *(tmpIm->layers[2]+x+1+(y*sx)) *= -1.0;
        }
       }
      }
      if (y<sy-1)
      {
       R=*(tmpIm->layers[0]+x+((y+1)*sx));
       G=*(tmpIm->layers[1]+x+((y+1)*sx));
       B=*(tmpIm->layers[2]+x+((y+1)*sx));
       if (R+G+B>0)
       {
        rgb2hsv(R/255.0,G/255.0,B/255.0,&tH,&tS,&tV);
        tHx=cos(tH);
        tHy=sin(tH);
        if (fabs((tHx*Hx)+(tHy*Hy))>colAngThresh)
        {
         stackPtr++;
         *(stack+(2*stackPtr))=x;
         *(stack+(2*stackPtr)+1)=y+1;
         *(tmpIm->layers[0]+x+((y+1)*sx)) *= -1.0;
         *(tmpIm->layers[1]+x+((y+1)*sx)) *= -1.0;
         *(tmpIm->layers[2]+x+((y+1)*sx)) *= -1.0;
        }
       }
      }
      if (x>0)
      {
       R=*(tmpIm->layers[0]+x-1+(y*sx));
       G=*(tmpIm->layers[1]+x-1+(y*sx));
       B=*(tmpIm->layers[2]+x-1+(y*sx));
       if (R+G+B>0)
       {
        rgb2hsv(R/255.0,G/255.0,B/255.0,&tH,&tS,&tV);
        tHx=cos(tH);
        tHy=sin(tH);
        if (fabs((tHx*Hx)+(tHy*Hy))>colAngThresh)
        {
         stackPtr++;
         *(stack+(2*stackPtr))=x-1;
         *(stack+(2*stackPtr)+1)=y;
         *(tmpIm->layers[0]+x-1+(y*sx)) *= -1.0;
         *(tmpIm->layers[1]+x-1+(y*sx)) *= -1.0;
         *(tmpIm->layers[2]+x-1+(y*sx)) *= -1.0;
        }
       }
      }
     }	// End while

     if (pixcnt>MIN_BLOB_SIZE)
     {
      // If the size of the blob is greater than a small threshold, inset it in the
      // blob list and fill-out the blob data structure.
      Ra/=pixcnt;            // Average RGB values
      Ga/=pixcnt;
      Ba/=pixcnt;
      xc/=pixcnt;            // Centroid of the blob
      yc/=pixcnt;
      Hacc/=pixcnt;          // Average HSV values
      Sacc/=pixcnt;
      Vacc/=pixcnt;
      bl=(struct blob *)calloc(1,sizeof(struct blob));
      bl->label=lab;         // Label for this blob (a unique int ID, not related to color)
      bl->vx=0;              // Velocity and motion vectors *THESE ARE UPDATED BY THE AI*
      bl->vy=0;
      bl->mx=0;
      bl->my=0;
      bl->cx=xc;             // Blob center location
      bl->cy=yc;
      bl->size=pixcnt;       // Size in pixels
      bl->x1=mix;            // Bounding box (top-left -> bottom-right)
      bl->y1=miy;
      bl->x2=mx;
      bl->y2=my;
      bl->R=Ra;              // Blob RGB color
      bl->G=Ga;
      bl->B=Ba;
      bl->H=Hacc;            // Blob HSV color
      bl->S=Sacc;
      bl->V=Vacc;
      bl->next=NULL;
      bl->idtype=0;          // Set by the AI to indicate if this blob is an agent, and which agent it is
      // Insert into linked list of detected blobs.
      if (*(blob_list)==NULL) *(blob_list)=bl;
      else {bl->next=(*(blob_list))->next; (*(blob_list))->next=bl;}     
     }
     lab++;
    
    }    // End if (((Hx...
   }    // End if (R+G+B>0)
  }   // End for i

 deleteImage(tmpIm);

 // Count number of blobs found
 bl=*blob_list;
 *(nblobs)=0;
 while (bl!=NULL)
 {
  *nblobs=(*nblobs) + 1;  
  bl=bl->next;
 }
 
 // Compute blob direction for each blob
 bl=*blob_list;
 while (bl!=NULL)
 {
  memset(&cov[0][0],0,4*sizeof(double));
  for (j=bl->y1;j<=bl->y2;j++)
   for (i=bl->x1;i<=bl->x2;i++)
    if (*(labIm->layers[0]+i+(j*labIm->sx))==bl->label)
    {
     xc=i-bl->cx;
     yc=j-bl->cy;
     cov[0][0]+=(xc*xc);
     cov[1][1]+=(yc*yc);
     cov[0][1]+=(yc*xc);
     cov[1][0]+=(xc*yc);
    }
  cov[0][0]/=bl->size;
  cov[0][1]/=bl->size;
  cov[1][0]/=bl->size;
  cov[1][1]/=bl->size;
  T=cov[0][0]+cov[1][1];
  D=(cov[0][0]*cov[1][1])-(cov[1][0]*cov[0][1]);
  L1=(.5*T)+sqrt(((T*T)/4)-D);
  L2=(.5*T)-sqrt(((T*T)/4)-D);
  if (fabs(L1)>fabs(L2))
  {
   bl->dx=L1-cov[1][1];
   bl->dy=cov[1][0];
  }
  else
  {
   bl->dx=L2-cov[1][1];
   bl->dy=cov[1][0];
  } 
  T=sqrt((bl->dx*bl->dx)+(bl->dy*bl->dy));
  bl->dx/=T;
  bl->dy/=T;

  // Finally, if we have offset correction data, store it in the blob
  if (got_Y==3&&adj_Y[0][0]>-1e5)
   memcpy(&bl->adj_Y[0][0],&adj_Y[0][0],4*sizeof(double));
  else
   memset(&bl->adj_Y[0][0],0,4*sizeof(double));

  bl=bl->next;
 }
 
 deleteKernel(kern);
 free(pixStack);
 
 return(labIm);
} 

struct image *renderBlobs(struct image *labels, struct blob *list)
{
 //////////////////////////////////////////////////////////////////////////////////////////////
 //
 // This function renders an image with the blobs and geometric information provided by the
 // AI code (provided the AI is toggled-on and has estimated the corresponding values for
 // the blobs).
 //
 // - It renders each blob as a uniform-colored region
 // - It draws bounding boxes
 // - It draws crosshairs at the centers of blobs (or perspective-corrected crosshairs
 //   indicating the estimated location of the bots)
 // - It draws heading and orientation vectors
 //
 // NOTE: This function is also in charge of updating the calibration data for perspective
 //       projection error correction while the user is calibrating for Y offset error.
 //////////////////////////////////////////////////////////////////////////////////////////////

 int i,j;
 struct blob *p;
 struct image *blobIm;
 double cR,cG,cB;
 double *labRGB;
 int maxLab,lab;
 FILE *f;

 if (list==NULL) return(NULL);

 // Find maximum label for this round
 maxLab=-1;
 p=list;
 while (p!=NULL) {if (p->label>maxLab) maxLab=p->label; p=p->next;}

 labRGB=(double *)calloc(3*(maxLab+1),sizeof(double));
 if (labRGB==NULL) return(NULL);
 p=list;
 while (p!=NULL)
 {
  if (*(labRGB+((p->label)*3)+0)==0)
  {
   *(labRGB+((p->label)*3)+0)=p->R;
   *(labRGB+((p->label)*3)+1)=p->G;
   *(labRGB+((p->label)*3)+2)=p->B;
  }
  p=p->next;
 }

 blobIm=imageFromBuffer(fieldIm,sx,sy,3);
  
 // Uniform coloured blobs - colour is average colour of the corresponding region in the image  
#pragma omp parallel for schedule(dynamic,32) private(i,j)
 for (j=0;j<sy;j++)
  for (i=0;i<sx;i++)
  {
   if (*(labels->layers[0]+i+(j*labels->sx))!=0)
   {
    lab=*(labels->layers[0]+i+(j*labels->sx));
    *(blobIm->layers[0]+i+(j*blobIm->sx))=*(labRGB+(3*lab)+0);
    *(blobIm->layers[1]+i+(j*blobIm->sx))=*(labRGB+(3*lab)+1);
    *(blobIm->layers[2]+i+(j*blobIm->sx))=*(labRGB+(3*lab)+2);
   }
  }
    
 // Draw bounding boxes & cross-hairs 
 p=list;
 while (p!=NULL)
 {
  if (p->idtype>0)
  {
   if (p->idtype==3)		// This blob is the ball
   {
    cR=255.0;
    cG=255.0;
    cB=32.0;
    drawLine(p->cx-1,p->cy,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx,p->cy-1,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx+1,p->cy,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx,p->cy+1,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx,p->cy,p->mx,p->my,55,cR,cG,cB,blobIm);
    cR=255;
    cG=255;
    cB=0;
    if (got_Y==1&&ref_Y[0]<-1e5)
     ref_Y[0]=p->cy;
    if (got_Y==2&&ref_Y[1]<-1e5)
     ref_Y[1]=p->cy;
   }
   else if (p->idtype==1)	// This blob is our own bot
   {
    cR=255.0;
    cG=255.0;
    cB=32.0;
    drawLine(p->cx-1,p->cy,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx,p->cy-1,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx+1,p->cy,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx,p->cy+1,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx,p->cy,p->mx,p->my,55,cR,cG,cB,blobIm);
    cR=32;
    cG=255;                     // Green box around our bot
    cB=32;
    if (got_Y==1&&adj_Y[0][0]<-1e5)
     adj_Y[0][0]=p->cy;
    if (got_Y==2&&adj_Y[1][0]<-1e5)
     adj_Y[1][0]=p->cy;
   }
   else if (p->idtype==2)	// This blob is the opponent
   {
    cR=255.0;
    cG=255.0;
    cB=255.0;
    drawLine(p->cx-1,p->cy,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx,p->cy-1,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx+1,p->cy,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx,p->cy+1,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx,p->cy,p->mx,p->my,55,cR,cG,cB,blobIm);
    cR=255;
    cG=32;                      // Purple box around opponent
    cB=255;
    if (got_Y==1&&adj_Y[0][1]<-1e5)
     adj_Y[0][1]=p->cy;
    if (got_Y==2&&adj_Y[1][1]<-1e5)
     adj_Y[1][1]=p->cy;
   }
   drawBox(p->x1-1,p->y1,p->x2-1,p->y2,cR,cG,cB,blobIm);
   drawBox(p->x1+1,p->y1,p->x2+1,p->y2,cR,cG,cB,blobIm);
   drawBox(p->x1,p->y1-1,p->x2,p->y2-1,cR,cG,cB,blobIm);
   drawBox(p->x1,p->y1+1,p->x2,p->y2+1,cR,cG,cB,blobIm);
   drawBox(p->x1,p->y1,p->x2,p->y2,cR,cG,cB,blobIm);

   drawCross(p->cx-1,p->cy,64,64,64,15,blobIm);
   drawCross(p->cx+1,p->cy,64,64,64,15,blobIm);
   drawCross(p->cx,p->cy-1,128,128,128,15,blobIm);
   drawCross(p->cx,p->cy+1,128,128,128,15,blobIm);
   drawCross(p->cx,p->cy,255,255,255,15,blobIm);

   drawLine(p->cx-1,p->cy,p->dx,p->dy,55,255,255,255,blobIm);
   drawLine(p->cx,p->cy-1,p->dx,p->dy,55,255,255,255,blobIm);
   drawLine(p->cx+1,p->cy,p->dx,p->dy,55,255,255,255,blobIm);
   drawLine(p->cx,p->cy+1,p->dx,p->dy,55,255,255,255,blobIm);
   drawLine(p->cx,p->cy,p->dx,p->dy,55,255,255,255,blobIm);
  }
  p=p->next;
 }
  
 if (got_Y==2&&ref_Y[1]>-1e5)
 {
  fprintf(stderr,"Saving offsets for perspective correction.\n");
  fprintf(stderr,"Adjustment amounts for bot 1\n");
  fprintf(stderr,"At upper-field (h=%f), adj=%f\n",ref_Y[0],ref_Y[0]-adj_Y[0][0]);
  fprintf(stderr,"At lower-field (h=%f), adj=%f\n",ref_Y[1],ref_Y[1]-adj_Y[1][0]);
  fprintf(stderr,"Adjustment amounts for bot 2\n");
  fprintf(stderr,"At upper-field (h=%f), adj=%f\n",ref_Y[0],ref_Y[0]-adj_Y[0][1]);
  fprintf(stderr,"At lower-field (h=%f), adj=%f\n",ref_Y[1],ref_Y[1]-adj_Y[1][1]);
  f=fopen("offsets.dat","w");
  fwrite(&adj_Y[0][0],4*sizeof(double),1,f);
  fwrite(&ref_Y[0],2*sizeof(double),1,f);
  fclose(f);
  got_Y=3;				// Complete! we have offset calibration data
  doAI=0;				// End calibration loop
 }

 free(labRGB);
 return(blobIm);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// A few miscellaneous drawing functions to overlay stuff onto images
// We can draw: lines, boxes, and crosses onto images stored in an image data structure
//              crosses onto an image stored as a framebuffer
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void drawLine(int x1, int y1, double vx, double vy, double scale, double R, double G, double B, struct image *dst)
{
 int i,j;
 double len;
 double l;

 len=sqrt((vx*vx)+(vy*vy));
 vx=vx/len;
 vy=vy/len;

 for (l=0;l<=len*scale;l++)
 {
  i=(int)(x1+(l*vx));
  j=(int)(y1+(l*vy));
  if (i>=0&&i<dst->sx&&j>=0&&j<dst->sy)
  {
   *(dst->layers[0]+i+(j*dst->sx))=R;
   *(dst->layers[1]+i+(j*dst->sx))=G;
   *(dst->layers[2]+i+(j*dst->sx))=B;
  }
 }

}
 
void drawBox(int x1, int y1, int x2, int y2, double R, double G, double B, struct image *dst)
{
 int i,j;

 for (i=x1;i<=x2;i++)
 {
  if (i>=0&&i<dst->sx)
  {
   if (y1>=0&&y1<dst->sy)
   {
    *(dst->layers[0]+i+(y1*dst->sx))=R;
    *(dst->layers[1]+i+(y1*dst->sx))=G;
    *(dst->layers[2]+i+(y1*dst->sx))=B;
   }
   if (y2>=0&&y2<dst->sy)
   {
    *(dst->layers[0]+i+(y2*dst->sx))=R;
    *(dst->layers[1]+i+(y2*dst->sx))=G;
    *(dst->layers[2]+i+(y2*dst->sx))=B;
   }
  } 
 }
 for (j=y1;j<=y2;j++)
 {
  if (j>=0&&j<dst->sy)
  {
   if (x1>=0&&x1<dst->sx)
   {
    *(dst->layers[0]+x1+(j*dst->sx))=R;
    *(dst->layers[1]+x1+(j*dst->sx))=G;
    *(dst->layers[2]+x1+(j*dst->sx))=B;
   }
   if (x2>=0&&x2<dst->sx)
   {
    *(dst->layers[0]+x2+(j*dst->sx))=R;
    *(dst->layers[1]+x2+(j*dst->sx))=G;
    *(dst->layers[2]+x2+(j*dst->sx))=B;
   }
  }
 }

}

void drawCross(int mcx, int mcy, double R, double G, double B, int len, struct image *dst)
{
 int i,j;

 for (j=-len; j<len; j++)
  if (mcy+j>=0&&mcy+j<dst->sy&&j!=0) 
  {
   *(dst->layers[0]+mcx+((mcy+j)*dst->sx))=R;
   *(dst->layers[1]+mcx+((mcy+j)*dst->sx))=G;
   *(dst->layers[2]+mcx+((mcy+j)*dst->sx))=B;
  }
 for (i=-len; i<len; i++)
  if (mcx+i>=0&&mcx+i<dst->sx&&i!=0)
  {
   *(dst->layers[0]+mcx+i+(mcy*dst->sx))=R;
   *(dst->layers[1]+mcx+i+(mcy*dst->sx))=G;
   *(dst->layers[2]+mcx+i+(mcy*dst->sx))=B;
  }
}

void drawCross_buf(int mcx, int mcy, double R, double G, double B, int len, unsigned char *dst)
{
 int i,j;

 for (j=-len; j<len; j++)
  if (mcy+j>=0&&mcy+j<sy&&j!=0) 
  {
   *(dst+((mcx+((mcy+j)*sx))*3)+0)=R;
   *(dst+((mcx+((mcy+j)*sx))*3)+1)=G;
   *(dst+((mcx+((mcy+j)*sx))*3)+2)=B;
  }
 for (i=-len; i<len; i++)
  if (mcx+i>=0&&mcx+i<sx&&i!=0)
  {
   *(dst+((mcx+i+(mcy*sx))*3)+0)=R;
   *(dst+((mcx+i+(mcy*sx))*3)+1)=G;
   *(dst+((mcx+i+(mcy*sx))*3)+2)=B;
  }
}


/*********************************************************************
End of frame processing functions
**********************************************************************/

/*********************************************************************
 OpenGL display setup.
*********************************************************************/
// Initialize glut and create a window with the specified caption
void initGlut(char* winName)
{
 //////////////////////////////////////////////////////////////////////
 //
 // This function sets up the OpenGL display window, and informs GLUT
 // of the names of the callbacks to be used to refresh the display
 // for each frame, to handle window changes, and to handle keyboard
 // input.
 //
 //////////////////////////////////////////////////////////////////////

 // Set video mode: double-buffered, color, depth-buffered
 glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

 // Create window
 glutInitWindowPosition (0, 0);
 glutInitWindowSize(Win[0],Win[1]);
 windowID = glutCreateWindow(winName);

 // Setup callback functions to handle window-related events.
 // In particular, OpenGL has to be informed of which functions
 // to call when the image needs to be refreshed, and when the
 // image window is being resized.
 glutReshapeFunc(WindowReshape);   // Call WindowReshape whenever window resized
 glutDisplayFunc(FrameGrabLoop);   // Main display function is also the main loop
 glutKeyboardFunc(kbHandler);
}

void kbHandler(unsigned char key, int x, int y)
{ 
 /////////////////////////////////////////////////////////////////
 //
 // This is the GLUT keyboard handler. It is a callback triggered by
 // keyboard input while the OpenGL windows is in-focus. The
 // function then performs the function required for each 
 // of the valid keyboard commands.
 //
 /////////////////////////////////////////////////////////////////
 FILE *f;
 struct displayList *q;
 double R,G,B,Hu,S,V,mx;
 int mod;
 
 mod=0;
 
 // Exit!
 if (key=='q') 
 {
  BT_all_stop(0);
  releaseBlobs(blobs);
  deleteImage(proc_im);
  glDeleteTextures(1,&texture);
  closeCam(webcam);
  while (skynet.DPhead!=NULL)
  {
    q=skynet.DPhead->next;
    free(skynet.DPhead);
    skynet.DPhead=q;
  }
  free(fieldIm);
  free(bgIm);
  free(frame_buffer);
  free(H);
  free(Hinv);
  exit(0);
 }

 // Toggle AI processing on/off
 if (key=='t') if (doAI==1) doAI=0; else if (doAI==0) doAI=1;		// Ignores doAI=2 (calibration)
 if (key=='r') {setupAI(AIMode,botCol,&skynet); doAI=0;}		// Resets the state of the AI (may need full reset)

 // Controls for recording the corners of the playing field
 if (key=='z')
 {
  // Start/stop calibration loop. User will have to place robot at center, press 'x'
  // then place robot at bottom and press 'x' again.
  fprintf(stderr,"Offset calibration started\n");
  got_Y=0;
  ref_Y[0]=-1e6;
  ref_Y[1]=-1e6;
  adj_Y[0][0]=-1e6;
  adj_Y[0][1]=-1e6;
  adj_Y[1][0]=-1e6;
  adj_Y[1][1]=-1e6;
  if (doAI==0) doAI=2; else if (doAI==2) doAI=0;
 }
 if (key=='x')
 {
  // Record Y offsets first at center then at bottom of the field
  // adj_Y[0][0] and adj_Y[1][0] will contain Y offsets for blue bot
  // adj_Y[0][1] and adj_Y[1][1] will contain Y offsets for red bot
  if (got_Y==0) {got_Y=1; fprintf(stderr,"Capturing Y offset at upper field\n");}
  else if (got_Y==1) {got_Y=2; fprintf(stderr,"Capturing Y offset at lower field\n");}
 }
 if (key=='c')
 {
  // Capture colour blob values - this sets the colour of blobs as currently visible to make
  // blob detection and tracking easier - it also caches the current thresholds for image
  // blob detection, so they don't have to be set each time
  Mhues[0]=-1;
  Mhues[1]=-1;
  Mhues[2]=-1;
  Mhues[3]=-1;
  memset(&Mrgb[0][0],4*3*sizeof(double),0);
  gotCol=0;
  colourIdx=0;
  toggleProc=1;		// Indicates manual color capture
  mcx=340;
  mcy=340;
  fprintf(stderr,"Starting colour selection - please place the crosshair on the blue bot, then the red bot, then the ball, then the *background*, and press space for each.\n");
 }
 if (key=='g')		// Load and use all calibration data
 {
  // Load cached H matrix and background image
  f=fopen("Homography.dat","r");
  if (f!=NULL)
  {
   if (H!=NULL) free(H);
   H=(double *)calloc(9,sizeof(double));
   Hinv=(double *)calloc(9,sizeof(double));
   fread(H,9*sizeof(double),1,f);
   fread(Hinv,9*sizeof(double),1,f);
   fread(bgIm,sx*sy*3*sizeof(unsigned char),1,f);
   fclose(f);
   gotbg=1;
   cornerIdx=4;
   fprintf(stderr,"Successfully read background and H matrix from file\n");
  }
  else
   fprintf(stderr,"No Homography calibration data. Please capture corners and background image (use 'm')\n");
  
  // Read offset calibration from file
  f=fopen("offsets.dat","r");
  if (f!=NULL)
  {
   fread(&adj_Y[0][0],4*sizeof(double),1,f);
   fread(&ref_Y[0],2*sizeof(double),1,f);
   fclose(f);
   fprintf(stderr,"Read offset data from file\n");
   got_Y=3;
   if (doAI==2) doAI=0;
  }
  else fprintf(stderr,"No offset calibration data, please capture robot height offsets (use 'z')\n");
  
  f=fopen("colours.dat","r");
  if (f!=NULL)
  {
    fread(&Mhues[0],4*sizeof(double),1,f);
    fread(&Mrgb[0][0],4*3*sizeof(double),1,f);
    fread(&bgThresh,sizeof(double),1,f);
    fread(&colAngThresh,sizeof(double),1,f);
    fread(&colThresh,sizeof(double),1,f);
    fclose(f);
    gotCol=1;
    fprintf(stderr,"Read colour calibration data from file\n");
    fprintf(stderr,"Reference colour 0: %f, %f, %f, Hue=%f\n",Mrgb[0][0],Mrgb[0][1],Mrgb[0][2],Mhues[0]);
    fprintf(stderr,"Reference colour 1: %f, %f, %f, Hue=%f\n",Mrgb[1][0],Mrgb[1][1],Mrgb[1][2],Mhues[1]);
    fprintf(stderr,"Reference colour 2: %f, %f, %f, Hue=%f\n",Mrgb[2][0],Mrgb[2][1],Mrgb[2][2],Mhues[2]);
    fprintf(stderr,"Reference colour 3: %f, %f, %f, Hue=%f\n",Mrgb[3][0],Mrgb[3][1],Mrgb[3][2],Mhues[3]);
  }

  else fprintf(stderr,"No colour calibration data, please capture colour values (use 'c')\n");     
 }
 if (key=='m')		// Manually select corners
 {
  if (H!=NULL) 
  {
   free(H);
   H=NULL; 
   free(Hinv); 
   Hinv=NULL;
  }
  memset(&Mcorners[0][0],0,8*sizeof(double));
  cornerIdx=0;
  toggleProc=2;		// Indicates manual corner detection active  
  mcx=340;
  mcy=340;
  fprintf(stderr,"Starting corner selection - please place the crosshair at corners top-left, top-right, bottom-right, and bottom-left, and press space for each.\n");
 }
 if (key=='d'&&toggleProc>0) if (mcx<sx-1) mcx++;
 if (key=='a'&&toggleProc>0) if (mcx>1) mcx--;
 if (key=='w'&&toggleProc>0) if (mcy>1) mcy--;
 if (key=='s'&&toggleProc>0) if (mcy<sy-1) mcy++;
 if (key=='D'&&toggleProc>0) if (mcx<sx-6) mcx+=5;
 if (key=='A'&&toggleProc>0) if (mcx>5) mcx-=5;
 if (key=='W'&&toggleProc>0) if (mcy>5) mcy-=5;
 if (key=='S'&&toggleProc>0) if (mcy<sy-6) mcy+=5;
 if (key==' '&&toggleProc==2) 
 {
  fprintf(stderr,"Recorded corner %d at %f,%f\n",cornerIdx,mcx,mcy);
  Mcorners[cornerIdx][0]=mcx;
  Mcorners[cornerIdx][1]=mcy;
  cornerIdx++;
  if (cornerIdx==4) toggleProc=0;	// Done!
 }
 if (key==' '&&toggleProc==1) 
  toggleProc=4;  // Only thing to do here is signal frameGrabLoop() the need to capture a colour

 // Image processing controls
 if (key=='<') {bgThresh-=50;fprintf(stderr,"BG subtract threshold now at %f\n",bgThresh);mod=1;}
 if (key=='>') {bgThresh+=50;fprintf(stderr,"BG subtract threshold now at %f\n",bgThresh);mod=1;}
 if (key=='{'&&colAngThresh>.5) {colAngThresh-=.01;fprintf(stderr,"Colour Angle Threshold now at %f\n",colAngThresh);mod=1;}
 if (key=='}'&&colAngThresh<.99) {colAngThresh+=.01;fprintf(stderr,"Colour Angle Threshold now at %f\n",colAngThresh);mod=1;}
 if (key=='['&&colThresh>=0.05) {colThresh-=.05;fprintf(stderr,"Saturation threshold now at %f\n",colThresh);mod=1;}
 if (key==']'&&colThresh<=0.95) {colThresh+=.05;fprintf(stderr,"Saturation threshold now at %f\n",colThresh);mod=1;}
 
 if (mod==1)
 {
  // Cache updated thresholds
  f=fopen("colours.dat","w");
  fwrite(&Mhues[0],4*sizeof(double),1,f);
  fwrite(&Mrgb[0][0],4*3*sizeof(double),1,f);
  fwrite(&bgThresh,sizeof(double),1,f);
  fwrite(&colAngThresh,sizeof(double),1,f);
  fwrite(&colThresh,sizeof(double),1,f);
  fclose(f);
  fprintf(stderr,"Updated colour calibration data.\n");
 }

 
 if (key=='f') {if (printFPS==0) printFPS=1; else printFPS=0;}

 // Robot robot manual override
 if (key=='i') {if (DIR_FWD==0) {DIR_FWD=1; DIR_L=0; DIR_R=0; DIR_BACK=0; BT_drive(LEFT_MOTOR, RIGHT_MOTOR,75);} else {DIR_FWD=0; BT_all_stop(0);}}
 if (key=='j') {if (DIR_L==0) {DIR_L=1; DIR_R=0; DIR_FWD=0; DIR_BACK=0; BT_turn(LEFT_MOTOR, 50, RIGHT_MOTOR, -50);} else {DIR_L=0; BT_all_stop(0);}}
 if (key=='l') {if (DIR_R==0) {DIR_R=1; DIR_L=0; DIR_FWD=0; DIR_BACK=0; BT_turn(LEFT_MOTOR, -50, RIGHT_MOTOR, 50);} else {DIR_R=0; BT_all_stop(0);}}
 if (key=='k') {if (DIR_BACK==0) {DIR_BACK=1; DIR_L=0; DIR_R=0; DIR_FWD=0; BT_drive(LEFT_MOTOR, RIGHT_MOTOR, -75);} else {DIR_BACK=0; BT_all_stop(0);}}
 if (key=='o') {BT_all_stop(0);doAI=0;}	// <-- Important!

 if (key=='1') {if (heightAdj==0) heightAdj=1; else heightAdj=0;}   
    
}

void WindowReshape(int w, int h)
{
 ///////////////////////////////////////////////////////////////////////
 //
 // This function handles changes in the geometry of the OpenGL window
 // and ensures OpenGL renders to the correct window dimensions
 //
 ///////////////////////////////////////////////////////////////////////
 glMatrixMode(GL_PROJECTION);
 glLoadIdentity();			// Initialize with identity matrix
 gluOrtho2D(0, 800, 800, 0);
 glViewport(0,0,w,h);
 Win[0] = w;
 Win[1] = h;
}
/*********************************************************************
End of OpenGL display setup
*********************************************************************/

/*********************************************************************
Camera initialization, frame grab, and frame conversion. 
*********************************************************************/
void yuyv_to_rgb (struct vdIn *vd, int sx, int sy)
{
  ///////////////////////////////////////////////////////////////////
  // The camera's video frame comes in a format called yuyv, this
  // means that 2 pixel RGB values are encoded as 4 bytes, two 
  // luminance samples (the two y's), and two colour samples (u and v).
  // To use the frame, we have to convert each set of 4 yuyv samples
  // into two RGB triplets. 
  //
  // The input is a video frame data structure, and the size of the
  // image (sx,sy).
  //
  // Returns a pointer to the newly allocated RGB image buffer, if
  // something goes wrong it returns NULL.
  //
  // Derived from compress_yuyv_to_jpeg() in uvccapture.c
  ///////////////////////////////////////////////////////////////////
  unsigned char *yuyv;
  int ii;
  unsigned char *ptr;

#pragma omp parallel for schedule(dynamic,32) private(ii,ptr,yuyv)
  for (ii=0;ii<vd->height;ii++)
  {
   int x;
   ptr=frame_buffer+((ii*vd->width)*3);
   yuyv=vd->framebuffer+((ii*vd->width)*2);
   for (x = 0; x < vd->width; x+=2) 
   {
    int r, g, b;
    int y, u, v;
    y = yuyv[0] << 8;
    u = yuyv[1] - 128;
    v = yuyv[3] - 128;

    r = (y + (359 * v)) >> 8;
    g = (y - (88 * u) - (183 * v)) >> 8;
    b = (y + (454 * u)) >> 8;

    *(ptr++) = (r > 255) ? 255 : ((r < 0) ? 0 : r);
    *(ptr++) = (g > 255) ? 255 : ((g < 0) ? 0 : g);
    *(ptr++) = (b > 255) ? 255 : ((b < 0) ? 0 : b);

    y = yuyv[2] << 8;
    r = (y + (359 * v)) >> 8;
    g = (y - (88 * u) - (183 * v)) >> 8;
    b = (y + (454 * u)) >> 8;

    *(ptr++) = (r > 255) ? 255 : ((r < 0) ? 0 : r);
    *(ptr++) = (g > 255) ? 255 : ((g < 0) ? 0 : g);
    *(ptr++) = (b > 255) ? 255 : ((b < 0) ? 0 : b);
    yuyv += 4;
   }   // End for x  
  }  
}
    
struct vdIn *initCam(const char *videodevice, int width, int height)
{
 /* 
    Camera initialization - sets up the camera communication, image
    format, fps, and other camera parmeters. Derived from uvccapture.c
 */
	int status;
	unsigned char *p = NULL;
	int hwaccel = 1;
	const char *mode = NULL;
	int format = V4L2_PIX_FMT_YUYV;
    int i;					
	int grabmethod = 1;
	int fps = 30;
	unsigned char frmrate = 0;
    char avifilename[1024]="video.avi";
	int queryformats = 0;
	int querycontrols = 0;
	int readconfigfile = 0;
	char *separateur;
	char *sizestring = NULL;
	char *fpsstring  = NULL;
	int enableRawStreamCapture = 0;
	int enableRawFrameCapture = 0;
	struct vdIn *videoIn;
	FILE *file;

	printf("%s\n\n",version);

	videoIn = (struct vdIn *) calloc(1, sizeof(struct vdIn));

	if (init_videoIn
			(videoIn, (char *) videodevice, width, height, fps, format,
			 grabmethod, &avifilename[0]) < 0)
		return(NULL);
	return(videoIn);		// Successfully opened a video device
}

void getFrame(struct vdIn *videoIn, int sx, int sy)
{
 /*
   Grab a single frame from the camera into the frame_buffer (global pointer). Derived from uvccapture.c
 */
    double dtime;

	// Grab a frame from the video device
	if (uvcGrab(videoIn) < 0) {
		fprintf(stderr,"getFrame(): There was an error grabbing the frame from the webcam.\n");
		return;
	}
    yuyv_to_rgb(videoIn, sx, sy); 
    videoIn->getPict = 0;

	// Print FPS if needed.
    frameNo++;
    time(&time2);
    dtime=difftime(time2,time1);
    if (printFPS) fprintf(stderr,"FPS= %f\n",(double)frameNo/dtime);
}

void closeCam(struct vdIn *videoIn)
{
	close_v4l2(videoIn);
	free(videoIn);
}

/*********************************************************************
End of Webcam manipulation code
*********************************************************************/


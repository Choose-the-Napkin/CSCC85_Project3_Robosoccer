/************************************************************************************
 CSC C85 - UTSC RoboSoccer
 
 Main - this sets up communications with the Lego bot, creates an AI data structure
        with the appropriate mode, and starts the image processing/AI main loop

        You only need to modify this file to set up your EV3's hex ID. Other than
        that you should *NOT* change anything here.

 roboSoccer.c - by Per Parker, Summer 2013
 updated by F. Estrada, Jul. 2022.
************************************************************************************/

/***********************************************************
 *  OVERVIEW of the whole process
 * 
 * roboSoccer.c
 *  - This is the main() entry point for the robo soccer software
 *      * it sets up a socket for communicating with the EV3
 *      * It then starts the imageCapture loop
 *      * Once started, the imageCapture loop takes over 
 * 
 *  imageCapture.c
 *  Image Capture loop
 *  - Sets up the image processing parameters (field location and
 *    size, robot calibration, etc. See the handout for details)
 *  - Once the corners of the playfield have been set up, it 
 *    processes each input frame to extract blobs and compute
 *    blob parameters. See the handout for instructions on how
 *    to fine-tune the camera parameters and the image capture
 *    loop thresholds to make the blob detection as good as possible!
 *    (this is ESSENTIAL for you to learn - crappy blobs->toast)
 *  - It processes input from the keyboard so you can 
 *    * Capture corners of the playfield
 *    * Do robot calibration
 *    * Toggle the AI on/off
 *    * Reset the bot (all stop!)
 *    * Manual control of the bot
 *  - Calls the AI once each frame to do its job - This works
 *    because the image capture loop has one AI data structure
 *    variable (aptly called 'skynet'. Look for it, and make
 *    sure you understand what it stores. You can also look
 *    in the image capture code and see when/how this is used
 *    to call your AI routines.
 *  - The imageCapture loop is in charge until the program
 *    exits!
 * 
 *  roboAI.c
 *  - This is where you will do most of your work, your AI
 *    state machine will be there, as will be any code that
 *    you implement to handle robot movement and control
 *  - The code here is called by the image capture loop, 
 *    once per frame, with the current values for any
 *    detected blobs.
 *  - The AI data structure has a display list that you can
 *    use to add graphical objects and markers you want the
 *    image capture loop to display on-screen. Use this to
 *    debug and test your AI routines (e.g. to plot the
 *    location you want your bot to go, to show directions
 *    of motion, to show the motion plan you want to
 *    carry out, etc.).
 *  
 *  Be careful to understand the flow of program execution as
 *  described above! you need to understand it so you know what
 *  is happening at different points in time, and where it is
 *  that something may have gone wrong.
 *************************************************************/

#include "imagecapture/imageCapture.h"
#include <stdio.h>
#include <GL/glut.h>
#include "roboAI.h"
#include "API/btcomm.h"

// Replace your robot's hex identifier here so the code can talk to it.
#ifndef HEXKEY
	#define HEXKEY "00:00:00:00:00:01"	
#endif

int main(int argc, char **argv)
{

  if (argc<4||(atoi(argv[2])>1||atoi(argv[2])<0)||(atoi(argv[3])>2||atoi(argv[3])<0))
  {
   fprintf(stderr,"roboSoccer: Incorrect number of parameters.\n");
   fprintf(stderr,"USAGE: roboSoccer video_device own_colour mode\n");
   fprintf(stderr,"  video_device - path to camera (typically /dev/video0 or /dev/video1)\n");
   fprintf(stderr,"  own_colour - colour of the EV3 bot controlled by this program, 0 = BLUE, 1 = RED\n");
   fprintf(stderr,"  mode - AI mode: 0 = SOCCER, 1 = PENALTY, 2 = CHASE\n");
   exit(0);
  }

  //Connect to device
  BT_open(HEXKEY);

  // Start GLUT
  glutInit(&argc, argv);

  // Launch imageCapture
  if (imageCaptureStartup(argv[1], 1280, 720, atoi(argv[2]), atoi(argv[3]))) {
    fprintf(stderr, "Couldn't start image capture, terminating...\n");
    exit(0);
  }

  return 0;
}

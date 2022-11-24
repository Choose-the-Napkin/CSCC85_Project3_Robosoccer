/**************************************************************************
  CSC C85 - UTSC RoboSoccer AI core

  This file is where the actual planning is done and commands are sent
  to the robot.

  Please read all comments in this file, and add code where needed to
  implement your game playing logic. 

  Things to consider:

  - Plan - don't just react
  - Use the heading vectors!
  - Mind the noise (it's everywhere)
  - Try to predict what your oponent will do
  - Use feedback from the camera

  What your code should not do: 

  - Attack the opponent, or otherwise behave aggressively toward the
    oponent
  - Hog the ball (you can kick it, push it, or leave it alone)
  - Sit at the goal-line or inside the goal
  - Run completely out of bounds

  AI scaffold: Parker-Lee-Estrada, Summer 2013

  EV3 Version 2.0 - Updated Jul. 2022 - F. Estrada
***************************************************************************/

#include "roboAI.h"			// <--- Look at this header file!
extern int sx;              // Get access to the image size from the imageCapture module
extern int sy;
int laggy=0;

/**************************************************************
 * Display List Management
 * 
 * The display list head is kept as a pointer inside the A.I. 
 * data structure. Initially NULL (of course). It works like
 * any other linked list - anytime you add a graphical marker
 * it's added to the list, the imageCapture code loops over
 * the list and draws any items in there.
 * 
 * The list IS NOT CLEARED between frames (so you can display
 * things like motion paths that go over mutiple frames).
 * Your code will need to call clearDP() when you want this
 * list cleared.
 * 
 * ***********************************************************/
struct displayList *addPoint(struct displayList *head, int x, int y, double R, double G, double B)
{
  struct displayList *newNode;
  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addPoint(): Out of memory!\n");
    return head;
  }
  newNode->type=0;
  newNode->x1=x;
  newNode->y1=y;
  newNode->x2=-1;
  newNode->y2=-1;
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  
  newNode->next=head;
  return(newNode);
}

struct displayList *addLine(struct displayList *head, int x1, int y1, int x2, int y2, double R, double G, double B)
{
  struct displayList *newNode;
  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addLine(): Out of memory!\n");
    return head;
  }
  newNode->type=1;
  newNode->x1=x1;
  newNode->y1=y1;
  newNode->x2=x2;
  newNode->y2=y2;
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  newNode->next=head;
  return(newNode);  
}

struct displayList *addVector(struct displayList *head, int x1, int y1, double dx, double dy, int length, double R, double G, double B)
{
  struct displayList *newNode;
  double l;
  
  l=sqrt((dx*dx)+(dy*dy));
  dx=dx/l;
  dy=dy/l;
  
  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addVector(): Out of memory!\n");
    return head;
  }
  newNode->type=1;
  newNode->x1=x1;
  newNode->y1=y1;
  newNode->x2=x1+(length*dx);
  newNode->y2=y1+(length*dy);
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  newNode->next=head;
  return(newNode);
}

struct displayList *addCross(struct displayList *head, int x, int y, int length, double R, double G, double B)
{
  struct displayList *newNode;
  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addLine(): Out of memory!\n");
    return head;
  }
  newNode->type=1;
  newNode->x1=x-length;
  newNode->y1=y;
  newNode->x2=x+length;
  newNode->y2=y;
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  newNode->next=head;
  head=newNode;

  newNode=(struct displayList *)calloc(1,sizeof(struct displayList));
  if (newNode==NULL)
  {
    fprintf(stderr,"addLine(): Out of memory!\n");
    return head;
  }
  newNode->type=1;
  newNode->x1=x;
  newNode->y1=y-length;
  newNode->x2=x;
  newNode->y2=y+length;
  newNode->R=R;
  newNode->G=G;
  newNode->B=B;
  newNode->next=head;
  return(newNode);
}

struct displayList *clearDP(struct displayList *head)
{
  struct displayList *q;
  while(head)
  {
      q=head->next;
      free(head);
      head=q;
  }
  return(NULL);
}

/**************************************************************
 * End of Display List Management
 * ***********************************************************/

/*************************************************************
 * Blob identification and tracking
 * ***********************************************************/

struct blob *id_coloured_blob2(struct RoboAI *ai, struct blob *blobs, int col)
{
 /////////////////////////////////////////////////////////////////////////////
 // This function looks for and identifies a blob with the specified colour.
 // It uses the hue and saturation values computed for each blob and tries to
 // select the blob that is most like the expected colour (red, green, or blue)
 //
 // If you find that tracking of blobs is not working as well as you'd like,
 // you can try to improve the matching criteria used in this function.
 // Remember you also have access to shape data and orientation axes for blobs.
 //
 // Inputs: The robot's AI data structure, a list of blobs, and a colour target:
 // Colour parameter: 0 -> Blue bot
 //                   1 -> Red bot
 //                   2 -> Yellow ball
 // Returns: Pointer to the blob with the desired colour, or NULL if no such
 // 	     blob can be found.
 /////////////////////////////////////////////////////////////////////////////

 struct blob *p, *fnd;
 double vr_x,vr_y,maxfit,mincos,dp;
 double vb_x,vb_y,fit;
 double maxsize=0;
 double maxgray;
 int grayness;
 int i;
 static double Mh[4]={-1,-1,-1,-1};
 static double mx0,my0,mx1,my1,mx2,my2;
 FILE *f;
 
 // Import calibration data from file - this will contain the colour values selected by
 // the user in the U.I.
 if (Mh[0]==-1)
 {
  f=fopen("colours.dat","r");
  if (f!=NULL)
  {
   fread(&Mh[0],4*sizeof(double),1,f);
   fclose(f);
   mx0=cos(Mh[0]);
   my0=sin(Mh[0]);
   mx1=cos(Mh[1]);
   my1=sin(Mh[1]);
   mx2=cos(Mh[2]);
   my2=sin(Mh[2]);
  }
 }

 if (Mh[0]==-1)
 {
     fprintf(stderr,"roboAI.c :: id_coloured_blob2(): No colour calibration data, can not ID blobs. Please capture colour calibration data on the U.I. first\n");
     return NULL;
 }
 
 maxfit=.025;                                             // Minimum fitness threshold
 mincos=.90;                                              // Threshold on colour angle similarity
 maxgray=.25;                                             // Maximum allowed difference in colour
                                                          // to be considered gray-ish (as a percentage
                                                          // of intensity)

 // The reference colours here are in the HSV colourspace, we look at the hue component, which is a
 // defined within a colour-wheel that contains all possible colours. Hence, the hue component
 // is a value in [0 360] degrees, or [0 2*pi] radians, indicating the colour's location on the
 // colour wheel. If we want to detect a different colour, all we need to do is figure out its
 // location in the colour wheel and then set the angles below (in radians) to that colour's
 // angle within the wheel.
 // For reference: Red is at 0 degrees, Yellow is at 60 degrees, Green is at 120, and Blue at 240.

  // Agent IDs are as follows: 0 : blue bot,  1 : red bot, 2 : yellow ball
  if (col==0) {vr_x=mx0; vr_y=my0;}                                                    
  else if (col==1) {vr_x=mx1; vr_y=my1;}
  else if (col==2) {vr_x=mx2; vr_y=my2;}

 // In what follows, colours are represented by a unit-length vector in the direction of the
 // hue for that colour. Similarity between two colours (e.g. a reference above, and a pixel's
 // or blob's colour) is measured as the dot-product between the corresponding colour vectors.
 // If the dot product is 1 the colours are identical (their vectors perfectly aligned), 
 // from there, the dot product decreases as the colour vectors start to point in different
 // directions. Two colours that are opposite will result in a dot product of -1.
 
 p=blobs;
 while (p!=NULL)
 { 
  if (p->size>maxsize) maxsize=p->size;
  p=p->next;
 }

 p=blobs;
 fnd=NULL;
 while (p!=NULL)
 {
  // Normalization and range extension
  vb_x=cos(p->H);
  vb_y=sin(p->H);

  dp=(vb_x*vr_x)+(vb_y*vr_y);                                       // Dot product between the reference color vector, and the
                                                                    // blob's color vector.

  fit=dp*p->S*p->S*(p->size/maxsize);                               // <<< --- This is the critical matching criterion.
                                                                    // * THe dot product with the reference direction,
                                                                    // * Saturation squared
                                                                    // * And blob size (in pixels, not from bounding box)
                                                                    // You can try to fine tune this if you feel you can
                                                                    // improve tracking stability by changing this fitness
                                                                    // computation

  // Check for a gray-ish blob - they tend to give trouble
  grayness=0;
  if (fabs(p->R-p->G)/p->R<maxgray&&fabs(p->R-p->G)/p->G<maxgray&&fabs(p->R-p->B)/p->R<maxgray&&fabs(p->R-p->B)/p->B<maxgray&&\
      fabs(p->G-p->B)/p->G<maxgray&&fabs(p->G-p->B)/p->B<maxgray) grayness=1;
  
  if (fit>maxfit&&dp>mincos&&grayness==0)
  {
   fnd=p;
   maxfit=fit;
  }
  
  p=p->next;
 }

 return(fnd);
}

void track_agents(struct RoboAI *ai, struct blob *blobs)
{
 ////////////////////////////////////////////////////////////////////////
 // This function does the tracking of each agent in the field. It looks
 // for blobs that represent the bot, the ball, and our opponent (which
 // colour is assigned to each bot is determined by a command line
 // parameter).
 // It keeps track within the robot's AI data structure of multiple 
 // parameters related to each agent:
 // - Position
 // - Velocity vector. Not valid while rotating, but possibly valid
 //   while turning.
 // - Motion direction vector. Not valid
 //   while rotating - possibly valid while turning
 // - Heading direction - vector obtained from the blob shape, it is
 //   correct up to a factor of (-1) (i.e. it may point backward w.r.t.
 //   the direction your bot is facing). This vector remains valid
 //   under rotation.
 // - Pointers to the blob data structure for each agent
 //
 // This function will update the blob data structure with the velocity
 // and heading information from tracking. 
 //
 // NOTE: If a particular agent is not found, the corresponding field in
 //       the AI data structure (ai->st.ball, ai->st.self, ai->st.opp)
 //       will remain NULL. Make sure you check for this before you 
 //       try to access an agent's blob data! 
 //
 // In addition to this, if calibration data is available then this
 // function adjusts the Y location of the bot and the opponent to 
 // adjust for perspective projection error. See the handout on how
 // to perform the calibration process.
 //
 // This function receives a pointer to the robot's AI data structure,
 // and a list of blobs.
 //
 // You can change this function if you feel the tracking is not stable.
 // First, though, be sure to completely understand what it's doing.
 /////////////////////////////////////////////////////////////////////////

 struct blob *p;
 double mg,vx,vy,pink,doff,dmin,dmax,adj;
 
 // Reset ID flags and agent blob pointers
 ai->st.ballID=0;
 ai->st.selfID=0;
 ai->st.oppID=0;
 ai->st.ball=NULL;			// Be sure you check these are not NULL before
 ai->st.self=NULL;			// trying to access data for the ball/self/opponent!
 ai->st.opp=NULL;
 
 // Find the ball
 p=id_coloured_blob2(ai,blobs,2);
 if (p)
 {
  ai->st.ball=p;			// New pointer to ball
  ai->st.ballID=1;			// Set ID flag for ball (we found it!)
  ai->st.bvx=p->cx-ai->st.old_bcx;	// Update ball velocity in ai structure and blob structure
  ai->st.bvy=p->cy-ai->st.old_bcy;
  ai->st.ball->vx=ai->st.bvx;
  ai->st.ball->vy=ai->st.bvy;
  ai->st.bdx=p->dx;
  ai->st.bdy=p->dy;

  ai->st.old_bcx=p->cx; 		// Update old position for next frame's computation
  ai->st.old_bcy=p->cy;
  ai->st.ball->idtype=3;

  vx=ai->st.bvx;			// Compute motion direction (normalized motion vector)
  vy=ai->st.bvy;
  mg=sqrt((vx*vx)+(vy*vy));
  if (mg>NOISE_VAR)			// Update heading vector if meaningful motion detected
  {
   vx/=mg;
   vy/=mg;
   ai->st.bmx=vx;
   ai->st.bmy=vy;
  }
  else
  {
    ai->st.bmx=0;
    ai->st.bmy=0;
  }
  ai->st.ball->mx=ai->st.bmx;
  ai->st.ball->my=ai->st.bmy;
 }
 else {
  ai->st.ball=NULL;
 }
 
 // ID our bot - the colour is set from commane line, 0=Blue, 1=Red
 p=id_coloured_blob2(ai,blobs,ai->st.botCol);
 if (p!=NULL&&p!=ai->st.ball)
 {
  ai->st.self=p;			// Update pointer to self-blob
  ai->st.selfID=1;
  ai->st.svx=p->cx-ai->st.old_scx;
  ai->st.svy=p->cy-ai->st.old_scy;
  ai->st.self->vx=ai->st.svx;
  ai->st.self->vy=ai->st.svy;
  ai->st.sdx=p->dx;
  ai->st.sdy=p->dy;

  vx=ai->st.svx;
  vy=ai->st.svy;
  mg=sqrt((vx*vx)+(vy*vy));
//  printf("--->    Track agents(): d=[%lf, %lf], [x,y]=[%3.3lf, %3.3lf], old=[%3.3lf, %3.3lf], v=[%2.3lf, %2.3lf], motion=[%2.3lf, %2.3lf]\n",ai->st.sdx,ai->st.sdy,ai->st.self->cx,ai->st.self->cy,ai->st.old_scx,ai->st.old_scy,vx,vy,vx/mg,vy/mg);
  if (mg>NOISE_VAR)
  {
   vx/=mg;
   vy/=mg;
   ai->st.smx=vx;
   ai->st.smy=vy;
  }
  else
  {
   ai->st.smx=0;
   ai->st.smy=0;
  }
  ai->st.self->mx=ai->st.smx;
  ai->st.self->my=ai->st.smy;
  ai->st.old_scx=p->cx; 
  ai->st.old_scy=p->cy;
  ai->st.self->idtype=1;
 }
 else ai->st.self=NULL;

 // ID our opponent - whatever colour is not botCol
 if (ai->st.botCol==0) p=id_coloured_blob2(ai,blobs,1);
 else p=id_coloured_blob2(ai,blobs,0);
 if (p!=NULL&&p!=ai->st.ball&&p!=ai->st.self)
 {
  ai->st.opp=p;	
  ai->st.oppID=1;
  ai->st.ovx=p->cx-ai->st.old_ocx;
  ai->st.ovy=p->cy-ai->st.old_ocy;
  ai->st.opp->vx=ai->st.ovx;
  ai->st.opp->vy=ai->st.ovy;
  ai->st.odx=p->dx;
  ai->st.ody=p->dy;

  ai->st.old_ocx=p->cx; 
  ai->st.old_ocy=p->cy;
  ai->st.opp->idtype=2;

  vx=ai->st.ovx;
  vy=ai->st.ovy;
  mg=sqrt((vx*vx)+(vy*vy));
  if (mg>NOISE_VAR)
  {
   vx/=mg;
   vy/=mg;
   ai->st.omx=vx;
   ai->st.omy=vy;
  }
  else
  {
   ai->st.omx=0;
   ai->st.omy=0;
  }
  ai->st.opp->mx=ai->st.omx;
  ai->st.opp->my=ai->st.omy;
 }
 else ai->st.opp=NULL;

}

void id_bot(struct RoboAI *ai, struct blob *blobs)
{
 ///////////////////////////////////////////////////////////////////////////////
 // ** DO NOT CHANGE THIS FUNCTION **
 // This routine calls track_agents() to identify the blobs corresponding to the
 // robots and the ball. It commands the bot to move forward slowly so heading
 // can be established from blob-tracking.
 //
 // NOTE 1: All heading estimates, velocity vectors, position, and orientation
 //         are noisy. Remember what you have learned about noise management.
 //
 // NOTE 2: Heading and velocity estimates are not valid while the robot is
 //         rotating in place (and the final heading vector is not valid either).
 //         To re-establish heading, forward/backward motion is needed.
 //
 // NOTE 3: However, you do have a reliable orientation vector within the blob
 //         data structures derived from blob shape. It points along the long
 //         side of the rectangular 'uniform' of your bot. It is valid at all
 //         times (even when rotating), but may be pointing backward and the
 //         pointing direction can change over time.
 //
 // You should *NOT* call this function during the game. This is only for the
 // initialization step. Calling this function during the game will result in
 // unpredictable behaviour since it will update the AI state.
 ///////////////////////////////////////////////////////////////////////////////
 
 struct blob *p;
 static double stepID=0;
 static double oldX,oldY;
 double frame_inc=1.0/5.0;
 double dist;
 
 track_agents(ai,blobs);		// Call the tracking function to find each agent

 BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 30);			// Start forward motion to establish heading
                                                // Will move for a few frames.
  
 if (ai->st.selfID==1&&ai->st.self!=NULL)
  fprintf(stderr,"Successfully identified self blob at (%f,%f)\n",ai->st.self->cx,ai->st.self->cy);
 if (ai->st.oppID==1&&ai->st.opp!=NULL)
  fprintf(stderr,"Successfully identified opponent blob at (%f,%f)\n",ai->st.opp->cx,ai->st.opp->cy);
 if (ai->st.ballID==1&&ai->st.ball!=NULL)
  fprintf(stderr,"Successfully identified ball blob at (%f,%f)\n",ai->st.ball->cx,ai->st.ball->cy);

 stepID+=frame_inc;
 if (stepID>=1&&ai->st.selfID==1)	// Stop after a suitable number of frames.
 {
  ai->st.state+=1;
  stepID=0;
  BT_all_stop(0);
 }
 else if (stepID>=1) stepID=0;

 // At each point, each agent currently in the field should have been identified.
 return;
}
/*********************************************************************************
 * End of blob ID and tracking code
 * ******************************************************************************/

/*********************************************************************************
 * Routine to initialize the AI 
 * *******************************************************************************/
int setupAI(int mode, int own_col, struct RoboAI *ai)
{
 /////////////////////////////////////////////////////////////////////////////
 // ** DO NOT CHANGE THIS FUNCTION **
 // This sets up the initial AI for the robot. There are three different modes:
 //
 // SOCCER -> Complete AI, tries to win a soccer game against an opponent
 // PENALTY -> Score a goal (no goalie!)
 // CHASE -> Kick the ball and chase it around the field
 //
 // Each mode sets a different initial state (0, 100, 200). Hence, 
 // AI states for SOCCER will be 0 through 99
 // AI states for PENALTY will be 100 through 199
 // AI states for CHASE will be 200 through 299
 //
 // You will of course have to add code to the AI_main() routine to handle
 // each mode's states and do the right thing.
 //
 // Your bot should not become confused about what mode it started in!
 //////////////////////////////////////////////////////////////////////////////        

 switch (mode) {
 case AI_SOCCER:
	fprintf(stderr,"Standard Robo-Soccer mode requested\n");
        ai->st.state=0;		// <-- Set AI initial state to 0
        break;
 case AI_PENALTY:
// 	fprintf(stderr,"Penalty mode! let's kick it!\n");
	ai->st.state=100;	// <-- Set AI initial state to 100
        break;
 case AI_CHASE:
	fprintf(stderr,"Chasing the ball...\n");
	ai->st.state=200;	// <-- Set AI initial state to 200
        break;	
 default:
	fprintf(stderr, "AI mode %d is not implemented, setting mode to SOCCER\n", mode);
	ai->st.state=0;
	}

 BT_all_stop(0);			// Stop bot,
 ai->runAI = AI_main;		// and initialize all remaining AI data
 ai->calibrate = AI_calibrate;
 ai->st.ball=NULL;
 ai->st.self=NULL;
 ai->st.opp=NULL;
 ai->st.side=0;
 ai->st.botCol=own_col;
 ai->st.old_bcx=0;
 ai->st.old_bcy=0;
 ai->st.old_scx=0;
 ai->st.old_scy=0;
 ai->st.old_ocx=0;
 ai->st.old_ocy=0;
 ai->st.bvx=0;
 ai->st.bvy=0;
 ai->st.svx=0;
 ai->st.svy=0;
 ai->st.ovx=0;
 ai->st.ovy=0;
 ai->st.sdx=0;
 ai->st.sdy=0;
 ai->st.odx=0;
 ai->st.ody=0;
 ai->st.bdx=0;
 ai->st.bdy=0;
 ai->st.selfID=0;
 ai->st.oppID=0;
 ai->st.ballID=0;
 ai->DPhead=NULL;
 fprintf(stderr,"Initialized!\n");

 return(1);
}

void AI_calibrate(struct RoboAI *ai, struct blob *blobs)
{
 // Basic colour blob tracking loop for calibration of vertical offset
 // See the handout for the sequence of steps needed to achieve calibration.
 // The code here just makes sure the image processing loop is constantly
 // tracking the bots while they're placed in the locations required
 // to do the calibration (i.e. you DON'T need to add anything more
 // in this function).
 track_agents(ai,blobs);
}


/**************************************************************************
 * AI state machine - this is where you will implement your soccer
 * playing logic
 * ************************************************************************/
double oldXHeading = -1000.0;
double oldYHeading = -1000.0;
double robustBallCx = -1000;
double robustBallCy = -1000;
double robustSelfCx = -1000;
double robustSelfCy = -1000;
double robustEnemyCx = -1000;
double robustEnemyCy = -1000;

// ============== Variables ==============
int TRANSITION_TABLE[300][NUMBER_OF_EVENTS * 2]; // %2==0 means we don't want event to happen, odd means we do
char motor_powers[9] = {-1, -1, -1, -1, -1, -1, -1, -1, -1};
int wanted_posX = -1;
int wanted_posY = -1;
double thresholdStrictness;
int takeShot = 0;
int numVeryHugeTurn = 0;
int lastAppliedToRetract = -1; // refactor this
int terminateAtEnd = 1; // refactor this



void AI_main(struct RoboAI *ai, struct blob *blobs, void *state)
{
 /*************************************************************************
  This is your robot's state machine.
  
  It is called by the imageCapture code *once* per frame. And it *must not*
  enter a loop or wait for visual events, since no visual refresh will happen
  until this call returns!
  
  Therefore. Everything you do in here must be based on the states in your
  AI and the actions the robot will perform must be started or stopped 
  depending on *state transitions*. 

  E.g. If your robot is currently standing still, with state = 03, and
   your AI determines it should start moving forward and transition to
   state 4. Then what you must do is 
   - send a command to start forward motion at the desired speed
   - update the robot's state
   - return
  
  I can not emphasize this enough. Unless this call returns, no image
  processing will occur, no new information will be processed, and your
  bot will be stuck on its last action/state.

  You will be working with a state-based AI. You are free to determine
  how many states there will be, what each state will represent, and
  what actions the robot will perform based on the state as well as the
  state transitions.

  You must *FULLY* document your state representation in the report

  The first two states for each more are already defined:
  State 0,100,200 - Before robot ID has taken place (this state is the initial
            	    state, or is the result of pressing 'r' to reset the AI)
  State 1,101,201 - State after robot ID has taken place. At this point the AI
            	    knows where the robot is, as well as where the opponent and
            	    ball are (if visible on the playfield)

  Relevant UI keyboard commands:
  'r' - reset the AI. Will set AI state to zero and re-initialize the AI
	data structure.
  't' - Toggle the AI routine (i.e. start/stop calls to AI_main() ).
  'o' - Robot immediate all-stop! - do not allow your EV3 to get damaged!

   IMPORTANT NOTE: There are TWO sources of information about the 
                   location/parameters of each agent
                   1) The 'blob' data structures from the imageCapture module
                   2) The values in the 'ai' data structure.
                      The 'blob' data is incomplete and changes frame to frame
                      The 'ai' data should be more robust and stable
                      BUT in order for the 'ai' data to be updated, you
                      must call the function 'track_agents()' in your code
                      after eah frame!
                      
    DATA STRUCTURE ORGANIZATION:

    'RoboAI' data structure 'ai'
         \    \    \   \--- calibrate()  (pointer to AI_clibrate() )
          \    \    \--- runAI()  (pointer to the function AI_main() )
           \    \------ Display List head pointer 
            \_________ 'ai_data' data structure 'st'
                         \  \   \------- AI state variable and other flags
                          \  \---------- pointers to 3 'blob' data structures
                           \             (one per agent)
                            \------------ parameters for the 3 agents
                              
  ** Do not change the behaviour of the robot ID routine **
 **************************************************************************/

  static double ux,uy,len,mmx,mmy,tx,ty,x1,y1,x2,y2;
  double angDif;
  char line[1024];
  static int count=0;
  static double old_dx=0, old_dy=0;
      
  /************************************************************
   * Standard initialization routine for starter code,
   * from state **0 performs agent detection and initializes
   * directions, motion vectors, and locations
   * Triggered by toggling the AI on.
   * - Modified now (not in starter code!) to have local
   *   but STATIC data structures to keep track of robot
   *   parameters across frames (blob parameters change
   *   frame to frame, memoryless).
   ************************************************************/
 if (ai->st.state==0||ai->st.state==100||ai->st.state==200)  	// Initial set up - find own, ball, and opponent blobs
 {
  // Carry out self id process.
  fprintf(stderr,"Initial state, self-id in progress...\n");
  
  id_bot(ai,blobs);
  if ((ai->st.state%100)!=0)	  // The id_bot() routine will change the AI state to initial state + 1
  {				                 // if robot identification is successful.
      
    if (ai->st.self->cx>=512) ai->st.side=1; else ai->st.side=0;         // This sets the side the bot thinks as its own side 0->left, 1->right
    BT_all_stop(0);
    
    fprintf(stderr,"Self-ID complete. Current position: (%f,%f), current heading: [%f, %f], blob direction=[%f, %f], AI state=%d\n",ai->st.self->cx,ai->st.self->cy,ai->st.smx,ai->st.smy,ai->st.sdx,ai->st.sdy,ai->st.state);
    
    if (ai->st.self!=NULL)
    {
        // This checks that the motion vector and the blob direction vector
        // are pointing in the same direction. If they are not (the dot product
        // is less than 0) it inverts the blob direction vector so it points
        // in the same direction as the motion vector.
        if (((ai->st.smx*ai->st.sdx)+(ai->st.smy*ai->st.sdy))<0)
        {
            ai->st.self->dx*=-1.0;
            ai->st.self->dy*=-1.0;
            ai->st.sdx*=-1;
            ai->st.sdy*=-1;
        }
        old_dx=ai->st.sdx;
        old_dy=ai->st.sdy;
    }
    
    if (ai->st.opp!=NULL)
    {
        // Checks motion vector and blob direction for opponent. See above.
        if (((ai->st.omx*ai->st.odx)+(ai->st.omy*ai->st.ody))<0)
        {
            ai->st.opp->dx*=-1;
            ai->st.opp->dy*=-1;
            ai->st.odx*=-1;
            ai->st.ody*=-1;
        }       
    }
    // Initialize BotInfo structures

    // Start by creating transition table
    // TODO: move this to a seperate config file
    printf("Initializing table\n");
    fflush(stdout);

    for (int i = 0; i < 300; i++){
      for (int j = 0; j < NUMBER_OF_EVENTS*2; j++) TRANSITION_TABLE[i][j] = -1;
    }
    
    if (ai->st.state>=200){
      // TODO: add something for chase 

    }else if (ai->st.state>=100){
      // T_T[STATE][EVENT * 2 + wantedToBeTrue] = newState
      // State 101
      TRANSITION_TABLE[STATE_P_hold][EVENT_carSeen * 2 + 1] = STATE_P_alignWithOffset;

      // State 102
      TRANSITION_TABLE[STATE_P_alignWithOffset][EVENT_carSeen * 2 + 0] = STATE_P_hold;
      TRANSITION_TABLE[STATE_P_alignWithOffset][EVENT_allignedWithPosition * 2 + 1] = STATE_P_driveToOffset;

      // State 103
      TRANSITION_TABLE[STATE_P_driveToOffset][EVENT_carSeen * 2 + 0] = STATE_P_hold;
      TRANSITION_TABLE[STATE_P_driveToOffset][EVENT_allignedWithPosition * 2 + 0] = STATE_P_alignWithOffset;
      TRANSITION_TABLE[STATE_P_driveToOffset][EVENT_atWantedPosition * 2 + 1] = STATE_P_alignWithBall;
      
      // State 104
      TRANSITION_TABLE[STATE_P_alignWithBall][EVENT_carSeen * 2 + 0] = STATE_P_hold;
      TRANSITION_TABLE[STATE_P_alignWithBall][EVENT_allignedWithPosition * 2 + 1] = STATE_P_driveCarefullyUntilShot;

      // State 105
      TRANSITION_TABLE[STATE_P_driveCarefullyUntilShot][EVENT_carSeen * 2 + 0] = STATE_P_hold;
      TRANSITION_TABLE[STATE_P_driveCarefullyUntilShot][EVENT_allignedWithPosition * 2 + 0] = STATE_P_alignWithBall;

    } else{ 
      // TODO: Add initial movement stage to verify heading before potentially doing a 180 flip
      TRANSITION_TABLE[STATE_S_think][EVENT_weWinRaceToBall* 2 + 0] = STATE_S_curveToInterceptBall;
      TRANSITION_TABLE[STATE_S_think][EVENT_weWinRaceToBall* 2 + 1] = STATE_S_curveToBall;//STATE_S_curveToShootingPosition;

      // Attack states
      TRANSITION_TABLE[STATE_S_curveToBall][EVENT_weWinRaceToBall* 2 + 0] = STATE_S_think;
      TRANSITION_TABLE[STATE_S_curveToBall][EVENT_atWantedPosition* 2 + 1] = STATE_S_alignRobotToShoot;//STATE_S_alignRobotToShoot;

      // get ball in pouch changes states within its own complex logic
      // STATE_S_OrientBallandShoot also does its own logic
      // TODO: move logic step to here ...

      TRANSITION_TABLE[STATE_S_alignRobotToShoot][EVENT_weWinRaceToBall* 2 + 0] = STATE_S_curveToInterceptBall;
      //TRANSITION_TABLE[STATE_S_alignRobotToShoot][EVENT_ballMoving* 2 + 1] = STATE_S_curveToShootingPosition;
      TRANSITION_TABLE[STATE_S_alignRobotToShoot][EVENT_allignedWithPosition* 2 + 1] = STATE_S_getBallInPouch;
      //TRANSITION_TABLE[STATE_S_alignRobotToShoot][EVENT_atWantedPosition* 2 + 0] = STATE_S_curveToBall;


      TRANSITION_TABLE[STATE_S_getBallInPouch][EVENT_ballIsConsiderablyFar* 2 + 1] = STATE_S_curveToBall;
      TRANSITION_TABLE[STATE_S_getBallInPouch][EVENT_allignedWithPosition * 2 + 0] = STATE_S_alignRobotToShoot;

      // STATE_S_runAtBallAndShoot will automatically curve to ball and go back to think after making a shot
      //TRANSITION_TABLE[STATE_S_runAtBallAndShoot][EVENT_robotIsStuckOnEnemy* 2 + 1] = STATE_S_FIGHT;
      //TRANSITION_TABLE[STATE_S_runAtBallAndShoot][EVENT_ballIsConsiderablyFar * 2 + 1] = STATE_S_think;

      // Defense states
      //TRANSITION_TABLE[STATE_S_curveToInterceptBall][EVENT_weWinRaceToBall* 2 + 1] = STATE_S_think;
      //TRANSITION_TABLE[STATE_S_curveToInterceptBall][EVENT_atWantedPosition* 2 + 1] = STATE_S_alignWithBallBeforeCreep;

      // Reorient
      //TRANSITION_TABLE[STATE_S_alignWithBallBeforeCreep][EVENT_allignedWithPosition* 2 + 1] = STATE_S_creepSlowlyToBall;

      // creep state aligns and moves forward;
      //TRANSITION_TABLE[STATE_S_creepSlowlyToBall][EVENT_weWinRaceToBall* 2 + 1] = STATE_S_alignRobotToShoot;
      //TRANSITION_TABLE[STATE_S_creepSlowlyToBall][EVENT_allignedWithPosition* 2 + 0] = STATE_S_alignWithBallBeforeCreep;
      //TRANSITION_TABLE[STATE_S_creepSlowlyToBall][EVENT_ballMoving* 2 + 1] = STATE_S_think;
    } 
  }
 }
 else
 {
  /****************************************************************************
   TO DO:
   You will need to replace this 'catch-all' code with actual program logic to
   implement your bot's state-based AI.

   After id_bot() has successfully completed its work, the state should be
   1 - if the bot is in SOCCER mode
   101 - if the bot is in PENALTY mode
   201 - if the bot is in CHASE mode

   Your AI code needs to handle these states and their associated state
   transitions which will determine the robot's behaviour for each mode.

   Please note that in this function you should add appropriate functions below
   to handle each state's processing, and the code here should mostly deal with
   state transitions and with calling the appropriate function based on what
   the bot is supposed to be doing.
  *****************************************************************************/
  //fprintf(stderr,"Just trackin'!\n");	// bot, opponent, and ball.
    track_agents(ai,blobs);		// Currently, does nothing but endlessly track
    
    fixAIHeadingDirection(ai);
    updateRobustPositions(ai);

    // Update state based on transition
    for (int i = 0; i < NUMBER_OF_EVENTS * 2; i++){
        if (TRANSITION_TABLE[ai->st.state][i] > -1){
            if (checkEventActive(ai, i)){
                printf("ABOUT TO CHANGE FROM %d state due to %d event", ai->st.state, i);
                changeMachineState(ai, TRANSITION_TABLE[ai->st.state][i]);
                break;
            }
        }
    }

    // Call function for state action
    handleStateActions(ai);

    // Call function to retract shooting mechanism or release
    handleShootingMechanism(ai);
    
  }
}

/**********************************************************************************
 TO DO:

 Add the rest of your game playing logic below. Create appropriate functions to
 handle different states (be sure to name the states/functions in a meaningful
 way), and do any processing required in the space below.

 AI_main() should *NOT* do any heavy lifting. It should only call appropriate
 functions based on the current AI state.

 You will lose marks if AI_main() is cluttered with code that doesn't belong
 there.
**********************************************************************************/
int wrong_path_beleif = 0;
struct coord lastDrivingPosition;

void fixAIHeadingDirection(struct RoboAI *ai){
    if (ai->st.self != NULL && oldXHeading!= -1000){
      int isDriving = get_curr_motor_power(MOTOR_DRIVE_LEFT) > 0 && get_curr_motor_power(MOTOR_DRIVE_RIGHT) > 0;
      int isTurning = get_curr_motor_power(MOTOR_DRIVE_LEFT) > 0 && get_curr_motor_power(MOTOR_DRIVE_RIGHT) < 0 ||
                      get_curr_motor_power(MOTOR_DRIVE_LEFT) < 0 && get_curr_motor_power(MOTOR_DRIVE_RIGHT) > 0;
      
      if (pow(pow(ai->st.sdx - oldXHeading, 2) + pow(ai->st.sdy - oldYHeading, 2), 0.5) > 
          getExpectedUnitCircleDistance(PI * 1.0 / 2.0)){ 
          // Too big of a jump, assume its a flip or rando jump and flip it
          ai->st.sdy *= -1;
          ai->st.sdx *= -1;
        }

      // Detect outlier jump
      if (!isTurning && pow(pow(ai->st.sdx - oldXHeading, 2) + pow(ai->st.sdy - oldYHeading, 2), 0.5) > 
          getExpectedUnitCircleDistance(PI * 1.0 / 4.0)){ 
        if (numVeryHugeTurn >= 3){ 
          // 3rd frame in a row we're reading far from heading, go back to trusting it
          ai->st.sdy = oldYHeading;
          ai->st.sdx = oldXHeading;
        }
        numVeryHugeTurn += 1;
      }else{
        numVeryHugeTurn = 0;
      }
      
      
      // Detect if we are actually going reverse of expected
      if (isDriving){ 
        struct coord velocityVector = normalize_vector(new_coords(robustSelfCx - lastDrivingPosition.x, robustSelfCy - lastDrivingPosition.y));
        //printf("Readings %f,%f and %f,%f\n", ai->st.sdx, ai->st.sdy, velocityVector.x, velocityVector.y);
        if (pow(pow(ai->st.sdx - velocityVector.x, 2) + pow(ai->st.sdy - velocityVector.y, 2), 0.5) > getExpectedUnitCircleDistance(PI * 1.0 / 2.0)){
          wrong_path_beleif++;

          printf("Detecting m to d anamoly %d\n", wrong_path_beleif);
          fflush(stdout);
          if (wrong_path_beleif == 2){
            printf("DECIDED WRONG DIRECTION HEADING\n");
            fflush(stdout);
            ai->st.sdy *= -1;
            ai->st.sdx *= -1;
            wrong_path_beleif = -3;
          }

        }else{
          wrong_path_beleif = -3;
        }

        lastDrivingPosition.x = robustSelfCx;
        lastDrivingPosition.y = robustSelfCy;
      }

    }

    // Recall headings so we can continue to ignore flips
    oldXHeading = ai->st.sdx;
    oldYHeading = ai->st.sdy;
}

void updateRobustPositions(struct RoboAI *ai){
  if (ai->st.ball != NULL){
    robustBallCx = ai->st.ball->cx;
    robustBallCy = ai->st.ball->cy;
  }

  if (ai->st.self != NULL){
    robustSelfCx = ai->st.self->cx;
    robustSelfCy = ai->st.self->cy;
  }

  if (ai->st.opp != NULL){
    robustEnemyCx = ai->st.opp->cx;
    robustEnemyCy = ai->st.opp->cy;
  }
}


void handleShootingMechanism(struct RoboAI *ai){
  
  if (takeShot){
    printf("Taking shot\n");
    fflush(stdout);

    // Keep pulling until not retract
    BT_timed_motor_port_start_v2(MOTOR_SHOOT_RETRACT, -100, 500);
    takeShot = 0;
    lastAppliedToRetract = 0;
    BT_motor_port_stop(MOTOR_SHOOT_RETRACT, 0);

    if (terminateAtEnd){ // Penalty mode, halt the program
      changeMachineState(ai, STATE_P_done);
    }  else{
      changeMachineState(ai, STATE_P_hold);
    }

  }else{ // Maintain retracted
    //printf("Maintaining shot\n");
    //fflush(stdout);
    int retraction = checkEventActive(ai, EVENT_shootingMechanismRetracted * 2 + 1);
    //printf("Retraction: %d\n", retraction);
    //fflush(stdout);

    if (retraction){
        if (lastAppliedToRetract != 0){
            lastAppliedToRetract = 0;
            BT_motor_port_stop(MOTOR_SHOOT_RETRACT, 0);
        }
    }else {
        if (lastAppliedToRetract != 1){
            lastAppliedToRetract = 1;
            //printf("Does this stall \n");
            //fflush(stdout);
            BT_motor_port_start(MOTOR_SHOOT_RETRACT, -60);
            //printf("No! \n");
            //fflush(stdout);
        }
    }
  }
}

double getPowerNeededToAlign(struct RoboAI *ai, double wanted_posX, double wanted_posY){
    double dir1 = ai->st.sdx;
    double dir2 = ai->st.sdy;

    double unit_diff = wanted_posX - robustSelfCx;
    if (dir1 == 0) dir1= 0.001;
    if (unit_diff == 0) unit_diff = 0.001;

    double units_moved = unit_diff / dir1;

    struct coord expectedVector = normalize_vector(new_coords(wanted_posX - robustSelfCx, wanted_posY - robustSelfCy));
    double vectorOffsets = pow(pow(expectedVector.x - dir1, 2) + pow(expectedVector.y - dir2, 2), 0.5);
    if (units_moved > 0 && vectorOffsets < getExpectedUnitCircleDistance(thresholdStrictness)){
      return 0.0;
    }

    double result_y = robustSelfCy + units_moved * dir2;
    
    int dir = 1;
    if (result_y < wanted_posY) dir *= -1;
    if (units_moved < 0) dir *= -1;
    if (robustSelfCx < wanted_posX) dir *= -1;

    double total_power = 25*vectorOffsets;
    if (total_power < 9) total_power = 9;
    if (total_power > 30) total_power = 30;
    if (units_moved < 0) total_power = 45 - (total_power*0.5);
    return total_power * dir;
}

int checkEventActive(struct RoboAI *ai, int event){
    int wantedResult = event % 2;
    int checkingEvent = (event - wantedResult) / 2;
    int result = 0; // set to 1 if event happens

    if (checkingEvent == EVENT_carSeen){
        result = ai->st.self != NULL;

    }else if (checkingEvent == EVENT_atWantedPosition){
        // Add distance checker
        double dist = pow(pow(robustSelfCx - wanted_posX, 2) + pow(robustSelfCy - wanted_posY, 2), 0.5);
        result = dist <= 40;

    }else if (checkingEvent == EVENT_allignedWithPosition){
        result = getPowerNeededToAlign(ai, wanted_posX, wanted_posY) == 0.0;
        
    }else if (checkingEvent == EVENT_ballIsInCage){
        // Read colour sensor reflectance
        int RGB[3];
        BT_read_colour_sensor_RGB(COLOUR_SENSOR_INPUT, RGB);
        result = RGB[2] > 20;

    }else if (checkingEvent == EVENT_shootingMechanismRetracted){
        result = BT_read_touch_sensor(TOUCH_SENSOR_INPUT);

    }else if (checkingEvent == EVENT_ballCagedAndCanShoot){
        result = checkEventActive(ai, EVENT_ballIsInCage * 2 + 1) && checkEventActive(ai, EVENT_shootingMechanismRetracted * 2 + 1);

    }else if (checkingEvent == EVENT_robotIsStuckOnWall){
        result = 0;
        // TODO: implement

    }else if (checkingEvent == EVENT_robotIsStuckOnEnemy){
        result = 0;
        // TODO: implement

    }else if (checkingEvent == EVENT_weWinRaceToBall){
      double ourDistToBall = 0; //pow(pow(robustSelfCx - robustBallCx, 2) + pow(robustSelfCy - robustBallCy, 2), 0.5);
      double thierDistToBall = 10000; //pow(pow(robustEnemyCx - robustBallCx, 2) + pow(robustEnemyCy - robustBallCy, 2), 0.5);
      printf("DISTANCE TO BALL CHECK %f\n", ourDistToBall);
      fflush(stdout);
      if (ourDistToBall <= 300 && thierDistToBall <= 200){
        result = 1; // TODO: split this for EVENT_bothRobotsCloseToBall and make that event direct us to fight state
      }else if (ourDistToBall <= 300 ){
        result = 1;
      }else if (thierDistToBall <= 200){
        result = 0;
      }else{
        result = 0; //ourDistToBall * 1.6 > thierDistToBall;
      }
      


    }else if (checkingEvent == EVENT_ballMoving){
        result = 0;
        // TODO: implement

    }else if (checkingEvent == EVENT_headingAwayFromwantedPosition){
        result = 1;
        // TODO: implement

    }else if (checkingEvent == EVENT_alignedToScore){
      struct coord net = getNet(ai->st.side);
      double dir1 = ai->st.sdx;
      double dir2 = ai->st.sdy;

      double unit_diff = net.x - robustBallCx;
      if (dir1 == 0) dir1= 0.001;
      if (unit_diff == 0) unit_diff = 0.001;

      double units_moved = unit_diff / dir1;
      double result_y = robustSelfCy + units_moved * dir2;
      result = fabs(result_y - net.y) < 50;

    }else if (checkingEvent == EVENT_ballIsConsiderablyFar){
      return pow(pow(robustSelfCx - robustBallCx, 2) + pow(robustSelfCy - robustBallCy, 2), 0.5) > 300;
    }

    return (result == wantedResult);
}

void changeMachineState(struct RoboAI *ai, int new_state){
    // Check for special conditions (stop motors etc)
    printf("SWICHING TO STATE %d\n", new_state);
    fflush(stdout);

    ai->st.state = new_state;
    if (new_state == STATE_P_driveToOffset || new_state == STATE_P_driveCarefullyUntilShot || new_state == STATE_S_getBallInPouch || new_state == STATE_S_creepSlowlyToBall){
      lastDrivingPosition = new_coords(ai->st.old_scx, ai->st.old_scy);
      wrong_path_beleif = -3; // Give us a new iterations to fix up the velocity calculations
    }else{
      thresholdStrictness = PI/15;
    }

    if (new_state == STATE_P_alignWithBall || new_state == STATE_P_alignWithOffset || new_state == STATE_S_alignRobotToShoot || STATE_S_alignWithBallBeforeCreep || new_state == STATE_S_OrientBallandShoot ){
        wanted_posX = -1;
        wanted_posY = -1;
        if (new_state == STATE_S_alignRobotToShoot) thresholdStrictness = PI/30; // set tigher strictness for allignment to wanted position, as its relevant to shooting
        else if (new_state == STATE_P_alignWithBall) thresholdStrictness = PI/30;
    }
    
    if (new_state == STATE_S_curveToInterceptBall){
      struct coord ourNetLoc = getNet(1 - ai->st.side);
      int d = 2 *(ourNetLoc.x == 0) - 1;
      ourNetLoc.x += d * 125;
      wanted_posX = ourNetLoc.x;
      wanted_posY = ourNetLoc.y;
      printf("Net offset; %d %d\n", wanted_posX, wanted_posY);
    }
    motor_power_async(MOTOR_DRIVE_LEFT, 0);
    motor_power_async(MOTOR_DRIVE_RIGHT, 0);

    if (new_state == STATE_P_done){
      exit(0);
    }
}

void handleAlignWithGivenOffset(struct RoboAI *ai, double offset){
  // Set wantedX and wantedY
  if (robustBallCx != -1000){
      // call his calculations
      struct coord location = calc_in_front_of_ball(ai, offset, getNet(ai->st.side));
      wanted_posX = location.x;
      wanted_posY = location.y;
  }

  if (wanted_posX != -1 && wanted_posY != -1){
      double power = getPowerNeededToAlign(ai, wanted_posX, wanted_posY);
      printf("DECIDING TO LINE UP WITH POWER %f \n", power);
      motor_power_async(MOTOR_DRIVE_LEFT, power);
      motor_power_async(MOTOR_DRIVE_RIGHT, -power); 
  }
}

void handleCurveToGivenLocation(struct RoboAI* ai){
    double curvePower = getPowerNeededToAlign(ai, wanted_posX, wanted_posY);
    double dist = pow(pow(robustSelfCx - wanted_posX, 2) + pow(robustSelfCy - wanted_posY, 2), 0.5);
    double pushPower = dist*60/500.0;
    if (pushPower > 65) pushPower = 65;
    else if (pushPower < 15) pushPower = 20; 

    double abs_curve = fabs(curvePower);
    if (abs_curve >= 30 && abs_curve < 40){ // just spin, we face the wrong way
      motor_power_async(MOTOR_DRIVE_LEFT, curvePower);
      motor_power_async(MOTOR_DRIVE_RIGHT, -curvePower); 

    }else if (abs_curve == 0){ // Very alligned, rush
      motor_power_async(MOTOR_DRIVE_LEFT, pushPower + 15);
      motor_power_async(MOTOR_DRIVE_RIGHT, pushPower + 15); 

    }else{
      double dir = 1;
      if (abs_curve >= 40){ // we can get to offset backwards
        dir = -1;
        pushPower -= 5;
      }

      //double res = 
      motor_power_async(MOTOR_DRIVE_LEFT, dir*(pushPower + curvePower));
      motor_power_async(MOTOR_DRIVE_RIGHT, dir*(pushPower - curvePower)); 
    }
}

double ALIGN_OFFSET = -250; // TODO: make it dynamic 
int driftingInPouch = 0;
void handleStateActions(struct RoboAI *ai){
    // Returns 1 if we want to asynchronously shoot
    int state = ai->st.state;
    if (state >= 200){

    } else if (state >= 100){
        if (state == STATE_P_hold) return;
        else if (state == STATE_P_alignWithOffset){
            handleAlignWithGivenOffset(ai, ALIGN_OFFSET);
            wrong_path_beleif = -3;

        }else if (state == STATE_P_driveToOffset){
            if (robustBallCx != -1000){
              struct coord location = calc_in_front_of_ball(ai, ALIGN_OFFSET, getNet(ai->st.side));
              wanted_posX = location.x;
              wanted_posY = location.y;
            }

            // Drive at 0.35
          if (wanted_posX != -1 && wanted_posY != -1){
              double dist = pow(pow(robustSelfCx - wanted_posX, 2) + pow(robustSelfCy - wanted_posY, 2), 0.5);
              double powerApplied = dist*35/500.0;
              if (powerApplied < 12.5) powerApplied = 12.5;
              if (powerApplied > 65) powerApplied = 65;
              motor_power_async(MOTOR_DRIVE_LEFT, powerApplied);
              motor_power_async(MOTOR_DRIVE_RIGHT, powerApplied);
            }

        } else if (state == STATE_P_alignWithBall){
            handleAlignWithGivenOffset(ai, 0);
            
        }else if (state == STATE_P_driveCarefullyUntilShot){

            wanted_posX = robustBallCx;
            wanted_posY = robustBallCy;

            if (checkEventActive(ai, EVENT_ballCagedAndCanShoot * 2 + 1)){
                takeShot = 1;
            }

            // Drive at 15
            motor_power_async(MOTOR_DRIVE_LEFT, 17);
            motor_power_async(MOTOR_DRIVE_RIGHT, 17);
        }

    } else{
        if (state == STATE_S_curveToBall){
          if (robustBallCx != -1000){
            struct coord location = calc_in_front_of_ball(ai, -ALIGN_OFFSET, new_coords(robustSelfCx, robustSelfCy));
            wanted_posX = location.x;
            wanted_posY = location.y;
          }

          if (wanted_posX != -1 && wanted_posY != -1){
            handleCurveToGivenLocation(ai);
          }

        }else if (state == STATE_S_curveToInterceptBall){
          handleCurveToGivenLocation(ai);

        }else if (state == STATE_S_alignRobotToShoot){
          handleAlignWithGivenOffset(ai, 0);

        }else if (state == STATE_S_alignWithBallBeforeCreep){
          // TODO: add some interesting checkers so we dont turn into scoring a goal on ourselves (as this is called after reaching goalie)
          handleAlignWithGivenOffset(ai, 0);
          
        }else if (state == STATE_S_getBallInPouch){
          wanted_posX = robustBallCx;
          wanted_posY = robustBallCy;

          if (checkEventActive(ai, EVENT_ballIsInCage * 2 + 1)){
            driftingInPouch = 0;
            printf("START THE DRIFT??\n");
            fflush(stdout);
            changeMachineState(ai, STATE_S_OrientBallandShoot);
          }

          // Drive at 15
          motor_power_async(MOTOR_DRIVE_LEFT, 17);
          motor_power_async(MOTOR_DRIVE_RIGHT, 17);


        }else if (state == STATE_S_OrientBallandShoot){
          wanted_posX = robustBallCx;
          wanted_posY = robustBallCy;
          struct coord net = getNet(ai->st.side);
          double curvePowerToNet = getPowerNeededToAlign(ai, net.x, net.y);

          if (curvePowerToNet == 0 || checkEventActive(ai, EVENT_alignedToScore *2 + 1)){
            // We are probably lined up to score
            
            if (checkEventActive(ai, EVENT_ballCagedAndCanShoot * 2 + 1)){
              // the ball is definitely ready to be shot, and we already checked that it'll probably make the shot

              printf("TAKE THE SHOT %d %d!!\n", curvePowerToNet == 0, checkEventActive(ai, EVENT_alignedToScore *2 + 1));
              fflush(stdout);
              motor_power_async(MOTOR_DRIVE_LEFT, 0);
              motor_power_async(MOTOR_DRIVE_RIGHT, 0);
              takeShot = 1;
              //ballWasSeenInPouch = 0;

            }else{
              printf("Slowing down to allow shots!!\n");
              fflush(stdout);
              motor_power_async(MOTOR_DRIVE_LEFT, 10);
              motor_power_async(MOTOR_DRIVE_RIGHT, 10);
              takeShot = 0;
              driftingInPouch++;

              if (driftingInPouch >= 100){
                //ballWasSeenInPouch = 0;
                //driftingInPouch = 0;
                changeMachineState(ai, STATE_S_think); // figure out whats going on
              }
            }

          }else{
            // we're probably not well alligned for a shot 
            double powerToApply = 10 * fabs(curvePowerToNet) / curvePowerToNet;
            printf("Orienting shot with %f!\n", powerToApply);
            motor_power_async(MOTOR_DRIVE_LEFT, powerToApply);
            motor_power_async(MOTOR_DRIVE_RIGHT, -powerToApply);
            wrong_path_beleif = -3;
            driftingInPouch = 0;
          }
        }
        
        else if (state == STATE_S_creepSlowlyToBall){
          if (robustBallCx != -1000){
            struct coord location = calc_in_front_of_ball(ai, 0, getNet(ai->st.side));
            wanted_posX = location.x;
            wanted_posY = location.y;
          }

          if (wanted_posX != -1 && wanted_posY != -1){
              motor_power_async(MOTOR_DRIVE_LEFT, 12);
              motor_power_async(MOTOR_DRIVE_RIGHT, 12);
          }
          
        }
        
        
        /*else if (state == STATE_S_runAtBallAndShoot){
          wanted_posX = robustBallCx;
          wanted_posY = robustBallCy;

          struct coord net = getNet(ai->st.side);
          double curvePowerToNet = getPowerNeededToAlign(ai, net.x, net.y);
          double curvePower = getPowerNeededToAlign(ai, wanted_posX, wanted_posY);

          double ourDistToBall = pow(pow(robustSelfCx - robustBallCx, 2) + pow(robustSelfCy - robustBallCy, 2), 0.5);
          printf("DISTANCE TO BALL CHECK %f\n", ourDistToBall);

          || ballWasSeenInPouch && ourDistToBall < 120){
            // The ball is likely inside of our net
            ballWasSeenInPouch = 1;

            
            
          }else{
            ballWasSeenInPouch = 0;
            driftingInPouch = 0;
            // No shots to be made, just get closer to the ball
            
          }
          */
      
    }
} 

int get_curr_motor_power(int port_id) {
  return motor_powers[port_id];
}

int motor_power_async(char port_id, char power) {
  if (get_curr_motor_power(port_id) == power) {
    return 0; // no need
  }
  motor_powers[port_id] = power;
  if (power == 0) {
    return BT_motor_port_stop(port_id, 1);
  }
  
  return BT_motor_port_start(port_id, power);
}

struct coord add_coords(struct coord a, struct coord b) {
  return (struct coord){a.x + b.x, a.y + b.y};
}

struct coord scale_coords(struct coord a, double b) {
  return (struct coord){a.x * b, a.y * b};
}

struct coord normalize_vector(struct coord v) {
  double m = pow(pow(v.x, 2) + pow(v.y, 2), 0.5);
  struct coord r;
  if (m == 0){
    r.x = 0;
    r.y = 0;
  }else{
    r.x = v.x / m;
    r.y = v.y / m;
  }
  //printf("Normalized %f, %f to %f, %f\n", v.x, v.y, r.x, r.y);
  //fflush(stdout);
  return r;
}

struct coord getNet(int side) {
  return (struct coord){(1 - side) * sx, sy / 2};
}

struct coord new_coords(double x, double y) {
  return (struct coord){x, y};
}

struct coord calc_in_front_of_ball(struct RoboAI *ai, double distanceOffset, struct coord net) {
  // Calc vector from ball to net nx ny
  //struct coord net = getNet(ai->st.side);
  struct coord netvec;
  netvec.x = net.x - robustBallCx;
  netvec.y = net.y - robustBallCy;

  struct coord res = add_coords(new_coords(robustBallCx, robustBallCy), scale_coords(normalize_vector(netvec), distanceOffset));
  ai->DPhead = addPoint(ai->DPhead, res.x, res.y, 0, 255, 0);
  //printf("Calculated ball offset from %f,%f to be %f,%f\n", ai->st.ball->cx, ai->st.ball->cy, res.x, res.y);
  //fflush(stdout);
  return res;
}


double getExpectedUnitCircleDistance(double angleOffset){ // MAX half rotation (ie pi)
    return pow(pow(1 - cos(angleOffset), 2) + pow(0 - sin(angleOffset), 2), 0.5);
}
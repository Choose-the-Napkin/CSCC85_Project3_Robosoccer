/////////////////////////////////////////////////////////////////////////
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
//////////////////////////////////////////////////////////////////////////

#include"imageProc.h"

//////////////////////////////////////////////////////////////////////////
// Filter kernels and simple filtering
//////////////////////////////////////////////////////////////////////////
struct kernel *GaussKernel(double sigma)
{
 // Compute and return a Gaussian kernel with specified sigma.
 // The length of the kernel is the closest integer to
 // 6*sigma, so there's three sigma at each side from center.
 // The Kernel is normalized to unit mass before being returned.

 int ksize,khsize,i;
 double vari,mass,val;
 struct kernel *k;

 ksize=(int)round(6*sigma);
 if (ksize%2==0) ksize++;
 khsize=(ksize-1)/2;
 vari=sigma*sigma;

 // Allocate data for this kernel
 k=(struct kernel *)calloc(1,sizeof(struct kernel));
 if (!k) return(NULL);
 k->size=ksize;
 k->halfsize=khsize;
 k->taps=(double *)calloc(ksize,sizeof(double));
 if (!k->taps){free(k); return(NULL);}

 // Compute kernel entries
 *(k->taps+khsize)=1.0;			// Center tap
 for (i=1; i<=khsize; i++)
 {
  val=exp(-(i*i)/(2*vari));
  *(k->taps+khsize+i)=val;
  *(k->taps+khsize-i)=val;
 }
 // Normalize kernel to unit mass
 mass=0;
 for (i=0; i<ksize; i++)
  mass+=*(k->taps+i);
 mass=1.0/mass;
 for (i=0; i<ksize; i++)
  (*(k->taps+i))*=mass;

 return(k);
}

struct kernel *doGKernel(double sigma)
{
 // Create and return a Gaussian derivative kernel with 
 // specified sigma. 
 struct kernel *k1;
 int i;

 k1=GaussKernel(sigma);
 for (i=-k1->halfsize;i<=k1->halfsize;i++)
  *(k1->taps+k1->halfsize+i) *= -(i/(sigma*sigma));
 return(k1);
} 

void deleteKernel(struct kernel *k)
{
 // Release memory used by a filter kernel
 if (!k) return;
 if (k->taps!=NULL) free(k->taps);
 free(k);
 return;
}

struct image *convolve_x(struct image *im, struct kernel *k)
{
 // Convolve (actually, correlate, but since the kernel is symmetric...) the
 // input image with the specified kernel along x. Boundary data is handled by
 // replicating the boundary values.
 // For multi-layer images, convolution is applied on each layer.

 double ksum,bnd;
 int i,j,l,ly;
 struct image *tmp;

 tmp=newImage(im->sx,im->sy,im->nlayers);
 if (!tmp){fprintf(stderr,"convolve_x(): Can not allocate memory for image data\n"); return(NULL);}

 for (ly=0; ly<im->nlayers; ly++)
#pragma omp parallel for schedule(dynamic,32) private(i,j,l,ksum,bnd)
  for (j=0; j<im->sy; j++)
  {
   for (i=k->halfsize; i < (im->sx)-(k->halfsize); i++)	// Away from boundaries
   {
    ksum=0;
    for (l=-k->halfsize;l<=k->halfsize;l++)
     ksum+=(*(im->layers[ly]+i+l+(j*im->sx)))*(*(k->taps+k->halfsize+l));
    *(tmp->layers[ly]+i+(j*im->sx))=ksum;
   }
   for (i=0;i<k->halfsize;i++)				// Left boundary
   {
    ksum=0;
    bnd=*(im->layers[ly]+(j*im->sx));
    for (l=-k->halfsize;l<=k->halfsize;l++)
     if (i+l>=0)
      ksum+=(*(im->layers[ly]+i+l+(j*im->sx)))*(*(k->taps+k->halfsize+l));
     else
      ksum+=bnd*(*(k->taps+k->halfsize+l));
    *(tmp->layers[ly]+i+(j*im->sx))=ksum;
   }
   for (i=(im->sx)-(k->halfsize);i<im->sx;i++)		// Right boundary
   {
    ksum=0;
    bnd=*(im->layers[ly]+im->sx-1+(j*im->sx));
    for (l=-k->halfsize;l<=k->halfsize;l++)
     if (i+l<im->sx)
      ksum+=(*(im->layers[ly]+i+l+(j*im->sx)))*(*(k->taps+k->halfsize+l));
     else
      ksum+=bnd*(*(k->taps+k->halfsize+l));
    *(tmp->layers[ly]+i+(j*im->sx))=ksum;
   }
  }

 return(tmp);
}

struct image *convolve_y(struct image *im, struct kernel *k)
{
 // Convolve (actually, correlate, but since the kernel is symmetric...) the
 // input image with the specified kernel along the y direction.
 // Like convolve_x(), this applies the convolution operation to each layer
 // of multi-layer images.
 double ksum,bnd;
 int i,j,l,ly;
 struct image *tmp;

 tmp=newImage(im->sx,im->sy,im->nlayers);
 if (!tmp){fprintf(stderr,"convolve_y(): Can not allocate memory for image data\n"); return(NULL);}

 for (ly=0; ly<im->nlayers; ly++)
#pragma omp parallel for schedule(dynamic,32) private(i,j,l,ksum,bnd)
  for (i=0; i<im->sx; i++)
  {
   for (j=k->halfsize; j<(im->sy)-(k->halfsize); j++)	// Away from boundaries
   {
    ksum=0;
    for (l=-k->halfsize;l<=k->halfsize;l++)
     ksum+=(*(im->layers[ly]+i+((j+l)*im->sx)))*(*(k->taps+k->halfsize+l));
    *(tmp->layers[ly]+i+(j*im->sx))=ksum;
   }
   for (j=0;j<k->halfsize;j++)				// Top boundary
   {
    ksum=0;
    bnd=*(im->layers[ly]+i);
    for (l=-k->halfsize;l<=k->halfsize;l++)
     if (j+l>=0)
      ksum+=(*(im->layers[ly]+i+((j+l)*im->sx)))*(*(k->taps+k->halfsize+l));
     else
      ksum+=bnd*(*(k->taps+k->halfsize+l));
    *(tmp->layers[ly]+i+(j*im->sx))=ksum;
   }
   for (j=(im->sy)-(k->halfsize);j<im->sy;j++)		// Bottom boundary
   {
    ksum=0;
    bnd=*(im->layers[ly]+i+((im->sy-1)*im->sx));
    for (l=-k->halfsize;l<=k->halfsize;l++)
     if (j+l<im->sy)
      ksum+=(*(im->layers[ly]+i+((j+l)*im->sx)))*(*(k->taps+k->halfsize+l));
     else
      ksum+=bnd*(*(k->taps+k->halfsize+l));
    *(tmp->layers[ly]+i+(j*im->sx))=ksum;
   }
  }

 return(tmp);
}

//////////////////////////////////////////////////////////////////////////
// Image feature computations
//////////////////////////////////////////////////////////////////////////
struct image *gradient(struct image *im, double sigma1)
{
 // Computes the image gradient. The Ix and Iy components are left on the
 // first and second layer. These are normalized so that each [ix,iy] pair
 // forms a unit vector in the gradient direction.
 // The third layer contains the gradient magnitude.
 
 struct kernel *k1, *k2;
 struct image *grad;
 struct image *ix;
 struct image *iy;
 struct image *t1;
 struct image *mag;
 int i,j;

 if (im->nlayers>1)
 {
  fprintf(stderr,"gradient(): The function expects a single-layer image. See desaturate()\n");
  return(NULL);
 }
 k1=doGKernel(sigma1);
 k2=GaussKernel(sigma1);
 mag=newImage(im->sx,im->sy,1);
 grad=newImage(im->sx,im->sy,3);
 ix=convolve_x(im,k1);
 iy=convolve_y(im,k1);
 if (!k1 || !ix || !iy || !mag || !grad)
 {
  fprintf(stderr,"gradient(): Out of memory!\n");
  return(NULL);
 }
 // Smooth derivatives
 t1=convolve_x(ix,k2);
 deleteImage(ix);
 ix=convolve_y(t1,k2);
 deleteImage(t1);
 t1=convolve_x(iy,k2);
 deleteImage(iy);
 iy=convolve_y(t1,k2);
 deleteImage(t1);

 // Gradient magnitude 
#pragma omp parallel for schedule(dynamic,32) private(i,j)
 for (j=0;j<im->sy;j++)
  for (i=0;i<im->sx;i++)
  {
   *(mag->layers[0]+i+(j*im->sx))=(*(ix->layers[0]+i+(j*ix->sx))*\
                                   *(ix->layers[0]+i+(j*ix->sx)));
   *(mag->layers[0]+i+(j*im->sx))+=(*(iy->layers[0]+i+(j*iy->sx))*\
                                    *(iy->layers[0]+i+(j*iy->sx)));
  }
 pointwise_pow(mag,.5);

 // Store unit Ix and Iy, plus magnitude, in the gradient map
#pragma omp parallel for schedule(dynamic,32) private(i,j)
 for (j=0;j<im->sy;j++)
  for (i=0;i<im->sx;i++)
  {
   *(grad->layers[0]+i+(j*grad->sx))=(*(ix->layers[0]+i+(j*ix->sx)))/(*(mag->layers[0]+i+(j*mag->sx)));
   *(grad->layers[1]+i+(j*grad->sx))=(*(iy->layers[0]+i+(j*iy->sx)))/(*(mag->layers[0]+i+(j*mag->sx)));
   *(grad->layers[2]+i+(j*grad->sx))=pow(*(mag->layers[0]+i+(j*mag->sx)),.5);
  }

 deleteImage(ix);
 deleteImage(iy);
 deleteImage(mag); 
 deleteKernel(k1);
 deleteKernel(k2);
 return(grad);
}

void nonMaxSuppression(struct image *grad)
{
 // Perform non-max suppresion on the gradient map
 int i,j;
 double vx,vy,th,the;
 int px,py,l;
 double mag1,mag2,mag3;

 for (j=0;j<grad->sy;j++)
  for (i=0;i<grad->sx;i++)
  {
   vx=*(grad->layers[0]+i+(j*grad->sx));
   vy=*(grad->layers[1]+i+(j*grad->sx));
   mag1=*(grad->layers[2]+i+(j*grad->sx));
   if (mag1>0)
   {
    th=atan2(vy,vx);
    for (the=th-.087;the<=th+.087;the+=.0174)
     for (l=-3;l<=3;l++)
     {
      vx=cos(the);
      vy=sin(the);
      px=(int)((double)i+((double)l*vx));
      py=(int)((double)j+((double)l*vy));
      if (px>=0&&px<grad->sx&&py>=0&&py<grad->sy)
      {
       mag2=*(grad->layers[2]+(int)px+(((int)py)*grad->sx));
       if (mag2>mag1)
       { 
        *(grad->layers[0]+i+(j*grad->sx))=0;
        *(grad->layers[1]+i+(j*grad->sx))=0;
        *(grad->layers[2]+i+(j*grad->sx))=0;
       }
      }
     }
   }
  } 
}

void thresholdGradient(struct image *grad, double thresh)
{
 // Gradient thresholding. Zeroes out gradient direction vectors
 // and gradient magnitude wherever the gradient is under the
 // specified threshold (in [0,1])
 int i,j;
#pragma omp parallel for schedule(dynamic,32) private(i,j)
 for (j=0;j<grad->sy;j++)
  for (i=0;i<grad->sx;i++)
   if (*(grad->layers[2]+i+(j*grad->sx))<thresh)
   {
    *(grad->layers[0]+i+(j*grad->sx))=0;
    *(grad->layers[1]+i+(j*grad->sx))=0;
    *(grad->layers[2]+i+(j*grad->sx))=0;
   }
}

struct image *contrast(struct image *im, double alpha)
{
 // Contrast feature - basically a low-level edge structure detector.
 // The original Exposure Fusion implementation uses a discrete
 // Laplace operator. I haven't (yet) implemented 2D kernels and
 // 2D convolution, so we will approximate the edge filter with
 // a Difference of Gaussians.
 //
 // The map is ormalized to [0,1], then raised to the power of alpha
 // elementwise as per the exposure fusion method.

 struct kernel *k1;
 double sig1=1.0;			// Sigma for small kernel
 struct image *t1,*t2,*t3;
 struct image *gray;
 int i,j;

 if (im->nlayers!=3){fprintf(stderr,"contrast(): Expected 3-layer image!\n");return(NULL);}

 gray=newImage(im->sx,im->sy,1);	// Grayscale image
 if (!gray){fprintf(stderr,"contrast(): Can't allocate memory for grayscale image\n");return(NULL);}

#pragma omp parallel for schedule(dynamic,32) private(i,j)
 for (j=0;j<im->sy;j++)
  for (i=0;i<im->sx;i++)
   *(gray->layers[0]+i+(j*im->sx))=(*(im->layers[0]+i+(j*im->sx))+\
                                    *(im->layers[1]+i+(j*im->sx))+\
                                    *(im->layers[2]+i+(j*im->sx)))/3.0;

 k1=GaussKernel(sig1);		// Gaussian kernel

 t2=copyImage(gray);		// Input weight map
 t1=convolve_x(gray,k1);
 t3=convolve_y(t1,k1);		// Result of Gaussian filtering with k1
 deleteImage(t1);
 pointwise_sub(t2,t3);		// Difference of Gaussians now in t2
 deleteImage(t3);

 pointwise_pow(t2,2.0);		// Edge energy!!!
 normalize(t2);			// Normalize to [0,1]
 pointwise_mul(t2,t2);		// Get absolute value - the EF paper specifies
				// gradient energy, but their implementation uses
				// absolute value of Laplacian
 normalize(t2);
 pointwise_pow(t2,.25);		// Scaled to agree with the Laplacian filter
				// implementation in the EF code by Mertens et al.

 deleteImage(gray);
 deleteKernel(k1);
 return(t2);
}

struct image *saturation(struct image *im, double alpha)
{
 // Saturation feature - this is defined in the exposure fusion
 // paper *NOT* as the standard saturation from the HSV colourspace,
 // but instead it is defined as the deviation between each colout
 // channel and the mean (grayscale) value at each pixel.
 //
 // The map is ormalized to [0,1], then raised to the power of alpha
 double mu,R,G,B;
 int i,j;
 struct image *t;

 if (im->nlayers!=3){fprintf(stderr,"saturation(): Expected 3 channel image!\n");return(NULL);}

 t=newImage(im->sx,im->sy,1);
 if (!t){fprintf(stderr,"saturation(): Can't allocate memory for saturation map\n");return(NULL);}

#pragma omp parallel for schedule(dynamic,32) private(i,j,R,G,B,mu)
 for (j=0;j<im->sy;j++)
  for (i=0;i<im->sx;i++)
  {
   R=*(im->layers[0]+i+(j*im->sx));
   G=*(im->layers[1]+i+(j*im->sx));
   B=*(im->layers[2]+i+(j*im->sx));
   mu=(R+G+B)/3.0;
   *(t->layers[0]+i+(j*im->sx))=sqrt( ( ((R-mu)*(R-mu)) + ((G-mu)*(G-mu)) + ((B-mu)*(B-mu)) )/3.0);
  }

 normalize(t);
 pointwise_pow(t,alpha);
 return(t);
}

struct image *exposedness(struct image *im, double alpha)
{
 // Well-exposedness feature - This is measured as deviation from
 // middle gray. The exposure fusion method uses Gaussians wih
 // mean .5 brightness to evauate the well-exposedness of each
 // colour component. Juts like the original exposure fusion
 // Matlab code here we use a sigma of .2
 double vari=.2*.2;
 double R,G,B;
 int i,j;
 struct image *t;

 if (im->nlayers!=3){fprintf(stderr,"exposedness(): Expected 3 channel image!\n");return(NULL);}

 t=newImage(im->sx,im->sy,1);
 if (!t){fprintf(stderr,"exposedness(): Can't allocate memory for saturation map\n");return(NULL);}

#pragma omp parallel for schedule(dynamic,32) private(i,j,R,G,B)
 for (j=0;j<im->sy;j++)
  for (i=0;i<im->sx;i++)
  {
   R=*(im->layers[0]+i+(j*im->sx));
   G=*(im->layers[1]+i+(j*im->sx));
   B=*(im->layers[2]+i+(j*im->sx));
   R=exp(-.5*((R-.5)*(R-.5))/vari);
   G=exp(-.5*((G-.5)*(G-.5))/vari);
   B=exp(-.5*((B-.5)*(B-.5))/vari);
   *(t->layers[0]+i+(j*im->sx))=R*G*B;
  }

 // Notice that we DO NOT normalize this map since for exposure fusion we
 // want to preserve very small weights for completely over or under
 // exposed images. The output will still be in [0,1]

 pointwise_pow(t,alpha);
 return(t);
}

struct image *timediff(struct image *im, struct image *old, double alpha)
{
 // Weight map based on the magnitude of the temporal derivative between two
 // consecutive frames. Can be used to give higher blending importance 
 // to moving structure
 struct kernel *k1;
 double sig1=1.0;		// May want to adjust this
 struct image *t1,*t2,*t3;
 struct image *gray;
 int i,j;

 k1=GaussKernel(sig1);		// Gaussian kernel
 
 t1=convolve_x(im,k1);
 t2=convolve_y(t1,k1);		// Smoothed image at current frame
 deleteImage(t1);

 t1=convolve_x(im,k1);
 t3=convolve_y(t1,k1);		// Smoothed image at previous frame
 deleteImage(t1);
 deleteKernel(k1);
 
 pointwise_sub(t2,t3);		// Difference now in t2
 deleteImage(t3);

 pointwise_pow(t2,2.0);		// Magnitude of the temporal derivative

 // Average derivative magnitude over all colour channels
 gray=newImage(im->sx,im->sy,1);
 if (!gray){fprintf(stderr,"contrast(): Can't allocate memory for temporal derivative map\n");return(NULL);}

#pragma omp parallel for schedule(dynamic,32) private(i,j)
 for (j=0;j<im->sy;j++)
  for (i=0;i<im->sx;i++)
   *(gray->layers[0]+i+(j*gray->sx))=(*(t2->layers[0]+i+(j*t2->sx))+\
                                      *(t2->layers[1]+i+(j*t2->sx))+\
                                      *(t2->layers[2]+i+(j*t2->sx))+.001)/3.0;
 deleteImage(t2);

 *(gray->layers[0])=0;		// Pixel at top-left takes one for the team!
 normalize(gray);		// Range normalization - dangerous?
 pointwise_pow(gray,alpha);	// Apply alpha

 return(gray);
}

struct image *computeWeightMap(struct image *im, struct image *old, double alphaC, double alphaS, double alphaE, double alphaT)
{
 // Computes and returns a weight map for the input image that includes the
 // contrast, sarutation, and well-exposedness terms from the original
 // exposure fusion formulation as well as a new term based on the temporal
 // derivative over consecutive frames.

 struct image *t1,*t2,*t3,*t4;
 int i,j;

 t1=contrast(im,alphaC);
 t2=saturation(im,alphaS);
 t3=exposedness(im,alphaE);
 t4=timediff(im,old,alphaT);
 if (!t1||!t2||!t2||!t4)
 {
  fprintf(stderr,"computeWeightMap(): Error, can not obtain weight maps (out of memory)\n");
  return(NULL);
 }
 // Multiply the 4 cue weight maps to obtain the fused weight map.
 pointwise_mul(t1,t2);
 deleteImage(t2);
 pointwise_mul(t1,t3);
 deleteImage(t3);
 pointwise_mul(t1,t4);
 deleteImage(t4);

 // Add a small, uniform component to avoid collapse and div. by zero!
#pragma omp parallel for schedule(dynamic,32) private(i,j)
 for (j=0;j<t1->sy;j++)
  for (i=0;i<t1->sx;i++)
   *(t1->layers[0]+i+(j*t1->sx))=(*(t1->layers[0]+i+(j*t1->sx)))+.0001;

 return(t1);		// Notice we do not normalize the combined weight map!
}

//////////////////////////////////////////////////////////////////////////
// Image operations
//////////////////////////////////////////////////////////////////////////
void rgb2hsv(double R, double G, double B, double *Hu, double *S, double *V)
{
  // Return the HSV components of the input RGB colour, R,G, and B in [0,1]
  // H in [0, 2*Pi], S and V in [0,1], 
  double C;
  
  if (R>G&&R>B) *V=R; else if (G>B) *V=G; else *V=B;
  if (R<G&&R<B) C=((*V)-R); else if (G<B) C=((*V)-G); else C=((*V)-B); 
  if (C==0) *Hu=0;      // Actually it is undefined, but this should never be allowed to happen
  else if (*V==R) {*Hu=(G-B)/C; if (*Hu<0) *Hu=6.0-(*Hu);}
  else if (*V==G) *Hu=((B-R)/C)+2.0;
  else *Hu=((R-G)/C)+4.0;
  *Hu=(*Hu)*60;
  *Hu=fmod(*Hu,360);
  *Hu=2*PI*(*Hu)/360.0;
  if (*V==0) *S=0;  // Once more, this should not happen
  else *S=C/(*V);
}

void hsv2rgb(double H, double S, double V, double *R, double *G, double *B)
{
 // Return the RGB components of a colour specified in HSV,
 // H in [0, 2*PI], S and V in [0,1]. RGB in [0,1]

 double C,X,m,hex;
    
 H=H*(360.0/(2*PI));

 C=V*S;                 // Chroma component
 hex=fmod(H/60.0,6);    // Hue quantized to [0,6)
 X=C*(1.0-fabs(fmod(hex,2.0)-1.0));
 m=V-C;   
 
 fprintf(stderr,"H=%f, C=%f, X=%f, m=%f, hex=%f\n",H,C,X,m,hex);

 if (hex<0) {*R=0; *G=0; *B=0;} 
 else if (hex>=0&&hex<1.0) {*R=C; *G=X; *B=0;}
 else if (hex<2.0) {*R=X; *G=C; *B=0;}
 else if (hex<3.0) {*R=0; *G=C; *B=X;}
 else if (hex<4.0) {*R=0; *G=X; *B=C;}
 else if (hex<5.0) {*R=X; *G=0; *B=C;}
 else {*R=C; *G=0; *B=X;}
 
 *R=((*R)+m)*255;
 *G=((*G)+m)*255;
 *B=((*B)+m)*255;
   
 if (*R<0) *R=0;
 if (*R>255) *R=255;
 if (*G<0) *G=0;
 if (*G>255) *G=255;
 if (*B<0) *B=0;
 if (*B>255) *B=255; 
}

struct image *newImage(int sx, int sy, int layers)
{
 // Create and return an empty (all zeroes) image with the
 // specified dimensions.
 struct image *im;
 int i;

 if (layers!=1&&layers!=3)
 {
  fprintf(stderr,"newImage(): Image must have 1 or 3 layers\n");
  return(NULL);
 }

 im=(struct image *)calloc(1,sizeof(struct image));
 if (!im) return(NULL);

 for (i=0;i<layers;i++)
 {
  im->layers[i]=(double *)calloc(sx*sy,sizeof(double));
  if (!im->layers[i]){free(im); return(NULL);}
 }

 im->sx=sx;
 im->sy=sy;
 im->nlayers=layers;

 return(im);
}

struct image *imageFromBuffer(unsigned char *buf, int sx, int sy, int nlayers)
{
 // Takes a frame buffer, expected to consist of an (sx *sy * nlayers) array of
 // unsigned char data, and converts it into an image data structure for use with
 // the functions in this library. Note that intensity in the generated image structure
 // will be in [0,1]
 int i,j;
 struct image *im;
 
 im=newImage(sx,sy,nlayers);
 if (!im)
 {
  fprintf(stderr,"imageFromBuffer(): Out of memory!\n");
  return(NULL);
 }
#pragma omp parallel for schedule(dynamic,32) private(i,j)
 for (j=0;j<sy;j++)
  for (i=0;i<sx;i++)
  {
   *(im->layers[0]+(i+(j*sx)))=(double)(*(buf+((i+(j*sx))*3)+0));
   *(im->layers[1]+(i+(j*sx)))=(double)(*(buf+((i+(j*sx))*3)+1));
   *(im->layers[2]+(i+(j*sx)))=(double)(*(buf+((i+(j*sx))*3)+2));
  }
 return(im);
}

unsigned char *bufferFromIm(struct image *im)
{
 // Create a frame buffer from the image data in 'im'.
 // This function assumes the data in the image is
 // in the range [0,1]. Call normalize() if needed
 // before invoking bufferFromIm()

 unsigned char *buf;
 int i,j;

 buf=(unsigned char *)calloc(im->sx*im->sy*im->nlayers,sizeof(unsigned char));
 if (!buf)
 {
  fprintf(stderr,"bufferFromIm(): Out of memory!\n");
  return(NULL);
 }
#pragma omp parallel for schedule(dynamic,32) private(i,j)
 for (j=0;j<im->sy;j++)
  for (i=0;i<im->sx;i++)
  {
   *(buf+((i+(j*im->sx))*im->nlayers)+0)=(unsigned char)((*(im->layers[0]+(i+(j*im->sx)))));
   *(buf+((i+(j*im->sx))*im->nlayers)+1)=(unsigned char)((*(im->layers[1]+(i+(j*im->sx)))));
   *(buf+((i+(j*im->sx))*im->nlayers)+2)=(unsigned char)((*(im->layers[2]+(i+(j*im->sx)))));
  }
 return(buf);
}

struct image *copyImage(struct image *im)
{
 // Make a copy of the input image
 struct image *imR;
 int i;

 imR=(struct image *)calloc(1,sizeof(struct image));
 if (!imR)
  return(NULL);

 imR->sx=im->sx;
 imR->sy=im->sy;
 imR->nlayers=im->nlayers;
 for (i=0; i<im->nlayers; i++)
 {
  imR->layers[i]=(double *)calloc(im->sx*im->sy,sizeof(double));
  if (!imR->layers[i]){free(im);return(NULL);}
  memcpy(imR->layers[i],im->layers[i],im->sx*im->sy*sizeof(double));
 }

 return(imR);
}

void deleteImage(struct image *im)
{
 // Free the memory allocated to an image's data structure
 int i;

 if (!im) return;

 for (i=0; i<im->nlayers; i++)	// Release layer data
  if (im->layers[i]!=NULL) free(im->layers[i]);

 free(im);
 return;
}

struct image *desaturate(struct image *im)
{
 // Create a grayscale version of the input image.
 // If the input is already a single layer image, returns NULL
 struct image *imD;
 int i,j;
 
 if (im->nlayers<3) return(NULL);
 imD=newImage(im->sx,im->sy,1);
#pragma omp parallel for schedule(dynamic,32) private(i,j)
 for (j=0;j<im->sy;j++)
  for (i=0;i<im->sx;i++)
   *(imD->layers[0]+i+(j*imD->sx))=(.289*(*(im->layers[0]+i+(j*im->sx))))+\
				   (.588*(*(im->layers[1]+i+(j*im->sx))))+\
				   (.125*(*(im->layers[2]+i+(j*im->sx))));

 return(imD);
}
 
double imMax(struct image *im)
{
 // Returns the max value over an image
 double max=-1.0;
 int i,j,ly;

 for (ly=0;ly<im->nlayers;ly++)
  for (j=0;j<im->sy;j++)
   for (i=0l;i<im->sx;i++)
    if (max<*(im->layers[ly]+i+(j*im->sx))) max=*(im->layers[ly]+i+(j*im->sx));

 return(max);
}

double imMin(struct image *im)
{
 // Returns the max value over an image
 double min=1e6;
 int i,j,ly;

 for (ly=0;ly<im->nlayers;ly++)
  for (j=0;j<im->sy;j++)
   for (i=0l;i<im->sx;i++)
    if (min>*(im->layers[ly]+i+(j*im->sx))) min=*(im->layers[ly]+i+(j*im->sx));

 return(min);
}

void pointwise_add(struct image *im1, struct image *im2)
{
 // Element-wise addition of two images:
 //
 // im1=im1+im2
 //
 // Checks that the dimensions are identical.
 int i,j,ly;
 if (im1->sx!=im2->sx || im1->sy!=im2->sy || im1->nlayers!=im2->nlayers)
 {
  fprintf(stderr,"pointwise_add(): Images have different sizes!\n");
  return;
 }

 for (ly=0; ly<im1->nlayers; ly++)
#pragma omp parallel for schedule(dynamic,32) private(i,j)
  for (j=0;j<im1->sy;j++)
   for (i=0;i<im1->sx;i++)
    *(im1->layers[ly]+i+(j*im1->sx))=(*(im1->layers[ly]+i+(j*im1->sx)))+(*(im2->layers[ly]+i+(j*im1->sx)));
}

void pointwise_sub(struct image *im1, struct image *im2)
{
 // Element-wise subtraction of two images:
 //
 // im1=im1-im2
 //
 // Checks that the dimensions are identical.
 int i,j,ly;
 if (im1->sx!=im2->sx || im1->sy!=im2->sy || im1->nlayers!=im2->nlayers)
 {
  fprintf(stderr,"pointwise_add(): Images have different sizes!\n");
  return;
 }

 for (ly=0; ly<im1->nlayers; ly++)
#pragma omp parallel for schedule(dynamic,32) private(i,j)
  for (j=0;j<im1->sy;j++)
   for (i=0;i<im1->sx;i++)
    *(im1->layers[ly]+i+(j*im1->sx))=(*(im1->layers[ly]+i+(j*im1->sx)))-(*(im2->layers[ly]+i+(j*im1->sx)));
}

void pointwise_pow(struct image *im1, double p)
{
 // Element-wise power of input image:
 //
 // im1=im1.^p
 int i,j,ly;

 for (ly=0;ly<im1->nlayers;ly++)
#pragma omp parallel for schedule(dynamic,32) private(i,j)
  for (j=0;j<im1->sy;j++)
   for (i=0;i<im1->sx;i++)
    *(im1->layers[ly]+i+(j*im1->sx))=pow(*(im1->layers[ly]+i+(j*im1->sx)),p);
}

void pointwise_mul(struct image *im1, struct image *im2)
{
 // Element-wise multiplication of two images:
 //
 // im1=im1.*im2
 //
 // Checks that the dimensions are identical.
 int i,j,ly;
 if (im1->sx!=im2->sx || im1->sy!=im2->sy || im1->nlayers!=im2->nlayers)
 {
  fprintf(stderr,"pointwise_mul(): Images have different sizes!\n");
  return;
 }

 for (ly=0; ly<im1->nlayers; ly++)
#pragma omp parallel for schedule(dynamic,32) private(i,j)
  for (j=0;j<im1->sy;j++)
   for (i=0;i<im1->sx;i++)
    *(im1->layers[ly]+i+(j*im1->sx))=(*(im1->layers[ly]+i+(j*im1->sx)))*(*(im2->layers[ly]+i+(j*im1->sx)));
}

void pointwise_div(struct image *im1, struct image *im2)
{
 // Element-wise division of two images:
 //
 // im1=im1./im2
 //
 // Checks that the dimensions are identical.
 int i,j,ly;
 if (im1->sx!=im2->sx || im1->sy!=im2->sy || im1->nlayers!=im2->nlayers)
 {
  fprintf(stderr,"pointwise_div(): Images have different sizes!\n");
  return;
 }

 for (ly=0; ly<im1->nlayers; ly++)
#pragma omp parallel for schedule(dynamic,32) private(i,j)
  for (j=0;j<im1->sy;j++)
   for (i=0;i<im1->sx;i++)
    *(im1->layers[ly]+i+(j*im1->sx))=(*(im1->layers[ly]+i+(j*im1->sx)))/(*(im2->layers[ly]+i+(j*im1->sx)));
}

void image_scale(struct image *im, double k)
{
 // Scalar multiply an image by k
 int i,j,ly;

 for (ly=0; ly<im->nlayers; ly++)
#pragma omp parallel for schedule(dynamic,32) private(i,j)
  for (j=0;j<im->sy;j++)
   for (i=0;i<im->sx;i++)
    *(im->layers[ly]+i+(j*im->sx))=(*(im->layers[ly]+i+(j*im->sx)))*k;
}

void normalize(struct image *im)
{
 // Normalizes an image to be in the range [0,1]
 // note that for multi-layer images this uses the max over all layers
 // for normalization.
 double min=1e15;
 double max=-1e15;
 int i,j,ly;

 for (ly=0; ly<im->nlayers; ly++)
#pragma omp parallel for schedule(dynamic,32) private(i,j) shared(min,max)
  for (j=0;j<im->sy;j++)
   for(i=0; i<im->sx; i++)
   {
    if (min>*(im->layers[ly]+i+(j*im->sx))) min=*(im->layers[ly]+i+(j*im->sx));
    if (max<*(im->layers[ly]+i+(j*im->sx))) max=*(im->layers[ly]+i+(j*im->sx));
   }

 // Avoid numerical insanity
 min=min-1e-6;
 max=max+1e-6;

 for (ly=0; ly<im->nlayers; ly++)
#pragma omp parallel for schedule(dynamic,1) private(i,j)
  for (j=0;j<im->sy;j++)
   for(i=0; i<im->sx; i++)
   *(im->layers[ly]+i+(j*im->sx))=((*(im->layers[ly]+i+(j*im->sx)))-min)/(max-min);
}

struct image *resize(struct image *im, int sx, int sy)
{
 // Resize an image to the desired size (sx,sy) using Bilinear interpolation

 double step_x,step_y;			// Step increase as per instructions above
 double R1,R2,R3,R4;			// Colours at the four neighbours
 double RT1;				// Interpolated colours at T1 and T2
 double RT2;
 double R;				// Final colour at a destination pixel
 struct image *dst;			// Destination image - allocated here!
 int x,y;				// Coordinates on destination image
 double fx,fy;				// Corresponding coordinates on source image
 double dx,dy;				// Fractional component of source image coordinates
 double ldx,ldy;
 int flx,fly;
 int ly;

 dst=newImage(sx,sy,im->nlayers);
 if (!dst){fprintf(stderr,"resize(): Unable to allocate memory for image\n");return(NULL);}

 // Step sizes for interpolation - Make sure we never exceed input image bounds!
 step_x=(double)((im->sx)-1)/(double)(sx-1);
 step_y=(double)((im->sy)-1)/(double)(sy-1);
 step_x*=.9999;
 step_y*=.9999;

 for (ly=0;ly<im->nlayers;ly++)
 {
#pragma omp parallel for schedule(dynamic,32) private(y,x,fx,fly,dy,ldy,flx,dx,ldx,R1,R2,R3,R4,RT1,RT2,R,fy)
  for (y=0;y<sy;y++)
  {
   fy=(y*step_y);
   fx=0;
   fly=(int)fy;
   dy=fy-fly;
   ldy=1.0-dy;
   for (x=0;x<sx-1;x++)			// Loop over destination image
   {
    flx=(int)fx;
    dx=fx-flx;
    ldx=1.0-dx;

    // Get colours for four neighbours.
    R1=*(im->layers[ly]+flx+(fly*im->sx));
    R2=*(im->layers[ly]+flx+1+(fly*im->sx));
    R3=*(im->layers[ly]+flx+((fly+1)*im->sx));
    R4=*(im->layers[ly]+flx+1+((fly+1)*im->sx));

    // Interpolate to get T1 and T2 colours
    RT1=(dx*R2)+(ldx*R1);
    RT2=(dx*R4)+(ldx*R3);

    // Obtain final colour by interpolating between T1 and T2
    R=((dy*RT2)+(ldy*RT1));

    // Store final colour for this pixel
    *(dst->layers[ly]+x+(y*sx))=R;
    fx+=step_x;
   }
   *(dst->layers[ly]+sx-1+(y*sx))=(ldy*R2)+(dy*R4);	// Last scanline pixel
  }
 }	// End for ly

 return(dst);
}

//////////////////////////////////////////////////////////////////////////
// Image pyramid operations
//////////////////////////////////////////////////////////////////////////
struct pyramid *LaplacianPyr(struct image *im, int levels)
{
 // Builds a Laplacian pyramid with the specified number of levels.
 // In practice, instead of LoG filters we use DoG filtering to
 // approximate the Laplacian at each scale.
 // Each level in the pyramid will have the same number of layers
 // as the input image.

 struct kernel *k;	// Gaussian Kernel for LoG filters
 double sig=1.0;
 int lv;
 struct pyramid *pyr;
 struct image *t1, *t2, *t3;

 pyr=(struct pyramid *)calloc(1,sizeof(struct pyramid));
 if (!pyr){fprintf(stderr,"LaplacianPyr(): Unable to allocate pyramid data structure\n");return(NULL);}

 // Allocate pointers for the image data structures at each level.
 pyr->images=(struct image **)calloc(levels,sizeof(struct image *));

 // Create kernel
 k=GaussKernel(sig);

 if (!k || !pyr->images){fprintf(stderr,"LaplacianPyr(): Can not allocate pyramid data or filter kernel\n");free(pyr);return(NULL);}

 // Make a copy of the input image (we don't want to destroy it!)
 t1=copyImage(im);

 // Perform DoG filtering, and resampling
 for(lv=0;lv<levels-1;lv++)
 {
  t2=convolve_x(t1,k);			// Gaussian filter current scale image
  t3=convolve_y(t2,k);			// Gaussian blurred image is in t3
  deleteImage(t2);
  pointwise_sub(t1,t3);		// DoG at this scale is now in t1
  *(pyr->images+lv)=t1;		// Store the DoG at this level of the pyramid
  t1=resize(t3,floor(t3->sx/2),floor(t3->sy/2));	// For the next level, downsample blurred image by 2
  deleteImage(t3);
  if (t1->sx<=10 || t1->sy<=10){lv++;break;}	// Break if image has become too tiny!
 }
 // Last level is simply the residual from all the filtering above
 *(pyr->images+lv)=t1;
 pyr->levels=++lv;				// Number of levels requires +1!

 // Done!
 deleteKernel(k);
 return(pyr);
}

struct pyramid *GaussianPyr(struct image *im, int levels)
{
 // Builds a Gaussian pyramid with the specified number of levels.
 // Each level in the pyramid will have the same number of layers
 // as the input image.

 struct kernel *k;
 double sig=1;
 int lv;
 struct pyramid *pyr;
 struct image *t1, *t2, *t3;

 pyr=(struct pyramid *)calloc(1,sizeof(struct pyramid));
 if (!pyr){fprintf(stderr,"GaussianPyr(): Unable to allocate pyramid data structure\n");return(NULL);}

 // Allocate pointers for the image data structures at each level.
 pyr->images=(struct image **)calloc(levels,sizeof(struct image *));

 // Create kernel
 k=GaussKernel(sig);

 if (!k || !pyr->images){fprintf(stderr,"GaussianPyr(): Can not allocate pyramid data or filter kernel\n");free(pyr);return(NULL);}

 // Make a copy of the input image (we don't want to destroy it!)
 t1=copyImage(im);

 // Perform filtering and resampling
 for(lv=0;lv<levels;lv++)
 {
  *(pyr->images+lv)=t1;			// Store image at this level
  t2=convolve_x(t1,k);			// Gaussian filter current scale image
  t3=convolve_y(t2,k);			// Gaussian blurred image is in t3
  deleteImage(t2);
  t1=resize(t3,floor(t3->sx/2),floor(t3->sy/2));	// For the next level, downsample blurred image by 2
  deleteImage(t3);
  if (t1->sx<=10 || t1->sy<=10){lv++;break;}		// Break if image has become too tiny!
 }
 deleteImage(t1);
 pyr->levels=lv;

 // Done!
 deleteKernel(k);
 return(pyr);
}

struct pyramid *weightedPyr(struct pyramid *lPyr, struct pyramid *gPyr)
{
 // Compute and return the result of applying the weights in 'gPyr' on
 // the Laplacian pyramid stored in lPyr, in effect returning a
 // weighed Laplacian pyramid. This function expects lPyr to be a 3
 // layer image, and gPyr a 1 layer image.
 int lv,sx,sy,i;
 struct pyramid *pyr;
 struct image *wght;

 if ((*(lPyr->images))->nlayers!=3||(*(gPyr->images))->nlayers!=1)
 {
  fprintf(stderr,"weightedPyr(): Error, expected 3-layer lPyr and 1-layer gPyr\n");
  return(NULL);
 }
 if (lPyr->levels!=gPyr->levels)
 {
  fprintf(stderr,"weightedPyr(): Error, pyramids must have the same number of levels\n");
  return(NULL);
 }
 pyr=(struct pyramid *)calloc(1,sizeof(struct pyramid));
 if (!pyr){fprintf(stderr,"weightedPyr(): Unable to allocated pyramid data structure!\n");return(NULL);}
 pyr->images=(struct image **)calloc(lPyr->levels,sizeof(struct image));
 if (!pyr->images){fprintf(stderr,"weightedPyr(): Unable to allocated pyramid data structure!\n");return(NULL);}

 for (i=0; i<lPyr->levels; i++)
 {
  *(pyr->images+i)=copyImage(*(lPyr->images+i));
  sx=(*(pyr->images+i))->sx;
  sy=(*(pyr->images+i))->sy;
  // Create a 3 layered weight image for pointwise_mul. (equivalent to doing
  // repmat() on Matlab ).
  wght=newImage(sx,sy,3);
  memcpy(wght->layers[0],(*(gPyr->images+i))->layers[0],sx*sy*sizeof(double));
  memcpy(wght->layers[1],(*(gPyr->images+i))->layers[0],sx*sy*sizeof(double));
  memcpy(wght->layers[2],(*(gPyr->images+i))->layers[0],sx*sy*sizeof(double));
  // Pointwise multiply
  pointwise_mul(*(pyr->images+i),wght);
  deleteImage(wght);
 }
 pyr->levels=lPyr->levels;
 return(pyr);
}

struct image *collapsePyr(struct pyramid *pyr)
{
 // Reconstructs an image by collapsing a Laplacian pyramid.
 int lv;
 struct image *t1, *t2, *t3;

 // lowest scale level of the pyramid is just a residual. This is
 // at pyr->levels - 1
 t1=copyImage(*(pyr->images+pyr->levels-1));

 for (lv=pyr->levels-2;lv>=0; lv--)
 {
  // No pyramid data is hurt in the execution of this loop!
  t2=resize(t1,(*(pyr->images+lv))->sx,(*(pyr->images+lv))->sy);	// Scale up!
  deleteImage(t1);							// Delete smaller image
  pointwise_add(t2,*(pyr->images+lv));					// Add detail at this level
  t1=t2;
 }

 // Final image is now in t1
 return(t1);
}

void deletePyramid(struct pyramid *pyr)
{
 // De-allocate memory occupied by an image pyramid
 int i;
 if (!pyr) return;
 for (i=0; i<pyr->levels; i++)
  deleteImage(*(pyr->images+i));
 free(pyr->images);
 free(pyr);
}

//////////////////////////////////////////////////////////////////////////
// Image I/O functions
//////////////////////////////////////////////////////////////////////////
struct image *readPPM(const char *name)
{
 // Reads an image from a .ppm file. A .ppm file is a very simple image representation
 // format with a text header followed by the binary RGB data at 24bits per pixel.
 // The header has the following form:
 //
 // P6
 // # One or more comment lines preceded by '#'
 // 340 200
 // 255
 //
 // The first line 'P6' is the .ppm format identifier, this is followed by one or more
 // lines with comments, typically used to inidicate which program generated the
 // .ppm file.
 //
 // After the comments, a line with two integer values specifies the image resolution
 // as number of pixels in x and number of pixels in y.
 //
 // The final line of the header stores the maximum value for pixels in the image,
 // usually 255 but does not actually matter for this code as long as it's present.
 //
 // After this last header line, binary data stores the RGB values for each pixel
 // in row-major order at 24bpp.
 //
 // readPPMdata converts the image colour information to floating point. And stores
 // it as a layered image structure. All data will be in the range [0,1]
 //

 FILE *f;
 struct image *im;
 char line[1024];
 int sizx,sizy,sx,sy;
 int i,j,ly;
 unsigned char *tmp;

 f=fopen(name,"r");
 if (!f){fprintf(stderr,"readPPM(): Unable to open specified image file %s %d\n",name,errno);free(im);return(NULL);}

 fgets(&line[0],1000,f);
 if (strcmp(&line[0],"P6\n")!=0)
 {
  fprintf(stderr,"readPPM(): Wrong file format, not a .ppm file or header data missing\n");
  free(im);
  fclose(f);
  return(NULL);
 }
 // Skip over comments
 fgets(&line[0],511,f);
 while (line[0]=='#')
  fgets(&line[0],511,f);
 sscanf(&line[0],"%d %d\n",&sizx,&sizy);           // Read file size

 im=newImage(sizx,sizy,3);
 tmp=(unsigned char *)calloc(sizx*sizy*3,sizeof(unsigned char));

 if (!im||!tmp)
 {
  fprintf(stderr,"readPPM(): Unable to allocate memory for image layers!\n");
  free(im);
  fclose(f);
  return(NULL);
 }
 fgets(&line[0],9,f);                          		// Read the remaining header line
 fread(tmp,sizx*sizy*3*sizeof(unsigned char),1,f);	// Read image data
 fclose(f);

 // Convert to 3-layer floating point
 for (ly=0;ly<3;ly++)
  for (j=0;j<im->sy;j++)
   for (i=0;i<im->sx;i++)
   {
    *(im->layers[ly]+i+(j*im->sx))=(double)(*(tmp+((i+(j*sizx))*3)+ly));
    (*(im->layers[ly]+i+(j*im->sx)))*.003921569;
   }

 free(tmp);
 return(im);
}

int writePPM(const char *name, struct image *im)
{
 // Writes out a .ppm file from the image in im. The image is assumed to be
 // in the range [0 255] - ensure this is the case!
 unsigned char *tmp;
 struct image *image;
 int i,j,ly;
 FILE *f;

 if (im->nlayers!=3)
 {
  image=newImage(im->sx,im->sy,3);
  memcpy(image->layers[0],im->layers[0],im->sx*im->sy*sizeof(double));
  memcpy(image->layers[1],im->layers[0],im->sx*im->sy*sizeof(double));
  memcpy(image->layers[2],im->layers[0],im->sx*im->sy*sizeof(double));
 }
 else image=copyImage(im);

 // Create .ppm file
 f=fopen(name,"w");
 if (!f){fprintf(stderr,"writePPM(): Unable to create output file %s\n",name);return(0);}

 tmp=(unsigned char *)calloc(image->sx*image->sy*3,sizeof(unsigned char));
 if (!tmp){fprintf(stderr,"writePPM(): Unable to allocate memory for uint8 image data\n");fclose(f);return(0);}

 // Clip to range
 for (ly=0; ly<3; ly++)
  for (j=0;j<image->sy;j++)
   for (i=0;i<image->sx;i++)
   {
    if (*(image->layers[ly]+i+(j*image->sx))<.000001) *(image->layers[ly]+i+(j*image->sx))=0;
    if (*(image->layers[ly]+i+(j*image->sx))>254.999999) *(image->layers[ly]+i+(j*image->sx))=255.0;
   }

 // Convert image data to uint8
 for (ly=0; ly<3; ly++)
  for (j=0;j<image->sy;j++)
   for (i=0;i<image->sx;i++)
    *(tmp+((i+(j*image->sx))*3)+ly)=(unsigned char)((*(image->layers[ly]+i+(j*image->sx))));

 // Write out .ppm file
 fprintf(f,"P6\n");
 fprintf(f,"# TimeLapseFusion .ppm output\n");
 fprintf(f,"%d %d\n",image->sx,image->sy);
 fprintf(f,"255\n");
 fwrite(tmp,image->sx*image->sy*3*sizeof(unsigned char),1,f);
 fclose(f);

 free(tmp);
 deleteImage(image);

 return(1);
}



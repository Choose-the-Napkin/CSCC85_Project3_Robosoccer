/************************************************************************************
#  FrankenLuvcView - An unholy subset of luvcview to grab frames for                #
#  use in our CrunchyQuartet code.                                                  #
#                                                                                   #
#  F.J.E., May 2013 - All the code here is derived from luvcview.c or uvccapture.c  #
#                     The original version of luvcview.c is included.               #
#                     luvcview and uvccapture by Laurent Pinchart & Michel Xhaard   #
#                                                                                   #
#  As per the GLP V2.0 license terms. This code is hereby distributed freely. You   #
#   are free to modify and redistribute this code under the provisions of the       #
#   GPL V2 or newer.                                                                #
************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <string.h>
#include <pthread.h>
#include <jpeglib.h>
#include <linux/videodev.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <X11/Xlib.h>
#include "v4l2uvc.h"
#include "gui.h"
#include "utils.h"
#include "color.h"
/* Fixed point arithmetic */
#define FIXED Sint32
#define FIXED_BITS 16
#define TO_FIXED(X) (((Sint32)(X))<<(FIXED_BITS))
#define FROM_FIXED(X) (((Sint32)(X))>>(FIXED_BITS))

#define INCPANTILT 64 // 1°

static const char version[] = VERSION;
struct vdIn *videoIn;
    
int compress_yuyv_to_jpeg (struct vdIn *vd, FILE * file, int quality)
{
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    JSAMPROW row_pointer[1];
    unsigned char *line_buffer, *yuyv;
    int z;

    fprintf (stderr, "Compressing YUYV frame to JPEG image.\n");

    line_buffer = (unsigned char *)calloc (vd->width * 3, 1);
    yuyv = vd->framebuffer;

    cinfo.err = jpeg_std_error (&jerr);
    jpeg_create_compress (&cinfo);
    jpeg_stdio_dest (&cinfo, file);

    cinfo.image_width = vd->width;
    cinfo.image_height = vd->height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults (&cinfo);
    jpeg_set_quality (&cinfo, quality, TRUE);

    jpeg_start_compress (&cinfo, TRUE);

    z = 0;
    while (cinfo.next_scanline < cinfo.image_height) {
        int x;
        unsigned char *ptr = line_buffer;

        for (x = 0; x < vd->width; x++) {
            int r, g, b;
            int y, u, v;

            if (!z)
                y = yuyv[0] << 8;
            else
                y = yuyv[2] << 8;
            u = yuyv[1] - 128;
            v = yuyv[3] - 128;

            r = (y + (359 * v)) >> 8;
            g = (y - (88 * u) - (183 * v)) >> 8;
            b = (y + (454 * u)) >> 8;

            *(ptr++) = (r > 255) ? 255 : ((r < 0) ? 0 : r);
            *(ptr++) = (g > 255) ? 255 : ((g < 0) ? 0 : g);
            *(ptr++) = (b > 255) ? 255 : ((b < 0) ? 0 : b);

            if (z++) {
                z = 0;
                yuyv += 4;
            }
        }

        row_pointer[0] = line_buffer;
        jpeg_write_scanlines (&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress (&cinfo);
    jpeg_destroy_compress (&cinfo);

    free (line_buffer);

    return (0);
}

int main(int argc, char *argv[])
{
	int status;
	unsigned char *p = NULL;
	int hwaccel = 0;
	const char *videodevice = NULL;
	const char *mode = NULL;
	int format = V4L2_PIX_FMT_MJPEG;
	int i;
	int grabmethod = 1;
	int width = 1280;
	int height = 720;
	int fps = 30;
	unsigned char frmrate = 0;
	char *avifilename = NULL;
	int queryformats = 0;
	int querycontrols = 0;
	int readconfigfile = 0;
	char *separateur;
	char *sizestring = NULL;
	char *fpsstring  = NULL;
	int enableRawStreamCapture = 0;
	int enableRawFrameCapture = 0;
	FILE *file;

	printf("FrankenLuvcview %s\n\n", version);
/*
	for (i = 1; i < argc; i++) {
		if (argv[i] == NULL || *argv[i] == 0 || *argv[i] != '-') {
			continue;
		}
		if (strcmp(argv[i], "-d") == 0) {
			if (i + 1 >= argc) {
				printf("No parameter specified with -d, aborting.\n");
				exit(1);
			}
			videodevice = strdup(argv[i + 1]);
		}
		if (strcmp(argv[i], "-g") == 0) {
			grabmethod = 0;
		}
		if (strcmp(argv[i], "-w") == 0) {
			hwaccel = 1;
		}
		if (strcmp(argv[i], "-f") == 0) {
			if (i + 1 >= argc) {
				printf("No parameter specified with -f, aborting.\n");
				exit(1);
			}
			mode = argv[i + 1];

			if (strcasecmp(mode, "yuv") == 0 || strcasecmp(mode, "YUYV") == 0) {
				format = V4L2_PIX_FMT_YUYV;
			} else if (strcasecmp(mode, "jpg") == 0 || strcasecmp(mode, "MJPG") == 0) {
				format = V4L2_PIX_FMT_MJPEG;
			} else {
				printf("Unknown format specified. Aborting.\n");
				exit(1);
			}
		}
		if (strcmp(argv[i], "-s") == 0) {
			if (i + 1 >= argc) {
				printf("No parameter specified with -s, aborting.\n");
				exit(1);
			}

			sizestring = strdup(argv[i + 1]);

			width = strtoul(sizestring, &separateur, 10);
			if (*separateur != 'x') {
				printf("Error in size use -s widthxheight\n");
				exit(1);
			} else {
				++separateur;
				height = strtoul(separateur, &separateur, 10);
				if (*separateur != 0)
					printf("hmm.. dont like that!! trying this height\n");
			}
		}
		if (strcmp(argv[i], "-i") == 0){
			if (i + 1 >= argc) {
				printf("No parameter specified with -i, aborting.\n");
				exit(1);
			}
			fpsstring = argv[i + 1];
			fps = strtoul(fpsstring, &separateur, 10);
			if(*separateur != '\0') {
				printf("Invalid frame rate '%s' specified with -i. "
						"Only integers are supported. Aborting.\n", fpsstring);
				exit(1);
			}
		}
		if (strcmp(argv[i], "-S") == 0) {
			enableRawStreamCapture = 1;
		}
		if (strcmp(argv[i], "-c") == 0) {
			enableRawFrameCapture = 1;
		}
		if (strcmp(argv[i], "-C") == 0) {
			enableRawFrameCapture = 2;
		}
		if (strcmp(argv[i], "-o") == 0) {
			if (i + 1 >= argc) {
				printf("No parameter specified with -o, aborting.\n");
				exit(1);
			}
			avifilename = strdup(argv[i + 1]);
		}
		if (strcmp(argv[i], "-L") == 0) {
			queryformats = 1;
		}
		if (strcmp(argv[i], "-l") == 0) {
			querycontrols = 1;
		}

		if (strcmp(argv[i], "-r") == 0) {
			readconfigfile = 1;
		}
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("usage: uvcview [-h -d -g -f -s -i -c -o -C -S -L -l -r]\n");
			printf("-h	print this message\n");
			printf("-d	/dev/videoX       use videoX device\n");
			printf("-g	use read method for grab instead mmap\n");
			printf("-w	disable SDL hardware accel.\n");
			printf("-f	choose video format (YUYV/yuv and MJPG/jpg are valid, MJPG is default)\n");
			printf("-i	fps           use specified frame interval\n");
			printf("-s	widthxheight      use specified input size\n");
			printf("-c	enable raw frame capturing for the first frame\n");
			printf("-C	enable raw frame stream capturing from the start\n");
			printf("-S	enable raw stream capturing from the start\n");
			printf("-o	avifile  create avifile, default video.avi\n");
			printf("-L	query valid video formats\n");
			printf("-l	query valid controls and settings\n");
			printf("-r	read and set control settings from luvcview.cfg\n");
			exit(0);
		}
	}
*/

	if (videodevice == NULL || *videodevice == 0) {
	 	videodevice = "/dev/video0";
	}

	if (avifilename == NULL || *avifilename == 0) {
		avifilename = "video.avi";
	}
	videoIn = (struct vdIn *) calloc(1, sizeof(struct vdIn));
	if ( queryformats ) {
		/* if we're supposed to list the video formats, do that now and go out */
		check_videoIn(videoIn,(char *) videodevice);
		free(videoIn);
		exit(1);
	}
	if (init_videoIn
			(videoIn, (char *) videodevice, width, height, fps, format,
			 grabmethod, avifilename) < 0)
		exit(1);
	/* if we're supposed to list the controls, do that now */
	if ( querycontrols )
		enum_controls(videoIn->fd);

	/* if we're supposed to read the control settings from a configfile, do that now */
	if ( readconfigfile )
		load_controls(videoIn->fd);

	/* main big loop */
	while (videoIn->signalquit) {
		if (uvcGrab(videoIn) < 0) {
			printf("Error grabbing\n");
			break;
		}

		file=fopen("test.jpg","w");
                if (file != NULL) {
                    switch (videoIn->formatIn) {
                    case V4L2_PIX_FMT_YUYV:
                        compress_yuyv_to_jpeg (videoIn, file, 100);
                        break;
                    default:
                        fwrite (videoIn->tmpbuffer, videoIn->buf.bytesused + DHT_SIZE, 1,
                                file);
                        break;
                    }
                    fclose (file);
                    videoIn->getPict = 0;
                }

	}
	close_v4l2(videoIn);
	free(videoIn);
	printf("Cleanup done. Exiting ...\n");
        exit(0);
}


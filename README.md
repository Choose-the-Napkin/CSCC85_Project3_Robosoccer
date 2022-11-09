UTSC - RoboSoccer

Will compile and run on Linux provided the OpenGL and Bluetooth libraries are
installed.

From the project's folder type:

>./configure

If any dependencies are missing, install them and re-run configure.

Once the configuration ends successfully, type

>make

The executable file will be in ./src so you can run it with

./src/roboSoccer /dev/video1 0 0

See the handout and the executable's usage doc. for the meaning of each
command line parmeter.



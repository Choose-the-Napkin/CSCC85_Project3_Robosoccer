#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.68])
AC_INIT([Robo Soccer], [1.0], [wisalam@live.com])
AM_INIT_AUTOMAKE([-Wall -Werror foreign subdir-objects])
# AC_CONFIG_SRCDIR([roboSoccer.c])
AC_CONFIG_HEADERS([include/config.h])

AC_PREFIX_DEFAULT([$PWD])

# Checks for programs.
AC_PROG_CC
AC_PROG_INSTALL
AC_PROG_AWK
AC_PROG_MKDIR_P
AC_PROG_LN_S

# Checks for libraries.
AC_CHECK_LIB([GL], [glEnable])
AC_CHECK_LIB([GLU], [gluBeginTrim])
AC_CHECK_LIB([SDL], [SDL_Init])
AC_CHECK_LIB([glut], [glutInit])
AC_CHECK_LIB([jpeg], [jpeg_CreateCompress])
AC_CHECK_LIB([bluetooth], [baswap])
AC_CHECK_LIB([m], [sqrt])
AC_CHECK_LIB([pthread], [pthread_create])

# Checks for header files.
AC_PATH_X
AC_CHECK_HEADERS([fcntl.h inttypes.h limits.h malloc.h stdint.h stdlib.h string.h sys/file.h sys/ioctl.h sys/param.h sys/socket.h sys/time.h unistd.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_C_INLINE
AC_TYPE_OFF_T
AC_TYPE_SIZE_T
AC_TYPE_SSIZE_T
AC_TYPE_UINT16_T
AC_TYPE_UINT32_T
AC_TYPE_UINT64_T
AC_TYPE_UINT8_T

# Checks for library functions.
AC_FUNC_MALLOC
AC_FUNC_MMAP
AC_FUNC_REALLOC
AC_CHECK_FUNCS([floor ftruncate memset pow select socket sqrt strdup strerror strncasecmp])

AC_PROG_RANLIB

AC_CONFIG_FILES([
	Makefile
	src/Makefile
])
AC_OUTPUT


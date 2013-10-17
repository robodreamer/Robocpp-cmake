/*
 * hubo-read-trajectory-as-func.h
 * This file is part of <Robocpp>
 *
 * Copyright (C) 2013 - Andy Park (andypark@purdue.edu)
 *
 * <Robocpp> is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * <Robocpp> is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with <Robocpp>. If not, see <http://www.gnu.org/licenses/>.
 */



#ifndef HUBO_READ_H
#define HUBO_READ_H

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <time.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>
#include <stdio.h>

// for timer
#include <time.h>
#include <sched.h>
#include <sys/io.h>
#include <unistd.h>

// for RT
#include <stdlib.h>
#include <sys/mman.h>

// for hubo
#include <hubo.h>

// for ach
#include <errno.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include <ach.h>

// for keyboard

#include <termio.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <armadillo>
#include <iostream>


#include "hubo-ref-filter.h"

using namespace arma;


#ifdef __cplusplus
extern "C" {
#endif

//int runTrajFunction(char*, int,  bool, bool);
int runTrajFunction(char* s, int mode,  bool compliance_mode, bool pause_feature);
double* getEncoderValues();

#ifdef __cplusplus
}
#endif

// Function to get encoder value from the Ach channel - store into ARMA vec format
vec getEncoderValues_new(int mode = 0);

// Function to get reference value from the Ach channel - store into ARMA vec format
vec getRefValues_new(int mode = 0);

#endif // HUBOREAD_H

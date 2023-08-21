/*
 * lookup.h
 *
 *  Created on: Jul 13, 2017
 *      Author: Ian
 */

#ifndef LOOKUP_H_
#define LOOKUP_H_

// includes
#include "PE_types.h"

unsigned int lookup_u16_u16(unsigned int X_value, volatile const unsigned int * p_X_axis, volatile const unsigned int * p_Y_axis, unsigned int breakpoints);
int lookup_s16_s16(int X_value, volatile const int * p_X_axis, volatile const int * p_Y_axis, unsigned int breakpoints);
int lookup_u16_s16(unsigned int X_value, volatile const unsigned int * p_X_axis, volatile const int * p_Y_axis, unsigned int breakpoints);

#endif /* LOOKUP_H_ */

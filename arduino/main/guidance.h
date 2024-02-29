/***
 * Copyright 2022, The Pennsylvania State University, All Rights Reserved.
 * Unauthorized use and/or redistribution is disallowed.
 * This library is distributed without any warranty; without even
 * the implied warranty of fitness for a particular purpose.
 *
 * Pennsylvania State University Unmanned Aerial System Research Laboratory (PURL)
 * Department of Aerospace Engineering
 * 229 Hammond
 * The Pennsylvania State University
 * University Park, PA 16802
 * http://purl.psu.edu
 *
 * Contact Information:
 * Dr. Thanakorn Khamvilai
 * Email : thanakorn.khamvilai@psu.edu
 *
 * EndCopyright
 ***/

#ifndef GUIDANCE_H
#define GUIDANCE_H

#include "math_utils.h"

class Guidance
{
public:
	Guidance();
	~Guidance();

	void init();
  void update(const sens_t&, const state_t&, const rc_t&);
  void print();

	guidance_t cmd;

private:

};

#endif
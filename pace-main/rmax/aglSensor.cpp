/*** BeginCopyright
 * Copyright 2002, Georgia Institute of Technology, All Rights Reserved.
 * Unauthorized use and/or redistribution is disallowed.
 * This library is distributed without any warranty; without even
 * the implied warranty of fitness for a particular purpose.
 *
 * UAV Laboratory
 * School of Aerospace Engineering
 * Georgia Institute of Technology
 * Atlanta, GA 30332
 * http://controls.ae.gatech.edu
 *
 * Contact Information:
 * Prof. Eric N. Johnson
 * http://www.ae.gatech.edu/~ejohnson
 * Tel : 404 385 2519
 * EndCopyright
 ***/

#include <stdlib.h>

#include "rmax/aglSensor.h"
#include "rmax/sensors_ref.h"
#include "rmax/serial.h"
#include "rmax/logger.h"
#include "esim/util.h"

unsigned char crc8_terra(unsigned char *p, unsigned char len){
    unsigned int i;
    unsigned int crc = 0x0;
    while(len--){
        i = (crc ^ *p++) & 0xFF;
        crc = (crc_table_terra[i] ^ (crc << 8)) & 0xFF;
    }
    return crc & 0xFF;
}


void isort(int *a, int n)//  *a is an array pointer function
{
  for (int i = 1; i < n; ++i)
  {
      int j = a[i];
      int k;

      for (k = i - 1; (k >= 0) && (j < a[k]) ; k--)
      {
          a[k + 1] = a[k];
      }

      a[k + 1] = j;
  }

}

void outputSonarToTempStructure(struct senSonar_ref *sonar, aglOut_ref *temp, unsigned char source)
{
	// sonarOut_ref and aglOut_ref are effectively the same structures, so just copy each field across
	temp->source = source;
	temp->align = sonar->out->align;
	temp->altitude = sonar->out->altitude;
	temp->alt_max = sonar->out->alt_max;
	temp->alt_min = sonar->out->alt_min;
	temp->itime = sonar->out->itime;
	temp->lastUpdate = sonar->out->lastUpdate;
	temp->status = sonar->out->status;
	temp->timeOut = sonar->out->timeOut;
	for(int index = 0; index < 3; ++index){
		temp->aglR[index] = sonar->data->sonR[index];
	}
	temp->RaglMax = sonar->data->RsonMax;
	temp->RaglMin = sonar->data->RsonMin;
	temp->RaglOOR = sonar->data->RsonOOR;
	temp->sqrtRaglMult = sonar->data->sqrtRsonMult;
}

void updateAglSensors(struct sensors_ref *sen, struct datalinkMessageRangeFinderCmd_ref* rangeFinderCmd,
					  double time, char sonUpdateMode)
{
	struct senAGL_ref *agl = sen->agl;
	struct serialPort_ref  *port;
	int index, done;
    static unsigned char input_terra[4];
    int terra_range;
    int crc_terra_output;
	unsigned char *bf;

	/* read new sonar data - can have up to 2 sonars*/
	for(int aglSensorCounter = 0; aglSensorCounter < 2; ++aglSensorCounter)
	{
		struct senSonar_ref *sonar = sen->agl->sonar0;
		port = sonar->p;

		readPort( port );
		if( port->dataSource != PORT_OFF ) {		

			done = 0;
			index = 0;
			input_terra[0]=0;
			input_terra[1]=0;
			input_terra[2]=0;
			input_terra[3]=0;
			// Enables up to 2 commands to be sent, which is all that is required (one for display mode, one for sensing mode)
			if(((sonar->decodeMode == SONAR_TERRA) || (sonar->decodeMode == SONAR_TERRA_DUO)) &&
				(sonar->data->terra_init < MAX_TERRA_COMMANDS))
			{ // Send the defined commands - it will be zero on startup
				if(sonar->data->terraCommands[sonar->data->terra_init] != '\0')
				{
					writePort(port, &sonar->data->terraCommands[sonar->data->terra_init], 1);
				}
				sonar->data->terra_init += 1;
			}

			switch( sonar->decodeMode ) {
			default:
			case SONAR_RMAX:
				sonar->sonarSize = 5;
				break;
			case SONAR_QUAD:
				sonar->sonarSize = 6;
				break;
			case SONAR_TERRA:
				sonar->sonarSize = 4;
				sonar->data->terra_avg = MAX( sonar->data->terra_avg, 1 );
				break;
			case SONAR_TERRA_DUO:
				sonar->sonarSize = 7;
				sonar->data->terra_avg = MAX( sonar->data->terra_avg, 1 );
				break;
			case SONAR_TERRA_MODE:
				sonar->sonarSize = 4;
				sonar->data->terra_avg = MAX( sonar->data->terra_avg, 1); //makes sure its bigger than 1
				sonar->data->terra_avg = MIN( sonar->data->terra_avg, 20); // makes sure smaller that 20(array size to hold onto-can be changed-but needs to be recompiled)
				break;
			}

			// look for a header
			while( index <= port->bytesread - sonar->sonarSize && !done ) {
					// sonarSize adds extra byte that is needed for the mb7066->all other models including rmax are 5, whereas the mb7066 is 6
					// sonarSize of teraranger is 4bytes(first byte "T", second,third bytes are data, fourth is checksum

				if( (port->buffer[index+sonar->sonarSize-1] == 0x0D ) || 
					sonar->decodeMode == SONAR_TERRA || 
					sonar->decodeMode == SONAR_TERRA_DUO ||
					sonar->decodeMode == SONAR_TERRA_MODE) {   // sequence header (carriage return = ascii 13)

					switch( sonar->decodeMode ) {
					case SONAR_RMAX:
						bf = &(port->buffer[index]);
						sonar->data->raw = strtol((char *)bf, NULL, 16);
						break;

					case SONAR_QUAD:
						if( port->buffer[index] == 'R') {
							bf = &(port->buffer[index+1]);
							sonar->data->raw = strtol((char *)bf, NULL, 10);
						} else {
							done = 1; /* not a valid packet */
						}
						break;

					case SONAR_TERRA:
						if( port->buffer[index] == 'T' ) {
							input_terra[0] = port->buffer[index];
							input_terra[1] = port->buffer[index+1];
							input_terra[2] = port->buffer[index+2];
							input_terra[3] = port->buffer[index+3];
							crc_terra_output = crc8_terra(input_terra,3u);

							if(crc_terra_output==input_terra[3]){
								//crc matched
								terra_range  = input_terra[1] << 8;
								terra_range |= input_terra[2];
								terra_range += (int)sonar->data->bias; // make sure bias is in mm

								sonar->data->terra_raw += terra_range;
								sonar->out->align++;
								if(sonar->out->align >= sonar->data->terra_avg) {
									sonar->out->altitude = (double)(sonar->data->terra_raw*C_MM2M*C_M2FT/sonar->out->align) ; //terra_range is reported in millimeters
									sonar->data->terra_raw = 0;
									sonar->out->align = 0;
								}

								if( (sonar->out->altitude > sonar->out->alt_max) ||
									(sonar->out->altitude < sonar->out->alt_min)){ //this part is done to not cause problems with below checking of raw data
									//out of range                                 // not sure if it's really needed
									sonar->data->raw = 0xffff;
								}else{
									sonar->data->raw = 0xf000;
								}

							}
						} else {
							done = 1;
						} break;

					case SONAR_TERRA_DUO:
						if( port->buffer[index] == 'T' && port->buffer[index+3] == 'S' ) {
							unsigned char checksumText;
							checksumText = crc8_terra(&(port->buffer[index]),6);

							if( checksumText == port->buffer[index+6] ) {
								//crc matched

								/* this driver uses the IR data */
								sonar->data->rawDuoIR = port->buffer[index+1] << 8;
								sonar->data->rawDuoIR |= port->buffer[index+2];
								sonar->data->rawDuoIR += (int)sonar->data->bias; // make sure bias is in mm

								/* puts the sonar here for reference (not used) */
								sonar->data->rawDuoSonar = port->buffer[index+4] << 8;
								sonar->data->rawDuoSonar |= port->buffer[index+5];

								sonar->data->terra_raw += sonar->data->rawDuoIR;
								sonar->out->align++;

								if( sonar->out->align >= sonar->data->terra_avg ) {
									sonar->out->altitude = (double)(sonar->data->terra_raw*C_MM2M*C_M2FT/sonar->out->align ) ; //terra_range is reported in millimeters
									sonar->data->terra_raw = 0;
									sonar->out->align = 0;
								}

								if( (sonar->out->altitude > sonar->out->alt_max) ||
									(sonar->out->altitude < sonar->out->alt_min)) { //this part is done to not cause problems with below checking of raw data
									//out of range                                        // not sure if it's really needed
									sonar->data->raw = 0xffff;
								} else {
									sonar->data->raw = 0xf000;
								}

							}
						} else {
							done = 1;
						} break ;

					case SONAR_TERRA_MODE:
						if( port->buffer[index] == 'T' ) {
							input_terra[0] = port->buffer[index];
							input_terra[1] = port->buffer[index+1];
							input_terra[2] = port->buffer[index+2];
							input_terra[3] = port->buffer[index+3];
							crc_terra_output = crc8_terra(input_terra,3u);

							if(crc_terra_output==input_terra[3]){
								//crc matched
								// this uses mode filtering instead of averaging-this should be better at throwing out single bad values-this might make more sense for sonar filtering(more dirty)
								// terra_avg will be amount to keep in array size with max being 20 for now
								terra_range  = input_terra[1] << 8;
								terra_range |= input_terra[2];
								terra_range += (int)sonar->data->bias; // make sure bias is in mm

								if(sonar->out->align >= sonar->data->terra_avg){
									// do mode/median filtering , not sure if we need to zero out terra_array(maybe its fine to just simply overwrite values)
									// currently only median is implemented->from maxbotic forum median/mode filtering works way better than averaging
									// terra_range is reported in millimeters
										isort(sonar->mode->terra_array, sonar->data->terra_avg); //sort values in array
										sonar->mode->terra_mode_out = sonar->mode->terra_array[(unsigned char)sonar->data->terra_avg/2]; //median filter
										sonar->out->altitude = (double)(sonar->mode->terra_mode_out*C_MM2M*C_M2FT) ;
										sonar->out->align = 0; //zero it out

									if( (sonar->out->altitude > sonar->out->alt_max) ||
										(sonar->out->altitude < sonar->out->alt_min)){ //this part is done to not cause problems with below checking of raw data
										//out of range                                 // not sure if it's really needed
										sonar->data->raw = 0xffff;
									}else{
										sonar->data->raw = 0xf000;
									}

									for(int j=0;j<sonar->data->terra_avg;j++){ //zero out values - just in-case
										sonar->mode->terra_array[j]=0;
									}

								}else{
									// add values to array for mode filtering
									//sonar->out->align = MAX(sonar->out->align,0); //make sure its non-neg (this might not be needed)
									sonar->mode->terra_array[(unsigned char)sonar->out->align] = terra_range;
									sonar->out->align++;
								}

								sonar->data->terra_raw = terra_range; //raw value
							}
						} else {
							done = 1;
						} break;

					default:
						done = 1; // to prevent somebody from errornously set the wrong sonar->hardware
						break;
					}

					if( !done ) { // to prevent somebody from errornously set the wrong sonar->decodeMode
						if( (sonar->data->raw < 0xff00)  && (sonar->decodeMode !=SONAR_TERRA) &&
								(sonar->decodeMode != SONAR_TERRA_DUO) && (sonar->decodeMode !=SONAR_TERRA_MODE)) { //decodeMode(2) is terraranger which doesn't have raw data/bias
							sonar->out->altitude = ((double)(sonar->data->raw - sonar->data->bias)*sonar->data->sf);
						}

						if ( ( sonar->out->altitude > sonar->out->alt_min ) &&
							 ( sonar->out->altitude < sonar->out->alt_max ) &&
							( sonar->data->raw < 0xff00) ) {// Henrik error code
							sonar->out->status  = AGL_GOOD;
						} else {
							sonar->out->status  = AGL_OUTOFRANGE;
						}

						if( 1 == sonUpdateMode ) {
							sonar->out->status = AGL_GOOD;
							//kind of a hack, but you always want sonar to do sensor update and outlier detection even if it is out of range
							//especially true when take off or jump is detected, otherwise nav will go crazy
						}

						sonar->out->lastUpdate = time;
						sonar->out->itime++; // for possible fake data
						index += sonar->sonarSize - 1;
					}
				}
				index++; // start seq not found, go to next byte

				if( index < 0 ) index = BUFFERSIZE - 1;
			}

			clearPort( port, index );
		}

		if( time > sonar->out->lastUpdate + sonar->out->timeOut ) sonar->out->status = AGL_OFF;
		if( time < sonar->out->lastUpdate ) sonar->out->lastUpdate = -99.0;
	}

	// Now that all AGL sensors have been updated,
	//  choose the appropriate sources to fill the temporary structures and filter/decide on final output
	// Also check whether nav should perform an update with the AGL sensors
	int numSensorsUpdated = 0;
	int numSensorsConfigured = 0;
	int prevNumSensorsConfigured = 0;
	for(int aglSourceIdx = 0; aglSourceIdx < MAX_AGL_SOURCES; ++aglSourceIdx)
	{
		struct aglOut_ref *tempOut = agl->temp0;
		switch(aglSourceIdx)
		{
			case 1:
				tempOut = agl->temp1;
				break;
			case 2:
				tempOut = agl->temp2;
				break;
			default:
				break;
			// default: do nothing - original case already set in variable definition
		}

		// Switch inputs for saving to temporary directory
		prevNumSensorsConfigured = numSensorsConfigured;
		numSensorsConfigured += 1; // Remove one off at the end if the sensor not configured
		switch(agl->set->sources[aglSourceIdx])
		{
			case AGL_SOURCE_ANY:
				// This is the backwards compatible case - the default in the db file.  Use the first valid (on) input
				//  from an AGL sensor source - the test is whether the port is not OFF.  If using this method, only
				//  one AGL sensor port should be turned on.
				if(agl->sonar0->p->dataSource != PORT_OFF) {
					// TODO: Handle the special case of DJI and Ardupilot
					outputSonarToTempStructure(agl->sonar0, tempOut, AGL_SOURCE_SONAR_1);
				} 
				else {
					// Not turned on
					tempOut->status = AGL_OFF;
					tempOut->source = AGL_SOURCE_SONAR_1; // Default
				}
				break;
			case AGL_SOURCE_SONAR_1:
				// TODO: Handle the special case of DJI and Ardupilot
				if(agl->sonar0->p->dataSource != PORT_OFF) {
					outputSonarToTempStructure(agl->sonar0, tempOut, AGL_SOURCE_SONAR_1);
				} else {
					// Not turned on
					tempOut->status = AGL_OFF;
				}
				break;
			case AGL_SOURCE_NONE:
			default:
				// Nothing is copied to the output directory; just make sure the output directory has
				//  a known bad output
				tempOut->status = AGL_OFF;
				tempOut->source = AGL_SOURCE_SONAR_1; // Default
				numSensorsConfigured -= 1; // No sensor configured for this input - remove the starting addition
		}

		// If a trusted sensor has timed out, enable all sensors to find a better one
		if((agl->set->useTrustSettings == 1) && (agl->work->aglTrusted[aglSourceIdx] == 1) &&
			((tempOut->status == AGL_OFF) || (agl->out->status == AGL_OFF))){
			// Trusted sensor timed out
			bool enabledOtherSensor = false;
			for(int trustedIdx = 0; trustedIdx < MAX_AGL_SOURCES; ++trustedIdx)
			{
				if(agl->work->aglTrusted[trustedIdx] == 0){
					enabledOtherSensor = true; // Another sensor was untrusted, and has now been enabled.
					agl->work->aglTrusted[trustedIdx] = 1;
				}
			}
			if(enabledOtherSensor == true){
				logVerbose("A trusted AGL sensor has timed out.  Enabling all AGL sensors to find a new valid source");
			}
		}

		if(numSensorsConfigured > prevNumSensorsConfigured){ // A sensor was checked/configured
			if(tempOut->itime != agl->work->aglUpdate[aglSourceIdx]){
				// An update has occured
				numSensorsUpdated += 1;
				if((tempOut->itime >= (agl->work->aglUpdate[aglSourceIdx] + agl->set->receivesToUpdate)) && // itime moved past the "do update now" trigger
					((agl->set->useTrustSettings == 0) || (agl->work->aglTrusted[aglSourceIdx] == 1))){ // Not using trust settings, or this is the trusted sensor
					// Enough updates has occured on one sensor to warrant an update
					agl->work->doUpdate = 1;
				}
			}
		}
	}
	// Check whether to update nav.  Note that it may already be set to 1,
	//  at which point this step doesn't do anything, but doesn't hurt anything either.
	// Doesn't matter whether trust settings are on.  If all sensors are updated, then the trusted sensor is updated...
	if(numSensorsUpdated >= numSensorsConfigured) {
		// Update NAV
		agl->work->doUpdate = 1;
	}

	// If the main aglOut hasn't been updated in nav (final integration of temporary updates) for a timeout period, then
	//  it, too, must be timed out.
	if( time > agl->out->lastUpdate + agl->out->timeOut ) agl->out->status = AGL_OFF;
	if( time < agl->out->lastUpdate ) agl->out->lastUpdate = -99.0;

	agl->out->status = agl->sonar0->out->status;
}

/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2013                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     vexuser.c                                                    */
/*    Author:     James Pearman                                                */
/*    Created:    7 May 2013                                                   */
/*                                                                             */
/*    Revisions:                                                               */
/*                V1.00  XX XXX 2013 - Initial release                         */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    The author is supplying this software for use with the VEX cortex        */
/*    control system. This file can be freely distributed and teams are        */
/*    authorized to freely use this program , however, it is requested that    */
/*    improvements or additions be shared with the Vex community via the vex   */
/*    forum.  Please acknowledge the work of the authors when appropriate.     */
/*    Thanks.                                                                  */
/*                                                                             */
/*    Licensed under the Apache License, Version 2.0 (the "License");          */
/*    you may not use this file except in compliance with the License.         */
/*    You may obtain a copy of the License at                                  */
/*                                                                             */
/*      http://www.apache.org/licenses/LICENSE-2.0                             */
/*                                                                             */
/*    Unless required by applicable law or agreed to in writing, software      */
/*    distributed under the License is distributed on an "AS IS" BASIS,        */
/*    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. */
/*    See the License for the specific language governing permissions and      */
/*    limitations under the License.                                           */
/*                                                                             */
/*    The author can be contacted on the vex forums as jpearman                */
/*    or electronic mail using jbpearman_at_mac_dot_com                        */
/*    Mentor for team 8888 RoboLancers, Pasadena CA.                           */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

#include <stdlib.h>

#include "ch.h"  		// needs for all ChibiOS programs
#include "hal.h" 		// hardware abstraction layer header
#include "vex.h"		// vex library header
#include "vexctl.h"

// Digi IO configuration
static  vexDigiCfg  dConfig[kVexDigital_Num] = {
        { kVexDigital_1,    kVexSensorDigitalOutput, kVexConfigOutput,      0 },
        { kVexDigital_2,    kVexSensorDigitalOutput, kVexConfigOutput,      0 },
        { kVexDigital_3,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_4,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_5,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_6,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_7,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_8,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_9,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_10,   kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_11,   kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_12,   kVexSensorDigitalInput,  kVexConfigInput,       0 }
};

static  vexMotorCfg mConfig[kVexMotorNum] = {
        { kVexMotor_1,      kVexMotor393T,           kVexMotorNormal,       kVexSensorIME,         kImeChannel_1 },
        { kVexMotor_2,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_3,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_4,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_5,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_6,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_7,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_8,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_9,      kVexMotorUndefined,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_10,     kVexMotor393T,           kVexMotorNormal,       kVexSensorIME,         kImeChannel_2 }
};


/*-----------------------------------------------------------------------------*/
/** @brief      User setup                                                     */
/*-----------------------------------------------------------------------------*/
/** @details
 *  The digital and motor ports can (should) be configured here.
 */
void
vexUserSetup()
{
	vexDigitalConfigure( dConfig, DIG_CONFIG_SIZE( dConfig ) );
	vexMotorConfigure( mConfig, MOT_CONFIG_SIZE( mConfig ) );
}

/*-----------------------------------------------------------------------------*/
/** @brief      User initialize                                                */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This function is called after all setup is complete and communication has
 *  been established with the master processor.
 *  Start other tasks and initialize user variables here
 */
void
vexUserInit()
{

}

/*-----------------------------------------------------------------------------*/
/** @brief      Autonomous                                                     */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This thread is started when the autonomous period is started
 */
msg_t
vexAutonomous( void *arg )
{
    (void)arg;

    // Must call this
    vexTaskRegister("auton");

    while(1)
        {
        // Don't hog cpu
        vexSleep( 25 );
        }

    return (msg_t)0;
}

#define TOP_ARM 			kVexMotor_6
#define BOTTOM_ARM		kVexMotor_8
#define ARM_SPEED_MULTIPLIER		0.7

#define INTAKE		kVexMotor_7

#define LEFT_INTAKE_ARM		kVexMotor_3
#define RIGHT_INTAKE_ARM		kVexMotor_2

#define LEVER	kVexMotor_4


#define INTAKE_ARM_SPEED_MULTIPLIER		1
bool_t toggleMultiplier = 0;
bool_t prevTogglePress = 0;
int8_t leverInput = 0;

int8_t getInput(tCtlIndex forward, tCtlIndex back);
int8_t getTripleInput(tCtlIndex first, tCtlIndex second, tCtlIndex third);
void moveIntakeArms(int speed);

//Seperate thread that is forked from the operator control thread that controls the lever
static WORKING_AREA(waLeverTask, 512);
static msg_t
LeverTask(void *arg)
{
	(void)arg;
	vexTaskRegister("Lever Task");

	//moves the lever forward for 250 ms then back for 233 ms
	while(1) {
		leverInput = getTripleInput(Btn8R, Btn8U, Btn8D);
		switch(leverInput) {
			case 1:
				vexMotorSet(LEVER, 127);
				vexSleep(250);
				vexMotorSet(LEVER, -127);
				vexSleep(233);
				vexMotorSet(LEVER, 0);
				break;
			case 2:
				vexMotorSet(LEVER, 63);
				break;
			case 3:
				vexMotorSet(LEVER, -63);
				break;
			default:
				vexMotorSet(LEVER, 0);
				break;

		}
		vexSleep(25);
	}
	return (msg_t)0;
}

/*-----------------------------------------------------------------------------*/
/** @brief      Driver control                                                 */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This thread is started when the driver control period is started
 */
msg_t
vexOperator( void *arg )
{

	(void)arg;

	// Must call this
	vexTaskRegister("operator");
	chThdCreateStatic(waLeverTask, sizeof(waLeverTask), NORMALPRIO - 1, LeverTask, NULL);
	// Run until asked to terminate
	while(!chThdShouldTerminate())
	{

		//toggles the speed multiplier
		if(vexControllerGet(Btn8L) && !prevTogglePress) {
			toggleMultiplier = !toggleMultiplier;
		}
		prevTogglePress = vexControllerGet(Btn8L);

		//intake arm movement
		moveIntakeArms(getInput(Btn6D, Btn5D) * 127 * INTAKE_ARM_SPEED_MULTIPLIER);

		//arm movement
		if(!toggleMultiplier) {
			vexMotorSet(TOP_ARM, ARM_SPEED_MULTIPLIER * vexControllerGet(Ch3));
			vexMotorSet(BOTTOM_ARM, ARM_SPEED_MULTIPLIER * vexControllerGet(Ch2));
		} else {
			vexMotorSet(TOP_ARM, vexControllerGet(Ch3));
			vexMotorSet(BOTTOM_ARM, vexControllerGet(Ch2));
		}

		//controls intake and outake
		vexMotorSet(INTAKE, getInput(Btn6U, Btn5U) * 127);

		// Don't hog cpu
		vexSleep( 25 );
	}

	return (msg_t)0;
}


//sets the speed of the intake arms
void moveIntakeArms(int speed)
{
	vexMotorSet(LEFT_INTAKE_ARM, speed);
	vexMotorSet(RIGHT_INTAKE_ARM, -speed);
}

//returns 1 if the forward digital input is pressed and back isn't and -1 if back digital input is pressed and forward isn't
int8_t getInput(tCtlIndex forward, tCtlIndex back)
{
	if(vexControllerGet(forward) && !vexControllerGet(back)) {
		return 1;
	} else if(vexControllerGet(back) && !vexControllerGet(forward)) {
		return -1;
	} else {
		return 0;
	}

}

//returns 1 if the first button was pressed, two if the second button was pressed, 3 if the third button was pressed, and 0 if nothing or multiple buttons were pressed
int8_t getTripleInput(tCtlIndex first, tCtlIndex second, tCtlIndex third) {
	if(vexControllerGet(first) && !vexControllerGet(second) && !vexControllerGet(third)) {
		return 1;
	} else if(!vexControllerGet(first) && vexControllerGet(second) && !vexControllerGet(third)) {
		return 2;
	} else if (!vexControllerGet(first) && !vexControllerGet(second) && vexControllerGet(third)){
		return 3;
	} else {
		return 0;
	}
}






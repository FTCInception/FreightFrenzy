/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package Inception.Skystone;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This file houses Autonomous code
 * It uses the main robot class Refbot to define the hardware and driving commands
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *
 *  See the Refbot class for encode-based driving controls that perform the actual movement.
 *
 */

@Autonomous(name="Mech: Auto Cal", group="Mechbot")
public class MechAuto_Cal extends LinearOpMode {
    /* Declare OpMode members. */

    private MechBot          robot   = new MechBot();   // Use a Pushbot's hardware

    private static final double     DRIVE_SPEED             = 0.9;
    private static final double     TURN_SPEED              = 0.5;
    double a=0;

    @Override
    public void runOpMode() {

        robot.logger.LOGLEVEL = robot.logger.LOGDEBUG ;

        robot.logger.logD("MechLog","  ");
        robot.logger.logD("MechLog","  ");
        robot.logger.logD("MechLog","Starting Auto");

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Init the robot setting for Autonomous play
        robot.initAutonomous(this);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        //sleep(24000);

        /*
        robot.fastEncoderStrafe(DRIVE_SPEED, 48, 10);
        robot.fastEncoderStrafe(DRIVE_SPEED, -48, 10);
        robot.fastEncoderStrafe(DRIVE_SPEED, 24, 10);
        robot.fastEncoderStrafe(DRIVE_SPEED, -24, 10);
        robot.fastEncoderStrafe(DRIVE_SPEED, 12, 10);
        robot.fastEncoderStrafe(DRIVE_SPEED, -12, 10);
        */

        //robot.fastEncoderStraight(DRIVE_SPEED, 96, 10,0.05);
        //sleep(1000);

        //robot.gyroRotate(TURN_SPEED,180, 6);

        //robot.fastEncoderStraight(DRIVE_SPEED, 96, 10,0.05);
        //sleep(1000);

        /*
        robot.fastEncoderStraight(DRIVE_SPEED, -48, 10,0.05);
        sleep(1000);
        robot.fastEncoderStraight(DRIVE_SPEED, 48, 10,0.05);
        sleep(1000);
        robot.fastEncoderStraight(DRIVE_SPEED, -24, 10,0.05);
        sleep(1000);
        robot.fastEncoderStraight(DRIVE_SPEED, 24, 10,0.05);
        sleep(1000);
        robot.fastEncoderStraight(DRIVE_SPEED, -12, 10,0.05);
        sleep(1000);
        robot.fastEncoderStraight(DRIVE_SPEED, 12, 10,0.05);
        */


/*
        robot.gyroRotate(TURN_SPEED,90-a, 6);
        robot.gyroRotate(TURN_SPEED,90-a, 6);
        robot.gyroRotate(TURN_SPEED,90-a, 6);
        robot.gyroRotate(TURN_SPEED,90-a, 6);
        robot.gyroRotate(TURN_SPEED,-90-a, 6);
        robot.gyroRotate(TURN_SPEED,-90-a, 6);
        robot.gyroRotate(TURN_SPEED,-90-a, 6);
        robot.gyroRotate(TURN_SPEED,-90-a, 6);

 */

/*
        robot.fastEncoderRotate(TURN_SPEED,90, 6);
        robot.logger.logD("MechLog",String.format("Turn err %f, heading %f",a,robot.getHeading()));
        robot.fastEncoderRotate(TURN_SPEED,90, 6);
        robot.logger.logD("MechLog",String.format("Turn err %f, heading %f",a,robot.getHeading()));
        robot.fastEncoderRotate(TURN_SPEED,90, 6);
        robot.logger.logD("MechLog",String.format("Turn err %f, heading %f",a,robot.getHeading()));
        robot.fastEncoderRotate(TURN_SPEED,90, 6);
        robot.logger.logD("MechLog",String.format("Turn err %f, heading %f",a,robot.getHeading()));
        robot.fastEncoderRotate(TURN_SPEED,-90, 6);
        robot.logger.logD("MechLog",String.format("Turn err %f, heading %f",a,robot.getHeading()));
        robot.fastEncoderRotate(TURN_SPEED,-90, 6);
        robot.logger.logD("MechLog",String.format("Turn err %f, heading %f",a,robot.getHeading()));
        robot.fastEncoderRotate(TURN_SPEED,-90, 6);
        robot.logger.logD("MechLog",String.format("Turn err %f, heading %f",a,robot.getHeading()));
        robot.fastEncoderRotate(TURN_SPEED,-90, 6);
        robot.logger.logD("MechLog",String.format("Turn err %f, heading %f",a,robot.getHeading()));
*/

        //sleep(5000);


        /*
        a = robot.fastEncoderStraight(DRIVE_SPEED, 72, 6, 0.06);
        a = robot.gyroRotate(TURN_SPEED,90-a, 4);
        a = robot.gyroRotate(TURN_SPEED,90-a, 4);
        robot.straightA = -a;
        a = robot.fastEncoderStraight(DRIVE_SPEED, 48, 6, 0.06);
        a = robot.gyroRotate(TURN_SPEED,90-a, 4);
        a = robot.gyroRotate(TURN_SPEED,90-a, 4);
        */

        robot.logger.logD("MechLog","Done");
        robot.logger.logD("MechLog","  ");
        robot.logger.logD("MechLog","  ");

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}

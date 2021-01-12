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

package Inception.UltimateGoal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
 * @Disabled
 */

@Autonomous(name="Mech: Auto Cal", group="Mechbot")
public class MechAuto_Cal extends LinearOpMode {
    /* Declare OpMode members. */

    private MechBot          robot   = new MechBot();   // Use a Pushbot's hardware

    private static final double     DRIVE_SPEED             = 1.0;
    private static final double     TURN_SPEED              = 0.8;
    double a=0;
    double P=0.1;

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

        while (!isStarted()) {
            sleep(250);
            telemetry.addData("Heading", "%f", robot.getHeading());
            telemetry.update();
        }

        // Wait for the game to start (driver presses PLAY)
        //waitForStart();

        //robot.MotorCal(robot.leftFDrive, "lf", 1.0 );
        //robot.MotorCal(robot.leftBDrive, "lb",1.0 );
        //robot.MotorCal(robot.rightFDrive, "rf", 1.0 );
        //robot.MotorCal(robot.rightBDrive, "rb", 1.0 );

        robot.straightA = a;
        // Drive straight a while without any gyro correction
        // Power adjust right/left to get a straight line
        //a=robot.fastEncoderStraight(DRIVE_SPEED,96.0,60, 0);
        //sleep(5000);

        /*
        // Now turn 4 full rotations and back with a pause to check for over/under rotation
        // Check for drift as well (not sure how to fix drift.)
        double turnSize=360.0*4.0;
        robot.gyroRotate(TURN_SPEED,turnSize-a, 60);
        telemetry.addData("Heading", "%f", robot.getHeading());
        telemetry.update();

        sleep(3000);

        robot.gyroRotate(TURN_SPEED,-turnSize-a, 60);
        telemetry.addData("Heading", "%f", robot.getHeading());
        telemetry.update();
        */

        /*
        // 3-ring pickup test.
        robot.intake_motor.setPower(1.0);
        a=robot.fastEncoderStraight(DRIVE_SPEED,30.0,60, P);
        robot.straightA = a;
        a=robot.fastEncoderStraight(.2,9.0,60, P);
        robot.straightA = a;
        sleep(2000);
        robot.intake_motor.setPower(0);
        */

        // Test strafe in here
        a=robot.fastEncoderStrafe(DRIVE_SPEED,48, 60, P );
        robot.straightA = a;
        a=robot.fastEncoderStrafe(DRIVE_SPEED,-48, 60, P);
        robot.straightA = a;
        a=robot.fastEncoderStrafe(DRIVE_SPEED,48, 60, P );
        robot.straightA = a;
        a=robot.fastEncoderStrafe(DRIVE_SPEED,-48, 60, P );


        // Now a bunch of junk to try different movements like short, medium, long, etc.
        /*
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,72.0,60, P);
        a=robot.gyroRotate(TURN_SPEED,(180)-a, 60);
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,72.0,60, P);
        a=robot.gyroRotate(TURN_SPEED,(-180)-a, 60);
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,72.0,60, P);
        a=robot.gyroRotate(TURN_SPEED,(180)-a, 60);
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,72.0,60, P);
        */

        /*
        double turnDirection = -1.0;

        robot.foundation2.setPosition(0.35);
        a=robot.gyroRotate(TURN_SPEED,(-90 * turnDirection)-a, 4);
        robot.claw.setPosition(0.7);
        sleep(500);
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,-(10 + 2),4, P);

        a=robot.gyroTurn((145 * turnDirection)-a, TURN_SPEED/2.0, -TURN_SPEED, 4);
        robot.claw.setPosition(0.25);

        a=robot.fastEncoderStraight(DRIVE_SPEED,-(10),4, P);
        a=robot.gyroRotate(TURN_SPEED,(-55 * turnDirection)-a, 4);
        robot.claw.setPosition(0.7);

        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,-68,4, P);
        robot.dropBlock(100);

        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,12,4, P);
        // End funky stuff
        */

        //sleep(24000);

        /*
        // Yeah, the encoder for one revolution is 537.6.
        // Not 570 like we think documentation said.
        robot.encoderTest(0.2,546,5);
        robot.encoderTest(0.2,546,5);
        robot.encoderTest(0.2,546,5);
        robot.encoderTest(0.2,546,5);
        robot.encoderTest(0.2,538*10,50);
        */

        /*
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,12, 2, P );
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,12, 2, P );
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,12, 2, P );
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,12, 2, P );
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,-12, 2, P );
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,-12, 2, P );
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,-12, 2, P );
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,-12, 2, P );
        robot.grabBlock(1000);
        robot.dropBlock(500);
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,24, 2, P );
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,24, 2, P );
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,-24, 2, P );
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,-24, 2, P );
        robot.grabBlock(1000);
        robot.dropBlock(500);
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,12, 5, P );
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,36, 5, P );
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,-36, 5, P );
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,-12, 5, P );
        robot.grabBlock(1000);
        robot.dropBlock(500);
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,48, 5, P );
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,-48, 5, P );
        robot.grabBlock(1000);
        robot.dropBlock(500);
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,72, 5, P );
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,-72, 5, P );
        robot.grabBlock(1000);
        robot.dropBlock(500);
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,96, 5, P );
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,-96, 5, P );
        robot.grabBlock(1000);
        robot.dropBlock(500);
        sleep(10000);
        */

        /*
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,96, 5, P );
        robot.grabBlock(1000);
        robot.dropBlock(500);
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,-96, 5, P );
        robot.grabBlock(1000);
        robot.dropBlock(500);
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,96, 5, P );
        robot.grabBlock(1000);
        robot.dropBlock(500);
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,-96, 5, P );
        robot.grabBlock(1000);
        robot.dropBlock(500);
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,96, 5, P );
        robot.grabBlock(1000);
        robot.dropBlock(500);
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,-96, 5, P );
        robot.grabBlock(1000);
        robot.dropBlock(500);
        sleep(5000);
        */

        /*
        robot.straightA = a;
        a=robot.fastEncoderStrafe(DRIVE_SPEED, -48, 10, P);
        robot.straightA = a;
        a=robot.fastEncoderStrafe(DRIVE_SPEED, 48, 10, P);
        robot.straightA = a;
        a=robot.fastEncoderStrafe(DRIVE_SPEED, -24, 10, P);
        robot.straightA = a;
        a=robot.fastEncoderStrafe(DRIVE_SPEED, 24, 10, P);
        robot.straightA = a;
        a=robot.fastEncoderStrafe(DRIVE_SPEED, -12, 10, P);
        robot.straightA = a;
        a=robot.fastEncoderStrafe(DRIVE_SPEED, 12, 10, P);
        */

        //robot.fastEncoderStraight(DRIVE_SPEED, 96, 10,0.05);
        //sleep(1000);

        /*
        a=robot.gyroRotate(TURN_SPEED,90-a, 6);
        a=robot.gyroRotate(TURN_SPEED,90-a, 6);
        a=robot.gyroRotate(TURN_SPEED,180-a, 6);
        a=robot.gyroRotate(TURN_SPEED,180-a, 6);
        a=robot.gyroRotate(TURN_SPEED,360-a, 6);
        a=robot.gyroRotate(TURN_SPEED,-360-a, 6);
        a=robot.gyroRotate(TURN_SPEED,-180-a, 6);
        a=robot.gyroRotate(TURN_SPEED,-180-a, 6);
        a=robot.gyroRotate(TURN_SPEED,-90-a, 6);
        a=robot.gyroRotate(TURN_SPEED,-90-a, 6);
        */

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

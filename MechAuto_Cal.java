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
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Image;

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
@Disabled
public class MechAuto_Cal extends LinearOpMode {
    /* Declare OpMode members. */

    private MechBot          robot   = new MechBot();   // Use a Pushbot's hardware

    private static final double     DRIVE_SPEED             = 1.0;
    private static final double     TURN_SPEED              = 0.8;

    private IncepVision vision = new IncepVision();
    int rings;

    double a=0;
    double P=0.075;

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

        vision.initAutonomous(this,"LeftWebcam");
        vision.clip = false;
        int deltaX=0, deltaY=0;
        do {
            rings = vision.countRings();
            if ( gamepad1.dpad_left ) { IncepVision.clipLeft -= 1; }
            if ( gamepad1.dpad_right ) { IncepVision.clipLeft += 1; }
            if ( gamepad1.dpad_down ) { IncepVision.clipTop += 1; }
            if ( gamepad1.dpad_up ) { IncepVision.clipTop -= 1; }
            if ( gamepad1.x ) { IncepVision.clipRight += 1; }
            if ( gamepad1.b ) { IncepVision.clipRight -= 1; }
            if ( gamepad1.a ) { IncepVision.clipBottom -= 1; }
            if ( gamepad1.y ) { IncepVision.clipBottom += 1; }
            if ( gamepad1.left_bumper ) { vision.tfod.deactivate(); vision.tfodState=false;}
            if ( gamepad1.right_bumper ) { vision.clip = true; }

            // Move the entire box with the joystick.
            deltaY = (int)(gamepad1.left_stick_y * 2.1);
            deltaX = (int)(gamepad1.left_stick_x * 2.1);
            deltaY += (int)(gamepad1.right_stick_y * 2.1);
            deltaX += (int)(gamepad1.right_stick_x * 2.1);
            IncepVision.clipTop += deltaY;
            IncepVision.clipBottom -= deltaY;
            IncepVision.clipLeft += deltaX;
            IncepVision.clipRight -= deltaX;

            // Observe some limits
            IncepVision.clipLeft   = Range.clip(IncepVision.clipLeft,  5,635);
            IncepVision.clipRight  = Range.clip(IncepVision.clipRight, 5,635);
            IncepVision.clipTop    = Range.clip(IncepVision.clipTop,   5,475);
            IncepVision.clipBottom = Range.clip(IncepVision.clipBottom,5,475);
        } while (!isStarted() && (!isStopRequested())) ;
        vision.shutdown();

        /*
        while (!isStarted() && (!isStopRequested())) {
            sleep(250);
            telemetry.addData("Heading", "%f", robot.getHeading());
            telemetry.update();
        }
        */

        //robot.MotorCal(robot.leftFDrive, "lf", 1.0 );
        //robot.MotorCal(robot.leftBDrive, "lb",1.0 );
        //robot.MotorCal(robot.rightFDrive, "rf", 1.0 );
        //robot.MotorCal(robot.rightBDrive, "rb", 1.0 );

        robot.straightA = a;

        // Test strafe in here
        /*
        P=0.075;
        a=robot.fastEncoderStrafe(DRIVE_SPEED,48, 60, P );
        robot.straightA = a;
        a=robot.fastEncoderStrafe(DRIVE_SPEED,-48, 60, P);
        robot.straightA = a;
        a=robot.fastEncoderStrafe(DRIVE_SPEED,48, 60, P );
        robot.straightA = a;
        a=robot.fastEncoderStrafe(DRIVE_SPEED,-48, 60, P );
        */

        /*
        double distance=16.0;
        P=0.075;
        // Now a bunch of junk to try different movements like short, medium, long, etc.
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,distance,60, P);
        sleep(2000);
        //a=robot.gyroRotate(TURN_SPEED,(180)-a, 60);
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,-distance,60, P);
        sleep(2000);
        //a=robot.gyroRotate(TURN_SPEED,(-180)-a, 60);
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,distance,60, P);
        sleep(2000);
        //a=robot.gyroRotate(TURN_SPEED,(180)-a, 60);
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,-distance,60, P);
        sleep(2000);

        distance = 30.0;
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,distance,60, P);
        sleep(2000);
        //a=robot.gyroRotate(TURN_SPEED,(180)-a, 60);
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,-distance,60, P);
        sleep(2000);
        //a=robot.gyroRotate(TURN_SPEED,(-180)-a, 60);
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,distance,60, P);
        sleep(2000);
        //a=robot.gyroRotate(TURN_SPEED,(180)-a, 60);
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,-distance,60, P);
        sleep(2000);

        distance = 72.0;
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,distance,60, P);
        sleep(2000);
        //a=robot.gyroRotate(TURN_SPEED,(180)-a, 60);
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,-distance,60, P);
        sleep(2000);
        //a=robot.gyroRotate(TURN_SPEED,(-180)-a, 60);
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,distance,60, P);
        sleep(2000);
        //a=robot.gyroRotate(TURN_SPEED,(180)-a, 60);
        robot.straightA = a;
        a=robot.fastEncoderStraight(DRIVE_SPEED,-distance,60, P);
        sleep(2000);
        */

        /*
        a=robot.gyroRotate(TURN_SPEED,5*360.0, 60);
        sleep(3000);
        a=robot.gyroRotate(TURN_SPEED,-5*360.0, 60);
        sleep(3000);
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

        robot.logger.logD("MechLog","Done");
        robot.logger.logD("MechLog","  ");
        robot.logger.logD("MechLog","  ");

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}

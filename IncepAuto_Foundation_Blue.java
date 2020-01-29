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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Autonomous(name="Incep: Auto Foundation Blue", group="IncepBot")
@Disabled
public class IncepAuto_Foundation_Blue extends LinearOpMode {

    /* Declare OpMode members. */
    private IncepBot         robot   = new IncepBot();   // Use a Pushbot's hardware

    private static final double     DRIVE_SPEED             = 0.9;
    private static final double     TURN_SPEED              = 0.6;
    private static final double     PIVOT_SPEED             = 0.75;
    private static final double     SQ                      = 70/3.0;        // Length of 3 squares / 3 in case we want to think that way
    private String className = this.getClass().getSimpleName().toLowerCase();

    private double a=0;
    private double start_a=0;

    @Override
    public void runOpMode() {
        double turnDirection;
        double d[] = {0.3, 0.4};

        // Red or blue alliance -- only difference is the turn direction
        if (className.contains("blue")) {
            turnDirection = 1.0;
        } else {
            turnDirection = -1.0;
        }

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.initAutonomous(this);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Capture the starting angle (probably close to 0.0)
        start_a = robot.getHeading();

        // Init servos
        robot.releaseFoundation(0);

        // Move off the wall
        a = robot.fastEncoderStraight(DRIVE_SPEED,-14,2);

        // Turn left
        a = robot.gyroRotate(TURN_SPEED, (turnDirection * 90)-a,2.5);

        // Offset to middle of foundation
        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED,-18,2);

        // Turn back
        a = robot.gyroRotate(TURN_SPEED, (turnDirection * -90)-a,2.5);

        // Drive to Foundation
        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED,-19,2);

        // Hook foundation
        robot.grabFoundation(1000);

        // Grab foundation a ways
        robot.straightA = a;
        a = robot.fastEncoderDrive(DRIVE_SPEED,15, 15, 3, 0.05, d, d );

        // Rotate foundation a little more than 90 degrees
        a = robot.gyroPivot(PIVOT_SPEED, (turnDirection * 110)-a, 10);

        // Push foundation back against the wall, also aligns the bot
        robot.fastEncoderDrive(DRIVE_SPEED,-22, -22,3, 0.0, d, d );

        // Release
        robot.releaseFoundation(250);

        // Separate from foundation to allow som rotating
        robot.fastEncoderDrive(DRIVE_SPEED,3, 3, 1, 0.05, d, d );

        // Now rotate to about +/-85 degrees to hug the wall
        a = robot.getHeading();
        robot.gyroRotate(TURN_SPEED, ((87 * turnDirection) - a) + start_a, 2);

        // Drive to the bridge along the wall
        robot.fastEncoderStraight(DRIVE_SPEED,40,15, 0.05);

        // Extend if we overshoot
        robot.grabBlock();

        /*

        robot.releaseFoundation(0);

        robot.fastEncoderStraight(DRIVE_SPEED,-17,2);
        robot.fastEncoderRotate(TURN_SPEED,turnDirection * -90,2.5);
        robot.fastEncoderStraight(DRIVE_SPEED,-18,2);
        robot.fastEncoderRotate(TURN_SPEED,turnDirection * 90,2.5);
        robot.fastEncoderStraight(DRIVE_SPEED,-19,2.5);

        robot.grabFoundation(1000);

        robot.fastEncoderStraight(DRIVE_SPEED,20,3, 0.05);

        robot.gyroPivot(PIVOT_SPEED, turnDirection * 110, 6);
        //robot.encoderArc(PIVOT_SPEED, 90*FDC, IncepBot.RIGHT, 0, 5);

        robot.fastEncoderStraight(DRIVE_SPEED,-22,2);
        robot.releaseFoundation(750);
        robot.fastEncoderRotate(TURN_SPEED, turnDirection * 10, 1);
        robot.fastEncoderStraight(DRIVE_SPEED,39,16, 0.05);

         */
    }
}

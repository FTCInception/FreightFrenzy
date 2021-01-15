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

@Autonomous(name="Mech: Auto Ring Red", group="MechBot")
public class MechAuto_Ring_Red extends LinearOpMode {

    private MechBot robot = new MechBot();   // Use a Pushbot's hardware

    private static final double DRIVE_SPEED = 0.8;
    private static final double TURN_SPEED = 0.65;
    private static final double PIVOT_SPEED = 0.40;
    private static final double SQ = 70 / 3.0;        // Length of 3 squares / 3 in case we want to think that way
    private static final double[] blocks = {0.0, 28.0, 36.0, 44.0, 4.0, 12.0, 20.0};
    private static final double dropZone = 82.0;
    private static final double bridge = 71.0;
    private String className = this.getClass().getSimpleName().toLowerCase();

    //private IncepVision vision = new IncepVision();
    private int block;
    private double P = 0.05;
    private double a = 0;

    @Override
    public void runOpMode() {
        double firstBlock, secondBlock, thirdBlock, fourthBlock;
        double turnDirection;
        double laneLength;
        double laneLengthWall;
        double laneAdjust1;
        double laneAdjust2;
        int    ideals[]={0, 412, 196, 152};

        //robot.logger.LOGLEVEL = robot.logger.LOGDEBUG ;

        // Init the robot and subsystems
        robot.init(hardwareMap);
        robot.initAutonomous(this);
        //vision.initAutonomous(this);

        // Wait until we're told to go and look for the block

        /*
        while (!isStarted()) {
            block = vision.getBlockNumber(ideals);
        }
        vision.shutdown();
        */

        // Red or blue alliance changes include:
        // Correction for block numbering
        // Turn direction

        // Wait for the game to start (driver presses PLAY)
        //waitForStart();

        int goalPos = 0;

        //just for testing purposes, use before vision is finished
        while (!isStarted()) {
           if(gamepad1.dpad_up)
               goalPos = 1;
            if(gamepad1.dpad_left)
                goalPos = 2;
            if(gamepad1.dpad_down)
                goalPos = 3;
            if(gamepad1.dpad_right)
                goalPos = 4;
        }

        if (goalPos == 1) {
            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, 6, 1, P);
            robot.straightA = a;
            a = robot.fastEncoderStrafe(DRIVE_SPEED, -6, 1.75);
            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, 50, 4, P);

            robot.intake_motor.setPower(0.2);
            sleep(400);
            robot.intake_motor.setPower(0);

            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, -3, 1, P);
            robot.straightA = a;
            a = robot.fastEncoderStrafe(DRIVE_SPEED, 30, 6, P);

            sleep(3000);

            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, 15, 1, P);

        } else if (goalPos == 2){
            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, 56, 4, P);
            robot.straightA = a;
            a = robot.fastEncoderStrafe(DRIVE_SPEED, 18, 1.75);
            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, 24, 4, P);

            robot.intake_motor.setPower(0.2);
            sleep(400);
            robot.intake_motor.setPower(0);

            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, -20, 2, P);

            sleep(3000);

            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, 5, 1, P);
        } else if (goalPos == 3){
            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, 6, 1, P);
            robot.straightA = a;
            a = robot.fastEncoderStrafe(DRIVE_SPEED, -6, 1.75);
            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, 96, 4, P);

            robot.intake_motor.setPower(0.2);
            sleep(400);
            robot.intake_motor.setPower(0);

            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, -50, 3, P);
            robot.straightA = a;
            a = robot.fastEncoderStrafe(DRIVE_SPEED, 30, 6, P);

            sleep(3000);

            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, 15, 1, P);
        } else if (goalPos==4){
            robot.claw.setPosition(0);
            sleep(500);
             robot.setWobblePosition(1,.6);
            sleep(2000);
            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, -10, 3, P);
            robot.claw.setPosition(1);
            sleep(2000);
            robot.setWobblePosition(2,.6);
            sleep(2000);
        }


    }
}

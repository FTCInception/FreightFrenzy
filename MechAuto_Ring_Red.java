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

    private static final double DRIVE_SPEED = 1.0;
    private static final double TURN_SPEED = 0.8;
    private static final double wobble_power = 0.6;
    private String className = this.getClass().getSimpleName().toLowerCase();

    private IncepVision vision = new IncepVision();
    private int block;
    private double P = 0.075;
    private double a = 0;
    private int ringCount = -1;

    @Override
    public void runOpMode() {

        //robot.logger.LOGLEVEL = robot.logger.LOGDEBUG ;

        // Init the robot and subsystems
        robot.init(hardwareMap);
        robot.initAutonomous(this);
        vision.initAutonomous(this);

        // Wait until we're told to go and look for the block

        /*
        while (!isStarted()) {
            ringCount = vision.ringCount();
        }
        vision.shutdown();

        if (rings == 0) {
            goalPos = 1;
        } else if (rings == 1) {
            goalPos = 2;
        } else if (rings == 4) {
            goalPos = 3;
        else {
            goalPos = 3;
        }
        */

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

            telemetry.addData("Current Auto: ",  goalPos);
            telemetry.update();
        }

        if (goalPos == 1) {
            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, 56, 60, P);

            robot.intake_motor.setPower(0.4);
            sleep(400);
            robot.intake_motor.setPower(0);

            robot.straightA = a;
            a = robot.fastEncoderStrafe(DRIVE_SPEED, 24, 60, P);

            // We had a strange twitch in the previous strafe, this delay was added to see if it
            // made any difference.  May not be necessary.
            sleep(500);

            // Shoot sequence
            robot.shoot1_motor.setPower(0.475);
            robot.shoot2_motor.setPower(0.475);
            sleep(500);
            robot.flicker.setPosition(1.0);
            sleep(500);
            robot.flicker.setPosition(0.0);
            sleep(750);
            robot.flicker.setPosition(1.0);
            sleep(500);
            robot.flicker.setPosition(0.0);
            sleep(750);
            robot.flicker.setPosition(1.0);
            sleep(500);
            robot.shoot1_motor.setPower(0.0);
            robot.shoot2_motor.setPower(0.0);
            robot.flicker.setPosition(0.0);

            //prime some stuff for wobble
            robot.claw.setPosition(0);
            sleep(500);
            robot.setWobblePosition(1,wobble_power);
            sleep(2000);

            robot.straightA = a;
            a = robot.fastEncoderStrafe(DRIVE_SPEED, 12, 60, P);
            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, -32, 60, P);

            robot.claw.setPosition(1);
            sleep(1000);
            robot.setWobblePosition(3,wobble_power);
            sleep(1000);

            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, 46, 60, P);
            a = robot.gyroRotate(TURN_SPEED,90-a, 4);
            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, -20, 60, P);

            robot.claw.setPosition(0);
            sleep(500);
            robot.setWobblePosition(0,wobble_power);
            sleep(2000);


        } else if (goalPos == 2){
            // Drive to drop zone
            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, 87, 60, P);
            robot.straightA = a;
            a = robot.fastEncoderStrafe(DRIVE_SPEED, 22, 60, P);

            // Drop the wobble
            robot.intake_motor.setPower(0.4);
            sleep(600);

            // Turn off intake
            robot.intake_motor.setPower(0);

            // Back to shooting zone
            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, -34, 60, P);

            // Start the shooter
            robot.shoot1_motor.setPower(0.475);
            robot.shoot2_motor.setPower(0.475);

            // Line up
            robot.straightA = a;
            a = robot.fastEncoderStrafe(DRIVE_SPEED, 4, 60, P);

            // Fire away
            robot.flicker.setPosition(1.0);
            sleep(300);
            robot.flicker.setPosition(0.0);
            sleep(650);
            robot.flicker.setPosition(1.0);
            sleep(300);
            // Get ready for wobble
            robot.claw.setPosition(0.25);
            robot.setWobblePosition(1,wobble_power);
            robot.flicker.setPosition(0.0);
            sleep(650);
            robot.flicker.setPosition(1.0);
            sleep(100);
            robot.shoot1_motor.setPower(0.0);
            robot.shoot2_motor.setPower(0.0);

            // Line up for wobble
            robot.straightA = a;
            a = robot.fastEncoderStrafe(DRIVE_SPEED, 11, 60, P);
            robot.flicker.setPosition(0.0);

            // Back up to wobble
            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, -29, 60, P);

            // Grab wobble
            robot.claw.setPosition(1);
            sleep(750);
            robot.setWobblePosition(3,wobble_power);
            sleep(500);

            // Strafe to line up on ring
            robot.straightA = a;
            a = robot.fastEncoderStrafe(DRIVE_SPEED, -18, 60, P);

            // Turn on intake
            robot.intake_motor.setPower(1.0);

            // Intake ring and drive to shoot distance
            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, 29, 60, P);

            // Line up for shot
            robot.straightA = a;
            a = robot.fastEncoderStrafe(DRIVE_SPEED, 7, 60, P);

            // Turn off intake
            robot.intake_motor.setPower(1.0);

            // Shoot the ring
            robot.intake_motor.setPower(0);
            robot.shoot1_motor.setPower(0.475);
            robot.shoot2_motor.setPower(0.475);
            sleep(500);
            robot.flicker.setPosition(1.0);
            sleep(300);
            robot.flicker.setPosition(0.0);
            robot.shoot1_motor.setPower(0.0);
            robot.shoot2_motor.setPower(0.0);

            a = robot.gyroRotate(TURN_SPEED,165-a, 60);

            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, -22, 60, P);

            robot.claw.setPosition(0.0);
            sleep(500);
            robot.setWobblePosition(0,wobble_power);
            sleep(1000);
            robot.claw.setPosition(1.0);
            sleep(1000);

        } else if (goalPos == 3){
            // Full speed length of field to the target zone
            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, 112, 60, P);

            // Drop the wobble and run
            robot.intake_motor.setPower(0.4);

            // Back up to a good shooting distance
            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, -58, 60, P);

            // Turn off intake now
            robot.intake_motor.setPower(0);

            // Move to line up shot
            robot.straightA = a;
            a = robot.fastEncoderStrafe(DRIVE_SPEED, 28, 60, P);

            // Fire away
            robot.shoot1_motor.setPower(0.475);
            robot.shoot2_motor.setPower(0.475);
            sleep(500);
            robot.flicker.setPosition(1.0);
            sleep(300);
            robot.flicker.setPosition(0.0);
            sleep(650);
            robot.flicker.setPosition(1.0);
            sleep(300);
            robot.claw.setPosition(0.25);
            robot.setWobblePosition(1,wobble_power);
            robot.flicker.setPosition(0.0);
            sleep(650);
            robot.flicker.setPosition(1.0);
            sleep(100);
            robot.shoot1_motor.setPower(0.0);
            robot.shoot2_motor.setPower(0.0);

            // Line up for wobble pickup
            robot.straightA = a;
            a = robot.fastEncoderStrafe(DRIVE_SPEED, 9, 60, P);
            robot.flicker.setPosition(0.0);

            // Back to wobble
            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, -30, 60, P);

            // Grab the wobble
            robot.claw.setPosition(1);
            sleep(750);
            robot.setWobblePosition(3,wobble_power);
            sleep(500);

            // Line up over 4-stack
            robot.straightA = a;
            a = robot.fastEncoderStrafe(DRIVE_SPEED, -18, 60, P);

            // Get ready to intake and shoot
            robot.intake_motor.setPower(1.0);

            robot.shoot1_motor.setPower(0.475);
            robot.shoot2_motor.setPower(0.475);

            // Go intake a few and shoot one
            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, 17.0, 60, P);

            // Turn a little towards the goal
            a = robot.gyroRotate(.8,5-a, 4);

            // Wait for the rings to settle
            sleep(500);
            robot.flicker.setPosition(1.0);

            // Turn a little back
            a = robot.gyroRotate(TURN_SPEED,-5-a, 4);
            robot.flicker.setPosition(0.0);

            // Intake the reset and shoot them
            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, 17.0, 60, P);
            robot.flicker.setPosition(1.0);
            sleep(650);
            robot.flicker.setPosition(0.0);
            sleep(350);
            robot.flicker.setPosition(1.0);
            sleep(650);
            robot.flicker.setPosition(0.0);
            sleep(350);
            robot.flicker.setPosition(1.0);
            sleep(250);

            // Stop the shooter
            robot.shoot1_motor.setPower(0.0);
            robot.shoot2_motor.setPower(0.0);

            // Stop the intake
            robot.intake_motor.setPower(0);

            // Turn around and back the wobble in
            a = robot.gyroRotate(TURN_SPEED,157-a, 4);
            robot.flicker.setPosition(0.0);

            // Drive the wobble in
            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, -40.0, 60, P);

            // Drop it off
            robot.claw.setPosition(0);
            sleep(250);
            robot.setWobblePosition(0,wobble_power);

            // Run like heck
            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, 24.0, 60, P);

        } else if (goalPos==4){
            robot.claw.setPosition(0);
            sleep(500);
             robot.setWobblePosition(1,wobble_power);
            sleep(2000);
            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, -10, 60, P);
            robot.claw.setPosition(1);
            sleep(2000);
            robot.setWobblePosition(2,wobble_power);
            sleep(2000);
        }


    }
}

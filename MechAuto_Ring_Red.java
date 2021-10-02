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

package Inception.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

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
@Disabled
public class MechAuto_Ring_Red extends LinearOpMode {

    private MechBot robot = new MechBot();   // Use a Pushbot's hardware

    private static final double DRIVE_SPEED = 1.0;
    private static final double TURN_SPEED = 0.8;
    private static final double wobble_power = 0.6;
    private String className = this.getClass().getSimpleName().toLowerCase();

    private static final double intake_eject_wobble = 0.6;
    private static final double intake_pickup_ring = 1.0;

    private IncepVision vision = new IncepVision();
    private int block;
    private double P = 0.075;
    private double a = 0;
    private int ringCount = -1;

    public static void gotHere(boolean running, double lineNumber, MechBot robot) {
        if(running) {
            robot.logger.logD("gotHere", "%.1f: running", lineNumber);
        } else {
            robot.logger.logD("gotHere", "%.1f: stopped", lineNumber);
        }
    }

    @Override
    public void runOpMode() {

        //robot.logger.LOGLEVEL = robot.logger.LOGDEBUG ;

        // Init the robot and subsystems
        robot.init(hardwareMap);
        robot.initAutonomous(this);

        // Stare at the rings really hard until its time to go or stop
        vision.initAutonomous(this,"LeftWebcam");
        vision.clip = false;
        int deltaX=0, deltaY=0;
        do {
            ringCount = vision.countRings();
            if ( gamepad1.dpad_left ) { IncepVision.clipLeft -= 1; }
            if ( gamepad1.dpad_right ) { IncepVision.clipLeft += 1; }
            if ( gamepad1.dpad_down ) { IncepVision.clipTop += 1; }
            if ( gamepad1.dpad_up ) { IncepVision.clipTop -= 1; }
            if ( gamepad1.x ) { IncepVision.clipRight += 1; }
            if ( gamepad1.b ) { IncepVision.clipRight -= 1; }
            if ( gamepad1.a ) { IncepVision.clipBottom -= 1; }
            if ( gamepad1.y ) { IncepVision.clipBottom += 1; }
            if ( gamepad1.left_bumper ) { vision.tfod.deactivate(); vision.tfodState=false;}
            if ( (gamepad1.right_bumper) && (!vision.tfodState) ) { vision.clip = true; }

            if ( gamepad2.dpad_left ) { IncepVision.clipLeft -= 1; }
            if ( gamepad2.dpad_right ) { IncepVision.clipLeft += 1; }
            if ( gamepad2.dpad_down ) { IncepVision.clipTop += 1; }
            if ( gamepad2.dpad_up ) { IncepVision.clipTop -= 1; }
            if ( gamepad2.x ) { IncepVision.clipRight += 1; }
            if ( gamepad2.b ) { IncepVision.clipRight -= 1; }
            if ( gamepad2.a ) { IncepVision.clipBottom -= 1; }
            if ( gamepad2.y ) { IncepVision.clipBottom += 1; }
            if ( gamepad2.left_bumper ) { vision.tfod.deactivate(); vision.tfodState=false;}
            if ( (gamepad2.right_bumper) && (!vision.tfodState) ) { vision.clip = true; }

            // Move the entire box with the joystick.
            deltaY = (int)(gamepad1.left_stick_y * 2.1);
            deltaX = (int)(gamepad1.left_stick_x * 2.1);
            deltaY += (int)(gamepad1.right_stick_y * 2.1);
            deltaX += (int)(gamepad1.right_stick_x * 2.1);
            IncepVision.clipTop += deltaY;
            IncepVision.clipBottom -= deltaY;
            IncepVision.clipLeft += deltaX;
            IncepVision.clipRight -= deltaX;

            // Move the entire box with the joystick.
            deltaY += (int)(gamepad2.left_stick_y * 2.1);
            deltaX += (int)(gamepad2.left_stick_x * 2.1);
            deltaY += (int)(gamepad2.right_stick_y * 2.1);
            deltaX += (int)(gamepad2.right_stick_x * 2.1);
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

        // FIXME: Test the stopping in auto here again.
        if (opModeIsActive()) {
            if (ringCount == 0) {
                Ring0();
            } else if (ringCount == 1) {
                Ring1();
            } else {
                // If it's not 0 or 1, assume 4 (highest possible point total)
                Ring4();
            }
        }
    }

    private void Ring0() {

        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED, 57, 60, P);
        if(!opModeIsActive()){ return; }

        robot.intakeEjectWobble(intake_eject_wobble);
        sleep(600);
        if(!opModeIsActive()){ return; }

        robot.intakeStop();
        if(!opModeIsActive()){ return; }

        // Start shooter 1 move early
        robot.shoot1_motor.setPower(0.480);
        robot.shoot2_motor.setPower(0.480);

        // Strafe to shooting position
        robot.straightA = a;
        a = robot.fastEncoderStrafe(DRIVE_SPEED, 24, 60, P);
        if(!opModeIsActive()){ return; }

        // Shoot sequence
        sleep(500);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.shoot1_motor.setPower(0.475);
        robot.shoot2_motor.setPower(0.475);

        robot.flicker.setPosition(0.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(500);
        if(!opModeIsActive()){ return; }

        robot.shoot1_motor.setPower(0.0);
        robot.shoot2_motor.setPower(0.0);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        if(!opModeIsActive()){ return; }

        //prime some stuff for wobble
        robot.claw.setPosition(0);
        sleep(500);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(1, wobble_power);
        sleep(2000);
        if(!opModeIsActive()){ return; }

        robot.straightA = a;
        a = robot.fastEncoderStrafe(DRIVE_SPEED, 13, 60, P);
        if(!opModeIsActive()){ return; }

        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED, -32, 60, P);
        if(!opModeIsActive()){ return; }

        robot.claw.setPosition(1);
        sleep(1000);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(3, wobble_power);
        sleep(1000);
        if(!opModeIsActive()){ return; }

        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED, 50, 60, P);
        if(!opModeIsActive()){ return; }

        a = robot.gyroRotate(TURN_SPEED, 90 - a, 4);
        if(!opModeIsActive()){ return; }

        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED, -21, 60, P);
        if(!opModeIsActive()){ return; }

        robot.claw.setPosition(0);
        sleep(500);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(0, wobble_power);
        sleep(500);
        if(!opModeIsActive()){ return; }

        robot.claw.setPosition(1.0);
        if(!opModeIsActive()){ return; }

        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED, 11, 60, P);
        sleep(1000);
        if(!opModeIsActive()){ return; }
    }

    private void Ring1() {

        // Drive to drop zone
        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED, 87, 60, P);
        if(!opModeIsActive()){ return; }

        robot.straightA = a;
        a = robot.fastEncoderStrafe(DRIVE_SPEED, 24, 60, P);
        if(!opModeIsActive()){ return; }

        // Drop the wobble
        robot.intakeEjectWobble(intake_eject_wobble);
        sleep(600);
        if(!opModeIsActive()){ return; }

        robot.intakeStop();
        if(!opModeIsActive()){ return; }

        // Start the shooter early
        robot.shoot1_motor.setPower(0.480);
        robot.shoot2_motor.setPower(0.480);
        if(!opModeIsActive()){ return; }

        // Back to shooting zone
        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED, -30, 60, P);
        if(!opModeIsActive()){ return; }

        sleep(500);
        if(!opModeIsActive()){ return; }

        // Fire away
        robot.flicker.setPosition(1.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.shoot1_motor.setPower(0.475);
        robot.shoot2_motor.setPower(0.475);

        robot.flicker.setPosition(0.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        // Get ready for wobble
        robot.claw.setPosition(0.25);
        robot.setWobblePosition(1, wobble_power);
        robot.flicker.setPosition(0.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(200);
        if(!opModeIsActive()){ return; }

        // Line up for wobble
        robot.straightA = a;
        a = robot.fastEncoderStrafe(DRIVE_SPEED, 13, 60, P);
        if(!opModeIsActive()){ return; }

        robot.shoot1_motor.setPower(0.0);
        robot.shoot2_motor.setPower(0.0);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        if(!opModeIsActive()){ return; }

        // Back up to wobble
        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED, -32, 60, P);
        if(!opModeIsActive()){ return; }

        // Grab wobble
        robot.claw.setPosition(1);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(3, wobble_power);
        sleep(500);
        if(!opModeIsActive()){ return; }

        // Strafe to line up on ring
        robot.straightA = a;
        a = robot.fastEncoderStrafe(DRIVE_SPEED, -18, 60, P);
        if(!opModeIsActive()){ return; }

        // Turn on intake
        robot.intakePickupRing(intake_pickup_ring);
        if(!opModeIsActive()){ return; }

        // Intake ring and drive to shoot distance
        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED, 29, 60, P);
        if(!opModeIsActive()){ return; }

        // Power up early
        robot.shoot1_motor.setPower(0.480);
        robot.shoot2_motor.setPower(0.480);
        if(!opModeIsActive()){ return; }

        // Line up for shot
        robot.straightA = a;
        a = robot.fastEncoderStrafe(DRIVE_SPEED, 5, 60, P);
        if(!opModeIsActive()){ return; }

        // Shoot the ring
        robot.intakeStop();
        sleep(500);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(400);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        robot.shoot1_motor.setPower(0.0);
        robot.shoot2_motor.setPower(0.0);
        if(!opModeIsActive()){ return; }

        a = robot.gyroRotate(TURN_SPEED, 170 - a, 60);
        if(!opModeIsActive()){ return; }

        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED, -24, 60, P);
        if(!opModeIsActive()){ return; }

        robot.claw.setPosition(0.0);
        sleep(500);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(0, wobble_power);
        sleep(1000);
        if(!opModeIsActive()){ return; }

        robot.claw.setPosition(1.0);
        sleep(1000);
        if(!opModeIsActive()){ return; }
    }

    private void Ring4() {

        // Full speed length of field to the target zone
        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED, 115, 60, P);
        if(!opModeIsActive()){ return; }

        // Drop the wobble and run
        robot.intakeEjectWobble(intake_eject_wobble);
        if(!opModeIsActive()){ return; }

        // Back up to a good shooting distance
        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED, -58, 60, P);
        if(!opModeIsActive()){ return; }

        // Turn off intake now
        robot.intakeStop();
        if(!opModeIsActive()){ return; }

        // Power up early
        robot.shoot1_motor.setPower(0.475);
        robot.shoot2_motor.setPower(0.475);
        if(!opModeIsActive()){ return; }

        // Move to line up shot
        robot.straightA = a;
        a = robot.fastEncoderStrafe(DRIVE_SPEED, 24, 60, P);
        if(!opModeIsActive()){ return; }

        // Fire away
        sleep(500);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(500);
        if(!opModeIsActive()){ return; }

        robot.shoot1_motor.setPower(0.475);
        robot.shoot2_motor.setPower(0.475);

        robot.flicker.setPosition(0.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(500);
        if(!opModeIsActive()){ return; }

        robot.claw.setPosition(0.25);
        robot.setWobblePosition(1, wobble_power);
        robot.flicker.setPosition(0.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(300);
        if(!opModeIsActive()){ return; }

        // Line up for wobble pickup
        robot.straightA = a;
        a = robot.fastEncoderStrafe(DRIVE_SPEED, 13, 60, P);
        if(!opModeIsActive()){ return; }

        robot.shoot1_motor.setPower(0.0);
        robot.shoot2_motor.setPower(0.0);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        if(!opModeIsActive()){ return; }

        // Back to wobble
        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED, -32, 60, P);
        if(!opModeIsActive()){ return; }

        // Grab the wobble
        robot.claw.setPosition(1);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(3, wobble_power);
        sleep(500);
        if(!opModeIsActive()){ return; }

        // Line up over 4-stack
        robot.straightA = a;
        a = robot.fastEncoderStrafe(DRIVE_SPEED, -18, 60, P);
        if(!opModeIsActive()){ return; }

        // Get ready to intake and shoot
        robot.intakePickupRing(intake_pickup_ring);
        if(!opModeIsActive()){ return; }

        // Power up early
        robot.shoot1_motor.setPower(0.475);
        robot.shoot2_motor.setPower(0.475);
        if(!opModeIsActive()){ return; }

        // Go intake a few and shoot one
        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED, 15, 60, P);
        if(!opModeIsActive()){ return; }

        // Turn a little towards the goal
        a = robot.gyroRotate(.8, 5 - a, 60);
        if(!opModeIsActive()){ return; }

        // Wait for the rings to settle
        sleep(500);
        robot.flicker.setPosition(1.0);
        sleep(200);
        if(!opModeIsActive()){ return; }

        robot.shoot1_motor.setPower(0.475);
        robot.shoot2_motor.setPower(0.475);

        // Turn a little back
        a = robot.gyroRotate(TURN_SPEED, -4 - a, 60);
        robot.flicker.setPosition(0.0);
        if(!opModeIsActive()){ return; }

        // Intake the reset and shoot them
        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED, 14, 60, P);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(500);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(500);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(300);
        if(!opModeIsActive()){ return; }

        // Stop the shooter
        robot.shoot1_motor.setPower(0.0);
        robot.shoot2_motor.setPower(0.0);
        if(!opModeIsActive()){ return; }

        // Stop the intake
        robot.intakeStop();
        if(!opModeIsActive()){ return; }

        // Turn around and back the wobble in
        a = robot.gyroRotate(TURN_SPEED, 155 - a, 60);
        robot.flicker.setPosition(0.0);
        if(!opModeIsActive()){ return; }

        // Drive the wobble in
        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED, -49.0, 60, P);
        if(!opModeIsActive()){ return; }

        // Drop it off
        robot.claw.setPosition(0);
        sleep(250);
        robot.setWobblePosition(0, wobble_power);
        if(!opModeIsActive()){ return; }

        // Run like heck
        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED, 28.0, 60, P);
        if(!opModeIsActive()){ return; }

        robot.claw.setPosition(1.0);
        sleep(500);
        if(!opModeIsActive()){ return; }
    }
}

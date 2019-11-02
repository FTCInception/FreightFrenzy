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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Refbot: Auto Drive By Encoder", group="Refbot")
public class RefbotAutoDriveByEncoder_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    Refbot robot   = new Refbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;         // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 10.0/11.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.54331 ;       // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.14159);
    static final double     AXLE_LENGTH             = 13.15;        // Width of robot through the pivot point (center wheels)
    static final double     INCHES_PER_DEGREE       = (AXLE_LENGTH * 3.14159) / 360.0;
    static final double     DRIVE_SPEED             = 0.7;
    static final double     TURN_SPEED              = 0.35;
    static final double     SECONDS_TO_FULL_POWER   = 1.5;
    static final double     SPEED_RAMP_PER_MS       = (1.0/1000.0) / SECONDS_TO_FULL_POWER;
    static final double     SPEED_OFFS              = 0.05;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at LF:%7d, RF:%7d, LB:%7d, RB:%7d",
                          robot.leftFDrive.getCurrentPosition(),
                          robot.rightFDrive.getCurrentPosition(),
                          robot.leftBDrive.getCurrentPosition(),
                          robot.rightBDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  72,  72, 10.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderRotate(TURN_SPEED,  360*5, 30.0);  // S2: Turn Right 3 rotations
        //encoderDrive(DRIVE_SPEED,  72,  72, 10.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(DRIVE_SPEED,  -48,  -48, 10.0);  // S1: Forward 47 Inches with 5 Sec timeout

        //encoderDrive(TURN_SPEED,   123, -123, 30.0);  // S2: Turn Right 12 Inches with 4 Sec timeout

        //encoderDrive(DRIVE_SPEED, -48, -48, 10.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        //robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
        //robot.rightClaw.setPosition(0.0);
        //sleep(10000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void encoderRotate(double speed,
                              double degrees,
                              double timeoutS) {
        encoderDrive( speed, degrees * INCHES_PER_DEGREE, -degrees * INCHES_PER_DEGREE, timeoutS);

    }


        /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFTarget;
        int newRightFTarget;
        int newLeftBTarget;
        int newRightBTarget;
        double curPos;
        double tgtPos;
        double lastPos=0;
        double toGo;
        double actSpeed=0.0;
        double lt;
        double rem;
        double rate;
        boolean decel = false;
        ElapsedTime     looptime = new ElapsedTime();


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.leftFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftFTarget = (int)(leftInches * COUNTS_PER_INCH);
            newRightFTarget = (int)(rightInches * COUNTS_PER_INCH);
            newLeftBTarget = (int)(leftInches * COUNTS_PER_INCH);
            newRightBTarget = (int)(rightInches * COUNTS_PER_INCH);
            tgtPos = Math.abs(newLeftFTarget);
            robot.leftFDrive.setTargetPosition(newLeftFTarget);
            robot.rightFDrive.setTargetPosition(newRightFTarget);
            robot.leftBDrive.setTargetPosition(newLeftBTarget);
            robot.rightBDrive.setTargetPosition(newRightBTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion at the minimum speed.
            runtime.reset();
            speed = Math.abs(speed);
            actSpeed = Math.min(speed, SPEED_OFFS);
            robot.leftFDrive.setPower(actSpeed);
            robot.rightFDrive.setPower(actSpeed*(67.5/70.0));
            robot.leftBDrive.setPower(actSpeed);
            robot.rightBDrive.setPower(actSpeed*(67.5/70.0));
            looptime.reset();

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.leftFDrive.isBusy() && robot.rightFDrive.isBusy() && robot.leftBDrive.isBusy() && robot.rightBDrive.isBusy())) {

                // This code implements a soft start and soft stop.  Turns still seem to have some issue.
                // Record elapsed time
                lt = looptime.milliseconds();
                // Reset the timer for next loop
                looptime.reset();
                // Get current position
                curPos = Math.abs(robot.leftFDrive.getCurrentPosition());
                // How much farther?
                toGo = tgtPos - curPos;
                // Compute remaining time based on encoder rate from last loop
                rate = ((curPos - lastPos ) / lt);
                rem = toGo / rate;
                // Save the new position
                lastPos = curPos;
                // If the delta to our target stop speed is > than our deceleration rate, start to slow down
                if ( (actSpeed > 0.20) && (( actSpeed - 0.20) > (rem * SPEED_RAMP_PER_MS)) ) {
                    actSpeed = Math.max(actSpeed - (lt * SPEED_RAMP_PER_MS), 0.20);
                    robot.leftFDrive.setPower(actSpeed);
                    robot.rightFDrive.setPower(actSpeed*(67.5/70.0));
                    robot.leftBDrive.setPower(actSpeed);
                    robot.rightBDrive.setPower(actSpeed*(67.5/70.0));
                    decel = true;
                } else {
                    if (!decel && (actSpeed < speed)) {
                        // If our speed is below our target and we're not slowing down, speed up
                        actSpeed = Math.min(actSpeed + (lt * SPEED_RAMP_PER_MS), speed);
                        robot.leftFDrive.setPower(actSpeed);
                        robot.rightFDrive.setPower(actSpeed*(67.5/70.0));
                        robot.leftBDrive.setPower(actSpeed);
                        robot.rightBDrive.setPower(actSpeed*(67.5/70.0));
                    }
                }

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to LF:%7d, RF: %7d, LB: %7d, RB: %7d", newLeftFTarget,  newRightFTarget, newLeftBTarget,  newRightBTarget);
                telemetry.addData("Path2",  "Running at LF:%7d, RF: %7d, LB: %7d, RB: %7d",
                                            robot.leftFDrive.getCurrentPosition(),
                                            robot.rightFDrive.getCurrentPosition(),
                                            robot.leftBDrive.getCurrentPosition(),
                                            robot.rightBDrive.getCurrentPosition());
                telemetry.addData("Path3",  "Running rate: %3.3f, rem: %7.0f, s: %1.3f", rate, rem, actSpeed);
                telemetry.update();
                sleep(10);   // optional pause after each move
            }

            // Stop all motion;
            robot.leftFDrive.setPower(0);
            robot.rightFDrive.setPower(0);
            robot.leftBDrive.setPower(0);
            robot.rightBDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(500);   // optional pause after each move
        }
    }
}

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

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

@Autonomous(name="Drive with encoder", group="Pushbot")
@Disabled
public class DriveWithEncoder extends LinearOpMode {

    /* Declare OpMode members. */
    IncepBot        robot   = new IncepBot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // NeveRest Orbital 20 Gearmotor
    static final double     DRIVE_GEAR_REDUCTION    = 10.0/11.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.54331 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                        (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     WHEEL_BASE_SIZE         = 13.05;
    static final double     INCHES_PER_DEGREE       = WHEEL_BASE_SIZE * 3.1415 / 360;


    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.25;


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
        robot.leftBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          robot.leftFDrive.getCurrentPosition(),
                          robot.rightBDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        encoderDrive(DRIVE_SPEED,  100,  100, 20.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderTurn(TURN_SPEED,   180, 20.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED,  60,  60, 20.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(DRIVE_SPEED,  -36,  -36, 20.0);  // S1: Forward 47 Inches with 5 Sec timeout


        /*
        encoderDrive(DRIVE_SPEED,  24,  24, 20.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderTurn(TURN_SPEED,   90, 20.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, 24, 24, 20.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        encoderTurn(TURN_SPEED,   -90, 20.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, 8, 8, 20.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -8, -8, 20.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        encoderTurn(TURN_SPEED,   -90, 20.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, 96, 96, 20.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        encoderTurn(TURN_SPEED,   90, 20.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, 8, 8, 20.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -30, -30, 20.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, 24, 24, 20.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        encoderTurn(TURN_SPEED,   90, 20.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, 36, 36, 20.0);  // S3: Reverse 24 Inches with 4 Sec timeout
*/


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
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
        int newLeftBTarget;
        int newRightFTarget;
        int newRightBTarget;
        int distanceToGo;
        int distanceGone;
        int halfwayPoint;
        double[] possibleSpeeds =  new double[]{0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6};
        int currentSpeedIndex = 0;
        double adjustedSpeed = possibleSpeeds[0];



        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            //Reset Encoder Values
            robot.leftFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.leftFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftFTarget = robot.leftFDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeftBTarget = robot.leftBDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFTarget = robot.rightFDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightBTarget = robot.rightBDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftFDrive.setTargetPosition(newLeftFTarget);
            robot.leftBDrive.setTargetPosition(newLeftBTarget);
            robot.rightFDrive.setTargetPosition(newRightFTarget);
            robot.rightBDrive.setTargetPosition(newRightBTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFDrive.setPower(Math.abs(adjustedSpeed));
            robot.leftBDrive.setPower(Math.abs(adjustedSpeed));
            robot.rightFDrive.setPower(Math.abs(adjustedSpeed * (69.0/70.0)));
            robot.rightBDrive.setPower(Math.abs(adjustedSpeed * (69.0/70.0)));
            halfwayPoint = newLeftFTarget / 2;

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.leftFDrive.isBusy() && robot.rightFDrive.isBusy() && robot.leftBDrive.isBusy() && robot.rightBDrive.isBusy())) {

                distanceToGo =  newLeftFTarget - robot.leftFDrive.getCurrentPosition();
                distanceGone = newLeftFTarget - distanceToGo;



                if (adjustedSpeed < speed && distanceGone < halfwayPoint){
                    if (Math.abs(distanceGone) > 100) {
                        currentSpeedIndex = (Math.abs(distanceGone) / 100) - 1;
                    }
                    adjustedSpeed = possibleSpeeds[currentSpeedIndex];
                } else if (adjustedSpeed > .15 && (distanceToGo < 0 && (Math.abs(distanceToGo) > Math.abs(halfwayPoint)) || (distanceToGo > 0 && Math.abs(distanceToGo) < Math.abs(halfwayPoint)))){
                    if (currentSpeedIndex >= 1) {
                        currentSpeedIndex = (Math.abs(distanceToGo) / 100) - 1;
                    }
                    if (currentSpeedIndex < possibleSpeeds.length-1) {
                        adjustedSpeed = possibleSpeeds[currentSpeedIndex];
                    }
                }


                /*
                if (adjustedSpeed < speed && !(Math.abs(newLeftFTarget * 0.6) < Math.abs(robot.leftFDrive.getCurrentPosition()))){
                    adjustedSpeed += .01;
                } else if (Math.abs(newLeftFTarget * 0.6) < Math.abs(robot.leftFDrive.getCurrentPosition()) && adjustedSpeed > .15){
                    adjustedSpeed -= .01;
                }
*/
                robot.leftFDrive.setPower(Math.abs(adjustedSpeed));
                robot.leftBDrive.setPower(Math.abs(adjustedSpeed));
                robot.rightFDrive.setPower(Math.abs(adjustedSpeed * (69.0/70.0)));
                robot.rightBDrive.setPower(Math.abs(adjustedSpeed * (69.0/70.0)));


                // Display it for the driver.
                telemetry.addData("Path1",  "Left Front: Current %7d | Target %7d", newLeftFTarget,  robot.leftFDrive.getCurrentPosition());
                telemetry.addData("Path2",  "Left Back: Current %7d | Target %7d", newLeftBTarget, robot.leftBDrive.getCurrentPosition());
                telemetry.addData("Path3",  "Right Front: Current %7d | Target %7d", newRightFTarget,  robot.rightFDrive.getCurrentPosition());
                telemetry.addData("Path4",  "Right Back: Current %7d | Target %7d", newRightBTarget,  robot.rightBDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFDrive.setPower(0);
            robot.leftBDrive.setPower(0);
            robot.rightFDrive.setPower(0);
            robot.rightBDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    /*
     *  Degrees is always to the right.
     */


    public void encoderTurn(double speed, double degrees, double timeoutS){

        double leftInches = degrees * INCHES_PER_DEGREE;
        double rightInches = -degrees * INCHES_PER_DEGREE;

        encoderDrive(speed,leftInches,rightInches,timeoutS);
    }





}

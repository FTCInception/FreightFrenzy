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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import Inception.FreightFrenzy.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


//import org.openftc.revextensions2.ExpansionHubEx;
//import org.openftc.revextensions2.ExpansionHubMotor;
//import org.openftc.revextensions2.RevBulkData;

/**
 * Made by DaSchelling for Testing programs for team 12533...
 * Here is the mapping for the motors:
 *      Left front motor -> Slot 0 | Zip-Tie Color: Blue
 *      Right front motor -> Slot 1 | Zip-Tie Color: Orange
 *      Left back motor -> Slot 2 | Zip-Tie Color: Purple
 *      Right back motor -> Slot 3 | Zip-Tie Color: Green
 */

/** Controller configuration
 * POV
 * Done: Left stick is straight/strafe
 * Done: Right stick is turn
 * A toggle for robot speed  75/100
 * Y reverse intake
 * Right bumper is intake (toggle)
 * Shooter is toggle high/low on dpad
 * left/right dpad is power shot rotate
 * Left bumper: flicker
 * B forwards for wobble cycle
 * X backwards for wobble cycle
 * Logitech button to reset shooter, reset intake, return arm and claw to defaults.
 * Reverse shooter on trigger?
 */

@TeleOp(name="RRMech", group="Linear Opmode")
public class RRMech_Teleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private static ExpansionHubEx expansionHub1;
    //private static ExpansionHubEx expansionHub2;
    //private static DcMotorEx l_f_motor, l_b_motor, r_f_motor, r_b_motor;
    private static DcMotor intake_motor;
    private static Servo bucket, slide, duckL, duckR;

    //Orientation angles,angles2;
    double MAX_INTAKE_POWER = 0.5;

    //private BotLog logger = new BotLog();
    private boolean enableCSVLogging = false;

    private RRMechBot robot = new RRMechBot();

    // Mech drive related variables
    double[] speedModifier = new double[] {0.99, 0.99};
    double[] forward = new double[2], strafe = new double[2], rotate = new double[2];
    double[] prevForward = new double[2], prevStrafe = new double[2], prevRotate = new double[2];
    double[] prevTime = new double[2];
    double maxForwardChange=4.0, maxRotateChange=5.0, maxStrafeChange=3.0;
    boolean smoothDrive = true;

    double l_f_motor_power;
    double l_b_motor_power;
    double r_f_motor_power;
    double r_b_motor_power;

    private static boolean SWPID = true;
    private boolean shooterDownShifted = false;

    double theta, r_speed, new_x, new_y, degrees;

    double shootingAngle;
    Pose2d shootingPose;
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }
    Mode currentMode = Mode.DRIVER_CONTROL;

    // Declare other variables
    public BNO055IMU initIMU(String imuName) {

        BNO055IMU imu;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, imuName);

        imu.initialize(parameters);

        telemetry.addData("Mode", "IMU calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        for (int i=0; (i<40 && !imu.isGyroCalibrated()); i++)
        {
            sleep(50);
            idle();
        }

        return(imu);
    }

    @Override
    public void runOpMode() {
        //RevBulkData bulkData1, bulkData2;
        boolean[] lBumpPrev = new boolean[]{false, false};
        boolean[] rBumpPrev = new boolean[]{false, false};
        boolean[] lTrigPrev = new boolean[]{false, false};
        boolean[] rTrigPrev = new boolean[]{false, false};
        boolean[] aPrev = new boolean[]{false, false};
        boolean[] bPrev = new boolean[]{false, false};
        boolean[] xPrev = new boolean[]{false, false};
        boolean[] yPrev = new boolean[]{false, false};
        boolean[] dUpPrev = new boolean[]{false, false};
        boolean[] dDownPrev = new boolean[]{false, false};
        boolean[] dLeftPrev = new boolean[]{false, false};
        boolean[] dRightPrev = new boolean[]{false, false};
        boolean[] guidePrev = new boolean[]{false, false};

        final double BUCKET_COLLECT = 0.0, BUCKET_DUMP=1.0;
        final double SLIDE_COLLECT = 1.0, SLIDE_DRIVE = 0.9, SLIDE_LOW = 0.8, SLIDE_MED = 0.5, SLIDE_HIGH = 0.0;
        double[] intakeSet = {0.0, MAX_INTAKE_POWER};
        double[] slideSet = {SLIDE_COLLECT, SLIDE_DRIVE, SLIDE_LOW, SLIDE_MED, SLIDE_HIGH};
        int slideIdx=0, bucketIdx=0, intakeIdx=0;
        double prevLTrigVal=0.0;
        double prevRTrigVal=0.0;

        double maxLag, prt, rt, nextLog = 0.0, iter=0.0;
        boolean gp1Present = false, gp2Present = false;

        double maxPwr = 0.0;

        double tps=0.0;

        if (enableCSVLogging) {
            // Enable debug logging
            robot.logger.LOGLEVEL |= robot.logger.LOGDEBUG;
        }

        //robot.logger.LOGLEVEL |= robot.logger.LOGDEBUG;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.acquireHW(hardwareMap);

        // Initialize custom cancelable SampleMecanumDrive class
        SampleMecanumDrive drive = robot.drive;

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // TODO: Revisit the PIDF values for motors and make sure they seem OK for both auto and teleop reference last year's code

        /*************************************************************/
        /************ No movement allowed during init! ***************/
        /************ No movement allowed during init! ***************/
        /************ No movement allowed during init! ***************/
        /*************************************************************/

        intake_motor = robot.intake_motor;
        intake_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slide = robot.slide;
        bucket = robot.bucket;
        duckL = robot.duckL;
        duckR = robot.duckR;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Waiting for start...");
        telemetry.update();
        /*************************************************************/
        /************ No movement allowed during init! ***************/
        /************ No movement allowed during init! ***************/
        /************ No movement allowed during init! ***************/
        /*************************************************************/

        waitForStart();
        runtime.reset();
        iter = 0.0;
        prt = rt = maxLag = 0.0;
        shooterDownShifted = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (!gp1Present) {
                if (!gamepad1.atRest()) {
                    gp1Present = true ;
                }
            }

            if (!gp2Present) {
                if (!gamepad2.atRest()) {
                    gp2Present = true ;
                }
            }

            prt = rt ;
            rt = runtime.seconds() ;
            maxLag = Math.max(maxLag, ((rt-prt)*1000.0));
            iter += 1;

            if (gp1Present) {
                // TODO: Left trigger is <TBD>; trigger is continuous scale
                if ((gamepad1.left_trigger > 0.0) || (prevLTrigVal > 0.0)) {
                    prevLTrigVal = gamepad1.left_trigger;
                    //servo1.setPosition(prevLTrigVal);
                }

                // TODO: Right trigger is <TBD>; trigger is continuous scale
                if ((gamepad1.right_trigger > 0.0) || (prevRTrigVal > 0.0)) {
                    prevRTrigVal = gamepad1.right_trigger;
                    //servo1.setPosition(prevRTrigVal);
                }

                // TODO: Left bumper is <TBD>; bumper is discrete
                if (gamepad1.left_bumper) {
                    if (!lBumpPrev[0]) {
                        intakeIdx = (intakeIdx + 1) % intakeSet.length;
                        intake_motor.setPower(intakeSet[intakeIdx]);
                        lBumpPrev[0] = true;
                    }
                } else {
                    lBumpPrev[0] = false;
                }

                // Start/stop the intake
                if (gamepad1.right_bumper) {
                    if (!rBumpPrev[0]) {
                        //Do something
                        rBumpPrev[0] = true;
                    }
                } else {
                    rBumpPrev[0] = false;
                }

                // TODO: 'a' <TBD>; button is discrete
                if (gamepad1.a) {
                    if (!aPrev[0]) {
                        // Do something
                        aPrev[0] = true;
                    }
                } else {
                    aPrev[0] = false;
                }

                // TODO: 'b' <TBD>; button is discrete
                if (gamepad1.b) {
                    if (!bPrev[0]) {
                        // Do something
                        bPrev[0] = true;
                    }
                } else {
                    bPrev[0] = false;
                }

                // TODO: 'x' <TBD>; button is discrete
                if (gamepad1.x) {
                    if (!xPrev[0]) {
                        // Do something
                        xPrev[0] = true;
                    }
                } else {
                    xPrev[0] = false;
                }

                // TODO: 'y' <TBD>; button is discrete
                if (gamepad1.y) {
                    if (!yPrev[0]) {
                        // Do something
                        yPrev[0] = true;
                    }
                } else {
                    yPrev[0] = false;
                }

                // TODO: 'up' <TBD>; button is discrete
                if (gamepad1.dpad_up) {
                    if (!dUpPrev[0]) {
                        slideIdx = Math.min((slideIdx + 1), slideSet.length-1);
                        slide.setPosition(slideSet[slideIdx]);
                        dUpPrev[0] = true;
                    }
                } else {
                    dUpPrev[0] = false;
                }

                // TODO: 'down' <TBD>; button is discrete
                if (gamepad1.dpad_down) {
                    if (!dDownPrev[0]) {
                        slideIdx = Math.max((slideIdx - 1), 0);
                        slide.setPosition(slideSet[slideIdx]);
                        dDownPrev[0] = true;
                    }
                } else {
                    dDownPrev[0] = false;
                }

                // TODO: 'left' <TBD>; button is discrete
                if (gamepad1.dpad_left) {
                    if (!dLeftPrev[0]) {
                        // Do something
                        dLeftPrev[0] = true;
                    }
                } else {
                    dLeftPrev[0] = false;
                }

                // TODO: 'right' <TBD>; button is discrete
                if (gamepad1.dpad_right) {
                    if (!dRightPrev[0]) {
                        // Do something
                        dRightPrev[0] = true;
                    }
                } else {
                    dRightPrev[0] = false;
                }
            }

            if (gp2Present) {
                // TODO: Left trigger is <TBD>; trigger is continuous scale
                if ((gamepad2.left_trigger > 0.0) || (prevLTrigVal > 0.0)) {
                    prevLTrigVal = gamepad2.left_trigger;
                    //servo1.setPosition(prevLTrigVal);
                }

                // TODO: Right trigger is <TBD>; trigger is continuous scale
                if ((gamepad2.right_trigger > 0.0) || (prevRTrigVal > 0.0)) {
                    prevRTrigVal = gamepad2.right_trigger;
                    //servo1.setPosition(prevRTrigVal);
                }

                // TODO: Left bumper is <TBD>; bumper is discrete
                if (gamepad2.left_bumper) {
                    if (!lBumpPrev[0]) {
                        intakeIdx = (intakeIdx + 1) % intakeSet.length;
                        intake_motor.setPower(intakeSet[intakeIdx]);
                        lBumpPrev[0] = true;
                    }
                } else {
                    lBumpPrev[0] = false;
                }

                // Start/stop the intake
                if (gamepad2.right_bumper) {
                    if (!rBumpPrev[0]) {
                        intakeIdx = (intakeIdx + 1) % intakeSet.length;
                        intake_motor.setPower(intakeSet[intakeIdx]);
                        rBumpPrev[0] = true;
                    }
                } else {
                    rBumpPrev[0] = false;
                }

                // TODO: 'b' <TBD>; button is discrete
                if (gamepad2.b) {
                    if (!bPrev[0]) {
                        // Do something
                        bPrev[0] = true;
                    }
                } else {
                    bPrev[0] = false;
                }

                // TODO: 'x' <TBD>; button is discrete
                if (gamepad2.x) {
                    if (!xPrev[0]) {
                        // Do something
                        xPrev[0] = true;
                    }
                } else {
                    xPrev[0] = false;
                }

                // TODO: 'y' <TBD>; button is discrete
                if (gamepad2.y) {
                    if (!yPrev[0]) {
                        // Do something
                        yPrev[0] = true;
                    }
                } else {
                    yPrev[0] = false;
                }

                // TODO: 'up' <TBD>; button is discrete
                if (gamepad2.dpad_up) {
                    if (!dUpPrev[0]) {
                        // Do something
                        dUpPrev[0] = true;
                    }
                } else {
                    dUpPrev[0] = false;
                }

                // TODO: 'down' <TBD>; button is discrete
                if (gamepad2.dpad_down) {
                    if (!dDownPrev[0]) {
                        // Do something
                        dDownPrev[0] = true;
                    }
                } else {
                    dDownPrev[0] = false;
                }

                // TODO: 'left' <TBD>; button is discrete
                if (gamepad2.dpad_left) {
                    if (!dLeftPrev[0]) {
                        // Do something
                        dLeftPrev[0] = true;
                    }
                } else {
                    dLeftPrev[0] = false;
                }

                // TODO: 'right' <TBD>; button is discrete
                if (gamepad2.dpad_right) {
                    if (!dRightPrev[0]) {
                        // Do something
                        dRightPrev[0] = true;
                    }
                } else {
                    dRightPrev[0] = false;
                }
            }

            // Read the controller 1 (driver) stick positions
            strafe[0] = gamepad1.left_stick_x*1.25;
            forward[0] = -gamepad1.left_stick_y;
            rotate[0] = gamepad1.right_stick_x*1.5;

            if(smoothDrive) {
                if ((prevStrafe[0] != 0) && (strafe[0] != 0) && (Math.signum(prevStrafe[0]) != Math.signum(strafe[0]))) {
                    strafe[0] = 0;
                }
                if ((prevForward[0] != 0) && (forward[0] != 0) && (Math.signum(prevForward[0]) != Math.signum(forward[0]))) {
                    forward[0] = 0;
                }
                if ((prevRotate[0] != 0) && (rotate[0] != 0) && (Math.signum(prevRotate[0]) != Math.signum(rotate[0]))) {
                    rotate[0] = 0;
                }

                double now = runtime.seconds();
                double deltaT = now - prevTime[0];
                if (Math.abs(strafe[0]) > 0.20) {
                    if (prevStrafe[0] < strafe[0]) {
                        strafe[0] = Math.min(strafe[0], prevStrafe[0] + (maxStrafeChange * deltaT));
                    } else {
                        strafe[0] = Math.max(strafe[0], prevStrafe[0] - (maxStrafeChange * deltaT));
                    }
                }
                if (Math.abs(forward[0]) > 0.20) {
                    if (prevForward[0] < forward[0]) {
                        forward[0] = Math.min(forward[0], prevForward[0] + (maxForwardChange * deltaT));
                    } else {
                        forward[0] = Math.max(forward[0], prevForward[0] - (maxForwardChange * deltaT));
                    }
                }
                if (Math.abs(rotate[0]) > 0.20) {
                    if (prevRotate[0] < rotate[0]) {
                        rotate[0] = Math.min(rotate[0], prevRotate[0] + (maxRotateChange * deltaT));
                    } else {
                        rotate[0] = Math.max(rotate[0], prevRotate[0] - (maxRotateChange * deltaT));
                    }
                }
                prevTime[0] = now;
                prevStrafe[0] = strafe[0];
                prevForward[0] = forward[0];
                prevRotate[0] = rotate[0];

                // Remove 15% deadzone
                if (strafe[0] >= 0.025) {
                    strafe[0] = (strafe[0] * 0.85) + 0.15;
                }
                if (forward[0] >= 0.025) {
                    forward[0] = (forward[0] * 0.85) + 0.15;
                }
                if (rotate[0] >= 0.025) {
                    rotate[0] = (rotate[0] * 0.85) + 0.15;
                }
                if (strafe[0] <= -0.025) {
                    strafe[0] = (strafe[0] * 0.85) - 0.15;
                }
                if (forward[0] <= -0.025) {
                    forward[0] = (forward[0] * 0.85) - 0.15;
                }
                if (rotate[0] <= -0.025) {
                    rotate[0] = (rotate[0] * 0.85) - 0.15;
                }
            }

            // Rotate a little left
            if (gamepad1.dpad_left) {
                rotate[0] -= 0.25;
            }
            // Rotate a little right
            if (gamepad1.dpad_right) {
                rotate[0] += 0.25;
            }

            strafe[1] = gamepad2.left_stick_x;
            forward[1] = -gamepad2.left_stick_y;
            rotate[1] = gamepad2.right_stick_x;

            if( smoothDrive ) {

                if((prevStrafe[1] != 0) && (strafe[1] != 0) && (Math.signum(prevStrafe[1]) != Math.signum(strafe[1]))) {
                    strafe[1] = 0;
                }
                if((prevForward[1] != 0) && (forward[1] != 0) && (Math.signum(prevForward[1]) != Math.signum(forward[1]))) {
                    forward[1] = 0;
                }
                if((prevRotate[1] != 0) && (rotate[1] != 0) && (Math.signum(prevRotate[1]) != Math.signum(rotate[1]))) {
                    rotate[1] = 0;
                }

                double now = runtime.seconds();
                double deltaT = now - prevTime[1];
                if (Math.abs(strafe[1]) > 0.20) {
                    if (prevStrafe[1] < strafe[1]) {
                        strafe[1] = Math.min(strafe[1], prevStrafe[1] + (maxStrafeChange * deltaT));
                    } else {
                        strafe[1] = Math.max(strafe[1], prevStrafe[1] - (maxStrafeChange * deltaT));
                    }
                }
                if (Math.abs(forward[1]) > 0.20) {
                    if (prevForward[1] < forward[1]) {
                        forward[1] = Math.min(forward[1], prevForward[1] + (maxForwardChange * deltaT));
                    } else {
                        forward[1] = Math.max(forward[1], prevForward[1] - (maxForwardChange * deltaT));
                    }
                }
                if (Math.abs(rotate[1]) > 0.20) {
                    if (prevRotate[1] < rotate[1]) {
                        rotate[1] = Math.min(rotate[1], prevRotate[1] + (maxRotateChange * deltaT));
                    } else {
                        rotate[1] = Math.max(rotate[1], prevRotate[1] - (maxRotateChange * deltaT));
                    }
                }
                prevTime[1] = now;
                prevStrafe[1] = strafe[1];
                prevForward[1] = forward[1];
                prevRotate[1] = rotate[1];

                // Remove 15% deadzone
                if (strafe[1] >= 0.025) {
                    strafe[1] = (strafe[1] * 0.85) + 0.15;
                }
                if (forward[1] >= 0.025) {
                    forward[1] = (forward[1] * 0.85) + 0.15;
                }
                if (rotate[1] >= 0.025) {
                    rotate[1] = (rotate[1] * 0.85) + 0.15;
                }
                if (strafe[1] <= -0.025) {
                    strafe[1] = (strafe[1] * 0.85) - 0.15;
                }
                if (forward[1] <= -0.025) {
                    forward[1] = (forward[1] * 0.85) - 0.15;
                }
                if (rotate[1] <= -0.025) {
                    rotate[1] = (rotate[1] * 0.85) - 0.15;
                }
            }

            // Rotate a little left
            if (gamepad2.dpad_left) {
                rotate[1] -= 0.25;
            }
            // Rotate a little right
            if (gamepad2.dpad_right) {
                rotate[1] += 0.25;
            }

            boolean doFOV = false;
            if (doFOV) {
                // Get the current robot heading
                degrees = Math.toDegrees(drive.getRawExternalHeading());
                // Convert the X/Y Cartesion for strafe and forward into Polar
                CarToPol(strafe[1], forward[1]);
                // Rotate the Polar coordinates by the robot's heading
                theta -= degrees;
                // Convert the new Polar back into Cartesian
                PolToCar(r_speed);
                // Replace the strafe and forward power with translated values
                strafe[1] = new_x;
                forward[1] = new_y;
                // Now the robot moves in orientation of the field
            }


            // This adds the powers from both controllers together scaled for each controller and FOV
            l_f_motor_power = ((forward[0] + strafe[0] + rotate[0]) * speedModifier[0]) +
                    ((forward[1] + strafe[1] + rotate[1]) * speedModifier[1]);
            l_b_motor_power = ((forward[0] - strafe[0] + rotate[0]) * speedModifier[0]) +
                    ((forward[1] - strafe[1] + rotate[1]) * speedModifier[1]);
            r_f_motor_power = ((forward[0] - strafe[0] - rotate[0]) * speedModifier[0]) +
                    ((forward[1] - strafe[1] - rotate[1]) * speedModifier[1]);
            r_b_motor_power = ((forward[0] + strafe[0] - rotate[0]) * speedModifier[0]) +
                    ((forward[1] + strafe[1] - rotate[1]) * speedModifier[1]);

            if(smoothDrive) {
                // Find the largest power request ignoring sign
                maxPwr = Math.max(Math.max(Math.max(Math.abs(l_f_motor_power), Math.abs(l_b_motor_power)),
                        Math.abs(r_f_motor_power)), Math.abs(r_b_motor_power));

                // If this is greater than 1.0, need to scale everything back equally
                // Max is now guaranteed positive which is good to reduce magnitude without changing sign
                // Now the power is scaled and limited to range of {-1.0, 1.0)
                if (maxPwr > 1.0) {
                    l_f_motor_power /= maxPwr;
                    l_b_motor_power /= maxPwr;
                    r_f_motor_power /= maxPwr;
                    r_b_motor_power /= maxPwr;
                }
            }

            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            drive.setMotorPowers(l_f_motor_power, l_b_motor_power, r_b_motor_power, r_f_motor_power);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("", "");
            if( false ) {
                telemetry.addData("Some message 1", "");
            } else {
                telemetry.addData("Some message 2", "");
            }
            telemetry.update();
        }

        // Stop all power
        drive.setMotorPowers(0,0,0,0);
    }

    void CarToPol(double x, double y) {
        r_speed = Math.sqrt(x * x + y * y);
        theta = Math.atan2(y, x);

        //theta to degrees
        theta = theta * 180 / 3.14159265358979323;
    }

    void PolToCar(double r) {
        theta = theta / 180 * 3.14159265358979323;
        new_x = r * Math.cos(theta);
        new_y = r * Math.sin(theta);
    }

}
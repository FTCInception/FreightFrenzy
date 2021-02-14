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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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
    private static DcMotorEx shoot1_motor, shoot2_motor;
    private static DcMotorEx wobble_motor;
    private static Servo claw,flicker;

    //Orientation angles,angles2;
    double MAX_INTAKE_POWER = 1.0;

    private BotLog logger = new BotLog();
    private boolean enableCSVLogging = false;

    // Mech drive related variables
    double[] speedModifier = new double[] {0.75,0.75};
    double[] forward = new double[2], strafe = new double[2], rotate = new double[2];

    double l_f_motor_power;
    double l_b_motor_power;
    double r_f_motor_power;
    double r_b_motor_power;

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

        final double CLAW_OPEN = 0.0, CLAW_CLOSED=1.0, CLAW_HALF=0.5;
        final double FLICKER_SHOOT = 0.7, FLICKER_WAIT=0.0;
        final double SHOOTER_NORMAL=0.475, SHOOTER_POWER_SHOT=0.4375;
        //wobble stuff
        //final double WOBBLE_TICKS_PER_DEGREE = 5264.0/360.0; // 30 RPM 6mm d-shaft (5202 series)
        //final double WOBBLE_TICKS_PER_DEGREE = 2786.0/360.0; // 60 RPM 6mm d-shaft (5202 series)
        final double startingAngle = 15.0;
        final double WOBBLE_TICKS_PER_DEGREE = 3892.0/360.0; // 43 RPM 8mm REX (5203 series)
        final double[][] wobbleSeq = {
                {CLAW_CLOSED,   0, 0.65,                  5*WOBBLE_TICKS_PER_DEGREE},
                {CLAW_OPEN,     0, 0.8, (235-startingAngle)*WOBBLE_TICKS_PER_DEGREE},
                {CLAW_CLOSED,   0, 0.8, (235-startingAngle)*WOBBLE_TICKS_PER_DEGREE},
                {CLAW_CLOSED,   0, 0.65,( 45-startingAngle)*WOBBLE_TICKS_PER_DEGREE},
                {CLAW_CLOSED,   0, 0.65,(180-startingAngle)*WOBBLE_TICKS_PER_DEGREE},
                {CLAW_OPEN,   500, 0.65,                  5*WOBBLE_TICKS_PER_DEGREE},
        };

        double[] flickerSet = {FLICKER_WAIT, FLICKER_SHOOT};
        double[] clawSet = {CLAW_CLOSED, CLAW_OPEN};
        double[] intakeSet = {0.0, MAX_INTAKE_POWER};
        double[] speedSet = {0.75, 0.90};
        double[] shooterSet = {0.0, SHOOTER_NORMAL};
        int flickerIdx=0, clawIdx=0, intakeIdx=0, speedIdx=0, shooterIdx=0, wobbleIdx=0;
        double flickerRelease=0.0, flickerRearm=0.0;
        double prevLTrigVal=0.0;
        double prevRTrigVal=0.0;

        double maxLag, prt, rt, nextLog = 0.0, iter=0.0;
        boolean gp1Present = false, gp2Present = false;

        double maxPwr = 0.0;

        double tps=0.0;
        double shoot2Pos, prevShoot2Pos=0.0;
        double nextPID=0.0, PIDTime = 0.1;

        if (enableCSVLogging) {
            // Enable debug logging
            logger.LOGLEVEL |= logger.LOGDEBUG;
        }

        //logger.LOGLEVEL |= logger.LOGDEBUG;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize custom cancelable SampleMecanumDrive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // This is a more sane PIDF for humans compared to the Autonomous
        //drive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10.0,3.0,0.0,0.0));
        // This is a guess at some PIDF from default roadrunner
        //drive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(2.0,0.5,0.0,11.1));

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        //drive.setPoseEstimate(PoseStorage.currentPose);
        shootingPose = drive.getPoseEstimate();

        /*************************************************************/
        /************ No movement allowed during init! ***************/
        /************ No movement allowed during init! ***************/
        /************ No movement allowed during init! ***************/
        /*************************************************************/

        intake_motor = hardwareMap.dcMotor.get("intake");
        intake_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // The wobble keeps it's encoder value from auto, this shoudl allow us to get back to '0' position
        // in case something bad happened.  DO NOT STOP_AND_RESET_ENCODER here to preserve the '0' postion.
        wobble_motor = hardwareMap.get(DcMotorEx.class,"wobble");
        wobble_motor.setPower(0.0);
        //wobble_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        wobble_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        wobble_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (wobble_motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            // We already set power to 0 so it should be safe to set a target position
            // so we can then put ourselves into RUN_TO_POSITION mode.
            wobble_motor.setTargetPosition(0);
            wobble_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        wobble_motor.setPositionPIDFCoefficients(5.0);
        wobble_motor.setVelocityPIDFCoefficients(2.0,0.5,0.0,11.1);

        shoot1_motor = hardwareMap.get(DcMotorEx.class,"shoot1");
        shoot2_motor = hardwareMap.get(DcMotorEx.class,"shoot2");
        shoot1_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot2_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot1_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shoot2_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shoot1_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot2_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot1_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot2_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // These PIDF values seem pretty good.
        //PIDFCoefficients pidFNew = new PIDFCoefficients(125.0, 2.5, 5.0, 4.0);
        // Raising 'F' increases overshoot a lot
        // Raising 'I' increases overshoot a lot
        // 'P' is in a sweet-spot, could go down to 75 and still be OK
        // 'D' didn't make a ton of different, not sure that is tuned properly
        // Quick spin-up and recovery.  There may be a little overshoot just after a shot.
        shoot1_motor.setVelocityPIDFCoefficients(125.0, 2.5, 5.0, 4.0);
        shoot2_motor.setVelocityPIDFCoefficients(125.0, 2.5, 5.0, 4.0);

        claw = hardwareMap.servo.get("claw");
        flicker = hardwareMap.servo.get("flicker");

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

            if ((rt > 90) && (shooterSet[1] != SHOOTER_POWER_SHOT)) {
                shooterSet[1] = SHOOTER_POWER_SHOT;
                shoot1_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shoot2_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shoot1_motor.setPower(shooterSet[shooterIdx]);
                shoot2_motor.setPower(shooterSet[shooterIdx]);
            }

            if (gp1Present) {
                // Left trigger is a flicker override to help un-jam if needed.
                // Move flicker to the trigger position if non-zero or
                // recently used the trigger to send a '0' position.
                if ((gamepad1.left_trigger > 0.0) || (prevLTrigVal > 0.0)) {
                    prevLTrigVal = gamepad1.left_trigger;
                    flicker.setPosition(prevLTrigVal);
                }

                // Right trigger is shooter reverse to help un-jam if needed.
                // Drive shooter at 40% trigger position if non-zero or
                // recently used the trigger to send a '0' position.
                if (((gamepad1.right_trigger > 0.0) || (prevRTrigVal > 0.0)) &&
                        (shooterIdx == 0)) {
                    prevRTrigVal = gamepad1.right_trigger * 0.30;
                    shoot1_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    shoot2_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    shoot1_motor.setPower(-prevRTrigVal);
                    shoot2_motor.setPower(-prevRTrigVal);
                }

                // One-button timed flick
                // States:  Release and Rearm
                // Release == 0 and rearm == 0 ==> at rest
                // Release > 0  and rearm == 0 ==> waiting for flicker to reach max, release in the future
                // Release == 0 and rearm > 0  ==> waiting for flicker to reach home, prevent re-flick until done
                // Release > 0  and Rearm > 0  ==> illegal
                if ((flickerRelease == 0.0) && (flickerRearm == 0.0)) {
                    // This is the 'normal'/'at-rest' case.  No flick in progress.
                    // Only flick if the shooter is running.
                    // Shoot the flicker
                    if ((gamepad1.left_bumper) && (shooterIdx == 1)) {
                        // Set a future time to return flicker to rest
                        flicker.setPosition(FLICKER_SHOOT);
                        flickerRelease = rt + .25;
                        shootingPose = drive.getPoseEstimate();
                    } else {
                        if (prevLTrigVal == 0.0) {
                            // Just keep asking to return to wait position
                            flicker.setPosition(FLICKER_WAIT);
                        }
                    }
                } else if (flickerRelease > 0.0) {
                    // This is the case that the flick is in progress of pushing the ring in
                    if (flickerRelease < rt) {
                        // Once the time has elapsed, move to the next state
                        // and set a timer to wait for release
                        flickerRelease = 0.0;
                        flickerRearm = rt + 0.30;
                    } else {
                        // Just keep asking to flick
                        flicker.setPosition(FLICKER_SHOOT);
                    }
                } else if (flickerRearm > 0.0) {
                    // This is when we are waiting for flicker to return to rest
                    if (flickerRearm < rt) {
                        // Once time has elapsed, leave the state
                        flickerRearm = 0.0;
                    } else {
                        // keep asking
                        flicker.setPosition(FLICKER_WAIT);
                    }
                } else {
                    // OK, something is messed up, lets just go back to steady-state
                    // Not sure it's possible to get here.
                    flicker.setPosition(FLICKER_WAIT);
                    flickerRelease = 0.0;
                    flickerRearm = 0.0;
                }

                // Start/stop the intake
                if (gamepad1.right_bumper) {
                    if (!rBumpPrev[0]) {
                        intakeIdx = (intakeIdx + 1) % intakeSet.length;
                        intake_motor.setPower(intakeSet[intakeIdx]);
                        rBumpPrev[0] = true;
                    }
                } else {
                    rBumpPrev[0] = false;
                }

                // Change the chassis speed
                if (gamepad1.a) {
                    if (!aPrev[0]) {
                        speedIdx = (speedIdx + 1) % speedSet.length;
                        speedModifier[0] = speedSet[speedIdx];
                        aPrev[0] = true;
                    }
                } else {
                    aPrev[0] = false;
                }

                // Advance the wobble sequence
                if (gamepad1.b) {
                    if (!bPrev[0]) {
                        wobbleIdx = (wobbleIdx + 1) % wobbleSeq.length;
                        claw.setPosition(wobbleSeq[wobbleIdx][0]);
                        sleep((long) wobbleSeq[wobbleIdx][1]);
                        wobble_motor.setTargetPosition((int) wobbleSeq[wobbleIdx][3]);
                        wobble_motor.setPower(wobbleSeq[wobbleIdx][2]);
                        bPrev[0] = true;
                    }
                } else {
                    bPrev[0] = false;
                }

                // Backup the wobble sequence
                if (gamepad1.x) {
                    if (!xPrev[0]) {
                        if (wobbleIdx == 0) {
                            wobbleIdx = (wobbleSeq.length - 1);
                        } else {
                            wobbleIdx = (wobbleIdx - 1) % wobbleSeq.length;
                        }
                        wobble_motor.setTargetPosition((int) wobbleSeq[wobbleIdx][3]);
                        wobble_motor.setPower(wobbleSeq[wobbleIdx][2]);
                        sleep((long) wobbleSeq[wobbleIdx][1]);
                        claw.setPosition(wobbleSeq[wobbleIdx][0]);
                        xPrev[0] = true;
                    }
                } else {
                    xPrev[0] = false;
                }

                // Let's turn off wobble motor when we're in the parked position.
                if ((wobble_motor.getCurrentPosition() <= (7.0 * WOBBLE_TICKS_PER_DEGREE)) && (wobbleIdx == 0)) {
                    wobble_motor.setPower(0);
                }

                // Stop the intake, then reverse power, and start again
                if (gamepad1.y) {
                    if (!yPrev[0]) {
                        intake_motor.setPower(0.0);
                        for (int i = 0; i < intakeSet.length; i++) {
                            intakeSet[i] *= -1.0;
                        }
                        sleep(250);
                        intake_motor.setPower(intakeSet[intakeIdx]);
                        yPrev[0] = true;
                    }
                } else {
                    yPrev[0] = false;
                }

                if (gamepad1.dpad_up) {
                    if (!dUpPrev[0]) {
                        shooterIdx = (shooterIdx + 1) % shooterSet.length;
                        if (shooterSet[shooterIdx] > 0.0) {
                            shoot1_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            shoot2_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        } else {
                            shoot1_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            shoot2_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        }
                        shoot1_motor.setPower(shooterSet[shooterIdx]);
                        shoot2_motor.setPower(shooterSet[shooterIdx]);

                        dUpPrev[0] = true;
                    }
                } else {
                    dUpPrev[0] = false;
                }

                /*
                if (gamepad1.dpad_down) {
                    if (!dDownPrev[0]) {
                        // Reset all indices and assign them
                        flickerIdx = 0;
                        flicker.setPosition(flickerSet[flickerIdx]);

                        wobbleIdx = 0;
                        wobble_motor.setTargetPosition((int) wobbleSeq[wobbleIdx][2]);
                        wobble_motor.setPower(wobbleSeq[wobbleIdx][1]);
                        claw.setPosition(wobbleSeq[wobbleIdx][0]);

                        shooterIdx = 0;
                        shoot1_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        shoot2_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        shoot1_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        shoot2_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        shoot1_motor.setPower(shooterSet[shooterIdx]);
                        shoot2_motor.setPower(shooterSet[shooterIdx]);

                        // Stop intake, set to positive
                        intakeIdx = 0;
                        intake_motor.setPower(intakeSet[intakeIdx]);
                        for (int i = 0; i < intakeSet.length; i++) {
                            intakeSet[i] = Math.abs(intakeSet[i]);
                        }

                        // Reset lag
                        maxLag = ((rt - prt) * 1000.0);

                        dDownPrev[0] = true;
                    }
                } else {
                    dDownPrev[0] = false;
                }
                */
            }

            if (gp2Present) {
                // Left trigger is a flicker override to help un-jam if needed.
                // Move flicker to the trigger position if non-zero or
                // recently used the trigger to send a '0' position.
                if ((gamepad2.left_trigger > 0.0) || (prevLTrigVal > 0.0)) {
                    prevLTrigVal = gamepad2.left_trigger;
                    flicker.setPosition(prevLTrigVal);
                }

                if (gamepad2.right_trigger > 0.3) {
                    if (!rTrigPrev[1]) {
                        shooterIdx = (shooterIdx + 1) % shooterSet.length;
                        if (shooterSet[shooterIdx] > 0.0) {
                            shoot1_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            shoot2_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        } else {
                            shoot1_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            shoot2_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        }
                        shoot1_motor.setPower(shooterSet[shooterIdx]);
                        shoot2_motor.setPower(shooterSet[shooterIdx]);

                        rTrigPrev[1] = true;
                    }
                } else {
                    rTrigPrev[1] = false;
                }

                // One-button timed flick
                // States:  Release and Rearm
                // Release == 0 and rearm == 0 ==> at rest
                // Release > 0  and rearm == 0 ==> waiting for flicker to reach max, release in the future
                // Release == 0 and rearm > 0  ==> waiting for flicker to reach home, prevent re-flick until done
                // Release > 0  and Rearm > 0  ==> illegal
                if ((flickerRelease == 0.0) && (flickerRearm == 0.0)) {
                    // This is the 'normal'/'at-rest' case.  No flick in progress.
                    // Only flick if the shooter is running.
                    // Shoot the flicker
                    if ((gamepad2.left_bumper) && (shooterIdx == 1)) {
                        // Set a future time to return flicker to rest
                        flicker.setPosition(FLICKER_SHOOT);
                        flickerRelease = rt + .25;
                        shootingPose = drive.getPoseEstimate();
                    } else {
                        if (prevLTrigVal == 0.0) {
                            // Just keep asking to return to wait position
                            flicker.setPosition(FLICKER_WAIT);
                        }
                    }
                } else if (flickerRelease > 0.0) {
                    // This is the case that the flick is in progress of pushing the ring in
                    if (flickerRelease < rt) {
                        // Once the time has elapsed, move to the next state
                        // and set a timer to wait for release
                        flickerRelease = 0.0;
                        flickerRearm = rt + 0.30;
                    } else {
                        // Just keep asking to flick
                        flicker.setPosition(FLICKER_SHOOT);
                    }
                } else if (flickerRearm > 0.0) {
                    // This is when we are waiting for flicker to return to rest
                    if (flickerRearm < rt) {
                        // Once time has elapsed, leave the state
                        flickerRearm = 0.0;
                    } else {
                        // keep asking
                        flicker.setPosition(FLICKER_WAIT);
                    }
                } else {
                    // OK, something is messed up, lets just go back to steady-state
                    // Not sure it's possible to get here.
                    flicker.setPosition(FLICKER_WAIT);
                    flickerRelease = 0.0;
                    flickerRearm = 0.0;
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

                // Change the chassis speed
                if (gamepad2.a) {
                    if (!aPrev[0]) {
                        speedIdx = (speedIdx + 1) % speedSet.length;
                        speedModifier[1] = speedSet[speedIdx];
                        aPrev[0] = true;
                    }
                } else {
                    aPrev[0] = false;
                }

                // Advance the wobble sequence
                if (gamepad2.b) {
                    if (!bPrev[0]) {
                        wobbleIdx = (wobbleIdx + 1) % wobbleSeq.length;
                        claw.setPosition(wobbleSeq[wobbleIdx][0]);
                        sleep((long) wobbleSeq[wobbleIdx][1]);
                        wobble_motor.setTargetPosition((int) wobbleSeq[wobbleIdx][3]);
                        wobble_motor.setPower(wobbleSeq[wobbleIdx][2]);
                        bPrev[0] = true;
                    }
                } else {
                    bPrev[0] = false;
                }

                // Backup the wobble sequence
                if (gamepad2.x) {
                    if (!xPrev[0]) {
                        if (wobbleIdx == 0) {
                            wobbleIdx = (wobbleSeq.length - 1);
                        } else {
                            wobbleIdx = (wobbleIdx - 1) % wobbleSeq.length;
                        }
                        wobble_motor.setTargetPosition((int) wobbleSeq[wobbleIdx][3]);
                        wobble_motor.setPower(wobbleSeq[wobbleIdx][2]);
                        sleep((long) wobbleSeq[wobbleIdx][1]);
                        claw.setPosition(wobbleSeq[wobbleIdx][0]);
                        xPrev[0] = true;
                    }
                } else {
                    xPrev[0] = false;
                }

                // Let's turn off wobble motor when we're in the parked position.
                if ((wobble_motor.getCurrentPosition() <= (7.0 * WOBBLE_TICKS_PER_DEGREE)) && (wobbleIdx == 0)) {
                    wobble_motor.setPower(0);
                }

                // Stop the intake, then reverse power, and start again
                if (gamepad2.y) {
                    if (!yPrev[0]) {
                        intake_motor.setPower(0.0);
                        for (int i = 0; i < intakeSet.length; i++) {
                            intakeSet[i] *= -1.0;
                        }
                        sleep(250);
                        intake_motor.setPower(intakeSet[intakeIdx]);
                        yPrev[0] = true;
                    }
                } else {
                    yPrev[0] = false;
                }

                if (gamepad2.dpad_up) {
                    if (!dUpPrev[0]) {
                        shooterIdx = (shooterIdx + 1) % shooterSet.length;
                        if (shooterSet[shooterIdx] > 0.0) {
                            shoot1_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            shoot2_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        } else {
                            shoot1_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            shoot2_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        }
                        shoot1_motor.setPower(shooterSet[shooterIdx]);
                        shoot2_motor.setPower(shooterSet[shooterIdx]);

                        dUpPrev[0] = true;
                    }
                } else {
                    dUpPrev[0] = false;
                }

                /*
                if (gamepad2.dpad_down) {
                    if (!dDownPrev[0]) {
                        // Reset all indices and assign them
                        flickerIdx = 0;
                        flicker.setPosition(flickerSet[flickerIdx]);

                        wobbleIdx = 0;
                        wobble_motor.setTargetPosition((int) wobbleSeq[wobbleIdx][2]);
                        wobble_motor.setPower(wobbleSeq[wobbleIdx][1]);
                        claw.setPosition(wobbleSeq[wobbleIdx][0]);

                        shooterIdx = 0;
                        shoot1_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        shoot2_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        shoot1_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        shoot2_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        shoot1_motor.setPower(shooterSet[shooterIdx]);
                        shoot2_motor.setPower(shooterSet[shooterIdx]);

                        // Stop intake, set to positive
                        intakeIdx = 0;
                        intake_motor.setPower(intakeSet[intakeIdx]);
                        for (int i = 0; i < intakeSet.length; i++) {
                            intakeSet[i] = Math.abs(intakeSet[i]);
                        }

                        // Reset lag
                        maxLag = ((rt - prt) * 1000.0);

                        dDownPrev[0] = true;
                    }
                } else {
                    dDownPrev[0] = false;
                }
                */
            }

            // Read the controller 1 (driver) stick positions
            strafe[0] = gamepad1.left_stick_x;
            forward[0] = -gamepad1.left_stick_y;
            rotate[0] = gamepad1.right_stick_x;

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

            // Rotate a little left
            if (gamepad2.dpad_left) {
                rotate[1] -= 0.25;
            }
            // Rotate a little right
            if (gamepad2.dpad_right) {
                rotate[1] += 0.25;
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

            // Find the largest power request ignoring sign
            maxPwr = Math.max(Math.max(Math.max(Math.abs(l_f_motor_power), Math.abs(l_b_motor_power)),
                    Math.abs(r_f_motor_power)),Math.abs(r_b_motor_power));

            // If this is greater than 1.0, need to scale everything back equally
            // Max is now guaranteed positive which is good to reduce magnitude without changing sign
            // Now the power is scaled and limited to range of {-1.0, 1.0)
            if (maxPwr > 1.0) {
                l_f_motor_power /= maxPwr;
                l_b_motor_power /= maxPwr;
                r_f_motor_power /= maxPwr;
                r_b_motor_power /= maxPwr;
            }

            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            switch (currentMode) {
                case DRIVER_CONTROL:

                   drive.setMotorPowers(l_f_motor_power, l_b_motor_power, r_b_motor_power, r_f_motor_power);

                   if ((gamepad1.dpad_down) || (gamepad2.dpad_down)) {
                       if (!dDownPrev[0]) {
                           Trajectory traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                   .lineToLinearHeading(shootingPose)
                                   .build();

                           drive.followTrajectoryAsync(traj1);

                           shooterIdx = 1;
                           shoot1_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                           shoot2_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                           shoot1_motor.setPower(shooterSet[shooterIdx]);
                           shoot2_motor.setPower(shooterSet[shooterIdx]);

                           currentMode = Mode.AUTOMATIC_CONTROL;
                           dDownPrev[0] = true;
                       }
                   } else {
                       dDownPrev[0] = false;
                   }
                   break;
                case AUTOMATIC_CONTROL:
                    if ((gamepad1.dpad_down) || (gamepad2.dpad_down)) {
                        if (!dDownPrev[0]) {
                            drive.cancelFollowing();
                            currentMode = Mode.DRIVER_CONTROL;

                            dDownPrev[0] = true;
                        }
                    } else {
                        dDownPrev[0] = false;
                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }

            if ( nextPID < rt ) {

                shoot2Pos = shoot2_motor.getCurrentPosition();
                tps = (shoot2Pos - prevShoot2Pos) / (rt - (nextPID-PIDTime));

                prevShoot2Pos = shoot2Pos;

                // 'Schedule' the next PID check
                nextPID = rt + PIDTime;
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Lag:", "avg: %.2f ms, max: %2f ms", ((rt/iter)*1000.0), maxLag);
            telemetry.addData("Shooter:", "%.3f, %.0f", shooterSet[shooterIdx], (tps/(28.0/2.0))*60.0);
            telemetry.update();
        }

        // Stop all power
        drive.setMotorPowers(0,0,0,0);
    }
}
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

import android.graphics.Color;

import Inception.FreightFrenzy.RRMechBot.SlideHeightTeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
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

@TeleOp(name="RedRRMech_MJ", group="Linear Opmode")
public class RRMech_Teleop_MJ extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private static ExpansionHubEx expansionHub1;
    //private static ExpansionHubEx expansionHub2;
    private static DcMotor intake_motor;
    private static DcMotorEx slide_motor;
    private static Servo bucket, duckL, duckR;
    private String className = this.getClass().getSimpleName().toLowerCase();
    public boolean RedAlliance = true;
    private boolean intakeAssist = false;
    double intakeWait = 0.25;

    // Now in the intakeSet array
    //double MAX_INTAKE_POWER = 0.6;

    //private BotLog logger = new BotLog();
    private boolean enableCSVLogging = false;

    private RRMechBot robot = new RRMechBot(true);

    // Mech drive related variables
    int[] speedIdx = new int[] {0, 0};
    double[] speedModifier = new double[] {0.75, 0.80};
    boolean[] FOD = new boolean[] {true, true};
    double[] forward = new double[2], strafe = new double[2], rotate = new double[2];
    double[] prevForward = new double[2], prevStrafe = new double[2], prevRotate = new double[2];
    double[] prevTime = new double[2];
    double maxForwardChange=4.0, maxRotateChange=5.0, maxStrafeChange=3.0;
    boolean smoothDrive = true;
    private TapeMeasureV4 tape = new TapeMeasureV4();

    double l_f_motor_power;
    double l_b_motor_power;
    double r_f_motor_power;
    double r_b_motor_power;

    double theta, r_speed, new_x, new_y, degrees;
    double[] adjustAngle = {180.0,180.0};

    double bucketFullTime = 0;
    boolean bucketFull=false,prevBucketFull=false;

    double aTime = 0;

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
        short pad1 = 0;
        short pad2 = 1;
        short padIdx = pad1;

        boolean[] lBumpPrev = new boolean[]{false, false};
        boolean[] rBumpPrev = new boolean[]{false, false};
        boolean[] lTrigPrev = new boolean[]{false, false};
        boolean[] lTrig = new boolean[]{false, false};
        boolean[] rTrigPrev = new boolean[]{false, false};
        boolean[] rTrig = new boolean[]{false, false};
        boolean[] backPrev = new boolean[]{false, false};
        boolean[] aPrev = new boolean[]{false, false};
        boolean[] bPrev = new boolean[]{false, false};
        boolean[] xPrev = new boolean[]{false, false};
        boolean[] yPrev = new boolean[]{false, false};
        boolean[] dUpPrev = new boolean[]{false, false};
        boolean[] dDownPrev = new boolean[]{false, false};
        boolean[] dLeftPrev = new boolean[]{false, false};
        boolean[] dRightPrev = new boolean[]{false, false};
        boolean[] guidePrev = new boolean[]{false, false};
        boolean[] optionsPrev = new boolean[]{false, false};


        final double DUCK_STOP = 0.5;
        final double MAX_DUCK_SPEED = 0.5;
        final double MAX_DUCK_CHANGE = 2.0;   // Change per second
        double prevDuckTime = 0;
        double duckRequest = DUCK_STOP;

        double[] bucketRequest = {robot.bucketDrive, robot.bucketDrive};
        double bucketAllowed = robot.bucketDrive;

        double[] intakeSet = {0.0, 0.75};
        int intakeIdx=0;
        double currIntakePower = 0.0;

        // 1:1 slide
        //final double SLIDE_INTAKE = 1.0, SLIDE_DRIVE = 0.9, SLIDE_LOW = 0.8, SLIDE_SHARED = 0.73, SLIDE_MED = 0.5, SLIDE_HIGH = 0.0;
        // 2:1 slide
        //final double SLIDE_INTAKE = (1.0-1.0)*.4+.3, SLIDE_DRIVE = (1.0-0.9)*.4+.3, SLIDE_LOW = (1.0-0.8)*.4+.3, SLIDE_SHARED = (1.0-0.73)*.4+.3, SLIDE_MED = (1.0-0.5)*.4+.3, SLIDE_HIGH = (1.0-0.0)*.4+.3;
        //double[] slideSet = {SLIDE_INTAKE, SLIDE_DRIVE, SLIDE_LOW, SLIDE_SHARED, SLIDE_MED, SLIDE_HIGH};
        //final int SLIDE_INTAKE_IDX = 0, SLIDE_DRIVE_IDX = 1, SLIDE_HIGH_IDX = slideSet.length-1;
        SlideHeightTeleOp slideLevel = SlideHeightTeleOp.Drive;
        //double slideRequest=robot.slideTargetsTeleOp[slideLevel];
        SlideHeightTeleOp prevSlideLevel = slideLevel;
        double futureSlideTime=0;
        double futureBucketTime=0;

        double prevLTrigVal=0.0;
        double prevRTrigVal=0.0;

        double maxLag, prt, rt, nextLog = 0.0, iter=0.0;
        boolean gp1Present = true, gp2Present = true, slidePressed = false;

        double maxPwr = 0.0;

        double tps=0.0;

        double now;
        double deltaT;

        double colorDist=0, colorUpperDist=0;

        if (enableCSVLogging) {
            // Enable debug logging
            robot.logger.LOGLEVEL |= robot.logger.LOGDEBUG;
        }

        //robot.logger.LOGLEVEL |= robot.logger.LOGDEBUG;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //robot.acquireHW(hardwareMap);
        robot.init(hardwareMap, 0.5, false);
        //robot.initAutonomous(this);  // Temporary for test only


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
        //intake_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        //intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intake_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slide_motor = robot.slide_motor;
        // Zero out the wobble motor in the auto init
        //slide_motor.setPower(0.0);
        //slide_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        //slide_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //slide_motor.setTargetPosition(0);
        //slide_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //// This is for the in-person autos.  Need to see if the non-in-person can handle these settings...
        //// Note that the RUN_TO_POSITION uses both the 'P' of the setPosition API
        //// and the IDF of the setVelocity API
        //slide_motor.setPositionPIDFCoefficients(5.0);
        //slide_motor.setVelocityPIDFCoefficients(5.0,0.5,3.0,11.1);

        duckL = robot.duckL;
        duckR = robot.duckR;
        bucket = robot.bucket;

        if (className.contains("blue")) { RedAlliance = false; }

        // Setup tapeMeasure object
        tape.init(this, robot, gamepad2, RedAlliance);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Alliance:", "%s", RedAlliance ? "RED" : "BLUE" );
        telemetry.addData("Status", "Waiting for start...");
        tape.telemetry( telemetry );
        telemetry.update();
        /*************************************************************/
        /************ No movement allowed during init! ***************/
        /************ No movement allowed during init! ***************/
        /************ No movement allowed during init! ***************/
        /*************************************************************/

        waitForStart();
        tape.setPosition(tape.TAPE_DRIVE);
        runtime.reset();
        iter = 0.0;
        prt = rt = maxLag = 0.0;
        Gamepad gamepad;

        // Controls:
        // Sticks == Robot drive
        // Dpad left/right == slow turn
        // Dpad up/down == Slide up/down (discrete positions)
        //
        // Right bumper == Start/Stop intake
        // Left bumper == Reverse/unreverse intake (only if running)
        //
        // Right trigger == dump bucket
        // Left trigger == Toggle between drive/intake position
        //
        // 'a' == Go to drive position
        // 'b' == Run duck wheels
        // 'x' == Robot speed
        // 'y' == Go to top slide position
        // 'back' == toggle FOD vs regular drive

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (!gp1Present) {
                if (!gamepad1.atRest()) {
                    gp1Present = true ;
                }
            }

            prt = rt ;
            rt = runtime.seconds() ;
            maxLag = Math.max(maxLag, ((rt-prt)*1000.0));
            iter += 1;

            if (gp1Present) {
                gamepad= gamepad1;
                padIdx = pad1;

                // lTrig is now a boolean
                if (lTrigPrev[padIdx] && (gamepad.left_trigger < 0.30)) {
                    lTrig[padIdx] = false;
                }
                if (!lTrigPrev[padIdx] && (gamepad.left_trigger > 0.70)) {
                    lTrig[padIdx] = true;
                }

                // 'lTrig': Toggle between drive and intake
                if (lTrig[padIdx]) {
                    if (!lTrigPrev[padIdx]) {
                        if (!slidePressed) {
                            // On the first press, INTAKE is not safe, force DRIVE.
                            slideLevel = SlideHeightTeleOp.Drive;
                            slidePressed = true;
                        } else {
                            if (slideLevel != SlideHeightTeleOp.Drive) {
                                slideLevel = SlideHeightTeleOp.Drive;
                            } else {
                                slideLevel = SlideHeightTeleOp.Intake;
                                intakeIdx = 0;
                                intake_motor.setPower(intakeSet[intakeIdx]);
                                currIntakePower = intakeSet[intakeIdx];
                            }
                        }
                        //slideRequest = slideSet[slideLevel];
                        lTrigPrev[padIdx] = true;
                    }
                } else {
                    lTrigPrev[padIdx] = false;
                }

                // Right trigger is dump block
                // The check on 'prev' is to make sure we set to '0' when we release
                if ((gamepad.right_trigger > 0.0) || (prevRTrigVal > 0.0)) {
                    prevRTrigVal = gamepad.right_trigger;

                    // Give some slop of 10% depression to avoid accidental presses
                    // 10% press is really '0' so subtract 0.1 but don't let it go negative
                    // Then scale back to 100%
                    final double travelDistance = (robot.bucketDrive - robot.bucketDumpTeleOp) * (1.1 * Math.max(0,  (gamepad.right_trigger - 0.1)));

                    bucketRequest[padIdx] = robot.bucketDrive - travelDistance;
                    //bucket.setPosition(bucketIntake - travelDistance);
                } else {
                    bucketRequest[padIdx] = robot.bucketDrive;
                }

                if(intakeAssist) {
                    // If the intake is running in forward and we're at intake level
                    if ((currIntakePower > 0.0) && (slideLevel == SlideHeightTeleOp.Intake)) {
                        // Check for an element in the bucket
                        colorDist = robot.color.getDistance(DistanceUnit.CM);
                        if (colorDist > 2.0) {
                            bucketFull = false;
                        } else {
                            if (bucketFull == false) {
                                bucketFullTime = runtime.seconds();
                                bucketFull = true;
                            } else {
                                // If we've been full for > intakeWait seconds
                                if ((runtime.seconds() - bucketFullTime) > intakeWait) {
                                    // And we're not holding the button down
                                    if (!gamepad.right_bumper) {
                                        // Turn the intake off
                                        intake_motor.setPower(-intakeSet[1]);
                                        sleep(100);
                                        intakeIdx = 0;
                                        intake_motor.setPower(intakeSet[intakeIdx]);
                                        currIntakePower = intakeSet[intakeIdx];
                                        if (robot.color.getDistance(DistanceUnit.CM) < 2.0) {
                                            slideLevel = SlideHeightTeleOp.Drive;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                // Reverse the intake
                if (gamepad.left_bumper) {
                    if (!lBumpPrev[padIdx]) {
                        // Reverse polarity on motor
                        if (currIntakePower >= -0.0) {
                            intake_motor.setPower(-intakeSet[1]);
                            currIntakePower = -intakeSet[1];
                        } else {
                            intake_motor.setPower(0);
                            currIntakePower = 0;
                        }
                        intakeIdx = 0;
                        lBumpPrev[padIdx] = true;
                    }
                } else {
                    lBumpPrev[padIdx] = false;
                }

                // Start/stop the intake
                if (gamepad.right_bumper) {
                    if (!rBumpPrev[padIdx]) {
                        intakeIdx = (intakeIdx + 1) % intakeSet.length;
                        intake_motor.setPower(intakeSet[intakeIdx]);
                        currIntakePower = intakeSet[intakeIdx];
                        rBumpPrev[padIdx] = true;
                    }
                } else {
                    rBumpPrev[padIdx] = false;
                }

                // 'back': Toggle FOD mode
                if (gamepad.back) {
                    if (!backPrev[padIdx]) {
                        FOD[padIdx] = !FOD[padIdx];
                        backPrev[padIdx] = true;

                        if (FOD[padIdx]) {
                            // Set the 'front' towards the front of the bot
                            adjustAngle[padIdx] = Math.toDegrees(drive.getRawExternalHeading());
                        }
                    }
                } else {
                    backPrev[padIdx] = false;
                }

                // 'guide' button controls Alliance and intake assist
                if (gamepad.guide) {
                    if (!guidePrev[padIdx]) {
                        if(RedAlliance && intakeAssist) {
                            RedAlliance = true;
                            intakeAssist = false;
                        } else if (RedAlliance && !intakeAssist) {
                            RedAlliance = false;
                            intakeAssist = true;
                        } else if (!RedAlliance && intakeAssist) {
                            RedAlliance = false;
                            intakeAssist = false;
                        } else if (!RedAlliance && !intakeAssist) {
                            RedAlliance = true;
                            intakeAssist = true;
                        }
                        guidePrev[padIdx] = true;
                    }
                } else {
                    guidePrev[padIdx] = false;
                }

                // 'a': Slide to drive position
                if (gamepad.a) {
                    if (!slidePressed) {
                        // On the first press, DRIVE is OK.
                        slidePressed = true;
                    }
                    if (!aPrev[padIdx]) {
                        slideLevel = SlideHeightTeleOp.Drive;
                        //slideRequest = slideSet[slideLevel];

                        aPrev[padIdx] = true;
                        aTime = runtime.seconds();
                    } else {
                        double aElapsed = (runtime.seconds() - aTime);
                        // You've been holding the 'a' button for 1 seconds, go to intake height
                        if((aTime != 0) && (aElapsed > 1.0)) {

                            slideLevel = SlideHeightTeleOp.Intake;
                            intakeIdx = 0;
                            intake_motor.setPower(intakeSet[intakeIdx]);
                            currIntakePower = intakeSet[intakeIdx];

                            // Now loop here until releasing 'a' button
                            while( gamepad.a ) {
                                aElapsed = (runtime.seconds() - aTime);
                                // Now you've been holding for 2 seconds, try to zero out
                                if ((aTime != 0) && (aElapsed > 2.0)) {

                                    // Go to bucketZero intake position to prevent interference with
                                    // intake but avoid a straight down bucket
                                    bucket.setPosition(robot.bucketZero);
                                    sleep(250);

                                    // You've been holding the 'a' button for 2 seconds
                                    // Try to zero the lift
                                    // Stop and switch to ENCODER mode
                                    slide_motor.setPower(0.0);
                                    slide_motor.setDirection(DcMotorSimple.Direction.REVERSE);
                                    slide_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                                    slide_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                    slide_motor.setPower(-0.25);

                                    // Wait while the button is held
                                    while (gamepad.a) {
                                        sleep(10);
                                    }

                                    // Once the button is released, change back to position mode and set to drive
                                    slide_motor.setPower(0.0);
                                    slide_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                    slide_motor.setDirection(DcMotorSimple.Direction.REVERSE);
                                    slide_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                                    slide_motor.setTargetPosition(0);
                                    slide_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                    slide_motor.setPositionPIDFCoefficients(5.0);
                                    slide_motor.setVelocityPIDFCoefficients(5.0, 3.5, 2.0, 12.0);
                                }
                            }

                            slideLevel = SlideHeightTeleOp.Drive;

                            aPrev[padIdx] = false;
                            aTime = 0;
                        }
                    }
                } else {
                    aPrev[padIdx] = false;
                }

                // gamped 'b' is combined duck wheel managed below

                // 'x': Speed toggle
                if (gamepad.x) {
                    if (!xPrev[padIdx]) {
                        speedIdx[padIdx] = (int)( (speedIdx[padIdx] + 1) % speedModifier.length);
                        /*
                        if (( intakeSet[1] == 0.6) && (intakeWait == 0.25)) {
                            intakeSet[1] = 0.75;
                            intakeWait = 0.5;
                        } else if (( intakeSet[1] == 0.75) && (intakeWait == 0.5)) {
                            intakeSet[1] =  0.75;
                            intakeWait = 0.25;
                        } else if (( intakeSet[1] == 0.75) && (intakeWait == 0.25)) {
                            intakeSet[1] =  0.6;
                            intakeWait = 0.25;
                        } else if (( intakeSet[1] == 0.6) && (intakeWait == 0.25)) {
                            intakeSet[1] =  0.75;
                            intakeWait = 0.5;
                        }
                        */
                        xPrev[padIdx] = true;
                    }
                } else {
                    xPrev[padIdx] = false;
                }

                // 'y': Slide to high position
                if (gamepad.y) {
                    if (!slidePressed) {
                        // On the first press, up is safe.  Just do it.
                        slidePressed = true;
                    }
                    if (!yPrev[padIdx]) {
                        slideLevel = SlideHeightTeleOp.HighDrop;
                        //slideRequest = slideSet[slideLevel];
                        yPrev[padIdx] = true;
                    }
                } else {
                    yPrev[padIdx] = false;
                }

                // 'up' slide higher
                if (gamepad.dpad_up) {
                    if (!slidePressed) {
                        // On the first press, 'up' is safe.  Just go up.
                        slidePressed = true;
                    }
                    if (!dUpPrev[padIdx]) {
                        slideLevel = SlideHeightTeleOp.values()[
                                Math.min((slideLevel.ordinal() + 1), robot.slideTargetsTeleOp.length - 1)
                        ];
                        //slideRequest = slideSet[slideLevel];
                        dUpPrev[padIdx] = true;
                    }
                } else {
                    dUpPrev[padIdx] = false;
                }

                // 'down' slide lower
                if (gamepad.dpad_down) {
                    if (!slidePressed) {
                        // On the first press, down is not safe.  Just go to DRIVE.
                        slidePressed = true;
                        slideLevel = SlideHeightTeleOp.Drive;
                    } else {
                        if (!dDownPrev[padIdx]) {
                            slideLevel = SlideHeightTeleOp.values()[
                                    Math.max((slideLevel.ordinal() - 1), 0)];

                            // Turn off Intake if you are going into intake position to avoid hooking the bucket
                            if (slideLevel == SlideHeightTeleOp.Intake ){
                                intakeIdx = 0;
                                intake_motor.setPower(intakeSet[intakeIdx]);
                                currIntakePower = intakeSet[intakeIdx];
                            }

                            //slideRequest = slideSet[slideLevel];
                            dDownPrev[padIdx] = true;
                        }
                    }
                } else {
                    dDownPrev[padIdx] = false;
                }

                // 'dpad left' = slow turn code below
                // 'dpad right' = slow turn code below
            }

            // If the tape is not in a safe position, then lock the slide to DRIVE or INTAKE
            if(!tape.SafePosition()) {
                if(( slideLevel != SlideHeightTeleOp.Drive) && (slideLevel != SlideHeightTeleOp.Intake)) {
                    slideLevel = SlideHeightTeleOp.Drive;
                }
            }

            // Only do this after someone has actually pressed a button.
            if (slidePressed) {
                // Manage the allowed and requested bucket positions.
                if (slideLevel.ordinal() > SlideHeightTeleOp.Drive.ordinal()) {
                    // We're above the drive position, pretty much anything goes here
                    bucketAllowed = bucketRequest[pad1];
                } else if (slideLevel == SlideHeightTeleOp.Drive) {
                    bucketAllowed = robot.bucketDrive;
                } else {
                    bucketAllowed = robot.bucketIntake;
                }

                // DRIVE to INTAKE --> Bucket first, then slide in .1s
                if ((prevSlideLevel == SlideHeightTeleOp.Drive) && (slideLevel == SlideHeightTeleOp.Intake)) {
                    futureSlideTime = runtime.seconds() + 0.1;
                }
                // INTAKE to HIGHER --> Slide first, then bucket in .25s
                if ((prevSlideLevel == SlideHeightTeleOp.Intake)) {
                    futureBucketTime = runtime.seconds() + 0.0;
                }

                if (slideLevel == SlideHeightTeleOp.HighDrop) {
                    // If we move to the HIGH Position, go fast because its a long ways away
                    robot.setSlidePositionTeleOp(slideLevel, 0.95);
                } else {
                    // If we're not waiting, move there
                    if (futureSlideTime < runtime.seconds()) {
                        robot.setSlidePositionTeleOp(slideLevel);
                    }
                }
                // If we're not waiting, move there
                if (futureBucketTime < runtime.seconds()) {
                    bucket.setPosition(bucketAllowed);
                }
            }
            prevSlideLevel = slideLevel ;

            // Manage combined duck wheel on 'b' buttons
            now = runtime.seconds();
            deltaT = now - prevDuckTime;
            if (gamepad1.b) {
                duckRequest = Math.min(MAX_DUCK_SPEED, duckRequest + (MAX_DUCK_CHANGE * deltaT));
            } else {
                duckRequest = 0;
            }
            prevDuckTime = now;
            if( RedAlliance ) {
                duckL.setPosition(DUCK_STOP - duckRequest);
                duckR.setPosition(DUCK_STOP - duckRequest);
            } else {
                duckL.setPosition(DUCK_STOP + duckRequest);
                duckR.setPosition(DUCK_STOP + duckRequest);
            }

            // Now handle driving commands below for both gamepads.
            gamepad = gamepad1;
            padIdx = pad1;
            // Read the controller 1 (driver) stick positions
            strafe[padIdx] = gamepad.left_stick_x;
            forward[padIdx] = -gamepad.left_stick_y;
            rotate[padIdx] = gamepad.right_stick_x;

            /* High Precision Mode is 25% power */
            if( gamepad.left_stick_button ) {
                strafe[padIdx] *= .25;
                forward[padIdx] *= .25;
            }

            if( gamepad.right_stick_button ) {
                rotate[padIdx] *= .25;
            }

            // Convert to FOD as soon as possible so all the smooth drive code applies correctly
            // This code is terrible.
            // Beware the function calls that pass information in and out via global vars
            if (FOD[pad1]) {
                // Get the current robot heading
                degrees = Math.toDegrees(drive.getRawExternalHeading());

                if (FOD[pad1]) {
                    // Convert the X/Y Cartesion for strafe and forward into Polar
                    CarToPol(strafe[pad1], forward[pad1]);
                    // Rotate the Polar coordinates by the robot's heading
                    theta -= AngleUnit.DEGREES.normalize(degrees - adjustAngle[pad1]);
                    // Convert the new Polar back into Cartesian
                    PolToCar(r_speed);
                    // Replace the strafe and forward power with translated values
                    strafe[pad1] = new_x;
                    forward[pad1] = new_y;
                    // Now the robot moves in orientation of the field
                }
            }

            // A little bump to make strafe/rotate feel more responsive
            strafe[padIdx] *= 1.25;
            rotate[padIdx] *= 1.5;

            if(smoothDrive) {
                now = runtime.seconds();
                deltaT = now - prevTime[padIdx];

                if ((prevStrafe[padIdx] != 0) && (strafe[padIdx] != 0) && (Math.signum(prevStrafe[padIdx]) != Math.signum(strafe[padIdx]))) {
                    strafe[padIdx] = 0;
                }
                if ((prevForward[padIdx] != 0) && (forward[padIdx] != 0) && (Math.signum(prevForward[padIdx]) != Math.signum(forward[padIdx]))) {
                    forward[padIdx] = 0;
                }
                if ((prevRotate[padIdx] != 0) && (rotate[padIdx] != 0) && (Math.signum(prevRotate[padIdx]) != Math.signum(rotate[padIdx]))) {
                    rotate[padIdx] = 0;
                }

                if (Math.abs(strafe[padIdx]) > 0.20) {
                    if (prevStrafe[padIdx] < strafe[padIdx]) {
                        strafe[padIdx] = Math.min(strafe[padIdx], prevStrafe[padIdx] + (maxStrafeChange * deltaT));
                    } else {
                        strafe[padIdx] = Math.max(strafe[padIdx], prevStrafe[padIdx] - (maxStrafeChange * deltaT));
                    }
                }
                if (Math.abs(forward[padIdx]) > 0.20) {
                    if (prevForward[padIdx] < forward[padIdx]) {
                        forward[padIdx] = Math.min(forward[padIdx], prevForward[padIdx] + (maxForwardChange * deltaT));
                    } else {
                        forward[padIdx] = Math.max(forward[padIdx], prevForward[padIdx] - (maxForwardChange * deltaT));
                    }
                }
                if (Math.abs(rotate[padIdx]) > 0.20) {
                    if (prevRotate[padIdx] < rotate[padIdx]) {
                        rotate[padIdx] = Math.min(rotate[padIdx], prevRotate[padIdx] + (maxRotateChange * deltaT));
                    } else {
                        rotate[padIdx] = Math.max(rotate[padIdx], prevRotate[padIdx] - (maxRotateChange * deltaT));
                    }
                }
                prevTime[padIdx] = now;
                prevStrafe[padIdx] = strafe[padIdx];
                prevForward[padIdx] = forward[padIdx];
                prevRotate[padIdx] = rotate[padIdx];

                // Remove 15% deadzone
                if (strafe[padIdx] >= 0.025) {
                    strafe[padIdx] = (strafe[padIdx] * 0.85) + 0.15;
                }
                if (forward[padIdx] >= 0.025) {
                    forward[padIdx] = (forward[padIdx] * 0.85) + 0.15;
                }
                if (rotate[padIdx] >= 0.025) {
                    rotate[padIdx] = (rotate[padIdx] * 0.85) + 0.15;
                }
                if (strafe[padIdx] <= -0.025) {
                    strafe[padIdx] = (strafe[padIdx] * 0.85) - 0.15;
                }
                if (forward[padIdx] <= -0.025) {
                    forward[padIdx] = (forward[padIdx] * 0.85) - 0.15;
                }
                if (rotate[padIdx] <= -0.025) {
                    rotate[padIdx] = (rotate[padIdx] * 0.85) - 0.15;
                }
            }

            // Rotate a little left
            if (gamepad.dpad_left) {
                rotate[padIdx] -= 0.4;
            }
            // Rotate a little right
            if (gamepad.dpad_right) {
                rotate[padIdx] += 0.4;
            }

            // This adds the powers from both controllers together scaled for each controller and FOD
            l_f_motor_power = ((forward[pad1] + strafe[pad1] + rotate[pad1]) * speedModifier[speedIdx[pad1]]);
            l_b_motor_power = ((forward[pad1] - strafe[pad1] + rotate[pad1]) * speedModifier[speedIdx[pad1]]);
            r_f_motor_power = ((forward[pad1] - strafe[pad1] - rotate[pad1]) * speedModifier[speedIdx[pad1]]);
            r_b_motor_power = ((forward[pad1] + strafe[pad1] - rotate[pad1]) * speedModifier[speedIdx[pad1]]);

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

            boolean doRumble = true;
            if (doRumble) {
                // Warn the driver they are moving with bucket down
                if (((Math.abs(l_f_motor_power) > 0.1) ||
                        (Math.abs(l_b_motor_power) > 0.1) ||
                        (Math.abs(r_f_motor_power) > 0.1) ||
                        (Math.abs(r_b_motor_power) > 0.1)) && (slideLevel == SlideHeightTeleOp.Intake)) {
                    gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
                } else {
                    if (gamepad1.isRumbling()) {
                        gamepad1.stopRumble();
                    }
                }
            }

            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            drive.setMotorPowers(l_f_motor_power, l_b_motor_power, r_b_motor_power, r_f_motor_power);

            tape.ManageTape( );

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("", "");
            //telemetry.addData("Slide Goal:", robot.slideTargetsTeleOp[slideLevel.ordinal()]);
            //telemetry.addData("Slide POS: ", slide_motor.getCurrentPosition());
            //telemetry.addData("Bucket Req: ", bucketRequest[pad1]);
            //telemetry.addData("Bucket POS: ", bucketAllowed);
            //telemetry.addData("Heading", "%.1f", Math.toDegrees(drive.getRawExternalHeading()));
            //telemetry.addData("Stick Buttons: ", "%s, %s", gamepad1.left_stick_button, gamepad1.right_stick_button);
            //telemetry.addData("Lower Intake Assist:", "D:%.2f, T:%.1f", colorDist, (bucketFull ? (runtime.seconds()-bucketFullTime) : (0.0)));
            //colorUpperDist = robot.colorUpper.getDistance(DistanceUnit.CM);
            //telemetry.addData("Side Assist:", "D:%.2f", colorUpperDist );
            tape.telemetry( telemetry );
            telemetry.addData("Alliance:", "%s", RedAlliance ? "RED" : "BLUE" );
            telemetry.addData("Intake Assist:", "%s", intakeAssist ? "Enabled" : "Disabled" );
            if( intakeAssist ) {
                telemetry.addData("Intake Delay:", "%s", (intakeWait == 0.25) ? "Fast" : "Slow");
            }
            telemetry.addData("Intake Power:", "%s", (intakeSet[1] > 0.7) ? "High" : "Low" );
            telemetry.addData("Drive Mode:", "%s", FOD[pad1] ? "FOD" : "Regular" );
            telemetry.addData("Motor Power:", "%d%%", (int)(speedModifier[speedIdx[pad1]]*100) );
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
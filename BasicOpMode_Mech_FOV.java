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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

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

@TeleOp(name="Mech FOV", group="Linear Opmode")
public class BasicOpMode_Mech_FOV extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private static ExpansionHubEx expansionHub1;
    //private static ExpansionHubEx expansionHub2;
    private static DcMotorEx l_f_motor, l_b_motor, r_f_motor, r_b_motor;
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

        final double CLAW_OPEN = 0.0, CLAW_CLOSED=1.0;
        final double FLICKER_SHOOT = 0.7, FLICKER_WAIT=0.0;
        final double SHOOTER_NORMAL=0.475, SHOOTER_POWER_SHOT=0.447;
        //wobble stuff
        //final double WOBBLE_TICKS_PER_DEGREE = 5264.0/360.0; // 30 RPM 6mm d-shaft (5202 series)
        //final double WOBBLE_TICKS_PER_DEGREE = 2786.0/360.0; // 60 RPM 6mm d-shaft (5202 series)
        final double WOBBLE_TICKS_PER_DEGREE = 3892.0/360.0; // 43 RPM 8mm REX (5203 series)
        final double[][] wobbleSeq = {
                {CLAW_CLOSED, 0.5,   5*WOBBLE_TICKS_PER_DEGREE},
                {CLAW_OPEN,   0.8, 225*WOBBLE_TICKS_PER_DEGREE},
                {CLAW_CLOSED, 0.8, 225*WOBBLE_TICKS_PER_DEGREE},
                {CLAW_CLOSED, 0.5,  90*WOBBLE_TICKS_PER_DEGREE},
                {CLAW_CLOSED, 0.5, 175*WOBBLE_TICKS_PER_DEGREE},
                {CLAW_OPEN,   0.5, 175*WOBBLE_TICKS_PER_DEGREE},
        };

        double[] flickerSet = {FLICKER_WAIT, FLICKER_SHOOT};
        double[] clawSet = {CLAW_CLOSED, CLAW_OPEN};
        double[] intakeSet = {0.0, MAX_INTAKE_POWER};
        double[] speedSet = {0.75, 1.0};
        double[] shooterSet = {0.0, SHOOTER_NORMAL};
        int flickerIdx=0, clawIdx=0, intakeIdx=0, speedIdx=0, shooterIdx=0, wobbleIdx=0;
        double flickerRelease=0.0, flickerRearm=0.0;
        double prevLTrigVal=0.0;
        double prevRTrigVal=0.0;

        double rt, nextLog = 0.0;

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

        /*************************************************************/
        /************ No movement allowed during init! ***************/
        /************ No movement allowed during init! ***************/
        /************ No movement allowed during init! ***************/
        /*************************************************************/

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        //expansionHub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        //expansionHub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        l_f_motor = hardwareMap.get(DcMotorEx.class,"left_front");
        l_b_motor = hardwareMap.get(DcMotorEx.class,"left_back");
        r_f_motor = hardwareMap.get(DcMotorEx.class,"right_front");
        r_b_motor = hardwareMap.get(DcMotorEx.class,"right_back");
        l_f_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        l_b_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        r_f_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        r_b_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        l_f_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        l_b_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r_f_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r_b_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        l_f_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        l_b_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r_f_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r_b_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake_motor = hardwareMap.dcMotor.get("intake");
        intake_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // The wobble keeps it's encoder value from auto, this shoudl allow us to get back to '0' position
        // in case something bad happened.  DO NOT STOP_AND_RESET_ENCODER here to preserve the '0' postion.
        wobble_motor = hardwareMap.get(DcMotorEx.class,"wobble");
        wobble_motor.setPower(0.0);
        wobble_motor.setDirection(DcMotorSimple.Direction.REVERSE);
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

        if (enableCSVLogging) {
            // Lay down a header for our logging
            //logger.logD("MechFOVCSV", String.format(",rt,heading,lfEnc,lbEnc,rfEnc,rbEnc,lfPwr,lbPwr,rfPwr,rbPwr"));
            logger.logD("MechFOVCSV", String.format(",rt,shoot2Enc,shootPwr"));
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            rt = runtime.seconds() ;

            if (rt > 120) {
                shooterSet[1] = SHOOTER_POWER_SHOT;
            }

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
                prevRTrigVal = gamepad1.right_trigger * 0.30 ;
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
                if(!rBumpPrev[0]) {
                    intakeIdx = (intakeIdx + 1 ) % intakeSet.length ;
                    intake_motor.setPower(intakeSet[intakeIdx]);
                    rBumpPrev[0] = true;
                }
            } else {
                rBumpPrev[0] = false;
            }

            // Change the chassis speed
            if (gamepad1.a) {
                if(!aPrev[0]) {
                    speedIdx = (speedIdx + 1) % speedSet.length;
                    speedModifier[0] = speedSet[speedIdx];
                    aPrev[0] = true;
                }
            } else {
                aPrev[0] = false;
            }

            // Advance the wobble sequence
            if (gamepad1.b) {
                if(!bPrev[0]) {
                    wobbleIdx = (wobbleIdx + 1) % wobbleSeq.length;
                    wobble_motor.setTargetPosition((int)wobbleSeq[wobbleIdx][2]);
                    wobble_motor.setPower(wobbleSeq[wobbleIdx][1]);
                    claw.setPosition(wobbleSeq[wobbleIdx][0]);
                    bPrev[0] = true;
                }
            } else {
                bPrev[0] = false;
            }

            // Backup the wobble sequence
            if (gamepad1.x) {
                if(!xPrev[0]) {
                    if (wobbleIdx == 0) {
                        wobbleIdx = (wobbleSeq.length - 1);
                    } else {
                        wobbleIdx = (wobbleIdx - 1) % wobbleSeq.length;
                    }
                    wobble_motor.setTargetPosition((int)wobbleSeq[wobbleIdx][2]);
                    wobble_motor.setPower(wobbleSeq[wobbleIdx][1]);
                    claw.setPosition(wobbleSeq[wobbleIdx][0]);
                    xPrev[0] = true;
                }
            } else {
                xPrev[0] = false;
            }

            // Let's turn off wobble motor when we're in the parked position.
            if ( (wobble_motor.getCurrentPosition() <= ( 7.0 * WOBBLE_TICKS_PER_DEGREE ) ) && ( wobbleIdx == 0 ) ) {
                wobble_motor.setPower(0);
            }

            // Stop the intake, then reverse power, and start again
            if (gamepad1.y) {
                if(!yPrev[0]) {
                    intake_motor.setPower(0.0);
                    for( int i=0; i < intakeSet.length; i++) { intakeSet[i] *= -1.0; }
                    sleep(250);
                    intake_motor.setPower(intakeSet[intakeIdx]);
                    yPrev[0] = true;
                }
            } else {
                yPrev[0] = false;
            }

            if (gamepad1.dpad_up) {
                if(!dUpPrev[0]) {
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

            if (gamepad1.dpad_down) {
                if(!dDownPrev[0]) {
                    // Reset all indices and assign them
                    flickerIdx = 0;
                    flicker.setPosition(flickerSet[flickerIdx]);

                    wobbleIdx = 0;
                    wobble_motor.setTargetPosition((int)wobbleSeq[wobbleIdx][2]);
                    wobble_motor.setPower(wobbleSeq[wobbleIdx][1]);
                    claw.setPosition(wobbleSeq[wobbleIdx][0]);

                    shooterIdx=0;
                    shoot1_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    shoot2_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    shoot1_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    shoot2_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    shoot1_motor.setPower(shooterSet[shooterIdx]);
                    shoot2_motor.setPower(shooterSet[shooterIdx]);

                    intakeIdx=0;
                    intake_motor.setPower(intakeSet[intakeIdx]);

                    dDownPrev[0] = true;
                }
            } else {
                dDownPrev[0] = false;
            }

            // Read the controller 1 (driver) stick positions
            strafe[0] = gamepad1.left_stick_x;
            forward[0] = -gamepad1.left_stick_y;
            rotate[0] = gamepad1.right_stick_x;

            strafe[1] = gamepad2.left_stick_x;
            forward[1] = -gamepad2.left_stick_y;
            rotate[1] = gamepad2.right_stick_x;

            // Rotate a little left
            if (gamepad1.dpad_left) {
                rotate[0] -= 0.25 ;
            }
            // Rotate a little right
            if (gamepad1.dpad_right) {
                rotate[0] += 0.25 ;
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

            // Send calculated power to wheels
            l_f_motor.setPower(l_f_motor_power);
            l_b_motor.setPower(l_b_motor_power);
            r_f_motor.setPower(r_f_motor_power);
            r_b_motor.setPower(r_b_motor_power);

            // Additional controls

            // Update the logger 10 times/second max
            if (enableCSVLogging) {
                if (rt > nextLog) {
                    // FIXME -- bulk data read returning strange data here.
                    //bulkData1 = expansionHub1.getBulkInputData();
                    //bulkData2 = expansionHub2.getBulkInputData();
                    //logger.logD("MechFOVCSV", String.format(",%f,%f,%d,%d,%d,%d,%f,%f,%f,%f", rt, getHeading(), bulkData1.getMotorCurrentPosition(l_f_motor),bulkData1.getMotorCurrentPosition(l_b_motor), bulkData2.getMotorCurrentPosition(r_f_motor), bulkData2.getMotorCurrentPosition(r_b_motor), l_f_motor_power, l_b_motor_power, r_f_motor_power, r_b_motor_power));

                    int lfPos = l_f_motor.getCurrentPosition();
                    int lbPos = l_b_motor.getCurrentPosition();
                    int rfPos = r_f_motor.getCurrentPosition();
                    int rbPos = r_b_motor.getCurrentPosition();

                    logger.logD("MechTeleopCSV", String.format(",%f,%d,%d,%d,%d,%f,%f,%f,%f", rt, lfPos, lbPos, rfPos, rbPos, l_f_motor_power, l_b_motor_power, r_f_motor_power, r_b_motor_power));
                    nextLog = rt + 0.1;
                }
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
            telemetry.addData("Shooter:", "%.3f, %.0f", shooterSet[shooterIdx], (tps/(28.0/2.0))*60.0);
            telemetry.update();
        }

        // Stop all power
        l_f_motor.setPower(0.0);
        l_b_motor.setPower(0.0);
        r_f_motor.setPower(0.0);
        r_b_motor.setPower(0.0);
    }
}


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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.util.Locale;

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

    BNO055IMU imu,imu2;
    //Orientation angles,angles2;
    double MAX_SHOOTER_POWER = 0.495;
    double MAX_INTAKE_POWER = 1.0;

    private BotLog logger = new BotLog();
    private boolean enableCSVLogging = true;


    // Mech drive related variables
    double theta, r_speed, new_x, new_y;
    double[] speedModifier = new double[] {0.5,0.5};
    double[] adjustAngle = new double[] {0.0,0.0};
    double[] forward = new double[2], strafe = new double[2], rotate = new double[2], degrees = new double[2];
    boolean[] FOD = new boolean[] {false,false};

    double l_f_motor_power;
    double l_b_motor_power;
    double r_f_motor_power;
    double r_b_motor_power;

    double wobble_power;
    double shooter_power;

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

    double getHeading() {
        Orientation angles;

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)));
    }

    public double getQHeading()
    {
        Quaternion q = imu.getQuaternionOrientation();
        q = q.normalized();

        // This code was leveraged from here:
        // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/

        //double t=q.x*q.y + q.z*q.w;
        //double h,a,b;
        //double sqy = q.y * q.y;
        double b;
        double sqx = q.x * q.x;
        double sqz = q.z * q.z;

        // We only need 'b' here since our bot is only intending to rotate in one dimension
        // We don't need to worry about singularities for Quaternion to Euler conversion
        // Because of a single dimension of turning.
        //h = Math.atan2(2 * q.y * q.w - 2 * q.x * q.z, 1 - 2 * sqy - 2 * sqz);
        //a = Math.asin(2 * t);
        b = Math.atan2(2 * q.x * q.w - 2 * q.y * q.z, 1 - 2 * sqx - 2 * sqz);

        // This code is a way to compare our Quaternion vs Euler gyro readings
        /*
        Orientation gyroOrien = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        logger.logD("MechLog",String.format(" getQHeadingNorm: t:%f, h:%f(%f), a:%f(%f), b:%f(%f)",
                     t,
                     -(h/Math.PI)*180,
                     AngleUnit.DEGREES.fromUnit(gyroOrien.angleUnit, gyroOrien.secondAngle),
                     -(a/Math.PI)*180,
                     AngleUnit.DEGREES.fromUnit(gyroOrien.angleUnit, gyroOrien.thirdAngle),
                     -(b/Math.PI)*180,
                     AngleUnit.DEGREES.fromUnit(gyroOrien.angleUnit, gyroOrien.firstAngle)));
         */

        return (-(b/Math.PI)*180.0);
    }



    @Override
    public void runOpMode() {
        //RevBulkData bulkData1, bulkData2;
        // Co-pilot toggles
        boolean lBump2Prev=false, rBump2Prev=false, lTrig2Prev=false, rTrig2Prev=false;
        boolean dDown2Prev = false, dUp2Prev = false, dLeft2Prev = false, dRight2Prev=false;

        // Pilot toggles
        double[] flickerSet = {0.0, 1.0};
        double[] clawSet = {1.0, 0.0};
        double[] intakeSet = {0.0, MAX_INTAKE_POWER};
        int clawPos=0, flickerPos=0, intakePwr=0;

        boolean lBump1Prev=false, rBump1Prev=false, lTrig1Prev=false, rTrig1Prev=false;
        boolean wobblePrev=false, clawPrev=false;
        boolean dDown1Prev = false, dUp1Prev = false, dLeft1Prev = false, dRight1Prev=false;
        double childLock=1.0;
        double currFlickerPos;

        // Elevator controls
        double prt=0.0, rt = 0.0, nextLog = 0.0;

        double maxPwr = 0.0;

        //wobble stuff
        final double WOBBLE_TICKS_PER_DEGREE = 2786.0/360.0;
        final int[] wobbleTargets = {(int)(5*WOBBLE_TICKS_PER_DEGREE),(int)(215*WOBBLE_TICKS_PER_DEGREE), (int)(90*WOBBLE_TICKS_PER_DEGREE), (int)(175*WOBBLE_TICKS_PER_DEGREE)};
        int wobblePos = 0;
        wobble_power = 0.4;

        /** 'Manual' PID or shooter
        // https://en.wikipedia.org/wiki/Ziegler–Nichols_method
        // Ku = 1/100; Tu = .5s @ 1200 RPM
        // Need to mess with this some more I guess?
        MiniPID pid= new MiniPID(0.10*(1.0/100.0),0.025*(1.0/100.0), 0.2*(1.0/100.0), 0.495/((3600.0*(28.0/1.5))/60.0));
        pid.setOutputLimits(0.0,0.7);


        double prevShooterReq=0.0;
        double shooterTarget = ((3600.0*(28.0/1.5))/60.0);
        double shooterStep = ((50.0*(28.0/1.5))/60.0);
        double iters=0.0;
        **/

        double PIDrt, tps=0.0, prevPIDrt=0.0;
        double shoot2Pos, prevShoot2Pos=0.0;
        double nextPID=0.0, PIDTime = 0.1;
        double shooterReq;
        double shooter_tgt_power = 0.475;
        double powerStep = 0.005;

        if (enableCSVLogging) {
            // Enable debug logging
            logger.LOGLEVEL |= logger.LOGDEBUG;
        }

        //logger.LOGLEVEL |= logger.LOGDEBUG;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
        l_f_motor.setVelocityPIDFCoefficients(2.0,0.5,0.0,11.1);
        l_b_motor.setVelocityPIDFCoefficients(2.0,0.5,0.0,11.1);
        r_f_motor.setVelocityPIDFCoefficients(2.0,0.5,0.0,11.1);
        r_b_motor.setVelocityPIDFCoefficients(2.0,0.5,0.0,11.1);

        intake_motor = hardwareMap.dcMotor.get("intake");
        intake_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // The wobble must be left alone here.  If we mess with it, we will potentially
        // lose our '0' position set at the start of auto.
        wobble_motor = hardwareMap.get(DcMotorEx.class,"wobble");
        wobble_motor.setPower(0.0);
        //wobble_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        //wobble_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wobble_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //wobble_motor.setTargetPosition(0);
        //wobble_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobble_motor.setPositionPIDFCoefficients(5.0);
        wobble_motor.setVelocityPIDFCoefficients(2.0,0.5,0.0,11.1);

        shoot1_motor = hardwareMap.get(DcMotorEx.class,"shoot1");
        shoot2_motor = hardwareMap.get(DcMotorEx.class,"shoot2");
        shoot1_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot2_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot1_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shoot2_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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

        if (false) {

            /*
            PIDFCoefficients pidfWPOrig = wobble_motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
            PIDFCoefficients pidfWEOrig = wobble_motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            */

            wobble_motor.setPositionPIDFCoefficients(5.0);
            wobble_motor.setVelocityPIDFCoefficients(2.0,0.5,0.0,11.1);

            /*
            // display info to user.
            PIDFCoefficients pidfWPAct = wobble_motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
            PIDFCoefficients pidfWEAct = wobble_motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("P,I,D,F (Orig Pos)", "%.02f, %.02f, %.02f, %.2f",
                    pidfWPOrig.p, pidfWPOrig.i, pidfWPOrig.d, pidfWPOrig.f);
            telemetry.addData("P,I,D,F (Orig Enc)", "%.02f, %.02f, %.02f, %.2f",
                    pidfWEOrig.p, pidfWEOrig.i, pidfWEOrig.d, pidfWEOrig.f);
            telemetry.addData("P,I,D,F (Act Pos)", "%.02f, %.02f, %.02f, %.2f",
                    pidfWPAct.p, pidfWPAct.i, pidfWPAct.d, pidfWPAct.f);
            telemetry.addData("P,I,D,F (Act Enc)", "%.02f, %.02f, %.02f, %.2f",
                    pidfWEAct.p, pidfWEAct.i, pidfWEAct.d, pidfWEAct.f);
            */

            /*
            PIDFCoefficients pidfValsShooter = shoot1_motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            */

            // These PIDF values seem pretty good.
            //PIDFCoefficients pidFNew = new PIDFCoefficients(125.0, 2.5, 5.0, 4.0);
            // Raising 'F' increases overshoot a lot
            // Raising 'I' increases overshoot a lot
            // 'P' is in a sweet-spot, could go down to 75 and still be OK
            // 'D' didn't make a ton of different, not sure that is tuned properly
            // Quick spin-up and recovery.  There may be a little overshoot just after a shot.
            //PIDFCoefficients pidFNew = new PIDFCoefficients(150.0, 2.5, 5.0, 4.0);
            //shoot1_motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidFNew);
            //shoot2_motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidFNew);
            // 'P' might need to be 100 here.
            shoot1_motor.setVelocityPIDFCoefficients(125.0, 2.5, 5.0, 4.0);
            shoot2_motor.setVelocityPIDFCoefficients(125.0, 2.5, 5.0, 4.0);

            /**
            // display info to user.
            PIDFCoefficients pidfAct1 = shoot1_motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            PIDFCoefficients pidfAct2 = shoot2_motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("P,I,D,F (orig Pos)", "%.02f, %.02f, %.02f, %.2f",
                    pidfValsShooter.p, pidfValsShooter.i, pidfValsShooter.d, pidfValsShooter.f);
            telemetry.addData("P,I,D,F (new)", "%.02f, %.02f, %.02f, %.2f",
                    pidFNew.p, pidFNew.i, pidFNew.d, pidFNew.f);
            telemetry.addData("P,I,D,F (act1)", "%.02f, %.02f, %.02f, %.2f",
                    pidfAct1.p, pidfAct1.i, pidfAct1.d, pidfAct1.f);
            telemetry.addData("P,I,D,F (act2)", "%.02f, %.02f, %.02f, %.2f",
                    pidfAct2.p, pidfAct2.i, pidfAct2.d, pidfAct2.f);

            telemetry.update();
            sleep(5000);
            **/
        }


        imu = initIMU("imu");
        //imu2 = initIMU("imu 1");

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Waiting for start...");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Initialize all servos
        // This must be done after the 'start' button is pressed or things may be moving during the match which shouldn't move.
        flicker.setPosition(flickerSet[flickerPos]);

        // We may want a button to do this init for the arm/claw increase they are hung up on something.
        // one thing to do would be to start in a special position that can only be set/reached in init.  Then just use the same
        // button that advances state to go to state '0'.
        claw.setPosition(clawSet[clawPos]);
        wobble_motor.setTargetPosition(0);

        if (enableCSVLogging) {
            // Lay down a header for our logging
            //logger.logD("MechFOVCSV", String.format(",rt,heading,lfEnc,lbEnc,rfEnc,rbEnc,lfPwr,lbPwr,rfPwr,rbPwr"));
            logger.logD("MechFOVCSV", String.format(",rt,shoot2Enc,shootPwr"));
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Begin controller 2 (co-pilot)
            if (gamepad2.right_bumper) {
                if(!lBump2Prev) {
                    // Nothing right now
                    lBump2Prev = true;
                }
            } else {
                lBump2Prev = false;
            }

            if (gamepad2.left_bumper) {
                if (!rBump2Prev){
                    // Nothing right now
                    rBump2Prev = true;
                }
            } else {
                rBump2Prev = false;
            }

            if (gamepad2.left_trigger > 0.9) {
                if (!lTrig2Prev){
                    // Nothing right now
                    lTrig2Prev = true;
                }
            } else if (gamepad2.left_trigger < 0.1) {
                lTrig2Prev = false;
            }

            rt = runtime.seconds();

            // Let the REV hub PID manage our shooter motors...
            // Read the trigger value
            shooterReq = gamepad1.left_trigger;
            if (shooterReq > 0.7) {
                if ( shooter_power == 0 ) {
                    shooter_power = shooter_tgt_power;
                    shoot1_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    shoot2_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    shoot1_motor.setPower(shooter_power);
                    shoot2_motor.setPower(shooter_power);
                }
            } else if (shooterReq < 0.3) {
                if ( shooter_power > 0.0 ) {
                    shooter_power = 0.0;
                    shoot1_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    shoot2_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    shoot1_motor.setPower(shooter_power);
                    shoot2_motor.setPower(shooter_power);
                }
            }

            if ( nextPID < rt ) {

                shoot2Pos = shoot2_motor.getCurrentPosition();
                PIDrt = runtime.seconds();

                tps = (shoot2Pos - prevShoot2Pos) / (PIDrt - prevPIDrt);

                prevShoot2Pos = shoot2Pos;
                prevPIDrt = PIDrt;

                // 'Schedule' the next PID check
                nextPID = rt + PIDTime;
            }


            /*** 'Manual' shooter PID
            prt = rt;
            rt = runtime.seconds();
            if ((int)(prt/5.0) != (int)(rt/5.0)){
                iters = 1;
            } else{
                iters += 1;
            }

            // Read the trigger value
            shooterReq = gamepad1.left_trigger;
            // If the request is non-zero, check PID
            if ( shooterReq != 0 ) {
                // Only check PID periodically, not every loop
                if ( nextPID < rt ) {
                    // Get current values
                    PIDrt = rt;
                    shoot2Pos = shoot2_motor.getCurrentPosition();

                    // If we just went off to on, we should prime the values for math
                    if (prevShooterReq == 0) {
                        // Reset PID
                        pid.reset();
                        // Update current TPS rate
                        prevPIDrt = PIDrt;
                        prevShoot2Pos = shoot2Pos;
                        shoot2Pos = shoot2_motor.getCurrentPosition();
                        PIDrt = runtime.seconds();
                    }

                    // Compute current rev per second
                    tps = (shoot2Pos - prevShoot2Pos) / (PIDrt - prevPIDrt);

                    // Save current values for the next loop
                    prevPIDrt = PIDrt;
                    prevShoot2Pos = shoot2Pos;

                    // Now adjust the power based on our PID
                    shooter_power = pid.getOutput(tps, shooterReq * shooterTarget);

                    // 'Schedule' the next PID check
                    nextPID = rt + PIDTime;

                    // Program the power (assume it changed)
                    shoot1_motor.setPower(Math.max(-0.7, Math.min(0.7, shooter_power)));
                    shoot2_motor.setPower(Math.max(-0.7, Math.min(0.7, shooter_power)));
                }
            } else {
                // 'else' means no shooter power request.  If the power is currently non-0, set to '0'
                if (shooter_power != 0) {
                    shooter_power = 0;
                    shoot1_motor.setPower(Math.max(-0.7, Math.min(0.7, shooter_power)));
                    shoot2_motor.setPower(Math.max(-0.7, Math.min(0.7, shooter_power)));
                }
            }

            // Recored the current request to handle special case for going off-on-off
            prevShooterReq = shooterReq;
            ***/

            if (gamepad1.dpad_right) {
                if(!clawPrev) {
                    clawPos++; clawPos %= 2;
                    claw.setPosition(clawSet[clawPos]);
                    clawPrev = true;
                }
            } else {
                clawPrev = false;
            }

            if(gamepad1.dpad_left){
                if(!wobblePrev) {

                    wobblePos++; wobblePos %= 4;
                    //wobble_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if(wobblePos == 0) {
                        wobble_motor.setPower(0.4);
                    } else if(wobblePos == 3) {
                        wobble_motor.setPower(0.3);
                    } else {
                        wobble_motor.setPower(wobble_power);
                    }

                    wobble_motor.setTargetPosition(wobbleTargets[wobblePos]);

                    wobblePrev = true;
                }
            } else {
                wobblePrev = false;
            }

            if(wobble_motor.getCurrentPosition() <= (10.0*WOBBLE_TICKS_PER_DEGREE) && wobblePos == 0){
                wobble_motor.setPower(0);
            }

            // Toggle the child lock between 0.0 and 1.0
            /*
            if (gamepad1.dpad_left) {
                if (!dLeft1Prev) {
                    if (childLock == 0.0) {
                        childLock = 1.0;
                    } else {
                        childLock = 0.0;
                    }
                    dLeft1Prev = true;
                }
            } else {
                dLeft1Prev = false;
            }
            */

            //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            ////angles2   = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //degrees[1] = degrees[0] = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

            // Get the current robot heading
            degrees[1] = degrees[0] = getHeading();

            // Read the controller 1 (driver) stick positions
            strafe[0] = gamepad1.left_stick_x;
            forward[0] = -gamepad1.left_stick_y;
            rotate[0] = gamepad1.right_stick_x;

            // Read the controller 2 (co-pilot) stick positions
            strafe[1] = gamepad2.left_stick_x;
            forward[1] = -gamepad2.left_stick_y;
            rotate[1] = gamepad2.right_stick_x;


            // Shooter power control
            if (gamepad1.dpad_up) {
                // If we detected dpad released since the last press
                if (!dUp1Prev) {
                    //shooterTarget += shooterStep;
                    shooter_tgt_power += powerStep;
                    // Prevent this path again until the dpad is released
                    dUp1Prev = true;
                }
            } else {
                // Detected the dpad released
                dUp1Prev = false;
            }

            //  Shooter power control
            if (gamepad1.dpad_down) {
                // If we detected dpad released since the last press
                if (!dDown1Prev) {
                    //shooterTarget -= shooterStep;
                    shooter_tgt_power -= powerStep;
                    // Prevent this path again until the dpad is released
                    dDown1Prev = true;
                }
            } else {
                // Detected the dpad released
                dDown1Prev = false;
            }

            //FOV // Is controller asking to update the FOD heading of switch to FOD from traditional?
            //FOV if (gamepad1.dpad_up) {
            //FOV     // If we detected dpad released since the last press
            //FOV     if (!dUp1Prev) {
            //FOV         if (FOD[0]) {
            //FOV             // If already in FOD, update the adjustment angle
            //FOV             adjustAngle[0] = degrees[0];
            //FOV         } else {
            //FOV             // If in traditional drive, just switch to FOD only
            //FOV             FOD[0] = true;
            //FOV         }
            //FOV         // Prevent this path again until the dpad is released
            //FOV         dUp1Prev = true;
            //FOV     }
            //FOV } else {
            //FOV     // Detected the dpad released
            //FOV     dUp1Prev = false;
            //FOV }

            //FOV // Is controller asking to switch FOD mode?
            //FOV if (gamepad1.dpad_down) {
            //FOV     // If we detected dpad released since the last press
            //FOV     if (!dDown1Prev) {
            //FOV         // Invert FOD mode
            //FOV         FOD[0] = ! FOD[0];
            //FOV         // Prevent this path again until the dpad is released
            //FOV         dDown1Prev = true;
            //FOV     }
            //FOV } else {
            //FOV     // Detected the dpad released
            //FOV     dDown1Prev = false;
            //FOV }

            //FOV // Is controller asking to update the FOD heading of switch to FOD from traditional?
            //FOV if (gamepad2.dpad_up) {
            //FOV     // If we detected dpad released since the last press
            //FOV     if (!dUp2Prev) {
            //FOV         if (FOD[1]) {
            //FOV             // If already in FOD, update the adjustment angle
            //FOV             adjustAngle[1] = degrees[1];
            //FOV         } else {
            //FOV             // If in traditional drive, just switch to FOD only
            //FOV             FOD[1] = true;
            //FOV         }
            //FOV         // Prevent this path again until the dpad is released
            //FOV         dUp2Prev = true;
            //FOV     }
            //FOV } else {
            //FOV     // Detected the dpad released
            //FOV     dUp2Prev = false;
            //FOV }

            //FOV // Is controller asking to switch FOD mode?
            //FOV if (gamepad2.dpad_down) {
            //FOV     // If we detected dpad released since the last press
            //FOV     if (!dDown2Prev) {
            //FOV         // Invert FOD mode
            //FOV         FOD[1] = ! FOD[1];
            //FOV         // Prevent this path again until the dpad is released
            //FOV         dDown2Prev = true;
            //FOV     }
            //FOV } else {
            //FOV     // Detected the dpad released
            //FOV     dDown2Prev = false;
            //FOV }

            // Compute the effective offset for each controller FOV
            degrees[0] = AngleUnit.DEGREES.normalize(degrees[0] - adjustAngle[0]);
            degrees[1] = AngleUnit.DEGREES.normalize(degrees[1] - adjustAngle[1]);

            // If FOD is enabled for this controller
            if (FOD[0]) {
                // Convert the X/Y Cartesion for strafe and forward into Polar
                CarToPol(strafe[0], forward[0]);
                // Rotate the Polar coordinates by the robot's heading
                theta -= degrees[0];
                // Convert the new Polar back into Cartesian
                PolToCar(r_speed);
                // Replace the strafe and forward power with translated values
                strafe[0] = new_x;
                forward[0] = new_y;
                // Now the robot moves in orientation of the field
            }

            // If FOD is enabled for this controller
            if (FOD[1]) {
                // Convert the X/Y Cartesion for strafe and forward into Polar
                CarToPol(strafe[1], forward[1]);
                // Rotate the Polar coordinates by the robot's heading
                theta -= degrees[1];
                // Convert the new Polar back into Cartesian
                PolToCar(r_speed);
                // Replace the strafe and forward power with translated values
                strafe[1] = new_x;
                forward[1] = new_y;
                // Now the robot moves in orientation of the field
            }

            // This adds the powers from both controllers together scaled for each controller and FOV
            l_f_motor_power = ((forward[0] + strafe[0] + rotate[0]) * speedModifier[0]) +
                              ((forward[1] + strafe[1] + rotate[1]) * speedModifier[1] * childLock);
            l_b_motor_power = ((forward[0] - strafe[0] + rotate[0]) * speedModifier[0]) +
                              ((forward[1] - strafe[1] + rotate[1]) * speedModifier[1] * childLock);
            r_f_motor_power = ((forward[0] - strafe[0] - rotate[0]) * speedModifier[0]) +
                              ((forward[1] - strafe[1] - rotate[1]) * speedModifier[1] * childLock);
            r_b_motor_power = ((forward[0] + strafe[0] - rotate[0]) * speedModifier[0]) +
                              ((forward[1] + strafe[1] - rotate[1]) * speedModifier[1] * childLock);

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

            //speed control
            if (gamepad1.x) {
                speedModifier[0] = 1;
            }
            else if (gamepad1.y) {
                speedModifier[0] = 0.75;
            }
            else if (gamepad1.b) {
                speedModifier[0] = .5;
            }
            else if (gamepad1.a) {
                speedModifier[0] = 0.35;
            }

            if (gamepad2.x) {
                speedModifier[1] = 1;
            }
            else if (gamepad2.y) {
                speedModifier[1] = 0.75;
            }
            else if (gamepad2.b) {
                speedModifier[1] = .5;
            }
            else if (gamepad2.a) {
                speedModifier[1] = 0.35;
            }

            // Just in case we got stuck in the middle somewhere, don't keep setting position forever.
            currFlickerPos = ((flicker.getPosition()-flickerSet[0]) / (flickerSet[1]-flickerSet[0]));
            if (gamepad1.left_bumper) {
                if (currFlickerPos < 0.90){
                    flicker.setPosition(flickerSet[1]);
                }
            } else {
                if (currFlickerPos > 0.1){
                    flicker.setPosition(flickerSet[0]);
                }
            }

            if (gamepad1.right_bumper) {
                if (!rBump1Prev) {
                    intakePwr++;
                    intakePwr %= 2;
                    // Intake power
                    intake_motor.setPower(intakeSet[intakePwr]);
                    rBump1Prev = true;
                }
            } else {
                rBump1Prev = false;
            }

            // Update the logger 10 times/second max
            if (enableCSVLogging) {
                rt = runtime.seconds();
                if (rt > nextLog) {
                    // FIXME -- bulk data read returning strange data here.
                    //bulkData1 = expansionHub1.getBulkInputData();
                    //bulkData2 = expansionHub2.getBulkInputData();
                    //logger.logD("MechFOVCSV", String.format(",%f,%f,%d,%d,%d,%d,%f,%f,%f,%f", rt, getHeading(), bulkData1.getMotorCurrentPosition(l_f_motor),bulkData1.getMotorCurrentPosition(l_b_motor), bulkData2.getMotorCurrentPosition(r_f_motor), bulkData2.getMotorCurrentPosition(r_b_motor), l_f_motor_power, l_b_motor_power, r_f_motor_power, r_b_motor_power));

                    //int lfPos = l_f_motor.getCurrentPosition();
                    //int lbPos = l_f_motor.getCurrentPosition();
                    //int rfPos = l_f_motor.getCurrentPosition();
                    //int rbPos = l_f_motor.getCurrentPosition();
                    int shoot2Posdbg = shoot2_motor.getCurrentPosition();

                    logger.logD("MechFOVCSV", String.format(",%f,%d,%f", rt, shoot2Posdbg, shooter_power));
                    //logger.logD("MechFOVCSV", String.format(",%f,%f,%d,%d,%d,%d,%f,%f,%f,%f", rt, getQHeading(), lfPos, lbPos, rfPos, rbPos, l_f_motor_power, l_b_motor_power, r_f_motor_power, r_b_motor_power));
                    nextLog = rt + 0.1;
                }
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Polar", "Speed (%.2f), theta (%.2f)", forward[0], theta);
            telemetry.addData("Gyro", degrees[0]);
            telemetry.addData("Shooter:", "%.3f, %.0f", shooter_tgt_power, (tps/(28.0/2.0))*60.0);
            telemetry.addData("Wobble", "Is busy: " + wobble_motor.isBusy() + "- Current Target: " + wobbleTargets[wobblePos] + "- Current Power: " + wobble_motor.getPower() + "- Current Pos: " + wobble_motor.getCurrentPosition()) ;
            telemetry.update();
        }

        // Stop all power
        l_f_motor.setPower(0.0);
        l_b_motor.setPower(0.0);
        r_f_motor.setPower(0.0);
        r_b_motor.setPower(0.0);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
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


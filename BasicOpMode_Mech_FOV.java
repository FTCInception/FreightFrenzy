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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
    private static DcMotor l_f_motor, l_b_motor, r_f_motor, r_b_motor;
    private static DcMotor intake_motor;
    private static DcMotor shoot1_motor, shoot2_motor;
    private static DcMotor wobble_motor;
    private static DcMotor l_in_motor, r_in_motor;
    private static DcMotor l_out_motor, r_out_motor;
    private static Servo foundation1, foundation2, claw,lift,flicker;
    private static Servo back_grabber, front_grabber, slide;

    BNO055IMU imu,imu2;
    //Orientation angles,angles2;
    double MAX_OUTTAKE_POWER = 1.0;
    double MAX_SHOOTER_POWER = 0.495;
    double MAX_INTAKE_POWER = 1.0;

    private BotLog logger = new BotLog();
    private boolean enableCSVLogging = true;

    // Declare other variables
    double in_pwr, out_pwr;

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
    double intake_power;
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
        double fGrabSet[] = {0.6, 0.2};
        double bGrabSet[] = {0.3, 0.0};
        double slideSet[] = {0.22, 0.77};
        boolean lBump2Prev=false, rBump2Prev=false, lTrigPrev=false;
        int fGrabPos=0, bGrabPos=0, slidePos=0;
        boolean dDown2Prev = false, dUp2Prev = false;

        // Pilot toggles
        double liftSet[] = {0.0, 0.3};
        double flickerSet[] = {0.0, 1.0};
        double lFounSet[] = {0.0, 0.9};
        double rFounSet[] = {1.0, 0.1};
        double clawSet[] = {1.0, 0.0};
        boolean lBump1Prev=false, rBump1Prev=false, wobblePrev=false, clawPrev=false;
        int lFounPos=0, rFounPos=0, clawPos=0,liftPos=0, flickerPos=0;
        boolean dLeft1Prev=false;
        double childLock=1.0;
        boolean dDown1Prev = false, dUp1Prev = false;
        double currFlickerPos;

        // Elevator controls
        double prt=0.0, rt = 0.0, nextLog = 0.0;
        int lPos, rPos;
        double lPwr=0, rPwr=0;

        double maxPwr = 0.0;

        //wobble stuff
        final double WOBBLE_TICKS_PER_DEGREE = 5264.0/360.0;
        //int wobbleTargets[] = {100, TOTAL_WOBBLE_TICKS/8*5, TOTAL_WOBBLE_TICKS/4, TOTAL_WOBBLE_TICKS/2};
        final int wobbleTargets[] = {(int)(5*WOBBLE_TICKS_PER_DEGREE),(int)(225*WOBBLE_TICKS_PER_DEGREE), (int)(90*WOBBLE_TICKS_PER_DEGREE), (int)(180*WOBBLE_TICKS_PER_DEGREE)};
        int wobblePos = 0;
        wobble_power = 0.6;

        //claw stuff
        double clawTarget = 0.0;
        double clawStep = 0.05;

        // https://en.wikipedia.org/wiki/Ziegler–Nichols_method
        // Ku = 1/100; Tu = .5s @ 1200 RPM
        // Need to mess with this some more I guess?
        MiniPID pid= new MiniPID(0.10*(1.0/100.0),0.025*(1.0/100.0), 0.2*(1.0/100.0), 0.495/((3600.0*(28.0/1.5))/60.0));
        pid.setOutputLimits(0.0,0.7);

        double shoot2Pos, prevShoot2Pos=0.0;
        double PIDrt, rps=0.0, prevPIDrt=0.0;
        double nextPID=0.0, PIDTime = 0.1;
        double shooterReq,prevShooterReq=0.0;
        double shooterTarget = ((3600.0*(28.0/1.5))/60.0);
        double shooterStep = ((50.0*(28.0/1.5))/60.0);
        double iters=0.0;

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

        l_f_motor = hardwareMap.dcMotor.get("left_front");
        l_b_motor = hardwareMap.dcMotor.get("left_back");
        r_f_motor = hardwareMap.dcMotor.get("right_front");
        r_b_motor = hardwareMap.dcMotor.get("right_back");
        intake_motor = hardwareMap.dcMotor.get("intake");
        shoot1_motor = hardwareMap.dcMotor.get("shoot1");
        shoot2_motor = hardwareMap.dcMotor.get("shoot2");
        wobble_motor = hardwareMap.dcMotor.get("wobble");
        //Ult l_in_motor = hardwareMap.dcMotor.get("right_in");
        //Ult r_in_motor = hardwareMap.dcMotor.get("left_in");

        //Ult foundation1 = hardwareMap.servo.get("foundation1");
        //Ult foundation2 = hardwareMap.servo.get("foundation2");
        claw = hardwareMap.servo.get("claw");
        //No lift: lift = hardwareMap.servo.get("lift");
        flicker = hardwareMap.servo.get("flicker");


        //Ult front_grabber = hardwareMap.servo.get("grabber1");
        //Ult back_grabber = hardwareMap.servo.get("grabber2");
        //Ult slide = hardwareMap.servo.get("slide");
        //Ult l_out_motor = hardwareMap.dcMotor.get("right_out");
        //Ult r_out_motor = hardwareMap.dcMotor.get("left_out");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        l_f_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        l_b_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        //Ult l_in_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        //Ult l_out_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        r_f_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        r_b_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        intake_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        wobble_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        wobble_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobble_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shoot1_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot2_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot1_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shoot2_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Ult r_in_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        //Ult r_out_motor.setDirection(DcMotorSimple.Direction.FORWARD);

        r_f_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        l_f_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r_b_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        l_b_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Ult r_out_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Ult l_out_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = initIMU("imu");
        //imu2 = initIMU("imu 1");

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Waiting for start...");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Take up any slack in the spools
        //Ult l_out_motor.setPower(0.20);
        //Ult r_out_motor.setPower(0.20);

        // Initialize all servos
        //Ult claw.setPosition(clawSet[clawPos]);
        //Ult foundation1.setPosition(lFounSet[lFounPos]);
        //Ult foundation2.setPosition(rFounSet[rFounPos]);
        //Ult slide.setPosition(slideSet[slidePos]);
        //Ult back_grabber.setPosition(bGrabSet[bGrabPos]);
        //Ult front_grabber.setPosition(fGrabSet[fGrabPos]);
        //No Lift: lift.setPosition(liftSet[liftPos]);
        flicker.setPosition(flickerSet[0]);

        sleep(500);

        //Ult l_out_motor.setPower(0.0);
        //Ult r_out_motor.setPower(0.0);

        if (enableCSVLogging) {
            // Lay down a header for our logging
            //logger.logD("MechFOVCSV", String.format(",rt,heading,lfEnc,lbEnc,rfEnc,rbEnc,lfPwr,lbPwr,rfPwr,rbPwr"));
            logger.logD("MechFOVCSV", String.format(",rt,shoot2Enc,shootPwr"));
        }

        // We need to run the motors at very low power and wait until the encoders stop counting.
        // Once they stop counting the slack is taken up from the string.
        // Now set power to 0 and reset encoders.
        // Now we want to let Rev Hub manage encoder position.

        //Ult l_out_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Ult r_out_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Ult l_out_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Ult r_out_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Begin controller 2 (co-pilot)
            // Single button toggle for grabbers
            //Ult if (gamepad2.right_bumper) {
            //Ult     if(!lBump2Prev) {
            //Ult         bGrabPos++; bGrabPos %= 2;
            //Ult         back_grabber.setPosition(bGrabSet[bGrabPos]);
            //Ult         lBump2Prev = true;
            //Ult         //logger.logD("MechFOVbGrab", String.format("%f, %f",runtime.seconds(), bGrabSet[bGrabPos]));
            //Ult     }
            //Ult } else {
            //Ult     lBump2Prev = false;
            //Ult }

            //Ult if (gamepad2.left_bumper) {
            //Ult     if (!rBump2Prev){
            //Ult         fGrabPos++; fGrabPos %= 2;
            //Ult         front_grabber.setPosition(fGrabSet[fGrabPos]);
            //Ult         rBump2Prev = true;
            //Ult         //logger.logD("MechFOVfGrab", String.format("%f, %f",runtime.seconds(), fGrabSet[fGrabPos]));
            //Ult     }
            //Ult } else {
            //Ult     rBump2Prev = false;
            //Ult }

            //Ult if (gamepad2.left_trigger > 0.9) {
            //Ult     if (!lTrigPrev){
            //Ult         slidePos++; slidePos %= 2;
            //Ult         slide.setPosition(slideSet[slidePos]);
            //Ult         lTrigPrev = true;
            //Ult         //logger.logD("MechFOVSlide", String.format("%f, %f",runtime.seconds(), slideSet[slidePos]));
            //Ult     }
            //Ult } else if (gamepad2.left_trigger < 0.1) {
            //Ult     lTrigPrev = false;
            //Ult }

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
                        // Update current RPS rate
                        prevPIDrt = PIDrt;
                        prevShoot2Pos = shoot2Pos;
                        shoot2Pos = shoot2_motor.getCurrentPosition();
                        PIDrt = runtime.seconds();
                    }

                    // Compute current rev per second
                    rps = (shoot2Pos - prevShoot2Pos) / (PIDrt - prevPIDrt);

                    // Save current values for the next loop
                    prevPIDrt = PIDrt;
                    prevShoot2Pos = shoot2Pos;

                    // Now adjust the power based on our PID
                    shooter_power = pid.getOutput(rps, shooterReq * shooterTarget);

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
                    wobble_motor.setTargetPosition(wobbleTargets[wobblePos]);
                    wobble_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if(wobblePos == 0){
                        wobble_motor.setPower(0.4);
                    }else{
                        wobble_motor.setPower(wobble_power);
                    }
                    wobblePrev = true;
                }
            } else {
                wobblePrev = false;

            }

            if(wobble_motor.getCurrentPosition() <= (10.0*WOBBLE_TICKS_PER_DEGREE) && wobblePos == 0){
                wobble_motor.setPower(0);
            }

            // Use the right trigger for elevator power
            //Ult out_pwr = gamepad2.right_trigger;
            //Ult lPos = l_out_motor.getCurrentPosition();
            //Ult rPos = r_out_motor.getCurrentPosition();

            // Adjust the power to help if the encoders get out of whack
            //Ult lPwr = Range.clip(out_pwr - ((lPos-rPos) / 75.0), 0.02, MAX_OUTTAKE_POWER);
            //Ult rPwr = Range.clip(out_pwr + ((lPos-rPos) / 75.0),0.02, MAX_OUTTAKE_POWER);

            // Limit the power at max height to avoid damage
            //Ult if ((lPos > 350) || (rPos > 350)) {
            //Ult     lPwr = Range.clip(lPwr, 0, 0.38);
            //Ult     rPwr = Range.clip(rPwr, 0, 0.38);
            //Ult }

            // At the bottom, with no power request, turn off motors to avoid heating and noise
            //Ult if (((lPos < 9) || (rPos < 9)) && (out_pwr == 0)) {
            //Ult     lPwr = 0;
            //Ult     rPwr = 0;
            //Ult }

            //out_pwr = gamepad2.right_trigger * MAX_OUTTAKE_POWER;
            //Ult l_out_motor.setPower(lPwr);
            //Ult r_out_motor.setPower(rPwr);

            //logger.logD("MechFOVOuttake", String.format("%f, %f, %f, %f, %d, %d",runtime.seconds(), getQHeading(), lPwr, rPwr, l_out_motor.getCurrentPosition(), r_out_motor.getCurrentPosition()));

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
            degrees[1] = degrees[0] = getQHeading();

            // Read the controller 1 (driver) stick positions
            strafe[0] = gamepad1.left_stick_x;
            forward[0] = -gamepad1.left_stick_y;
            rotate[0] = gamepad1.right_stick_x;

            // Read the controller 2 (co-pilot) stick positions
            strafe[1] = gamepad2.left_stick_x;
            forward[1] = -gamepad2.left_stick_y;
            rotate[1] = gamepad2.right_stick_x;


            // Is controller asking to update the FOD heading of switch to FOD from traditional?
            if (gamepad1.dpad_up) {
                // If we detected dpad released since the last press
                if (!dUp1Prev) {
                    shooterTarget += shooterStep;
                    // Prevent this path again until the dpad is released
                    dUp1Prev = true;
                }
            } else {
                // Detected the dpad released
                dUp1Prev = false;
            }

            // Is controller asking to switch FOD mode?
            if (gamepad1.dpad_down) {
                // If we detected dpad released since the last press
                if (!dDown1Prev) {
                    shooterTarget -= shooterStep;
                    // Prevent this path again until the dpad is released
                    dDown1Prev = true;
                }
            } else {
                // Detected the dpad released
                dDown1Prev = false;
            }

            // Is controller asking to changeupdate the FOD heading of switch to FOD from traditional?
            //No lift: if (gamepad1.dpad_up) {
            //No lift:     // If we detected dpad released since the last press
            //No lift:     if (!dUp1Prev) {
            //No lift:         lift.setPosition(liftSet[liftPos]);
            //No lift:         // Prevent this path again until the dpad is released
            //No lift:         dUp1Prev = true;
            //No lift:     }
            //No lift: } else {
            //No lift:     // Detected the dpad released
            //No lift:     dUp1Prev = false;
            //No lift: }

            //ULT // Is controller asking to update the FOD heading of switch to FOD from traditional?
            //ULT if (gamepad1.dpad_up) {
            //ULT     // If we detected dpad released since the last press
            //ULT     if (!dUp1Prev) {
            //ULT         if (FOD[0]) {
            //ULT             // If already in FOD, update the adjustment angle
            //ULT             adjustAngle[0] = degrees[0];
            //ULT         } else {
            //ULT             // If in traditional drive, just switch to FOD only
            //ULT             FOD[0] = true;
            //ULT         }
            //ULT         // Prevent this path again until the dpad is released
            //ULT         dUp1Prev = true;
            //ULT     }
            //ULT } else {
            //ULT     // Detected the dpad released
            //ULT     dUp1Prev = false;
            //ULT }

            //ULT // Is controller asking to switch FOD mode?
            //ULT if (gamepad1.dpad_down) {
            //ULT     // If we detected dpad released since the last press
            //ULT     if (!dDown1Prev) {
            //ULT         // Invert FOD mode
            //ULT         FOD[0] = ! FOD[0];
            //ULT         // Prevent this path again until the dpad is released
            //ULT         dDown1Prev = true;
            //ULT     }
            //ULT } else {
            //ULT     // Detected the dpad released
            //ULT     dDown1Prev = false;
            //ULT }

            // Is controller asking to switch FOD mode?
            if (gamepad2.dpad_down) {
                // If we detected dpad released since the last press
                if (!dDown2Prev) {
                    // Invert FOD mode
                    FOD[1] = ! FOD[1];
                    // Prevent this path again until the dpad is released
                    dDown2Prev = true;
                }
            } else {
                // Detected the dpad released
                dDown2Prev = false;
            }

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

            /*
            // No scaling version of driving.
            // This adds the requested powers from both controllers together scaled for each controller and FOV (clipped to 1.0)
            l_f_motor_power   = Range.clip(((forward[0] + strafe[0] + rotate[0]) * speedModifier[0])+((forward[1] + strafe[1] + rotate[1]) * speedModifier[1] * childLock), -1.0, 1.0) ;
            l_b_motor_power   = Range.clip(((forward[0] - strafe[0] + rotate[0]) * speedModifier[0])+((forward[1] - strafe[1] + rotate[1]) * speedModifier[1] * childLock), -1.0, 1.0) ;
            r_f_motor_power   = Range.clip(((forward[0] - strafe[0] - rotate[0]) * speedModifier[0])+((forward[1] - strafe[1] - rotate[1]) * speedModifier[1] * childLock), -1.0, 1.0) ;
            r_b_motor_power   = Range.clip(((forward[0] + strafe[0] - rotate[0]) * speedModifier[0])+((forward[1] + strafe[1] - rotate[1]) * speedModifier[1] * childLock), -1.0, 1.0) ;
            */

            // Additional controls

            // Intake power
            //Ult in_pwr = gamepad1.right_trigger * MAX_INTAKE_POWER;
            //Ult in_pwr -= gamepad1.left_trigger * MAX_INTAKE_POWER;
            //Ult l_in_motor.setPower(in_pwr);
            //Ult r_in_motor.setPower(in_pwr);

            //intake_power = (gamepad1.right_trigger - gamepad1.left_trigger) * MAX_INTAKE_POWER;
            intake_power = (gamepad1.right_trigger) * MAX_INTAKE_POWER;
            intake_motor.setPower(intake_power);

            //shooter_power = gamepad1.left_trigger * MAX_SHOOTER_POWER;
            //shoot1_motor.setPower(shooter_power);
            //shoot2_motor.setPower(shooter_power);

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

            //No lift: if (gamepad1.left_bumper) {
            //No lift:     if (!lBump1Prev){
            //No lift:         liftPos++; liftPos %= 2;
            //No lift:         lift.setPosition(liftSet[liftPos]);
            //No lift:         lBump1Prev = true;
            //No lift:     }
            //No lift: } else {
            //No lift:     lBump1Prev = false;
            //No lift: }

            // Single button toggle for foundation servos
            // Left Bumper for Foundation
            //Ult if (gamepad1.left_bumper) {
            //Ult     if(!lBump1Prev) {
            //Ult         lFounPos++; lFounPos %= 2;
            //Ult         rFounPos++; rFounPos %= 2;
            //Ult         foundation1.setPosition(lFounSet[lFounPos]);
            //Ult         foundation2.setPosition(rFounSet[rFounPos]);
            //Ult         lBump1Prev = true;
            //Ult     }
            //Ult } else {
            //Ult     lBump1Prev = false;
            //Ult }

            // Single button toggle for claw servo
            // Right bumper for claw
            //Ult if (gamepad1.right_bumper) {
            //Ult     // The elevator interferes with the claw, don't use it
            //Ult     if (lPos < 15) {
            //Ult         if (!rBump1Prev) {
            //Ult             clawPos++;
            //Ult             clawPos %= 2;
            //Ult             claw.setPosition(clawSet[clawPos]);
            //Ult             rBump1Prev = true;
            //Ult         }
            //Ult     }
            //Ult } else {
            //Ult     rBump1Prev = false;
            //Ult }

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
            telemetry.addData("Iters", iters);
            telemetry.addData("Polar", "Speed (%.2f), theta (%.2f)", forward[0], theta);
            telemetry.addData("Gyro", degrees[0]);
            telemetry.addData("Shooter:", "tgt: %.0f (%.0f) vs %.0f",(shooterTarget/(28.0/1.5))*60.0, (shooterTarget/(28.0/1.5))*60.0*shooterReq, (rps/(28.0/1.5))*60.0);
            telemetry.addData("Wobble", "Is busy: " + wobble_motor.isBusy() + "- Current Target: " + wobbleTargets[wobblePos] + "- Current Power: " + wobble_motor.getPower() + "- Current Pos: " + wobble_motor.getCurrentPosition()) ;
            telemetry.update();
        }

        // Stop all power
        //Ult l_out_motor.setPower(0.0);
        //Ult r_out_motor.setPower(0.0);
        //Ult l_in_motor.setPower(0.0);
        //Ult r_in_motor.setPower(0.0);
        l_f_motor.setPower(0.0);
        l_b_motor.setPower(0.0);
        r_f_motor.setPower(0.0);
        r_b_motor.setPower(0.0);
        //Ult l_out_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Ult r_out_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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



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
    private static DcMotor l_in_motor, r_in_motor;
    private static DcMotor l_out_motor, r_out_motor;
    private static Servo foundation1, foundation2, claw;
    private static Servo back_grabber, front_grabber, slide;

    BNO055IMU imu,imu2;
    //Orientation angles,angles2;
    double MAX_OUTTAKE_POWER = 1.0;
    double MAX_INTAKE_POWER = 1.0;

    private BotLog logger = new BotLog();
    private boolean enableCSVLogging = false;

    // Declare other variables
    double in_pwr, out_pwr;

    // Mech drive related variables
    double theta, r_speed, new_x, new_y;
    double[] speedModifier = new double[] {0.5,0.5};
    double[] adjustAngle = new double[] {0.0,0.0};
    double[] forward = new double[2], strafe = new double[2], rotate = new double[2], degrees = new double[2];
    boolean[] FOD = new boolean[] {true,true};

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
        double lFounSet[] = {0.0, 0.9};
        double rFounSet[] = {1.0, 0.1};
        double clawSet[] = {0.0, 1.0};
        boolean lBump1Prev=false, rBump1Prev=false;
        int lFounPos=0, rFounPos=0, clawPos=0;
        boolean dLeft1Prev=false;
        double childLock=1.0;
        boolean dDown1Prev = false, dUp1Prev = false;

        // Elevator controls
        double rt = 0.0, nextLog = 0.0;
        int lPos, rPos;
        double lPwr=0, rPwr=0;

        double maxPwr = 0.0;

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
        l_in_motor = hardwareMap.dcMotor.get("right_in");
        r_in_motor = hardwareMap.dcMotor.get("left_in");

        foundation1 = hardwareMap.servo.get("foundation1");
        foundation2 = hardwareMap.servo.get("foundation2");
        claw = hardwareMap.servo.get("claw");

        front_grabber = hardwareMap.servo.get("grabber1");
        back_grabber = hardwareMap.servo.get("grabber2");
        slide = hardwareMap.servo.get("slide");
        l_out_motor = hardwareMap.dcMotor.get("right_out");
        r_out_motor = hardwareMap.dcMotor.get("left_out");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        l_f_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        l_b_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        l_in_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        l_out_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        r_f_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        r_b_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        r_in_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        r_out_motor.setDirection(DcMotorSimple.Direction.FORWARD);

        r_f_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        l_f_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r_b_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        l_b_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        r_out_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        l_out_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = initIMU("imu");
        //imu2 = initIMU("imu 1");

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Waiting for start...");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Take up any slack in the spools
        l_out_motor.setPower(0.20);
        r_out_motor.setPower(0.20);

        // Initialize all servos
        claw.setPosition(clawSet[clawPos]);
        foundation1.setPosition(lFounSet[lFounPos]);
        foundation2.setPosition(rFounSet[rFounPos]);
        slide.setPosition(slideSet[slidePos]);
        back_grabber.setPosition(bGrabSet[bGrabPos]);
        front_grabber.setPosition(fGrabSet[fGrabPos]);

        sleep(500);

        l_out_motor.setPower(0.0);
        r_out_motor.setPower(0.0);

        if (enableCSVLogging) {
            // Lay down a header for our logging
            logger.logD("MechFOVCSV", String.format(",rt,heading,lfEnc,lbEnc,rfEnc,rbEnc,lfPwr,lbPwr,rfPwr,rbPwr"));
        }

        // We need to run the motors at very low power and wait until the encoders stop counting.
        // Once they stop counting the slack is taken up from the string.
        // Now set power to 0 and reset encoders.
        // Now we want to let Rev Hub manage encoder position.

        l_out_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r_out_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        l_out_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r_out_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Begin controller 2 (co-pilot)
            // Single button toggle for grabbers
            if (gamepad2.right_bumper) {
                if(!lBump2Prev) {
                    bGrabPos++; bGrabPos %= 2;
                    back_grabber.setPosition(bGrabSet[bGrabPos]);
                    lBump2Prev = true;
                    //logger.logD("MechFOVbGrab", String.format("%f, %f",runtime.seconds(), bGrabSet[bGrabPos]));
                }
            } else {
                lBump2Prev = false;
            }

            if (gamepad2.left_bumper) {
                if (!rBump2Prev){
                    fGrabPos++; fGrabPos %= 2;
                    front_grabber.setPosition(fGrabSet[fGrabPos]);
                    rBump2Prev = true;
                    //logger.logD("MechFOVfGrab", String.format("%f, %f",runtime.seconds(), fGrabSet[fGrabPos]));
                }
            } else {
                rBump2Prev = false;
            }

            if (gamepad2.left_trigger > 0.9) {
                if (!lTrigPrev){
                    slidePos++; slidePos %= 2;
                    slide.setPosition(slideSet[slidePos]);
                    lTrigPrev = true;
                    //logger.logD("MechFOVSlide", String.format("%f, %f",runtime.seconds(), slideSet[slidePos]));
                }
            } else if (gamepad2.left_trigger < 0.1) {
                lTrigPrev = false;
            }

            rt = runtime.seconds();

            // Use the rigth trigger for elevator power
            out_pwr = gamepad2.right_trigger;
            lPos = l_out_motor.getCurrentPosition();
            rPos = r_out_motor.getCurrentPosition();

            // Adjust the power to help if the encoders get out of whack
            lPwr = Range.clip(out_pwr - ((lPos-rPos) / 75.0), 0.02, MAX_OUTTAKE_POWER);
            rPwr = Range.clip(out_pwr + ((lPos-rPos) / 75.0),0.02, MAX_OUTTAKE_POWER);

            // Limit the power at max height to avoid damage
            if ((lPos > 350) || (rPos > 350)) {
                lPwr = Range.clip(lPwr, 0, 0.38);
                rPwr = Range.clip(rPwr, 0, 0.38);
            }

            // At the bottom, with no power request, turn off motors to avoid heating and noise
            if (((lPos < 9) || (rPos < 9)) && (out_pwr == 0)) {
                lPwr = 0;
                rPwr = 0;
            }

            //out_pwr = gamepad2.right_trigger * MAX_OUTTAKE_POWER;
            l_out_motor.setPower(lPwr);
            r_out_motor.setPower(rPwr);

            //logger.logD("MechFOVOuttake", String.format("%f, %f, %f, %f, %d, %d",runtime.seconds(), getQHeading(), lPwr, rPwr, l_out_motor.getCurrentPosition(), r_out_motor.getCurrentPosition()));

            // Toggle the child lock between 0.0 and 1.0
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
                    if (FOD[0]) {
                        // If already in FOD, update the adjustment angle
                        adjustAngle[0] = degrees[0];
                    } else {
                        // If in traditional drive, just switch to FOD only
                        FOD[0] = true;
                    }
                    // Prevent this path again until the dpad is released
                    dUp1Prev = true;
                }
            } else {
                // Detected the dpad released
                dUp1Prev = false;
            }

            // Is controller asking to update the FOD heading of switch to FOD from traditional?
            if (gamepad2.dpad_up) {
                // If we detected dpad released since the last press
                if (!dUp2Prev) {
                    if (FOD[1]) {
                        // If already in FOD, update the adjustment angle
                        adjustAngle[1] = degrees[1];
                    } else {
                        // If in traditional drive, just switch to FOD only
                        FOD[1] = true;
                    }
                    // Prevent this path again until the dpad is released
                    dUp2Prev = true;
                }
            } else {
                // Detected the dpad released
                dUp2Prev = false;
            }

            // Is controller asking to switch FOD mode?
            if (gamepad1.dpad_down) {
                // If we detected dpad released since the last press
                if (!dDown1Prev) {
                    // Invert FOD mode
                    FOD[0] = ! FOD[0];
                    // Prevent this path again until the dpad is released
                    dDown1Prev = true;
                }
            } else {
                // Detected the dpad released
                dDown1Prev = false;
            }

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
            in_pwr = gamepad1.right_trigger * MAX_INTAKE_POWER;
            in_pwr -= gamepad1.left_trigger * MAX_INTAKE_POWER;
            l_in_motor.setPower(in_pwr);
            r_in_motor.setPower(in_pwr);

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

            // Single button toggle for foundation servos
            // Left Bumper for Foundation
            if (gamepad1.left_bumper) {
                if(!lBump1Prev) {
                    lFounPos++; lFounPos %= 2;
                    rFounPos++; rFounPos %= 2;
                    foundation1.setPosition(lFounSet[lFounPos]);
                    foundation2.setPosition(rFounSet[rFounPos]);
                    lBump1Prev = true;
                }
            } else {
                lBump1Prev = false;
            }

            // Single button toggle for claw servo
            // Right bumper for claw
            if (gamepad1.right_bumper) {
                // The elevator interferes with the claw, don't use it
                if (lPos < 15) {
                    if (!rBump1Prev) {
                        clawPos++;
                        clawPos %= 2;
                        claw.setPosition(clawSet[clawPos]);
                        rBump1Prev = true;
                    }
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

                    int lfPos = l_f_motor.getCurrentPosition();
                    int lbPos = l_f_motor.getCurrentPosition();
                    int rfPos = l_f_motor.getCurrentPosition();
                    int rbPos = l_f_motor.getCurrentPosition();

                    logger.logD("MechFOVCSV", String.format(",%f,%f,%d,%d,%d,%d,%f,%f,%f,%f", rt, getQHeading(), lfPos, lbPos, rfPos, rbPos, l_f_motor_power, l_b_motor_power, r_f_motor_power, r_b_motor_power));
                    nextLog = rt + 0.1;
                }
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Polar", "Speed (%.2f), theta (%.2f)", forward[0], theta);
            telemetry.addData("Gyro", degrees[0]);
            telemetry.update();
        }

        // Stop all power
        l_out_motor.setPower(0.0);
        r_out_motor.setPower(0.0);
        l_in_motor.setPower(0.0);
        r_in_motor.setPower(0.0);
        l_f_motor.setPower(0.0);
        l_b_motor.setPower(0.0);
        r_f_motor.setPower(0.0);
        r_b_motor.setPower(0.0);
        l_out_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r_out_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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



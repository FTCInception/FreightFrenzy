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

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_front"
 * Motor channel:  Right drive motor:        "right_front"
 * Motor channel:  Left  drive motor:        "left_back"
 * Motor channel:  Right drive motor:        "right_back"
 * Servo channel:  Servo to grab foundation: "foundation"
 * Servo channel:  Servo to grab blocks:     "claw"
 */
public class MechBot {
    /* Public OpMode members. */
    public DcMotor leftFDrive = null;
    public DcMotor rightFDrive = null;
    public DcMotor leftBDrive = null;
    public DcMotor rightBDrive = null;
    public DcMotor l_in_motor = null;
    public DcMotor r_in_motor = null;

    public Servo foundation1 = null;
    public Servo foundation2 = null;
    public Servo claw = null;

    private static final double COUNTS_PER_MOTOR_REV = 537.6;         // eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = 10.0 / 10.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4;       // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159);
    private static final double AXLE_LENGTH = 13.5;          // Width of robot through the pivot point (center wheels)
    // Multiply INCHES_PER_DEGREE by 1.75 to handle the sliding of the MECH wheel rollers?
    private static final double INCHES_PER_DEGREE = 1.52 * ( (AXLE_LENGTH * 3.14159) / 360.0 );
    private static final double SOFT_D_UP = 50.0;
    private static final double SOFT_D_UP_DEGREE = 5;
    private static final double SOFT_D_DOWN = 70.0;
    private static final double SOFT_D_DOWN2 = 100.0;
    private static final double SOFT_D_DOWN_DEGREE = 5;
    private static final double fKpL = 1.0;
    private static final double fKpR = 0.92;
    private static final double PIVOT_FACTOR = 2.05;
    static double KpL;
    static double KpR;
    private static final double CLOSE_ENOUGH = 15.0;
    private static final double CLOSE_ENOUGH_DEGREE = 3.0;
    private static final double RIGHT_ARC_COEFFICENT = 1.00;
    private static final double LEFT_ARC_COEFFICENT = 1.25;
    private static final double IMU_CORR = 1.01;
    static final double RIGHT = 1;
    static final double LEFT = 0;
    BNO055IMU imu;
    private Orientation angles;
    private Orientation lastAngles = new Orientation();
    private double globalHeading = 0;
    private boolean useIntegratedGyro = true;  // Controls whether we use raw gyro values or the integrated version we make.
    //ColorSensor colorSensor;
    private static final boolean DEBUG = false;
    double minUp = 0.0;
    double straightA = 0.0;
    public BotLog logger = new BotLog();

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    private static LinearOpMode myLOpMode;

    /* Constructor */
    public MechBot() {
    }

    public void initIMU() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        myLOpMode.telemetry.addData("Mode", "IMU calibrating...");
        myLOpMode.telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        for (int i=0; (i<40 && !imu.isGyroCalibrated()); i++)
        {
            myLOpMode.sleep(50);
            myLOpMode.idle();
        }
    }

    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hwMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    public void initAutonomous(LinearOpMode lOpMode) {
        myLOpMode = lOpMode;
        // Send telemetry message to signify robot waiting;
        myLOpMode.telemetry.addData("Status", "Resetting Encoders");    //
        myLOpMode.telemetry.update();

        leftFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        myLOpMode.telemetry.addData("Path0", "Starting at LF:%7d, RF:%7d, LB:%7d, RB:%7d",
                leftFDrive.getCurrentPosition(),
                rightFDrive.getCurrentPosition(),
                leftBDrive.getCurrentPosition(),
                rightBDrive.getCurrentPosition());

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        myLOpMode.telemetry.addData("Mode", "IMU calibrating...");
        myLOpMode.telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        for (int i=0; (i<40 && !myLOpMode.isStopRequested() && !imu.isGyroCalibrated()); i++)
        {
            myLOpMode.sleep(50);
            myLOpMode.idle();
        }

        //colorSensor = hwMap.colorSensor.get("color");
        //colorSensor.enableLed(false);

        // Adjust our overall power based on a 12.5V battery.
        // Set some min and max to avoid anything crazy.
        // This may need some more characterization and it may be battery specific.
        double volts = Math.max(11.0,Math.min(12.5,getBatteryVoltage()));
        KpL = fKpL * (12.5 / volts);
        KpR = fKpR * (12.5 / volts);

        myLOpMode.telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        myLOpMode.telemetry.addData("voltage", "%.1f, KpR: %.3f, KpL: %.3f", volts, KpR, KpL);
        composeTelemetry();
        myLOpMode.telemetry.update();

        logger.logD("MechLogCSV",String.format(",rt,heading,lfEnc,lbEnc,rfEnc,rbEnc,lfPwr,lbPwr,rfPwr,rbPwr"));
    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        myLOpMode.telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        });

        myLOpMode.telemetry.addLine()
                .addData("Z", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("Y", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("X", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                })
        ;
    };

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------
    String formatColor(int argb){
        int a = argb >> 24 & 0xFF;
        int r = argb >> 16 & 0xFF;
        int g = argb >> 8 & 0xFF;
        int b = argb >> 0 & 0xFF;

        if ( a < 0x5 ) {
            return String.format(Locale.getDefault(), "Unknown (0x%08x)", argb);
        } else if ((r>>1 > b ) && (g<<1 > b)) {
            return String.format(Locale.getDefault(), "Yellow (0x%08x)", argb);
        } else {
            return String.format(Locale.getDefault(), "Black (0x%08x)", argb);
        }
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    /* Check color sensor */
    /*
    public boolean isColorSensorYellow() {
        int argb = colorSensor.argb();
        int a = argb >> 24 & 0xFF;
        int r = argb >> 16 & 0xFF;
        int g = argb >> 8 & 0xFF;
        int b = argb >> 0 & 0xFF;

        // Pure yellow == 100% r + 100% g + 0% b
        // For us, r/2 > b and g/2 > b is sufficent
        // a < 5 is too far away to be accurate.
        if ( a < 0x5 ) {
            return false;
        } else if ((r>>1 > b ) && (g>>1 > b)) {
            return true;
        } else {
            return false;
        }
    }
    */

        /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFDrive  = hwMap.get(DcMotor.class, "left_front");
        rightFDrive = hwMap.get(DcMotor.class, "right_front");
        leftBDrive  = hwMap.get(DcMotor.class, "left_back");
        rightBDrive = hwMap.get(DcMotor.class, "right_back");
        r_in_motor = hwMap.get(DcMotor.class, "right_in");
        l_in_motor = hwMap.get(DcMotor.class, "left_in");


        //leftArm    = hwMap.get(DcMotor.class, "left_arm");
        leftFDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftBDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightBDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        l_in_motor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        r_in_motor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors

        // Set all motors to zero power
        leftFDrive.setPower(0);
        rightFDrive.setPower(0);
        leftBDrive.setPower(0);
        rightBDrive.setPower(0);
        r_in_motor.setPower(0);
        l_in_motor.setPower(0);

        //leftArm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //l_in_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //r_in_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        foundation1  = hwMap.get(Servo.class, "foundation1");
        foundation2  = hwMap.get(Servo.class, "foundation2");
        claw = hwMap.get(Servo.class, "claw");
        foundation1.setPosition(0);
        foundation2.setPosition(1);
        claw.setPosition(0);
    }

    //Used to change motor direction before and after strafing.
    public void invertMotorDirection(DcMotor motor) {
        if (motor.getDirection() == DcMotor.Direction.FORWARD) {
            motor.setDirection(DcMotor.Direction.REVERSE);
        } else {
            motor.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    public void grabFoundation(int wait) {
        foundation1.setPosition(0.9);
        foundation2.setPosition(0.1);
        myLOpMode.sleep(wait);
    }

    public void releaseFoundation(int wait) {
        foundation1.setPosition(0);
        foundation2.setPosition(1);
        myLOpMode.sleep(wait);
    }

    public void grabBlock(int wait) {
        claw.setPosition(1);
        myLOpMode.sleep(wait);
    }

    public void dropBlock(int wait) {
        claw.setPosition(0);
        myLOpMode.sleep(wait);
    }

    public double fastEncoderStraight(double speed, double distance, double timeoutS, double P) {
        return(fastEncoderDrive( speed, distance, distance, timeoutS, P ));
    }

    public double fastEncoderStraight(double speed, double distance, double timeoutS) {
        return(fastEncoderDrive( speed, distance, distance, timeoutS ));
    }

    public void encoderStraight(double speed, double distance, double timeoutS) {
        encoderDrive( speed, distance, distance, timeoutS );
    }

    public void fastEncoderRotate(double speed, double degrees, double timeoutS) {
        fastEncoderDrive( speed, degrees * INCHES_PER_DEGREE, -degrees * INCHES_PER_DEGREE, timeoutS );
    }

    public void encoderRotate(double speed, double degrees, double timeoutS) {
        encoderDrive( speed, degrees * INCHES_PER_DEGREE, -degrees * INCHES_PER_DEGREE, timeoutS );
    }

    public double gyroRotate(double speed, double degrees, double timeoutS) {
        if (degrees > 0){
            return(gyroTurn( degrees, -speed, speed, timeoutS ));
        } else {
            return(gyroTurn( degrees, speed, -speed, timeoutS ));
        }
    }

    public double gyroPivot(double speed, double degrees, double timeoutS) {
        // +speed and +degrees requires right power
        // -speed and -degrees requires right power
        if (((degrees > 0) && (speed > 0)) || ((degrees < 0) && (speed < 0))) {
            return(gyroTurn( degrees, 0, speed, timeoutS ));
        } else {
            return(gyroTurn( degrees, speed, 0, timeoutS ));
        }
    }

    public void encoderPivot(double speed, double degrees, double timeoutS) {
        if (degrees > 0) {
            encoderDrive(speed, degrees * PIVOT_FACTOR * INCHES_PER_DEGREE, 0, timeoutS);
        } else {
            // We use -degrees, since to cancel out the negative degrees (we want positive right wheel rotation
            encoderDrive(speed, 0, -degrees * PIVOT_FACTOR * INCHES_PER_DEGREE, timeoutS);
        }
    }

    public void fastEncoderPivot(double speed, double degrees, double timeoutS) {
        if (degrees > 0) {
            fastEncoderDrive(speed, degrees * PIVOT_FACTOR * INCHES_PER_DEGREE, 0, timeoutS);
        } else {
            // We use -degrees, since to cancel out the negative degrees (we want positive right wheel rotation
            fastEncoderDrive(speed, 0, -degrees * PIVOT_FACTOR * INCHES_PER_DEGREE, timeoutS);
        }
    }


    public void encoderArc(double speed, double degrees, double LorR, double radius, double timeoutS) {

        // ARC_COEFFICENT to overcome friction/slide/etc?
        double InnerDist = 3.14159*(radius*2) * (degrees / 360.0) ;
        double OuterDist = 3.14159*((radius+AXLE_LENGTH)*2) * (degrees / 360.0) ;
        double ratio = InnerDist/OuterDist;
        //int newInnerTarget = (int) (InnerDist * COUNTS_PER_INCH);
        //int newOuterTarget = (int) (OuterDist * COUNTS_PER_INCH);
        double tgtPos = Math.abs(OuterDist * COUNTS_PER_INCH);
        double KpI;
        double KpO;
        double curPos;
        double toGo;
        double actSpeed ;
        double newSpeed ;
        double spdUp,spdDn;
        double[] speedRampUp = {0.20, 0.25, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0};
        double[] speedRampDown = {0.1, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0};
        ElapsedTime     runtime = new ElapsedTime();
        DcMotor  OFDrive ;
        DcMotor  OBDrive ;
        DcMotor  IFDrive ;
        DcMotor  IBDrive ;

        // Define inner and outer motors
        if (LorR == RIGHT) {
            OFDrive = leftFDrive ;
            OBDrive = leftBDrive ;
            IFDrive = rightFDrive ;
            IBDrive = rightBDrive ;
            KpO = KpL ;
            KpI = KpR ;
            InnerDist *= RIGHT_ARC_COEFFICENT ;
            OuterDist *= RIGHT_ARC_COEFFICENT ;
            tgtPos *= RIGHT_ARC_COEFFICENT ;
        } else {
            OFDrive = rightFDrive ;
            OBDrive = rightBDrive ;
            IFDrive = leftFDrive ;
            IBDrive = leftBDrive ;
            KpO = KpR ;
            KpI = KpL ;
            //KpO = 1.0 ;
            //KpI = 1.0 ;
            InnerDist *= LEFT_ARC_COEFFICENT ;
            OuterDist *= LEFT_ARC_COEFFICENT ;
            tgtPos *= LEFT_ARC_COEFFICENT ;
        }

        // Ensure that the opmode is still active
        if (myLOpMode.opModeIsActive()) {

            OFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            OBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            IFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            IBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn On RUN_TO_POSITION
            OFDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            OBDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            IFDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            IBDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // reset the timeout time and start motion at the minimum speed.
            runtime.reset();
            speed = Math.abs(speed);
            actSpeed = Math.min(speedRampUp[0],speed);
            if ( OuterDist < 0 ) {
                actSpeed *= -1.0;
            }
            OFDrive.setPower(actSpeed * KpO );
            OBDrive.setPower(actSpeed * KpO );
            IFDrive.setPower(actSpeed * KpI * ratio );
            IBDrive.setPower(actSpeed * KpI * ratio );

            // keep looping while we are still active, and there is time left, and the motors haven't reached end point
            while (myLOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (Math.max(0,(tgtPos - (curPos = Math.abs(OFDrive.getCurrentPosition())))) > CLOSE_ENOUGH)) {

                // This code implements a soft start and soft stop.
                // Use current position from loop abpvr
                // How much farther?
                toGo  = Math.max(0,tgtPos - curPos);

                // Compute speed on acceleration and deceleration legs
                spdUp = Math.min(speedRampUp[Math.min((int)(curPos/SOFT_D_UP),speedRampUp.length-1)],speed);
                spdDn = Math.min(speedRampDown[Math.min((int)(toGo/SOFT_D_DOWN),speedRampDown.length-1)],speed);

                // Use the minimum speed
                newSpeed = Math.min(spdUp, spdDn);

                // Change power if necessary
                if ( OuterDist < 0 ) {
                    newSpeed *= -1.0;
                }
                if (newSpeed != actSpeed) {
                    actSpeed = newSpeed;
                    OFDrive.setPower(actSpeed * KpO );
                    OBDrive.setPower(actSpeed * KpO );
                    IFDrive.setPower(actSpeed * KpI * ratio );
                    IBDrive.setPower(actSpeed * KpI * ratio );
                }

                if (DEBUG) {
                    myLOpMode.telemetry.addData("Path3", "up: %1.3f, dn: %1.3f, s: %1.3f, p: %5d, t: %5d", spdUp, spdDn, actSpeed, (int) curPos, (int) toGo);
                    myLOpMode.telemetry.update();
                }
            }

            // Stop all motion;
            leftFDrive.setPower(0);
            rightFDrive.setPower(0);
            leftBDrive.setPower(0);
            rightBDrive.setPower(0);

            // Turn off RUN_WITHOUT_ENCODERS
            leftFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(500);   // optional pause after each move
        }
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
        double toGo;
        double actSpeed;
        double newSpeed;
        double spdUp,spdDn;
        double[] speedRampUp = {0.20, 0.25, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0};
        double[] speedRampDown = {0.1, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0};
        ElapsedTime     runtime = new ElapsedTime();

        logger.logD("MechLog",String.format("encoderDrive start"));

        // Ensure that the opmode is still active
        if (myLOpMode.opModeIsActive()) {

            leftFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftFTarget = (int)(leftInches * COUNTS_PER_INCH);
            newRightFTarget = (int)(rightInches * COUNTS_PER_INCH);
            newLeftBTarget = (int)(leftInches * COUNTS_PER_INCH);
            newRightBTarget = (int)(rightInches * COUNTS_PER_INCH);
            tgtPos = Math.abs(newLeftFTarget);
            leftFDrive.setTargetPosition(newLeftFTarget);
            rightFDrive.setTargetPosition(newRightFTarget);
            leftBDrive.setTargetPosition(newLeftBTarget);
            rightBDrive.setTargetPosition(newRightBTarget);

            // Turn On RUN_TO_POSITION
            leftFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // The front wheels are slipping so they stop and fight the back.  Set them
            // to FLOAT here to try to avoid that.  Not sure if this actually works
            leftFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


            // reset the timeout time and start motion at the minimum speed.
            runtime.reset();
            speed = Math.abs(speed);
            actSpeed = speedRampUp[0];
            leftFDrive.setPower(actSpeed * KpL);
            rightFDrive.setPower(actSpeed * KpR);
            leftBDrive.setPower(actSpeed * KpL);
            rightBDrive.setPower(actSpeed * KpR);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (myLOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftBDrive.isBusy() && rightBDrive.isBusy() &&
                            ((Math.abs(newLeftBTarget  -  leftBDrive.getCurrentPosition()) > CLOSE_ENOUGH) ||
                                    (Math.abs(newRightBTarget - rightBDrive.getCurrentPosition()) > CLOSE_ENOUGH)))) {

                // This code implements a soft start and soft stop.
                // Get current position
                curPos = Math.abs(leftFDrive.getCurrentPosition());

                // How much farther?
                toGo  = Math.max(0,(tgtPos - curPos));

                // Compute speed on acceleration and deceleration legs
                spdUp = Math.min(speedRampUp[Math.min((int)(curPos/SOFT_D_UP),speedRampUp.length-1)],speed);
                spdDn = Math.min(speedRampDown[Math.min((int)(toGo/SOFT_D_DOWN),speedRampDown.length-1)],speed);

                // Use the minimum speed
                newSpeed = Math.min(spdUp, spdDn);

                // Special case when we get really close, go back to full power and let the PID
                // motor controller handle it
                if (toGo < (SOFT_D_DOWN - 5)) { newSpeed = speed; }

                // Change power if necessary
                if (newSpeed != actSpeed) {
                    actSpeed = newSpeed;
                    leftFDrive.setPower(actSpeed * KpL);
                    rightFDrive.setPower(actSpeed * KpR);
                    leftBDrive.setPower(actSpeed * KpL);
                    rightBDrive.setPower(actSpeed * KpR);
                }

                if (DEBUG) {
                    myLOpMode.telemetry.addData("Path3", "up: %1.3f, dn: %1.3f, s: %1.3f, p: %5d, t: %5d", spdUp, spdDn, actSpeed, (int) curPos, (int) toGo);
                    myLOpMode.telemetry.update();
                }
            }

            // The front wheels are slipping so they stop and fight the back.  Set them
            // to FLOAT earlier, so reset to BRAKE here.
            leftFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Stop all motion;
            leftFDrive.setPower(0);
            rightFDrive.setPower(0);
            leftBDrive.setPower(0);
            rightBDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(500);   // optional pause after each move
        }

        logger.logD("MechLog",String.format("encoderDrive end"));
    }

    public double getHeading()
    {
        Orientation     gyroOrien;
        double          a;

        if(useIntegratedGyro) {
            return getIntHeading();
        } else {

            gyroOrien = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            a = AngleUnit.DEGREES.fromUnit(gyroOrien.angleUnit, gyroOrien.firstAngle);
            logger.logD("MechLog", String.format(" getHeading: %f, (%f), %f, (%f), %f", a, AngleUnit.DEGREES.normalize(a), a * IMU_CORR, AngleUnit.DEGREES.normalize(a * IMU_CORR), AngleUnit.DEGREES.normalize(a) - AngleUnit.DEGREES.normalize(a * IMU_CORR)));

            return (AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(gyroOrien.angleUnit, gyroOrien.firstAngle) * IMU_CORR));
        }
    }

    // Reset the integrated heading tracker
    public void resetIntHeading() {

        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalHeading = 0.0 ;
    }

    // Create an integrated heading tracker.  This is useful for turning since the gyro only report -180 <--> 180.
    // We have some correction factors that need to be handled.
    public double getIntHeading()
    {
        Orientation     gyroOrien;
        double          a;
        double          delta;

        gyroOrien = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        delta = AngleUnit.DEGREES.fromUnit(gyroOrien.angleUnit, gyroOrien.firstAngle) - AngleUnit.DEGREES.fromUnit(gyroOrien.angleUnit, lastAngles.firstAngle);

        if (delta <= -180) {
            delta += 360;
        } else if (delta > 180) {
            delta -= 360;
        }

        globalHeading += (delta * IMU_CORR);

        lastAngles = gyroOrien;

        logger.logD("MechLog",String.format(" getIntHeading: %f, %f", delta*IMU_CORR, globalHeading));

        return( globalHeading );
    }

    private double normalizeAngle( double degrees ) {

        if(useIntegratedGyro) {
            return(degrees);
        } else {
            return(AngleUnit.normalizeDegrees(degrees));
        }
    }

    // Calling with an unknown accel/decel profile, try to find a good one for the caller.
    public double gyroTurn(double degrees,
                           double leftPower, double rightPower,
                           double timeoutS) {

        // This profile is a hold over from the straight commands
        double[] speedRampUp = {0.20, 0.25, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0};
        double[] speedRampDown = {0.1, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0};

        // This profile is for rotate
        double[] speedRampUpR = {0.275, 0.325, 0.40, 0.475, 0.55, 0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0};
        double[] speedRampDownR = {0.20, 0.20, 0.225, 0.225, 0.25, 0.25, 0.275, 0.275, 0.30, 0.30, 0.325, 0.325, 0.35, 0.35, 0.375, 0.40, 0.425, 0.45, 0.475, 0.5, 1.0};
        //double[] speedRampDownR = {0.20, 0.20, 0.225, 0.225, 0.25, 0.25, 0.275, 0.275, 0.0, 0.35, 0.35, 0.375, 0.40, 1.0};
        //double[] speedRampDownR = {0.20, 0.20, 0.225, 0.225, 0.1, 0.25, 0.1, 0.275, 0.1, 0.30, 0.1, 0.325, 0.1, 0.35, 0.375, 0.40, 1.0};


        // This profile is for the foundation movement which is the only thing with Pivot right now.  Better to let foundation manage
        // it special profile fro itself and build a general purpose pivot here.
        double[] speedRampUpP = {0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0};
        double[] speedRampDownP = {0.30, 0.35, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.0};

        // If this is a rotate, use rotate profile
        if (leftPower == -rightPower) {
            speedRampUp = speedRampUpR;
            speedRampDown = speedRampDownR;
        }

        // If this is a Pivot, use Pivot profile
        if ((leftPower == 0) || (rightPower == 0) ) {
            speedRampUp = speedRampUpP;
            speedRampDown = speedRampDownP;
        }

        return(gyroTurn(degrees, leftPower, rightPower, timeoutS, speedRampUp, speedRampDown));
    }

    /*
     *  Method to perfmorm an arbitrary turn with differential wheel power and gyro feedback
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired heading
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public double gyroTurn(double degrees,
                           double leftPower, double rightPower,
                           double timeoutS, double[] up, double[] down) {
        double toGo;
        double maxPower;
        double actSpeed=0.0;
        double newSpeed ;
        double spdUp, spdDn;
        double[] speedRampUp = up;
        double[] speedRampDown = down;
        ElapsedTime runtime = new ElapsedTime();
        double tgtHeading, curHeading, startHeading;
        double KsL = 1.0;
        double KsR = 1.0;
        double delta, sign=1.0;
        double rt = 0.0;

        // The gyro is still changing at this point. Try to dump values and understand why.
        //for(int i=0; i<10; i++) {
        //    logger.logD("MechLogfoostart", String.format("getHeading: %f", getHeading()));
        //    myLOpMode.sleep(100);
        //}

        // Get the current Heading
        startHeading = curHeading = getHeading();
        // Where we are headed
        tgtHeading = normalizeAngle(curHeading + degrees) ;
        // Initialize toGo
        toGo = Math.abs(degrees);

        logger.logD("MechLog",String.format("gyroTurn: start: %f, tgt: %f", startHeading, tgtHeading));

        if (DEBUG) {
            myLOpMode.telemetry.addData("gyro", "d: %3.1f, c: %3.1f, s: %3.1f, t: %3.1f, e: %3.1f",degrees,curHeading,startHeading,tgtHeading,normalizeAngle(curHeading - tgtHeading));
            myLOpMode.telemetry.update();
            //myLOpMode.sleep(3000);
        }

        // FIXME: Let's assume the caller has made sure our left/right power will turn the same direction as our degrees for now

        // Ensure that the opmode is still active
        if (myLOpMode.opModeIsActive()) {

            leftFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // We'll handle the power/speed/encoders
            leftFDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Initialize Power Ratio
            //init maxpower
            maxPower = Math.max( Math.abs(rightPower), Math.abs(leftPower) );

            // Power ratio for '0' values is special case
            if (leftPower == 0) {
                KsL = 0.0 ;
                KsR = Math.abs(rightPower)/rightPower;
            }

            if (rightPower == 0 ) {
                KsR = 0.0 ;
                KsL = Math.abs(leftPower)/leftPower;
            }

            // Find the ratio for left and right.  The larger absolute value always is the (+/-)1.0 ratio
            if (( leftPower != 0 ) && (rightPower != 0)) {
                if ( Math.abs(leftPower) > Math.abs(rightPower) ) {
                    // This division preserves the direction (+/-)
                    KsL = Math.abs(leftPower)/leftPower;
                    KsR = rightPower/Math.abs(leftPower);
                } else {
                    // This division preserves the direction (+/-)
                    KsR = Math.abs(rightPower)/rightPower;
                    KsL = leftPower/Math.abs(rightPower);
                }
            }

            if (DEBUG) {
                myLOpMode.telemetry.addData("gyro", "KL: %.3f, KR: %.3f, max: %.2f",KsL, KsR,maxPower);
                myLOpMode.telemetry.update();
                //myLOpMode.sleep(5000);
            }

            // reset the timeout time and start motion at the minimum speed.
            runtime.reset();

            // Setting the starting speed is now handled inside the loop below on the first iteration.
            // Don't set the speed out here because a 'pivot' where one side has a '0' distance
            // request would start moving too soon.

            delta = normalizeAngle(curHeading - tgtHeading);
            if ( delta != 0) {
                sign = delta/Math.abs(delta);
            }

            // Initialize variables before the loop
            // Use done to jump out of loop when you get close enough to the end
            // Keep looping while we are still active, and there is time left, and we haven't reached the target
            while (myLOpMode.opModeIsActive() &&
                    ((rt=runtime.seconds()) < timeoutS)) {

                // Check if we're done
                // This code implements a soft start and soft stop.
                curHeading = getHeading();

                // Look to see if our delta is really close or if we over rotated already (sign changed on delta)
                delta = normalizeAngle(curHeading - tgtHeading);
                if (((Math.abs(delta)) < CLOSE_ENOUGH_DEGREE) || (delta/Math.abs(delta) != sign)) {
                        break;
                }

                // How much farther?
                toGo = Math.min(toGo, Math.max(0, Math.abs(normalizeAngle(tgtHeading - curHeading))));

                // Compute speed on acceleration and deceleration legs
                spdUp = Math.min(speedRampUp[Math.min((int) (Math.abs(normalizeAngle(startHeading - curHeading)) / SOFT_D_UP_DEGREE), speedRampUp.length - 1)], maxPower);
                spdDn = Math.min(speedRampDown[Math.min((int) (Math.abs(toGo) / SOFT_D_DOWN_DEGREE), speedRampDown.length - 1)], maxPower);

                // Scale the final speed against the input power.
                newSpeed = Math.min(spdDn, spdUp);

                // Only update the motors if we really need too
                if (newSpeed != actSpeed ) {

                    // Record the new base speed
                    actSpeed = newSpeed;

                    // Change and scale power, the direction is already baked into KsR and KsL
                    leftFDrive.setPower(Math.max(-1.0, Math.min(1.0, newSpeed * KpL * KsL)));
                    rightFDrive.setPower(Math.max(-1.0, Math.min(1.0, newSpeed * KpR * KsR)));
                    rightBDrive.setPower(Math.max(-1.0, Math.min(1.0, newSpeed * KpR * KsR)));
                    leftBDrive.setPower(Math.max(-1.0, Math.min(1.0, newSpeed * KpL * KsL)));
                }

                if (DEBUG) {
                    myLOpMode.telemetry.addData("left", "s: %1.3f",newSpeed * KpL * KsL);
                    myLOpMode.telemetry.addData("right", "s: %1.3f",newSpeed * KpR * KsR);
                    myLOpMode.telemetry.addData("gyro", "deg: %1.3f, curr: %1.3f, start: %1.3f, tgt: %1.3f",degrees,curHeading,startHeading,tgtHeading);
                    myLOpMode.telemetry.update();
                }
                // FIXME -- Read the front encoders only for logging
                //logger.logD("MechLogTurnCSV",String.format(",%f,%f,%d,%d,%d,%d,%f,%f,%f,%f", rt, getHeading(), leftFDrive.getCurrentPosition(), leftBDrive.getCurrentPosition(), rightFDrive.getCurrentPosition(), rightBDrive.getCurrentPosition(), Math.max(-1.0, Math.min(1.0, newSpeed * KpL * KsL)), Math.max(-1.0, Math.min(1.0, newSpeed * KpL * KsL)), Math.max(-1.0, Math.min(1.0, newSpeed * KpR * KsR)), Math.max(-1.0, Math.min(1.0, newSpeed * KpR * KsR))));
            }

            // Stop all motion;
            leftFDrive.setPower(0);
            rightFDrive.setPower(0);
            leftBDrive.setPower(0);
            rightBDrive.setPower(0);

            // Reset run mode
            leftFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (DEBUG) {
                myLOpMode.telemetry.addData("gyro", "d: %3.1f, c: %3.1f, s: %3.1f, t: %3.1f, e: %3.1f", degrees, curHeading, startHeading, tgtHeading, normalizeAngle(curHeading - tgtHeading));
                myLOpMode.telemetry.update();
                myLOpMode.sleep(1000);
            }
        }

        // The gyro is still changing at this point. Try to dump values and understand why.
        //for(int i=0; i<10; i++) {
        //    Orientation gyroOrien = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //    logger.logD("MechLogfooend", String.format("getHeading: x:%f, y:%f, z:%f", AngleUnit.DEGREES.fromUnit(gyroOrien.angleUnit, gyroOrien.thirdAngle), AngleUnit.DEGREES.fromUnit(gyroOrien.angleUnit, gyroOrien.secondAngle), AngleUnit.DEGREES.fromUnit(gyroOrien.angleUnit, gyroOrien.firstAngle) ));
        //    myLOpMode.sleep(100);
        //}
        // FIXME - this is three calls to getHeading(), need to fix this up to use a variable.
        logger.logD("MechLog",String.format("turn done: final: %f, get: %f, tgt: %f, err: %f", curHeading, getHeading(), tgtHeading, getHeading() - tgtHeading ));
        return(normalizeAngle(getHeading() - tgtHeading));
    }


    public double fastEncoderStrafe(double speed,
                                   double inchesToLeft,
                                   double timeoutS) {
        double a;
        double[] speedRampUp = {0.35, 0.45, 0.55, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0};
        double[] speedRampDown = {0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.675, 0.70, 0.725, 0.75, 0.775, 0.80, 0.825, 0.85, 0.875, 0.90, 0.925, 0.95, 0.975, 1.0};

        invertMotorDirection(leftFDrive);
        invertMotorDirection(rightBDrive);

        // Default value of P is 0.0 (no gyro assist)
        a = fastEncoderDrive( speed, inchesToLeft, inchesToLeft, timeoutS, 0.0 , speedRampUp, speedRampDown, 0.9);

        invertMotorDirection(rightBDrive);
        invertMotorDirection(leftFDrive);

        return( a ) ;

    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  IMU gyro is used to help steer if we're going straight.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public double fastEncoderDrive(double speed,
                                 double leftInches, double rightInches,
                                 double timeoutS) {

        // Default value of P is 0.0 (no gyro assist)
        return(fastEncoderDrive( speed, leftInches, rightInches, timeoutS,0.0 ));
    }

    public double fastEncoderDrive(double speed,
                                   double leftInches, double rightInches,
                                   double timeoutS, double P) {

        double[] speedRampUp = {0.20, 0.225, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0};
        double[] speedRampDown = {0.10, 0.125, 0.125, 0.15, 0.175, 0.20, 0.225, 0.25, 0.275, 0.30, 0.325, 0.35, 0.375, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0};

        double[] speedRampUpR = {0.325, 0.40, 0.475, 0.55, 0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0};
        double[] speedRampDownR = {0.275, 0.30, 0.325, 0.35, 0.375, 0.40, 0.425, 0.45, 0.475, 0.5, 1.0};

        double[] speedRampUpT = {0.325, 0.40, 0.475, 0.55, 0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0};
        double[] speedRampDownT = {0.275, 0.30, 0.325, 0.35, 0.375, 0.40, 0.425, 0.45, 0.475, 0.5, 1.0};

        if (leftInches == -rightInches) {
            speedRampUp = speedRampUpR;
            speedRampDown = speedRampDownR;
        } else if (leftInches != rightInches) {
            speedRampUp = speedRampUpT;
            speedRampDown = speedRampDownT;
        }

        return(fastEncoderDrive( speed, leftInches, rightInches, timeoutS, P, speedRampUp, speedRampDown ));
    }


    public double fastEncoderDrive(double speed,
                                   double leftInches, double rightInches,
                                   double timeoutS, double P, double[] up, double[] down) {
        return (fastEncoderDrive(speed, leftInches, rightInches, timeoutS, P, up, down, 1.0));
    }
        /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  IMU gyro is used to help steer if we're going straight.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public double fastEncoderDrive(double speed,
                               double leftInches, double rightInches,
                               double timeoutS, double P, double[] up, double[] down,
                               double strafeCorrect) {

        int newLeftFTarget;
        int newRightFTarget;
        int newLeftBTarget;
        int newRightBTarget;
        double curPosL, curPosR;
        double curPosLf, curPosRf;
        double toGoL, toGoR;
        double actSpeedL=0, actSpeedR=0, spdPosL=0, spdPosR=0, spdToGoL=0, spdToGoR=0;
        double newSpeedL, newSpeedR;
        double spdUpL,spdDnL,spdUpR,spdDnR;
        boolean doneL, doneR;
        double[] speedRampUp = up;
        double[] speedRampDown = down;
        ElapsedTime     runtime = new ElapsedTime();
        double tgtHeading, curHeading, deltaHeading;
        // For gyro heading adjustments
        double KhL = 1.0;
        double KhR = 1.0;
        double rt = 0.0;

        // Get the current Heading and update for the correction angle
        curHeading = tgtHeading = getHeading();
        logger.logD("MechLog",String.format("fastEncodeDrive: tgt: %f, A: %f", tgtHeading, straightA));
        tgtHeading = normalizeAngle(tgtHeading - straightA) ;

        // now clear the correction angle so it's never used again
        straightA = 0.0;

        // Ensure that the opmode is still active
        if (myLOpMode.opModeIsActive()) {

            // Reset encoders
            leftFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // We'll handle the power/speed/encoders
            leftFDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // The front wheels are slipping so they stop and fight the back.  Set them
            // to FLOAT here to try to avoid that.  Not sure if this actually works
            leftFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            //leftBDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            //rightBDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // Compute desired position
            newLeftFTarget = (int)((leftInches * COUNTS_PER_INCH)*strafeCorrect);
            newRightFTarget = (int)(rightInches * COUNTS_PER_INCH);
            newLeftBTarget = (int)(leftInches * COUNTS_PER_INCH);
            newRightBTarget = (int)((rightInches * COUNTS_PER_INCH)*strafeCorrect);

            // Initialze toGo == target (maximum toGo)
            toGoL = Math.abs(newLeftBTarget);
            toGoR = Math.abs(newRightBTarget);

            // reset the timeout time and start motion at the minimum speed.
            runtime.reset();
            speed = Math.abs(speed);

            // Setting the starting speed is now handled inside the loop below on the first iteration.
            // Don't set the speed out here because a 'pivot' where one side has a '0' distance
            // request would start moving too soon.

            // Initialize variables before the loop
            doneL = doneR = false;
            spdUpL = spdUpR = newSpeedL = newSpeedR = speedRampUp[0];
            spdDnL = spdDnR = speedRampDown[0];
            curPosL = leftBDrive.getCurrentPosition();
            curPosR = rightBDrive.getCurrentPosition();

            // FIXME -- Read the front encoders only for logging
            //curPosLf = leftFDrive.getCurrentPosition();
            //curPosRf = rightFDrive.getCurrentPosition();

            // Keep looping while we are still active, and there is time left, and we haven't reached the target
            while (myLOpMode.opModeIsActive() &&
                    ((rt=runtime.seconds()) < timeoutS) &&
                    ( !doneL || !doneR)) {

                // Check if we're done
                // This code implements a soft start and soft stop.
                if (!doneL) {
                    spdPosL = curPosL = leftBDrive.getCurrentPosition();
                    toGoL = Math.min(toGoL, Math.max(0, Math.abs(newLeftBTarget - curPosL)));
                    // FIXME -- Read the front encoders only for logging
                    //curPosLf = leftFDrive.getCurrentPosition();
                }
                if (!doneR) {
                    spdPosR = curPosR = rightBDrive.getCurrentPosition();
                    toGoR  = Math.min(toGoR, Math.max(0,Math.abs(newRightBTarget - curPosR)));
                    // FIXME -- Read the front encoders only for logging
                    //curPosRf = rightFDrive.getCurrentPosition();
                }
                if (leftInches == rightInches) {
                    spdPosL = spdPosR = (curPosL + curPosR)/2;
                    toGoL = toGoR = (toGoL + toGoR)/2;
                }

                if (!doneL) {
                    doneL = ((Math.abs(newLeftBTarget) - Math.abs(curPosL)) < CLOSE_ENOUGH);

                    // Compute speed on acceleration and deceleration legs
                    spdUpL = Math.max(Math.min(speedRampUp[Math.min((int) (Math.abs(spdPosL) / SOFT_D_UP), speedRampUp.length - 1)], speed),minUp);
                    spdDnL = Math.min(speedRampDown[Math.min((int) (Math.abs(toGoL) / SOFT_D_DOWN2), speedRampDown.length - 1)], speed);

                    // Use the minimum speed or 0
                    newSpeedL = doneL ? 0.05 : Math.min(spdUpL, spdDnL);

                    // Change power and steer
                    // Reverse stuff if driving backwards
                    if ( newLeftFTarget < 0 ) {
                        newSpeedL *= -1.0;
                    }
                }

                if (!doneR) {
                    doneR = ((Math.abs(newRightBTarget) - Math.abs(curPosR)) < CLOSE_ENOUGH);

                    // Compute speed on acceleration and deceleration legs
                    spdUpR = Math.max(Math.min(speedRampUp[Math.min((int)(Math.abs(spdPosR)/SOFT_D_UP),speedRampUp.length-1)],speed),minUp);
                    spdDnR = Math.min(speedRampDown[Math.min((int)(Math.abs(toGoR)/SOFT_D_DOWN2),speedRampDown.length-1)],speed);

                    // Use the minimum speed or 0
                    newSpeedR = doneR ? 0.05 : Math.min(spdUpR, spdDnR) ;

                    // Change power and steer
                    // Reverse stuff if driving backwards
                    if ( newRightFTarget < 0 ) {
                        newSpeedR *= -1.0;
                    }
                }

                curHeading = getHeading();
                deltaHeading = -1.0 * normalizeAngle(tgtHeading - curHeading);

                // Compute drift, negate for correction
                if ((P != 0.0) && (leftInches == rightInches)) {
                    // If driving straight backwards, negate again
                    if (leftInches < 0 ) {
                        deltaHeading *= -1.0;
                    }
                    // Never change the sign. Make the change proportional to the error
                    KhL = Math.max(0.0, (1.0 + (deltaHeading * P)));
                    KhR = Math.max(0.0, (1.0 - (deltaHeading * P)));
                }

                newSpeedL *= (KpL * KhL);
                newSpeedR *= (KpR * KhR);

                if ((newSpeedL != actSpeedL ) || (newSpeedR != actSpeedR )) {

                    leftFDrive.setPower(Math.max(-1.0, Math.min(1.0, strafeCorrect*newSpeedL)));
                    rightFDrive.setPower(Math.max(-1.0, Math.min(1.0, newSpeedR)));
                    rightBDrive.setPower(Math.max(-1.0, Math.min(1.0, strafeCorrect*newSpeedR)));
                    leftBDrive.setPower(Math.max(-1.0, Math.min(1.0, newSpeedL)));
                    actSpeedL = newSpeedL;
                    actSpeedR = newSpeedR;
                }

                //logger.logD("MechLogDriveCSV",String.format(",%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", rt, deltaHeading, curPosLf, curPosL, curPosRf, curPosR, Math.max(-1.0, Math.min(1.0, newSpeedL)), Math.max(-1.0, Math.min(1.0, newSpeedL)), Math.max(-1.0, Math.min(1.0, newSpeedR)), Math.max(-1.0, Math.min(1.0, newSpeedR))));

                if (DEBUG) {
                    myLOpMode.telemetry.addData("left", "up: %1.3f, dn: %1.3f, s: %1.3f, p: %5d, t: %5d, g: %5d", spdUpL, spdDnL, newSpeedL, (int) curPosL, newLeftBTarget, (int) toGoL);
                    myLOpMode.telemetry.addData("rght", "up: %1.3f, dn: %1.3f, s: %1.3f, p: %5d, t: %5d, g: %5d", spdUpR, spdDnR, newSpeedR, (int) curPosR, newRightBTarget, (int) toGoR);
                    myLOpMode.telemetry.update();
                }
            }

            // The front wheels are slipping so they stop and fight the back.  Set them
            // to FLOAT earlier, so reset to BRAKE here.
            // No longer needed in this mode.
            leftFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //leftBDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //rightBDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Stop all motion;
            leftFDrive.setPower(0);
            rightFDrive.setPower(0);
            leftBDrive.setPower(0);
            rightBDrive.setPower(0);

            if (DEBUG) {
                curPosL = leftBDrive.getCurrentPosition();
                curPosR = rightBDrive.getCurrentPosition();
                myLOpMode.telemetry.addData("left", "up: %1.3f, dn: %1.3f, s: %1.3f, p: %5d, t: %5d, g: %5d", spdUpL, spdDnL, newSpeedL, (int) curPosL, newLeftBTarget, (int) toGoL);
                myLOpMode.telemetry.addData("rght", "up: %1.3f, dn: %1.3f, s: %1.3f, p: %5d, t: %5d, g: %5d", spdUpR, spdDnR, newSpeedR, (int) curPosR, newRightBTarget, (int) toGoR);
                myLOpMode.telemetry.update();
            }

            // Reset run mode
            leftFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        if (leftInches == rightInches) {
            // FIXME - this is three calls to getHeading(), need to fix this up to use a variable.
            logger.logD("MechLog",String.format("straight done: final: %f, get: %f, tgt: %f, err: %f", curHeading, getHeading(), tgtHeading, getHeading() - tgtHeading ));
            return (normalizeAngle((getHeading() - tgtHeading)));
        } else {
            return(0.0);
        }
    }
}


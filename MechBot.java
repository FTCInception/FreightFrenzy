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

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

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
    public DcMotorEx l_f_motor, l_b_motor, r_f_motor, r_b_motor;
    public DcMotor intake_motor;
    public DcMotorEx shoot1_motor, shoot2_motor;
    public DcMotorEx wobble_motor;
    public Servo claw=null,flicker=null;

    private static final double COUNTS_PER_MOTOR_REV = (537.6);         // eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = (10.0/10.0);     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = (96.0/25.4);       // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159);
    private static final double AXLE_LENGTH = 17.0;          // Width of robot through the pivot point (center wheels)

    // The gyro seems to be a teeny/tiny off, this is the correction factor
    private static final double IMU_CORR = 1.00445;
    // Multiply INCHES_PER_DEGREE by 1.75 to handle the sliding of the MECH wheel rollers?
    private static final double INCHES_PER_DEGREE = 1.52 * ( (AXLE_LENGTH * 3.14159) / 360.0 );
    // This controls the rate of increase or decrease of speed, expressed as endcoder counts or degrees
    private static final double SOFT_D_UP = 50.0;
    private static final double SOFT_D_UP_DEGREE = 5;
    private static final double SOFT_D_DOWN = 70.0;
    private static final double SOFT_D_DOWN2 = 70.0;
    private static final double SOFT_D_DOWN_DEGREE = 5;
    // This determines when we've reached the end point, when are we done?
    private static final double CLOSE_ENOUGH = 15.0;
    private static final double CLOSE_ENOUGH_DEGREE = 2.0;
    // Once a side reaches the target, this controls the speed it continues to run
    // at waiting on the other side.  We think this is better than '0'
    private static final double WAIT_SPEED = 0.005;
    // Staic ratio between left and right power for 'normal' driving
    private static final double fKpL = 1.0;
    private static final double fKpR = 1.0;
    //private static final double fKpR = 0.997;

    // Staic ratio between left and right power for 'strafe' driving
    private static final double sKpL = 0.90;
    private static final double sKpR = 1.0;
    // Seems like we have slipping while strafing?
    // This seems to match the 'right' side (front of bot).
    // Maybe this is due to weight/balance issues?
    private static final double STRAFE_DISTANCE_FACTOR = 0.90;

    private static final double PIVOT_FACTOR = 2.05;
    private static final double RIGHT_ARC_COEFFICENT = 1.00;
    private static final double LEFT_ARC_COEFFICENT = 1.25;
    static final double RIGHT = 1;
    static final double LEFT = 0;

    //final double WOBBLE_TICKS_PER_DEGREE = 5264.0/360.0; // 30 RPM 6mm d-shaft (5202 series)
    //final double WOBBLE_TICKS_PER_DEGREE = 2786.0/360.0; // 60 RPM 6mm d-shaft (5202 series)
    final double WOBBLE_TICKS_PER_DEGREE = 3892.0/360.0; // 43 RPM 8mm REX (5203 series)
    final int wobbleTargets[] = {(int)(5*WOBBLE_TICKS_PER_DEGREE),(int)(225*WOBBLE_TICKS_PER_DEGREE), (int)(90*WOBBLE_TICKS_PER_DEGREE), (int)(175*WOBBLE_TICKS_PER_DEGREE)};

    static double KpL;
    static double KpR;
    BNO055IMU imu;
    private Orientation angles;
    private Orientation lastAngles = new Orientation();
    private double lastQ = 0;
    private double globalHeading = 0;
    private boolean useIntegratedGyro = true;  // Controls whether we use raw gyro values or the integrated version we make.
    private static final boolean DEBUG = false;
    double minUp = 0.0;
    double straightA = 0.0;
    public BotLog logger = new BotLog();

    /* local OpMode members. */
    HardwareMap hardwareMap = null;
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
        imu = hardwareMap.get(BNO055IMU.class, "imu");

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
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
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
        myLOpMode.telemetry.addData("Status", "Resetting Encoders");
        myLOpMode.telemetry.addData("Status", "Zeroing wobble arm");
        myLOpMode.telemetry.update();

        // Reset encoders 
        l_f_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l_b_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r_f_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r_b_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l_f_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        l_b_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r_f_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r_b_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Run the wobble arm for a teeny little bit at low power just to take out and slack...
        wobble_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobble_motor.setPower(-0.033);
        myLOpMode.sleep(500);
        wobble_motor.setPower(0.0);
        wobble_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobble_motor.setTargetPosition(0);
        wobble_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        myLOpMode.telemetry.addData("Mode", "IMU calibrating...");
        myLOpMode.telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        for (int i=0; (i<40 && !myLOpMode.isStopRequested() && !imu.isGyroCalibrated()); i++)
        {
            myLOpMode.sleep(50);
            myLOpMode.idle();
        }

        resetIntHeading();

        //colorSensor = hardwareMap.colorSensor.get("color");
        //colorSensor.enableLed(false);

        // Adjust our overall power based on a 12.5V battery.
        // Set some min and max to avoid anything crazy.
        // This may need some more characterization and it may be battery specific.
        double volts = Range.clip(getBatteryVoltage(),11.0,13.0);
        KpL = fKpL * (12.5 / volts);
        KpR = fKpR * (12.5 / volts);

        KpL = fKpL;
        KpR = fKpR;

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
        hardwareMap = ahwMap;

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
        l_f_motor.setPower(0.0);
        l_b_motor.setPower(0.0);
        r_f_motor.setPower(0.0);
        r_b_motor.setPower(0.0);
        l_f_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        l_b_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        r_f_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        r_b_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        l_f_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        l_b_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r_f_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r_b_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //l_f_motor.setVelocityPIDFCoefficients(2.0,0.5,0.0,11.1);
        //l_b_motor.setVelocityPIDFCoefficients(2.0,0.5,0.0,11.1);
        //r_f_motor.setVelocityPIDFCoefficients(2.0,0.5,0.0,11.1);
        //r_b_motor.setVelocityPIDFCoefficients(2.0,0.5,0.0,11.1);
        l_f_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l_b_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r_f_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r_b_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l_f_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        l_b_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r_f_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r_b_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake_motor = hardwareMap.dcMotor.get("intake");
        intake_motor.setPower(0.0);
        intake_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Zero out the wobble motor in the auto init
        wobble_motor = hardwareMap.get(DcMotorEx.class,"wobble");
        wobble_motor.setPower(0.0);
        wobble_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        wobble_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobble_motor.setTargetPosition(0);
        wobble_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobble_motor.setPositionPIDFCoefficients(5.0);
        wobble_motor.setVelocityPIDFCoefficients(2.0,0.5,0.0,11.1);

        shoot1_motor = hardwareMap.get(DcMotorEx.class,"shoot1");
        shoot2_motor = hardwareMap.get(DcMotorEx.class,"shoot2");
        shoot1_motor.setPower(0.0);
        shoot2_motor.setPower(0.0);
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
        claw.setPosition(1.0);

        flicker = hardwareMap.servo.get("flicker");
        flicker.setPosition(0.0);
    }

    //Used to change motor direction before and after strafing.
    public void invertMotorDirection(DcMotor motor) {
        if (motor.getDirection() == DcMotor.Direction.FORWARD) {
            motor.setDirection(DcMotor.Direction.REVERSE);
        } else {
            motor.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    public double fastEncoderStraight(double speed, double distance, double timeoutS, double P, double flickL, double flickR) {
        return(fastEncoderDrive( speed, distance, distance, timeoutS, P, flickL, flickR ));
    }

    public double fastEncoderStraight(double speed, double distance, double timeoutS, double P) {
        return(fastEncoderDrive( speed, distance, distance, timeoutS, P ));
    }

    public double fastEncoderStraight(double speed, double distance, double timeoutS) {
        return(fastEncoderDrive( speed, distance, distance, timeoutS ));
    }

    public void fastEncoderRotate(double speed, double degrees, double timeoutS) {
        fastEncoderDrive( speed, degrees * INCHES_PER_DEGREE, -degrees * INCHES_PER_DEGREE, timeoutS );
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
            OFDrive = l_f_motor ;
            OBDrive = l_b_motor ;
            IFDrive = r_f_motor ;
            IBDrive = r_b_motor ;
            KpO = KpL ;
            KpI = KpR ;
            InnerDist *= RIGHT_ARC_COEFFICENT ;
            OuterDist *= RIGHT_ARC_COEFFICENT ;
            tgtPos *= RIGHT_ARC_COEFFICENT ;
        } else {
            OFDrive = r_f_motor ;
            OBDrive = r_b_motor ;
            IFDrive = l_f_motor ;
            IBDrive = l_b_motor ;
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
            l_f_motor.setPower(0);
            r_f_motor.setPower(0);
            l_b_motor.setPower(0);
            r_b_motor.setPower(0);

            // Turn off RUN_WITHOUT_ENCODERS
            l_f_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r_f_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            l_b_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r_b_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(500);   // optional pause after each move
        }
    }


    /**
            l_f_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            l_f_motor.setTargetPosition(ticks);
            // Turn On RUN_TO_POSITION
            l_f_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            l_f_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // reset the timeout time and start motion at the minimum speed.
            runtime.reset();
            l_f_motor.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (myLOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (l_b_motor.isBusy() && r_b_motor.isBusy())) {

            }

            // Stop all motion;
            l_f_motor.setPower(0);

            // Turn off RUN_TO_POSITION
            l_f_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

        myLOpMode.sleep(500);
        logger.logD("MechLog",String.format("encoderTest end: %d, %d",l_b_motor.getCurrentPosition(),r_b_motor.getCurrentPosition()));
    }
    **/

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
        double          delta;

        gyroOrien = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        delta = AngleUnit.DEGREES.fromUnit(gyroOrien.angleUnit, gyroOrien.firstAngle) - AngleUnit.DEGREES.fromUnit(gyroOrien.angleUnit, lastAngles.firstAngle);

        if (delta <= -180.0) {
            delta += 360.0;
        } else if (delta > 180.0) {
            delta -= 360.0;
        }

        globalHeading += (delta * IMU_CORR);

        lastAngles = gyroOrien;

        //logger.logD("MechLog",String.format(" getIntHeading: %f, %f", delta*IMU_CORR, globalHeading));

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
        //double[] speedRampUp = {0.20, 0.25, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0};
        //double[] speedRampDown = {0.1, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0};
        double[] speedRampUp = {0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0};
        double[] speedRampDown = {0.30, 0.35, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.0};

        // This profile is for rotate
        //ult double[] speedRampUpR = {0.275, 0.325, 0.40, 0.475, 0.55, 0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0};
        //ult double[] speedRampDownR = {0.20, 0.20, 0.225, 0.225, 0.25, 0.25, 0.275, 0.275, 0.30, 0.30, 0.325, 0.325, 0.35, 0.35, 0.375, 0.40, 0.425, 0.45, 0.475, 0.5, 1.0};
        double[] speedRampUpR = {0.30, 0.45, 0.60, 0.75, 0.90, 1.0};
        double[] speedRampDownR = {0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0};
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

            l_f_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            r_f_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            l_b_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            r_b_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // We'll handle the power/speed/encoders
            l_f_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            r_f_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            l_b_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            r_b_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            l_f_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            r_f_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            l_b_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            r_b_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
                    l_f_motor.setPower(Math.max(-1.0, Math.min(1.0, newSpeed * KpL * KsL)));
                    r_f_motor.setPower(Math.max(-1.0, Math.min(1.0, newSpeed * KpR * KsR)));
                    r_b_motor.setPower(Math.max(-1.0, Math.min(1.0, newSpeed * KpR * KsR)));
                    l_b_motor.setPower(Math.max(-1.0, Math.min(1.0, newSpeed * KpL * KsL)));
                }

                if (DEBUG) {
                    myLOpMode.telemetry.addData("left", "s: %1.3f",newSpeed * KpL * KsL);
                    myLOpMode.telemetry.addData("right", "s: %1.3f",newSpeed * KpR * KsR);
                    myLOpMode.telemetry.addData("gyro", "deg: %1.3f, curr: %1.3f, start: %1.3f, tgt: %1.3f",degrees,curHeading,startHeading,tgtHeading);
                    myLOpMode.telemetry.update();
                }
                // FIXME -- Read the front encoders only for logging
                /*
                Orientation gyroOrien = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                Quaternion q = imu.getQuaternionOrientation();
                double t=q.x*q.y + q.z*q.w;
                double h,a,b;
                if (t > 0.4999) {
                    h = 2*Math.atan2(q.x,q.w);
                    a = Math.PI/2;
                    b = 0;
                } else if (t <  -0.4999) {
                    h = -2*Math.atan2(q.x,q.w);
                    a = -Math.PI/2;
                    b = 0;
                } else {
                    double sqx = q.x * q.x;
                    double sqy = q.y * q.y;
                    double sqz = q.z * q.z;
                    h = Math.atan2(2 * q.y * q.w - 2 * q.x * q.z, 1 - 2 * sqy - 2 * sqz);
                    a = Math.asin(2 * t);
                    b = Math.atan2(2 * q.x * q.w - 2 * q.y * q.z, 1 - 2 * sqx - 2 * sqz);
                }
                logger.logD("MechLogfooTurn", String.format("quat: h:%f, a:%f, b:%f", (h/Math.PI)*180, (a/Math.PI)*180, (b/Math.PI)*180 ));
                logger.logD("MechLogfooTurn", String.format("getHeading: x:%f, y:%f, z:%f", AngleUnit.DEGREES.fromUnit(gyroOrien.angleUnit, gyroOrien.thirdAngle), AngleUnit.DEGREES.fromUnit(gyroOrien.angleUnit, gyroOrien.secondAngle), AngleUnit.DEGREES.fromUnit(gyroOrien.angleUnit, gyroOrien.firstAngle) ));
                */
                //logger.logD("MechLogfooTurnCSV",String.format(",%f,%f,%d,%d,%d,%d,%f,%f,%f,%f", rt, getHeading(), l_f_motor.getCurrentPosition(), l_b_motor.getCurrentPosition(), r_f_motor.getCurrentPosition(), r_b_motor.getCurrentPosition(), Math.max(-1.0, Math.min(1.0, newSpeed * KpL * KsL)), Math.max(-1.0, Math.min(1.0, newSpeed * KpL * KsL)), Math.max(-1.0, Math.min(1.0, newSpeed * KpR * KsR)), Math.max(-1.0, Math.min(1.0, newSpeed * KpR * KsR))));
            }

            // Stop all motion;
            l_f_motor.setPower(0);
            r_f_motor.setPower(0);
            l_b_motor.setPower(0);
            r_b_motor.setPower(0);

            // Reset run mode
            l_f_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r_f_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            l_b_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r_b_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (DEBUG) {
                myLOpMode.telemetry.addData("gyro", "d: %3.1f, c: %3.1f, s: %3.1f, t: %3.1f, e: %3.1f", degrees, curHeading, startHeading, tgtHeading, normalizeAngle(curHeading - tgtHeading));
                myLOpMode.telemetry.update();
                myLOpMode.sleep(1000);
            }
        }

        /*
        // The gyro is still changing at this point. Try to dump values and understand why.
        for(int i=0; i<20; i++) {
            Orientation gyroOrien = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            Quaternion q = imu.getQuaternionOrientation();
            double t=q.x*q.y + q.z*q.w;
            double h,a,b;
            if (t > 0.4999) {
                h = 2*Math.atan2(q.x,q.w);
                a = Math.PI/2;
                b = 0;
            } else if (t <  -0.4999) {
                h = -2*Math.atan2(q.x,q.w);
                a = -Math.PI/2;
                b = 0;
            } else {
                double sqx = q.x * q.x;
                double sqy = q.y * q.y;
                double sqz = q.z * q.z;
                h = Math.atan2(2 * q.y * q.w - 2 * q.x * q.z, 1 - 2 * sqy - 2 * sqz);
                a = Math.asin(2 * t);
                b = Math.atan2(2 * q.x * q.w - 2 * q.y * q.z, 1 - 2 * sqx - 2 * sqz);
            }
            logger.logD("MechLogfooend", String.format("quat: h:%f, a:%f, b:%f", (h/Math.PI)*180, (a/Math.PI)*180, (b/Math.PI)*180 ));
            logger.logD("MechLogfooend", String.format("getHeading: x:%f, y:%f, z:%f", AngleUnit.DEGREES.fromUnit(gyroOrien.angleUnit, gyroOrien.thirdAngle), AngleUnit.DEGREES.fromUnit(gyroOrien.angleUnit, gyroOrien.secondAngle), AngleUnit.DEGREES.fromUnit(gyroOrien.angleUnit, gyroOrien.firstAngle) ));
            logger.logD("MechLogfooTurnCSV",String.format(",%f,%f,%d,%d,%d,%d,%f,%f,%f,%f", runtime.seconds(), getHeading(), l_f_motor.getCurrentPosition(), l_b_motor.getCurrentPosition(), r_f_motor.getCurrentPosition(), r_b_motor.getCurrentPosition(), 0.0,0.0,0.0,0.0));
            myLOpMode.sleep(100);
        }
        */

        // FIXME - this is three calls to getHeading(), need to fix this up to use a variable.
        logger.logD("MechLog",String.format("turn done: final: %f, get: %f, tgt: %f, err: %f", curHeading, getHeading(), tgtHeading, getHeading() - tgtHeading ));
        return(normalizeAngle(getHeading() - tgtHeading));
    }


    public double fastEncoderStrafe(double speed,
                                   double inchesToLeft,
                                   double timeoutS) {

        // Default value of P is 0.0 (no gyro assist)
        return(fastEncoderStrafe( speed, inchesToLeft, timeoutS, 0.0));
    }

    public double fastEncoderStrafe(double speed,
                                    double inchesToLeft,
                                    double timeoutS,
                                    double P) {

        // Default is no flicking
        return(fastEncoderStrafe( speed, inchesToLeft, timeoutS, P, 0.0, 0.0));
    }

    public double fastEncoderStrafe(double speed,
                                    double inchesToLeft,
                                    double timeoutS,
                                    double P,
                                    double flickL, double flickR ) {
        double a;

        // Long straight (default)
        double[] speedRampUp = {0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.0};
        double[] speedRampDown = {0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.0};

        invertMotorDirection(l_f_motor);
        invertMotorDirection(r_b_motor);

        a = fastEncoderDrive2( speed, inchesToLeft, inchesToLeft, timeoutS, P, flickL, flickR, speedRampUp, speedRampDown, l_b_motor, r_b_motor, l_f_motor, r_f_motor);

        invertMotorDirection(r_b_motor);
        invertMotorDirection(l_f_motor);

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
        return(fastEncoderDrive( speed, leftInches, rightInches, timeoutS, 0.0 ));
    }

    public double fastEncoderDrive(double speed,
                                   double leftInches, double rightInches,
                                   double timeoutS, double P) {

        // Default is no flicking
        return(fastEncoderDrive( speed, leftInches, rightInches, timeoutS, P, 0.0, 0.0 ));
    }


    public double fastEncoderDrive(double speed,
                                   double leftInches, double rightInches,
                                   double timeoutS, double P, double flickL, double flickR) {

        // Rotate acceleration curves
        //double[] speedRampUpR = {0.325, 0.40, 0.475, 0.55, 0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0};
        //double[] speedRampDownR = {0.275, 0.30, 0.325, 0.35, 0.375, 0.40, 0.425, 0.45, 0.475, 0.5, 1.0};
        double[] speedRampUpR = {0.30, 0.45, 0.60, 0.75, 0.90, 1.0};
        double[] speedRampDownR = {0.15, 0.30, 0.45, 0.60, 0.75, 0.90, 1.0};

        // Generic turn acceleration curves
        double[] speedRampUpT = {0.325, 0.40, 0.475, 0.55, 0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0};
        double[] speedRampDownT = {0.275, 0.30, 0.325, 0.35, 0.375, 0.40, 0.425, 0.45, 0.475, 0.5, 1.0};

        // Long straight (default)
        //double[] speedRampUp = {0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.0};
        double[] speedRampUp = {0.30, 0.40, 0.55, 0.70, 0.85, 1.0};
        //double[] speedRampDown = {0.10, 0.125, 0.125, 0.15, 0.175, 0.20, 0.225, 0.25, 0.275, 0.30, 0.325, 0.35, 0.375, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0};
        //double[] speedRampDown = {0.20, 0.225, 0.25, 0.275, 0.30, 0.325, 0.35, 0.375, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0};
        //double[] speedRampDown = {0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.0};
        double[] speedRampDown = {0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.0};
        //double[] speedRampDown = {0.20, 0.30, 0.40, 0.60, 0.80, 1.0};

        // Medium straight
        double[] speedRampUp30 = {0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.0};
        // Good profile for picking up wobble! double[] speedRampDown30 = {0.10, 0.125, 0.175, 0.225, 0.275, 0.325, 0.375, 0.45,};
        double[] speedRampDown30 = {0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.0};

        // Short straight
        double[] speedRampUp16 = {0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.0};
        double[] speedRampDown16 = {0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.0};
        //double[] speedRampDown16 = {0.10, 0.15, 0.225, 0.35};

        // Choose some accel/decel curves based on distance
        // Need more time to decelerate for longer distances (higher top speeds)
        if (leftInches == rightInches) {
            if (Math.abs(leftInches) <= 16.0) {
                speedRampUp = speedRampUp16;
                speedRampDown = speedRampDown16;
            } else if (Math.abs(leftInches) <= 30.0) {
                speedRampUp = speedRampUp30;
                speedRampDown = speedRampDown30;
            }
        } else if (leftInches == -rightInches) {
            speedRampUp = speedRampUpR;
            speedRampDown = speedRampDownR;
        } else if (leftInches != rightInches) {
            speedRampUp = speedRampUpT;
            speedRampDown = speedRampDownT;
        }

        /**
        // Experimentally determined driving results.
        // The accel curves produce some shift in projected distances.
        // These adjustments approximate the actual distances.
        if (leftInches == rightInches) {
            if (Math.abs(leftInches) <= 12.0) {
                leftInches *= (477/COUNTS_PER_MOTOR_REV);
                rightInches = leftInches;
            } else if (Math.abs(leftInches) <= 24.0) {
                leftInches *= (487/COUNTS_PER_MOTOR_REV);
                rightInches = leftInches;
            } else if (Math.abs(leftInches) <= 48.0) {
                leftInches *= (508 / COUNTS_PER_MOTOR_REV);
                rightInches = leftInches;
            } else if (Math.abs(leftInches) <= 96.0) {
                leftInches *= (508 / COUNTS_PER_MOTOR_REV);
                rightInches = leftInches;
            }
        }
        **/

        return(fastEncoderDrive( speed, leftInches, rightInches, timeoutS, P, flickL, flickR, speedRampUp, speedRampDown));
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
                               double timeoutS, double P,
                               double flickL, double flickR,
                               double[] up, double[] down) {


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
        double fRT = 0.0;

        // Get the current Heading and update for the correction angle
        curHeading = tgtHeading = getHeading();
        logger.logD("MechLog",String.format("fastEncodeDrive: tgt: %f, A: %f", tgtHeading, straightA));
        tgtHeading = normalizeAngle(tgtHeading - straightA) ;

        // now clear the correction angle so it's never used again
        straightA = 0.0;

        // Ensure that the opmode is still active
        if (myLOpMode.opModeIsActive()) {

            // Reset encoders
            l_f_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            r_f_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            l_b_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            r_b_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // We'll handle the power/speed/encoders
            //l_f_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //r_f_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //l_b_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //r_b_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // FIXME -- Lets just run in this mode all the time and not keep programming the motors
            // We'll handle power/steering but let the PID try to maintain
            // a constant axle speed to overcome hub, motor, build variability.
            l_f_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r_f_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            l_b_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r_b_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // FIXME -- Lets just run in this mode all the time and not keep programming the motors
            // Braking is good if we have gyro correct
            l_f_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            r_f_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            l_b_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            r_b_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Compute desired position
            newLeftFTarget = (int)(leftInches * COUNTS_PER_INCH);
            newRightFTarget = (int)(rightInches * COUNTS_PER_INCH);
            newLeftBTarget = (int)(leftInches * COUNTS_PER_INCH);
            newRightBTarget = (int)(rightInches * COUNTS_PER_INCH);

            flickR *= COUNTS_PER_INCH;
            flickL *= COUNTS_PER_INCH;

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
            curPosL = l_b_motor.getCurrentPosition();
            curPosR = r_b_motor.getCurrentPosition();

            // FIXME -- Read the front encoders only for logging
            curPosLf = l_f_motor.getCurrentPosition();
            curPosRf = r_f_motor.getCurrentPosition();

            // Keep looping while we are still active, and there is time left, and we haven't reached the target
            while (myLOpMode.opModeIsActive() &&
                    ((rt=runtime.seconds()) < timeoutS) &&
                    ( !doneL || !doneR)) {

                // Check if we're done
                // This code implements a soft start and soft stop.
                // Compute the distance gone and how far to go.
                if (!doneL) {
                    spdPosL = curPosL = l_b_motor.getCurrentPosition();
                    toGoL = Math.min(toGoL, Math.max(0, Math.abs(newLeftBTarget - curPosL)));
                }
                if (!doneR) {
                    spdPosR = curPosR = r_b_motor.getCurrentPosition();
                    toGoR  = Math.min(toGoR, Math.max(0,Math.abs(newRightBTarget - curPosR)));
                }
                // Average them if driving straight to reduce twist/steer.
                if (leftInches == rightInches) {
                    spdPosL = spdPosR = (curPosL + curPosR)/2;
                    toGoL = toGoR = (toGoL + toGoR)/2;
                }

                // If there is a flick target and we aren't already flicking and its past our flick target
                if ((flickL != 0) && (fRT == 0) && (Math.abs(spdPosL) > Math.abs(flickL))) {
                    // flick
                    flicker.setPosition(1.0);
                    // Set the time to reset the flick
                    fRT = rt + 500;
                } else if ((flickR != 0) && (fRT == 0) && (Math.abs(spdPosR) > Math.abs(flickR))) {
                    // flick
                    flicker.setPosition(1.0);
                    // Set the time to reset the flick
                    fRT = rt + 500;
                }
                //} else if (fRT > rt) {
                //    flicker.setPosition(0.0);
                //    // We already flicked, no more flicking flicker.
                //    fRT = -1.0;
                //}

                if (!doneL) {
                    doneL = ((Math.abs(newLeftBTarget) - Math.abs(curPosL)) < CLOSE_ENOUGH);

                    // Compute speed on acceleration and deceleration legs
                    spdUpL = Math.max(Math.min(speedRampUp[Math.min((int) (Math.abs(spdPosL) / SOFT_D_UP), speedRampUp.length-1)], speed),minUp);
                    spdDnL = Math.min(speedRampDown[Math.min((int) (Math.abs(toGoL) / SOFT_D_DOWN2), speedRampDown.length-1)], speed);

                    // Use the minimum speed or 0
                    newSpeedL = doneL ? WAIT_SPEED : Math.min(spdUpL, spdDnL);

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
                    newSpeedR = doneR ? WAIT_SPEED : Math.min(spdUpR, spdDnR) ;

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

                if ((newSpeedL != actSpeedL) || (newSpeedR != actSpeedR)) {

                    l_f_motor.setPower(Math.max(-1.0, Math.min(1.0, newSpeedL)));
                    r_f_motor.setPower(Math.max(-1.0, Math.min(1.0, newSpeedR)));
                    r_b_motor.setPower(Math.max(-1.0, Math.min(1.0, newSpeedR)));
                    l_b_motor.setPower(Math.max(-1.0, Math.min(1.0, newSpeedL)));
                    actSpeedL = newSpeedL;
                    actSpeedR = newSpeedR;
                }

                logger.logD("MechLogDriveCSV",String.format(",%f,%f,%f,%f,%f,%f,%d,%d,%d,%f,%f,%d,%d,%d,%f,%f,%f,%f", rt, deltaHeading, KhL, KhR, curPosLf, curPosL, l_f_motor.getCurrentPosition(),l_b_motor.getCurrentPosition(), newLeftBTarget, curPosRf, curPosR, r_f_motor.getCurrentPosition(), r_b_motor.getCurrentPosition(), newRightBTarget, Math.max(-1.0, Math.min(1.0, newSpeedL)), Math.max(-1.0, Math.min(1.0, newSpeedL)), Math.max(-1.0, Math.min(1.0, newSpeedR)), Math.max(-1.0, Math.min(1.0, newSpeedR))));

                if (DEBUG) {
                    myLOpMode.telemetry.addData("left", "up: %1.3f, dn: %1.3f, s: %1.3f, p: %5d, t: %5d, g: %5d", spdUpL, spdDnL, newSpeedL, (int) curPosL, newLeftBTarget, (int) toGoL);
                    myLOpMode.telemetry.addData("rght", "up: %1.3f, dn: %1.3f, s: %1.3f, p: %5d, t: %5d, g: %5d", spdUpR, spdDnR, newSpeedR, (int) curPosR, newRightBTarget, (int) toGoR);
                    myLOpMode.telemetry.update();
                }
            }

            // Stop all motion;
            l_f_motor.setPower(0);
            r_f_motor.setPower(0);
            r_b_motor.setPower(0);
            l_b_motor.setPower(0);

            if (DEBUG) {
                curPosL = l_b_motor.getCurrentPosition();
                curPosR = r_b_motor.getCurrentPosition();
                myLOpMode.telemetry.addData("left", "up: %1.3f, dn: %1.3f, s: %1.3f, p: %5d, t: %5d, g: %5d", spdUpL, spdDnL, newSpeedL, (int) curPosL, newLeftBTarget, (int) toGoL);
                myLOpMode.telemetry.addData("rght", "up: %1.3f, dn: %1.3f, s: %1.3f, p: %5d, t: %5d, g: %5d", spdUpR, spdDnR, newSpeedR, (int) curPosR, newRightBTarget, (int) toGoR);
                myLOpMode.telemetry.update();
            }

            // FIXME -- Lets just run in this mode all the time and not keep programming the motors
            // Reset run mode
            l_f_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r_f_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            l_b_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r_b_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        // FIXME
        //while (runtime.seconds() < fRT) { };
        if ((flickL != 0.0) || (flickR != 0.0)) {
            flicker.setPosition(0.0);
        }

        if (leftInches == rightInches) {
            // FIXME -- do we really need to burn 500ms every move here?
            //  Seems like the gyro is still moving a little here. We might consider switching to
            //  using something like a 'heading' instead of a angular delta. Then we don't really
            //  need to carry error forward, it's just known from the heading value.
            //  Then we also don't need the delay here either.
            myLOpMode.sleep(500);
            // FIXME - this is three calls to getHeading(), need to fix this up to use a variable.
            logger.logD("MechLog",String.format("straight done: final: %f, get: %f, tgt: %f, err: %f", curHeading, getHeading(), tgtHeading, getHeading() - tgtHeading ));
            return (normalizeAngle((getHeading() - tgtHeading)));
        } else {
            return(0.0);
        }
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  IMU gyro is used to help steer if we're going straight.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public double fastEncoderDrive2(double speed,
                                    double leftInches, double rightInches,
                                    double timeoutS, double P,
                                    double flickL, double flickR,
                                    double[] up, double[] down,
                                    DcMotor l_f_motor, DcMotor l_b_motor, DcMotor r_f_motor, DcMotor r_b_motor) {


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
        double fRT = 0.0;

        // Get the current Heading and update for the correction angle
        curHeading = tgtHeading = getHeading();
        logger.logD("MechLog",String.format("fastEncodeDrive2: tgt: %f, A: %f", tgtHeading, straightA));
        tgtHeading = normalizeAngle(tgtHeading - straightA) ;

        // now clear the correction angle so it's never used again
        straightA = 0.0;

        // Ensure that the opmode is still active
        if (myLOpMode.opModeIsActive()) {

            // Reset encoders
            l_f_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            r_f_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            l_b_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            r_b_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // FIXME -- Lets just run in this mode all the time and not keep programming the motors
            // We'll handle power/steering but let the PID try to maintain
            // a constant axle speed to overcome hub, motor, build variability.
            l_f_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r_f_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            l_b_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r_b_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // FIXME -- Lets just run in this mode all the time and not keep programming the motors
            // Braking is good if we have gyro correct
            l_f_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            r_f_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            l_b_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            r_b_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Compute desired position
            newLeftFTarget = (int)(leftInches * COUNTS_PER_INCH / STRAFE_DISTANCE_FACTOR);
            newRightFTarget = (int)(rightInches * COUNTS_PER_INCH / STRAFE_DISTANCE_FACTOR);
            newLeftBTarget = (int)(leftInches * COUNTS_PER_INCH / STRAFE_DISTANCE_FACTOR);
            newRightBTarget = (int)(rightInches * COUNTS_PER_INCH / STRAFE_DISTANCE_FACTOR);

            flickR *= COUNTS_PER_INCH / STRAFE_DISTANCE_FACTOR ;
            flickL *= COUNTS_PER_INCH / STRAFE_DISTANCE_FACTOR ;

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
            curPosL = l_b_motor.getCurrentPosition();
            curPosR = r_b_motor.getCurrentPosition();

            // FIXME -- Read the front encoders only for logging
            curPosLf = l_f_motor.getCurrentPosition();
            curPosRf = r_f_motor.getCurrentPosition();

            // Keep looping while we are still active, and there is time left, and we haven't reached the target
            while (myLOpMode.opModeIsActive() &&
                    ((rt=runtime.seconds()) < timeoutS) &&
                    ( !doneL || !doneR)) {

                // Check if we're done
                // This code implements a soft start and soft stop.
                // Compute the distance gone and how far to go.
                if (!doneL) {
                    spdPosL = curPosL = l_b_motor.getCurrentPosition();
                    toGoL = Math.min(toGoL, Math.max(0, Math.abs(newLeftBTarget - curPosL)));
                }
                if (!doneR) {
                    spdPosR = curPosR = r_b_motor.getCurrentPosition();
                    toGoR  = Math.min(toGoR, Math.max(0,Math.abs(newRightBTarget - curPosR)));
                }
                // Average them if driving straight to reduce twist/steer.
                if (leftInches == rightInches) {
                    spdPosL = spdPosR = (curPosL + curPosR)/2;
                    toGoL = toGoR = (toGoL + toGoR)/2;
                }

                // If there is a flick target and we aren't already flicking and its past our flick target
                if ((flickL != 0) && (fRT == 0) && (Math.abs(spdPosL) > Math.abs(flickL))) {
                    // flick
                    flicker.setPosition(1.0);
                    // Set the time to reset the flick
                    fRT = rt + 500;
                } else if ((flickR != 0) && (fRT == 0) && (Math.abs(spdPosR) > Math.abs(flickR))) {
                    // flick
                    flicker.setPosition(1.0);
                    // Set the time to reset the flick
                    fRT = rt + 500;
                } else if (fRT > rt) {
                    flicker.setPosition(0.0);
                    // We already flicked, no more flicking flicker.
                    fRT = -1.0;
                }

                if (!doneL) {
                    doneL = ((Math.abs(newLeftBTarget) - Math.abs(curPosL)) < CLOSE_ENOUGH);

                    // Compute speed on acceleration and deceleration legs
                    spdUpL = Math.max(Math.min(speedRampUp[Math.min((int) (Math.abs(spdPosL) / SOFT_D_UP), speedRampUp.length-1)], speed),minUp);
                    spdDnL = Math.min(speedRampDown[Math.min((int) (Math.abs(toGoL) / SOFT_D_DOWN2), speedRampDown.length-1)], speed);

                    // Use the minimum speed or 0
                    newSpeedL = doneL ? WAIT_SPEED : Math.min(spdUpL, spdDnL);

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
                    newSpeedR = doneR ? WAIT_SPEED : Math.min(spdUpR, spdDnR) ;

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

                newSpeedL *= (sKpL * KhL);
                newSpeedR *= (sKpR * KhR);

                if ((newSpeedL != actSpeedL) || (newSpeedR != actSpeedR)) {

                    l_f_motor.setPower(Math.max(-1.0, Math.min(1.0, newSpeedL)));

                    // FIXME, rightF seems to be either ahead or behind by different amounts.
                    //  Could this be due to shimming on the bearings and rubbing the outside
                    //  screws?
                    r_f_motor.setPower(Math.max(-1.0, Math.min(1.0, newSpeedR)));
                    //if (newRightFTarget > 0) {
                    //    r_f_motor.setPower(Math.max(-1.0, Math.min(1.0, newSpeedR)) * 0.92);
                    //} else {
                    //    r_f_motor.setPower(Math.max(-1.0, Math.min(1.0, newSpeedR)) * 0.97);
                    //}
                    r_b_motor.setPower(Math.max(-1.0, Math.min(1.0, newSpeedR)));
                    l_b_motor.setPower(Math.max(-1.0, Math.min(1.0, newSpeedL)));
                    actSpeedL = newSpeedL;
                    actSpeedR = newSpeedR;
                }

                logger.logD("MechLogDriveCSV",String.format(",%f,%f,%f,%f,%f,%f,%d,%d,%d,%f,%f,%d,%d,%d,%f,%f,%f,%f", rt, deltaHeading, KhL, KhR, curPosLf, curPosL, l_f_motor.getCurrentPosition(),l_b_motor.getCurrentPosition(), newLeftBTarget, curPosRf, curPosR, r_f_motor.getCurrentPosition(), r_b_motor.getCurrentPosition(), newRightBTarget, Math.max(-1.0, Math.min(1.0, newSpeedL)), Math.max(-1.0, Math.min(1.0, newSpeedL)), Math.max(-1.0, Math.min(1.0, newSpeedR)), Math.max(-1.0, Math.min(1.0, newSpeedR))));

                if (DEBUG) {
                    myLOpMode.telemetry.addData("left", "up: %1.3f, dn: %1.3f, s: %1.3f, p: %5d, t: %5d, g: %5d", spdUpL, spdDnL, newSpeedL, (int) curPosL, newLeftBTarget, (int) toGoL);
                    myLOpMode.telemetry.addData("rght", "up: %1.3f, dn: %1.3f, s: %1.3f, p: %5d, t: %5d, g: %5d", spdUpR, spdDnR, newSpeedR, (int) curPosR, newRightBTarget, (int) toGoR);
                    myLOpMode.telemetry.update();
                }
            }

            // Stop all motion;
            l_f_motor.setPower(0);
            r_f_motor.setPower(0);
            r_b_motor.setPower(0);
            l_b_motor.setPower(0);

            if (DEBUG) {
                curPosL = l_b_motor.getCurrentPosition();
                curPosR = r_b_motor.getCurrentPosition();
                myLOpMode.telemetry.addData("left", "up: %1.3f, dn: %1.3f, s: %1.3f, p: %5d, t: %5d, g: %5d", spdUpL, spdDnL, newSpeedL, (int) curPosL, newLeftBTarget, (int) toGoL);
                myLOpMode.telemetry.addData("rght", "up: %1.3f, dn: %1.3f, s: %1.3f, p: %5d, t: %5d, g: %5d", spdUpR, spdDnR, newSpeedR, (int) curPosR, newRightBTarget, (int) toGoR);
                myLOpMode.telemetry.update();
            }

            // FIXME -- Lets just run in this mode all the time and not keep programming the motors
            // Reset run mode
            l_f_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r_f_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            l_b_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r_b_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        // FIXME
        //while (runtime.seconds() < fRT) { };
        if ((flickL != 0.0) || (flickR != 0.0)) {
            flicker.setPosition(0.0);
        }

        if (leftInches == rightInches) {
            // FIXME -- do we really need to burn 500ms every move here?
            //  Seems like the gyro is still moving a little here. We might consider switching to
            //  using something like a 'heading' instead of a angular delta. Then we don't really
            //  need to carry error forward, it's just known from the heading value.
            //  Then we also don't need the delay here either.
            myLOpMode.sleep(500);
            // FIXME - this is three calls to getHeading(), need to fix this up to use a variable.
            logger.logD("MechLog",String.format("straight done: final: %f, get: %f, tgt: %f, err: %f", curHeading, getHeading(), tgtHeading, getHeading() - tgtHeading ));
            return (normalizeAngle((getHeading() - tgtHeading)));
        } else {
            return(0.0);
        }
    }

    public void setWobblePosition(int wobblePos, double power){
        wobble_motor.setTargetPosition(wobbleTargets[wobblePos]);
        wobble_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobble_motor.setPower(power);
    }


    public void MotorCal( DcMotor motorToTest, String name, double K ) {
        //double[] speedsToTest = { 0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0 };
        double[] speedsToTest = { 0.1,0.3,0.5,0.7,0.9 };
        double startTime, endTime;
        double startEncoder, endEncoder;
        ElapsedTime runtime = new ElapsedTime();

        motorToTest.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorToTest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorToTest.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorToTest.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorToTest.setPower(1.0);
        wait(1000); //wait half a second?
        motorToTest.setPower(0);
        wait(1000); //wait half a second?

        for(int i = 0; i<speedsToTest.length; i++){
            //runtime.reset();
            motorToTest.setPower(speedsToTest[i]*K);
            wait(1000); //wait half a second?
            startTime = runtime.seconds();
            startEncoder = motorToTest.getCurrentPosition();
            wait(5000); //wait 2 seconds?
            endTime = runtime.seconds();
            endEncoder = motorToTest.getCurrentPosition();

            logger.logD("MechLogDriveCSV_UsingEncoder",String.format(",%s,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f", name, speedsToTest[i], K, ((endEncoder-startEncoder)/(endTime-startTime)), startTime, endTime, startEncoder, endEncoder));
        }
        motorToTest.setPower(0);
    }

    //for waiting set time (testing purposes only)
    public static void wait(int ms)
    {
        try
        {
            Thread.sleep(ms);
        }
        catch(InterruptedException ex)
        {
            Thread.currentThread().interrupt();
        }
    }
}


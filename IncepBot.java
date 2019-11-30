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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
public class IncepBot
{
    /* Public OpMode members. */
    public DcMotor  leftFDrive   = null;
    public DcMotor  rightFDrive  = null;
    public DcMotor  leftBDrive   = null;
    public DcMotor  rightBDrive  = null;
    public Servo    foundation1    = null;
    public Servo    foundation2    = null;
    public Servo    claw   = null;

    private static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;         // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 10.0/11.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 3.54331 ;       // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159);
    private static final double     AXLE_LENGTH             = 13.25;          // Width of robot through the pivot point (center wheels)
    private static final double     INCHES_PER_DEGREE       = (AXLE_LENGTH * 3.14159) / 360.0;
    private static final double     SOFT_D_UP               = 50.0;
    private static final double     SOFT_D_DOWN             = 70.0;
    private static final double     KpL                     = 1.0;
    private static final double     KpR                     = 67.5/70.0;
    private static final double     PIVOT_FACTOR            = 2.05;
    private static final double     CLOSE_ENOUGH            = 15.0;
    private static final double     RIGHT_ARC_COEFFICENT    = 1.00;
    private static final double     LEFT_ARC_COEFFICENT     = 1.25;
    static final double             RIGHT                   = 1;
    static final double             LEFT                    = 0;
    BNO055IMU               imu;
    private Orientation             angles;
    ColorSensor             colorSensor;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    private static LinearOpMode myLOpMode;

    /* Constructor */
    public IncepBot(){
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

        colorSensor = hwMap.colorSensor.get("color");

        myLOpMode.telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        composeTelemetry();
        myLOpMode.telemetry.update();
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
                .addData("argb", new Func<String>() {
                    @Override public String value() {
                        return formatColor(colorSensor.argb());
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

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFDrive  = hwMap.get(DcMotor.class, "left_front");
        rightFDrive = hwMap.get(DcMotor.class, "right_front");
        leftBDrive  = hwMap.get(DcMotor.class, "left_back");
        rightBDrive = hwMap.get(DcMotor.class, "right_back");

        //leftArm    = hwMap.get(DcMotor.class, "left_arm");
        leftFDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightFDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftBDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightBDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftFDrive.setPower(0);
        rightFDrive.setPower(0);
        leftBDrive.setPower(0);
        rightBDrive.setPower(0);

        //leftArm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        foundation1  = hwMap.get(Servo.class, "foundation1");
        foundation2  = hwMap.get(Servo.class, "foundation2");
        claw = hwMap.get(Servo.class, "claw");
        foundation1.setPosition(0);
        foundation2.setPosition(0);
        claw.setPosition(0);
    }

    public void grabFoundation() {
        foundation1.setPosition(0);
        foundation2.setPosition(0);
        myLOpMode.sleep(1000);
    }

    public void releaseFoundation() {
        foundation1.setPosition(1);
        foundation2.setPosition(1);
        myLOpMode.sleep(750);
    }

    public void grabBlock() {
        claw.setPosition(1);
        myLOpMode.sleep(400);
    }

    public void dropBlock() {
        claw.setPosition(0);
        myLOpMode.sleep(250);
    }

    public void encoderStraight(double speed, double distance, double timeoutS) {
        encoderDrive( speed, distance, distance, timeoutS );
    }

    public void encoderRotate(double speed, double degrees, double timeoutS) {
        encoderDrive( speed, degrees * INCHES_PER_DEGREE, -degrees * INCHES_PER_DEGREE, timeoutS );
    }

    public void encoderPivot(double speed, double degrees, double timeoutS) {
        if (degrees > 0) {
            encoderDrive(speed, degrees * PIVOT_FACTOR * INCHES_PER_DEGREE, 0, timeoutS);
        } else {
            // We use -degrees, since to cancel out the negative degrees (we want positive right wheel rotation
            encoderDrive(speed, 0, -degrees * PIVOT_FACTOR * INCHES_PER_DEGREE, timeoutS);
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
            //KpO = KpR ;
            //KpI = KpL ;
            KpO = 1.0 ;
            KpI = 1.0 ;
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

                myLOpMode.telemetry.addData("Path3",  "up: %1.3f, dn: %1.3f, s: %1.3f, p: %5d, t: %5d", spdUp, spdDn, actSpeed, (int)curPos, (int)toGo);
                myLOpMode.telemetry.update();
                //sleep(10);   // optional pause after each move
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

                // Display it for the driver.
                //telemetry.addData("Path1",  "Running to LF:%7d, RF: %7d, LB: %7d, RB: %7d", newLeftFTarget,  newRightFTarget, newLeftBTarget,  newRightBTarget);
                //telemetry.addData("Path2",  "Running at LF:%7d, RF: %7d, LB: %7d, RB: %7d",
                //                            leftFDrive.getCurrentPosition(),
                //                            rightFDrive.getCurrentPosition(),
                //                            leftBDrive.getCurrentPosition(),
                //                            rightBDrive.getCurrentPosition());
                myLOpMode.telemetry.addData("Path3",  "up: %1.3f, dn: %1.3f, s: %1.3f, p: %5d, t: %5d", spdUp, spdDn, actSpeed, (int)curPos, (int)toGo);
                myLOpMode.telemetry.update();
                //sleep(10);   // optional pause after each move
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
    }
}


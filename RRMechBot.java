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

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import Inception.FreightFrenzy.drive.SampleMecanumDrive;


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
public class RRMechBot {
    /* Public OpMode members. */
    public DcMotor intake_motor;
    public Servo slide=null,bucket=null;
    public Servo duckL=null,duckR=null;
    public SampleMecanumDrive drive = null;

    public BotLog logger = new BotLog();

    /* local OpMode members. */
    HardwareMap hardwareMap = null;
    private static LinearOpMode myLOpMode;

    // SW PID with some limits
    //MiniPID pid = new MiniPID(P,I,D,F);
    //private NanoClock PIDClock = NanoClock.system();
    //private double shooterRPM = 0.0 ;
    //public double PIDTime = 0.025;
    //private double PIDStartTime = PIDClock.seconds();
    //private double nextPID = PIDStartTime;
    //private int PIDAvgSize = 10;
    //private int PIDAvgCount = 0;
    //private double lastPIDTime = PIDTime;
    //private double prevPIDTime1 = PIDTime;
    //private double output1 = 0;
    public VoltageSensor Vsense;

    /* Constructor */
    public RRMechBot() {
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

        // TODO: Unique prep for autonomous
        // Do things here at the beginning of autonomous that might be needed to setup
        // Take up slack on a spindle, run to a 'stop position' to index '0'
        // Set the starting location of a servo
        /*
        // Run the wobble arm for a teeny little bit at low power just to take out and slack...
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setPower(-0.033);
        myLOpMode.sleep(500);
        motor1.setPower(0.0);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setTargetPosition(0);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        servo1.setPosition(1.0);
        servo2.setPosition(0.0);
        */
    }

    /* Initialize standard Hardware interfaces */
    public void acquireHW(HardwareMap ahwMap) {
        acquireHW(ahwMap, 0.5);
    }

        /* Initialize standard Hardware interfaces */
    public void acquireHW(HardwareMap ahwMap, double driveTimeout) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        drive = new SampleMecanumDrive(hardwareMap, 0.5);

        // TODO: Finish getting all the device mapped
        intake_motor = hardwareMap.dcMotor.get("intake_motor");
        slide = hardwareMap.servo.get("slide");
        //bucket = hardwareMap.servo.get("bucket");
        duckL = hardwareMap.servo.get("duck_left");
        //duckR = hardwareMap.servo.get("duckR");

        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            if (Vsense == null) {
                double voltage = sensor.getVoltage();
                if (voltage > 0) {
                    Vsense = sensor;
                }
            }
        }

        //pid.reset();
        //pid.setOutputLimits(0.0,MAX_SHOOTER_PWR);
        //pid.setMaxIOutput(0.05);
        //// Voltage adjust F
        //vF = (F*(12.8/Vsense.getVoltage()));
        //pid.setPID(P,I,D,vF);
    }

    public void init(HardwareMap ahwMap) {
        init( ahwMap, 0.5);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, double driveTimeout) {

        acquireHW(ahwMap, driveTimeout);

        // Common init for for auto and non-auto cases
        intake_motor.setPower(0.0);
        intake_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void intakeStop(){
        intake_motor.setPower(0.0);
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void intakeElement(double power){
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake_motor.setPower(power);
    }

    public void intakeEject(double power){
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake_motor.setPower(power);
    }

    public double gyroRotatePrecise(double speed, double degrees, double timeoutS) {

        // This profile is for rotate
        double[] speedRampUpR = {0.225, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0};
        double[] speedRampDownR = {0.20, 0.225, 0.25, 0.275, 0.30, 0.325, 0.35, 0.375, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0};

        return(gyroRotate(degrees, speed, timeoutS, speedRampUpR, speedRampDownR, 0.25));
    }

    /*
     *  Method to perfmorm an arbitrary turn with differential wheel power and gyro feedback
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired heading
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public double gyroRotate(double degrees,
                           double maxPower,
                           double timeoutS, double[] up, double[] down,
                           double precision ) {
        double toGo;
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
        double KpL = 1.0;
        double KpR = 1.0;

        // Ensure that the opmode is still active
        if (myLOpMode.opModeIsActive() && !myLOpMode.isStopRequested()) {

            // Get the current Heading
            startHeading = curHeading = Math.toDegrees(drive.getRawExternalHeading());
            // Where we are headed
            tgtHeading = AngleUnit.normalizeDegrees(curHeading + degrees) ;
            // Initialize toGo
            toGo = Math.abs(degrees);

            logger.logD("RRMechLog",String.format("gyroTurn: start: %f, tgt: %f", startHeading, tgtHeading));

            if (false) {
                myLOpMode.telemetry.addData("gyro", "d: %3.1f, c: %3.1f, s: %3.1f, t: %3.1f, e: %3.1f",degrees,curHeading,startHeading,tgtHeading,AngleUnit.normalizeDegrees(curHeading - tgtHeading));
                myLOpMode.telemetry.update();
                //myLOpMode.sleep(3000);
            }

            // Determine rotation direction
            if (degrees > 0){
                KsL = -1.0;
                KsR = 1.0;
            } else {
                KsL = 1.0;
                KsR = -1.0;
            }

            if (false) {
                myLOpMode.telemetry.addData("gyro", "KL: %.3f, KR: %.3f, max: %.2f",KsL, KsR,maxPower);
                myLOpMode.telemetry.update();
            }

            // reset the timeout time and start motion at the minimum speed.
            runtime.reset();

            // Setting the starting speed is now handled inside the loop below on the first iteration.
            delta = AngleUnit.normalizeDegrees(curHeading - tgtHeading);
            if ( delta != 0) {
                sign = delta/Math.abs(delta);
            }

            // Initialize variables before the loop
            // Use done to jump out of loop when you get close enough to the end
            // Keep looping while we are still active, and there is time left, and we haven't reached the target
            while (myLOpMode.opModeIsActive() &&
                    !myLOpMode.isStopRequested() &&
                    ((rt=runtime.seconds()) < timeoutS)) {

                // Check if we're done
                // This code implements a soft start and soft stop.
                curHeading = Math.toDegrees(drive.getRawExternalHeading());

                // Look to see if our delta is really close or if we over rotated already (sign changed on delta)
                delta = AngleUnit.normalizeDegrees(curHeading - tgtHeading);
                if (((Math.abs(delta)) < precision) || (delta/Math.abs(delta) != sign)) {
                    break;
                }

                // How much farther?
                toGo = Math.min(toGo, Math.max(0, Math.abs(AngleUnit.normalizeDegrees(tgtHeading - curHeading))));

                // Compute speed on acceleration and deceleration legs
                spdUp = Math.min(speedRampUp[Math.min((int) (Math.abs(AngleUnit.normalizeDegrees(startHeading - curHeading)) / 5.0), speedRampUp.length - 1)], maxPower);
                spdDn = Math.min(speedRampDown[Math.min((int) (Math.abs(toGo) / 5.0), speedRampDown.length - 1)], maxPower);

                // Scale the final speed against the input power.
                newSpeed = Math.min(spdDn, spdUp);

                // Only update the motors if we really need too
                if (newSpeed != actSpeed ) {

                    // Record the new base speed
                    actSpeed = newSpeed;

                    drive.setMotorPowers(Range.clip(newSpeed * KpL * KsL,-1.0,1.0),
                            Range.clip(newSpeed * KpL * KsL,-1.0,1.0),
                            Range.clip(newSpeed * KpR * KsR,-1.0,1.0),
                            Range.clip(newSpeed * KpR * KsR,-1.0,1.0));
                }

                if (false) {
                    myLOpMode.telemetry.addData("left", "s: %1.3f",newSpeed * KpL * KsL);
                    myLOpMode.telemetry.addData("right", "s: %1.3f",newSpeed * KpR * KsR);
                    myLOpMode.telemetry.addData("gyro", "deg: %1.3f, curr: %1.3f, start: %1.3f, tgt: %1.3f",degrees,curHeading,startHeading,tgtHeading);
                    myLOpMode.telemetry.update();
                }
                // FIXME -- Read the front encoders only for logging
            }

            // Stop all motion;
            drive.setMotorPowers(0,0,0,0);

            if (false) {
                myLOpMode.telemetry.addData("gyro", "d: %3.1f, c: %3.1f, s: %3.1f, t: %3.1f, e: %3.1f", degrees, curHeading, startHeading, tgtHeading, AngleUnit.normalizeDegrees(curHeading - tgtHeading));
                myLOpMode.telemetry.update();
                myLOpMode.sleep(1000);
            }

            double tmpHeading = Math.toDegrees(drive.getRawExternalHeading());
            logger.logD("RRMechLog",String.format("turn done: final: %f, get: %f, tgt: %f, err: %f", curHeading, tmpHeading, tgtHeading, tmpHeading - tgtHeading ));
            return(AngleUnit.normalizeDegrees(tmpHeading - tgtHeading));
        }
        return(0.0);
    }
}


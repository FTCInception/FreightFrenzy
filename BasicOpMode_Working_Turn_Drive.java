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

import java.util.Locale;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

/**
 * Made by DaSchelling for Testing programs for team 12533...
 * Here is the mapping for the motors:
 *      Left front motor -> Slot 0 | Zip-Tie Color: Blue
 *      Right front motor -> Slot 1 | Zip-Tie Color: Orange
 *      Left back motor -> Slot 2 | Zip-Tie Color: Purple
 *      Right back motor -> Slot 3 | Zip-Tie Color: Green
 */

@TeleOp(name="Double Stick Turn Drive", group="Linear Opmode")
public class BasicOpMode_Working_Turn_Drive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private static ExpansionHubEx expansionHub;
    private static ExpansionHubMotor  l_f_motor, l_b_motor, r_f_motor, r_b_motor;
    private static Servo foundation1, foundation2, claw;
    private BNO055IMU imu;
    //private ColorSensor colorSensor;
    private BotLog logger = new BotLog();

    // Declare other variables
    private double speedModifier = 0.5;
    private int dirInvert = 1;
    private boolean enableCSVLogging = false;
    private Orientation angles;

    private BNO055IMU initIMU(String imuName) {

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


    @Override
    public void runOpMode() {
        RevBulkData bulkData;
        double lFounSet[] = {1.0, 0.0};
        double rFounSet[] = {1.0, 0.0};
        double clawSet[] = {1.0, 0.0};
        boolean lBump1Prev=false, rBump1Prev=false;
        int lFounPos=0, rFounPos=0, clawPos=0;
        double rt = 0.0, nextLog = 0.0;

        if (enableCSVLogging) {
            // Enable debug logging
            logger.LOGLEVEL |= logger.LOGDEBUG;
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");

        l_f_motor = (ExpansionHubMotor) hardwareMap.dcMotor.get("left_front");
        l_b_motor = (ExpansionHubMotor) hardwareMap.dcMotor.get("left_back");
        r_f_motor = (ExpansionHubMotor) hardwareMap.dcMotor.get("right_front");
        r_b_motor = (ExpansionHubMotor) hardwareMap.dcMotor.get("right_back");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        l_f_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        l_b_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        r_f_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        r_b_motor.setDirection(DcMotorSimple.Direction.FORWARD);

        foundation1 = hardwareMap.servo.get("foundation1");
        foundation2 = hardwareMap.servo.get("foundation2");
        claw = hardwareMap.servo.get("claw");

        imu = initIMU("imu");

        composeTelemetry();
        telemetry.log().add("Waiting for start...");

        // Wait until we're told to go
        while (!isStarted()) {
            telemetry.update();
            idle();
        }

        // Wait for the game to start (driver presses PLAY)
        //waitForStart();
        runtime.reset();

        if (enableCSVLogging) {
            // Lay down a header for our logging
            logger.logD("TeleopTurnCSV", String.format(",rt,heading,lEnc,rEnc,lPwr,rPwr"));
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double l_f_motor_power;
            double l_b_motor_power;
            double r_f_motor_power;
            double r_b_motor_power;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = Math.cbrt(gamepad1.left_stick_y);
            double turn  =  Math.cbrt(gamepad1.right_stick_x);

            //speed control
            if (gamepad1.x) {
                speedModifier = 1;
            }
            else if (gamepad1.y) {
                speedModifier = 0.75;
            }
            else if (gamepad1.b) {
                speedModifier = .5;
            }
            else if (gamepad1.a) {
                speedModifier = 0.25;
            }

            if (gamepad1.dpad_up) {
                dirInvert = 1;
            }
            else if (gamepad1.dpad_down) {
                dirInvert = -1;
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
                if (!rBump1Prev){
                    clawPos++; clawPos %= 2;
                    claw.setPosition(clawSet[clawPos]);
                    rBump1Prev = true;
                }
            } else {
                rBump1Prev = false;
            }

            // Create power settings
            l_f_motor_power   = Range.clip(((drive* dirInvert) - turn) * speedModifier, -1.0, 1.0) ;
            l_b_motor_power   = Range.clip(((drive* dirInvert) - turn) * speedModifier, -1.0, 1.0) ;
            r_f_motor_power   = Range.clip(((drive* dirInvert) + turn) * speedModifier, -1.0, 1.0) ;
            r_b_motor_power   = Range.clip(((drive* dirInvert) + turn) * speedModifier, -1.0, 1.0) ;

            // Send calculated power to wheels
            l_f_motor.setPower(l_f_motor_power);
            l_b_motor.setPower(l_b_motor_power);
            r_f_motor.setPower(r_f_motor_power);
            r_b_motor.setPower(r_b_motor_power);

            // Update the logger 10 times/second max
            if (enableCSVLogging) {
                rt = runtime.seconds();
                if (rt > nextLog) {
                    bulkData = expansionHub.getBulkInputData();
                    logger.logD("TeleopTurnCSV", String.format(",%f,%f,%d,%d,%f,%f", rt, getHeading(), bulkData.getMotorCurrentPosition(l_f_motor), bulkData.getMotorCurrentPosition(r_f_motor), l_f_motor_power, r_f_motor_power));
                    nextLog = rt + 0.1;
                }
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", l_f_motor_power, l_b_motor_power, r_f_motor_power, r_b_motor_power);
            telemetry.addData("Speed Modifier", "Speed Modifier:" , speedModifier);
            telemetry.update();
        }
        //colorSensor.enableLed(false);
    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        });
        
        telemetry.addLine()
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
    }

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
}

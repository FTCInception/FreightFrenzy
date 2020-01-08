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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import java.util.Locale;


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
    private static DcMotor l_f_motor, l_b_motor, r_f_motor, r_b_motor;
    private static DcMotor l_in_motor, r_in_motor;
    private static DcMotor l_out_motor, r_out_motor;
    private static Servo foundation1, foundation2, claw;
    private static Servo back_grabber, front_grabber, slide;

    BNO055IMU imu,imu2;
    Orientation angles,angles2;

    double theta, r_speed, new_x, new_y;

    double MAX_OUTTAKE_POWER = 0.5;
    double MAX_INTAKE_POWER = 0.9;

    // Declare other variables
    double speedModifier = 0.5;
    double adjustAngle = 0.0;

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
        double fGrabSet[] = {1.0, 0.53};
        double bGrabSet[] = {0.30, 0.0};
        boolean lBump2Prev=false, rBump2Prev=false;
        int fGrabPos=0, bGrabPos=0;
        double lFounSet[] = {0.0, 0.9};
        double rFounSet[] = {1.0, 0.1};
        double clawSet[] = {0.0, 1.0};
        boolean lBump1Prev=false, rBump1Prev=false;
        int lFounPos=0, rFounPos=0, clawPos=0;


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        l_f_motor = hardwareMap.dcMotor.get("left_front");
        l_b_motor = hardwareMap.dcMotor.get("left_back");
        r_f_motor = hardwareMap.dcMotor.get("right_front");
        r_b_motor = hardwareMap.dcMotor.get("right_back");
        l_in_motor = hardwareMap.dcMotor.get("right_in");
        r_in_motor = hardwareMap.dcMotor.get("left_in");

        foundation1 = hardwareMap.servo.get("foundation1");
        foundation2 = hardwareMap.servo.get("foundation2");
        claw = hardwareMap.servo.get("claw");

        back_grabber = hardwareMap.servo.get("grabber1");
        front_grabber = hardwareMap.servo.get("grabber2");
        // FIXME -- controller 2
        // slide = hardwareMap.servo.get("slide");
        l_out_motor = hardwareMap.dcMotor.get("right_out");
        r_out_motor = hardwareMap.dcMotor.get("left_out");


        imu = initIMU("imu");
        imu2 = initIMU("imu 1");

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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double forward, strafe, rotate, degrees;
            double in_pwr, out_pwr;
            double l_f_motor_power;
            double l_b_motor_power;
            double r_f_motor_power;
            double r_b_motor_power;

            // Begin controller 1 (driver)
            // Single button toggle for grabbers
            if (gamepad2.left_bumper) {
                if(!lBump2Prev) {
                    fGrabPos++; fGrabPos %= 2;
                    front_grabber.setPosition(fGrabSet[fGrabPos]);
                    lBump2Prev = true;
                }
            } else {
                lBump2Prev = false;
            }

            if (gamepad2.right_bumper) {
                if (!rBump2Prev){
                    bGrabPos++; bGrabPos %= 2;
                    back_grabber.setPosition(bGrabSet[bGrabPos]);
                    rBump2Prev = true;
                }
            } else {
                rBump2Prev = false;
            }

            // slide.setPosition( gamepad2.a ? 1.0 : 0.0 );
            out_pwr = gamepad2.left_trigger * MAX_OUTTAKE_POWER;
            out_pwr -= gamepad2.right_trigger * MAX_OUTTAKE_POWER;
            l_out_motor.setPower(out_pwr);
            r_out_motor.setPower(out_pwr);

            // Begin controller 1 (driver)
            //speed control
            strafe = gamepad1.left_stick_x;
            forward = -gamepad1.left_stick_y;
            rotate = gamepad1.right_stick_x;

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //angles2   = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            degrees = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

            if (gamepad1.dpad_up) {
                adjustAngle = degrees;
            }
            degrees = AngleUnit.DEGREES.normalize(degrees - adjustAngle);

            CarToPol(strafe,forward);
            theta -= degrees;
            PolToCar(r_speed);
            strafe = new_x;
            forward = new_y;

            l_f_motor_power   = Range.clip((forward + strafe + rotate) * speedModifier, -1.0, 1.0) ;
            l_b_motor_power   = Range.clip((forward - strafe + rotate) * speedModifier, -1.0, 1.0) ;
            r_f_motor_power   = Range.clip((forward - strafe - rotate) * speedModifier, -1.0, 1.0) ;
            r_b_motor_power   = Range.clip((forward + strafe - rotate) * speedModifier, -1.0, 1.0) ;

            // Send calculated power to wheels
            l_f_motor.setPower(l_f_motor_power);
            l_b_motor.setPower(l_b_motor_power);
            r_f_motor.setPower(r_f_motor_power);
            r_b_motor.setPower(r_b_motor_power);

            // Additional driver controls
            //speed control
            in_pwr = gamepad1.right_trigger * MAX_INTAKE_POWER;
            in_pwr -= gamepad1.left_trigger * MAX_INTAKE_POWER;
            l_in_motor.setPower(in_pwr);
            r_in_motor.setPower(in_pwr);

            if (gamepad1.y) {
                speedModifier = 1;
            }
            else if (gamepad1.b) {
                speedModifier = 0.75;
            }
            else if (gamepad1.a) {
                speedModifier = .5;
            }
            else if (gamepad1.x) {
                speedModifier = 0.25;
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

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Polar", "Speed (%.2f), theta (%.2f)", forward, theta);
            telemetry.addData("Gyro", degrees);
            telemetry.update();
        }
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



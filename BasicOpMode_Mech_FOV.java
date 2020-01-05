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
    private static Servo foundation1, foundation2;
    private static Servo grabber1, grabber2, slide;

    BNO055IMU imu,imu2;
    Orientation angles,angles2;

    double theta, r_speed, new_x, new_y;

    double MAX_INTAKE_POWER = 0.8;
    double MAX_DRIVE_POWER = 0.7;

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

        // FIXME -- controller 2
        // grabber1 = hardwareMap.servo.get("grabber1");
        // grabber2 = hardwareMap.servo.get("grabber2");
        // slide = hardwareMap.servo.get("slide");
        // l_out_motor = hardwareMap.dcMotor.get("right_out");
        // r_out_motor = hardwareMap.dcMotor.get("left_out");


        imu = initIMU("imu");
        imu2 = initIMU("imu 1");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        l_f_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        l_b_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        l_in_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        //l_out_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        r_f_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        r_b_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        r_in_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        //r_out_motor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double forward, strafe, rotate, degrees;
            double in_pwr, out_pwr;

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angles2   = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            // FIXME -- controller 2
            // out_pwr = gamepad2.left_trigger * MAX_INTAKE_POWER;
            // out_pwr -= gamepad2.right_trigger * MAX_INTAKE_POWER;
            // slide.setPosition( gamepad2.a ? 1.0 : 0.0 );
            // grabber1.setPosition( gamepad2.left_bumper ? 1.0 : 0.0 );
            // grabber2.setPosition( gamepad2.right_bumper ? 1.0 : 0.0 );
            // l_out_motor.setPower(out_pwr);
            // r_out_motor.setPower(out_pwr);

            //speed control
            strafe = gamepad1.left_stick_x;
            forward = -gamepad1.left_stick_y;
            rotate = gamepad1.right_stick_x;
            degrees = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

            CarToPol(strafe,forward);
            theta -= degrees;
            PolToCar(r_speed);
            strafe = new_x;
            forward = new_y;

            //Quarter speed
            strafe *= MAX_DRIVE_POWER;
            forward *= MAX_DRIVE_POWER;
            rotate *= MAX_DRIVE_POWER;

            // Send calculated power to wheels
            l_f_motor.setPower(forward + strafe + rotate);
            l_b_motor.setPower(forward - strafe + rotate);
            r_f_motor.setPower(forward - strafe - rotate);
            r_b_motor.setPower(forward + strafe - rotate);

            // Additional driver controls
            in_pwr = gamepad1.left_trigger * MAX_INTAKE_POWER;
            in_pwr -= gamepad1.right_trigger * MAX_INTAKE_POWER;
            l_in_motor.setPower(in_pwr);
            r_in_motor.setPower(in_pwr);

            foundation1.setPosition( gamepad1.left_bumper ? 1.0 : 0.0 );
            foundation2.setPosition( gamepad1.right_bumper ? 0.0 : 1.0 );

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


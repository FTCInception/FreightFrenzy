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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;



/**
 * Made by DaSchelling for Testing programs for team 12533...
 * Here is the mapping for the motors:
 *      Left front motor -> Slot 0 | Zip-Tie Color: Blue
 *      Right front motor -> Slot 1 | Zip-Tie Color: Orange
 *      Left back motor -> Slot 2 | Zip-Tie Color: Purple
 *      Right back motor -> Slot 3 | Zip-Tie Color: Green
 */

@TeleOp(name="Tank Drive", group="Linear Opmode")
public class BasicOpMode_Working_Tank_Drive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private static DcMotor l_f_motor, l_b_motor, r_f_motor, r_b_motor;
    private static Servo foundation, claw;

    // Declare other variables
    double speedModifier = 0.75;

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

        foundation = hardwareMap.servo.get("foundation");
        claw = hardwareMap.servo.get("claw");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        l_f_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        l_b_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        r_f_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        r_b_motor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double l_motor_power;
            double r_motor_power;

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

            //foundation servo control
            if(gamepad1.right_trigger > .5){
                foundation.setPosition(1.0);
            }
            else if(gamepad1.right_trigger < .5){
                foundation.setPosition(0);
            }

            //claw servo control
            if(gamepad1.left_trigger > .5){
                claw.setPosition(1.0);
            }
            else if(gamepad1.left_trigger < .5){
                claw.setPosition(0);
            }

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            l_motor_power = Math.cbrt(gamepad1.left_stick_y) * speedModifier;
            r_motor_power = Math.cbrt(gamepad1.right_stick_y) * speedModifier;

            // If the values are close to each other, we're probably trying to drive straight, help it out.
            if (Math.abs(1-(l_motor_power/r_motor_power)) < .2) {
                // Take the average of the power
                l_motor_power = r_motor_power = ((l_motor_power + r_motor_power) / 2);
            }

            // Send calculated power to wheels
            l_f_motor.setPower(l_motor_power);
            l_b_motor.setPower(l_motor_power);
            r_f_motor.setPower(r_motor_power);
            r_b_motor.setPower(r_motor_power);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", l_motor_power, r_motor_power);
            telemetry.addData("Speed Modifier", "Speed Modifier:" , speedModifier);
            telemetry.update();
        }
    }
}

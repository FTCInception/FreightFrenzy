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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


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
    public DcMotorEx shoot1_motor, shoot2_motor;
    public DcMotorEx wobble_motor;
    public Servo claw=null,flicker=null;
    public SampleMecanumDrive drive = null;

    public int WOBBLE_START = 0;
    public int WOBBLE_PICKUP = 1;
    public int WOBBLE_CARRY = 2;
    public int WOBBLE_DROP = 3;

    //final double WOBBLE_TICKS_PER_DEGREE = 5264.0/360.0; // 30 RPM 6mm d-shaft (5202 series)
    //final double WOBBLE_TICKS_PER_DEGREE = 2786.0/360.0; // 60 RPM 6mm d-shaft (5202 series)
    final double startingAngle = 15.0;
    final double WOBBLE_TICKS_PER_DEGREE = 3892.0/360.0; // 43 RPM 8mm REX (5203 series)
    final int wobbleTargets[] = {(int)(5*WOBBLE_TICKS_PER_DEGREE),(int)((235-startingAngle)*WOBBLE_TICKS_PER_DEGREE), (int)((180-startingAngle)*WOBBLE_TICKS_PER_DEGREE),(int)((225-startingAngle)*WOBBLE_TICKS_PER_DEGREE)};

    public BotLog logger = new BotLog();

    /* local OpMode members. */
    HardwareMap hardwareMap = null;
    private static LinearOpMode myLOpMode;

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

        // Run the wobble arm for a teeny little bit at low power just to take out and slack...
        wobble_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobble_motor.setPower(-0.033);
        myLOpMode.sleep(500);
        wobble_motor.setPower(0.0);
        wobble_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobble_motor.setTargetPosition(0);
        wobble_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;

        drive = new SampleMecanumDrive(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        intake_motor = hardwareMap.dcMotor.get("intake");
        intake_motor.setPower(0.0);
        intake_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //intake_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Zero out the wobble motor in the auto init
        wobble_motor = hardwareMap.get(DcMotorEx.class,"wobble");
        wobble_motor.setPower(0.0);
        //wobble_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        wobble_motor.setDirection(DcMotorSimple.Direction.FORWARD);
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

    public void setWobblePosition(int wobblePos, double power){
        wobble_motor.setTargetPosition(wobbleTargets[wobblePos]);
        wobble_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobble_motor.setPower(power);
    }

    public void intakeStop(){
        intake_motor.setPower(0.0);
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void intakeEjectWobble(double power){
        intake_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake_motor.setPower(power);
    }

    public void intakePickupRing(double power){
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake_motor.setPower(power);
    }
}


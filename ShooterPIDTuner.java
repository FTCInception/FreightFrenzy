package Inception.UltimateGoal;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

/*
 * This routine is designed to tune the PID coefficients used by the REV Expansion Hubs for closed-
 * loop velocity control. Although it may seem unnecessary, tuning these coefficients is just as
 * important as the positional parameters. Like the other manual tuning routines, this op mode
 * relies heavily upon the dashboard. To access the dashboard, connect your computer to the RC's
 * WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're using the RC
 * phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once you've successfully
 * connected, start the program, and your robot will begin moving forward and backward according to
 * a motion profile. Your job is to graph the velocity errors over time and adjust the PID
 * coefficients (note: the tuning variable will not appear until the op mode finishes initializing).
 * Once you've found a satisfactory set of gains, add them to the DriveConstants.java file under the
 * MOTOR_VELO_PID field.
 *
 * Recommended tuning process:
 *
 * 1. Increase kP until any phase lag is eliminated. Concurrently increase kD as necessary to
 *    mitigate oscillations.
 * 2. Add kI (or adjust kF) until the steady state/constant velocity plateaus are reached.
 * 3. Back off kP and kD a little until the response is less oscillatory (but without lag).
 *
 * Pressing X (on the Xbox and Logitech F310 gamepads, square on the PS4 Dualshock gamepad) will
 * pause the tuning process and enter driver override, allowing the user to reset the position of
 * the bot in the event that it drifts off the path.
 * Pressing A (on the Xbox and Logitech F310 gamepads, X on the PS4 Dualshock gamepad) will cede
 * control back to the tuning process.
 */
@Config
@TeleOp(name="ShooterPIDTuner", group="Linear Opmode")
@Disabled
public class ShooterPIDTuner extends LinearOpMode {
    //public static double P = 45.0;
    //public static double I = 0.0;
    //public static double D = 30.0;
    //public static double F = 13.0;
    public static double P = 0.0005;
    public static double I = 0.0000003;
    public static double D = 0.00007;
    public static double F = 1.0/7250.0;  // @ 13V resting (12.8V under load)

    public static double RPM = 3550;
    public static double RPMPwrShot = 3225;
    public static double Pwr = 0.478;
    public static double PIDTime = 0.05;
    public static double PIDTimeMax = 0.25;
    public static double ExVal = 0.02;

    // Declare OpMode members.
    private static DcMotorEx shoot1_motor, shoot2_motor;
    private static VoltageSensor Vsense = null;
    private static Servo flicker;
    private static double ratio=1.5;

    private BotLog logger = new BotLog();
    private boolean enableCSVLogging = true;

    private int avgSize = 10;
    private int avgCount = 0;
    private double[] Pos1 = new double[avgSize];
    private double[] Pos2 = new double[avgSize];
    private double[] Time1 = new double[avgSize];
    private double[] Time2 = new double[avgSize];

    @Override
    public void runOpMode() {
        //RevBulkData bulkData1, bulkData2;
        boolean[] dUpPrev = new boolean[]{false, false};

        double[] shooterSet = {0.0, Pwr};
        int shooterIdx=0;

        final double FLICKER_SHOOT = 0.7, FLICKER_WAIT=0.0;
        double[] flickerSet = {FLICKER_WAIT, FLICKER_SHOOT};
        int flickerIdx=0;
        double flickerRelease=0.0, flickerRearm=0.0;

        double maxLag, prt, rt, now, iter=0, td1=0, td2=0, td3=0;

        double tps1=0.0, tps2=0.0;
        double shoot1Pos, prevShoot1Pos=0.0;
        double shoot2Pos, prevShoot2Pos=0.0;
        double nextPID=0.0;

        // SW PID with some limits
        MiniPID pid = new MiniPID(P,I,D,F);
        pid.setOutputLimits(0.0,0.90);
        pid.setMaxIOutput(0.05);

        if (enableCSVLogging) {
            // Enable debug logging
            logger.LOGLEVEL |= logger.LOGDEBUG;
        }

        //logger.LOGLEVEL |= logger.LOGDEBUG;

        shoot1_motor = hardwareMap.get(DcMotorEx.class,"shoot1");
        shoot2_motor = hardwareMap.get(DcMotorEx.class,"shoot2");
        shoot1_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot2_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot1_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoot2_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoot1_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot2_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //shoot1_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //shoot2_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //shoot1_motor.setVelocityPIDFCoefficients(P, I, D, F);
        //shoot2_motor.setVelocityPIDFCoefficients(P, I, D, F);
        shoot1_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot2_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // These PIDF values seem pretty good.
        //PIDFCoefficients pidFNew = new PIDFCoefficients(125.0, 2.5, 5.0, 4.0);
        // Raising 'F' increases overshoot a lot
        // Raising 'I' increases overshoot a lot
        // 'P' is in a sweet-spot, could go down to 75 and still be OK
        // 'D' didn't make a ton of different, not sure that is tuned properly
        // Quick spin-up and recovery.  There may be a little overshoot just after a shot.
        //shoot1_motor.setVelocityPIDFCoefficients(P, I, D, F);
        //shoot2_motor.setVelocityPIDFCoefficients(P, I, D, F);

        flicker = hardwareMap.servo.get("flicker");

        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            if (Vsense == null) {
                double voltage = sensor.getVoltage();
                if (voltage > 0) {
                    Vsense = sensor;
                }
            }
        }

        waitForStart();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        double lastKp = P;
        double lastKi = I;
        double lastKd = D;
        double lastKf = F;
        double lastPwr = Pwr;
        double lastPIDTime = PIDTime;
        double prevPIDTime1 = PIDTime;
        double prevPIDTime2 = PIDTime;
        double myRPM1 = 0;
        double myRPM2 = 0;
        double myiRPM1 = 0;
        double myiRPM2 = 0;
        double output1 = 0;
        double output2 = 0;
        int singleMode = (int) (1/PIDTime);

        NanoClock clock = NanoClock.system();
        double startTime;

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (enableCSVLogging) {
            // Lay down a header for our logging
            logger.logD("ShooterCSV", String.format(",time,dtime,enc1,enc2,d1,d2,RPM1,RPM2,iRPM1,iRPM2,voltage,Pwr,output1,output2,td1,td2,td3,iter"));
        }

        // Voltage adjust F
        pid.setF(F*(12.8/Vsense.getVoltage()));

        if (isStopRequested()) return;

        startTime = clock.seconds();

        while (!isStopRequested()) {

            rt = clock.seconds() ;
            prt = rt ;
            rt = clock.seconds() ;
            //maxLag = Math.max(maxLag, ((rt-prt)*1000.0));
            iter += 1;

            // One-button timed flick
            // States:  Release and Rearm
            // Release == 0 and rearm == 0 ==> at rest
            // Release > 0  and rearm == 0 ==> waiting for flicker to reach max, release in the future
            // Release == 0 and rearm > 0  ==> waiting for flicker to reach home, prevent re-flick until done
            // Release > 0  and Rearm > 0  ==> illegal
            if ((flickerRelease == 0.0) && (flickerRearm == 0.0)) {
                // This is the 'normal'/'at-rest' case.  No flick in progress.
                // Only flick if the shooter is running.
                // Shoot the flicker
                if ((gamepad1.left_bumper) && (shooterIdx == 1)) {
                    // Set a future time to return flicker to rest
                    flicker.setPosition(FLICKER_SHOOT);
                    flickerRelease = rt + .25;
                    //singleMode = (int)(.75/PIDTime);
                } else {
                    // Just keep asking to return to wait position
                    flicker.setPosition(FLICKER_WAIT);
                }
            } else if (flickerRelease > 0.0) {
                // This is the case that the flick is in progress of pushing the ring in
                if (flickerRelease < rt) {
                    // Once the time has elapsed, move to the next state
                    // and set a timer to wait for release
                    flickerRelease = 0.0;
                    flickerRearm = rt + 0.30;
                } else {
                    // Just keep asking to flick
                    flicker.setPosition(FLICKER_SHOOT);
                }
            } else if (flickerRearm > 0.0) {
                // This is when we are waiting for flicker to return to rest
                if (flickerRearm < rt) {
                    // Once time has elapsed, leave the state
                    flickerRearm = 0.0;
                } else {
                    // keep asking
                    flicker.setPosition(FLICKER_WAIT);
                }
            } else {
                // OK, something is messed up, lets just go back to steady-state
                // Not sure it's possible to get here.
                flicker.setPosition(FLICKER_WAIT);
                flickerRelease = 0.0;
                flickerRearm = 0.0;
            }

            if (gamepad1.dpad_up) {
                if (!dUpPrev[0]) {
                    shooterIdx = (shooterIdx + 1) % shooterSet.length;
                    dUpPrev[0] = true;
                    avgCount = 0;
                    startTime = clock.seconds();
                    nextPID = PIDTime;
                    shoot1_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    shoot2_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    shoot1_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    shoot2_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            } else {
                dUpPrev[0] = false;
            }

            now = clock.seconds()-startTime ;
            if ( nextPID < now ) {

                if ( shooterSet[shooterIdx] > 0.0) {
                    double t1_1 = clock.seconds();
                    shoot1Pos = shoot1_motor.getCurrentPosition();
                    double t1_2 = clock.seconds();
                    td2 = t1_2-t1_1;

                    double t2_1 = clock.seconds();
                    shoot2Pos = shoot2_motor.getCurrentPosition();
                    double t2_2 = clock.seconds();
                    td3 = t2_2-t2_1;

                    int avgIndex = avgCount % avgSize;
                    Pos1[avgIndex] = shoot1Pos;
                    Time1[avgIndex] = t1_2;
                    Pos2[avgIndex] = shoot2Pos;
                    Time2[avgIndex] = t2_2;
                    avgCount++;

                    if (avgCount > 1) {
                        double deltaPos1;
                        double deltaPos2;
                        double deltaTime1;
                        double deltaTime2;

                        int avgIndexH = ((avgCount-1) % avgSize);
                        int avgIndexL = (avgCount % avgSize);
                        if (singleMode > 0 ) {
                            avgIndexL = ((avgCount-2) % avgSize);
                            singleMode--;
                        }

                        deltaPos1 = Pos1[avgIndexH] - Pos1[avgIndexL];
                        deltaPos2 = Pos2[avgIndexH] - Pos2[avgIndexL];
                        deltaTime1 = Time1[avgIndexH] - Time1[avgIndexL];
                        deltaTime2 = Time2[avgIndexH] - Time2[avgIndexL];

                        tps1 = deltaPos1 / deltaTime1;
                        tps2 = deltaPos2 / deltaTime2;

                        myRPM1 = (((tps1 / 28.0) * ratio) * 60.0);
                        myRPM2 = (((tps2 / 28.0) * ratio) * 60.0);
                        double RPMDelta1 = RPM - myRPM1;
                        double RPMDelta2 = RPM - myRPM2;

                        output1 = pid.getOutput(myRPM1, RPM);
                        output2 = pid.getOutput(myRPM2, RPM);

                        //shoot1_motor.setPower(Range.clip(Pwr, 0.0, 0.75));
                        //shoot2_motor.setPower(Range.clip(Pwr, 0.0, 0.75));
                        shoot1_motor.setPower(Range.clip(output1, 0.0, 0.9));
                        shoot2_motor.setPower(Range.clip(output2, 0.0, 0.9));

                        myiRPM1 = (((((shoot1Pos - prevShoot1Pos) / (t1_2 - prevPIDTime1)) / 28.0) * ratio) * 60.0);
                        myiRPM2 = (((((shoot2Pos - prevShoot2Pos) / (t2_2 - prevPIDTime2)) / 28.0) * ratio) * 60.0);

                        if ((myiRPM1 < (RPM*0.90)) || (myiRPM2 < (RPM*0.90))) {
                            singleMode = (int) (.5 / PIDTime);
                        }

                        if (enableCSVLogging) {
                            logger.logD("ShooterCSV", String.format(",%f,%f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.2f,%.3f,%.3f,%.3f,%f,%f,%f,%.0f", now, now - lastPIDTime, shoot1Pos, shoot2Pos, shoot1Pos - prevShoot1Pos, shoot2Pos - prevShoot2Pos, myRPM1, myRPM2, myiRPM1, myiRPM2, Vsense.getVoltage(), Pwr, output1, output2, td1, td2, td3, iter));
                        }
                    }

                    prevShoot1Pos = shoot1Pos;
                    prevShoot2Pos = shoot2Pos;

                    prevPIDTime1 = t1_2;
                    prevPIDTime2 = t2_2;

                    // 'Schedule' the next PID check
                    nextPID = now + PIDTime;
                    lastPIDTime = now;
                } else {
                    shoot1_motor.setPower(0);
                    shoot2_motor.setPower(0);
                    // 'Schedule' the next PID check
                    nextPID = now + PIDTime;
                    lastPIDTime = now;
                }
            }

            td1 = clock.seconds();
            // Show the elapsed game time and wheel power.
            telemetry.addData("Shooter1", myRPM1);
            telemetry.addData("Shooter2", myRPM2);
            telemetry.addData("iShooter1", myiRPM1);
            telemetry.addData("iShooter2", myiRPM2);
            telemetry.addData("PIDOutput1", (output1*1000.0)+3000.0);
            telemetry.addData("PIDOutput2", (output2*1000.0)+3000.0);
            telemetry.addData("FlickerPos", (flicker.getPosition()*1000.0)+3000);
            //telemetry.addData("Shooter2", ((tps2/28.0)*ratio)*60.0);
            telemetry.update();
            td1 = clock.seconds()-td1;

            if (lastKp != P || lastKd != D || lastKi != I || lastKf != F) {
                shoot1_motor.setVelocityPIDFCoefficients(P,I,D,F);
                shoot2_motor.setVelocityPIDFCoefficients(P,I,D,F);
                pid.setPID(P,I,D,F);
                pid.reset();

                lastKp = P;
                lastKi = I;
                lastKd = D;
                lastKf = F;
            }
            if (lastPwr != Pwr) {
                shooterSet[1] = Pwr;
                lastPwr=Pwr;
            }
        }
    }
}

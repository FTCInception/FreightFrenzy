package Inception.FreightFrenzy;

import android.graphics.Color;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



public class TapeMeasureV4 {

    private static DcMotorEx tapeLength_motor;
    private static Servo tapeHeight, tapeRotation;
    private double tapeLengthReq=0.0, tapeHeightReq=0.0, tapeRotationReq=0.0;
    private double targTapeLength=0, targTapeHeight=0.5, targTapeRotation=0.5;
    private double prevTime, deltaT;
    private static double divisor=6.0;
    private static LinearOpMode lOpMode;
    private static RRMechBot robot;
    static final int NONE = 0;
    static final int TAPE_LENGTH_HOME = 0;
    static final int TAPE_DRIVE = 1;
    static final int TAPE_AUTO = 2;
    static final int TAPE_ENDGAME_RED = 3;
    static final int TAPE_ENDGAME_BLUE = 4;
    static final int TAPE_SCORE_RED = 5;
    static final int TAPE_SCORE_BLUE = 6;
    private static int specialTapeRequest = NONE;
    private static Gamepad gamepad;
    private static boolean RedAlliance = true;

    private static final int HGT = 0;
    private static final int ROT = 1;
    private static final double[][] posTargets = {
            { 0.71, 0.71 },     // 0 Nothing
            { 0.71, 0.71 },     // 1 TAPE_DRIVE
            { 0.95, 0.71 },     // 2 TAPE_AUTO
            { 0.65, 0.55 },     // 3 TAPE_ENDGAME_RED
            { 0.71, 0.71 },     // 4 TAPE_ENDGAME_BLUE
            { 0.65, 0.55 },     // 5 TAPE_SCORE_RED
            { 0.71, 0.71 } } ;  // 6 TAPE_SCORE_BLUE

    private static boolean lBumpPrev  = false;
    private static boolean rBumpPrev  = false;
    //private static boolean lTrigPrev  = false;
    //private static boolean lTrig      = false;
    //private static boolean rTrigPrev  = false;
    //private static boolean rTrig      = false;
    private static boolean backPrev   = false;
    private static boolean aPrev      = false;
    private static boolean bPrev      = false;
    private static boolean xPrev      = false;
    private static boolean yPrev      = false;
    private static boolean dUpPrev    = false;
    private static boolean dDownPrev  = false;
    private static boolean dLeftPrev  = false;
    private static boolean dRightPrev = false;
    private static boolean guidePrev  = false;

    private static double dUpPrevTime    = 0;
    private static double dDownPrevTime  = 0;
    private static double dLeftPrevTime  = 0;
    private static double dRightPrevTime = 0;

    NanoClock localClock = NanoClock.system();
    double now = localClock.seconds();
    double safeLockout = 0;

    /* Constructor */
    public TapeMeasureV4() {
    }

    /*
    // TODO: I have no clue if this works
    private boolean isAutoOpMode() {
        String lOpMode_class = lOpMode.getClass().getSimpleName();
        return lOpMode_class.substring(lOpMode_class.length() - 4, lOpMode_class.length()).equals("Auto");
    }
    */

    public void setPosition(int position) {
        targTapeHeight=posTargets[position][HGT];
        targTapeRotation=posTargets[position][ROT];
        tapeRotation.setPosition(targTapeRotation);
        tapeHeight.setPosition(targTapeHeight);
        safeLockout = localClock.seconds() + 2.0;
    }

    public void init(LinearOpMode i_lOpMode, RRMechBot i_robot, Gamepad i_gamepad, boolean i_red) {
        gamepad = i_gamepad;
        lOpMode = i_lOpMode;
        robot = i_robot;
        RedAlliance = i_red;
        tapeHeight = robot.tapeHeight;
        tapeRotation = robot.tapeRotation;
        tapeLength_motor = robot.tapeLength_motor;

        if( tapeLength_motor != null ) {
            tapeLength_motor.setPower(0.0);
            tapeLength_motor.setDirection(DcMotorSimple.Direction.REVERSE);
            tapeLength_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            tapeLength_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            tapeLength_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public boolean SafePosition( ) {
        // Check if height and rotation are within reasonable distances of the DRIVE position
        // Check for a time-based lock-out
        double deltaH = Math.abs(tapeHeight.getPosition() - posTargets[TAPE_DRIVE][HGT]);
        double deltaR = Math.abs(tapeRotation.getPosition() - posTargets[TAPE_DRIVE][ROT]);

        if(safeLockout > localClock.seconds()) {
            return(false);
        }

        if( deltaH > 0.03) {
            return(false);
        }

        if(deltaR > 0.03) {
            return(false);
        }
        return(true);
    }

    private void TapeSpecialReqCheck() {
        if ( (gamepad.left_stick_x > 0.025) ||
             (gamepad.left_stick_y > 0.025) ||
             (gamepad.right_stick_x > 0.025) ||
             (gamepad.right_stick_y > 0.025) ||
             (gamepad.left_trigger > 0.025) ||
             (gamepad.right_trigger > 0.025) ) {
            specialTapeRequest = NONE;
        }
    }

    public void ManageTape( ) {
        // TODO: Add check for unreasonable tape positions in certain drive positions
        if(tapeLength_motor != null) {
            now  = localClock.seconds();
            // Buttons first as these may setup certain states for motion
            ManageButtons(now);
            ManageMotion(now);
        }
    }

    private void ManageButtons( double now ) {

        // Just for debug
        boolean debug = false;
        if (debug) {
            // TODO: Find the right divisor for each length
            if (gamepad.y) {
                if (!yPrev) {
                    divisor += 1.0;
                    yPrev = true;
                }
            } else {
                yPrev = false;
            }

            // TODO: Find the right divisor for each length
            if (gamepad.a) {
                if (!aPrev) {
                    divisor = Math.max(1.0, divisor - 1.0);
                    aPrev = true;
                }
            } else {
                aPrev = false;
            }
        }

        // TODO: Choose the right presets for SCORING and make these alliance dependent
        if (gamepad.x) {
            if (!xPrev) {
                xPrev = true;
                if( RedAlliance ) {
                    setPosition(TAPE_SCORE_RED);
                } else {
                    setPosition(TAPE_SCORE_BLUE);
                }
            }
        } else {
            xPrev = false;
        }

        if (gamepad.b) {
            if (!bPrev) {
                bPrev = true;
                if (tapeLength_motor.getCurrentPosition() < 350) {
                    setPosition(TAPE_DRIVE);
                }
            }
        } else {
            bPrev = false;
        }

        double rAmount = 0.004;
        double hAmount = 0.0025;
        double repeat = 0.1;
        // Use dpad and bumpers to nudge the tape a teeny little bit
        // Add continuous hold mode?
        if (tapeLength_motor.getCurrentPosition() > 4500) {
            rAmount *= 0.3;
            hAmount *= 0.6;
        }
        if (gamepad.dpad_up) {
            if (!dUpPrev) {
                targTapeHeight -= hAmount ;
                dUpPrev = true;
                dUpPrevTime = now;
            }
            else if ((now - dUpPrevTime) > repeat) {
                targTapeHeight -= hAmount ;
                dUpPrevTime = now;
            }
        } else {
            dUpPrev = false;
            dUpPrevTime = 0;
        }

        if (gamepad.dpad_down) {
            if (!dDownPrev) {
                targTapeHeight += hAmount ;
                dDownPrev = true;
                dDownPrevTime = now;
            }
            else if ((now - dDownPrevTime) > repeat) {
                targTapeHeight += hAmount ;
                dDownPrevTime = now;
            }
        } else {
            dDownPrev = false;
            dDownPrevTime = 0;
        }

        if (gamepad.dpad_right) {
            if (!dRightPrev) {
                targTapeRotation -= rAmount ;
                dRightPrev = true;
                dRightPrevTime = now;
            }
            else if ((now - dRightPrevTime) > repeat) {
                targTapeRotation -= rAmount ;
                dRightPrevTime = now;
            }
        } else {
            dRightPrev = false;
            dRightPrevTime = 0;
        }

        if (gamepad.dpad_left) {
            if (!dLeftPrev) {
                targTapeRotation += rAmount ;
                dLeftPrev = true;
                dLeftPrevTime = now;
            }
            else if ((now - dLeftPrevTime) > repeat) {
                targTapeRotation += rAmount ;
                dLeftPrevTime = now;
            }
        } else {
            dLeftPrev = false;
            dLeftPrevTime = 0;
        }

        /*
        if(isAutoOpMode()) {
            specialTapeRequest = TAPE_AUTO;
        }
        */
    }

    private void NormalMotion() {
        // Use discrete programming for height
        // Range is 0 <--> 1.0
        // Add or subtract a little from the position base on stick and time
        tapeLengthReq = (gamepad.right_trigger * 0.8) - gamepad.left_trigger;
        tapeRotationReq = gamepad.left_stick_x + gamepad.right_stick_x;
        tapeHeightReq = gamepad.left_stick_y + gamepad.right_stick_y;

        // Increase the '0' range
        if (Math.abs(tapeLengthReq) <= 0.05) {
            tapeLengthReq = 0;
        }
        if (Math.abs(tapeRotationReq) <= 0.05) {
            tapeRotationReq = 0;
        }
        if (Math.abs(tapeHeightReq) <= 0.05) {
            tapeHeightReq = 0;
        }

        if (tapeLengthReq >= 0.05) {
            tapeLengthReq = (tapeLengthReq * 0.85) + 0.15;
        }
        if (tapeLengthReq <= -0.05) {
            tapeLengthReq = (tapeLengthReq * 0.85) - 0.15;
        }

        // Slow in/out
        if (gamepad.left_bumper) {
            tapeLengthReq = -0.2;
        }
        if (gamepad.right_bumper) {
            tapeLengthReq = 0.2;
        }
    }

    public void ManageMotion( double now ) {
        double currPos;
        // Make sure we start at rest
        tapeLengthReq = 0;
        tapeRotationReq = 0;
        tapeHeightReq = 0;

        // Check if we're trying to move manually
        TapeSpecialReqCheck();

        if (specialTapeRequest == NONE) {
            NormalMotion();
        } else {
            RequestTapePosition(specialTapeRequest);
        }

        deltaT = now - prevTime;
        prevTime = now;

        currPos = Math.max(0.0,(double)(tapeLength_motor.getCurrentPosition()));

        //1500 is close bar code, 3000 is far
        // Points for divisor
        // 6.5 near TSE @ 2000
        // 11 at wobble @ 4000
        // 13 far TSE @ 5200 (button at 60% here)
        tapeHeightReq *= deltaT / (divisor + Math.max(0.0,((currPos-1400)/400.0)));
        tapeHeightReq += targTapeHeight;
        tapeHeightReq = Math.max(0.0, Math.min(tapeHeightReq, 1.0));
        targTapeHeight = tapeHeightReq;

        // Points for divisor
        // 8 near TSE @ 2000
        // 15 at the wobble @ 4000
        // 18 far TSE @ 5200 (button at 60% here)
        tapeRotationReq *= -(deltaT / (divisor + Math.max(0.0,((currPos-1250)/325.0))));
        tapeRotationReq += targTapeRotation;
        tapeRotationReq = Math.max(0.00, Math.min(tapeRotationReq, 1.0));
        targTapeRotation = tapeRotationReq;

        // Half power when we're close
        if ((tapeLength_motor.getCurrentPosition() < 1000.0) && (tapeLengthReq < 0.0)  ) {
            tapeLengthReq *= 0.25;
        }
        // Now apply the power:
        tapeLength_motor.setPower(tapeLengthReq);
        tapeRotation.setPosition(tapeRotationReq);
        tapeHeight.setPosition(tapeHeightReq);
    }

    public void RequestTapePosition(int pos) {
        double targRot = posTargets[pos][ROT] ;
        double targHgt = posTargets[pos][HGT] ;
        double rotationDelta = Math.abs(targRot - targTapeRotation);
        double heightDelta = Math.abs(targHgt - targTapeHeight);

        if (rotationDelta > 0.01) {
            // If we're 'far' away, fake some controller requests to limit correction speed
            tapeRotationReq = 0.5;
            if (targTapeRotation < targRot) {
                tapeRotationReq *= -1.0;
            }
        } else {
            // if we're close, just set the position directly and no controller
            targTapeRotation = targRot;
            tapeRotationReq = 0;
        }

        if (heightDelta > 0.01) {
            // If we're 'far' away, fake some controller requests to limit correction speed
            tapeHeightReq = -0.25;
            if (targTapeHeight < targHgt) {
                tapeHeightReq *= -1.0;
            }
        } else {
            // if we're close, just set the position directly and no controller
            targTapeHeight = targHgt ;
            tapeHeightReq = 0;
        }

        // If we're really close on both, then we're done
        if ((rotationDelta <= 0.005) && (heightDelta <= 0.005)) {
            specialTapeRequest = NONE;
        }
    }

    public void telemetry( Telemetry telem ) {
        if( tapeLength_motor != null ) {
            double currPos = Math.max(0.0,(double)(tapeLength_motor.getCurrentPosition()));
            double dH = (divisor + Math.max(0.0,((currPos-1700)/450.0)));
            double dR = (divisor + Math.max(0.0,((currPos-1250)/325.0)));

            telem.addData("Tape: ", "L:%.2f, H:%.2f, R:%.2f, P:%.0f, dH:%.1f, dR:%.1f", tapeLengthReq, tapeHeight.getPosition(), tapeRotation.getPosition(), currPos, dH, dR);
        } else {
            telem.addData("Tape: ", "Not installed");
        }
    }

}

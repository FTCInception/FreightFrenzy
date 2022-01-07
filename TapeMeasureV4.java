package Inception.FreightFrenzy;

import android.graphics.Color;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



public class TapeMeasureV4 {

    private static DcMotorEx tapeLength_motor;
    private static Servo tapeHeight, tapeRotation;
    private double tapeLengthReq, tapeHeightReq, tapeRotationReq;
    private double prevTapeLength, prevTapeHeight, prevTapeRotation;
    private double prevTime, deltaT;
    private static double divisor=0.0;
    private static LinearOpMode lOpMode;
    private static int tape;
    private static final int NONE = 0;
    private static final int TAPE_LENGTH_HOME = 0;
    private static final int TAPE_DRIVE = 1;
    private static final int TAPE_AUTO = 2;
    private static final int TAPE_ENDGAME_RED = 3;
    private static final int TAPE_ENDGAME_BLUE = 4;
    private static final int TAPE_SCORE_RED = 5;
    private static final int TAPE_SCORE_BLUE = 6;
    private static int specialTapeRequest = NONE;
    private static Gamepad gamepad;

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

    /* Constructor */
    public TapeMeasureV4() {
    }

    public void init(LinearOpMode i_lOpMode, Servo i_tapeHeight, Servo i_tapeRotation, DcMotorEx i_tapeLength_motor, Gamepad i_gamepad) {

        gamepad = i_gamepad;
        lOpMode = i_lOpMode;
        tapeHeight = i_tapeHeight;
        tapeRotation = i_tapeRotation;
        tapeLength_motor = i_tapeLength_motor;

        if( tapeLength_motor != null ) {
            tapeLength_motor.setPower(0.0);
            tapeLength_motor.setDirection(DcMotorSimple.Direction.REVERSE);
            tapeLength_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            tapeLength_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            tapeLength_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
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

    public void ManageTape( double now ) {
        if( tapeLength_motor != null ) {
            // Buttons first as these may setup certain states for motion
            ManageButtons(now);
            ManageMotion(now);
        }
    }

    private void ManageButtons( double now ) {

        // TODO: Find the right divisor for each length
        if (gamepad.y) {
            if (!yPrev) {
                divisor += 1 ;
                yPrev = true;
            }
        } else {
            yPrev = false;
        }

        // TODO: Find the right divisor for each length
        if (gamepad.a) {
            if (!aPrev) {
                divisor = Math.max(1, divisor - 1) ;
                aPrev = true;
            }
        } else {
            aPrev = false;
        }

        // TODO: Choose the right presets for SCORING and make these alliance dependent
        if (gamepad.x) {
            if (!xPrev) {
                specialTapeRequest = TAPE_SCORE_RED ;
                xPrev = true;
            }
        } else {
            xPrev = false;
        }

        // Use dpad and bumpers to nudge the tape a teeny little bit
        // Add continuous hold mode?
        if (gamepad.dpad_up) {
            if (!dUpPrev) {
                prevTapeHeight -= 0.0025 ;
                dUpPrev = true;
            }
        } else {
            dUpPrev = false;
        }

        if (gamepad.dpad_down) {
            if (!dDownPrev) {
                prevTapeHeight += 0.0025 ;
                dDownPrev = true;
            }
        } else {
            dDownPrev = false;
        }

        if (gamepad.dpad_right) {
            if (!dRightPrev) {
                prevTapeRotation -= 0.0025 ;
                dRightPrev = true;
            }
        } else {
            dRightPrev = false;
        }

        if (gamepad.dpad_left) {
            if (!dLeftPrev) {
                prevTapeRotation += 0.0025 ;
                dLeftPrev = true;
            }
        } else {
            dLeftPrev = false;
        }

        // TODO: Add button/bumper nudge for Length?
    }

    private void NormalMotion( ) {
        // Use discrete programming for height
        // Range is 0 <--> 1.0
        // Add or subtract a little from the position base on stick and time
        tapeLengthReq = gamepad.right_trigger - gamepad.left_trigger;
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
    }

    public void ManageMotion( double now ) {

        // Make sure we start at rest
        tapeLengthReq = 0;
        tapeRotationReq = 0;
        tapeHeightReq = 0;

        // Check if we're trying to move manually
        TapeSpecialReqCheck();

        if( specialTapeRequest == NONE) {
            NormalMotion();
        } else if(specialTapeRequest == TAPE_SCORE_RED) {
            RequestTapeScoringPositionRed();
        }

        tapeLengthReq *= 0.40;

        deltaT = now - prevTime;
        prevTime = now;

        //1500 is close bar code, 3000 is far
        //tapeHeightReq *= deltaT / (2.0 + ((double)(tapeLength_motor.getCurrentPosition())/300.0));
        tapeHeightReq *= (deltaT / (2.0 + divisor));
        tapeHeightReq += prevTapeHeight;
        tapeHeightReq = Math.max(0.0, Math.min(tapeHeightReq, 1.0));
        prevTapeHeight = tapeHeightReq;

        //tapeRotationReq *= deltaT / (2.0 + ((double)(tapeLength_motor.getCurrentPosition())/150.0));
        tapeRotationReq *= -(deltaT / (2.0 + divisor));
        tapeRotationReq += prevTapeRotation;
        tapeRotationReq = Math.max(0.00, Math.min(tapeRotationReq, 1.0));
        prevTapeRotation = tapeRotationReq;

        // Now apply the power:
        if ((tapeLength_motor.getCurrentPosition() < 50.0) && (tapeLengthReq < 0.0)  ) {
            tapeLengthReq = 0.0;
        }
        tapeLength_motor.setPower(tapeLengthReq);
        tapeRotation.setPosition(tapeRotationReq);
        tapeHeight.setPosition(tapeHeightReq);
    }

    public void RequestTapeLengthHome() {
    }

    public void RequestTapeDrivePosition() {
    }

    public void RequestTapeAutoPosition() {
    }

    public void RequestTapeStartEndGamePositionRed() {
    }

    public void RequestTapeStartEndGamePositionBlue() {
    }

    public void RequestTapeScoringPositionRed() {
        double rotationDelta = Math.abs(0.28 - prevTapeRotation);
        double heightDelta = Math.abs(0.34 - prevTapeHeight);

        if (rotationDelta > 0.01) {
            if (prevTapeRotation < 0.28) {
                tapeRotationReq = -0.5;
            } else {
                tapeRotationReq = 0.5;
            }
        } else {
            tapeRotationReq = 0;
        }
        if (heightDelta > 0.01) {
            if (prevTapeHeight < 0.34) {
                tapeHeightReq = 0.5;
            } else {
                tapeHeightReq = -0.5;
            }
        } else {
            tapeHeightReq = 0;
        }
        if ((rotationDelta <= 0.01) && (heightDelta <= 0.01)) {
            specialTapeRequest = NONE;
        }
    }

    public void RequestTapeScoringPositionBlue() {
    }

    public void telemetry( Telemetry telem ) {
        if( tapeLength_motor != null ) {
            telem.addData("Tape: ", "L:%.2f, H:%.2f, R:%.2f, P:%d, D:%.0f", tapeLengthReq, tapeHeightReq, tapeRotationReq, tapeLength_motor.getCurrentPosition(), divisor);
        } else {
            telem.addData("Tape: ", "Not installed");
        }
    }

}

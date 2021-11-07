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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

/**
 * This file houses Autonomous code
 * It uses the main robot class Refbot to define the hardware and driving commands
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *
 *  See the Refbot class for encode-based driving controls that perform the actual movement.
 *
 */
@Disabled
@Autonomous(name="Red_Ref_Auto", group="RRMechBot")
public class Red_Ref_Auto extends LinearOpMode {
    private final int BAR1=0;
    private final int BAR2=1;
    private final int BAR3=2;

    private RRMechBot robot = new RRMechBot();

    private static final double intake_pickup_ring = 1.0;

    // This is the starting position of the center of the robot.
    private static final double startingX = -63.0;
    private static final double startingY = -17.0;

    private static boolean option1 = true;
    private static boolean option2 = true;
    private static boolean option3 = true;

    private IncepVision vision = new IncepVision();
    private IncepVision.MarkerPos grnLocation = IncepVision.MarkerPos.Unseen;
    private Trajectory[][] trajs = {new Trajectory[25], new Trajectory[25], new Trajectory[25]} ;

    @Override
    public void runOpMode() {
        //robot.logger.LOGLEVEL = robot.logger.LOGDEBUG ;

        // Init the robot and subsystems.  Set Roadrunner timeout to 0.25s to save some time.
        robot.init(hardwareMap,0.5);
        robot.initAutonomous(this);
        robot.logger.LOGLEVEL |= robot.logger.LOGDEBUG;

        // This code allows for processing the starting location of something variable
        // And controls some enable/disable options
        double Sx = 0.0;
        double Sy = 0.0;
        boolean leftOK = true, rightOK = true, upOK = true, downOK = true, bOK=true, xOK=true, yOK=true;
        do {
            if ( gamepad1.dpad_left || gamepad2.dpad_left ) {
                if (leftOK) {
                    Sx -= 1.0;
                    leftOK = false;
                }
            } else {
                leftOK = true;
            }
            if ( gamepad1.dpad_right || gamepad2.dpad_right) {
                if (rightOK) {
                    Sx += 1.0;
                    rightOK = false;
                }
            } else {
                rightOK = true;
            }
            if ( gamepad1.dpad_down || gamepad2.dpad_down ) {
                if (downOK) {
                    Sy -= 1.0;
                    downOK = false;
                }
            } else {
                downOK = true;
            }
            if ( gamepad1.dpad_up || gamepad2.dpad_up ) {
                if (upOK) {
                    Sy += 1.0;
                    upOK = false;
                }
            } else {
                upOK = true;
            }
            if ( gamepad1.y || gamepad2.y ) {
                if (yOK) {
                    option1 = !option1;
                    yOK = false;
                }
            } else {
                yOK = true;
            }

            if ( gamepad1.x || gamepad2.x ) {
                if (xOK) {
                    option2 = !option2;
                    xOK = false;
                }
            } else {
                xOK = true;
            }
            if ( gamepad1.b ||gamepad2.b ) {
                if (bOK) {
                    option3 = !option3;
                    bOK = false;
                }
            } else {
                bOK = true;
            }
            if ( gamepad1.a ||gamepad2.a ) {
                break;
            }

            telemetry.addData("Sx,Sy:", "(%.0f, %.0f); New:(%.0f, %.0f); Delta:(%.0f%s, %.0f%s)", startingX, startingY, startingX+Sx, startingY+Sy, Math.abs(Sx), (Sx<0)?" left":(Sx>0)?" right":"", Math.abs(Sy),(Sy<0)?" down":(Sy>0)?" up":"");
            telemetry.addData("Use dpad to adjust robot start position","");
            telemetry.addData("'Y' option1:","(%s)", option1?"true":"false");
            telemetry.addData("'X' option2:","(%s)", option2?"true":"false");
            telemetry.addData("'B' option3:","(%s)", option3?"true":"false");
            telemetry.addData("'A' NA:","(%s)", "NA");
            telemetry.update();

        } while (!isStarted() && (!isStopRequested())) ;

        // Robot center is 9" from each edge:
        // Back against the -x wall, wheels aligned on first tile in -y
        // THIS MUST BE DONE BEFORE BUILDING
        // THIS MUST BE DONE AFTER THE ROBOT IS IN ITS FINAL POSITION
        Pose2d startPose = new Pose2d(startingX+Sx, startingY+Sy, robot.drive.getRawExternalHeading());
        robot.drive.setPoseEstimate(startPose);

        telemetry.addData("Computing paths","");
        telemetry.update();

        BuildBar1(trajs[BAR1]);
        BuildBar2(trajs[BAR2]);
        BuildBar3(trajs[BAR3]);

        telemetry.addData("Starting vision", "");
        telemetry.update();
        // Stare at the rings really hard until its time to go or stop
        vision.initAutonomous(this, "webcam", vision.RED_DUCK);
        vision.clip = false;
        boolean leftBOK = true, rightBOK = true;
        do {
            grnLocation = vision.getGrnLocation();
            vision.manageVisionBox(gamepad1, gamepad2);
        } while (!isStarted() && (!isStopRequested()));
        vision.shutdown();

        // TODO: Make sure the INNER/OUTER/UNSEEN mapping here is correct for every auto.
        // FIXME: Test the stopping in auto here again.
        if (opModeIsActive()) {
            if (grnLocation == IncepVision.MarkerPos.Inner) {
                Bar1(trajs[BAR1]);
            } else if (grnLocation == IncepVision.MarkerPos.Outer) {
                Bar2(trajs[BAR2]);
            } else {
                // If it's not INNER or OUTER, assume unseen
                Bar3(trajs[BAR3]);
            }
        }
    }

    private void showTrajPoses( String trajName, int TIdx, Trajectory[] traj ) {

        Pose2d tmpPose = traj[0].start();
        double time = 0;
        robot.logger.logD("showTrajPoses:",String.format("%s, Idx:%d, X: %.2f, Y:%.2f, H:%.2f, t:%.2f", trajName, -1, tmpPose.getX(), tmpPose.getY(), Math.toDegrees(tmpPose.getHeading()),time));

        for (int i=0; i<TIdx; i++ ) {
            tmpPose = traj[i].end();
            time += traj[i].duration();
            robot.logger.logD("showTrajPoses:",String.format("%s, Idx:%d, X: %.2f, Y:%.2f, H:%.2f, t:%.2f", trajName, i, tmpPose.getX(), tmpPose.getY(), Math.toDegrees(tmpPose.getHeading()),time));
        }
    }

    private void CheckWait(boolean checkDrive, double minMS, double maxMS) {

        NanoClock localClock = NanoClock.system();
        double now = localClock.seconds();

        // Convert to seconds
        minMS /= 1000;
        maxMS /= 1000;

        minMS += now ;
        if (maxMS > 0) {
            maxMS += now;
        } else {
            maxMS = Double.POSITIVE_INFINITY;
        }

        while(opModeIsActive() ) {
            // Get the time
            now = localClock.seconds();

            // Master stop
            if(!opModeIsActive()){ return; }

            // Update the drive
            if( checkDrive ) { robot.drive.update(); }

            // Check timer expiration, bail if too long
            if(maxMS < now) { return; }

            // Make sure to wait for the minimum time
            if(minMS > now) {
                continue;
            }

            // Drive still running? Wait for it.
            if ( checkDrive ) {
                if(robot.drive.isBusy()){ continue; }
            }

            // No reason to be here (past the minMS timer, drive is idle)
            return;
        }
    }

    private void BuildBar1(Trajectory[] traj) {

        int TIdx = 0;
        // Starting X,Y = ??,??

        // Drive to first position
        traj[TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .forward(12)
                .build();

        // Drive to second position
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                .back(12)
                .build();

        showTrajPoses( "BAR1", TIdx, traj ) ;
    }

    private void BuildBar2(Trajectory[] traj) {

        int TIdx = 0;
        // Starting X,Y = ??,??

        // Drive to first position
        traj[TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .forward(12)
                .build();

        // Drive to second position
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                .back(12)
                .build();

        showTrajPoses( "BAR2", TIdx, traj ) ;
    }

    private void BuildBar3(Trajectory[] traj) {

        int TIdx = 0;
        // Starting X,Y = ??,??

        // Drive to first position
        traj[TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .forward(12)
                .build();

        // Drive to second position
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                .back(12)
                .build();

        showTrajPoses( "BAR3", TIdx, traj ) ;
    }

    private void Bar1(Trajectory[] traj) {
        int TIdx = 0;

        // Drive to position 1
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, 0, 0);
        if(!opModeIsActive()){ return; }

        // Drive to position 2
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, 0, 0);
        if(!opModeIsActive()){ return; }
    }

    private void Bar2(Trajectory[] traj) {
        int TIdx = 0;

        // Drive to position 1
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, 0, 0);
        if(!opModeIsActive()){ return; }

        // Drive to position 2
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, 0, 0);
        if(!opModeIsActive()){ return; }
    }

    private void Bar3(Trajectory[] traj) {
        int TIdx = 0;

        // Drive to position 1
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, 0, 0);
        if(!opModeIsActive()){ return; }

        // Drive to position 2
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, 0, 0);
        if(!opModeIsActive()){ return; }
    }
}
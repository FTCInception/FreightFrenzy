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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="Red_Middle_IP", group="MechBot")
public class Red_Middle_IP extends LinearOpMode {
    private final int RING0_IP=0;
    private final int RING1_IP=1;
    private final int RING4_IP=2;

    private double POWER_SHOT_ANGLE1 = 6.5;
    private double POWER_SHOT_ANGLE2 = 0.0;
    private double POWER_SHOT_ANGLE3 = -6.5;

    private double TOWER_SHOT_ANGLE  = -14.0;

    private double yLane = -9.0;

    private RRMechBot robot = new RRMechBot();

    // These are for the REV HUB...
    private static final double high_tower_power = 0.4775;
    private static final double power_shot_power = 0.435;

    // Stealth Wheel blue RPMs 3/26/21
    // These are for the SW PID...
    private static final double side_high_tower_RPM = 3450;
    private static final double power_shot_RPM = 3125;

    private static final double flicker_shot_delay = 250;
    private static final double flicker_return_delay = 350;

    // This is a one-stop switchover from SWPID back to REV HUB PID
    // The setShooter function consumes this flag and will switch back and forth as needed.
    private static boolean SWPID = true;

    // This is the starting position of the center of the robot.
    private static final double startingX = -63.0;
    private static final double startingY = -17.0;

    private static boolean wobbleEnabled = true;
    private static boolean powerShots = false;
    private static boolean sideDelivery = true;

    private IncepVision vision = new IncepVision();
    private int ringCount = -1;
    private Trajectory[][] trajs = {new Trajectory[25], new Trajectory[25], new Trajectory[25]} ;

    @Override
    public void runOpMode() {
        //robot.logger.LOGLEVEL = robot.logger.LOGDEBUG ;

        // Init the robot and subsystems.  Set Roadrunner timeout to 0.25s to save some time.
        robot.init(hardwareMap,0.25);
        robot.initAutonomous(this);
        robot.logger.LOGLEVEL |= robot.logger.LOGDEBUG;

        // This code is for changing the location of the second wobble
        // It is unused for middle lane right now.  But if we want to
        // allow some variation in our startign location, we'll re-use this code.
        double Sx = 0.0;
        double Sy = 0.0;
        boolean leftOK = true, rightOK = true, upOK = true, downOK = true, bOK=true, xOK=true;
        do {
            if ( gamepad2.dpad_left ) {
                if (leftOK) {
                    Sx -= 1.0;
                    leftOK = false;
                }
            } else {
                leftOK = true;
            }
            if ( gamepad2.dpad_right ) {
                if (rightOK) {
                    Sx += 1.0;
                    rightOK = false;
                }
            } else {
                rightOK = true;
            }
            if ( gamepad2.dpad_down ) {
                if (downOK) {
                    Sy -= 1.0;
                    downOK = false;
                }
            } else {
                downOK = true;
            }
            if ( gamepad2.dpad_up ) {
                if (upOK) {
                    Sy += 1.0;
                    upOK = false;
                }
            } else {
                upOK = true;
            }
            if ( gamepad2.y ) {
                if (xOK) {
                    sideDelivery = !(sideDelivery);
                    xOK = false;
                }
            } else {
                xOK = true;
            }
            if ( gamepad2.x ) {
                if (xOK) {
                    wobbleEnabled = !(wobbleEnabled);
                    xOK = false;
                }
            } else {
                xOK = true;
            }
            if ( gamepad2.b ) {
                if (bOK) {
                    powerShots = !(powerShots);
                    bOK = false;
                }
            } else {
                bOK = true;
            }
            if ( gamepad2.a ) {
                break;
            }

            telemetry.addData("Sx,Sy:", "(%.0f, %.0f); New:(%.0f, %.0f); Delta:(%.0f%s, %.0f%s)", startingX, startingY, startingX+Sx, startingY+Sy, Math.abs(Sx), (Sx<0)?" left":(Sx>0)?" right":"", Math.abs(Sy),(Sy<0)?" down":(Sy>0)?" up":"");
            telemetry.addData("Use dpad to adjust robot start position","");
            telemetry.addData("'X' to toggle wobble drop-off:","(%s)", wobbleEnabled?"enable":"disabled");
            telemetry.addData("'B' to toggle 2x power shots vs 3x tower shots:","(%s)", powerShots?"power":"tower");

            if(wobbleEnabled) {
                telemetry.addData("'Y' to toggle side or back delivery:", "(%s)", sideDelivery ? "side" : "back");
                telemetry.addData("'A' to proceed to vision ", "(%.1f)", Math.toDegrees(robot.drive.getRawExternalHeading()));
            } else {
                telemetry.addData("'A' to proceed (no vision)", "(%.1f)", Math.toDegrees(robot.drive.getRawExternalHeading()));
            }
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

        if (wobbleEnabled) {
            if (sideDelivery) {
                BuildR0_IP(trajs[RING0_IP]);
                BuildR1_IP(trajs[RING1_IP]);
            } else {
                BuildR0Back_IP(trajs[RING0_IP]);
                BuildR1Back_IP(trajs[RING1_IP]);
            }

            BuildR4_IP(trajs[RING4_IP]);
        } else {
            BuildNoWobble_IP(trajs[RING0_IP]);
        }

        if(wobbleEnabled) {
            telemetry.addData("Starting vision", "");
            telemetry.update();
            // Stare at the rings really hard until its time to go or stop
            vision.initAutonomous(this, "RightWebcam");
            vision.clip = false;
            int deltaX = 0, deltaY = 0;
            boolean leftBOK = true, rightBOK = true;
            do {
                ringCount = vision.countRings();

                if (gamepad2.dpad_left) {
                    IncepVision.clipLeft -= 1;
                }
                if (gamepad2.dpad_right) {
                    IncepVision.clipLeft += 1;
                }
                if (gamepad2.dpad_down) {
                    IncepVision.clipTop += 1;
                }
                if (gamepad2.dpad_up) {
                    IncepVision.clipTop -= 1;
                }
                if (gamepad2.x) {
                    IncepVision.clipRight += 1;
                }
                if (gamepad2.b) {
                    IncepVision.clipRight -= 1;
                }
                if (gamepad2.a) {
                    IncepVision.clipBottom -= 1;
                }
                if (gamepad2.y) {
                    IncepVision.clipBottom += 1;
                }
                if (gamepad2.left_bumper) {
                    if (leftBOK) {
                        if (vision.tfodState) {
                            vision.tfod.deactivate();
                            vision.tfodState = false;
                            vision.clip = true;
                        } else {
                            vision.tfod.activate();
                            vision.tfodState = true;
                            vision.clip = false;
                        }
                        leftBOK = false;
                    }
                } else {
                    leftBOK = true;
                }

                // Move the entire box with the joystick.
                deltaY += (int) (gamepad2.left_stick_y * 2.1);
                deltaX += (int) (gamepad2.left_stick_x * 2.1);
                deltaY += (int) (gamepad2.right_stick_y * 2.1);
                deltaX += (int) (gamepad2.right_stick_x * 2.1);
                IncepVision.clipTop += deltaY;
                IncepVision.clipBottom -= deltaY;
                IncepVision.clipLeft += deltaX;
                IncepVision.clipRight -= deltaX;

                // Observe some limits
                IncepVision.clipLeft = Range.clip(IncepVision.clipLeft, 5, 635);
                IncepVision.clipRight = Range.clip(IncepVision.clipRight, 5, 635);
                IncepVision.clipTop = Range.clip(IncepVision.clipTop, 5, 475);
                IncepVision.clipBottom = Range.clip(IncepVision.clipBottom, 5, 475);
            } while (!isStarted() && (!isStopRequested()));
            vision.shutdown();
        } else {
            telemetry.addData("Ready","");
            while (!isStarted() && (!isStopRequested())) {}
        }

        // FIXME: Test the stopping in auto here again.
        if (opModeIsActive()) {
            if (wobbleEnabled) {
                if (ringCount == 0) {
                    if (sideDelivery) {
                        Ring_IP(trajs[RING0_IP], wobbleEnabled, powerShots);
                    } else {
                        Ring0Back_IP(trajs[RING0_IP], wobbleEnabled, powerShots);
                    }
                } else if (ringCount == 1) {
                    if (sideDelivery) {
                        Ring_IP(trajs[RING1_IP], wobbleEnabled, powerShots);
                    } else {
                        Ring1Back_IP(trajs[RING1_IP], wobbleEnabled, powerShots);
                    }
                } else {
                    // If it's not 0 or 1, assume 4 (highest possible point total)
                    Ring_IP(trajs[RING4_IP], wobbleEnabled, powerShots);
                }
            } else {
                NoWobble_IP(trajs[RING0_IP], wobbleEnabled, powerShots);
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

    private void CheckWait(boolean checkDrive, boolean runShooterPID, double minMS, double maxMS) {

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

            // Update the shooterPID
            if ( runShooterPID ) { robot.updateShooterPID(); }

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

    private void BuildNoWobble_IP(Trajectory[] traj) {

        int TIdx = 0;
        // Starting X,Y = -63,-17

        // Extract from wobble
        traj[TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(robot.drive.getPoseEstimate().getX(),robot.drive.getPoseEstimate().getY()+6, Math.toRadians(0.0)))
                .build();

        // Drive to first shot
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                .lineToConstantHeading(new Vector2d(-12, yLane))
                .build();

        // Turn to shoot 3 power shots, finish pointing in towards the tower
        // or
        // Turn to shoot 3 tower shots

        // Go park
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                .lineToLinearHeading(new Pose2d(12, yLane, Math.toRadians(0.0)))
                .build();

        showTrajPoses( "NO_WOBBLE_IP", TIdx, traj ) ;
    }

    private void BuildR0_IP(Trajectory[] traj) {

        int TIdx = 0;
        // Starting X,Y = -63,-17

        // Drive to first shot
        traj[TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(-12, yLane))
                .build();

        // Turn to shoot 3 power shots, finish pointing in towards the tower
        // or
        // Turn to shoot 3 tower shots

        // Drive to wobble drop -- side delivery
        traj[TIdx++] = robot.drive.trajectoryBuilder(new Pose2d(traj[TIdx-2].end().getX(),traj[TIdx-2].end().getY(), Math.toRadians(180.0)),true)
                .splineToSplineHeading(new Pose2d(12,-36, Math.toRadians(90.0)),Math.toRadians(-90.0))
                .build();

        // Lower arm, drop wobble, return arm

        // Go park
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                .splineToSplineHeading(new Pose2d(12, yLane, Math.toRadians(180.0)),Math.toRadians(180.0))
                .build();

        showTrajPoses( "RING0_IP", TIdx, traj ) ;
    }

    private void BuildR1_IP(Trajectory[] traj) {

        int TIdx = 0;
        // Starting X,Y = -63,-17

        // Drive to first shot
        traj[TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(-12, yLane))
                .build();

        // Turn to shoot 3 power shots, finish pointing in towards the tower
        // or
        // Turn to shoot 3 tower shots

        // Drive to wobble drop - side delivery
        traj[TIdx++] = robot.drive.trajectoryBuilder(new Pose2d(traj[TIdx-2].end().getX(),traj[TIdx-2].end().getY(), Math.toRadians(180.0)),true)
                .lineToLinearHeading(new Pose2d(36,-12, Math.toRadians(90.0)))
                .build();
        // Lower arm, drop wobble, return arm

        // Go park
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                .splineToSplineHeading(new Pose2d(12, yLane, Math.toRadians(180.0)),Math.toRadians(180.0))
                .build();

        showTrajPoses( "RING1_IP", TIdx, traj ) ;
    }

    private void BuildR4_IP(Trajectory[] traj) {

        int TIdx = 0;
        // Starting X,Y = -63,-17

        // Drive to first shot
        traj[TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(-12, yLane))
                .build();

        // Turn to shoot 3 power shots, finish pointing in towards the tower
        // or
        // Turn to shoot 3 tower shots

        // Drive to Wobble drop -- side delivery
        traj[TIdx++] = robot.drive.trajectoryBuilder(new Pose2d(traj[TIdx-2].end().getX(),traj[TIdx-2].end().getY(), Math.toRadians(180.0)),true)
                .splineToSplineHeading(new Pose2d(58,-36, Math.toRadians(90.0)),Math.toRadians(-90.0))
                .build();

        // Lower arm, drop wobble, return arm

        // Go park
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                .splineToSplineHeading(new Pose2d(12, yLane, Math.toRadians(180.0)),Math.toRadians(180.0))
                .build();

        showTrajPoses( "RING4_IP", TIdx, traj ) ;
    }

    private void NoWobble_IP(Trajectory[] traj, boolean wobbleEnabled, boolean powerShots) {
        int TIdx = 0;

        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);

        // Start shooter
        if(powerShots) {
            robot.setShooter(power_shot_RPM, power_shot_power, SWPID);
        } else {
            robot.setShooter(side_high_tower_RPM, high_tower_power, SWPID);
        }

        // Drive to shooting location
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Handle the shooting
        power_tower_shots(powerShots);
        if(!opModeIsActive()){ return; }

        // Turn around and small pause
        robot.drive.turnAsync(Math.toRadians(0.0) - robot.drive.getRawExternalHeading());
        CheckWait(true, SWPID, 0, 0);
        CheckWait(true, SWPID, 250, 0);
        if(!opModeIsActive()){ return; }

        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

    }



    private void Ring_IP(Trajectory[] traj, boolean wobbleEnabled, boolean powerShots) {
        int TIdx = 0;

        // Start shooter, raise wobble high
        if(powerShots) {
            robot.setShooter(power_shot_RPM, power_shot_power, SWPID);
        } else {
            robot.setShooter(side_high_tower_RPM, high_tower_power, SWPID);
        }
        robot.setWobblePosition(robot.WOBBLE_UP,0.4);

        // Drive to shooting location
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Handle the shooting
        power_tower_shots(powerShots);
        if(!opModeIsActive()){ return; }

        // Turn around and small pause
        robot.drive.turnAsync(Math.toRadians(180.0) - robot.drive.getRawExternalHeading());
        CheckWait(true, SWPID, 0, 0);
        CheckWait(true, SWPID, 250, 0);
        if(!opModeIsActive()){ return; }

        // Go to wobble drop
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Lower arm
        robot.setWobblePosition(robot.WOBBLE_DROP,0.5);
        CheckWait(true, SWPID, 1500, 0);
        if(!opModeIsActive()){ return; }

        // Drop wobble
        robot.claw.setPosition(0.5);
        CheckWait(true, SWPID, 250, 0);
        if(!opModeIsActive()){ return; }

        // Return arm
        robot.setWobblePosition(robot.WOBBLE_START,0.5);
        CheckWait(true, SWPID, 500, 0);
        if(!opModeIsActive()){ return; }

        // Close claw
        robot.claw.setPosition(1.0);
        if(!opModeIsActive()){ return; }

        // Park in the middle
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

    }


    private void power_tower_shots( boolean powerShots ) {

        if(powerShots) {
            robot.drive.turnAsync(Math.toRadians(POWER_SHOT_ANGLE1) - robot.drive.getRawExternalHeading());
            CheckWait(true, SWPID, 0, 0);

            // First Power Shot
            robot.flicker.setPosition(1.0);
            CheckWait(true, SWPID, flicker_shot_delay, 0);
            robot.flicker.setPosition(0.0);
            if (!opModeIsActive()) { return; }

            // Second Power shot
            robot.drive.turnAsync(Math.toRadians(POWER_SHOT_ANGLE2) - robot.drive.getRawExternalHeading());
            CheckWait(true, SWPID, flicker_return_delay, 0);
            robot.flicker.setPosition(1.0);
            CheckWait(true, SWPID, flicker_shot_delay, 0);
            robot.flicker.setPosition(0.0);
            if (!opModeIsActive()) { return; }

            // Third Power shot
            robot.drive.turnAsync(Math.toRadians(POWER_SHOT_ANGLE3) - robot.drive.getRawExternalHeading());
            CheckWait(true, SWPID, flicker_return_delay, 0);
            robot.flicker.setPosition(1.0);
            CheckWait(true, SWPID, flicker_shot_delay, 0);
            robot.flicker.setPosition(0.0);
            robot.setShooter(0, 0, SWPID);
            if (!opModeIsActive()) { return; }

        } else {
            robot.drive.turnAsync(Math.toRadians(TOWER_SHOT_ANGLE) - robot.drive.getRawExternalHeading());
            CheckWait(true, SWPID, 0, 0);

            // First Tower Shot
            robot.flicker.setPosition(1.0);
            CheckWait(true, SWPID, flicker_shot_delay, 0);
            robot.flicker.setPosition(0.0);
            CheckWait(true, SWPID, flicker_return_delay, 0);
            if (!opModeIsActive()) { return; }

            // Second Tower shot
            robot.flicker.setPosition(1.0);
            CheckWait(true, SWPID, flicker_shot_delay, 0);
            robot.flicker.setPosition(0.0);
            CheckWait(true, SWPID, flicker_return_delay, 0);
            if (!opModeIsActive()) { return; }

            // Third tower shot
            robot.flicker.setPosition(1.0);
            CheckWait(true, SWPID, flicker_shot_delay, 0);
            robot.flicker.setPosition(0.0);
            CheckWait(true, SWPID, flicker_return_delay, 0);

            // Fourth for good measure
            robot.flicker.setPosition(1.0);
            CheckWait(true, SWPID, flicker_shot_delay, 0);
            robot.flicker.setPosition(0.0);
            CheckWait(true, SWPID, flicker_return_delay, 0);

            robot.setShooter(0, 0, SWPID);
            if (!opModeIsActive()) { return; }
        }
    }

    private void BuildR0Back_IP(Trajectory[] traj) {

        int TIdx = 0;
        // Starting X,Y = -63,-17

        // Drive to first shot
        traj[TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(-12, yLane))
                .build();

        // Turn to shoot 3 power shots, finish pointing in towards the tower
        // or
        // Turn to shoot 3 tower shots

        // Drive to Wobble drop - Back delivery
        traj[TIdx++] = robot.drive.trajectoryBuilder(new Pose2d(traj[TIdx-2].end().getX(),traj[TIdx-2].end().getY(), Math.toRadians(0.0)))
                .splineToSplineHeading(new Pose2d(58,-36, Math.toRadians(-90.0)),Math.toRadians(-90.0))
                .splineToSplineHeading(new Pose2d(58,-44, Math.toRadians(-90.0)),Math.toRadians(-90.0))
                .splineToSplineHeading(new Pose2d(34,-58, Math.toRadians(0.0)),Math.toRadians(180.0))
                .build();

        // Lower arm, drop wobble, return arm

        // Go park
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                .splineToSplineHeading(new Pose2d(40,-58,Math.toRadians(0.0)),Math.toRadians(0.0))
                .splineToSplineHeading(new Pose2d(58,-36,Math.toRadians(90.0)),Math.toRadians(90.0))
                .splineToSplineHeading(new Pose2d(40,-12,Math.toRadians(180.0)),Math.toRadians(180.0))
                .splineToSplineHeading(new Pose2d(-12,-12,Math.toRadians(TOWER_SHOT_ANGLE)),Math.toRadians(180.0))
                .addDisplacementMarker(80, () -> {
                    robot.setShooter(side_high_tower_RPM, high_tower_power, SWPID);
                })
                .build();

        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                .lineToLinearHeading(new Pose2d(12,-12, Math.toRadians(0.0)))
                .build();

        showTrajPoses( "RING0_IP", TIdx, traj ) ;
    }

    private void BuildR1Back_IP(Trajectory[] traj) {

        int TIdx = 0;
        // Starting X,Y = -63,-17

        // Drive to first shot
        traj[TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(-12, yLane))
                .build();

        // Turn to shoot 3 power shots, finish pointing in towards the tower
        // or
        // Turn to shoot 3 tower shots

        // Drive to Wobble drop - Back delivery
        traj[TIdx++] = robot.drive.trajectoryBuilder(new Pose2d(traj[TIdx-2].end().getX(),traj[TIdx-2].end().getY(), Math.toRadians(0.0)))
                .splineToSplineHeading(new Pose2d(56,-30, Math.toRadians(0.0)),Math.toRadians(-90.0))
                .build();

        // Lower arm, drop wobble, return arm

        // Turn to side

        // Go park
        traj[TIdx++] = robot.drive.trajectoryBuilder(new Pose2d(traj[TIdx-2].end().getX(),traj[TIdx-2].end().getY(), Math.toRadians(90.0)))
                .splineToSplineHeading(new Pose2d(44,-12,Math.toRadians(-180.0)),Math.toRadians(-180.0))
                .splineToSplineHeading(new Pose2d(-12,-12,Math.toRadians(TOWER_SHOT_ANGLE)),Math.toRadians(-180.0))
                .addDisplacementMarker(80, () -> {
                    robot.setShooter(side_high_tower_RPM, high_tower_power, SWPID);
                })
                .build();

        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                .lineToLinearHeading(new Pose2d(12,-12,Math.toRadians(0.0)))
                .build();

        showTrajPoses( "RING1_IP", TIdx, traj ) ;
    }
    private void Ring0Back_IP(Trajectory[] traj, boolean wobbleEnabled, boolean powerShots) {
        int TIdx = 0;

        // Start shooter, raise wobble high
        if(powerShots) {
            robot.setShooter(power_shot_RPM, power_shot_power, SWPID);
        } else {
            robot.setShooter(side_high_tower_RPM, high_tower_power, SWPID);
        }
        robot.setWobblePosition(robot.WOBBLE_UP,0.4);

        // Drive to shooting location
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Handle the shooting
        power_tower_shots(powerShots);
        if(!opModeIsActive()){ return; }

        robot.intakePickupRing(1.0);

        // Turn around and small pause
        robot.drive.turnAsync(Math.toRadians(0.0) - robot.drive.getRawExternalHeading());
        CheckWait(true, SWPID, 0, 0);
        CheckWait(true, SWPID, 250, 0);
        if(!opModeIsActive()){ return; }

        // Go to wobble drop
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Lower arm
        robot.setWobblePosition(robot.WOBBLE_DROP,0.5);
        CheckWait(true, SWPID, 1500, 0);
        if(!opModeIsActive()){ return; }

        // Drop wobble
        robot.claw.setPosition(0.5);
        CheckWait(true, SWPID, 250, 0);
        if(!opModeIsActive()){ return; }

        // Return arm
        robot.setWobblePosition(robot.WOBBLE_START,0.5);
        CheckWait(true, SWPID, 500, 0);
        if(!opModeIsActive()){ return; }

        // Close claw
        robot.claw.setPosition(1.0);
        if(!opModeIsActive()){ return; }

        // Go to shot location
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Tower Shot
        robot.flicker.setPosition(1.0);
        CheckWait(true, SWPID, flicker_shot_delay, 0);
        robot.flicker.setPosition(0.0);
        CheckWait(true, SWPID, flicker_return_delay, 0);
        if (!opModeIsActive()) { return; }

        robot.flicker.setPosition(1.0);
        CheckWait(true, SWPID, flicker_shot_delay, 0);
        robot.flicker.setPosition(0.0);
        CheckWait(true, SWPID, flicker_return_delay, 0);
        if (!opModeIsActive()) { return; }

        robot.flicker.setPosition(1.0);
        CheckWait(true, SWPID, flicker_shot_delay, 0);
        robot.flicker.setPosition(0.0);
        CheckWait(true, SWPID, flicker_return_delay, 0);
        if (!opModeIsActive()) { return; }

        robot.intakeStop();
        robot.setShooter(0, 0, SWPID);

        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

    }

    private void Ring1Back_IP(Trajectory[] traj, boolean wobbleEnabled, boolean powerShots) {
        int TIdx = 0;

        // Start shooter, raise wobble high
        if(powerShots) {
            robot.setShooter(power_shot_RPM, power_shot_power, SWPID);
        } else {
            robot.setShooter(side_high_tower_RPM, high_tower_power, SWPID);
        }
        robot.setWobblePosition(robot.WOBBLE_UP,0.4);

        // Drive to shooting location
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Handle the shooting
        power_tower_shots(powerShots);
        if(!opModeIsActive()){ return; }

        robot.intakePickupRing(1.0);

        // Turn around and small pause
        robot.drive.turnAsync(Math.toRadians(0.0) - robot.drive.getRawExternalHeading());
        CheckWait(true, SWPID, 0, 0);
        CheckWait(true, SWPID, 250, 0);
        if(!opModeIsActive()){ return; }

        // Go to wobble drop
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Lower arm
        robot.setWobblePosition(robot.WOBBLE_DROP,0.5);
        CheckWait(true, SWPID, 1500, 0);
        if(!opModeIsActive()){ return; }

        // Drop wobble
        robot.claw.setPosition(0.5);
        CheckWait(true, SWPID, 250, 0);
        if(!opModeIsActive()){ return; }

        // Return arm
        robot.setWobblePosition(robot.WOBBLE_START,0.5);
        CheckWait(true, SWPID, 500, 0);
        if(!opModeIsActive()){ return; }

        // Close claw
        robot.claw.setPosition(1.0);
        if(!opModeIsActive()){ return; }

        // Turn to side
        robot.drive.turnAsync(Math.toRadians(90.0) - robot.drive.getRawExternalHeading());
        CheckWait(true, SWPID, 0, 0);

        // Go to shot location
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Tower Shot
        robot.flicker.setPosition(1.0);
        CheckWait(true, SWPID, flicker_shot_delay, 0);
        robot.flicker.setPosition(0.0);
        CheckWait(true, SWPID, flicker_return_delay, 0);
        if (!opModeIsActive()) { return; }

        robot.flicker.setPosition(1.0);
        CheckWait(true, SWPID, flicker_shot_delay, 0);
        robot.flicker.setPosition(0.0);
        CheckWait(true, SWPID, flicker_return_delay, 0);
        if (!opModeIsActive()) { return; }

        robot.flicker.setPosition(1.0);
        CheckWait(true, SWPID, flicker_shot_delay, 0);
        robot.flicker.setPosition(0.0);
        CheckWait(true, SWPID, flicker_return_delay, 0);
        if (!opModeIsActive()) { return; }

        robot.intakeStop();
        robot.setShooter(0, 0, SWPID);

        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

    }
}
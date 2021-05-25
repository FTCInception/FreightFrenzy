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

@Autonomous(name="Blue_Outside_IP_Scrimmage", group="MechBot")
public class Blue_Outside_IP_Scrimmage extends LinearOpMode {
    private final int RING0_IP=0;
    private final int RING1_IP=1;
    private final int RING4_IP=2;
    private final int ANGLE_TEST = 3;

    private double RING0_TURN1, RING0_TURN2;
    private double RING1_TURN1, RING1_TURN2, RING1_TURN3;
    private double RING4_TURN1, RING4_TURN2, RING4_TURN3, RING4_TURN4;
    private double RINGP0_TURN1, RINGP0_TURN2, RINGP0_TURN3;
    private double RINGP1_TURN1, RINGP1_TURN2, RINGP1_TURN3, RINGP1_TURN4;

    private double RING0_IP_TURN1, RING0_IP_TURN2, RING0_IP_TURN3, RING0_IP_TURN4;
    private double RING1_IP_TURN1, RING1_IP_TURN2, RING1_IP_TURN3, RING1_IP_TURN4;
    private double RING4_IP_TURN1, RING4_IP_TURN2, RING4_IP_TURN3, RING4_IP_TURN4;


    //private double POWER_SHOT_ANGLE = 25.25;
    private double POWER_SHOT_ANGLE = 26.50;
    private double POWER_SHOT2_TURN = 5.75;
    private double POWER_SHOT_ANGLE_RING4 = 28.25;

    private double POWER_SHOT_ANGLE1 = -28.5;
    private double POWER_SHOT_ANGLE2 = -23.00;
    private double TOWER_SHOT_ANGLE = -12.00;

    private RRMechBot robot = new RRMechBot();

    private static final double wobble_power = 0.6;
    private String className = this.getClass().getSimpleName().toLowerCase();

    private static final double intake_eject_wobble = 0.75;
    private static final double intake_pickup_ring = 1.0;

    // These are for the REV HUB...
    private static final double high_tower_power = 0.4775;
    private static final double long_shot_boost = 0.000;
    private static final double power_shot_power = 0.435;
    private static final double power_offset = 0.003;

    // Stealth Wheel blue RPMs 3/26/21
    // These are for the SW PID...
    private static final double high_tower_RPM = 3575;
    private static final double long_shot_RPM_boost = -75;
    private static final double long_shot_RPM = 3500;
    private static final double side_high_tower_RPM = 3450;
    private static final double power_shot_RPM = 3175;
    private static final double power_RPM_offset = 0;

    /*
    private static final double high_tower_RPM = 0;
    private static final double long_shot_RPM_boost = 0;
    private static final double power_shot_RPM = 0;
    private static final double power_RPM_offset = 0;
    */

    // BaneBot Blue RPMs:
    //private static final double high_tower_RPM = 6200;
    //private static final double long_shot_RPM_boost = 0;
    //private static final double long_shot_RPM_boost = 0;
    //private static final double power_shot_RPM = 4950;
    //private static final double power_RPM_offset = 0;

    private static final double flicker_shot_delay = 250;
    private static final double flicker_return_delay = 350;

    // This is a one-stop switchover from SWPID back to REV HUB PID
    // The setShooter function consumes this flag and will switch back and forth as needed.
    private static boolean SWPID = true;

    // This is the starting position of the center of the robot.
    private static final double startingX = -63.0;
    private static final double startingY = 57.0;

    // This is relative position of center of the robot compared to the claw for second wobble pickup
    // Adjusting this can change the chance of teammate/claw interference
    private static final double WxOffset = 9.0;
    private static final double WyOffset = 17.0;

    private static boolean wobbleEnabled = true;
    private static boolean powerShots = false;
    private static boolean middleLanePark = true;
    private static boolean starterStack = true;
    private static boolean MKDrop = false;

    private IncepVision vision = new IncepVision();
    private int ringCount = -1;
    private Trajectory[][] trajs = {new Trajectory[25], new Trajectory[25], new Trajectory[25], new Trajectory[25]};

    @Override
    public void runOpMode() {
        //robot.logger.LOGLEVEL = robot.logger.LOGDEBUG ;

        // Init the robot and subsystems.  Set Roadrunner timeout to 0.25s to save some time.
        robot.init(hardwareMap,0.05);
        robot.initAutonomous(this);
        robot.logger.LOGLEVEL |= robot.logger.LOGDEBUG;

        // Robot center is 9" from each edge:
        // Back against the -x wall, wheels aligned on first tile in -y
        // THIS MUST BE DONE BEFORE BUILDING
        // THIS MUST BE DONE AFTER THE ROBOT IS IN ITS FINAL POSITION
        Pose2d startPose = new Pose2d(startingX, startingY, Math.toRadians(0));
        robot.drive.setPoseEstimate(startPose);

        double Wx = -48.0;
        double Wy = 24.0;
        boolean leftOK = true, rightOK = true, upOK = true, downOK = true, bOK = true, xOK = true, yOK = true;
        do {
            if ( gamepad2.dpad_left ) {
                if (leftOK) {
                    Wx += 1.0;
                    leftOK = false;
                }
            } else {
                leftOK = true;
            }
            if ( gamepad2.dpad_right ) {
                if (rightOK) {
                    Wx -= 1.0;
                    rightOK = false;
                }
            } else {
                rightOK = true;
            }
            if ( gamepad2.dpad_down ) {
                if (downOK) {
                    Wy += 1.0;
                    downOK = false;
                }
            } else {
                downOK = true;
            }
            if ( gamepad2.dpad_up ) {
                if (upOK) {
                    Wy -= 1.0;
                    upOK = false;
                }
            } else {
                upOK = true;
            }
            if (gamepad2.y) {
                if (yOK) {
                    middleLanePark = !(middleLanePark);
                    yOK = false;
                }
            } else {
                yOK = true;
            }
            if ( gamepad2.b ) {
                if (bOK) {
                    powerShots = !(powerShots);
                    bOK = false;
                }
            } else {
                bOK = true;
            }
            if ( gamepad2.x ) {
                if (xOK) {
                    if (wobbleEnabled && starterStack) {
                        wobbleEnabled = false;
                    } else if (!wobbleEnabled && starterStack) {
                        starterStack = false;
                    } else {
                        wobbleEnabled = true;
                        starterStack = true;
                    }
                    xOK = false;
                }
            } else {
                xOK = true;
            }
            if ( gamepad2.a ) {
                break;
            }

            telemetry.addData("Dpad Wx,Wy:", "(%.0f, %.0f) --> (%.0f, %.0f); Delta:(%.0f%s, %.0f%s)", -48.0, 24.0, Wx, Wy, Math.abs(-48 - Wx), (Wx > -48) ? " left" : (Wx < -48) ? " right" : "", Math.abs(24 - Wy), (Wy > 24) ? " down" : (Wy < 24) ? " up" : "");
            telemetry.addData("'Y' for 0-ring park lane:", "(%s)", middleLanePark ? "second" : "third");
            telemetry.addData("'X' for wobble/starter stack:", "(%s)", wobbleEnabled ? (starterStack ? "wobble + stack" : "illegal") : (starterStack ? "no wobble" : "neither"));
            telemetry.addData("'B' to toggle 2x power shots vs 3x tower shots:", "(%s)", powerShots ? "power" : "tower");
            telemetry.addData("'A' to proceed to vision", "");
            telemetry.update();

        } while (!isStarted() && (!isStopRequested())) ;

        telemetry.addData("Computing paths", "");
        telemetry.update();
        BuildR0_IP(trajs[RING0_IP], Wx, Wy, wobbleEnabled, powerShots, starterStack);
        BuildR1_IP(trajs[RING1_IP], Wx, Wy, wobbleEnabled, powerShots, starterStack);
        BuildR4_IP(trajs[RING4_IP], Wx, Wy, wobbleEnabled, powerShots, starterStack);
        //BuildAngleTest(trajs[ANGLE_TEST], Wx, Wy, wobbleEnabled, powerShots);

        telemetry.addData("Starting vision","");
        telemetry.update();
        // Stare at the rings really hard until its time to go or stop
        vision.initAutonomous(this, "RightWebcam", vision.BLUE_OUTSIDE);
        vision.clip = false;
        boolean leftBOK = true, rightBOK = true;
        do {
            ringCount = vision.countRings();
            vision.manageVisionBox(gamepad2);
        } while (!isStarted() && (!isStopRequested()));
        vision.shutdown();

        //AngleTest(trajs[ANGLE_TEST], wobbleEnabled, powerShots);

        // FIXME: Test the stopping in auto here again.
        if (opModeIsActive()) {
            if (ringCount == 0) {
                Ring0_IP(trajs[RING0_IP], wobbleEnabled, powerShots, starterStack);
            } else if (ringCount == 1) {
                Ring1_IP(trajs[RING1_IP], wobbleEnabled, powerShots, starterStack);
            } else {
                // If it's not 0 or 1, assume 4 (highest possible point total)
                Ring4_IP(trajs[RING4_IP], wobbleEnabled, powerShots, starterStack);
            }
        }
    }

    private void showTrajPoses(String trajName, int TIdx, Trajectory[] traj) {

        Pose2d tmpPose = traj[0].start();
        double time = 0;
        robot.logger.logD("showTrajPoses:", String.format("%s, Idx:%d, X: %.2f, Y:%.2f, H:%.2f, t:%.2f", trajName, -1, tmpPose.getX(), tmpPose.getY(), Math.toDegrees(tmpPose.getHeading()), time));

        for (int i=0; i<TIdx; i++) {
            tmpPose = traj[i].end();
            time += traj[i].duration();
            robot.logger.logD("showTrajPoses:", String.format("%s, Idx:%d, X: %.2f, Y:%.2f, H:%.2f, t:%.2f", trajName, i, tmpPose.getX(), tmpPose.getY(), Math.toDegrees(tmpPose.getHeading()), time));
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

        while (opModeIsActive() ) {
            // Get the time
            now = localClock.seconds();

            // Master stop
            if (!opModeIsActive()) { return; }

            // Update the drive
            if (checkDrive) { robot.drive.update(); }

            // Update the shooterPID
            if (runShooterPID) { robot.updateShooterPID(); }

            // Check timer expiration, bail if too long
            if (maxMS < now) { return; }

            // Make sure to wait for the minimum time
            if (minMS > now) { continue; }

            // Drive still running? Wait for it.
            if (checkDrive) {
                if (robot.drive.isBusy()) { continue; }
            }

            // No reason to be here (past the minMS timer, drive is idle)
            return;
        }
    }

    private void BuildAngleTest(Trajectory[] traj, double Wx, double Wy, boolean wobbleEnabled, boolean powerShots) {

        int TIdx = 0;
        // Starting X,Y = -63,57

        // Drive to first shot
        traj[TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(-8,57, Math.toRadians(powerShots?(POWER_SHOT_ANGLE1):(TOWER_SHOT_ANGLE))), Math.toRadians(0.0))
                .build();

        traj[TIdx++] = robot.drive.trajectoryBuilder(new Pose2d(traj[TIdx-2].end().getX(),traj[TIdx-2].end().getY(), Math.toRadians(powerShots?(TOWER_SHOT_ANGLE):(TOWER_SHOT_ANGLE))), true)
                .splineToLinearHeading(new Pose2d(-62,57, Math.toRadians(0.0)), Math.toRadians(180.0))
                .build();
    }

    private void AngleTest(Trajectory[] traj, boolean wobbleEnabled, boolean powerShots) {
        int TIdx = 0;

        // Go to shooting location
        if(powerShots) {
            robot.setShooter(power_shot_RPM, power_shot_power, SWPID);
        } else {
            robot.setShooter(side_high_tower_RPM, high_tower_power, SWPID);
        }
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        power_tower_shots(powerShots);
        if(!opModeIsActive()){ return; }

        // Go back
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
    }

    private void BuildStraightTest(Trajectory[] traj, double Wx, double Wy, boolean wobbleEnabled, boolean powerShots) {

        int TIdx = 0;

        traj[TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .forward(96)
                .build();
    }

    private void StraightTest(Trajectory[] traj, boolean wobbleEnabled, boolean powerShots) {
        int TIdx = 0;

        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
    }

    private void BuildR0_IP(Trajectory[] traj, double Wx, double Wy, boolean wobbleEnabled, boolean powerShots, boolean starterStack) {

        int TIdx = 0;
        // Starting X,Y = -63,-57

        // Pose: -8, -57, 19
        // Drive to first shot
        traj[TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(-8,57, Math.toRadians(powerShots?(POWER_SHOT_ANGLE1):(TOWER_SHOT_ANGLE))), Math.toRadians(0.0))
                .build();
        // shoot #1

        // Pose: TBD
        // Turn based on heading
        //robot.drive.turnAsync(Math.toRadians(Math.toRadians(POWER_SHOT_ANGLE2) - robot.drive.getRawExternalHeading()));
        // shoot #2

        // Pose: TBD
        // Turn based on heading
        //robot.drive.turnAsync(Math.toRadians(Math.toRadians(TOWER_SHOT_ANGLE) - robot.drive.getRawExternalHeading()));
        // shoot #3

        // Pose: TBD
        if(!wobbleEnabled) {
            // 0-Ring wobble dropoff
            traj[TIdx++] = robot.drive.trajectoryBuilder(new Pose2d(traj[TIdx - 2].end().getX(), traj[TIdx - 2].end().getY(), Math.toRadians(TOWER_SHOT_ANGLE)))
                    .lineToLinearHeading(new Pose2d(-6, 59, Math.toRadians(0.0)))   // Tuned value to work with MegaKnytes
                    .build();

            // Drop wobble
        } else {
            // 0-Ring wobble dropoff
            traj[TIdx++] = robot.drive.trajectoryBuilder(new Pose2d(traj[TIdx - 2].end().getX(), traj[TIdx - 2].end().getY(), Math.toRadians(TOWER_SHOT_ANGLE)))
                    .lineToLinearHeading(new Pose2d(2,57, Math.toRadians(0.0)))     // Original value
                    .build();

            // Drop wobble

            // Line up on 2nd wobble
            traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end(), true)
                    .lineToLinearHeading(new Pose2d(-38, 57, Math.toRadians(-315.0)))
                    .addDisplacementMarker(0.05, 0, () -> {
                        if (wobbleEnabled) {
                            robot.setWobblePosition(robot.WOBBLE_PICKUP, 0.4);
                            robot.claw.setPosition(0.5);
                        }
                    })
                    .build();

            // Prep Wobble arm and claw
            // Backup to Wobble
            // The 0-ring case has a little bit less drift, we make a slight adjustment here.
            traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end(), true)
                    .lineToConstantHeading(new Vector2d(Wx + WxOffset - 0.5, Wy + WyOffset - 0.5))
                    .build();

            // Grab wobble

            // Drive away before lifting
            traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                    .lineToConstantHeading(new Vector2d(traj[TIdx - 2].end().getX(), traj[TIdx - 2].end().getY() + 4.0))
                    .addDisplacementMarker(0.50, 0, () -> {
                        robot.setWobblePosition(robot.WOBBLE_UP, 0.8);
                    })
                    .build();

            // Go to lane
            // Pose: Wx, -57, 0
            traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                    .lineToLinearHeading(new Pose2d(Wx + WxOffset, 57, Math.toRadians(-180.0)))
                    .build();

            // Back up
            traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end(), true)
                    .lineToLinearHeading(new Pose2d(-12, 60, Math.toRadians(-190.0)))
                    .build();

            // Lower wobble
            // Drop wobble
        }

        // Drive away and wait
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                .lineToLinearHeading(new Pose2d(-48, 60, Math.toRadians(wobbleEnabled?(-180.0):(0.0))))
                .addDisplacementMarker(0.25, 0, () -> {
                    // Return arm
                    robot.setWobblePosition(robot.WOBBLE_START, 0.4);
                    robot.claw.setPosition(1.0);
                })
                .build();

        if(middleLanePark) {
            // Strafe and park
            traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                    .lineToConstantHeading(new Vector2d(-24, 36))
                    .build();

            traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                    .lineToConstantHeading(new Vector2d(6, 38))
                    .build();
        } else {
            // Strafe and park
            traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                    .lineToConstantHeading(new Vector2d(-24, 12))
                    .build();
            traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                    .lineToConstantHeading(new Vector2d(6, 12))
                    .build();
        }

        showTrajPoses( "RING0_IP", TIdx, traj ) ;
    }

    private void Ring0_IP(Trajectory[] traj, boolean wobbleEnabled, boolean powerShots, boolean starterStack) {
        int TIdx = 0;

        // We have extra time if not picking up wobble, lets wait here a while
        if(!wobbleEnabled) {
            robot.claw.setPosition(0.95);
            CheckWait(true, SWPID, 1000, 0);
            robot.claw.setPosition(0.9);
            CheckWait(true, SWPID, 1000, 0);
            robot.claw.setPosition(0.95);
            CheckWait(true, SWPID, 1000, 0);
            robot.claw.setPosition(0.90);
            CheckWait(true, SWPID, 1000, 0);
            robot.claw.setPosition(0.95);
            CheckWait(true, SWPID, 1000, 0);
            robot.claw.setPosition(0.9);
            CheckWait(true, SWPID, 1000, 0);
            robot.claw.setPosition(0.95);
            CheckWait(true, SWPID, 1000, 0);
            robot.claw.setPosition(1.0);
            CheckWait(true, SWPID, 1000, 0);
        }

        // Go to shooting location
        if(powerShots) {
            robot.setShooter(power_shot_RPM, power_shot_power, SWPID);
        } else {
            robot.setShooter(side_high_tower_RPM, high_tower_power, SWPID);
        }
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        power_tower_shots(powerShots);
        if(!opModeIsActive()){ return; }

        // Go to wobble drop
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);

        // Drop wobble
        robot.intakeEjectWobble(intake_eject_wobble);
        CheckWait(true, SWPID, 750, 0);
        if(!opModeIsActive()){ return; }

        robot.intakeStop();
        if(!opModeIsActive()){ return; }

        if(wobbleEnabled) {

            // Back up to wobble alignment
            robot.drive.followTrajectoryAsync(traj[TIdx++]);
            CheckWait(true, SWPID, 0, 0);
            if(!opModeIsActive()){ return; }

            // Wait on wobble arm
            CheckWait(true, SWPID, 500, 0);

            robot.drive.followTrajectoryAsync(traj[TIdx++]);
            CheckWait(true, SWPID, 0, 0);
            if (!opModeIsActive()) { return; }

            // Pick up wobble
            robot.claw.setPosition(1);
            CheckWait(true, SWPID, 750, 0);

            // Drive away before lifting
            robot.drive.followTrajectoryAsync(traj[TIdx++]);
            CheckWait(true, SWPID, 0, 0);

            // Return to lane
            robot.drive.followTrajectoryAsync(traj[TIdx++]);
            CheckWait(true, SWPID, 0, 0);
            if (!opModeIsActive()) { return; }

            // Back wobble in
            robot.drive.followTrajectoryAsync(traj[TIdx++]);
            CheckWait(true, SWPID, 0, 0);
            if (!opModeIsActive()) { return; }

            // Lower wobble
            robot.setWobblePosition(robot.WOBBLE_DROP, 0.3);
            CheckWait(true, SWPID, 2000, 0);

            // Drop wobble
            robot.claw.setPosition(0.5);
            CheckWait(true, SWPID, 500, 0);
            if (!opModeIsActive()) { return; }
        }

        // Move away from wobble
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);

        // Wait for arm (minimum of 2 seconds and also time this to delay final motion to the buzzer)
        CheckWait(true, SWPID, (wobbleEnabled?3000:5000), 0);
        if (!opModeIsActive()) { return; }

        // Drive partway to park
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);

        // Back over park line
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);

    }

    private void BuildR1_IP(Trajectory[] traj, double Wx, double Wy, boolean wobbleEnabled, boolean powerShots, boolean starterStack) {

        int TIdx = 0;
        // Starting X,Y = -63,57

        // Pose: -8, 57, 19
        // Drive to first shot
        traj[TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(-8,57, Math.toRadians(powerShots?(POWER_SHOT_ANGLE1):(TOWER_SHOT_ANGLE))), Math.toRadians(0.0))
                .build();
        // shoot #1

        // Pose: TBD
        // Turn based on heading
        //robot.drive.turnAsync(Math.toRadians(Math.toRadians(POWER_SHOT_ANGLE2) - robot.drive.getRawExternalHeading()));
        // shoot #2

        // Pose: TBD
        // Turn based on heading
        //robot.drive.turnAsync(Math.toRadians(Math.toRadians(TOWER_SHOT_ANGLE) - robot.drive.getRawExternalHeading()));
        // shoot #3

        // Drive to wobble drop

        // 1-Ring
        traj[TIdx++] = robot.drive.trajectoryBuilder(new Pose2d(traj[TIdx-2].end().getX(),traj[TIdx-2].end().getY(), Math.toRadians(TOWER_SHOT_ANGLE)))
                .splineToLinearHeading(new Pose2d(36,54, Math.toRadians(-90.0)),Math.toRadians(-40.0))
                .build();

        // Drop wobble

        if ( !wobbleEnabled && !starterStack ) {

            // Park if nothing else to do
            traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end(), true)
                    .lineToLinearHeading(new Pose2d(6, 60, Math.toRadians(0.0)))
                    .build();
        } else {

            // Line up for wobble or starterStack
            traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end(), true)
                    .lineToLinearHeading(new Pose2d(-38, 57, Math.toRadians(-315.0)))
                    .addDisplacementMarker(0.50, 0, () -> {
                        if (wobbleEnabled) {
                            robot.setWobblePosition(robot.WOBBLE_PICKUP, 0.4);
                        }
                    })
                    .addDisplacementMarker(0.60, 0, () -> {
                        if (wobbleEnabled) {
                            robot.claw.setPosition(0.5);
                        }
                    })
                    .build();

            // Prep Wobble arm and claw
            // Backup to Wobble
            if (wobbleEnabled) {
                traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end(), true)
                        .lineToConstantHeading(new Vector2d(Wx + WxOffset, Wy + WyOffset))
                        .build();

                // Grab wobble

                // Drive away before lifting
                traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                        .lineToConstantHeading(new Vector2d(traj[TIdx - 2].end().getX(), traj[TIdx - 2].end().getY() + 4.0))
                        .addDisplacementMarker(0.50, 0, () -> {
                            robot.setWobblePosition(robot.WOBBLE_UP, 0.8);
                        })
                        .build();
            }

            // Line up on ring
            traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                    .lineToConstantHeading(new Vector2d(-42.1, 37.1))
                    .build();

            // Turn on intake

            // Turn to face heading 0
            // robot.drive.turnAsync(-robot.drive.getRawExternalHeading());

            // Drive over ring
            // Pose: -7, -38, 0
            traj[TIdx++] = robot.drive.trajectoryBuilder(new Pose2d(traj[TIdx - 2].end().getX(), traj[TIdx - 2].end().getY(), Math.toRadians(0.0)))
                    .lineToLinearHeading(new Pose2d(-7, 35, Math.toRadians(3.5)))
                    .build();

            // Shoot

            if (wobbleEnabled) {
                // Turn to 170 degress
                //robot.drive.turnAsync(Math.toRadians(170.0) - robot.drive.getRawExternalHeading());

                // Lower wobble

                // Back up
                traj[TIdx++] = robot.drive.trajectoryBuilder(new Pose2d(traj[TIdx - 2].end().getX(), traj[TIdx - 2].end().getY(), Math.toRadians(170.0)), true)
                        .lineToConstantHeading(new Vector2d(12, 39))
                        .build();

                // Drop wobble

                // Drive away a little
                traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                        .lineToConstantHeading(new Vector2d(8, 39))
                        .build();

                // Return wobble arm
            } else {
                // Park if no wobble
                traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                        .lineToLinearHeading(new Pose2d(6, 60, Math.toRadians(0.0)))
                        .build();
            }
        }

        showTrajPoses( "RING1_IP", TIdx, traj );
    }

    private void Ring1_IP(Trajectory[] traj, boolean wobbleEnabled, boolean powerShots, boolean starterStack) {
        int TIdx = 0;

        // If we are not doing wobble or starter stack, just wait a little while here.
        if(!wobbleEnabled && !starterStack) {
            robot.claw.setPosition(0.95);
            CheckWait(true, SWPID, 1000, 0);
            robot.claw.setPosition(0.9);
            CheckWait(true, SWPID, 1000, 0);
            robot.claw.setPosition(0.95);
            CheckWait(true, SWPID, 1000, 0);
            robot.claw.setPosition(0.90);
            CheckWait(true, SWPID, 1000, 0);
            robot.claw.setPosition(0.95);
            CheckWait(true, SWPID, 1000, 0);
            robot.claw.setPosition(0.9);
            CheckWait(true, SWPID, 1000, 0);
            robot.claw.setPosition(0.95);
            CheckWait(true, SWPID, 1000, 0);
            robot.claw.setPosition(1.0);
            CheckWait(true, SWPID, 1000, 0);
        }

        // Go to shooting location
        if(powerShots) {
            robot.setShooter(power_shot_RPM, power_shot_power, SWPID);
        } else {
            robot.setShooter(side_high_tower_RPM, high_tower_power, SWPID);
        }
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        power_tower_shots(powerShots);
        if(!opModeIsActive()){ return; }

        // Go to wobble drop
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);

        // Drop wobble
        robot.intakeEjectWobble(intake_eject_wobble);
        CheckWait(true, SWPID, 750, 0);
        if(!opModeIsActive()){ return; }

        robot.intakeStop();
        if(!opModeIsActive()){ return; }

        // Park or line up on wobble
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        if(wobbleEnabled || starterStack) {
            if(wobbleEnabled) {
                robot.drive.followTrajectoryAsync(traj[TIdx++]);
                CheckWait(true, SWPID, 0, 0);
                if (!opModeIsActive()) { return; }

                // Pick up wobble
                robot.claw.setPosition(1);
                CheckWait(true, SWPID, 750, 0);

                // Drive away before lifting
                robot.drive.followTrajectoryAsync(traj[TIdx++]);
                CheckWait(true, SWPID, 0, 0);
                CheckWait(true, SWPID, 500, 0);

            }

            // Turn on intake
            robot.intakePickupRing(intake_pickup_ring);
            if(!opModeIsActive()){ return; }

            // Lineup on ring
            robot.drive.followTrajectoryAsync(traj[TIdx++]);
            CheckWait(true, SWPID, 0, 0);
            if(!opModeIsActive()){ return; }

            robot.drive.turnAsync(-robot.drive.getRawExternalHeading());
            CheckWait(true, SWPID, 0, 0);

            // Turn on shooter
            robot.setShooter(high_tower_RPM, high_tower_power, SWPID);

            // Drive over ring
            robot.drive.followTrajectoryAsync(traj[TIdx++]);
            CheckWait(true, SWPID, 0, 0);
            if(!opModeIsActive()){ return; }

            // Stop intake
            robot.intakeStop();

            // 2 shots
            robot.flicker.setPosition(1.0);
            CheckWait(true, SWPID, flicker_shot_delay, 0);
            if(!opModeIsActive()){ return; }

            robot.flicker.setPosition(0.0);
            CheckWait(true, SWPID, flicker_return_delay, 0);
            if(!opModeIsActive()){ return; }

            robot.flicker.setPosition(1.0);
            CheckWait(true, SWPID, flicker_shot_delay, 0);
            if(!opModeIsActive()){ return; }

            // Wait before turning motor down
            robot.setShooter(0, 0, SWPID);

            robot.flicker.setPosition(0.0);
            if(!opModeIsActive()){ return; }

            if (wobbleEnabled) {
                // Turn to 180 degrees COUNTER CLOCKWISE to stay in lane
                robot.drive.turnAsync(Math.toRadians(170.0) - robot.drive.getRawExternalHeading());
                CheckWait(true, SWPID, 1000, 1000);
                robot.setWobblePosition(robot.WOBBLE_DROP, 0.3);
                CheckWait(true, SWPID, 1000, 0);
                if (!opModeIsActive()) { return; }

                // Back wobble in
                robot.drive.followTrajectoryAsync(traj[TIdx++]);
                CheckWait(true, SWPID, 0, 0);

                // Drop wobble
                robot.claw.setPosition(0);
                CheckWait(true, SWPID, 500, 0);
                if (!opModeIsActive()) { return; }

                // Move away from wobble
                robot.drive.followTrajectoryAsync(traj[TIdx++]);
                CheckWait(true, SWPID, 0, 0);

                // Return arm
                robot.setWobblePosition(robot.WOBBLE_START, 0.4);
                CheckWait(true, SWPID, 500, 0);
                robot.claw.setPosition(1.0);
                CheckWait(true, SWPID, 2000, 0);
                if (!opModeIsActive()) { return; }
            }  else {
                // Go park next to wall
                robot.drive.followTrajectoryAsync(traj[TIdx++]);
                CheckWait(true, SWPID, 0, 0);
                if (!opModeIsActive()) { return; }
            }
        }
    }

    private void BuildR4_IP(Trajectory[] traj, double Wx, double Wy, boolean wobbleEnabled, boolean powerShots, boolean starterStack) {

        int TIdx = 0;
        // Starting X,Y = -63,57

        // Drive to wobble drop
        if(!MKDrop || wobbleEnabled) {
            // This is the new location
            // We may override to the old location below if we're playing with MK but ONLY if we're not doing both wobbles.
            // We are in a hurry if we are getting both wobbles and we want to put the wobble in the corner
            traj[TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .splineToSplineHeading(new Pose2d(0, 57, Math.toRadians(0.0)), Math.toRadians(0.0))
                    .splineToSplineHeading(new Pose2d(52, 55, Math.toRadians(45.0)), Math.toRadians(0.0))
                    .addDisplacementMarker(0.95, 0, () -> {
                        robot.intakeEjectWobble(intake_eject_wobble);
                    })
                    .build();
        } else {
            // This is the original location practiced with MK
            // We hope they can work with the new back corner drop, but set MKDrop to true and then signle-wobble selection
            // to get the old location back.
            traj[TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(48, 57, Math.toRadians(0.0)), Math.toRadians(0.0))
                    .build();
        }

        // Drop wobble

        // Drive to shot
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end(), true)
                .lineToLinearHeading(new Pose2d(-8,57, Math.toRadians(powerShots?(POWER_SHOT_ANGLE1):(TOWER_SHOT_ANGLE))))
                .addDisplacementMarker(0.15, 0, () -> {
                    robot.intakeStop();
                })
                .build();

        // Shoot

        if ( !wobbleEnabled && !starterStack ) {

            // Go Park if needed
            traj[TIdx++] = robot.drive.trajectoryBuilder(new Pose2d(traj[TIdx-2].end().getX(),traj[TIdx-2].end().getY(), Math.toRadians(TOWER_SHOT_ANGLE)))
                    .lineToLinearHeading(new Pose2d(6, 60, Math.toRadians(0.0)))
                    .build();

        } else {

            // Line up for wobble or starterStack
            traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end(), true)
                    .lineToLinearHeading(new Pose2d(-38, 57, Math.toRadians(-315.0)))
                    .addDisplacementMarker(0.05, 0, () -> {
                        if (wobbleEnabled) {
                            robot.setWobblePosition(robot.WOBBLE_PICKUP, 0.4);
                            robot.claw.setPosition(0.5);
                        }
                    })
                    .build();

            // Prep Wobble arm and claw
            // Backup to Wobble
            if (wobbleEnabled) {
                traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end(), true)
                        .lineToConstantHeading(new Vector2d(Wx + WxOffset - 2.0, Wy + WyOffset - 2.0))
                        .build();

                // Grab wobble

                // Drive away before lifting
                traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                        .lineToConstantHeading(new Vector2d(traj[TIdx - 2].end().getX(), traj[TIdx - 2].end().getY() + 4.0))
                        .addDisplacementMarker(0.50, 0, () -> {
                            robot.setWobblePosition(robot.WOBBLE_UP, 0.8);
                        })
                        .build();
            }

            // Line up on ring
            traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                    .lineToConstantHeading(new Vector2d(-42.1, 34.1))
                    .build();

            // Turn on intake

            // Turn to face heading 0
            // robot.drive.turnAsync(-robot.drive.getRawExternalHeading());

            // Drive over 1-2 rings
            traj[TIdx++] = robot.drive.trajectoryBuilder(new Pose2d(traj[TIdx - 2].end().getX(), traj[TIdx - 2].end().getY(), Math.toRadians(0.0)))
                    .lineToLinearHeading(new Pose2d(-24, 32, Math.toRadians(2.5)))
                    .build();

            // Back up to let any weird rings fall over
            traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                    .back(2.0)
                    .build();

            // Shoot

            // Drive over more rings
            traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                    .lineToLinearHeading(new Pose2d(-7, 34, Math.toRadians(3.5)))
                    .build();

            // Shoot

            if (wobbleEnabled) {

                // Drive to wobble and lower
                traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                        .lineToLinearHeading(new Pose2d(39, 56, Math.toRadians(160.0)))
                        .addDisplacementMarker(0.50, 0, () -> {
                            robot.setWobblePosition(robot.WOBBLE_DROP, 0.5);
                        })
                        .build();

                // Drop wobble

                // Drive away and park
                traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                        .lineToLinearHeading(new Pose2d(8, 57, Math.toRadians(-180.0)))
                        .addDisplacementMarker(.20, 0, () -> {
                            robot.setWobblePosition(robot.WOBBLE_START, 0.6);
                            robot.claw.setPosition(1.0);
                        })
                        .build();

                // Return wobble arm

            } else {
                traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                        .lineToLinearHeading(new Pose2d(6, 60, Math.toRadians(0.0)))
                        .build();
            }
        }

        showTrajPoses( "RING4_IP", TIdx, traj ) ;
    }

    private void Ring4_IP(Trajectory[] traj, boolean wobbleEnabled, boolean powerShots, boolean starterStack) {
        int TIdx = 0;

        // Go to wobble drop
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);

        if(MKDrop && !wobbleEnabled) {
            // Drop wobble manually and wait a little if we are playing with the old MK position and
            // are only doing a single wobble.
            robot.intakeEjectWobble(intake_eject_wobble);
            CheckWait(true, SWPID, 750, 0);
            if(!opModeIsActive()){ return; }
        }

        // Drive to shot location
        if(powerShots) {
            robot.setShooter(power_shot_RPM, power_shot_power, SWPID);
        } else {
            robot.setShooter(side_high_tower_RPM, high_tower_power, SWPID);
        }
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        power_tower_shots(powerShots);
        if(!opModeIsActive()){ return; }

        // Park or line up on wobble
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        if(wobbleEnabled || starterStack) {
            if(wobbleEnabled) {
                robot.drive.followTrajectoryAsync(traj[TIdx++]);
                CheckWait(true, SWPID, 0, 0);
                if (!opModeIsActive()) { return; }

                // Pick up wobble
                robot.claw.setPosition(1);
                CheckWait(true, SWPID, 500, 0);

                // Drive away before lifting
                robot.drive.followTrajectoryAsync(traj[TIdx++]);
                CheckWait(true, SWPID, 0, 0);

                // Seems extra
                //CheckWait(true, SWPID, 500, 0);
            }

            // Turn on intake
            robot.intakePickupRing(intake_pickup_ring);
            if(!opModeIsActive()){ return; }

            // Lineup on ring
            robot.drive.followTrajectoryAsync(traj[TIdx++]);
            CheckWait(true, SWPID, 0, 0);
            if(!opModeIsActive()){ return; }

            robot.drive.turnAsync(-robot.drive.getRawExternalHeading());
            CheckWait(true, SWPID, 0, 0);

            // Turn on shooter
            robot.setShooter(long_shot_RPM, high_tower_power, SWPID);

            // Drive over 1-2 rings
            robot.drive.followTrajectoryAsync(traj[TIdx++]);
            CheckWait(true, SWPID, 0, 0);
            if(!opModeIsActive()){ return; }

            // Backup, but don't wait for it here, shoot through motion
            robot.drive.followTrajectoryAsync(traj[TIdx++]);

            // 3 shots
            robot.flicker.setPosition(1.0);
            CheckWait(true, SWPID, flicker_shot_delay, flicker_shot_delay);
            if(!opModeIsActive()){ return; }

            robot.flicker.setPosition(0.0);
            CheckWait(true, SWPID, flicker_return_delay, flicker_return_delay);
            if(!opModeIsActive()){ return; }

            robot.flicker.setPosition(1.0);
            CheckWait(true, SWPID, flicker_shot_delay, flicker_shot_delay);
            if(!opModeIsActive()){ return; }

            robot.flicker.setPosition(0.0);
            CheckWait(true, SWPID, flicker_return_delay, flicker_return_delay);
            if(!opModeIsActive()){ return; }

            robot.flicker.setPosition(1.0);
            CheckWait(true, SWPID, flicker_shot_delay, flicker_shot_delay);
            if(!opModeIsActive()){ return; }

            robot.flicker.setPosition(0.0);

            // Drive over rings
            robot.drive.followTrajectoryAsync(traj[TIdx++]);
            CheckWait(true, SWPID, flicker_return_delay, 0);
            if(!opModeIsActive()){ return; }

            // 4 shots
            robot.flicker.setPosition(1.0);
            CheckWait(true, SWPID, flicker_shot_delay, 0);
            if(!opModeIsActive()){ return; }

            robot.flicker.setPosition(0.0);
            CheckWait(true, SWPID, flicker_return_delay, 0);
            if(!opModeIsActive()){ return; }

            robot.flicker.setPosition(1.0);
            CheckWait(true, SWPID, flicker_shot_delay, 0);
            if(!opModeIsActive()){ return; }

            robot.flicker.setPosition(0.0);
            CheckWait(true, SWPID, flicker_return_delay, 0);
            if(!opModeIsActive()){ return; }

            robot.flicker.setPosition(1.0);
            CheckWait(true, SWPID, flicker_shot_delay, 0);
            if(!opModeIsActive()){ return; }

            robot.flicker.setPosition(0.0);
            CheckWait(true, SWPID, flicker_return_delay, 0);
            if(!opModeIsActive()){ return; }

            robot.flicker.setPosition(1.0);
            CheckWait(true, SWPID, flicker_shot_delay, 0);
            if(!opModeIsActive()){ return; }

            // Extra delay on the last one
            CheckWait(true, SWPID, 125, 0);

            // Stop intake
            robot.intakeStop();

            // Turn motor down
            robot.setShooter(0, 0, SWPID);

            robot.flicker.setPosition(0.0);
            if(!opModeIsActive()){ return; }

            if (wobbleEnabled) {
                // Drive/Turn wobble in and lower arm
                robot.drive.followTrajectoryAsync(traj[TIdx++]);
                CheckWait(true, SWPID, 0, 0);

                // Drop wobble
                robot.claw.setPosition(0);
                CheckWait(true, SWPID, 250, 0);
                if(!opModeIsActive()){ return; }

                // Move away from wobble
                robot.drive.followTrajectoryAsync(traj[TIdx++]);
                CheckWait(true, SWPID, 0, 0);

                // Return arm
                CheckWait(true, SWPID, 1000, 0);
                if(!opModeIsActive()){ return; }

            } else {
                // Go Park on wall
                robot.drive.followTrajectoryAsync(traj[TIdx++]);
                CheckWait(true, SWPID, 0, 0);
                if(!opModeIsActive()){ return; }
            }
        }
    }

    private void power_tower_shots( boolean powerShots ) {

        if(powerShots) {
            // First Power Shot
            //robot.logger.logD("powerShots:", String.format("P1: %.2f (%.2f)", Math.toDegrees(robot.drive.getRawExternalHeading()),POWER_SHOT_ANGLE1));
            robot.flicker.setPosition(1.0);
            CheckWait(true, SWPID, flicker_shot_delay, 0);
            robot.flicker.setPosition(0.0);
            if (!opModeIsActive()) { return; }

            // Second Power shot
            robot.drive.turnAsync(Math.toRadians(POWER_SHOT_ANGLE2) - robot.drive.getRawExternalHeading());
            CheckWait(true, SWPID, flicker_return_delay, 0);
            //robot.logger.logD("powerShots:", String.format("P2: %.2f (%.2f)", Math.toDegrees(robot.drive.getRawExternalHeading()),POWER_SHOT_ANGLE2));
            robot.flicker.setPosition(1.0);
            CheckWait(true, SWPID, flicker_shot_delay, 0);
            robot.flicker.setPosition(0.0);
            if (!opModeIsActive()) { return; }

            // Tower shot
            robot.setShooter(side_high_tower_RPM, high_tower_power, SWPID);
            robot.drive.turnAsync(Math.toRadians(TOWER_SHOT_ANGLE) - robot.drive.getRawExternalHeading());
            CheckWait(true, SWPID, flicker_return_delay, 0);
            //robot.logger.logD("powerShots:", String.format("T: %.2f (%.2f)", Math.toDegrees(robot.drive.getRawExternalHeading()),TOWER_SHOT_ANGLE));
            robot.flicker.setPosition(1.0);
            CheckWait(true, SWPID, flicker_shot_delay, 0);
            robot.flicker.setPosition(0.0);
            robot.setShooter(0, 0, SWPID);
            if (!opModeIsActive()) { return; }
        } else {
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
}

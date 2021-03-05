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

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Vector2d;

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

@Autonomous(name="RR_Ring_Red_Right", group="MechBot")
public class RR_Ring_Red_Right extends LinearOpMode {
    private final int RING0=0;
    private final int RING1=1;
    private final int RING4=2;
    private final int RINGP0=3;
    private final int RINGP1=4;

    private double RING0_TURN1, RING0_TURN2;
    private double RING1_TURN1, RING1_TURN2, RING1_TURN3;
    private double RING4_TURN1, RING4_TURN2, RING4_TURN3, RING4_TURN4;
    private double RINGP0_TURN1, RINGP0_TURN2, RINGP0_TURN3;
    private double RINGP1_TURN1, RINGP1_TURN2, RINGP1_TURN3, RINGP1_TURN4;
    private double POWER_SHOT_ANGLE = 24.25;

    private RRMechBot robot = new RRMechBot();

    private static final double wobble_power = 0.6;
    private String className = this.getClass().getSimpleName().toLowerCase();

    private static final double intake_eject_wobble = 0.6;
    private static final double intake_pickup_ring = 1.0;

    // These are for the REV HUB...
    private static final double high_tower_power = 0.4775;
    private static final double long_shot_boost = 0.000;
    private static final double power_shot_power = 0.435;
    private static final double power_offset = 0.003;

    // These are for the SW PID...
    private static final double high_tower_RPM = 3525;
    private static final double long_shot_RPM_boost = -35;
    private static final double power_shot_RPM = 3250;
    private static final double power_RPM_offset = 20;

    private static final double flicker_shot_delay = 250;
    private static final double flicker_return_delay = 350;

    // This is a one-stop switchover from SWPID back to REV HUB PID
    // The setShooter function consumes this flag and will switch back and forth as needed.
    private static boolean SWPID = true;

    private static final double startingX = -63.0;
    private static final double startingY = -57.0;

    private IncepVision vision = new IncepVision();
    private int ringCount = -1;
    private Trajectory[][] trajs = {new Trajectory[25], new Trajectory[25], new Trajectory[25],new Trajectory[25],new Trajectory[25]} ;

    @Override
    public void runOpMode() {
        //robot.logger.LOGLEVEL = robot.logger.LOGDEBUG ;

        // Init the robot and subsystems
        robot.init(hardwareMap);
        robot.initAutonomous(this);
        //robot.logger.LOGLEVEL |= robot.logger.LOGDEBUG;

        // Robot center is 9" from each edge:
        // Back against the -x wall, wheels aligned on first tile in -y
        // THIS MUST BE DONE BEFORE BUILDING
        // THIS MUST BE DONE AFTER THE ROBOT IS IN ITS FINAL POSITION
        Pose2d startPose = new Pose2d(startingX, startingY, Math.toRadians(0));
        robot.drive.setPoseEstimate(startPose);

        BuildPowerRing0();
        BuildRing0();
        BuildPowerRing1();
        BuildRing1();
        BuildRing4(true);

        // Stare at the rings really hard until its time to go or stop
        vision.initAutonomous(this);
        vision.clip = false;
        int deltaX=0, deltaY=0;
        do {
            ringCount = vision.countRings();
            if ( gamepad1.dpad_left ) { IncepVision.clipLeft -= 1; }
            if ( gamepad1.dpad_right ) { IncepVision.clipLeft += 1; }
            if ( gamepad1.dpad_down ) { IncepVision.clipTop += 1; }
            if ( gamepad1.dpad_up ) { IncepVision.clipTop -= 1; }
            if ( gamepad1.x ) { IncepVision.clipRight += 1; }
            if ( gamepad1.b ) { IncepVision.clipRight -= 1; }
            if ( gamepad1.a ) { IncepVision.clipBottom -= 1; }
            if ( gamepad1.y ) { IncepVision.clipBottom += 1; }
            if ( gamepad1.left_bumper ) { vision.tfod.deactivate(); vision.tfodState=false;}
            if ( (gamepad1.right_bumper) && (!vision.tfodState) ) { vision.clip = true; }

            if ( gamepad2.dpad_left ) { IncepVision.clipLeft -= 1; }
            if ( gamepad2.dpad_right ) { IncepVision.clipLeft += 1; }
            if ( gamepad2.dpad_down ) { IncepVision.clipTop += 1; }
            if ( gamepad2.dpad_up ) { IncepVision.clipTop -= 1; }
            if ( gamepad2.x ) { IncepVision.clipRight += 1; }
            if ( gamepad2.b ) { IncepVision.clipRight -= 1; }
            if ( gamepad2.a ) { IncepVision.clipBottom -= 1; }
            if ( gamepad2.y ) { IncepVision.clipBottom += 1; }
            if ( gamepad2.left_bumper ) { vision.tfod.deactivate(); vision.tfodState=false;}
            if ( (gamepad2.right_bumper) && (!vision.tfodState) ) { vision.clip = true; }

            // Move the entire box with the joystick.
            deltaY = (int)(gamepad1.left_stick_y * 2.1);
            deltaX = (int)(gamepad1.left_stick_x * 2.1);
            deltaY += (int)(gamepad1.right_stick_y * 2.1);
            deltaX += (int)(gamepad1.right_stick_x * 2.1);
            IncepVision.clipTop += deltaY;
            IncepVision.clipBottom -= deltaY;
            IncepVision.clipLeft += deltaX;
            IncepVision.clipRight -= deltaX;

            // Move the entire box with the joystick.
            deltaY += (int)(gamepad2.left_stick_y * 2.1);
            deltaX += (int)(gamepad2.left_stick_x * 2.1);
            deltaY += (int)(gamepad2.right_stick_y * 2.1);
            deltaX += (int)(gamepad2.right_stick_x * 2.1);
            IncepVision.clipTop += deltaY;
            IncepVision.clipBottom -= deltaY;
            IncepVision.clipLeft += deltaX;
            IncepVision.clipRight -= deltaX;

            // Observe some limits
            IncepVision.clipLeft   = Range.clip(IncepVision.clipLeft,  5,635);
            IncepVision.clipRight  = Range.clip(IncepVision.clipRight, 5,635);
            IncepVision.clipTop    = Range.clip(IncepVision.clipTop,   5,475);
            IncepVision.clipBottom = Range.clip(IncepVision.clipBottom,5,475);
        } while (!isStarted() && (!isStopRequested())) ;
        vision.shutdown();

        // FIXME: Test the stopping in auto here again.
        if (opModeIsActive()) {
            if (ringCount == 0) {
                RingP0(trajs[RINGP0]);
                //Ring0(trajs[RING0]);
            } else if (ringCount == 1) {
                RingP1(trajs[RINGP1]);
                //Ring1(trajs[RING1]);
            } else {
                // If it's not 0 or 1, assume 4 (highest possible point total)
                Ring4(trajs[RING4]);
            }
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

    private void BuildPowerRing0() {

        int TIdx = 0;

        // Starting X,Y = -63,-57

        // Pose: -8, -57, 24.25
        trajs[RINGP0][TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-8,-57, Math.toRadians(POWER_SHOT_ANGLE)))
                .build();
        // shoot #1

        // Pose: -8, -57, 31.0
        RINGP0_TURN1 = 6.0;
        //robot.drive.turnAsync(Math.toRadians(RINGP0_TURN1));
        // shoot #2

        // Pose: 14, -50, -90.0
        trajs[RINGP0][TIdx++] = robot.drive.trajectoryBuilder(trajs[RINGP0][TIdx-2].end().plus(new Pose2d(0, 0, Math.toRadians(RINGP0_TURN1))))
                .lineToLinearHeading(new Pose2d(14,-50, Math.toRadians(-90.0)))
                .build();

        // Drop wobble

        // Pose: -33, -18, 0
        trajs[RINGP0][TIdx++] = robot.drive.trajectoryBuilder(trajs[RINGP0][TIdx-2].end(),true)
                .splineTo(new Vector2d(-22,-18), Math.toRadians(180.0))
                .splineTo(new Vector2d(-33,-18), Math.toRadians(180.0))
                .build();

        // Grab wobble

        // Drive to final shot
        // Pose:  -7, -31, 0
        trajs[RINGP0][TIdx++] = robot.drive.trajectoryBuilder(trajs[RINGP0][TIdx-2].end())
                .lineTo(new Vector2d(-7,-35))
                .build();
        // Shoot

        // Turn to drop wobble
        // Pose:  -7, -31, 110
        RINGP0_TURN2 = 110;
        //robot.drive.turnAsync(Math.toRadians(RINGP0_TURN2));

        // Drive to drop wobble
        // Pose: Trig...
        trajs[RINGP0][TIdx++] = robot.drive.trajectoryBuilder(trajs[RINGP0][TIdx-2].end().plus(new Pose2d(0, 0, Math.toRadians(RINGP0_TURN2))))
                .back(6)
                .build();

        // Drop wobble
        // Park
        // Pose:  Trig...
        trajs[RINGP0][TIdx++] = robot.drive.trajectoryBuilder(trajs[RINGP0][TIdx-2].end())
                .forward(5)
                .build();

        // Pose:  Trig...
        trajs[RINGP0][TIdx++] = robot.drive.trajectoryBuilder(trajs[RINGP0][TIdx-2].end())
                .strafeRight(14)
                .build();

        // Turn to face 90
        // Pose:  Trig...
        //RINGP0_TURN3 = -(RINGP0_TURN2-90);
        //robot.drive.turnAsync(Math.toRadians(RINGP0_TURN3));

        trajs[RINGP0][TIdx++] = robot.drive.trajectoryBuilder(trajs[RINGP0][TIdx-2].end())
                .lineToLinearHeading(new Pose2d(12,-24, Math.toRadians(0.0)))
                .build();
    }

    private void RingP0(Trajectory[] traj) {
        int TIdx = 0;

        // Go to shooting location
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Start shooter
        robot.setShooter(power_shot_RPM, power_shot_power, SWPID);

        // Shoot sequence
        CheckWait(true, SWPID, 1500, 0);
        if(!opModeIsActive()){ return; }

        //robot.logger.logD("RRMechLog PRing0-0",String.format("turn done: final: %f, exp: %f", Math.toDegrees(robot.drive.getRawExternalHeading()), POWER_SHOT_ANGLE ));

        robot.flicker.setPosition(1.0);
        CheckWait(true, SWPID, flicker_shot_delay, 0);
        if(!opModeIsActive()){ return; }

        robot.setShooter(power_shot_RPM+power_RPM_offset, power_shot_power+power_offset, SWPID);

        robot.drive.turnAsync(Math.toRadians(RINGP0_TURN1));
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        CheckWait(true, SWPID, flicker_return_delay, 0);
        if(!opModeIsActive()){ return; }

        //robot.logger.logD("RRMechLog PRin01-1",String.format("turn done: final: %f, exp: %f", Math.toDegrees(robot.drive.getRawExternalHeading()), POWER_SHOT_ANGLE+RINGP0_TURN1 ));

        // Extra delay for stability
        CheckWait(true, SWPID, 500, 0);

        robot.flicker.setPosition(1.0);
        CheckWait(true, SWPID, flicker_shot_delay, 0);
        if(!opModeIsActive()){ return; }

        // Wait before turning motor down
        CheckWait(true, SWPID, 250, 0);
        robot.setShooter(0, 0, SWPID);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        if(!opModeIsActive()){ return; }

        // Go to wobble drop
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Drop wobble
        robot.intakeEjectWobble(intake_eject_wobble);
        CheckWait(true, SWPID, 600, 0);
        if(!opModeIsActive()){ return; }

        robot.intakeStop();
        if(!opModeIsActive()){ return; }

        //prime some stuff for wobble
        robot.claw.setPosition(0);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(robot.WOBBLE_PICKUP, 0.8);
        if(!opModeIsActive()){ return; }

        // Go to second wobble
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Pick up wobble
        robot.claw.setPosition(1);
        CheckWait(true, SWPID, 1000, 0);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(robot.WOBBLE_CARRY, 0.8);
        CheckWait(true, SWPID, 500, 0);
        if(!opModeIsActive()){ return; }

        robot.setShooter(high_tower_RPM, high_tower_power, SWPID);

        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Delay for stability
        CheckWait(true, SWPID, 500, 0);

        robot.flicker.setPosition(1.0);
        CheckWait(true, SWPID, flicker_shot_delay, 0);
        if(!opModeIsActive()){ return; }

        // Turn to drop wobble
        robot.drive.turnAsync(Math.toRadians(RINGP0_TURN2));
        CheckWait(true, SWPID, 0, 0);

        robot.setShooter(0, 0, SWPID);

        robot.flicker.setPosition(0.0);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(robot.WOBBLE_DROP, 0.6);
        if(!opModeIsActive()){ return; }

        // Back wobble in
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Drop wobble
        robot.claw.setPosition(0);
        CheckWait(true, SWPID, 500, 0);
        if(!opModeIsActive()){ return; }

        // Park over line
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(robot.WOBBLE_START, 0.8);
        robot.claw.setPosition(1.0);
        if(!opModeIsActive()){ return; }

        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        //robot.drive.turnAsync(Math.toRadians(RINGP0_TURN3));
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 2000, 0);
        if(!opModeIsActive()){ return; }

    }

    private void BuildPowerRing1() {

        int TIdx = 0;

        // Starting X,Y = -63,-57

        // Pose: -8, -57, 24.25
        // Drive to first shot
        trajs[RINGP1][TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-8,-57, Math.toRadians(POWER_SHOT_ANGLE)))
                .build();
        // shoot #1

        // Pose: -8, -57, 31.0
        RINGP1_TURN1 = 6.0;
        //robot.drive.turnAsync(Math.toRadians(RINGP0_TURN1));
        // shoot #2

        // Drive to wobble drop

        // Pose: 25, -34, 0
        trajs[RINGP1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RINGP1][TIdx-2].end().plus(new Pose2d(0, 0, Math.toRadians(RINGP1_TURN1))))
                .lineToLinearHeading(new Pose2d(25,-34, Math.toRadians(0.0)))
                .build();
        // Drop wobble

        // Drive to 2nd wobble

        // Pose: -33, -18, 0
        trajs[RINGP1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RINGP1][TIdx-2].end(),true)
                .splineTo(new Vector2d(-22,-18), Math.toRadians(180.0))
                .splineTo(new Vector2d(-33,-18), Math.toRadians(180.0))
                .build();

        // Grab wobble

        // Line up on ring
        // Pose: -40, -38, 0
        trajs[RINGP1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RINGP1][TIdx-2].end())
                .lineTo(new Vector2d(-40,-38))
                .build();

        // Turn on intake
        // Drive over ring
        // Pose: -7, -38, 0
        trajs[RINGP1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RINGP1][TIdx-2].end())
                .forward(33)
                .build();

        // Turn to shoot just a little
        // Pose: -7, -38, 2.5
        RINGP1_TURN2 = 2.5;
        //robot.drive.turnAsync(Math.toRadians(RINGP1_TURN2));

        // Shoot

        // Turn to drop wobble
        // Pose: -7, -38, 165
        RINGP1_TURN3 = 162.5;
        //robot.drive.turnAsync(Math.toRadians(RINGP1_TURN2));

        // Drive to drop wobble
        // Pose: Trig
        trajs[RINGP1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RINGP1][TIdx-2].end().plus(new Pose2d(0, 0, Math.toRadians(RINGP1_TURN3+RINGP1_TURN2))))
                .back(19)
                .build();

        // Drop wobble
        // Park
        // Pose: Trig
        trajs[RINGP1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RINGP1][TIdx-2].end())
                .forward(6)
                .build();

        // Return wobble arm

        // Turn to face 90
        // Pose: Trig
        //RINGP1_TURN4 = -((RINGP1_TURN2+RINGP1_TURN3)-90);
        //robot.drive.turnAsync(Math.toRadians(RINGP1_TURN4));

        trajs[RINGP1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RINGP1][TIdx-2].end())
                .lineToLinearHeading(new Pose2d(12,-24, Math.toRadians(0.0)))
                .build();
    }

    private void RingP1(Trajectory[] traj) {
        int TIdx = 0;

        // Go to shooting location
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Start shooter
        robot.setShooter(power_shot_RPM, power_shot_power, SWPID);

        // Shoot sequence
        CheckWait(true, SWPID, 1500, 0);
        if(!opModeIsActive()){ return; }

        //robot.logger.logD("RRMechLog PRing1-0",String.format("turn done: final: %f, exp: %f", Math.toDegrees(robot.drive.getRawExternalHeading()), POWER_SHOT_ANGLE ));

        robot.flicker.setPosition(1.0);
        CheckWait(true, SWPID, flicker_shot_delay, 0);
        if(!opModeIsActive()){ return; }

        robot.setShooter(power_shot_RPM+power_RPM_offset, power_shot_power+power_offset, SWPID);

        robot.drive.turnAsync(Math.toRadians(RINGP1_TURN1));
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        CheckWait(true, SWPID, flicker_return_delay, 0);
        if(!opModeIsActive()){ return; }

        //robot.logger.logD("RRMechLog PRing1-1",String.format("turn done: final: %f, exp: %f", Math.toDegrees(robot.drive.getRawExternalHeading()), POWER_SHOT_ANGLE+RINGP1_TURN1 ));

        // Extra delay for stability
        CheckWait(true, SWPID, 500, 0);

        robot.flicker.setPosition(1.0);
        CheckWait(true, SWPID, flicker_shot_delay, 0);
        if(!opModeIsActive()){ return; }

        CheckWait(true, SWPID, 250, 0);
        robot.setShooter(0, 0, SWPID);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        if(!opModeIsActive()){ return; }

        // Go to wobble drop
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Drop wobble
        robot.intakeEjectWobble(intake_eject_wobble);
        CheckWait(true, SWPID, 600, 0);
        if(!opModeIsActive()){ return; }

        robot.intakeStop();
        if(!opModeIsActive()){ return; }

        //prime some stuff for wobble
        robot.claw.setPosition(0);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(robot.WOBBLE_PICKUP, 0.8);
        if(!opModeIsActive()){ return; }

        // Go to second wobble
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Pick up wobble
        robot.claw.setPosition(1);
        CheckWait(true, SWPID, 1000, 0);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(robot.WOBBLE_CARRY, 0.8);
        CheckWait(true, SWPID, 500, 0);
        if(!opModeIsActive()){ return; }

        // Lineup on ring
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        robot.intakePickupRing(intake_pickup_ring);

        // Turn on shooter
        robot.setShooter(high_tower_RPM, high_tower_power, SWPID);

        // Drive over ring
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Stop intake
        robot.intakeStop();

        // Turn a little to shoot
        robot.drive.turnAsync(Math.toRadians(RINGP1_TURN2));
        CheckWait(true, SWPID, 0, 0);

        // Extra delay for stability
        CheckWait(true, SWPID, 500, 0);

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
        CheckWait(true, SWPID, 250, 0);
        robot.setShooter(0, 0, SWPID);

        robot.flicker.setPosition(0.0);
        if(!opModeIsActive()){ return; }

        // Turn to drop wobble
        robot.drive.turnAsync(Math.toRadians(RINGP1_TURN3));
        CheckWait(true, SWPID, 0, 0);

        robot.setWobblePosition(robot.WOBBLE_DROP, 0.6);
        if(!opModeIsActive()){ return; }

        // Back wobble in
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Drop wobble
        robot.claw.setPosition(0);
        CheckWait(true, SWPID, 500, 0);
        if(!opModeIsActive()){ return; }

        // Park over line
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Return arm
        robot.setWobblePosition(robot.WOBBLE_START, 0.8);
        robot.claw.setPosition(1.0);
        if(!opModeIsActive()){ return; }

        // Face forward
        //robot.drive.turnAsync(Math.toRadians(RINGP1_TURN4));
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 2000, 0);
        if(!opModeIsActive()){ return; }
    }

    private void BuildRing4(boolean powerShot) {
        int TIdx = 0;

        // Starting X,Y = -63,-57

        // Are we gonna pot-shot at the power shot?
        if (powerShot) {
            // Pose: 48, -57, 0.0
            // Drive to the wobble drop zone, don't put it on the wall
            trajs[RING4][TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .addDisplacementMarker(3, () -> {
                        robot.setShooter(power_shot_RPM, power_shot_power, SWPID);
                    })
                    .splineToSplineHeading(new Pose2d(-16, -57, Math.toRadians(POWER_SHOT_ANGLE + 4)), Math.toRadians(0))
                    .addDisplacementMarker(43, () -> {
                        robot.flicker.setPosition(1.0);
                    })
                    .splineToSplineHeading(new Pose2d(48, -57, Math.toRadians(0)), Math.toRadians(0))
                    .addDisplacementMarker(90, () -> {
                        robot.flicker.setPosition(0.0);
                        robot.setShooter(0, 0, SWPID);
                    })
                    .build();
        } else {
            // Pose: 48, -57, 0.0
            // Drive to the wobble drop zone, don't put it on the wall
            trajs[RING4][TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                    .forward(111)
                    .build();
        }

        // Drive to the shooting location
        // Be careful here, this move is relative, not absolute x,y.
        // Pose: -7, -33, 0.0
        trajs[RING4][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING4][TIdx-2].end())
                .lineTo((trajs[RING4][TIdx-2].end().vec().plus(new Vector2d(-55.0, 24.0))))
                .build();

        // Shoot all 3

        // The next 2 things may need to be a reverse spline move with constant heading to save time
        // Line up and back into the wobble
        // Pose: -7, -17, 0.0
        trajs[RING4][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING4][TIdx-2].end())
                .strafeLeft(16)
                .build();

        // Back to wobble
        // Pose: -34, -17, 0.0
        trajs[RING4][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING4][TIdx-2].end())
                .back(27)
                .build();

        // Grab wobble

        // Line up on ring
        // Pose: -40, -38, 0.0
        trajs[RING4][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING4][TIdx-2].end())
                .lineTo(new Vector2d(-40,-38))
                .build();

        // Pickup 1-3 rings
        // Pose: -24, -38, 0.0
        trajs[RING4][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING4][TIdx-2].end())
                .forward(16)
                .build();

        // Turn to line up
        // Pose: -24, -38, 2.5
        RING4_TURN1 = 2.5;
        //robot.drive.turnAsync(Math.toRadians(RING4_TURN1));

        // shoot 1 ring

        // Turn back
        // Pose: -24, -38, 2.0
        RING4_TURN2 = 0.0;
        //robot.drive.turnAsync(Math.toRadians(RING4_TURN2));

        // Go to shooting location
        // Pose: ~(-10, -38, 2.5)
        trajs[RING4][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING4][TIdx-2].end().plus(new Pose2d(0, 0, Math.toRadians(RING4_TURN1+RING4_TURN2))))
                .forward(14)
                .build();

        // Shoot 3

        // Turn for wobble back-in
        // Pose: Trig
        RING4_TURN3 = 152.5;
        //robot.drive.turnAsync(Math.toRadians(RING4_TURN3));

        // Lower wobble

        // Back the wobble in
        // Pose: Trig
        trajs[RING4][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING4][TIdx-2].end().plus(new Pose2d(0, 0, Math.toRadians(RING4_TURN3))))
                .back(51)
                .build();

        // Run to park
        // Pose: Trig
        trajs[RING4][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING4][TIdx-2].end())
                .lineTo(new Vector2d(12,-24))
                .addTemporalMarker(0.35, 0.0, () -> {
                    robot.setWobblePosition(robot.WOBBLE_START, wobble_power);
                    robot.claw.setPosition(1.0);
                })
                .build();

        // Face back to 0
        //robot.drive.turnAsync(-robot.drive.getRawExternalHeading());

        // Raise arm etc.
    }

    private void Ring4(Trajectory[] traj) {
        int TIdx = 0;

        // Full speed length of field to the target zone
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID,0,0);
        if(!opModeIsActive()){ return; }

        // Drop the wobble and run
        robot.intakeEjectWobble(intake_eject_wobble);
        CheckWait(true, SWPID,300,0);
        if(!opModeIsActive()){ return; }

        // Power up early
        robot.setShooter( high_tower_RPM,high_tower_power+.0075, SWPID);
        if(!opModeIsActive()){ return; }

        // Move to line up shot
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID,0,0);
        if(!opModeIsActive()){ return; }

        // Turn off intake now
        robot.intakeStop();
        CheckWait(true, SWPID,250,0);
        if(!opModeIsActive()){ return; }

        // Fire away
        robot.flicker.setPosition(1.0);
        CheckWait(true, SWPID,flicker_shot_delay,0);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        CheckWait(true, SWPID,flicker_return_delay,0);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        CheckWait(true, SWPID,flicker_shot_delay,0);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(robot.WOBBLE_PICKUP, wobble_power);
        robot.flicker.setPosition(0.0);
        CheckWait(true, SWPID,flicker_return_delay,0);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        CheckWait(true, SWPID,flicker_shot_delay,0);
        if(!opModeIsActive()){ return; }

        // Line up for wobble pickup
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID,0,0);
        if(!opModeIsActive()){ return; }

        // Open claw
        robot.claw.setPosition(0.25);

        robot.setShooter(0,0, SWPID);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        if(!opModeIsActive()){ return; }

        // Back to wobble
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID,0,0);
        if(!opModeIsActive()){ return; }

        // Grab the wobble
        robot.claw.setPosition(1);
        CheckWait(true, SWPID,750,0);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(robot.WOBBLE_CARRY, wobble_power);
        CheckWait(true, SWPID,200,0);
        if(!opModeIsActive()){ return; }

        // Line up over 4-stack
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID,0,0);
        if(!opModeIsActive()){ return; }

        // Get ready to intake and shoot
        robot.intakePickupRing(intake_pickup_ring);
        if(!opModeIsActive()){ return; }

        // Power up early
        robot.setShooter(high_tower_RPM+long_shot_RPM_boost, high_tower_power+long_shot_boost, SWPID );
        if(!opModeIsActive()){ return; }

        // Go intake a few and shoot one
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID,0,0);
        if(!opModeIsActive()){ return; }

        // Turn a little towards the goal
        robot.drive.turnAsync(Math.toRadians(RING4_TURN1));
        CheckWait(true, SWPID,0,0);
        if(!opModeIsActive()){ return; }

        // Wait for the rings to settle
        CheckWait(true, SWPID,750,0);
        robot.flicker.setPosition(1.0);
        CheckWait(true, SWPID,flicker_shot_delay,0);
        if(!opModeIsActive()){ return; }

        robot.setShooter(high_tower_RPM, high_tower_power, SWPID);

        // Turn a little back
        robot.drive.turnAsync(Math.toRadians(RING4_TURN2));
        CheckWait(true, SWPID,0,0);

        robot.flicker.setPosition(0.0);
        if(!opModeIsActive()){ return; }

        // Intake the reset and shoot them
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID,0,0);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        CheckWait(true, SWPID,flicker_shot_delay,0);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        CheckWait(true, SWPID,flicker_return_delay,0);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        CheckWait(true, SWPID,flicker_shot_delay,0);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        CheckWait(true, SWPID,flicker_return_delay,0);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        CheckWait(true, SWPID,flicker_shot_delay,0);
        if(!opModeIsActive()){ return; }

        // Stop the shooter
        CheckWait(true, SWPID,250,0);
        robot.setShooter(0, 0, SWPID);
        if(!opModeIsActive()){ return; }

        // Stop the intake
        robot.intakeStop();
        if(!opModeIsActive()){ return; }

        // Turn around and back the wobble in
        robot.drive.turnAsync(Math.toRadians(RING4_TURN3));
        CheckWait(true, SWPID,0,0);

        robot.flicker.setPosition(0.0);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(robot.WOBBLE_DROP, wobble_power);

        // Drive the wobble in
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID,0,0);
        if(!opModeIsActive()){ return; }

        // Drop it off
        robot.claw.setPosition(0);
        CheckWait(true, SWPID,250,0);
        if(!opModeIsActive()){ return; }

        // Run like heck
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID,0,0);
        if(!opModeIsActive()){ return; }

        // Face the return rack
        robot.drive.turnAsync(-robot.drive.getRawExternalHeading());
        CheckWait(true, SWPID,0,0);
        if(!opModeIsActive()){ return; }

        CheckWait(true, SWPID,500,0);
        if(!opModeIsActive()){ return; }

    }

    private void BuildRing0() {

        int TIdx = 0;


        trajs[RING0][TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-8,-57, Math.toRadians(24.25)))
                .build();

        trajs[RING0][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING0][TIdx-2].end())
                .lineToLinearHeading(new Pose2d(14,-50, Math.toRadians(-90.0)))
                .build();

        // Drop wobble

        // Drive to second wobble
        trajs[RING0][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING0][TIdx-2].end(),true)
                .splineTo(new Vector2d(-22,-18), Math.toRadians(180.0))
                .splineTo(new Vector2d(-33,-18), Math.toRadians(180.0))
                .build();

        // Grab wobble
        // Drive to final shot
        trajs[RING0][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING0][TIdx-2].end())
                .lineTo(new Vector2d(-7,-35))
                .build();
        // Shoot

        // Turn to drop wobble
        RING0_TURN1 = 110;
        //robot.drive.turnAsync(Math.toRadians(RING0_TURN1));

        // Drive to drop wobble
        trajs[RING0][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING0][TIdx-2].end().plus(new Pose2d(0, 0, Math.toRadians(RING0_TURN1))))
                .back(6)
                .build();

        // Drop wobble
        // Park
        trajs[RING0][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING0][TIdx-2].end())
                .forward(5)
                .build();

        trajs[RING0][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING0][TIdx-2].end())
                .strafeRight(14)
                .build();

        // Turn to face 90
        // Pose:  Trig...
        RING0_TURN2 = -(RING0_TURN1-90);
        //robot.drive.turnAsync(Math.toRadians(RING0_TURN3));

    }

    private void Ring0(Trajectory[] traj) {
        int TIdx = 0;

        // Go to wobble drop
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Drop wobble
        robot.intakeEjectWobble(intake_eject_wobble);
        CheckWait(true, SWPID, 600, 0);
        if(!opModeIsActive()){ return; }

        robot.intakeStop();
        if(!opModeIsActive()){ return; }

        //prime some stuff for wobble
        robot.claw.setPosition(0);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(robot.WOBBLE_PICKUP, 0.8);
        if(!opModeIsActive()){ return; }

        // Go to second wobble
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Pick up wobble
        robot.claw.setPosition(1);
        CheckWait(true, SWPID, 1000, 0);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(robot.WOBBLE_CARRY, 0.8);
        CheckWait(true, SWPID, 500, 0);
        if(!opModeIsActive()){ return; }

        robot.setShooter(high_tower_RPM, high_tower_power, SWPID);

        // Go to shooting location
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }
        CheckWait(true, SWPID, 1000, 0);

        // Shoot 3 times
        robot.flicker.setPosition(1.0);
        CheckWait(true, SWPID, 250, 0);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        CheckWait(true, SWPID, 300, 0);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        CheckWait(true, SWPID, 250, 0);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        CheckWait(true, SWPID, 300, 0);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        CheckWait(true, SWPID, 250, 0);
        if(!opModeIsActive()){ return; }

        robot.setShooter(0, 0, SWPID);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        if(!opModeIsActive()){ return; }

        // Turn to drop wobble
        robot.drive.turnAsync(Math.toRadians(RING0_TURN1));
        CheckWait(true, SWPID, 0, 0);

        robot.setShooter(0, 0, SWPID);

        robot.flicker.setPosition(0.0);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(robot.WOBBLE_DROP, 0.6);
        if(!opModeIsActive()){ return; }

        // Back wobble in
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Drop wobble
        robot.claw.setPosition(0);
        CheckWait(true, SWPID, 500, 0);
        if(!opModeIsActive()){ return; }

        // Park over line
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(robot.WOBBLE_START, 0.8);
        robot.claw.setPosition(1.0);
        if(!opModeIsActive()){ return; }

        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        robot.drive.turnAsync(Math.toRadians(RING0_TURN2));
        CheckWait(true, SWPID, 0, 0);

        CheckWait(true, SWPID, 1000, 0);
        if(!opModeIsActive()){ return; }

    }

    private void BuildRing1() {

        int TIdx = 0;

        // Drive to shooting location
        trajs[RING1][TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(-7,-57))
                .build();

        trajs[RING1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING1][TIdx-2].end())
                .lineToConstantHeading(new Vector2d(-7,-35))
                .build();

        // Shoot 3x

        // Drive to wobble drop
        trajs[RING1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING1][TIdx-2].end())
                .lineToConstantHeading(new Vector2d(25,-34))
                .build();
        // Drop wobble

        // Drive to 2nd wobble
        trajs[RING1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING1][TIdx-2].end(),true)
                .splineTo(new Vector2d(-22,-18), Math.toRadians(180.0))
                .splineTo(new Vector2d(-33.5,-18), Math.toRadians(180.0))
                .build();

        // Grab wobble
        // Line up on ring
        trajs[RING1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING1][TIdx-2].end())
                .lineTo(new Vector2d(-40,-38))
                .build();

        // Turn on intake
        // Drive over ring
        trajs[RING1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING1][TIdx-2].end())
                .forward(33)
                .build();

        // Shoot

        // Turn to shoot just a little
        // Pose: -7, -38, 2.5
        RING1_TURN1 = 0.0;
        //robot.drive.turnAsync(Math.toRadians(RING1_TURN1));

        // Shoot

        // Turn to drop wobble
        // Pose: -7, -38, 165
        RING1_TURN2 = 165.0;
        //robot.drive.turnAsync(Math.toRadians(RING1_TURN2));

        // Drive to drop wobble
        trajs[RING1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING1][TIdx-2].end().plus(new Pose2d(0, 0, Math.toRadians(RING1_TURN1+RING1_TURN2))))
                .back(18)
                .build();

        // Drop wobble
        // Park
        trajs[RING1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING1][TIdx-2].end())
                .forward(5)
                .build();

        // Return wobble arm

        // Turn to face 90
        RING1_TURN3 = -((RING1_TURN1+RING1_TURN2)-90);
        //robot.drive.turnAsync(Math.toRadians(RINGP1_TURN3));
    }

    private void Ring1(Trajectory[] traj) {
        int TIdx = 0;

        // Start shooter early
        robot.setShooter(high_tower_RPM, high_tower_power, SWPID);

        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Go to shooting location
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }
        CheckWait(true, SWPID, 1000, 0);

        // Shoot 3 times
        robot.flicker.setPosition(1.0);
        CheckWait(true, SWPID, 250, 0);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        CheckWait(true, SWPID, 300, 0);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        CheckWait(true, SWPID, 250, 0);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        CheckWait(true, SWPID, 300, 0);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        CheckWait(true, SWPID, 250, 0);
        if(!opModeIsActive()){ return; }

        robot.setShooter(0, 0, SWPID);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        if(!opModeIsActive()){ return; }

        // Go to wobble drop
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Drop wobble
        robot.intakeEjectWobble(intake_eject_wobble);
        CheckWait(true, SWPID, 600, 0);
        if(!opModeIsActive()){ return; }

        robot.intakeStop();
        if(!opModeIsActive()){ return; }

        //prime some stuff for wobble
        robot.claw.setPosition(0);
        robot.setWobblePosition(robot.WOBBLE_PICKUP, 0.8);
        if(!opModeIsActive()){ return; }

        // Go to second wobble
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Pick up wobble
        robot.claw.setPosition(1);
        CheckWait(true, SWPID, 1000, 0);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(robot.WOBBLE_CARRY, 0.8);
        CheckWait(true, SWPID, 500, 0);
        if(!opModeIsActive()){ return; }

        // Lineup on ring
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        robot.intakePickupRing(intake_pickup_ring);

        // Turn on shooter
        robot.setShooter(high_tower_RPM, high_tower_power, SWPID);

        // Drive over ring
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Turn a little to shoot
        robot.drive.turnAsync(Math.toRadians(RING1_TURN1));
        CheckWait(true, SWPID, 0, 0);
        CheckWait(true, SWPID, 1000, 0);

        // Stop intake
        robot.intakeStop();

        // 2 shots
        robot.flicker.setPosition(1.0);
        CheckWait(true, SWPID, 750, 0);
        if(!opModeIsActive()){ return; }

        // Turn to drop wobble
        robot.drive.turnAsync(Math.toRadians(RING1_TURN2));
        CheckWait(true, SWPID, 0, 0);

        robot.setShooter(0, 0, SWPID);

        robot.flicker.setPosition(0.0);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(robot.WOBBLE_DROP, 0.6);
        if(!opModeIsActive()){ return; }

        // Back wobble in
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Drop wobble
        robot.claw.setPosition(0);
        CheckWait(true, SWPID, 500, 0);
        if(!opModeIsActive()){ return; }

        // Park over line
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, SWPID, 0, 0);
        if(!opModeIsActive()){ return; }

        // Return arm
        robot.setWobblePosition(robot.WOBBLE_START, 0.8);
        robot.claw.setPosition(1.0);
        if(!opModeIsActive()){ return; }

        // Face forward
        robot.drive.turnAsync(Math.toRadians(RING1_TURN3));
        CheckWait(true, SWPID, 0, 0);
        CheckWait(true, SWPID, 1000, 0);
        if(!opModeIsActive()){ return; }
    }
}
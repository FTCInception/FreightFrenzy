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

    private double RING0_TURN1;
    private double RING1_TURN1;
    private double RING4_TURN1, RING4_TURN2, RING4_TURN3;
    private double RINGP0_TURN1, RINGP0_TURN2;
    private double RINGP1_TURN1, RINGP1_TURN2, RINGP1_TURN3;

    private RRMechBot robot = new RRMechBot();

    private static final double wobble_power = 0.6;
    private String className = this.getClass().getSimpleName().toLowerCase();

    private static final double intake_eject_wobble = 0.6;
    private static final double intake_pickup_ring = 1.0;

    private IncepVision vision = new IncepVision();
    private int ringCount = -1;
    private Trajectory[][] trajs = {new Trajectory[25], new Trajectory[25], new Trajectory[25],new Trajectory[25],new Trajectory[25]} ;

    @Override
    public void runOpMode() {
        //robot.logger.LOGLEVEL = robot.logger.LOGDEBUG ;

        // Init the robot and subsystems
        robot.init(hardwareMap);
        robot.initAutonomous(this);

        // Robot center is 9" from each edge:
        // Back against the -x wall, wheels aligned on first tile in -y
        // THIS MUST BE DONE BEFORE BUILDING
        // THIS MUST BE DONE AFTER THE ROBOT IS IN ITS FINAL POSITION
        Pose2d startPose = new Pose2d(-72+9, -48-9, Math.toRadians(0));
        robot.drive.setPoseEstimate(startPose);

        BuildPowerRing0();
        BuildRing0();
        BuildPowerRing1();
        BuildRing1();
        BuildRing4();

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

    private void BuildPowerRing0() {

        int TIdx = 0;

        trajs[RINGP0][TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-8,-57, Math.toRadians(25.0)))
                .build();
        // shoot #1
        RINGP0_TURN1 = 6.0;
        //robot.drive.turn(Math.toRadians(RINGP0_TURN1));
        // shoot #2
        trajs[RINGP0][TIdx++] = robot.drive.trajectoryBuilder(trajs[RINGP0][TIdx-2].end().plus(new Pose2d(0, 0, Math.toRadians(RINGP0_TURN1))))
                .lineToLinearHeading(new Pose2d(14,-50, Math.toRadians(-90.0)))
                .build();
        // Drop wobble
        trajs[RINGP0][TIdx++] = robot.drive.trajectoryBuilder(trajs[RINGP0][TIdx-2].end(),true)
                .splineTo(new Vector2d(-22,-18), Math.toRadians(180.0))
                .splineTo(new Vector2d(-33,-18), Math.toRadians(180.0))
                .build();

        // Grab wobble
        // Drive to final shot
        trajs[RINGP0][TIdx++] = robot.drive.trajectoryBuilder(trajs[RINGP0][TIdx-2].end())
                .lineTo(new Vector2d(-7,-31))
                .build();
        // Shoot

        // Turn to drop wobble
        RINGP0_TURN2 = 110;
        //robot.drive.turn(Math.toRadians(RINGP0_TURN2));

        // Drive to drop wobble
        trajs[RINGP0][TIdx++] = robot.drive.trajectoryBuilder(trajs[RINGP0][TIdx-2].end().plus(new Pose2d(0, 0, Math.toRadians(RINGP0_TURN2))))
                .back(10)
                .build();

        // Drop wobble
        // Park
        trajs[RINGP0][TIdx++] = robot.drive.trajectoryBuilder(trajs[RINGP0][TIdx-2].end())
                .forward(5)
                .build();

        trajs[RINGP0][TIdx++] = robot.drive.trajectoryBuilder(trajs[RINGP0][TIdx-2].end())
                .strafeRight(14)
                .build();
    }

    private void RingP0(Trajectory[] traj) {
        int TIdx = 0;

        // Start shooter 1 move early
        robot.shoot1_motor.setPower(0.440);
        robot.shoot2_motor.setPower(0.440);

        // Go to shooting location
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        robot.shoot1_motor.setPower(0.437);
        robot.shoot2_motor.setPower(0.437);

        // Shoot sequence
        sleep(1000);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.shoot1_motor.setPower(0.439);
        robot.shoot2_motor.setPower(0.439);

        robot.drive.turn(Math.toRadians(RINGP0_TURN1));
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.shoot1_motor.setPower(0.0);
        robot.shoot2_motor.setPower(0.0);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        if(!opModeIsActive()){ return; }

        // Go to wobble drop
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        // Drop wobble
        robot.intakeEjectWobble(intake_eject_wobble);
        sleep(600);
        if(!opModeIsActive()){ return; }

        robot.intakeStop();
        if(!opModeIsActive()){ return; }

        //prime some stuff for wobble
        robot.claw.setPosition(0);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(robot.WOBBLE_PICKUP, 0.8);
        if(!opModeIsActive()){ return; }

        // Go to second wobble
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        // Pick up wobble
        robot.claw.setPosition(1);
        sleep(1000);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(robot.WOBBLE_CARRY, 0.8);
        sleep(500);
        if(!opModeIsActive()){ return; }

        robot.shoot1_motor.setPower(0.475);
        robot.shoot2_motor.setPower(0.475);

        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }
        sleep(500);

        robot.flicker.setPosition(1.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        // Turn to drop wobble
        robot.drive.turn(Math.toRadians(RINGP0_TURN2));

        robot.shoot1_motor.setPower(0.0);
        robot.shoot2_motor.setPower(0.0);

        robot.flicker.setPosition(0.0);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(robot.WOBBLE_DROP, 0.6);
        if(!opModeIsActive()){ return; }

        // Back wobble in
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        // Drop wobble
        robot.claw.setPosition(0);
        sleep(500);
        if(!opModeIsActive()){ return; }

        // Park over line
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(robot.WOBBLE_START, 0.8);
        sleep(500);
        if(!opModeIsActive()){ return; }

        robot.claw.setPosition(1.0);
        sleep(500);
        if(!opModeIsActive()){ return; }
    }

    private void BuildPowerRing1() {

        int TIdx = 0;

        // Drive to first shot
        trajs[RINGP1][TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-8,-57, Math.toRadians(25.0)))
                .build();
        // shoot #1

        RINGP1_TURN1 = 6.0;
        //robot.drive.turn(Math.toRadians(RINGP0_TURN1));
        // shoot #2

        // Drive to wobble drop
        trajs[RINGP1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RINGP1][TIdx-2].end().plus(new Pose2d(0, 0, Math.toRadians(RINGP1_TURN1))))
                .lineToLinearHeading(new Pose2d(25,-34, Math.toRadians(0.0)))
                .build();
        // Drop wobble

        // Drive to 2nd wobble
        trajs[RINGP1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RINGP1][TIdx-2].end(),true)
                .splineTo(new Vector2d(-22,-18), Math.toRadians(180.0))
                .splineTo(new Vector2d(-33,-18), Math.toRadians(180.0))
                .build();

        // Grab wobble
        // Line up on ring
        trajs[RINGP1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RINGP1][TIdx-2].end())
                .lineTo(new Vector2d(-40,-38))
                .build();

        // Turn on intake
        // Drive over ring
        trajs[RINGP1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RINGP1][TIdx-2].end())
                .forward(33)
                .build();

        // Shoot

        // Turn to drop wobble
        RINGP1_TURN2 = 165;
        //robot.drive.turn(Math.toRadians(RINGP1_TURN2));

        // Drive to drop wobble
        trajs[RINGP1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RINGP1][TIdx-2].end().plus(new Pose2d(0, 0, Math.toRadians(RINGP1_TURN2))))
                .back(18)
                .build();

        // Drop wobble
        // Park
        trajs[RINGP1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RINGP1][TIdx-2].end())
                .forward(5)
                .build();

        // Return wobble arm

        // Turn to face 90
        RINGP1_TURN3 = -(RINGP1_TURN2-90);
        //robot.drive.turn(Math.toRadians(RINGP1_TURN3));
    }

    private void RingP1(Trajectory[] traj) {
        int TIdx = 0;

        // Start shooter 1 move early
        robot.shoot1_motor.setPower(0.440);
        robot.shoot2_motor.setPower(0.440);

        // Go to shooting location
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        robot.shoot1_motor.setPower(0.437);
        robot.shoot2_motor.setPower(0.437);

        // Shoot sequence
        sleep(1000);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.shoot1_motor.setPower(0.439);
        robot.shoot2_motor.setPower(0.439);

        robot.drive.turn(Math.toRadians(RINGP1_TURN1));
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.shoot1_motor.setPower(0.0);
        robot.shoot2_motor.setPower(0.0);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        if(!opModeIsActive()){ return; }

        // Go to wobble drop
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        // Drop wobble
        robot.intakeEjectWobble(intake_eject_wobble);
        sleep(600);
        if(!opModeIsActive()){ return; }

        robot.intakeStop();
        if(!opModeIsActive()){ return; }

        //prime some stuff for wobble
        robot.claw.setPosition(0);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(robot.WOBBLE_PICKUP, 0.8);
        if(!opModeIsActive()){ return; }

        // Go to second wobble
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        // Pick up wobble
        robot.claw.setPosition(1);
        sleep(1000);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(robot.WOBBLE_CARRY, 0.8);
        sleep(500);
        if(!opModeIsActive()){ return; }

        // Lineup on ring
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        robot.intakePickupRing(intake_pickup_ring);

        // Turn on shooter
        robot.shoot1_motor.setPower(0.475);
        robot.shoot2_motor.setPower(0.475);

        // Drive over ring
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        // Stop intake
        robot.intakeStop();

        // 2 shots
        robot.flicker.setPosition(1.0);
        sleep(500);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        sleep(600);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        // Turn to drop wobble
        robot.drive.turn(Math.toRadians(RINGP1_TURN2));

        robot.shoot1_motor.setPower(0.0);
        robot.shoot2_motor.setPower(0.0);

        robot.flicker.setPosition(0.0);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(robot.WOBBLE_DROP, 0.6);
        if(!opModeIsActive()){ return; }

        // Back wobble in
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        // Drop wobble
        robot.claw.setPosition(0);
        sleep(500);
        if(!opModeIsActive()){ return; }

        // Park over line
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        // Return arm
        robot.setWobblePosition(robot.WOBBLE_START, 0.8);
        robot.claw.setPosition(1.0);
        if(!opModeIsActive()){ return; }

        // Face forward
        robot.drive.turn(Math.toRadians(RINGP1_TURN3));
        if(!opModeIsActive()){ return; }
    }

    private void BuildRing4() {
        int TIdx = 0;

        // Drive to the wobble drop zone, don't put it on the wall
        trajs[RING4][TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .forward(111)
                .build();

        // Drive to the shooting location
        // Be careful here, this move is relative, not absolute x,y.
        trajs[RING4][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING4][TIdx-2].end())
                .lineTo((trajs[RING4][TIdx-2].end().vec().plus(new Vector2d(-53.0, 24.0))))
                .build();

        // Shoot all 3

        // The next 2 things may need to be a reverse spline move with constant heading to save time
        // Line up and back into the wobble
        trajs[RING4][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING4][TIdx-2].end())
                .strafeLeft(16)
                .build();

        // Back to wobble
        trajs[RING4][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING4][TIdx-2].end())
                .back(29)
                .build();

        // Grab wobble

        // Line up on ring
        trajs[RING4][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING4][TIdx-2].end())
                .lineTo(new Vector2d(-40,-38))
                .build();

        // Pickup 1-3 rings
        trajs[RING4][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING4][TIdx-2].end())
                .forward(16)
                .build();

        // Turn to line up
        RING4_TURN1 = 3.0;
        //robot.drive.turn(Math.toRadians(RING4_TURN1));

        // shoot 1 ring

        // Turn back
        RING4_TURN2 = -1.0;
        //robot.drive.turn(Math.toRadians(RING4_TURN2));

        // Go to shooting location
        trajs[RING4][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING4][TIdx-2].end().plus(new Pose2d(0, 0, Math.toRadians(RING4_TURN1+RING4_TURN2))))
                .forward(14)
                .build();

        // Shoot 3

        // Turn for wobble back-in
        RING4_TURN3 = 152.0;
        //robot.drive.turn(Math.toRadians(RING4_TURN3));

        // Lower wobble

        // Back the wobble in
        trajs[RING4][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING4][TIdx-2].end().plus(new Pose2d(0, 0, Math.toRadians(RING4_TURN3))))
                .back(51)
                .build();

        // Run to park
        trajs[RING4][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING4][TIdx-2].end())
                .forward(28)
                .addTemporalMarker(0.25, 0.0, () -> {
                    robot.setWobblePosition(robot.WOBBLE_START, wobble_power);
                    robot.claw.setPosition(1.0);
                })
                .build();

        // Raise arm etc.
    }

    private void Ring4(Trajectory[] traj) {
        int TIdx = 0;

        // Full speed length of field to the target zone
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        // Drop the wobble and run
        robot.intakeEjectWobble(intake_eject_wobble);
        if(!opModeIsActive()){ return; }

        // Power up early
        robot.shoot1_motor.setPower(0.475);
        robot.shoot2_motor.setPower(0.475);
        if(!opModeIsActive()){ return; }

        // Move to line up shot
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        // Turn off intake now
        robot.intakeStop();
        sleep(250);
        if(!opModeIsActive()){ return; }

        // Fire away
        robot.flicker.setPosition(1.0);
        sleep(500);
        if(!opModeIsActive()){ return; }

        robot.shoot1_motor.setPower(0.475);
        robot.shoot2_motor.setPower(0.475);

        robot.flicker.setPosition(0.0);
        sleep(650);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(500);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(robot.WOBBLE_PICKUP, wobble_power);
        robot.flicker.setPosition(0.0);
        sleep(650);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(300);
        if(!opModeIsActive()){ return; }

        // Line up for wobble pickup
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        // Open claw
        robot.claw.setPosition(0.25);

        robot.shoot1_motor.setPower(0.0);
        robot.shoot2_motor.setPower(0.0);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        if(!opModeIsActive()){ return; }

        // Back to wobble
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        // Grab the wobble
        robot.claw.setPosition(1);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(robot.WOBBLE_CARRY, wobble_power);
        sleep(200);
        if(!opModeIsActive()){ return; }

        // Line up over 4-stack
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        // Get ready to intake and shoot
        robot.intakePickupRing(intake_pickup_ring);
        if(!opModeIsActive()){ return; }

        // Power up early
        robot.shoot1_motor.setPower(0.485);
        robot.shoot2_motor.setPower(0.485);
        if(!opModeIsActive()){ return; }

        // Go intake a few and shoot one
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        // Turn a little towards the goal
        robot.drive.turn(Math.toRadians(RING4_TURN1));
        if(!opModeIsActive()){ return; }

        // Wait for the rings to settle
        sleep(500);
        robot.flicker.setPosition(1.0);
        sleep(200);
        if(!opModeIsActive()){ return; }

        robot.shoot1_motor.setPower(0.475);
        robot.shoot2_motor.setPower(0.475);

        // Turn a little back
        robot.drive.turn(Math.toRadians(RING4_TURN2));

        robot.flicker.setPosition(0.0);
        if(!opModeIsActive()){ return; }

        // Intake the reset and shoot them
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(500);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        sleep(650);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(500);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        sleep(650);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(300);
        if(!opModeIsActive()){ return; }

        // Stop the shooter
        robot.shoot1_motor.setPower(0.0);
        robot.shoot2_motor.setPower(0.0);
        if(!opModeIsActive()){ return; }

        // Stop the intake
        robot.intakeStop();
        if(!opModeIsActive()){ return; }

        // Turn around and back the wobble in
        robot.drive.turn(Math.toRadians(RING4_TURN3));

        robot.flicker.setPosition(0.0);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(robot.WOBBLE_DROP, wobble_power);

        // Drive the wobble in
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        // Drop it off
        robot.claw.setPosition(0);
        sleep(250);
        if(!opModeIsActive()){ return; }

        //robot.setWobblePosition(robot.WOBBLE_START, wobble_power);

        // Run like heck
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        //robot.claw.setPosition(1.0);
        sleep(500);
        if(!opModeIsActive()){ return; }
    }

    private void BuildRing0() {

        int TIdx = 0;

        trajs[RING0][TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .forward(57)
                .build();
        trajs[RING0][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING0][TIdx-2].end())
                .strafeLeft(24)
                .build();
        trajs[RING0][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING0][TIdx-2].end())
                .strafeLeft(13)
                .build();
        trajs[RING0][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING0][TIdx-2].end())
                .back(32)
                .build();
        trajs[RING0][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING0][TIdx-2].end())
                .forward(50)
                .build();
        RING0_TURN1 = 90.0;
        //robot.drive.turn(Math.toRadians(RING0_TURN1));
        trajs[RING0][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING0][TIdx-2].end().plus(new Pose2d(0, 0, Math.toRadians(RING0_TURN1))))
                .back(21)
                .build();
        trajs[RING0][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING0][TIdx-2].end())
                .forward(11)
                .build();
    }

    private void Ring0(Trajectory[] traj) {
        int TIdx = 0;

        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        robot.intakeEjectWobble(intake_eject_wobble);
        sleep(600);
        if(!opModeIsActive()){ return; }

        robot.intakeStop();
        if(!opModeIsActive()){ return; }

        // Start shooter 1 move early
        robot.shoot1_motor.setPower(0.480);
        robot.shoot2_motor.setPower(0.480);

        // Strafe to shooting position
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        // Shoot sequence
        sleep(500);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.shoot1_motor.setPower(0.475);
        robot.shoot2_motor.setPower(0.475);

        robot.flicker.setPosition(0.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(500);
        if(!opModeIsActive()){ return; }

        robot.shoot1_motor.setPower(0.0);
        robot.shoot2_motor.setPower(0.0);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        if(!opModeIsActive()){ return; }

        //prime some stuff for wobble
        robot.claw.setPosition(0);
        sleep(500);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(1, wobble_power);
        sleep(2000);
        if(!opModeIsActive()){ return; }

        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        robot.claw.setPosition(1);
        sleep(1000);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(3, wobble_power);
        sleep(1000);
        if(!opModeIsActive()){ return; }

        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        robot.drive.turn(Math.toRadians(RING0_TURN1));
        if(!opModeIsActive()){ return; }

        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        robot.claw.setPosition(0);
        sleep(500);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(0, wobble_power);
        sleep(500);
        if(!opModeIsActive()){ return; }

        robot.claw.setPosition(1.0);
        if(!opModeIsActive()){ return; }

        robot.drive.followTrajectory(traj[TIdx++]);
        sleep(1000);
        if(!opModeIsActive()){ return; }
    }
    private void BuildRing1() {

        int TIdx = 0;

        trajs[RING1][TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .forward(87)
                .build();
        trajs[RING1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING1][TIdx-2].end())
                .strafeLeft(24)
                .build();
        trajs[RING1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING1][TIdx-2].end())
                .back(30)
                .build();
        trajs[RING1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING1][TIdx-2].end())
                .strafeLeft(13)
                .build();
        trajs[RING1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING1][TIdx-2].end())
                .back(32)
                .build();
        trajs[RING1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING1][TIdx-2].end())
                .strafeRight(18)
                .build();
        trajs[RING1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING1][TIdx-2].end())
                .forward(29)
                .build();
        trajs[RING1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING1][TIdx-2].end())
                .strafeLeft(5)
                .build();
        RING1_TURN1 = 167.5;
        //robot.drive.turn(Math.toRadians(RING1_TURN1));
        trajs[RING1][TIdx++] = robot.drive.trajectoryBuilder(trajs[RING1][TIdx-2].end().plus(new Pose2d(0, 0, Math.toRadians(RING1_TURN1))))
                .back(24)
                .build();
    }

    private void Ring1(Trajectory[] traj) {
        int TIdx = 0;

        // Drive to drop zone
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        // Drop the wobble
        robot.intakeEjectWobble(intake_eject_wobble);
        sleep(600);
        if(!opModeIsActive()){ return; }

        robot.intakeStop();
        if(!opModeIsActive()){ return; }

        // Start the shooter early
        robot.shoot1_motor.setPower(0.480);
        robot.shoot2_motor.setPower(0.480);
        if(!opModeIsActive()){ return; }

        // Back to shooting zone
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        sleep(500);
        if(!opModeIsActive()){ return; }

        // Fire away
        robot.flicker.setPosition(1.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.shoot1_motor.setPower(0.475);
        robot.shoot2_motor.setPower(0.475);

        robot.flicker.setPosition(0.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        // Get ready for wobble
        robot.claw.setPosition(0.25);
        robot.setWobblePosition(1, wobble_power);
        robot.flicker.setPosition(0.0);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(1.0);
        sleep(200);
        if(!opModeIsActive()){ return; }

        // Line up for wobble
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        robot.shoot1_motor.setPower(0.0);
        robot.shoot2_motor.setPower(0.0);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        if(!opModeIsActive()){ return; }

        // Back up to wobble
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        // Grab wobble
        robot.claw.setPosition(1);
        sleep(750);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(3, wobble_power);
        sleep(500);
        if(!opModeIsActive()){ return; }

        // Strafe to line up on ring
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        // Turn on intake
        robot.intakePickupRing(intake_pickup_ring);
        if(!opModeIsActive()){ return; }

        // Intake ring and drive to shoot distance
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        // Power up early
        robot.shoot1_motor.setPower(0.480);
        robot.shoot2_motor.setPower(0.480);
        if(!opModeIsActive()){ return; }

        // Line up for shot
        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        // Stop the intake
        robot.intakeStop();
        sleep(500);
        if(!opModeIsActive()){ return; }

        // Shoot the ring
        robot.flicker.setPosition(1.0);
        sleep(400);
        if(!opModeIsActive()){ return; }

        robot.flicker.setPosition(0.0);
        robot.shoot1_motor.setPower(0.0);
        robot.shoot2_motor.setPower(0.0);
        if(!opModeIsActive()){ return; }

        robot.drive.turn(Math.toRadians(RING1_TURN1));
        if(!opModeIsActive()){ return; }

        robot.drive.followTrajectory(traj[TIdx++]);
        if(!opModeIsActive()){ return; }

        robot.claw.setPosition(0.0);
        sleep(500);
        if(!opModeIsActive()){ return; }

        robot.setWobblePosition(0, wobble_power);
        sleep(1000);
        if(!opModeIsActive()){ return; }

        robot.claw.setPosition(1.0);
        sleep(1000);
        if(!opModeIsActive()){ return; }
    }

}
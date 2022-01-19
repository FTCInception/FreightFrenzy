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

import Inception.FreightFrenzy.RRMechBot.SlideHeight;

import static Inception.FreightFrenzy.drive.DriveConstants.MAX_ACCEL;
import static Inception.FreightFrenzy.drive.DriveConstants.MAX_ANG_ACCEL;
import static Inception.FreightFrenzy.drive.DriveConstants.MAX_ANG_VEL;
import static Inception.FreightFrenzy.drive.DriveConstants.MAX_VEL;
import static Inception.FreightFrenzy.drive.DriveConstants.TRACK_WIDTH;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


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
@Autonomous(name="Red_MultiblockTAS_Auto", group="RRMechBot")
public class Red_MultiblockTAS_Auto extends LinearOpMode {

    public SlideHeight targetLevel = SlideHeight.HighDrop;

    private RRMechBot robot = new RRMechBot(true);
    private TapeMeasureV4 tape = new TapeMeasureV4();

    private static final double intake_pickup_ring = 1.0;

    // This is the starting position of the center of the robot.
    private static final double startingX = 0.0;
    private static final double startingY = 0.0;

    // 1:1 slide
    //final double SLIDE_INTAKE = 1.0, SLIDE_DRIVE = 0.9, SLIDE_LOW = 0.8, SLIDE_SHARED = 0.73, SLIDE_MED = 0.5, SLIDE_HIGH = 0.0;
    // 2:1 slide
    final double SLIDE_INTAKE = (1.0-1.0)*.4+.3, SLIDE_DRIVE = (1.0-0.9)*.4+.3, SLIDE_LOW = (1.0-0.8)*.4+.3, SLIDE_SHARED = (1.0-0.73)*.4+.3, SLIDE_MED = (1.0-0.5)*.4+.3, SLIDE_HIGH = (1.0-0.0)*.4+.3;
    private static boolean parkThroughOpening = true;
    private static boolean secondBlock = false;
    private static boolean option3 = false;

    private static double Dx = 0.0;
    private static double Dy = 0.0;

    private IncepVision vision = new IncepVision();
    private IncepVision.MarkerPos grnLocation = IncepVision.MarkerPos.Unseen;
    private Trajectory[] trajs = new Trajectory[25];
    double bucketFullTime = 0;
    boolean bucketFull = false;

    @Override
    public void runOpMode() {
        //robot.logger.LOGLEVEL = robot.logger.LOGDEBUG ;

        // Init the robot and subsystems.  Set Roadrunner timeout to 0.25s to save some time.
        robot.init(hardwareMap,0.0);
        robot.initAutonomous(this);
        tape.init(this, robot, gamepad2, true);
        tape.setPosition(tape.TAPE_AUTO);

        robot.logger.LOGLEVEL |= robot.logger.LOGDEBUG;

        parkThroughOpening = true;
        secondBlock = false;
        option3 = true;

        Dx = 0.0;
        Dy = 0.0;

        // This code allows for processing the starting location of something variable
        // And controls some enable/disable options
        boolean leftOK = true, rightOK = true, upOK = true, downOK = true, bOK=true, xOK=true, yOK=true;
        do {
            if ( gamepad1.dpad_left || gamepad2.dpad_left ) {
                if (leftOK) {
                    Dx -= 1.0;
                    leftOK = false;
                }
            } else {
                leftOK = true;
            }
            if ( gamepad1.dpad_right || gamepad2.dpad_right) {
                if (rightOK) {
                    Dx += 1.0;
                    rightOK = false;
                }
            } else {
                rightOK = true;
            }
            if ( gamepad1.dpad_down || gamepad2.dpad_down ) {
                if (downOK) {
                    Dy -= 1.0;
                    downOK = false;
                }
            } else {
                downOK = true;
            }
            if ( gamepad1.dpad_up || gamepad2.dpad_up ) {
                if (upOK) {
                    Dy += 1.0;
                    upOK = false;
                }
            } else {
                upOK = true;
            }
            if ( gamepad1.y || gamepad2.y ) {
                if (yOK) {
                    parkThroughOpening = !parkThroughOpening;
                    yOK = false;
                }
            } else {
                yOK = true;
            }

            if ( gamepad1.x || gamepad2.x ) {
                if (xOK) {
                    secondBlock = !secondBlock;
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

            telemetry.addData("Park Changes:", "x:%f, y:%f", Dx, Dy);
            telemetry.addData("Use dpad to adjust robot start position","");
            telemetry.addData("'Y' Park through opening?:","(%s)", parkThroughOpening?"true":"false");
            telemetry.addData("'X' Second Block Attempt?:","(%s)", secondBlock?"true":"false");
            telemetry.addData("'B' option3:","(%s)", option3?"true":"false");
            telemetry.addData("'A' Proceed to Vision:","(%s)", "NA");
            telemetry.update();

        } while (!isStarted() && (!isStopRequested())) ;

        // Robot center is 9" from each edge:
        // Back against the -x wall, wheels aligned on first tile in -y
        // THIS MUST BE DONE BEFORE BUILDING
        // THIS MUST BE DONE AFTER THE ROBOT IS IN ITS FINAL POSITION
        Pose2d startPose = new Pose2d(startingX, startingY, robot.drive.getRawExternalHeading()+Math.toRadians(270));
        robot.drive.setPoseEstimate(startPose);

        telemetry.addData("Computing paths","");
        telemetry.update();

        buildTrajs(trajs);

        telemetry.addData("Starting vision", "");
        telemetry.update();
        // Stare at the rings really hard until its time to go or stop
        vision.initAutonomous(this, "webcam", vision.RED_WAREHOUSE);
        vision.clip = false;
        boolean leftBOK = true, rightBOK = true;
        do {
            grnLocation = vision.getGrnLocation();
            vision.manageVisionBox(gamepad1, gamepad2);
        } while (!isStarted() && (!isStopRequested()));
        vision.shutdown();

        // TODO: Make sure the LEFT/RIGHT/UNSEEN mapping here is correct for every auto.
        // FIXME: Test the stopping in auto here again.
        if (opModeIsActive()) {
            tape.setPosition(tape.TAPE_DRIVE);

            if (grnLocation == IncepVision.MarkerPos.Outer) {
                targetLevel = SlideHeight.LowDrop;
            } else if (grnLocation == IncepVision.MarkerPos.Inner) {
                targetLevel = SlideHeight.MidDrop;
            } else {
                // If it's not LEFT or RIGHT, assume unseen
                targetLevel = SlideHeight.HighDrop;
            }

            runTrajs(trajs ,targetLevel);
        }
    }

    private void showTrajPoses( String trajName, int TIdx, Trajectory[] traj ) {

        Pose2d tmpPose = traj[0].start();
        double time = 0;
        robot.logger.logD("showTrajPoses:",String.format("%s, Idx:%d, %s", trajName, -1, tmpPose));

        for (int i=0; i<TIdx; i++ ) {
            tmpPose = traj[i].end();
            time += traj[i].duration();
            robot.logger.logD("showTrajPoses:",String.format("%s, Idx:%d, %s", trajName, i, tmpPose));
        }
    }

    private void CheckWait(boolean checkDrive, double minMS, double maxMS) {

        NanoClock localClock = NanoClock.system();
        double now = localClock.seconds();
        double colorDist=0, sideDist=0;

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

            // Explanation for useless telemetry messages here:
            // http://firsttechchallenge.blogspot.com/2018/11/info-on-wi-fi-disconnects-for-motorola.html
            telemetry.addData("This is a message with no purpose besides preventing Motorola from scanning networks. Thanks a lot Motorola.","");
            telemetry.update();

            // Master stop
            if(!opModeIsActive()){ return; }

            // Update the drive
            if( checkDrive ) { robot.drive.update(); }

            // If the intake is running in forward
            if(robot.intake_motor.getPower() > 0.4) {
                // Check for an element in the bucket
                colorDist = robot.color.getDistance(DistanceUnit.CM);
                if (colorDist > 2.0) {
                    bucketFull = false;
                } else {
                    if (bucketFull == false) {
                        bucketFullTime = localClock.seconds();
                        bucketFull = true;
                    } else {
                        // If we've been full for > 0.25 seconds
                        if ((localClock.seconds() - bucketFullTime) > 0.25) {
                            // Turn the intake off, raise the bucket
                            robot.intake_motor.setPower(0);
                            robot.bucket.setPosition(robot.bucketDrive);
                            robot.setSlidePosition(SlideHeight.Drive);
                        }
                    }
                }
            }

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

    private void buildTrajs(Trajectory[] traj) {

        int TIdx = 0;
        double wall1 = -4;
        double wall2 = -6;
        double wall3 = -8;
        double wall4 = -8;
        double deliverAngle = -75.0;
        double angle1 = 0.0;
        // Starting X,Y = 4,-65

        traj[TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate(), true)
                .lineToLinearHeading(new Pose2d(-10,23, Math.toRadians(deliverAngle)))
                .addDisplacementMarker(.75,0,() -> {
                    robot.bucket.setPosition(robot.bucketDump);
                })
                .build();

        /***** Starter *****/
        // Drive to Wall
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end(), false)
                .addDisplacementMarker(() -> {
                    robot.bucket.setPosition(robot.bucketIntake);
                })
                .addDisplacementMarker( 6, () -> {
                    robot.setSlidePosition(SlideHeight.Drive);
                })
                .lineToLinearHeading(new Pose2d(6, wall1, Math.toRadians(angle1)),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.5)
                )
                .build();

        // Drive to blocks
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end(), false)
                .lineToConstantHeading(new Vector2d(46, wall1),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.5)
                )
                .addDisplacementMarker(12,() -> {
                    robot.setSlidePosition(SlideHeight.Intake);
                    robot.intake_motor.setPower(0.4);
                })
                .build();

        //Maybe need to reset position here?

        // Drive back to wall
        //traj[TIdx++] = robot.drive.trajectoryBuilder(new Pose2d(traj[TIdx - 2].end().getX(), 0, traj[TIdx - 2].end().getHeading()), true)
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end(), true)
                .lineToConstantHeading(new Vector2d(6, wall1),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.5)
                )
                .addTemporalMarker(.25,() -> {
                    robot.setSlidePosition(SlideHeight.Drive);
                    robot.intake_motor.setPower(-0.6);
                })
                .addTemporalMarker(.5,() -> {
                    robot.bucket.setPosition(robot.bucketDrive);
                })
                .build();

        // Drive to hub
        //traj[TIdx++] = robot.drive.trajectoryBuilder(new Pose2d(traj[TIdx - 2].end().getX(), 0, traj[TIdx - 2].end().getHeading()), true)
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end(), true)
                .addDisplacementMarker(() -> {
                    robot.setSlidePosition(SlideHeight.HighDrop);
                    robot.intake_motor.setPower(0.0);
                })
                .lineToLinearHeading(new Pose2d(-12,21, Math.toRadians(deliverAngle)),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.5)
                )
                .addDisplacementMarker(.95,0,() -> {
                    robot.bucket.setPosition(robot.bucketDump);
                })
                .build();

        /***** One-cycle *****/

        // Drive to Wall
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end(), false)
                .addDisplacementMarker(() -> {
                    robot.bucket.setPosition(robot.bucketIntake);
                })
                .addDisplacementMarker( 6, () -> {
                    robot.setSlidePosition(SlideHeight.Drive);
                })
                .lineToLinearHeading(new Pose2d(6, wall2, Math.toRadians(angle1)),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.5)
                )
                .build();

        // Drive to blocks
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end(), false)
                .lineToConstantHeading(new Vector2d(48, wall2),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.5)
                )
                .addDisplacementMarker(12,() -> {
                    robot.setSlidePosition(SlideHeight.Intake);
                    robot.intake_motor.setPower(0.4);
                })
                .build();

        //Maybe need to reset position here?

        // Drive back to wall
        //traj[TIdx++] = robot.drive.trajectoryBuilder(new Pose2d(traj[TIdx - 2].end().getX(), 0, traj[TIdx - 2].end().getHeading()), true)
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end(), true)
                .lineToConstantHeading(new Vector2d(6, wall2),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.5)
                )
                .addTemporalMarker(.25,() -> {
                    robot.setSlidePosition(SlideHeight.Drive);
                    robot.intake_motor.setPower(-0.6);
                })
                .addTemporalMarker(.5,() -> {
                    robot.bucket.setPosition(robot.bucketDrive);
                })
                .build();

        // Drive to hub
        //traj[TIdx++] = robot.drive.trajectoryBuilder(new Pose2d(traj[TIdx - 2].end().getX(), 0, traj[TIdx - 2].end().getHeading()), true)
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end(), true)
                .addDisplacementMarker(() -> {
                    robot.setSlidePosition(SlideHeight.HighDrop);
                    robot.intake_motor.setPower(0.0);
                })
                .lineToLinearHeading(new Pose2d(-14,19, Math.toRadians(deliverAngle)),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.5)
                )
                .addDisplacementMarker(.95,0,() -> {
                    robot.bucket.setPosition(robot.bucketDump);
                })
                .build();

        /***** Two-cycle *****/

        // Drive to Wall
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end(), false)
                .addDisplacementMarker(() -> {
                    robot.bucket.setPosition(robot.bucketIntake);
                })
                .addDisplacementMarker( 6, () -> {
                    robot.setSlidePosition(SlideHeight.Drive);
                })
                .lineToLinearHeading(new Pose2d(6, wall3, Math.toRadians(angle1)),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.5)
                )
                .build();

        // Drive to blocks
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end(), false)
                .lineToConstantHeading(new Vector2d(50, wall3),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.5)
                )
                .addDisplacementMarker(12,() -> {
                    robot.setSlidePosition(SlideHeight.Intake);
                    robot.intake_motor.setPower(0.4);
                })
                .build();

        //Maybe need to reset position here?

        // Drive back to wall
        //traj[TIdx++] = robot.drive.trajectoryBuilder(new Pose2d(traj[TIdx - 2].end().getX(), 0, traj[TIdx - 2].end().getHeading()), true)
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end(), true)
                .lineToConstantHeading(new Vector2d(6, wall3),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.5)
                )
                .addTemporalMarker(.25,() -> {
                    robot.setSlidePosition(SlideHeight.Drive);
                    robot.intake_motor.setPower(-0.6);
                })
                .addTemporalMarker(.5,() -> {
                    robot.bucket.setPosition(robot.bucketDrive);
                })
                .build();

        // Drive to hub
        //traj[TIdx++] = robot.drive.trajectoryBuilder(new Pose2d(traj[TIdx - 2].end().getX(), 0, traj[TIdx - 2].end().getHeading()), true)
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end(), true)
                .addDisplacementMarker(() -> {
                    robot.setSlidePosition(SlideHeight.HighDrop);
                    robot.intake_motor.setPower(0.0);
                })
                .lineToLinearHeading(new Pose2d(-16,17, Math.toRadians(deliverAngle)),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.5)
                )
                .addDisplacementMarker(.95,0,() -> {
                    robot.bucket.setPosition(robot.bucketDump);
                })
                .build();

        /***** Three-cycle *****/

        // Drive to Wall
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end(), false)
                .addDisplacementMarker(() -> {
                    robot.bucket.setPosition(robot.bucketDrive);
                })
                .addDisplacementMarker( 6, () -> {
                    robot.setSlidePosition(SlideHeight.Drive);
                })
                .lineToLinearHeading(new Pose2d(6, wall4, Math.toRadians(angle1)),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.5)
                )
                .build();

        // Drive to blocks (park)
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end(), false)
                .lineToConstantHeading(new Vector2d(42, wall4),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.5)
                )
                .build();

        showTrajPoses( "Trajs", TIdx, traj ) ;
    }

    private void runTrajs(Trajectory[] traj, SlideHeight level) {
        int TIdx = 0;

        // Raise slide, set bucket to drive
        robot.setSlidePosition(SlideHeight.HighDrop); //Reset Bucket to safe level
        CheckWait(true, 100, 0);
        robot.bucket.setPosition(robot.bucketDrive); //Reset Bucket to drive position

        // Drive to Hub
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, 0, 0);
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        if(!opModeIsActive()){ return; }

        CheckWait(true, 250, 0);

        // Drive to wall
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, 0, 0);
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        if(!opModeIsActive()){ return; }
        //robot.drive.setPoseEstimate(new Pose2d(robot.drive.getPoseEstimate().getX(), 0, robot.drive.getPoseEstimate().getHeading()));

        // Drive to blocks
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, 0, 0);
        if (robot.color.getDistance(DistanceUnit.CM) > 2.0) {
            CheckWait(true, 1000, 0);
        }
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        if(!opModeIsActive()){ return; }
        //robot.drive.setPoseEstimate(new Pose2d(robot.drive.getPoseEstimate().getX(), 0, robot.drive.getPoseEstimate().getHeading()));

        // Drive to wall
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, 0, 0);
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        if(!opModeIsActive()){ return; }

        // Drive to Hub
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, 0, 0);
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        if(!opModeIsActive()){ return; }
        //robot.drive.setPoseEstimate(new Pose2d(robot.drive.getPoseEstimate().getX(), 0, robot.drive.getPoseEstimate().getHeading()));

        CheckWait(true, 250, 0);

        // Drive to wall
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, 0, 0);
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        if(!opModeIsActive()){ return; }
        //robot.drive.setPoseEstimate(new Pose2d(robot.drive.getPoseEstimate().getX(), 0, robot.drive.getPoseEstimate().getHeading()));

        // Drive to blocks
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, 0, 0);
        if (robot.color.getDistance(DistanceUnit.CM) > 2.0) {
            CheckWait(true, 1000, 0);
        }
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        if(!opModeIsActive()){ return; }
        //robot.drive.setPoseEstimate(new Pose2d(robot.drive.getPoseEstimate().getX(), 0, robot.drive.getPoseEstimate().getHeading()));

        // Drive to wall
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, 0, 0);
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        if(!opModeIsActive()){ return; }

        // Drive to Hub
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, 0, 0);
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        if(!opModeIsActive()){ return; }
        //robot.drive.setPoseEstimate(new Pose2d(robot.drive.getPoseEstimate().getX(), 0, robot.drive.getPoseEstimate().getHeading()));

        CheckWait(true, 250, 0);

        // Drive to wall
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, 0, 0);
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        if(!opModeIsActive()){ return; }
        //robot.drive.setPoseEstimate(new Pose2d(robot.drive.getPoseEstimate().getX(), 0, robot.drive.getPoseEstimate().getHeading()));

        // Drive to blocks
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, 0, 0);
        if (robot.color.getDistance(DistanceUnit.CM) > 2.0) {
            CheckWait(true, 1000, 0);
        }
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        if(!opModeIsActive()){ return; }
        //robot.drive.setPoseEstimate(new Pose2d(robot.drive.getPoseEstimate().getX(), 0, robot.drive.getPoseEstimate().getHeading()));

        // Drive to wall
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, 0, 0);
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        if(!opModeIsActive()){ return; }

        // Drive to Hub
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, 0, 0);
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        if(!opModeIsActive()){ return; }
        //robot.drive.setPoseEstimate(new Pose2d(robot.drive.getPoseEstimate().getX(), 0, robot.drive.getPoseEstimate().getHeading()));

        CheckWait(true, 250, 0);

        // Drive to wall
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, 0, 0);
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        if(!opModeIsActive()){ return; }
        //robot.drive.setPoseEstimate(new Pose2d(robot.drive.getPoseEstimate().getX(), 0, robot.drive.getPoseEstimate().getHeading()));

        // Drive to blocks
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, 0, 0);
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        if(!opModeIsActive()){ return; }
        //robot.drive.setPoseEstimate(new Pose2d(robot.drive.getPoseEstimate().getX(), 0, robot.drive.getPoseEstimate().getHeading()));

        /*
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, 0, 0);
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        if(!opModeIsActive()){ return; }

        CheckWait(true, 500, 0);

        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        robot.drive.followTrajectoryAsync(traj[TIdx++]);
        CheckWait(true, 0, 0);
        robot.logger.logD("showTraj:",String.format("%s", robot.drive.getPoseEstimate()));
        if(!opModeIsActive()){ return; }
        robot.drive.setPoseEstimate(new Pose2d(robot.drive.getPoseEstimate().getX(), 0, robot.drive.getPoseEstimate().getHeading()));
        */

    }
}
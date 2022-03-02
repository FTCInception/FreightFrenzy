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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import Inception.FreightFrenzy.RRMechBot.SlideHeight;

import static Inception.FreightFrenzy.drive.DriveConstants.MAX_ACCEL;
import static Inception.FreightFrenzy.drive.DriveConstants.MAX_ANG_VEL;
import static Inception.FreightFrenzy.drive.DriveConstants.MAX_VEL;
import static Inception.FreightFrenzy.drive.DriveConstants.TRACK_WIDTH;

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
@Autonomous(name="Red_2x_Auto", group="RRMechBot")
public class Red_Multiblock_Auto extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();

    public SlideHeight targetLevel = SlideHeight.HighDrop;

    private RRMechBot robot = new RRMechBot(true);
    private TapeMeasureV4 tape = new TapeMeasureV4();

    private static final double intake_pickup_ring = 1.0;

    // This is the starting position of the center of the robot.
    private static final double startingX = 3.5;
    private static final double startingY = -65.0;

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
    private Trajectory ohShootItMissedInOutOnWarehouseOneMotionPartOne = null;
    private Trajectory ohShootItMissedInOutOnWarehouseOneMotionPartTwo = null;
    private Trajectory ohShootItMissedInOutOnWarehouseTwoMotionPartOne = null;
    private Trajectory ohShootItMissedInOutOnWarehouseTwoMotionPartTwo = null;



    double bucketFullTime = 0;
    boolean bucketFull = false;
    private boolean abortAuto = false;

    double maxAngleDelta = 10.0, imu_RR_offset = Math.toRadians(90.0), abortTurn = 0.0;

    @Override
    public void runOpMode() {
        //robot.logger.LOGLEVEL = robot.logger.LOGDEBUG ;

        // Init the robot and subsystems.  Set Roadrunner timeout to 0.25s to save some time.
        robot.init(hardwareMap,0.1);
        robot.initAutonomous(this);
        tape.init(this, robot, gamepad2, true);
        tape.setPosition(tape.TAPE_AUTO);
        //robot.logger.LOGLEVEL |= robot.logger.LOGDEBUG;

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
        runtime.reset();
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

            if (abortAuto) {
                robot.bucket.setPosition(robot.bucketDrive);
                CheckWait(true, 500, 0);
                robot.setSlidePosition(SlideHeight.Drive);
                CheckWait(true, 500, 0);
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
            if(isIntaking()) {
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
                            robot.bucket.setPosition(robot.bucketDrive);
                            robot.setSlidePosition(SlideHeight.MidDrop);
                            sleep(100);
                            robot.intake_motor.setPower(0);
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
        double scaleSpeed = 0.75;
        double bfDistance = 8.0;
        // Starting X,Y = 4,-63

        // First trip to Hub
        traj[TIdx++] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate(), true)
                .lineToLinearHeading(new Pose2d(-7,-42, Math.toRadians(280)),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*2.5*scaleSpeed)
                )

                .addDisplacementMarker(.90,0,() -> {
                    safeBucketDump(); //Drop freight
                })

                .build();

        // First trip to warehouse
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                .splineToSplineHeading(new Pose2d(4, -67, Math.toRadians(0)), Math.toRadians(340.0),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.35*scaleSpeed)
                )
                .splineToConstantHeading(new Vector2d(13, -69), Math.toRadians(0),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.75*scaleSpeed)
                )

                .splineToConstantHeading(new Vector2d(51, -69), Math.toRadians(0),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.75*scaleSpeed)
                )

                .addDisplacementMarker(10, () -> {
                    robot.setSlidePosition(SlideHeight.Drive);}) //Slide to drive

                .addDisplacementMarker(35, () -> {
                    robot.bucket.setPosition(robot.bucketIntake); // Bucket to intake position
                    robot.setSlidePosition(SlideHeight.Intake);   // Slide to intake
                })

                .addDisplacementMarker(40, () -> {
                    robot.intake_motor.setPower(0.6);}) // Turn on intake
                .build();

        // Second trip to Hub
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end(), true)
                .splineToConstantHeading(new Vector2d(13, -69), Math.toRadians(180),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.75*scaleSpeed)
                )
                .splineToConstantHeading(new Vector2d(4, -67), Math.toRadians(160),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.75*scaleSpeed)
                )
                .splineToSplineHeading(new Pose2d(-7,-42.5, Math.toRadians(280)), Math.toRadians(100),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.35*scaleSpeed)
                )

                .addDisplacementMarker(10, () -> {
                    doubleElementFollowing(abortTurn,false);}) // Emergency stop

                .addDisplacementMarker(20, () -> {
                    robot.setSlidePosition(SlideHeight.HighDrop);}) //Slide to high drop

                .addDisplacementMarker(0.12,0,() -> {
                    robot.intake_motor.setPower(0);
                    robot.bucket.setPosition(robot.bucketDrive);
                    robot.setSlidePosition(SlideHeight.MidDrop);
                })

                .addDisplacementMarker(0.15,0,() -> {
                    robot.intake_motor.setPower(-0.6); // Reverse intake
                })

                //.addDisplacementMarker(0.5,0,() -> {
                //    robot.intake_motor.setPower(0); // Stop Intake
                //})

                .addDisplacementMarker(.95,0,() -> {
                    safeBucketDump(); //Drop freight
                })

                .build();

        // Second trip to warehouse
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                .splineToSplineHeading(new Pose2d(4, -67, Math.toRadians(0)), Math.toRadians(340.0),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.35*scaleSpeed)
                )
                .splineToConstantHeading(new Vector2d(13, -69), Math.toRadians(0),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.75*scaleSpeed)
                )

                .splineToConstantHeading(new Vector2d(55, -69), Math.toRadians(0),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.75*scaleSpeed)
                )

                .addDisplacementMarker(10, () -> {
                    robot.setSlidePosition(SlideHeight.Drive);}) //Slide to drive

                .addDisplacementMarker(35, () -> {
                    robot.bucket.setPosition(robot.bucketIntake); // Bucket to intake position
                    robot.setSlidePosition(SlideHeight.Intake);   // Slide to intake
                })

                .addDisplacementMarker(40, () -> {
                    robot.intake_motor.setPower(0.6);}) // Turn on intake
                .build();

        // Third trip to Hub
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end(), true)
                .splineToConstantHeading(new Vector2d(13, -69), Math.toRadians(180),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.75*scaleSpeed)
                )
                .splineToConstantHeading(new Vector2d(4, -67), Math.toRadians(160),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.75*scaleSpeed)
                )
                .splineToSplineHeading(new Pose2d(-7,-42.5, Math.toRadians(280)), Math.toRadians(100),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.35*scaleSpeed)
                )

                .addDisplacementMarker(10, () -> {
                    doubleElementFollowing(abortTurn,false);}) // Emergency stop

                .addDisplacementMarker(20, () -> {
                    robot.setSlidePosition(SlideHeight.HighDrop);}) //Slide to high drop

                .addDisplacementMarker(0.12,0,() -> {
                    robot.intake_motor.setPower(0);
                    robot.bucket.setPosition(robot.bucketDrive);
                    robot.setSlidePosition(SlideHeight.MidDrop);
                })

                .addDisplacementMarker(0.15,0,() -> {
                    robot.intake_motor.setPower(-0.6); // Reverse intake
                })

                //.addDisplacementMarker(0.5,0,() -> {
                //    robot.intake_motor.setPower(0); // Stop Intake
                //})

                .addDisplacementMarker(.95,0,() -> {
                    safeBucketDump(); //Drop freight
                })

                .build();

        // Parking trip
        traj[TIdx++] = robot.drive.trajectoryBuilder(traj[TIdx - 2].end())
                .splineToSplineHeading(new Pose2d(4, -67, Math.toRadians(0)), Math.toRadians(340.0),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.35*scaleSpeed)
                )
                .splineToConstantHeading(new Vector2d(13, -69), Math.toRadians(0),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.75*scaleSpeed)
                )

                .splineToConstantHeading(new Vector2d(45, -69), Math.toRadians(0),
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*2.0*scaleSpeed)
                )
                .addDisplacementMarker(10, () -> {
                    robot.setSlidePosition(SlideHeight.Drive);}) //Slide to drive

                .build();

        ohShootItMissedInOutOnWarehouseOneMotionPartOne = robot.drive.trajectoryBuilder(traj[1].end())
                .back(bfDistance,
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.75*scaleSpeed)
                )
                .build();

        ohShootItMissedInOutOnWarehouseOneMotionPartTwo = robot.drive.trajectoryBuilder(ohShootItMissedInOutOnWarehouseOneMotionPartOne.end())
                .forward(bfDistance,
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.75*scaleSpeed)
                )
                .build();

        ohShootItMissedInOutOnWarehouseTwoMotionPartOne = robot.drive.trajectoryBuilder(traj[3].end())
                .back(bfDistance,
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.75*scaleSpeed)
                )
                .build();

        ohShootItMissedInOutOnWarehouseTwoMotionPartTwo = robot.drive.trajectoryBuilder(ohShootItMissedInOutOnWarehouseTwoMotionPartOne.end())
                .forward(bfDistance,
                        robot.drive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        robot.drive.getAccelerationConstraint(MAX_ACCEL*1.75*scaleSpeed)
                )
                .build();

        showTrajPoses( "LEVEL3", TIdx, traj ) ;
    }

    private void runTrajs(Trajectory[] traj, SlideHeight level) {
        int TIdx = 0;
        double credits = 30;

        robot.setSlidePosition(SlideHeight.Drive); //Reset Bucket to safe level
        CheckWait(true, 50, 0);
        robot.bucket.setPosition(robot.bucketDrive); //Reset Bucket to drive position
        CheckWait(true, 50, 0);

        //Pick Level based on detected team marker placement
        robot.setSlidePosition(level);

        // Update credits to start.  Time is reset at the very start after pressing 'play'
        credits = credits - runtime.seconds();
        robot.logger.logD("Credits           :",String.format("Current: %2.2f, Elapsed: %2.2f, Credits: %2.2f", runtime.seconds(), 0.0, credits));

        credits = hubAndWarehouse(traj[TIdx++], traj[TIdx++], null, null, credits, 6.0) ;

        credits = hubAndWarehouse(traj[TIdx++], traj[TIdx++], ohShootItMissedInOutOnWarehouseOneMotionPartOne, ohShootItMissedInOutOnWarehouseOneMotionPartTwo, credits, 8.0) ;

        credits = hubAndWarehouse(traj[TIdx++], traj[TIdx++], ohShootItMissedInOutOnWarehouseTwoMotionPartOne, ohShootItMissedInOutOnWarehouseTwoMotionPartTwo, credits, 8.0) ;

        robot.logger.logD("Credits Done     :",String.format("Current: %2.2f, Elapsed: %2.2f, Credits: %2.2f", runtime.seconds(), 0.0, credits));
    }

    private boolean doubleElement(double degrees, boolean test) {
        double aVel = Math.toRadians(288*0.85);
        double aAccel = Math.toRadians(200.0);

        // We picked up two elements, better stop and drop
        if ((robot.colorUpper.getDistance(DistanceUnit.CM) < 2.0) || test) {
            robot.intake_motor.setPower(0);
            robot.bucket.setPosition(robot.bucketDrive);
            robot.setSlidePosition(SlideHeight.Drive);
            CheckWait(true, 500, 0);
            //robot.drive.turnAsync(Math.toRadians(degrees));
            robot.drive.turnAsync(Math.toRadians(degrees)-robot.drive.getRawExternalHeading(), aVel, aAccel);
            CheckWait(true, 0, 0);
            robot.intake_motor.setPower(0.5);
            robot.setSlidePosition(SlideHeight.MidDrop);
            CheckWait(true, 500, 0);
            robot.bucket.setPosition(robot.bucketDump);
            CheckWait(true, 1500, 0);
            robot.intake_motor.setPower(0);
            robot.bucket.setPosition(robot.bucketDrive);
            CheckWait(true, 500, 0);
            robot.setSlidePosition(SlideHeight.Drive);
            CheckWait(true, 500, 0);
            return(true);
        }
        return(false);
    }

    private boolean doubleElementFollowing(double degrees, boolean test) {
        double aVel = Math.toRadians(288*0.85);
        double aAccel = Math.toRadians(200.0);

        // We picked up two elements, better stop and drop
        if ((robot.colorUpper.getDistance(DistanceUnit.CM) < 2.0) || test) {
            abortAuto=true;
            robot.drive.breakFollowing();
            robot.drive.setMotorPowers(0,0,0,0);
            robot.intake_motor.setPower(0);
            robot.bucket.setPosition(robot.bucketDrive);
            robot.setSlidePosition(SlideHeight.Drive);
            CheckWait(true, 500, 0);
            //robot.drive.turnAsync(Math.toRadians(degrees));
            robot.drive.turnAsync(Math.toRadians(degrees)-robot.drive.getRawExternalHeading(), aVel, aAccel);
            CheckWait(true, 0, 0);
            robot.intake_motor.setPower(0.5);
            robot.setSlidePosition(SlideHeight.MidDrop);
            CheckWait(true, 500, 0);
            robot.bucket.setPosition(robot.bucketDump);
            CheckWait(true, 1500, 0);
            robot.intake_motor.setPower(0);
            robot.bucket.setPosition(robot.bucketDrive);
            CheckWait(true, 500, 0);
            robot.setSlidePosition(SlideHeight.Drive);
            CheckWait(true, 500, 0);
            return(true);
        }
        return(false);
    }

    private boolean tooTwisted(Trajectory traj, double imu_offset, double limit) {
        double angleDelta=0;

        // If something bad happened and we are really twisted, stop
        angleDelta = Math.abs(Math.toDegrees(AngleUnit.normalizeRadians((robot.drive.getRawExternalHeading()-imu_offset) - traj.end().getHeading())));
        if(angleDelta > limit) {
            robot.intake_motor.setPower(0);
            robot.bucket.setPosition(robot.bucketDrive);
            robot.setSlidePosition(SlideHeight.Drive);
            return (true);
        }
        return(false);
    }

    private double backAndForth(Trajectory back, Trajectory forth, double credits) {

        double start1 = runtime.seconds();
        double creditsNeeded = 2.5;

        if (!opModeIsActive() || abortAuto) { return(0); }

        // Handle the simple case quickly
        if ((back == null) || (forth == null)) {
            return(credits);
        }

        // Keep trying until we have an element or run out of credits
        while (isIntaking() && (credits > creditsNeeded)) {

            // Keep a timer
            double start2 = runtime.seconds();

            // First motion
            robot.drive.followTrajectoryAsync(back);
            CheckWait(true, 0, 0);
            if (!opModeIsActive() || abortAuto) { return(0); }

            // If we still don't have an element, try to unjam
            if(isIntaking()) {
                robot.intake_motor.setPower(-0.8);
                CheckWait(true, 500, 0);
                robot.intake_motor.setPower(0.6);
            }

            // Second motion
            robot.drive.followTrajectoryAsync(forth);
            CheckWait(true, 0, 0);
            if (!opModeIsActive() || abortAuto) { return(0); }

            // TODO Do we need this?
            // If we still don't have an element, wait a little bit
            if (isIntaking()) {
                CheckWait(true, 300, 0);
            }

            // Update credits (based on time?)
            // credits -= creditsNeeded;
            credits -= runtime.seconds() - start2 ;
        }

        // Print out for debug
        robot.logger.logD("Credits B&F       :",String.format("Current: %2.2f, Elapsed: %2.2f, Credits: %2.2f", runtime.seconds(), runtime.seconds()-start1, credits));

        return(credits);
    }

    private double hubAndWarehouse(Trajectory hub, Trajectory warehouse, Trajectory back, Trajectory forth, double credits, double creditsNeeded) {

        double start1 = runtime.seconds();

        if (!opModeIsActive() || abortAuto) { return(0); }

        // Handle simple case quickly
        if( credits < creditsNeeded ) { return(credits); }

        // Save the credits we'll need to deliver, no need to back and forth if we can't make it to hub.
        // Then add back whatever we didn't use during back and forth
        credits = creditsNeeded + backAndForth(back, forth, (credits - creditsNeeded));

        // Protect against danger
        if(doubleElement(abortTurn, false)) {
            return (0);
        }

        // If we have enough credits and we might have an element
        if((credits > creditsNeeded) && !isIntaking()) {

            double start2 = runtime.seconds();

            // Go to hub
            robot.drive.followTrajectoryAsync(hub);
            CheckWait(true, 0, 0);
            if (!opModeIsActive() || abortAuto) { return(0); }

            // Dump element, reset bucket
            CheckWait(true, 300, 0);
            robot.bucket.setPosition(robot.bucketDrive);

            // Back to warehouse
            robot.drive.followTrajectoryAsync(warehouse);
            CheckWait(true, 0, 0);
            if (!opModeIsActive() || abortAuto) { return(0); }
            //Slide is set to drive during above motion

            // Subtract some credits
            credits -= (runtime.seconds() - start2);

            // If something bad happened and we are really twisted, stop
            if(tooTwisted(warehouse, imu_RR_offset, maxAngleDelta)) {
                return(0);
            }

            // Some telemetry
            robot.logger.logD("Credits H&W Motion:",String.format("Current: %2.2f, Elapsed: %2.2f, Credits: %2.2f", runtime.seconds(), runtime.seconds()-start2, credits));

            // TODO Do we need this?  Remove to save time?
            if (isIntaking()) {
                CheckWait(true, 400, 0);
                // Subtract some more credits
                credits -= 0.4;
            }
        }

        // Some telemetry
        robot.logger.logD("Credits H&W Total :",String.format("Current: %2.2f, Elapsed: %2.2f, Credits %2.2f", runtime.seconds(), runtime.seconds()-start1, credits));
        return(credits);
    }

    private boolean isIntaking() {

        if (robot.intake_motor.getPower() > 0.5) {
            return ( true ) ;
        } else {
            return ( false ) ;
        }
    }

    private void safeBucketDump() {

        if (robot.colorUpper.getDistance(DistanceUnit.CM) < 2.0) {
            return;
        }
        robot.bucket.setPosition(robot.bucketDump);
    }
}

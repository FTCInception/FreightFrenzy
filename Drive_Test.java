package Inception.FreightFrenzy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Drive_Test", group="MechBot")
public class Drive_Test extends LinearOpMode {
    private RRMechBot robot = new RRMechBot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, 0.25);
        robot.initAutonomous(this);
        robot.logger.LOGLEVEL |= robot.logger.LOGDEBUG;

        do {

        } while(!isStarted() && !isStopRequested());
/*
        int squareLen = 20;
        Trajectory[] trajectory = new Trajectory[5];
        trajectory[0] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(0, squareLen))
                .build();
        int[][] xy = { {squareLen, squareLen}, {squareLen, 0}, {0, 0}};
        for(int i = 1; i < 4; i++){
            trajectory[i] = robot.drive.trajectoryBuilder(trajectory[i-1].end())
                    .lineToConstantHeading(new Vector2d(xy[i-1][0],xy[i-1][1]))
                    .build();
        }*/
        int radius = 15;
        Trajectory[] trajectory = new Trajectory[5];
        trajectory[0] = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(0, radius))
                .build();
        int[][] xy = { {radius, 0}, {0, -radius}, {-radius, 0}, {0, radius}};
        double[] circleTangents = { Math.toRadians(270), Math.toRadians(0), Math.toRadians(90), Math.toRadians(180)};
        boolean[] reversed = { false, true, true, false };
        for(int i = 1; i <= 4; i++){
            trajectory[i] = robot.drive.trajectoryBuilder(trajectory[i-1].end(), reversed[i-1])
                    .splineToConstantHeading(new Vector2d(xy[i-1][0],xy[i-1][1]), circleTangents[i-1])
                    .build();
        }


        Pose2d startPose = new Pose2d(0, 0, robot.drive.getRawExternalHeading());
        robot.drive.setPoseEstimate(startPose);
        if(opModeIsActive()){
            for(int i = 0; i <= 4; i++) {
                robot.drive.followTrajectoryAsync(trajectory[i]);
                CheckWait(true, true, 0, 0);
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
}

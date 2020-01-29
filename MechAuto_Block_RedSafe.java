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

package Inception.Skystone;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

@Autonomous(name="Mech: Auto Block RedSafe", group="MechBot")
public class MechAuto_Block_RedSafe extends LinearOpMode {

    private MechBot robot = new MechBot();   // Use a Pushbot's hardware

    private static final double DRIVE_SPEED = 0.8;
    private static final double TURN_SPEED = 0.65;
    private static final double PIVOT_SPEED = 0.40;
    private static final double SQ = 70 / 3.0;        // Length of 3 squares / 3 in case we want to think that way
    private static final double[] blocks = {0.0, 28.0, 36.0, 44.0, 4.0, 12.0, 20.0};
    private static final double dropZone = 92.0;
    private static final double bridge = 80.0;
    private String className = this.getClass().getSimpleName().toLowerCase();

    private IncepVision vision = new IncepVision();
    private int block;
    private double P = 0.05;
    private double a = 0;

    @Override
    public void runOpMode() {
        double firstBlock, secondBlock, thirdBlock, fourthBlock;
        double turnDirection;
        double laneLength;
        double laneLengthWall;
        double laneAdjust1;
        double laneAdjust2;
        int    ideals[]={0, 376, 160, 116};

        //robot.logger.LOGLEVEL = robot.logger.LOGDEBUG ;

        // Init the robot and subsystems
        robot.init(hardwareMap);
        robot.initAutonomous(this);
        vision.initAutonomous(this);

        // Wait until we're told to go and look for the block
        while (!isStarted()) {
            block = vision.getBlockNumber(ideals);
        }
        vision.shutdown();

        // Red or blue alliance changes include:
        // Correction for block numbering
        // Turn direction
        // And some KpX and lane control to handle some unknown red-side drift.
        if (className.contains("blue")) {
            turnDirection = 1.0;
            laneLengthWall = 29;
        } else {
            if (block == 3) {
                block = 1;
            } else if (block == 1) {
                block = 3;
            }
            turnDirection = -1.0;
            laneLengthWall = 28;
        }

        // Wall or block lane -- only difference is a straight distance
        if (className.contains("wall")) {
            laneLength = laneLengthWall;
        } else {
            laneLength = 10.0;
        }

        // Wait for the game to start (driver presses PLAY)
        //waitForStart();

        // Choose the order we grab blocks in.
        if (block == 3) {
            firstBlock=blocks[3];
            secondBlock=blocks[6];
            thirdBlock=blocks[0];
            fourthBlock=blocks[0];

        } else if (block == 2) {
            firstBlock=blocks[2];
            secondBlock=blocks[5];
            thirdBlock=blocks[0];
            fourthBlock=blocks[0];
            robot.KpR *= 1.05;
            robot.KpL *= 1.05;

        } else if (block == 1) {
            //This is the crazy one.  Not doing this is safe mode
            //firstBlock=blocks[1];
            //secondBlock=blocks[4];
            //thirdBlock=blocks[0];
            //fourthBlock=blocks[0];

            // This is functioning single-skystone (2 blocks total)
            firstBlock=blocks[3];
            secondBlock=blocks[1];
            thirdBlock=blocks[0];
            fourthBlock=blocks[0];

        } else {
            firstBlock=blocks[3];
            secondBlock=blocks[1];
            thirdBlock=blocks[0];
            fourthBlock=blocks[0];
        }

        // Crazy -- double block skystone on the wall...
        if ( ( block == 1) && (secondBlock == blocks[4] )  ) {

            double du[] = {.25};

            // This is crazy, but we need more power
            robot.KpR *= 1.10;
            robot.KpL *= 1.10;

            // Separate from wall
            //robot.fastEncoderDrive(DRIVE_SPEED,10, 10, 2, P, du, du );
            a = robot.fastEncoderStraight(DRIVE_SPEED, 28, 3, P);

            robot.straightA = a;
            a = robot.fastEncoderStrafe(DRIVE_SPEED, -4, 1);

            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, 28, 3, P);

            robot.straightA = a;
            a = robot.fastEncoderDrive(DRIVE_SPEED, -2, -2, .25, P, du, du);

            robot.straightA = a;
            a = robot.fastEncoderStrafe(DRIVE_SPEED, 9.5, 1.25);

            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, -26, 3, P);

            robot.grabBlock(400);

            a = robot.gyroRotate(TURN_SPEED, (-90 * turnDirection) - a, 4);

            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, -55, 3, P);

            robot.dropBlock(350);

            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, 46, 3, P);

            a = robot.gyroRotate(TURN_SPEED, (90 * turnDirection) - a, 4);

            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, 26, 3, P);

            robot.straightA = a;
            a = robot.fastEncoderStrafe(DRIVE_SPEED, 34, 2.5);

            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, -40, 2.5, P);

            robot.grabBlock(400);

            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, 11, 3, P);

            robot.straightA = a;
            a = robot.fastEncoderStrafe(DRIVE_SPEED, -12, 1.25);

            a = robot.gyroRotate(TURN_SPEED, (-90 * turnDirection) - a, 4);

            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, -62, 3, P);

            robot.dropBlock(350);

            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, 9, 3, P);

            // Do not extend claw at the end, too much chance of damage
            //robot.grabBlock(1000);

        } else {

            // This is the "normal" flow start for all three blocks
            // Separate from wall
            //robot.fastEncoderDrive(DRIVE_SPEED,10, 10, 2, P, du, du );
            a = robot.fastEncoderStraight(DRIVE_SPEED, 18, 3, P);
            // Do 180
            a = robot.gyroRotate(TURN_SPEED, 180 - a, 4);

            // 'a' is used as an adjustment angle throughout the following code
            // Each step along the way has a target angle returns the delta to that
            // angle after each operation.  We keep track of the err adjustment
            // and ask the next move to handle it so the error shouldn't build.

            robot.straightA = a;
            a = robot.fastEncoderStrafe(DRIVE_SPEED, -turnDirection * (firstBlock - blocks[3]), 3);

            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, -18, 3, P);

            robot.grabBlock(500);

            // Backup and turn towards the drop zone
            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, laneLength, 4, P);

            a = robot.gyroRotate(TURN_SPEED, (90 * turnDirection) - a, 4);

            //go to drop zone
            robot.straightA = a;
            a = robot.fastEncoderStraight(DRIVE_SPEED, firstBlock - dropZone, 4, P);

            robot.dropBlock(350);

            // Middle block near wall is also special on red side
            if (secondBlock == blocks[5]) {

                // Come back to 30 inches from the wall.
                robot.straightA = a;
                a = robot.fastEncoderStraight(DRIVE_SPEED, dropZone - 34.0, 4, P);

                // Turn towards the blocks
                a = robot.gyroRotate(TURN_SPEED, (-90 * turnDirection) - a, 2);
                robot.straightA = a;

                // Push these blocks out of the way to clear room for the claw
                a = robot.fastEncoderStraight(DRIVE_SPEED, -(laneLength + 12), 2.5, P);
                robot.straightA = a;

                a = robot.fastEncoderStraight(DRIVE_SPEED, (laneLength + 12), 2.5, P);
                robot.straightA = a;

                // Srafe to the wall, reduce the timeout in case we hit the wall
                a = robot.fastEncoderStrafe(DRIVE_SPEED, -25, 1.75);
                robot.straightA = a;

                // Drive in and grab the block
                a = robot.fastEncoderStraight(DRIVE_SPEED, -(laneLength), 2, P);
                robot.straightA = a;

                robot.grabBlock(500);

                // Srtafe out to avoid catching on the wall
                a = robot.fastEncoderStrafe(DRIVE_SPEED, 12, 1.5);
                robot.straightA = a;

                // Back out
                a = robot.fastEncoderStraight(DRIVE_SPEED, (laneLength + 2), 2, P);
                robot.straightA = a;

                // Turn back downfield
                a = robot.gyroRotate(TURN_SPEED, (90 * turnDirection) - a, 2);
                robot.straightA = a;

                // Deliver the block, to drop zone (we shoudl be at coordinate 25.0 here)
                a = robot.fastEncoderStraight(DRIVE_SPEED, 25.0 - dropZone, 4, P);
                robot.straightA = a;

                robot.dropBlock(350);

            } else {

                //come back and go for next one
                robot.straightA = a;
                a = robot.fastEncoderStraight(DRIVE_SPEED, dropZone - secondBlock, 4, P);
                a = robot.gyroRotate(TURN_SPEED, (-90 * turnDirection) - a, 4);
                robot.straightA = a;
                a = robot.fastEncoderStraight(DRIVE_SPEED, -(laneLength + 2), 4, P);

                robot.grabBlock(500);

                robot.straightA = a;
                a = robot.fastEncoderStraight(DRIVE_SPEED, laneLength + 2, 4, P);
                a = robot.gyroRotate(TURN_SPEED, (90 * turnDirection) - a, 4);

                //go to drop zone
                robot.straightA = a;
                a = robot.fastEncoderStraight(DRIVE_SPEED, secondBlock - dropZone, 4, P);

                //drop block
                robot.dropBlock(350);
            }

            if (thirdBlock != blocks[0]) {
                //come back and go for next one
                robot.straightA = a;
                a = robot.fastEncoderStraight(DRIVE_SPEED, dropZone - thirdBlock, 4, P);
                a = robot.gyroRotate(TURN_SPEED, (-90 * turnDirection) - a, 4);
                robot.straightA = a;
                a = robot.fastEncoderStraight(DRIVE_SPEED, -(laneLength + 2), 4, P);

                robot.grabBlock(500);

                robot.straightA = a;
                a = robot.fastEncoderStraight(DRIVE_SPEED, laneLength + 2, 4, P);

                a = robot.gyroRotate(TURN_SPEED, (90 * turnDirection) - a, 4);

                //go to drop zone
                robot.straightA = a;
                a = robot.fastEncoderStraight(DRIVE_SPEED, thirdBlock - dropZone, 4, P);

                //drop block
                robot.dropBlock(350);
            }

            //Park under bridge
            a = robot.fastEncoderStraight(DRIVE_SPEED, dropZone - bridge, 4, P);

            // Do not extend claw at the end, too much chance of damage
            //robot.grabBlock(1000);
        }
    }
}

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

@Autonomous(name="Incep: Auto Block Red", group="Incepbot")
public class IncepAuto_Block_Red extends LinearOpMode {

    private IncepBot          robot   = new IncepBot();   // Use a Pushbot's hardware

    private static final double     DRIVE_SPEED             = 0.9;
    private static final double     TURN_SPEED              = 0.65;
    private static final double     PIVOT_SPEED             = 0.40;
    private static final double     SQ                      = 70/3.0;        // Length of 3 squares / 3 in case we want to think that way
    private static final double[] blocks = {0.0, 28.0, 36.0, 44.0, 4.0, 12.0, 20.0};
    private static final double dropZone = 82.0;
    private static final double bridge = 71.0;
    private static double firstBlock, secondBlock, thirdBlock, fourthBlock;
    private static double turnDirection;
    private static double laneLength;
    private String className = this.getClass().getSimpleName().toLowerCase();

    private IncepVision        vision   = new IncepVision();
    private int block;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Init the robot setting for Autonomous play
        robot.initAutonomous(this);

        vision.initAutonomous(this);

        // Wait until we're told to go
        while (!isStarted()) {
            block = vision.getBlockNumber();
        }
        vision.shutdown();

        // Red or blue alliance -- only difference is the turn direction
        // and the block numbering
        if (className.contains("blue")) {
            turnDirection = 1.0;
        } else {
            turnDirection = -1.0;
            if (block == 3) {
                block = 1;
            } else if (block == 1) {
                block = 3;
            }
        }

        // Wall or block lane -- only difference is a straight distance
        if (className.contains("wall")) {
            laneLength = 27.0;
        } else {
            laneLength = 10.0;
        }

        // Wait for the game to start (driver presses PLAY)
        //waitForStart();

        //Set block for testing purposes
        //block = 3;

        if (block == 3) {
            firstBlock=blocks[2];
            secondBlock=blocks[3];
            thirdBlock=blocks[1];
            fourthBlock=blocks[6];

        } else if (block == 2) {
            firstBlock=blocks[2];
            secondBlock=blocks[5];
            thirdBlock=blocks[3];
            fourthBlock=blocks[1];

        } else if (block == 1) {
            firstBlock=blocks[2];
            secondBlock=blocks[1];
            thirdBlock=blocks[3];
            fourthBlock=blocks[6];

        } else {
            firstBlock=blocks[2];
            secondBlock=blocks[1];
            thirdBlock=blocks[3];
            fourthBlock=blocks[6];
        }

        // First Section, doesn't change no matter which block until we get 's' curve
        robot.encoderStraight(DRIVE_SPEED, -32, 3);

        // This is backup code in case we need to fix broken vision for some reason.
        // If block 1 is yellow, we better grab block 3 next (we're getting 2 now.
        // This should reverse for red auto.
        // Enable the LED, sample the color sensor automatically via telemetery update, turn off LED
        //robot.colorSensor.enableLed(true);
        //sleep(100);
        //if (isColorSensorYellow) {
        //   block = 3;
        //}
        //robot.colorSensor.enableLed(false);

        robot.grabBlock();

        // Make the turn
        //robot.gyroPivot( 1.0, 90.0, 6 );
        robot.encoderStraight(DRIVE_SPEED,laneLength,4);
        robot.gyroRotate(TURN_SPEED,90 * turnDirection, 4);

        //Section 2, dependant on whether block is 1,2 or 3.
        //go to drop zone
        //robot.encoderStraight(DRIVE_SPEED,-50, 3);
        robot.encoderStraight(DRIVE_SPEED,firstBlock - dropZone, 4);

        robot.dropBlock();

        //come back and go for next one
        //robot.encoderStraight(DRIVE_SPEED,42, 3);
        robot.encoderStraight(DRIVE_SPEED,dropZone - secondBlock, 4);

        //robot.gyroPivot( -1.0, -90.0, 6 );
        robot.gyroRotate(TURN_SPEED,-90 * turnDirection, 4);
        robot.encoderStraight(DRIVE_SPEED,-(laneLength+2),4);

        robot.grabBlock();

        //robot.gyroPivot( 1.0, 90.0, 6 );
        robot.encoderStraight(DRIVE_SPEED,laneLength+2,4);
        robot.gyroRotate(TURN_SPEED,90 * turnDirection, 4);

        //go to drop zone
        //robot.encoderStraight(DRIVE_SPEED,-42, 3);
        robot.encoderStraight(DRIVE_SPEED, secondBlock - dropZone, 4);

        //drop block
        robot.dropBlock();

        //come back and go for next one
        //robot.encoderStraight(DRIVE_SPEED,42, 3);
        robot.encoderStraight(DRIVE_SPEED,dropZone - thirdBlock, 4);

        //robot.gyroPivot( -1.0, -90.0, 6 );
        robot.gyroRotate(TURN_SPEED,-90 * turnDirection, 4);
        robot.encoderStraight(DRIVE_SPEED,-(laneLength+2),4);

        robot.grabBlock();

        //robot.gyroPivot( 1.0, 90.0, 6 );
        robot.encoderStraight(DRIVE_SPEED,laneLength+2,4);
        robot.gyroRotate(TURN_SPEED,90 * turnDirection, 4);

        //go to drop zone
        //robot.encoderStraight(DRIVE_SPEED,-42, 3);
        robot.encoderStraight(DRIVE_SPEED, thirdBlock - dropZone, 4);

        //drop block
        robot.dropBlock();

        //Park under bridge
        //robot.encoderStraight(DRIVE_SPEED,15, 1.5);
        robot.encoderStraight(DRIVE_SPEED,dropZone - bridge, 4);

        // Extend for parking reach
        robot.grabBlock();

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
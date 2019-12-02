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

@Autonomous(name="Incep: Auto Block Blue", group="Incepbot")
public class IncepAuto_Block_Blue extends LinearOpMode {

    /* Declare OpMode members. */
    private IncepBot          robot   = new IncepBot();   // Use a Pushbot's hardware

    private static final double     DRIVE_SPEED             = 0.9;
    private static final double     TURN_SPEED              = 0.65;
    private static final double     PIVOT_SPEED             = 0.40;
    private static final double     SQ                      = 70/3.0;        // Length of 3 squares / 3 in case we want to think that way
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

        // Wait for the game to start (driver presses PLAY)
        //waitForStart();
 /*
        // Calibration code
        encoderStraight(DRIVE_SPEED,50,13);
        gyroRotate(TURN_SPEED,360,10);
        gyroRotate(TURN_SPEED,360,10);
        encoderStraight(DRIVE_SPEED,-25,4);

        robot.gyroRotate(TURN_SPEED,90, 25);
        sleep(1000);
        robot.gyroRotate(TURN_SPEED,-90, 25);
        sleep(1000);
        robot.gyroRotate(TURN_SPEED,180, 25);
        sleep(1000);
        robot.gyroRotate(TURN_SPEED,-180, 25);
        sleep(1000);
*/

        robot.encoderStraight(DRIVE_SPEED,-32,3);

        // Enable the LED, sample the color sensor automatically via telemetery update, turn off LED
        robot.colorSensor.enableLed(true);
        sleep(100);
        telemetry.update();
        robot.colorSensor.enableLed(false);

        robot.grabBlock();

        // Make the turn
        robot.gyroPivot( 1.0, 90.0, 6 );
        //robot.encoderStraight(DRIVE_SPEED,10,2);
        //robot.gyroRotate(TURN_SPEED,90, 2.5);
        robot.encoderStraight(DRIVE_SPEED,-38, 3);

        robot.dropBlock();

        //come back and go for next one
        robot.encoderStraight(DRIVE_SPEED,46, 3);

        robot.gyroPivot( -1.0, -90.0, 6 );
        //robot.gyroRotate(TURN_SPEED,-90, 2.5);
        //robot.encoderStraight(DRIVE_SPEED,-12,3);

        robot.grabBlock();

        //go to other side
        robot.gyroPivot( 1.0, 90.0, 6 );
        //robot.encoderStraight(DRIVE_SPEED,10,2);
        //robot.gyroRotate(TURN_SPEED,90, 2.5);

        robot.encoderStraight(DRIVE_SPEED,-48, 3);

        //drop block
        robot.dropBlock();

        //come back
        robot.encoderStraight(DRIVE_SPEED,15, 1.5);

        // Extend for parking reach
        robot.grabBlock();

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}

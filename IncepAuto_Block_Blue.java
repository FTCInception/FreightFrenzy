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
            sleep(100);
        }
        vision.shutdown();

        // Wait for the game to start (driver presses PLAY)
        //waitForStart();
 /*
        // Calibration code
        encoderStraight(DRIVE_SPEED,50,13);
        encoderRotate(TURN_SPEED,360,10);
        encoderRotate(TURN_SPEED,360,10);
        encoderStraight(DRIVE_SPEEC,-25,4);
*/

        robot.encoderStraight(DRIVE_SPEED,-32,3);

        // Enable the LED, sample the color sensor automatically via telemetery update, turn off LED
        robot.colorSensor.enableLed(true);
        sleep(100);
        telemetry.update();
        robot.colorSensor.enableLed(false);

        robot.grabBlock();

        //go to other side
        robot.encoderStraight(DRIVE_SPEED,10,2);
        robot.encoderRotate(TURN_SPEED,-90, 2.5);
        robot.encoderStraight(DRIVE_SPEED,-38, 3);

        robot.dropBlock();

        //come back and go for next one
        robot.encoderStraight(DRIVE_SPEED,46, 3);
        robot.encoderRotate(TURN_SPEED,90, 2.5);
        robot.encoderStraight(DRIVE_SPEED,-12,3);

        robot.grabBlock();

        //go to other side
        robot.encoderStraight(DRIVE_SPEED,10,2);
        robot.encoderRotate(TURN_SPEED,-90, 2.5);
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


        /*
        // Old unused code for posterity
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        encoderDrive(DRIVE_SPEED,  24*4,  24*4, 10.0);    // S1: Forward 72 Inches
        encoderRotate(TURN_SPEED,  360, 10.0);                    // S2: Turn Right 1 rotations
        sleep(500);     // pause for servos to move
        encoderRotate(TURN_SPEED,  360, 10.0);                    // S3: Turn Right 1 rotations
        sleep(500);     // pause for servos to move
        encoderRotate(TURN_SPEED,  360, 10.0);                    // S4: Turn Right 1 rotations
        sleep(500);     // pause for servos to move
        encoderRotate(TURN_SPEED,  360, 10.0);                    // S5: Turn Right 1 rotations
        sleep(500);     // pause for servos to move
        encoderDrive(DRIVE_SPEED,  -24*3.5,  -24*3.5, 10.0);  // S6: Backwards 48 Inches

        encoderStraight(37.0,3);
        encoderPivot(180,4);
        encoderStraight(20,2);
        encoderPivot(-90,2);
        encoderStraight(24,2);
        encoderStraight(-70,3);
        encoderRotate(-90,2);
        encoderStraight(24,2);
        encoderPivot(180,3);
        encoderStraight(20,2);
        encoderPivot(-90,2);
        encoderStraight(70,3);
        */

        /*
        *         // This code implements a soft start and soft stop based on time/speed/distance
        *         // It's likely more complicated than necessary.  A distance=only approach is
        *         // better below.  Turns still seem to have some issue in the time/speed/distance.
        *         static final double     SECONDS_TO_FULL_POWER   = 1.5;
        *         static final double     SPEED_RAMP_PER_MS       = (1.0/1000.0) / SECONDS_TO_FULL_POWER;
        *         static final double     SPEED_OFFS              = 0.05;
        *         double lastPos=0;
        *         double lt;
        *         double rem;
        *         double rate;
        *         boolean decel = false;
        *         ElapsedTime     looptime = new ElapsedTime();
        *         looptime.reset()
        *                 if(false) {
        *                     // Record elapsed time
        *                     lt = looptime.milliseconds();
        *                     // Reset the timer for next loop
        *                     looptime.reset();
        *                     // Get current position
        *                     curPos = Math.abs(robot.leftFDrive.getCurrentPosition());
        *                     // How much farther?
        *                     toGo = tgtPos - curPos;
        *                     // Compute remaining time based on encoder rate from last loop
        *                     rate = ((curPos - lastPos) / lt);
        *                     rem = toGo / rate;
        *                     // Save the new position
        *                     lastPos = curPos;
        *                     // If the delta to our target stop speed is > than our deceleration rate, start to slow down
        *                     if ((actSpeed > 0.20) && ((actSpeed - 0.20) > (rem * SPEED_RAMP_PER_MS))) {
        *                         actSpeed = Math.max(actSpeed - (lt * SPEED_RAMP_PER_MS), 0.20);
        *                         robot.leftFDrive.setPower(actSpeed);
        *                         robot.rightFDrive.setPower(actSpeed * (67 / 70.0));
        *                         robot.leftBDrive.setPower(actSpeed);
        *                         robot.rightBDrive.setPower(actSpeed * (67 / 70.0));
        *                         decel = true;
        *                     } else {
        *                         if (!decel && (actSpeed < speed)) {
        *                             // If our speed is below our target and we're not slowing down, speed up
        *                             actSpeed = Math.min(actSpeed + (lt * SPEED_RAMP_PER_MS), speed);
        *                             robot.leftFDrive.setPower(actSpeed);
        *                             robot.rightFDrive.setPower(actSpeed * (67 / 70.0));
        *                             robot.leftBDrive.setPower(actSpeed);
        *                             robot.rightBDrive.setPower(actSpeed * (67 / 70.0));
        *                         }
        *                     }
        *                 }
        *                 //telemetry.addData("Path3",  "Running rate: %3.3f, rem: %7.0f, s: %1.3f", rate, rem, actSpeed);
        */
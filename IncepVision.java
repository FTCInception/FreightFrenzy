/* Copyright (c) 2019 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Image;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CloseableFrame;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.nio.ByteBuffer;
import java.util.List;


/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

public class IncepVision {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static LinearOpMode myLOpMode;
    private int BlockNumber;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AfWZ0Nj/////AAABma9i7nGZSk81hrDHleShtMuKJES27HbNIQandd3JejLnjvR3256AZU4KbwLKM3zRbhT54zvMHzIwofU7N0TwRifRjMB9sPJ/GZoVpvrcOTNl0F3G6ynufbSkLWWRAGzf3ffMAWeB97a8iF/fPSC5kYY7u56rj2IXVXw7zB2GrTIlFIgkGmy+faJST+4838yCmE4kZFqSc8qnKW1zG0qh9EhMdg8KobZkODSkG2r2uDHXEcvnD8zLKQMIZGm3ueWs1aWvJRZZgx6wDFr1LFnnzZDdJ1en1TjkVWt7Mv+pb8j+9j/9W7Fp4Q5yUrqDl64aeNe7pLplamMYlZXBSOmevv/4r+h6SdQKeimUeP5dCZ6m";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    /* init function for Autonomous mode
     *
     */
    public void initAutonomous(LinearOpMode lOpMode) {
        myLOpMode = lOpMode;
        BlockNumber = 0;

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            myLOpMode.telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        myLOpMode.telemetry.addData(">", "Vision Code Initialized");
        myLOpMode.telemetry.update();
        //myLOpMode.waitForStart();
    }



    //below function returns skystone blocks
    public int getBlockNumber() {
        int column;
        int left;
        int boundary;
        int center;
        int step = 4;
        int windowwidth = 16;
        float threshold = 0x50 * windowwidth;
        int PixelIndex;
        int Thirds;

        int bytes_per_pixel = -1;
        int row = -1;

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    myLOpMode.telemetry.addData("Here 1", "");
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        myLOpMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        Recognition skystone_rec = null;
                        for (Recognition recognition : updatedRecognitions) {
                        /*myLOpMode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        //below will give the left, top, right, bottom but it is disabled because of unnecessary feedback
                        myLOpMode.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                          recognition.getLeft(), recognition.getTop());
                        myLOpMode.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());*/

                            if(recognition.getLabel().contentEquals("Skystone")) {
                                skystone_rec = recognition;
                            }
                        }

                        // get Frame
                        CloseableFrame myFrame = vuforia.getFrameQueue().poll();

                        if ((myFrame != null) && (skystone_rec != null)) {
                            myLOpMode.telemetry.addData("Here 2", "");
                            Image myImage = myFrame.getImage(0);
                            myLOpMode.telemetry.addData("Feedback", "Buffer height and width %d %d", myImage.getBufferHeight(), myImage.getBufferWidth());

                            ByteBuffer myBuffer = myImage.getPixels();
                            bytes_per_pixel = myImage.getStride() / myImage.getBufferWidth();
                            row = ((int) (skystone_rec.getTop()) + (int) (skystone_rec.getBottom())) / 4;

                            if (row < 0) {
                                row = 0;
                            }

                            if (row > (myImage.getBufferHeight() - 1)) {
                                row = myImage.getBufferHeight() - 1;
                            }

                            column = (int) skystone_rec.getLeft() / 2;
                            left = (int) skystone_rec.getLeft() / 2;
                            boundary = (int) skystone_rec.getRight() / 2;
                            center = (int) myImage.getBufferWidth() / 2;
                            PixelIndex = row * myImage.getBufferWidth() + column;
                            Thirds = skystone_rec.getImageWidth() / 2 / 3;


                            Boolean edgeFound = false;
                            while(!edgeFound) {
                                myLOpMode.telemetry.addData("Here 3", "");
                                int sum = 0;

                                for(int index=0; index < windowwidth; index++) {
                                    sum += myBuffer.getChar(PixelIndex + index) & 0x00ff;
                                }
                                PixelIndex += step;
                                column += step;
                                boolean beyondBoundary = false;
                                if (column > boundary) {
                                    edgeFound = true;
                                    beyondBoundary = true;

                                    myLOpMode.telemetry.addData("Warning: ", "code is looking for skystone past boundary");
                                }
                                if (sum <= threshold) {
                                    edgeFound = true;
                                }
                                if (edgeFound == true && beyondBoundary == false) {
                                    myLOpMode.telemetry.addData("sum, Pixel Index, column, found edge, offset from center", "%d %d %d %b %d %d", sum, PixelIndex, column, edgeFound, column - center, column - left);
                                }
                            }
                            int dist = column - left;

                            if (dist >= (2 * Thirds)) {
                                BlockNumber = 1;
                            }else if (dist >= Thirds) {
                                BlockNumber = 2;
                            }else {
                                BlockNumber = 3;
                            }

                            // DEBUG Output

                            myLOpMode.telemetry.addData("Feedback", "Row %d", row);
                            myLOpMode.telemetry.addData("Feedback", "Pixel bytes %d", bytes_per_pixel);
                            myLOpMode.telemetry.addData("Block Number: ", "%d", BlockNumber);
                            myLOpMode.telemetry.update();

                        }else if (myFrame == null) {
                            myLOpMode.telemetry.addData("Note: ", "No Frame");
                            myLOpMode.telemetry.update();
                        } else {
                            myLOpMode.telemetry.update();
                        }
                    }
                }

        //myLOpMode.telemetry.addData("Feedback", "Row %d", row);
        //myLOpMode.telemetry.addData("Block Number: ", "%d", BlockNumber);
        //myLOpMode.telemetry.update();
        //will the return the block number
        return BlockNumber;
    }

    public void shutdown() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();



        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        //Get Frame
        vuforia.setFrameQueueCapacity(1);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = myLOpMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", myLOpMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}

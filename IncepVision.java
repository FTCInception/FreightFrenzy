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
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CloseableFrame;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaException;

import android.graphics.Color;
import android.app.Activity;
import android.view.View;

import java.nio.ByteBuffer;
import java.util.List;
import java.util.concurrent.TimeUnit;


/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

public class IncepVision {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static LinearOpMode myLOpMode;
    private int BlockNumber;
    private int bytes_per_pixel;
    private int row;
    private int column;
    private int botGrade = Color.RED;
    private int bufWidth;
    private int bufHeight;
    private int recWidth;
    private int recHeight;
    private int step;
    private int windowwidth;
    private int threshold;
    private int left, boundary, top, bottom;
    private Boolean edgeFound;
    private int sum;
    private int pixelArray[];
    private double BoundOffset;
    private int vScale;
    private Recognition skystone_rec;
    private int pixelFormat;
    private int framefound;
    // get a reference to the RelativeLayout so we can change the
    // background color of the Robot Controller
    int relativeLayoutId;
    View relativeLayout;

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

        // constants
        step = 4;
        windowwidth = 16;
        // Turn the threshold down just a little as we were triggering on the gap
        // between blocks and sometimes the mat.
        threshold = 0x40 * windowwidth;

        BlockNumber = 0;
        bytes_per_pixel = -1;
        row = -1;
        column = -1;
        botGrade = Color.RED;
        bufWidth = -1;
        bufHeight = -1;
        recWidth = -1;
        recHeight = -1;
        left = -1;
        boundary = -1;
        top = -1;
        bottom = -1;
        sum = 0;
        edgeFound = false;
        pixelArray = new int[windowwidth];
        BoundOffset = 0;
        vScale = 1;
        Recognition skystone_rec = null;
        pixelFormat = 0;
        framefound = 0;


        for (int i = 0; i < windowwidth; i++) {
            pixelArray[i] = 0;
        }
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

        // get a reference to the RelativeLayout so we can change the
        // background color of the Robot Controller
        relativeLayoutId = lOpMode.hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", lOpMode.hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) lOpMode.hardwareMap.appContext).findViewById(relativeLayoutId);

        /** Wait for the game to begin */
        myLOpMode.telemetry.addData(">", "Vision Code Initialized");
        myLOpMode.telemetry.update();
        //myLOpMode.waitForStart();
    }


    //below function returns skystone blocks
    public void RestoreWhite() {
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }


    public int getBlockNumber() {

        int ideals[] = {0, 468, 252, 208};

        return (getBlockNumber(ideals));
    }


    //below function returns skystone blocks
    public int getBlockNumber( int[] ideals ) {
        int center;
        int ideal=0;

        int PixelIndex;
        int Third;
        edgeFound = false;

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            //List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            List<Recognition> updatedRecognitions = tfod.getRecognitions();


            if (updatedRecognitions != null) {
                // step through the list of recognitions and display boundary info.
                int i = 0;
                skystone_rec = null;
                for (Recognition recognition : updatedRecognitions) {
                            /*myLOpMode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            //below will give the left, top, right, bottom but it is disabled because of unnecessary feedback
                            myLOpMode.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                          recognition.getLeft(), recognition.getTop());
                            myLOpMode.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());*/

                    // Choose highest confidence
                    if ((skystone_rec == null) || (recognition.getConfidence() > skystone_rec.getConfidence())) {
                        skystone_rec = recognition;
                    }
                }
                if (updatedRecognitions.size() > 0) {
                    myLOpMode.telemetry.addData("# Object Detected", "%d (%.2f)", updatedRecognitions.size(), skystone_rec.getConfidence());
                } else {
                    myLOpMode.telemetry.addData("# Object Detected", "%d", updatedRecognitions.size());
                }

                // get Frame
                CloseableFrame myFrame;
                try {
                     myFrame = vuforia.getFrameQueue().poll(100, TimeUnit.MILLISECONDS);
                } catch(Exception e) {
                    //throw new RuntimeException(e);
                    myFrame = null;
                }

                framefound = 0;
                if (myFrame != null) framefound = 1;

                if ((myFrame != null) && (skystone_rec != null)) {

                    // Image stats
                    Image myImage = myFrame.getImage(0);
                    bufWidth = myImage.getBufferWidth();
                    bufHeight = myImage.getBufferHeight();
                    //center = (int) myImage.getBufferWidth() / 2;

                    recWidth = skystone_rec.getImageWidth();
                    recHeight = skystone_rec.getImageHeight();

                    ByteBuffer myBuffer = myImage.getPixels();
                    bytes_per_pixel = myImage.getStride() / myImage.getBufferWidth();

                    // discover how the pixels are stored
                    pixelFormat = myImage.getFormat();

                    // Shrink the sampel window just a little to make the block lineup more tolerant.
                    BoundOffset = bufWidth * 0.1125;

                    // object stats.
                    //int hScale = skystone_rec.getImageWidth() / bufWidth;
                    vScale = skystone_rec.getImageHeight() / bufHeight;

                            /*column   = (int) skystone_rec.getLeft() / hScale; //starts at left edge
                            left     = (int) skystone_rec.getLeft() / hScale; // left edge
                            boundary = (int) skystone_rec.getRight() / hScale; // right edge*/
                    left = (int) BoundOffset;
                    column = left;

                    boundary = (int) (bufWidth - BoundOffset); // right edge
                    top = (int) skystone_rec.getTop() / vScale; // top of detected skystone
                    bottom = (int) skystone_rec.getBottom() / vScale; // bottom of detected skystone

                    Third = (boundary - left) / 3;

                    row = ((int) (top) + (int) (bottom)) / 2; // halfway between the top and the bottom of the detected block object

                    if (row < 0) {
                        row = 0;
                    }

                    if (row > (bufHeight - 1)) {
                        row = bufHeight - 1;
                    }

                    // debug: grab first few pixels on left edge
                    PixelIndex = (row * bufWidth * bytes_per_pixel) + column * bytes_per_pixel;
                    for (int index = 0; index < windowwidth; index++) {
                        if (pixelFormat == PIXEL_FORMAT.GRAYSCALE) { // GRAYSCALE = 4
                            pixelArray[index] = myBuffer.getChar(PixelIndex + index * bytes_per_pixel); // & 0x00ff; // debug
                        } else if (pixelFormat == PIXEL_FORMAT.RGB565) { // RGB565 = 1
                            // Mask out the green bits
                            // right shift out the blue bits
                            // multiply by 4 since this is 6 bits and not a full 8
                            char pixelChar = myBuffer.getChar(PixelIndex + index * bytes_per_pixel);
                            double red = ((pixelChar & 0xf800) >> 11) << 3; // 5 bits to 8
                            double green = ((pixelChar & 0x07e0) >> 5) << 2; // 6 bits to 8
                            double blue = ((pixelChar & 0x001F)) << 3; // 5 bits to 8;
                            double gray = 0.2126 * red + 0.7152 * green + 0.0722 * blue;

                            sum += (int) (gray) & 0xff;
                        }
                    }

                    // Find the skystone black edge from left to right
                    edgeFound = false;
                    Boolean done = false;
                    while (!done) {
                        sum = 0;
                        PixelIndex = (row * bufWidth * bytes_per_pixel) + column * bytes_per_pixel; // column updates every loop - must update pixelIndex
                        if (pixelFormat == PIXEL_FORMAT.GRAYSCALE) { // GRAYSCALE = 4
                            for (int index = 0; index < windowwidth; index++) {
                                sum += myBuffer.getChar(PixelIndex + index * bytes_per_pixel) & 0x00ff;
                            }
                        } else if (pixelFormat == PIXEL_FORMAT.RGB565) { // RGB565 = 1
                            for (int index = 0; index < windowwidth; index++) {
                                // Mask out the green bits
                                // right shift out the blue bits
                                // multiply by 4 since this is 6 bits and not a full 8
                                char pixelChar = myBuffer.getChar(PixelIndex + index * bytes_per_pixel);
                                double red = ((pixelChar & 0xf800) >> 11) << 3; // 5 bits to 8
                                double green = ((pixelChar & 0x07e0) >> 5) << 2; // 6 bits to 8
                                double blue = ((pixelChar & 0x001F)) << 3; // 5 bits to 8;
                                double gray = 0.2126 * red + 0.7152 * green + 0.0722 * blue;

                                sum += (int) (gray) & 0xff;
                            }
                        }

                        //PixelIndex += step;
                        column += step;

                        // if column is more than the right edge of the object
                        if (column > boundary) {
                            done = true;
                        }

                        if (sum <= threshold) {
                            edgeFound = true;
                            done = true;
                        }

                    }

                    // If edge is found, calc the block number
                    if (edgeFound) {
                        int dist = column - left;

                        //if (dist >= (2 * Third)) {
                        // Detecting block 1 only has about 1/2"-1" tolerance. Give a little more tolerance
                        if (dist >= ((3 * Third)/2)) {
                            BlockNumber = 1;
                        } else if (dist >= (Third / 2)) {
                            BlockNumber = 2;
                        } else {
                            BlockNumber = 3;

                            column = boundary - windowwidth;
                            // Find the skystone black edge from right to left to help with alignment
                            done = false;
                            while (!done) {
                                sum = 0;
                                PixelIndex = (row * bufWidth * bytes_per_pixel) + column * bytes_per_pixel; // column updates every loop - must update pixelIndex
                                if (pixelFormat == PIXEL_FORMAT.GRAYSCALE) { // GRAYSCALE = 4
                                    for (int index = 0; index < windowwidth; index++) {
                                        sum += myBuffer.getChar(PixelIndex + index * bytes_per_pixel) & 0x00ff;
                                    }
                                } else if (pixelFormat == PIXEL_FORMAT.RGB565) { // RGB565 = 1
                                    for (int index = 0; index < windowwidth; index++) {
                                        // Mask out the green bits
                                        // right shift out the blue bits
                                        // multiply by 4 since this is 6 bits and not a full 8
                                        char pixelChar = myBuffer.getChar(PixelIndex + index * bytes_per_pixel);
                                        double red = ((pixelChar & 0xf800) >> 11) << 3; // 5 bits to 8
                                        double green = ((pixelChar & 0x07e0) >> 5) << 2; // 6 bits to 8
                                        double blue = ((pixelChar & 0x001F)) << 3; // 5 bits to 8;
                                        double gray = 0.2126 * red + 0.7152 * green + 0.0722 * blue;

                                        sum += (int) (gray) & 0xff;
                                    }
                                }

                                //PixelIndex += step;
                                column -= step;

                                // if column is less than the left edge of the object
                                if (column < left) {
                                    done = true;
                                }

                                if (sum <= threshold) {
                                    done = true;
                                }
                            }
                        }
                    }
                    //myFrame.close();
                }
            }
        }

        // Experimentally determine ideal columns for each configuration
        // just in case the referees don't setup the blocks in the center position.
        if (BlockNumber == 1) {
            ideal = ideals[1];
        } else if (BlockNumber == 2) {
            ideal = ideals[2];
        } else if (BlockNumber == 3) {
            ideal = ideals[3];
        } else {
            ideal = ideals[0];
        }

        // Now create color ranges
        // Our vision code only returns columns in 4 pixel alignments so +/-7 is really about as
        // close as possible.  +/-15 is probably within 1 inch of center.
        if ( Math.abs(ideal - column) < 7 ) {
            botGrade = Color.GREEN;
        } else if ( Math.abs(ideal - column) < 15 ) {
            botGrade = Color.YELLOW;
        } else {
            botGrade = Color.RED;
        }

        // Leveraged from sample code from MR Gyro example
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(botGrade);
            }
        });

        myLOpMode.telemetry.addData(">", "Col: %d (%d) (%d)", column, ideal, framefound);
        myLOpMode.telemetry.addData(">", "Block Number: %d", BlockNumber);
        //myLOpMode.telemetry.addData(">", "Buffer(WxH): (%d x %d) (%d x %d)", bufWidth, bufHeight, recWidth, recHeight);
        //myLOpMode.telemetry.addData(">", "BPP: %d pf: %d, Edge Found: %b %x %x %x %x", bytes_per_pixel, pixelFormat, edgeFound, pixelArray[0], pixelArray[1], pixelArray[2], pixelArray[3]);
        //myLOpMode.telemetry.addData("Top Bottom Left Right ", "%d %d %d %d", top, bottom, left, boundary);
        //myLOpMode.telemetry.addData(">", "Row, vScale: %d, %d", row, vScale);
        //myLOpMode.telemetry.addData(">", "Threshold: %d  sum: %d", threshold, sum);
        //myLOpMode.telemetry.addData("updatedrecognitions size", "%d", tfod.getRecognitions().size());
        myLOpMode.telemetry.update();

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

        CameraName savedName = parameters.cameraName;
        try {
            parameters.cameraName = myLOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        } catch (Exception e) {
            parameters.cameraName = savedName;
            parameters.cameraDirection = CameraDirection.BACK;
        }

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);


        //Get Frame
        vuforia.setFrameQueueCapacity(1);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = myLOpMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", myLOpMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        // This matches the .1 bound above.
        // FIXME: make the boundary offset and this match for sure
        tfod.setClippingMargins(72,125,72,125);
    }
}
/*
code will auto detect if a webcam is connected or not
example code on how to use:
package Inception.Skystone;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Incep: Vision Test 2", group="Incepbot")
public class (class goes here) extends LinearOpMode {

    private IncepVision        vision   = new IncepVision();


    @Override
    public void runOpMode() {
    //starts the code only needs to be run once
        vision.initAutonomous(this);

        //runs during init
        while(!isStopRequested())
        {
            vision.getBlockNumber();
        }
        //stops vision code fully
        vision.shutdown();
    }
}

 */

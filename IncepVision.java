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
package Inception.UltimateGoal;

import android.app.Activity;
import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.Arrays;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.view.View;

/*
 * This is a modified version of the vuforia code by G-FORCE that uses Vuforia to capture images from the camera
 * Leveraged from here:
 * https://gist.github.com/brandonwang1/d5c02f64d8ab05e2dc8c18c7ba4e199a
 */

public class IncepVision {

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.  Alt. is BACK
    //private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.FRONT;

    /* Private class members. */
    private LinearOpMode myLOpMode;       // Access to the LinearOpMode object
    public VuforiaLocalizer vuforia;     // The localizer
    public TFObjectDetector tfod;
    private static final String VUFORIA_KEY = "AfWZ0Nj/////AAABma9i7nGZSk81hrDHleShtMuKJES27HbNIQandd3JejLnjvR3256AZU4KbwLKM3zRbhT54zvMHzIwofU7N0TwRifRjMB9sPJ/GZoVpvrcOTNl0F3G6ynufbSkLWWRAGzf3ffMAWeB97a8iF/fPSC5kYY7u56rj2IXVXw7zB2GrTIlFIgkGmy+faJST+4838yCmE4kZFqSc8qnKW1zG0qh9EhMdg8KobZkODSkG2r2uDHXEcvnD8zLKQMIZGm3ueWs1aWvJRZZgx6wDFr1LFnnzZDdJ1en1TjkVWt7Mv+pb8j+9j/9W7Fp4Q5yUrqDl64aeNe7pLplamMYlZXBSOmevv/4r+h6SdQKeimUeP5dCZ6m";
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_QUAD = "Quad";
    private static final String LABEL_SINGLE = "Single";
    public static int clipLeft=280;
    public static int clipTop=260;
    public static int clipRight=260;
    public static int clipBottom=155;
    public boolean clip=false;
    public boolean tfodState=false;
    private int ringCount = -1 ;
    private String webcamName;
    public final int NONE=0, BLUE_INSIDE=1, BLUE_OUTSIDE=2, RED_INSIDE=3, RED_OUTSIDE=4;
    private final int iTOP=0, iBOTTOM=1, iLEFT=2, iRIGHT=3;
    public int auto= NONE;
    final int[][] defStack = {
            // top, bottom, left, right
              {189,    217,  252,   285},  // NONE
              {189,    212,  301,   224},  // BLUE_INSIDE
              {185,    221,  264,   273},  // BLUE_OUTSIDE
              {185,    216,  203,   328},  // RED_INSIDE
              {190,    219,  255,   283}}; // RED_OUTSIDE
    /***
     * Initialize the Target Tracking and navigation interface
     * @param lOpMode    pointer to OpMode
     */
    public void initAutonomous(LinearOpMode lOpMode, String webcamName, int myAuto) {

        // Save reference to OpMode and Hardware map
        myLOpMode = lOpMode;
        webcamName = webcamName;
        auto = myAuto;
        setDefStack(auto);

        initVuforia( webcamName );

        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfodState=true;
        }
    }

    public void initAutonomous(LinearOpMode lOpMode, String webcamName) {
        initAutonomous(lOpMode, webcamName, NONE);
    }

    public void setDefStack(int auto) {
        clipTop = defStack[auto][iTOP];
        clipLeft = defStack[auto][iLEFT];
        clipRight = defStack[auto][iRIGHT];
        clipBottom = defStack[auto][iBOTTOM];
    }

    public void setDefStack() {
        setDefStack(auto);
    }

    public void initVuforia(String webcamName) {

        // This line allows the output of Vuforia to be connected to the Camera Stream window of the DS
        // We need to get the clipping effect to help align the rings in our frame so we can't use this directly
        // because we don't know how to make Vuforia do the right thing.  But it's interesting anyway
        //int viewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier(
        //        "tfodMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(viewId);  // Use this line to see camera display

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.cameraName = myLOpMode.hardwareMap.get(WebcamName.class, webcamName);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        vuforia.setFrameQueueCapacity(1); // Keeps the most recent image captured by the camera
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); // Sets the image format to RGBA565

    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    public void initTfod() {
        // This finds the ID of the DS camera stream window
        int tfodMonitorViewId = myLOpMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", myLOpMode.hardwareMap.appContext.getPackageName());

        // This creates a 'params' object servicing the DS camera stream with 0.6 confidence
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = (float)(0.4);

        // Create the object
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Load the model
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_QUAD, LABEL_SINGLE);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_QUAD);

        // bound the window down to help with ring alignment
        if(clip) {
            tfod.setClippingMargins(clipLeft, clipTop, clipRight, clipBottom);
        }
    }

    public void processImage( Image image ) {

        int bufWidth = image.getBufferWidth();
        int bufHeight = image.getBufferHeight();

        Bitmap bitmap = Bitmap.createBitmap(image.getWidth(), image.getHeight(),
                Bitmap.Config.RGB_565);
        bitmap.copyPixelsFromBuffer(image.getPixels());

        // I think x,y is 0,0 in the top,left corner of the image
        // x goes left to right
        // y goes top to bottom

        // It's imperative that the rings are aligned properly in the frame in the clip box.
        // Divide the clipping region in 10 slices.
        // Compute the average value of the pixel inside each slice of the center 80%.
        // Extend each slice to the right and left of the clipping box by 40% of box width.
        // Compute the average value of the pixel in these wings.
        // The wings should represent a reference value of gray for each slice.
        // Compare the wings and the rings and see which slices look like they have ring.

        // Slice the clipped image into 8 slices and check each slice.
        final double NUM_SLICES = 8.0;
        final int RED=0, GREEN=1, BLUE=2;
        int x, y, sIdx, color;

        int[][] sliceColor = new int[3][(int)NUM_SLICES];
        int[][] gndColor = new int[3][(int)NUM_SLICES];
        int pixCount=0;
        int gndCount=0;
        int[] isRing = new int[(int)NUM_SLICES];
        //int[] isRing2 = new int[(int)NUM_SLICES];

        int ringTop    = clipTop ;
        int ringBottom = bufHeight - clipBottom ;
        double sliceHeight = ((ringBottom - clipTop) / NUM_SLICES) ;
        int ringRight  = bufWidth - clipRight ;
        int ringLeft   = clipLeft ;
        int ringWidth = ringRight - ringLeft ;

        // FIXME -- We need to bounds-check our x and y against the edge of the image
        // Maybe add some telemetry to say something is suspicious if the box
        // is very far outside our expectations?
        //myLOpMode.telemetry.addData("bufsize", "w: %d x h: %d", bitmap.getWidth(),bitmap.getHeight());
        //myLOpMode.telemetry.addData("clipping", "%d, %d, %d, %d", ringTop, ringBottom, ringRight, ringLeft);
        //myLOpMode.telemetry.update();

        // For each slice
        for ( sIdx = 0 ; sIdx < NUM_SLICES ; sIdx++ ) {
            pixCount=0;
            gndCount=0;
            for ( y = (int) ( ringBottom - ( sliceHeight * ( sIdx+1 ) ) ); y < ( ringBottom - ( sliceHeight * sIdx ) ) ; y++ ) {
                // Wing on the left is 30% of the ring width
                for ( x = (int) (ringLeft-(ringWidth*0.50)) ; x > (ringLeft-(ringWidth*0.20))  ; x-- ) {
                    color = bitmap.getPixel(x, y);
                    gndColor[RED][sIdx] += Color.red(color);
                    gndColor[GREEN][sIdx] += Color.red(color);
                    gndColor[BLUE][sIdx] += Color.red(color);
                    gndCount++;
                }
                // Use center 60% of the ring
                for ( x = (int) (ringLeft+(ringWidth*0.20)) ; x < (ringRight-(ringWidth*0.20)) ; x++ ) {
                    color = bitmap.getPixel(x, y);
                    sliceColor[RED][sIdx] += Color.red(color);
                    sliceColor[GREEN][sIdx] += Color.green(color);
                    sliceColor[BLUE][sIdx] += Color.blue(color);
                    pixCount++;
                }
                // Wing on the right is 30% of the ring width
                for( x = (int) (ringRight+(ringWidth*0.20)) ; x < (ringRight+(ringWidth*0.50)) ; x++ ) {
                    color = bitmap.getPixel(x, y);
                    gndColor[RED][sIdx] += Color.red(color);
                    gndColor[GREEN][sIdx] += Color.red(color);
                    gndColor[BLUE][sIdx] += Color.red(color);
                    gndCount++;
                }
            }

            // Get the averages
            if ( gndCount != 0 ) {
                gndColor[RED][sIdx] /= gndCount;
                gndColor[GREEN][sIdx] /= gndCount;
                gndColor[BLUE][sIdx] /= gndCount;
            }

            if ( pixCount != 0 ) {
                sliceColor[RED][sIdx] /= pixCount;
                sliceColor[GREEN][sIdx] /= pixCount;
                sliceColor[BLUE][sIdx] /= pixCount;
            }

             // Do a straight-up compare of average blue.  No need to normalize
            if ( sliceColor[BLUE][sIdx] < (gndColor[BLUE][sIdx]/2.0) ) {
                isRing[sIdx]=1;
            } else {
                isRing[sIdx]=0;
            }

            /*
            // Now check the average for each slice
            // What's the relative percentage of blue compared to the other colors?
            // If the ground has more than twice the blue of the ring, then it's probably a ring
            double gndBlue, sliceBlue;
            gndBlue = (double)gndColor[BLUE][sIdx] / (double)(gndColor[RED][sIdx]+gndColor[GREEN][sIdx]+gndColor[BLUE][sIdx]) ;
            sliceBlue = (double)sliceColor[BLUE][sIdx] / (double)(sliceColor[RED][sIdx]+sliceColor[GREEN][sIdx]+sliceColor[BLUE][sIdx]) ;
            if ( sliceBlue < (gndBlue/2.0) ) {
                isRing[sIdx]=1;
            } else {
                isRing[sIdx]=0;
            }
            */
        }

        // FIXME: Check the blue average in the top and bottom slices to the middle?  See if they need to be adjusted. Print to telemetry?

        //ringSlices is how many slices we detect as ring
        //ringCount is the actual amount of rings present
        // Count the ring slices
        int ringSlices = 0 ;
        for(int i = 0; i<isRing.length; i++){
            if(isRing[i]==1){
                ringSlices++;
            }
        }

        // Convert to the number of actual rings
        // The bottom ring is viewed from above, so it appears to be much larger in the bitmap
        // than it actually is.  In fact, it coves 4 slices.  So we wil only call this '4' rings
        // when we are REALLY sure (>=6 slices.)
        if (ringSlices >= 6){
            ringCount = 4;
        } else if (ringSlices > 0){
            ringCount = 1;
        } else {
            ringCount = 0;
        }

        // Now display everything we learned.
        myLOpMode.telemetry.addData("box", "T %3s, B %3d, L %3d, R %3d, <-- %s/%s", clipTop, clipBottom, clipLeft, clipRight, tfodState?"Updating":"Frozen",clip?"Clipped":"Unclipped");

        myLOpMode.telemetry.addData("gb", "%s", Arrays.toString(gndColor[BLUE]));
        myLOpMode.telemetry.addData("sb", "%s", Arrays.toString(sliceColor[BLUE]));
        myLOpMode.telemetry.addData("isRing", "%s", Arrays.toString(isRing));
        myLOpMode.telemetry.addData("ringCount", "%d", ringCount);

        myLOpMode.telemetry.update();
    }

    public int countRings() {

        if (tfod != null) {

            if(tfodState) {
                List<Recognition> Recognitions = tfod.getRecognitions();
                if (Recognitions != null) {
                    for (Recognition recognition : Recognitions) {
                        if (recognition.getLabel().equals(LABEL_QUAD)) {
                            // TensorFow is always capturing just a little more on the top of the
                            // ring stack than we'd like.  Lets help it a little
                            clipTop = (int) (recognition.getTop() + (recognition.getHeight() * 0.15));
                            clipBottom = 480 - (int) recognition.getBottom();
                            clipLeft = (int) recognition.getLeft();
                            clipRight = 640 - (int) recognition.getRight();
                        }
                    }
                }
            }

            VuforiaLocalizer.CloseableFrame frame;
            try {
                frame = vuforia.getFrameQueue().poll(100, TimeUnit.MILLISECONDS);
            } catch(Exception e) {
                frame = null;
            }

            if (frame != null) {
                long numImgs = frame.getNumImages();
                for (int i = 0; i < numImgs; i++) {
                    if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                        //Log.d("Vuforia", "Success");
                        if (clip == true) {
                            tfod.setClippingMargins(clipLeft,clipTop,clipRight,clipBottom);
                        } else{
                            tfod.setClippingMargins(0, 0, 0, 0);
                        }
                        try {
                            processImage(frame.getImage(i));
                        } catch(Exception e) {
                            myLOpMode.telemetry.addData("TENSOR FLOW BARFED -- You need to align the camera or 'freeze' and manually align","");
                            setDefStack();
                            myLOpMode.telemetry.update();
                        }
                    }
                }
            }
        }

        return ringCount;
    }

    public void shutdown() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}
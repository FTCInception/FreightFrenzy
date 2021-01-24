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
    public static int imageLeft=280;
    public static int imageTop=260;
    public static int imageRight=260;
    public static int imageBottom=155;
    public boolean clip=false;
    public boolean tfodState=false;
    // get a reference to the RelativeLayout so we can change the
    // background color of the Robot Controller
    int relativeLayoutId;
    View relativeLayout;

    /***
     * Initialize the Target Tracking and navigation interface
     * @param lOpMode    pointer to OpMode
     */

    public void initAutonomous(LinearOpMode lOpMode) {

        // Save reference to OpMode and Hardware map
        myLOpMode = lOpMode;

        initVuforia();

        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfodState=true;
        }

        // We really only want tensor flow loaded just so we can see the clipped image on the DS.
        //tfod.deactivate();

        // get a reference to the RelativeLayout so we can change the
        // background color of the Robot Controller
        relativeLayoutId = myLOpMode.hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", myLOpMode.hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) myLOpMode.hardwareMap.appContext).findViewById(relativeLayoutId);

    }

    public void initVuforia() {

        // This line allows the output of Vuforia to be connected to the Camera Stream window of the DS
        // We need to get the clipping effect to help align the rings in our frame so we can't use this directly
        // because we don't know how to make Vuforia do the right thing.  But it's interesting anyway
        //int viewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier(
        //        "tfodMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(viewId);  // Use this line to see camera display

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.cameraName = myLOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");

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

        // Create the object
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Load the model
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_QUAD, LABEL_SINGLE);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_QUAD);

        // bound the window down to help with ring alignment
        if(clip) {
            tfod.setClippingMargins(imageLeft, imageTop, imageRight, imageBottom);
        }
    }

    public int processImage( Image image ) {

        int bufWidth = image.getBufferWidth();
        int bufHeight = image.getBufferHeight();

        Bitmap bitmap = Bitmap.createBitmap(image.getBufferWidth(), image.getBufferHeight(),
                Bitmap.Config.RGB_565);
        bitmap.copyPixelsFromBuffer(image.getPixels());

        // Optionally create a few smaller copies of the bitmap to make processing easier below.
        // Also consider increasing contrast and/or brightness to compensate for lighting.
        // Likely break the bitmaps up into one slice for each expected ring and a slice
        // for the mat just in front of the rings.
        // Or perhaps use the mat to the right and left of each ring slice.
        // This might be better because removing a ring exposes the mat 'in the distance'/behind
        // the missing ring.  That may have different lighting than the ring itself.
        // Comparing the region to the left and right of each ring against the ring slice
        // might give the best indication of ring presence.
        // In any case, it's imperative that the rings are aligned properly in the frame.


        // Divide the clipping region in 10 slices.
        // Compute the average value of the pixel inside each slice of the center 80%.
        // Extend each slice to the right and left of the clipping box by 40% of box width.
        // Compute the average value of the pixel in these wings.
        // The wings should represent a reference value of gray for each slice.
        // Compare the wings and the rings and see which slices look like they have ring.
        double sliceCount = 8.0;
        int ringTop    = imageTop ;
        int ringBottom = bufHeight - imageBottom ;
        double sliceHeight = ((ringBottom - imageTop) / sliceCount) ;
        int ringRight  = bufWidth - imageRight ;
        int ringLeft   = imageLeft ;
        int ringWidth = ringRight - ringLeft ;

        // I think x,y is 0,0 in the top,left corner of the image
        // x goes left to right
        // y goes top to bottom
        int x, y, sIdx, color;
        // Slice the clipped image into 10 slices and check each slice.
        int[] sliceR = new int[] {0,0,0,0,0,0,0,0};
        int[] sliceG = new int[] {0,0,0,0,0,0,0,0};
        int[] sliceB = new int[] {0,0,0,0,0,0,0,0};
        int pixCount=0;
        int[] gndR = new int[] {0,0,0,0,0,0,0,0};
        int[] gndG = new int[] {0,0,0,0,0,0,0,0};
        int[] gndB = new int[] {0,0,0,0,0,0,0,0};
        int[] ring = new int[] {0,0,0,0,0,0,0,0};
        int gndCount=0;

        // For each slice
        for ( sIdx = 0 ; sIdx < sliceCount ; sIdx++ ) {
            pixCount=0;
            for ( y = (int) ( ringBottom - ( sliceHeight * ( sIdx+1 ) ) ); y < ( ringBottom - ( sliceHeight * sIdx ) ) ; y++ ) {
                // Wing on the left is 30% of the ring width
                for ( x = (int) (ringLeft-(ringWidth*0.50)) ; x > (ringLeft-(ringWidth*0.20))  ; x-- ) {
                    color = bitmap.getPixel(x, y);
                    gndR[sIdx] += Color.red(color);
                    gndG[sIdx] += Color.green(color);
                    gndB[sIdx] += Color.blue(color);
                    gndCount++;
                }
                // Use center 60% of the ring
                for ( x = (int) (ringLeft+(ringWidth*0.20)) ; x < (ringRight-(ringWidth*0.20)) ; x++ ) {
                    color = bitmap.getPixel(x, y);
                    sliceR[sIdx] += Color.red(color);
                    sliceG[sIdx] += Color.green(color);
                    sliceB[sIdx] += Color.blue(color);
                    pixCount++;
                }
                // Wing on the right is 30% of the ring width
                for( x = (int) (ringRight+(ringWidth*0.20)) ; x < (ringRight+(ringWidth*0.50)) ; x++ ) {
                    color = bitmap.getPixel(x, y);
                    gndR[sIdx] += Color.red(color);
                    gndG[sIdx] += Color.green(color);
                    gndB[sIdx] += Color.blue(color);
                    gndCount++;
                }
            }

            // Get the averages
            if ( gndCount != 0 ) {
                gndR[sIdx] = gndR[sIdx] / gndCount;
                gndG[sIdx] = gndG[sIdx] / gndCount;
                gndB[sIdx] = gndB[sIdx] / gndCount;
            }

            if ( pixCount != 0 ) {
                sliceR[sIdx] /= pixCount;
                sliceG[sIdx] /= pixCount;
                sliceB[sIdx] /= pixCount;
            }

            // Now check the average for each slice
            // If the ground has more than twice the blue of the ring, then it's probably a ring
            double gndBlue, sliceBlue;

            // What's the relative percentage of blue compared to the other colors?
            // FIXME
            // Should we be doing this?  Or just comparing the blue element straight-up?
            gndBlue = (double)gndB[sIdx] / (double)(gndR[sIdx]+gndG[sIdx]+gndB[sIdx]) ;
            sliceBlue = (double)sliceB[sIdx] / (double)(sliceR[sIdx]+sliceG[sIdx]+sliceB[sIdx]) ;
            if ( sliceBlue < (gndBlue/2.0) ) {
                ring[sIdx]=1;
            } else {
                ring[sIdx]=0;
            }
        }

        // FIXME: If gray count is low enough, color DS green:
        // Now create color ranges
        /*
        if ( grayCount < 2% ) {
            relativeLayout.post(new Runnable() { public void run() { relativeLayout.setBackgroundColor(Color.GREEN); } });
        } else if ( grayCount < 5% ) {
            relativeLayout.post(new Runnable() { public void run() { relativeLayout.setBackgroundColor(Color.YELLOW); } });
        } else {
            relativeLayout.post(new Runnable() { public void run() { relativeLayout.setBackgroundColor(Color.RED); } });
        }
        */

        //ringCount is how many slots we detect as ring
        //ringPos is the actual amount of rings present
        int ringSlices = 0 ;
        int ringCount = -1 ;
        // FIXME David:
        for(int i = 0; i<ring.length; i++){
            if(ring[i]==1){
                ringSlices++;
            }
        }

        if (ringSlices >= 5){
            ringCount = 4;
        } else if (ringSlices > 0){
            ringCount = 1;
        } else {
            ringCount = 0;
        }

        // Now display everything we learned.
        // FIXME: Add the actual ring count code here.  What will be used to actually count rings presence?
        //myLOpMode.telemetry.addData("PI", "W: %d, H: %d, S:%d", bufWidth, bufHeight, image.getStride());
        if (tfodState) {
            if (clip) {
                myLOpMode.telemetry.addData("PI", "L %3d, T %3s R %3d B %3d <-- Updating/Clipped", imageLeft, imageTop, imageRight, imageBottom);
            } else {
                myLOpMode.telemetry.addData("PI", "L %3d, T %3s R %3d B %3d <-- Updating/Unclipped", imageLeft, imageTop, imageRight, imageBottom);
            }
        } else {
            if (clip) {
                myLOpMode.telemetry.addData("PI", "L %3d, T %3s R %3d B %3d <-- Frozen/Clipped", imageLeft, imageTop, imageRight, imageBottom);
            } else {
                myLOpMode.telemetry.addData("PI", "L %3d, T %3s R %3d B %3d <-- Frozen/Unclipped", imageLeft, imageTop, imageRight, imageBottom);
            }
        }
        myLOpMode.telemetry.addData("PI", "r: %s", Arrays.toString(ring));
        myLOpMode.telemetry.addData("Ring Count", "%d", ringCount);

        myLOpMode.telemetry.update();

        return ringCount;
    }

    public int countRings() {

        if (tfod != null) {

            List<Recognition> Recognitions = tfod.getRecognitions();
            if (Recognitions != null) {
                for (Recognition recognition : Recognitions) {
                    if (recognition.getLabel().equals(LABEL_QUAD)) {
                        imageLeft   = (int)recognition.getLeft();
                        imageTop    = (int)recognition.getTop();
                        imageRight  = 640-(int)recognition.getRight();
                        imageBottom = 480-(int)recognition.getBottom();
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
                            tfod.setClippingMargins(imageLeft,imageTop,imageRight,imageBottom);
                        } else{
                            tfod.setClippingMargins(0, 0, 0, 0);
                        }
                        return processImage(frame.getImage(i));
                    }
                }
            }
        }

        // Unknown rings on any error.
        return -1;
    }

    public void shutdown() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}
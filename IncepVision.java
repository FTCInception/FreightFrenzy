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

        // If the rings are 'perfectly' aligned inside the image, then the ring height is ~1/4
        // of the clipped region.

        int ringTop    = imageTop ;
        int ringBottom = bufHeight - imageBottom ;
        int ringHeight = ((ringBottom - imageTop)/4) ;
        int ringRight  = bufWidth - imageRight ;
        int ringLeft   = imageLeft ;

        // I think x,y is 0,0 in the top,left corner of the image
        // x goes left to right
        // y goes top to bottom

        // Go just in front of the ring stack and measure a slice of ground there.
        // FIXME: We'd probably be better off measuring to the left and right of the ring stack
        // but the front seems to be OK for Remote field.
        int x, y, rIdx, color;
        int gndR=0, gndG=0, gndB=0;
        int gndCount=0;
        for(x=ringLeft;x<ringRight;x++) {
            for (y = (ringBottom + (int)(ringHeight*0.5)); y < (ringBottom + (int)(ringHeight*1.5)); y++) {
                color = bitmap.getPixel(x, y);
                gndR += Color.red(color);
                gndG += Color.green(color);
                gndB += Color.blue(color);
                gndCount++;
            }
        }

        if ( gndCount != 0 ) {
            gndR = gndR / gndCount;
            gndG = gndG / gndCount;
            gndB = gndB / gndCount;
        }

        // Slice the slipped image into 4 slices and assume each is a ring.
        // This is not strictly true since we're not looking head-on but close enough.
        int[] ringR = new int[] {0,0,0,0};
        int[] ringG = new int[] {0,0,0,0};
        int[] ringB = new int[] {0,0,0,0};
        int pixCount=0;
        for (rIdx=0;rIdx<4;rIdx++) {
            pixCount=0;
            for (x = ringLeft; x < ringRight; x++) {
                for (y = (ringBottom-(ringHeight*(rIdx+1))); y < (ringBottom-(ringHeight*rIdx)); y++) {
                    color = bitmap.getPixel(x, y);
                    ringR[rIdx] += Color.red(color);
                    ringG[rIdx] += Color.green(color);
                    ringB[rIdx] += Color.blue(color);

                    // FIXME: Compare this pixel against the average gray pixel.
                    // If it's similar enough, then count it as gray.
                    // During the line-up portion of the match, the camera should be placed
                    // in such a way that there is very little to no gray pixels
                    // Once the camera is adjusted (gray pixel count is low enough),
                    // then turn the screen green.

                    pixCount++;
                }
            }
            if ( pixCount != 0 ) {
                ringR[rIdx] /= pixCount;
                ringG[rIdx] /= pixCount;
                ringB[rIdx] /= pixCount;
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
        myLOpMode.telemetry.addData("PI", " gnd, r:%3d,g:%3d,b:%3d (%d)", gndR, gndG, gndB, gndCount);
        myLOpMode.telemetry.addData("PI", "r[0], r:%3d,g:%3d,b:%3d (%d)", ringR[0], ringG[0], ringB[0], pixCount);
        myLOpMode.telemetry.addData("PI", "r[1], r:%3d,g:%3d,b:%3d (%d)", ringR[1], ringG[1], ringB[1], pixCount);
        myLOpMode.telemetry.addData("PI", "r[2], r:%3d,g:%3d,b:%3d (%d)", ringR[2], ringG[2], ringB[2], pixCount);
        myLOpMode.telemetry.addData("PI", "r[3], r:%3d,g:%3d,b:%3d (%d)", ringR[3], ringG[3], ringB[3], pixCount);

        myLOpMode.telemetry.update();

        // -1 for unknown rings for now.
        return -1;
    }

    public int ringCount() {

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
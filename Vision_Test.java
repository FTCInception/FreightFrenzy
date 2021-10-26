package Inception.FreightFrenzy;

import android.app.Activity;
import android.os.Build;
import android.util.Log;
import java.util.Arrays.*;
import java.util.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
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

import androidx.annotation.RequiresApi;

/*
 * This is a modified version of the vuforia code by G-FORCE that uses Vuforia to capture images from the camera
 * Leveraged from here:
 * https://gist.github.com/brandonwang1/d5c02f64d8ab05e2dc8c18c7ba4e199a
 */

class Pixel {
    int red;
    int green;
    int blue;

    public Pixel(int r, int g, int b) {
        this.red = r;
        this.green = g;
        this.blue = b;
    }
}

@Autonomous(name="Vision_Test", group="MechBot")
public class Vision_Test extends LinearOpMode {

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
    private static boolean leftBOK = true, rightBOK = true;
    private static boolean dLeftOK = true, dRightOK = true, dUpOK = true, dDownOK = true;
    private static boolean aOK = true, bOK = true, xOK=true, yOK=true;
    public static int clipLeft = 280;
    public static int clipTop = 260;
    public static int clipRight = 260;
    public static int clipBottom = 155;
    public boolean clip = false;
    public boolean tfodState = false;
    private int ringCount = -1;
    private String webcamName = "pp";
    public final int NONE = 0, BLUE_INSIDE = 1, BLUE_OUTSIDE = 2, RED_INSIDE = 3, RED_OUTSIDE = 4;
    private final int iTOP = 0, iBOTTOM = 1, iLEFT = 2, iRIGHT = 3;
    public int auto = NONE;
    final int[][] defStack = {
            // top, bottom, left, right
            {189,    217,  252,   285},  // NONE
            {202,    199,  327,   204},  // BLUE_INSIDE
            {170,    227,  279,   247},  // BLUE_OUTSIDE
            {166,    228,  221,   302},  // RED_INSIDE
            {191,    208,  270,   264}  // RED_OUTSIDE
    };

    public void runOpMode() {
        initAutonomous(myLOpMode, "pp", NONE);
    }
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

        initVuforia(webcamName);

        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfodState = true;
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
        tfodParameters.minResultConfidence = (float) (0.4);

        // Create the object
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Load the model
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_QUAD, LABEL_SINGLE);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_QUAD);

        // bound the window down to help with ring alignment
        if (clip) {
            tfod.setClippingMargins(clipLeft, clipTop, clipRight, clipBottom);
        }
    }

    public enum Pos { Left, Center, Right, Err };
    @RequiresApi(api = Build.VERSION_CODES.N)
    public Pos findMarker() {
        Pos ret = Pos.Err; // Just in case
        VuforiaLocalizer.CloseableFrame frame;
        try {
            frame = vuforia.getFrameQueue().poll(100, TimeUnit.MILLISECONDS);
        } catch (Exception e) {
            frame = null;
        }

        if (frame != null) {
            long numImgs = frame.getNumImages();
            for (int i = 0; i < numImgs; i++) {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    //Log.d("Vuforia", "Success");
                    if (clip == true)
                        tfod.setClippingMargins(clipLeft, clipTop, clipRight, clipBottom);
                    else
                        tfod.setClippingMargins(0, 0, 0, 0);
                    try {
                        ret = processMarkerImage(frame.getImage(i));
                    } catch (Exception e) {
                        myLOpMode.telemetry.addData("TENSOR FLOW BARFED -- You need to align the camera or 'freeze' and manually align", "");
                        setDefStack();
                        myLOpMode.telemetry.update();
                    }
                }
            }
        }

        return ret;
    }

    private Pixel bitmapPixelToPixel(int bitPix) {
        return new Pixel(Color.red(bitPix), Color.green(bitPix), Color.blue(bitPix));
    }

    private boolean isGreen(Pixel p) {
        return (p.blue < 2*p.green) && (p.red < 2*p.green);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public Pos processMarkerImage(Image img) {
        Bitmap bitmap_img = Bitmap.createBitmap(img.getWidth(), img.getHeight(),
                Bitmap.Config.RGB_565);
        bitmap_img.copyPixelsFromBuffer(img.getPixels());
        int[] BitPixels = new int[img.getHeight() * img.getWidth()];
        bitmap_img.getPixels(BitPixels, 0, 1, 0, 0, img.getWidth(), img.getHeight());

        Pixel[] pixels = {};
        for(int i = 0; i < BitPixels.length; i++) {
            Pixel p = bitmapPixelToPixel(BitPixels[i]);
            if(isGreen(p))
                pixels[i] = p;
            else
                pixels[i] = new Pixel(0, 0, 0);
        }

        return Pos.Center;
    }

    public void manageVisionBox( Gamepad gamepad ) {

        // Keep the tensorFlow info
        if (gamepad.left_bumper) {
            if (leftBOK) {
                if (tfodState) {
                    tfod.deactivate();
                    tfodState = false;
                    clip = true;
                } else {
                    tfod.activate();
                    tfodState = true;
                    clip = false;
                }
                leftBOK = false;
            }
        } else {
            leftBOK = true;
        }
        // Default positions if tensorFlow failed
        if (gamepad.right_bumper) {
            if (rightBOK) {
                if (tfodState) {
                    tfod.deactivate();
                    tfodState = false;
                    clip = true;
                }
                setDefStack();
                rightBOK = false;
            }
        } else {
            rightBOK = true;
        }

        if (gamepad.dpad_left) {
            if (dLeftOK) {
                clipLeft -= 2;
                clipRight += 2;
            }
            dLeftOK = false;
        } else {
            dLeftOK = true;
        }

        if (gamepad.dpad_right) {
            if (dRightOK) {
                clipLeft += 2;
                clipRight -= 2;
            }
            dRightOK = false;
        } else {
            dRightOK = true;
        }

        if (gamepad.dpad_up) {
            if (dUpOK) {
                clipTop -= 2;
                clipBottom += 2;
            }
            dUpOK = false;
        } else {
            dUpOK = true;
        }

        if (gamepad.dpad_down) {
            if (dDownOK) {
                clipTop += 2;
                clipBottom -= 2;
            }
            dDownOK = false;
        } else {
            dDownOK = true;
        }

        if (gamepad.x) {
            if (xOK) {
                clipLeft -= 10;
                clipRight += 10;
            }
            xOK = false;
        } else {
            xOK = true;
        }

        if (gamepad.b) {
            if (bOK) {
                clipLeft += 10;
                clipRight -= 10;
            }
            bOK = false;
        } else {
            bOK = true;
        }

        if (gamepad.y) {
            if (yOK) {
                clipTop -= 10;
                clipBottom += 10;
            }
            yOK = false;
        } else {
            yOK = true;
        }

        if (gamepad.a) {
            if (aOK) {
                clipTop += 10;
                clipBottom -= 10;
            }
            aOK = false;
        } else {
            aOK = true;
        }

        // Observe some limits
        clipLeft = Range.clip(clipLeft, 5, 635);
        clipRight = Range.clip(clipRight, 5, 635);
        clipTop = Range.clip(clipTop, 5, 475);
        clipBottom = Range.clip(clipBottom, 5, 475);
    }

    public void shutdown() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}
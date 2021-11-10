package org.firstinspires.ftc.teamcode.cv;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Constants;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class Webcam {

    HardwareMap hm;
    Telemetry telemetry;

    public boolean isVuforia = false, isTrackingEnabled = false, isTrackingActivated = false, isTfodEnabled = false, isTfodActivated = false;

    public OpenGLMatrix lastLocation = new OpenGLMatrix();

    private int captureCounter = 0;
    private File captureDirectory = AppUtil.ROBOT_DATA_DIR;

    // Our instance of Vuforia
    VuforiaLocalizer vuforia;
    VuforiaLocalizer.Parameters parameters;

    //targets
    VuforiaTrackables targets;
    List<VuforiaTrackable> allTrackables;

    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    public TFObjectDetector tfod;


    public Webcam(HardwareMap hm, Telemetry telemetry){
        this.hm = hm;
        this.telemetry = telemetry;
    }

    public void activateTracking(){
        if(isTrackingActivated) {
            telemetry.addData("/> ", "Tracking is already activated");
        } else {
            if(!isTrackingEnabled)
                enableTracking();

            // activate tracking
            targets.activate();
            isTrackingActivated = true;
        }

    }
    public void deactivateTracking(){if(isTrackingActivated)targets.deactivate();}

    public void enableVuforia(){
        if(isVuforia){
            telemetry.addData("/> ", "Vuforia already enabled");
        } else {
            // start vuforia
            int cameraMonitorViewId = hm.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hm.appContext.getPackageName());
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId); // uses camera preview

            telemetry.addData("/> CAMERA", " Camera Monitor View Is Disabled...");

            // Vuforia parameters
            parameters.vuforiaLicenseKey = "AU6DGO3/////AAABmfbaGbX2lU7yobzEFgj/TC95dmC+wGBKUjoXoXSYSiz92D3Y5XU2YY4TlNcgnQLdXr8Pz3zstBN9KHBPTMczwa4QWR0rqGKqC5L3rdvyZM/bFd2v9/YkKpd54Uyl0tX1CyEB9XSW2HKhFjcofvkud19pT1nqEuQBU+Q8zKCJXc8gSycUPELKVARHhsMPOoJMH4wlS7QmwWde4q/nolTJIjolaLvSemiql29GodpyuXfxCyjRKlCLvEZ1GbwhfdDwrPsZM1QBbOJgdnAIGZ00FNf+059bdvUv3SkcfacMRVua/Jp1BWPgkocF3y2PZrBN28s0AGIlbFBMkYSDZ8stGOWDI/a9nM1EXutODEZUGOUd";
            parameters.useExtendedTracking = false;
            parameters.cameraName = hm.get(WebcamName.class, Constants.webcam);

            // Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            // Ensures the bitmap file type is available
            vuforia.enableConvertFrameToBitmap();

            vuforia.setFrameQueueCapacity(6);
            Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565,true);

            // @see #captureFrameToFile()
            AppUtil.getInstance().ensureDirectoryExists(captureDirectory);
            isVuforia = true;
        }
    }

    public Bitmap getBitmap(){
        VuforiaLocalizer.CloseableFrame frame = null;
        try {
            frame = vuforia.getFrameQueue().take();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        return vuforia.convertFrameToBitmap(frame);
    }

    public Bitmap getImage(){
        VuforiaLocalizer.CloseableFrame frame = null;
        try{
            frame = vuforia.getFrameQueue().take();
            long numImages = frame.getNumImages();
            Image rgbImage = null;
            for (int i = 0; i < numImages; i++) {
                Image img = frame.getImage(i);
                int fmt = img.getFormat();
                if (fmt == PIXEL_FORMAT.RGB565) {
                    rgbImage = frame.getImage(i);
                    break;
                }
            }

            Bitmap bm = Bitmap.createBitmap(rgbImage.getWidth(), rgbImage.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(rgbImage.getPixels());

            return bm;
        }
        catch(InterruptedException exc){
            return null;
        }
        finally{
            if (frame != null) frame.close();
        }

    }

    public void enableTracking(){
        if (isTrackingEnabled) {
            telemetry.addData("/> ", "Tracking is already enabled");
        } else {
            if(!isVuforia)
                enableVuforia();

            // init tracking
            VuforiaTrackables targets = vuforia.loadTrackablesFromAsset("UltimateGoal");
            Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

            VuforiaTrackable redWall = targets.get(2); // image 3
            redWall.setName("RedWall");

            VuforiaTrackable blueWall = targets.get(1); // image 2
            blueWall.setName("BlueWall");

            VuforiaTrackable frontWall = targets.get(0); // image 1
            frontWall.setName("FrontWall");

            VuforiaTrackable redTower = targets.get(4); // image 5
            redTower.setName("RedTower");

            VuforiaTrackable blueTower = targets.get(3); // image 4
            blueTower.setName("BlueTower");

            // For convenience, gather together all the trackable objects in one easily-iterable collection
            List<VuforiaTrackable> allTrackables = new ArrayList<>();
            allTrackables.addAll(targets);


            //Set the position of the perimeter targets with relation to origin (center of field)
//            redWall.setLocation(OpenGLMatrix
//                    .translation(0, (float) -Constants.halfFieldLength, (float) Constants.mmTargetHeight)
//                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
//
//            blueWall.setLocation(OpenGLMatrix
//                    .translation(0, halfField, mmTargetHeight)
//                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
//            frontWall.setLocation(OpenGLMatrix
//                    .translation(-halfField, 0, mmTargetHeight)
//                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));
//
//            // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
//            blueTower.setLocation(OpenGLMatrix
//                    .translation(halfField, quadField, mmTargetHeight)
//                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
//            redTower.setLocation(OpenGLMatrix
//                    .translation(halfField, -quadField, mmTargetHeight)
//                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

            // Now we put the location of the camera on the robot
            OpenGLMatrix robotFromCamera = OpenGLMatrix
                    .translation(9, -6.5f,3.5f)
                    .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZY,
                            AngleUnit.DEGREES, 90, 90, 0));


            // Tell the target listeners where the camera is
            ((VuforiaTrackableDefaultListener)redWall.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
            ((VuforiaTrackableDefaultListener)blueWall.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
            ((VuforiaTrackableDefaultListener)frontWall.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
            ((VuforiaTrackableDefaultListener)redTower.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
            ((VuforiaTrackableDefaultListener)blueTower.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);

            this.targets = targets;
            this.allTrackables = allTrackables;
            isTrackingEnabled = true;
        }

    }

    public void activateTensorflow(){
        if (isTfodActivated) {
            telemetry.addData("/> ", "Tensorflow is already activated");
        } else {
            if(isTfodEnabled)
                enableTensorflow();

            // activate tensorflow
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);
            isTfodActivated = true;
        }

    }

    public void deactivateTensorflow(){if(isTfodActivated) tfod.shutdown();}

    public void enableTensorflow(){
        if(isTfodEnabled) {
            telemetry.addData("/> ", "Tensorflow is already enabled");
        } else {
            if(!isVuforia)
                enableVuforia();

            // enable tensorflow
            //int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(); // no monitor view
            tfodParameters.minResultConfidence = 0.8f;
            tfodParameters.isModelTensorFlow2 = true;
            tfodParameters.inputSize = 320;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
            isTfodEnabled = true;
        }
    }

    /**
     * Element is on the left - level 1
     * Element is in the center - level 2
     * Element is on the right - level 3
     *
     * The duck should face the right
     * @return 1, 2, 3, for the corresponding level and 0 if something is wrong
     */
    public int scanBarcode(){
        int level = 0;
        // getUpdatedRecognitions() will return null if no new information is available since the last time that call was made.
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            // step through the list of recognitions and display boundary info.
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                telemetry.addData("/> CONFIDENCE", recognition.getConfidence());
                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());

                // Add scanning logic here

                i++;
            }

        }
        return level;
    }
}

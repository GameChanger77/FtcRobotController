package org.firstinspires.ftc.teamcode.cv;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class CamManager {

    HardwareMap hardwareMap;
    Telemetry telemetry;

    private static final String VUFORIA_KEY =
            "AU6DGO3/////AAABmfbaGbX2lU7yobzEFgj/TC95dmC+wGBKUjoXoXSYSiz92D3Y5XU2YY4TlNcgnQLdXr8Pz3zstBN9KHBPTMczwa4QWR0rqGKqC5L3rdvyZM/bFd2v9/YkKpd54Uyl0tX1CyEB9XSW2HKhFjcofvkud19pT1nqEuQBU+Q8zKCJXc8gSycUPELKVARHhsMPOoJMH4wlS7QmwWde4q/nolTJIjolaLvSemiql29GodpyuXfxCyjRKlCLvEZ1GbwhfdDwrPsZM1QBbOJgdnAIGZ00FNf+059bdvUv3SkcfacMRVua/Jp1BWPgkocF3y2PZrBN28s0AGIlbFBMkYSDZ8stGOWDI/a9nM1EXutODEZUGOUd";

    private VuforiaLocalizer vuforia;

    public WebcamName webcam1, webcam2;
    private SwitchableCamera switchableCamera;

    private TFObjectDetector tfod;

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    public CamManager(HardwareMap hm, Telemetry telemetry) {
        this.hardwareMap = hm;
        this.telemetry = telemetry;
    }

    public void init(HardwareMap hm){
        this.hardwareMap = hm;
        initVuforia();
        initTfod();
        tfod.activate();
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

        // Indicate that we wish to be able to switch cameras.
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        parameters.cameraName = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Set the active camera to Webcam 1.
        switchableCamera = (SwitchableCamera) vuforia.getCamera();
        switchableCamera.setActiveCamera(webcam1);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.setZoom(2.5, 16.0/9.0);
    }

    public void switchCamera(WebcamName name){
        switchableCamera.setActiveCamera(name);
    }

    public List<Recognition> update(Telemetry telemetry){
        List<Recognition> recognitions = tfod.getRecognitions();
        telemetry.addData("# Object Detected", recognitions.size());

        int i = 0;
        for (Recognition recognition : recognitions) {
            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                    recognition.getLeft(), recognition.getTop());
            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                    recognition.getRight(), recognition.getBottom());
            i++;
        }
        telemetry.update();
        return recognitions;
    }

}

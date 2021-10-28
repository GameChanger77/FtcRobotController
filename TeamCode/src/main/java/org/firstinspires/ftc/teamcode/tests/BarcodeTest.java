package org.firstinspires.ftc.teamcode.tests;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.cv.Barcode;
import org.firstinspires.ftc.teamcode.cv.BarcodeScanner;
import org.firstinspires.ftc.teamcode.cv.Webcam;
import org.firstinspires.ftc.teamcode.submodules.RobotHardware;

@Autonomous(group="Test")
public class BarcodeTest extends LinearOpMode {
    int[][] coords = { {64, 64}, {128, 64}, {256, 64} };
    int width = 32, height = 32;

    GlobalTelemetry gt = new GlobalTelemetry(telemetry);
    RobotHardware robot = new RobotHardware(gt);
    Webcam cam = new Webcam(hardwareMap, telemetry);
    Barcode area = new Barcode(coords, width, height);
    BarcodeScanner scanner = new BarcodeScanner(cam);

    @RequiresApi(api = Build.VERSION_CODES.Q)
    @Override
    public void runOpMode() throws InterruptedException {
        while (getRuntime() < 30){
            int level = scanner.scan(area, new float[] {128, 128, 128}, 0.75f);
            telemetry.addData("LEVEL", level);
            telemetry.update();
        }
    }
}

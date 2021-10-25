package org.firstinspires.ftc.teamcode.cv;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Build;

import androidx.annotation.ColorInt;
import androidx.annotation.RequiresApi;

import com.vuforia.Image;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;

public class BarcodeScanner {
    Webcam cam;
    Telemetry telemetry;

    public BarcodeScanner(Webcam cam) {
        this.cam = cam;
        this.telemetry = cam.telemetry;
    }

    /**
     * Scans the barcode by searching for at least two empty slots.
     * This way should scan with or without a duck.
     * By determining which slots are empty we can conclude which slot the element is in.
     *
     * Element is on the left - level 1
     * Element is in the center - level 2
     * Element is on the right - level 3
     *
     * The duck should face the right
     *
     * @param barcode is the area on the screen to scan
     * @param rgbAcc is the lowest accuracy rating that will be accepted for each color channel
     * @param threshold is the lowest amount of red or blue pixels that will be accepted as a %
     * @returns 0, 1, 2, 3 for the scoring level
     */
    @RequiresApi(api = Build.VERSION_CODES.Q)
    public int scan(Barcode barcode, float[] rgbAcc, float threshold){
        int[] levels = {};

        for (int i = 0; i < 3; i++) {
            int reds = 0;
            int blues = 0;

            int[] area = barcode.getArea(i);
            Bitmap img = cam.getBitmap();

            int epsilon = 10;
            int count = 0;
            for (int x = area[0]; x < area[1]; ++x)
            {
                for (int y = area[2]; y < area[3]; ++y)
                {
                    Color pixelColor = img.getColor(x, y);
                    float red = pixelColor.red();
                    float green = pixelColor.green();
                    float blue = pixelColor.blue();

                    telemetry.addData("RED", red);
                    telemetry.addData("GREEN", green);
                    telemetry.addData("BLUE", blue);


                    // If the color of the pixel is red
                    if (red >= rgbAcc[0] && green <= rgbAcc[1] && blue <= rgbAcc[2]){
                        reds++;
                    } else if (red <= rgbAcc[0] && green <= rgbAcc[1] && blue >= rgbAcc[2]){
                        blues++;
                    }

                }
            }

            telemetry.addData("Reds %", reds/barcode.getTotalPixels());
            telemetry.addData("Blues %", blues/barcode.getTotalPixels());

            // could be a shorter, faster way of finding the element
//            if (reds/barcode.getTotalPixels() < threshold && blues/barcode.getTotalPixels() < threshold){
//                return i + 1;
//            }

            if (reds/barcode.getTotalPixels() >= threshold) {  // empty slot
                levels[i] = 0;
            }else if (blues/barcode.getTotalPixels() >= threshold) {  // empty slot
                levels[i] = 0;
            } else {            // Element slot
                levels[i] = 1;
            }
        }

        if (levels[0] == 1){
            return 1;  // left
        } else if (levels[1] == 1){
            return 2;  // center
        } else if (levels[2] == 1){
            return 3; // right
        }

        return 0; // All slots are empty
    }

}

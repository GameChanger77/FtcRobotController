package org.firstinspires.ftc.teamcode.cv;

public class Barcode {

    int[][] coords = { {}, {}, {} };  // [level] [x, y]
    int width, height;

    public Barcode(int[][] coords, int width, int height) {
        this.coords = coords;
        this.width = width;
        this.height = height;
    }

    /**
     * Returns the top left point and the bottom right point of the scanning level
     *
     * @param level Which scanning area to return
     * @returns X1, X2, Y1, Y2
     */
    public int[] getArea(int level){
        return new int[] {coords[level][0],            // Start X
                          coords[level][0] + width,    // End x
                          coords[level][1],            // Start Y
                          coords[level][1] + height};  // End Y
    }

    public int getTotalPixels(){
        return width * height * 3;
    }

    /**
     * @return The entire coordinate array
     */
    public int[][] getCoords() {
        return coords;
    }

    /**
     * Gets the width of the scan area of each level
     * @return
     */
    public int getWidth() {
        return width;
    }

    /**
     * Gets the height of the scan area of each level
     * @return
     */
    public int getHeight() {
        return height;
    }
}

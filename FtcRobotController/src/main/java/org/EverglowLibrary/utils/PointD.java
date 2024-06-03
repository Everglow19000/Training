package org.EverglowLibrary.utils;

/**
 * A class that represents a point in 2D space.
 */
public class PointD {
    public double x;
    public double y;

    /**
     * Given no values, sets the properties' values to 0.
     */
    public PointD() {
        this.x = 0;
        this.y = 0;
    }

    public PointD(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * @return sqrt(x ^ 2 + y ^ 2).
     */
    public double hyp() {
        return Math.hypot(x, y);
    }

    public static PointD difference(PointD p1, PointD p2){
        return new PointD(p1.x - p2.x, p1.y - p2.y);
    }
}

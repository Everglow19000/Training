package org.EverglowLibrary.utils;

import static java.lang.Math.PI;

/**
 * An object encompassing a set of two dimensional position and an azimuth.
 */
public class Pose {
    /**
     * Position on the sideways-axis.
     */
    public double x;
    /**
     * Position on the forward-axis.
     */
    public double y;
    /**
     * Azimuth angle relative to y-axis (in radians).
     */
    public double angle;

    /**
     * Given no values, sets the properties' values to 0.
     */
    public Pose() {
        this.x = 0;
        this.y = 0;
        this.angle = 0;
    }

    /**
     * @param x     position on the sideways-axis.
     * @param y     position on the forward-axis.
     * @param angle azimuth angle relative to y-axis (in radians).
     */
    public Pose(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    /**
     * @param other another Pose object to copy.
     */
    public Pose(Pose other) {
        this.x = other.x;
        this.y = other.y;
        this.angle = other.angle;
    }

    /**
     * @return sqrt(x ^ 2 + y ^ 2).
     */
    public double hyp() {
        return Math.hypot(x, y);
    }

    /**
     * Given any angle, normalizes it such that it is between -PI and PI radians,
     * increasing or decreasing by 2PI radians to make it so.
     */
    public void normalizeAngle() {
        while (angle >= PI) angle -= 2.0 * PI;
        while (angle < -PI) angle += 2.0 * PI;
    }

    /**
     * Given any angle, normalizes it such that it is between -PI and PI radians,
     * increasing or decreasing by 2PI radians to make it so.
     *
     * @param angle Given angle in radians.
     * @return The angle normalized (-PI < angle < PI).
     */
    public static double normalizeAngle(double angle) {
        while (angle >= PI) angle -= 2.0 * PI;
        while (angle < -PI) angle += 2.0 * PI;
        return angle;
    }

    /**
     * Sets the pose's values to the given pose's values.
     *
     * @param pose A given pose.
     */
    public void setValue(Pose pose) {
        this.x = pose.x;
        this.y = pose.y;
        this.angle = pose.angle;
    }

    /**
     * Sets the pose's values to the given values.
     *
     * @param x     position on the sideways-axis.
     * @param y     position on the forward-axis.
     * @param angle azimuth angle relative to y-axis (in radians).
     */
    public void setValue(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    /**
     * Add a given pose's values to the pose's values.
     *
     * @param other A given pose.
     */
    public void add(Pose other) {
        this.x += other.x;
        this.y += other.y;
        this.angle += other.angle;
    }

    /**
     * Calculates the difference between the values of two poses.
     *
     * @param pose1 A given pose.
     * @param pose2 Another given pose.
     * @return A new pose object containing the differences in values (pose1 - pose2).
     */
    public static Pose difference(Pose pose1, Pose pose2) {
        return new Pose(pose1.x - pose2.x, pose1.y - pose2.y, pose1.angle - pose2.angle);
    }
}

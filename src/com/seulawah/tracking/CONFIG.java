package com.seulawah.tracking;

import java.awt.Toolkit;
import org.opencv.core.Scalar;

public class CONFIG {

    public static String path = "D:/Java/Project/OPENCV/Ball Tracking/data";

    public static double min_blob_area = 50;
    public static double max_blob_area = 300;

    public static int frame_width = 750;
    public static int frame_height = 750;

    public static int delay = 100;

    public static Scalar Colors[] = {
            new Scalar(255, 0, 0),
            new Scalar(0, 255, 0),
            new Scalar(0, 0, 255),
            new Scalar(0, 255, 255),
            new Scalar(255, 0, 255),
            new Scalar(255, 255, 0),
            new Scalar(255, 127, 255),
            new Scalar(127, 0, 255),
            new Scalar(127, 0, 127) };

    public static double dt = 0.2;
    public static double accel_noise = 0.5;
    public static double dist_thres = 30;
    public static int max_allowed_skipped_frames = 30;
    public static int max_trace_length = 10;
}

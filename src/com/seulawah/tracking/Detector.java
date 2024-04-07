package com.seulawah.tracking;

import com.seulawah.tracking.tracker.Tracker;
import org.opencv.core.*;
import org.opencv.imgproc.Moments;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;
import java.util.Vector;


public class Detector {
    public Vector<Rect> rectArray = new Vector<>();
    public Vector<Point> detections = new Vector<>();

    public static Tracker tracker;

    public void detect(Mat im, Mat img) {
        long startTime = System.nanoTime();

        //--Start filling hole--//
        //3.)
        Mat imgray = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Mat thres = new Mat();

        Imgproc.cvtColor(im, imgray, Imgproc.COLOR_BGR2GRAY);
        Imgproc.threshold(imgray, thres, 127, 255, 0);
        Imgproc.findContours(thres, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            List<MatOfPoint> contourList = new ArrayList<>();
            contourList.add(contour);
            Imgproc.drawContours(im, contourList, -1, new Scalar(0, 255, 0), Imgproc.FILLED);
        }
        //--End of filling hole--//

        //--Start of detect by contour area--//
        for (MatOfPoint contourM : contours) {
            if (Imgproc.contourArea(contourM) > CONFIG.min_blob_area && Imgproc.contourArea(contourM) < CONFIG.max_blob_area) {
                List<MatOfPoint> contourList = new ArrayList<>();
                contourList.add(contourM);
                Imgproc.drawContours(im, contourList, -1, new Scalar(0, 255, 0), Imgproc.FILLED);

                Rect boundRect = Imgproc.boundingRect(contourM);
                this.rectArray.add(boundRect);

                /*Imgproc.rectangle(im, boundRect.tl(), boundRect.br(), new Scalar(0, 0, 255), 2);
                Imgproc.rectangle(img, boundRect.tl(), boundRect.br(), new Scalar(0, 0, 255), 2);*/

                Moments moments = Imgproc.moments(contourM);

                Point pt = new Point(moments.m10 / (moments.m00), moments.m01 / (moments.m00));
                this.detections.add(pt);

                /*Imgproc.circle(im, pt, 1, new Scalar(255, 0, 0), 3);
                Imgproc.circle(img, pt, 1, new Scalar(255, 0, 0), 3);*/
            }
        }
        //--End of detect by contour area--//

        long endTime = System.nanoTime();
        //System.out.println("Detector time execute: " + ((endTime - startTime)/1_000_000) + " ms");
    }

    public Vector<Rect> getRectArray() {
        return this.rectArray;
    }

    public Vector<Point> getDetections() {
        return this.detections;
    }

}

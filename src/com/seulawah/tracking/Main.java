package com.seulawah.tracking;

import com.seulawah.tracking.tracker.Tracker;

import org.opencv.core.*;
import org.opencv.core.Point;
import org.opencv.highgui.HighGui;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import javax.swing.*;

import java.awt.*;

import java.awt.event.MouseEvent;
import java.io.File;

import java.util.*;
import java.util.List;

public class Main {
    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    public static Tracker tracker;
    public static Detector detector;

    public static void main(String[] args) {

        Mouse mickey = new Mouse();

        //---Set JFrame---
        JFrame jframe = new JFrame("JFrame Tracking");
        jframe.pack();
        Insets insets = jframe.getInsets();
        jframe.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        JLabel imgLabel = new JLabel();
        jframe.setContentPane(imgLabel);
        imgLabel.addMouseListener(mickey);
        jframe.setSize(new Dimension(
                insets.left + insets.right + CONFIG.frame_width,
                insets.top + insets.bottom + CONFIG.frame_height)
        );
        jframe.setResizable(false);
        jframe.setVisible(true);

        tracker = new Tracker(CONFIG.dist_thres, CONFIG.max_allowed_skipped_frames, CONFIG.max_trace_length);

        File directoryPath = new File(CONFIG.path);

        //---Directory ball images---
        File[] filesList = directoryPath.listFiles((dir, name) -> name.toLowerCase().endsWith(".jpg"));
        List<String> path = new ArrayList<>();


        for (int i =0; i < filesList.length; i++) {
            String new_name = "D:/Java/Project/OPENCV/Ball Tracking/data/image"+ i +".jpg";
            path.add(new_name);
        }

        Vector<Point> objClicked = new Vector<>();


        for (String file : path) {
            Mat im = Imgcodecs.imread(file);
            Imgproc.resize(im, im, new Size(CONFIG.frame_width, CONFIG.frame_height));

            Mat img = im.clone();

            Vector<Rect> rectArray;
            Vector<Point> detections;


            detector = new Detector();
            detector.detect(im, img);

            detections = detector.getDetections();
            rectArray = detector.getRectArray();

            Vector<Point> candidate = new Vector<>();

            candidate.addAll(detections);

            if (mickey.isLeftMouseButton()) {
                //---Select moment by radius area---
                for (Point detect : detections) {
                    if (tracker.euclideanDist(mickey.getPoint(), detect) < 20) {
                        objClicked.add(detect);
                        break;
                    }

                }
            }

//            System.out.println("Number object clicked: " + objClicked.size());

            //System.out.println("detections: " + detections);

            //---Kalman Filter Tracker---
            if (detections.size() > 0) {
                tracker.updateTrack(detections, objClicked, rectArray, img);
            } else if (rectArray.size() == 0) {
                tracker.updateKalman(detections);
            }

            //---Extended Kalman Filter Tracker---
            /*if (rectArray.size() > 0) {
                tracker.updateTrack(detections);
            } else if (rectArray.size() == 0)  {
                tracker.updateExtendedKalman(detections);
            }*/

            //---Draw predicted tracks---
            for (int k = 0; k < tracker.tracks.size(); k++) {
                int traceNum = tracker.tracks.get(k).trace.size();
                if (traceNum > 1) {
                    for (int jt = 1; jt < traceNum; jt++) {
                        Imgproc.line(
                                img,
                                tracker.tracks.get(k).trace.get(jt - 1),
                                tracker.tracks.get(k).trace.get(jt),
                                CONFIG.Colors[tracker.tracks.get(k).track_id % 9],
                                2, 4, 0
                        );
                    }
                }
            }

            /*HighGui.namedWindow("img", HighGui.WINDOW_AUTOSIZE);
            HighGui.imshow("img", img);
            HighGui.waitKey(CONFIG.delay);*/

            Image image = HighGui.toBufferedImage(img);
            imgLabel.setIcon(new ImageIcon(image));
            imgLabel.repaint();

            try {
                Thread.sleep(CONFIG.delay);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
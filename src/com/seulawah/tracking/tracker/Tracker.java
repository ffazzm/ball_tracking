package com.seulawah.tracking.tracker;

import com.seulawah.tracking.Mouse;
import com.seulawah.tracking.assignment.AssignmentOptimal;
import com.seulawah.tracking.kalman.DataPoint;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.Collections;
import java.util.Vector;

import static org.opencv.imgproc.Imgproc.FONT_HERSHEY_PLAIN;

public class Tracker extends JTracker{

    int TractID = 0;
    Vector<Integer> assigned_tracks = new Vector<>();

    public Tracker(double _dist_thres, int _max_allowed_skipped_frames, int _max_trace_length) {
        tracks = new Vector<>();
        dist_thres = _dist_thres;
        max_allowed_skipped_frames = _max_allowed_skipped_frames;
        max_trace_length = _max_trace_length;
        track_removed = 0;
    }

    public double euclideanDist(Point pt2, Point pt1) {
        Point diff = new Point(pt2.x - pt1.x, pt2.y - pt1.y);
        return Math.sqrt(Math.pow(diff.x, 2) + Math.pow(diff.y, 2));
    }

    public void updateTrack(Vector<Point> detections, Vector<Point> objClicked, Vector<Rect> rectArray, Mat imag) {
        //long startTime = System.nanoTime();

        if (objClicked.size() != 0) {
            //---Create tracks if no tracks vector found---
            if (tracks.size() == 0) {
                for (Point obj : objClicked) {
                    for (Point detection : detections) {
                        if (euclideanDist(obj, detection) == 0) {
                            Track tr = new Track(detection, TractID++);
                            tracks.add(tr);
                        }
                    }
                }
            }

            //---Calculate cost using sum of square distance between predicted vs detected centroids---
            int N = tracks.size();
            int M = detections.size();
            double[][] Cost = new double[N][M];
            assigned_tracks.clear();

            for (int i = 0; i < tracks.size(); i++) {
                for (int j = 0; j < detections.size(); j++) {
                    Cost[i][j] = euclideanDist(tracks.get(i).prediction, detections.get(j));
                }
            }

            AssignmentOptimal APS = new AssignmentOptimal();
            APS.Solve(Cost, assigned_tracks);
            Vector<Integer> not_assigned_tracks = new Vector<>();

            //---If tracks are not detected for long time, remove them---
            for (int i = 0; i < assigned_tracks.size(); i++) {
                if (assigned_tracks.get(i) != -1) {
                    if (Cost[i][assigned_tracks.get(i)] > dist_thres) {
                        assigned_tracks.set(i, -1); //remove
                        not_assigned_tracks.add(i);
                    }
                } else {
                    tracks.get(i).skipped_frames++;
                    not_assigned_tracks.add(i);
                }
            }

            for (int i = 0; i < tracks.size(); i++) {
                if (tracks.get(i).skipped_frames > max_allowed_skipped_frames) {
                    tracks.remove(i);
                    assigned_tracks.remove(i);
                    track_removed++;
                    i--;
                }
            }

            //---Look for not_assigned_detections---
            Vector<Integer> not_assigned_detections = new Vector<>();
            for (int i = 0; i < detections.size(); i++) {
                if (!assigned_tracks.contains(i)) {
                    not_assigned_detections.add(i);
                }
            }

            //Start new tracks for not_assigned_tracks
            if (not_assigned_detections.size() > 0) {
                for (Point obj : objClicked) {
                    for (Integer not_assigned_detection : not_assigned_detections) {
                        if (euclideanDist(obj, detections.get(not_assigned_detection)) < 20) {
                            Track tr = new Track(detections.get(not_assigned_detection), TractID++);
                            tracks.add(tr);
                        }
                    }
                }
            }

            updateKalman(detections);
            //updateExtendedKalman(detections);

            for (int j = 0; j < assigned_tracks.size(); j++) {
                if (assigned_tracks.get(j) != -1) {
                    Point pt1 = new Point(
                            (int) (
                                    (rectArray.get(assigned_tracks.get(j)).tl().x +
                                            rectArray.get(assigned_tracks.get(j)).br().x) / 2
                            ),
                            rectArray.get(assigned_tracks.get(j)).br().y
                    );
                    Point pt2 = new Point(
                            (int) (
                                    (rectArray.get(assigned_tracks.get(j)).tl().x +
                                            rectArray.get(assigned_tracks.get(j)).br().x) / 2
                            ),
                            rectArray.get(assigned_tracks.get(j)).tl().y
                    );

                    Imgproc.putText(imag, "id"+tracks.get(j).track_id, pt2,
                            1 * FONT_HERSHEY_PLAIN, 2, new Scalar(0, 0,
                                    0), 1);
                    if (tracks.get(j).history.size() < 20) {
                        tracks.get(j).history.add(pt1);
                    } else {
                        tracks.get(j).history.remove(0);
                        tracks.get(j).history.add(pt1);
                    }
                }
            }
            objClicked.clear();
            for (Track tr : tracks) {
                for (Rect rect : rectArray) {
                    if (tr.prediction.inside(rect)) {
                        Imgproc.rectangle(imag, rect.br(), rect.tl(), new Scalar(0, 0, 255), 2);
                        Imgproc.circle(imag, tr.prediction, 1, new Scalar(255, 0, 0), 3);
                    }
                }
                objClicked.add(tr.prediction);
            }
        /*long endTime = System.nanoTime();
        System.out.println("Tracker time execute: " + ((endTime - startTime)/1_000_000) + " ms");*/
        }
    }


    public void updateKalman(Vector<Point> detections) {
        if (detections.size() == 0) {
            for(int i = 0; i < assigned_tracks.size(); i++) {
                assigned_tracks.set(i, -1);
            }
        }

        for (int i = 0; i < assigned_tracks.size(); i++) {

            if(!tracks.get(i).kf.getInitialized()) {
                tracks.get(i).kf.start(tracks.get(i).track_data);
                tracks.get(i).prediction = tracks.get(i).kf.getPrediction();

            } else {
                tracks.get(i).kf.predictStep();
                tracks.get(i).prediction = tracks.get(i).kf.getPrediction();

                if (assigned_tracks.get(i) != -1) {
                    tracks.get(i).skipped_frames = 0;
                    RealVector vec = new ArrayRealVector(new double[]
                            {detections.get(assigned_tracks.get(i)).x, detections.get(assigned_tracks.get(i)).y}
                    );
                    tracks.get(i).kf.updateStep(new DataPoint(vec));
                    tracks.get(i).prediction = tracks.get(i).kf.getPrediction();

                } else {
                    RealVector vec = new ArrayRealVector(new double[]
                            {tracks.get(i).kf.getPrediction().x, tracks.get(i).kf.getPrediction().y}
                    );
                    tracks.get(i).kf.updateStep(new DataPoint(vec));
                    tracks.get(i).prediction = tracks.get(i).kf.getPrediction();
                }
            }


            if (tracks.get(i).trace.size() > max_trace_length) {
                for (int j = 0; j < tracks.get(i).trace.size() - max_trace_length; j++) {
                    tracks.get(i).trace.remove(j);
                }
            }

            tracks.get(i).trace.add(tracks.get(i).prediction);
            //tracks.get(i).kf.setLastResult(tracks.get(i).prediction);

        }

    }

    public void updateExtendedKalman(Vector<Point> detections) {
        if (detections.size() == 0) {
            for(int i = 0; i < assigned_tracks.size(); i++) {
                assigned_tracks.set(i, -1);
            }
        }

        for (int i = 0; i < assigned_tracks.size(); i++) {
            if(!tracks.get(i).ekf.getInitialized()) {
                tracks.get(i).ekf.start(tracks.get(i).track_data);
                tracks.get(i).prediction = tracks.get(i).ekf.getPrediction();

            } else {
                tracks.get(i).ekf.predictStep();
                tracks.get(i).prediction = tracks.get(i).ekf.getPrediction();

                if (assigned_tracks.get(i) != -1) {
                    tracks.get(i).skipped_frames = 0;
                    RealVector vec = new ArrayRealVector(new double[]
                            {detections.get(assigned_tracks.get(i)).x, detections.get(assigned_tracks.get(i)).y}
                    );
                    tracks.get(i).ekf.updateStep(new DataPoint(vec));
                    tracks.get(i).prediction = tracks.get(i).ekf.getPrediction();

                    /*System.out.println("predict: " + tracks.get(i).prediction + " (" + assigned_tracks.get(i) + ")");
                    System.out.println("groundt: " + vec);
                    System.out.println("-----------------------------------------");*/

                } else {
                    RealVector vec = new ArrayRealVector(new double[]
                            {tracks.get(i).ekf.getPrediction().x, tracks.get(i).ekf.getPrediction().y}
                    );
                    tracks.get(i).ekf.updateStep(new DataPoint(vec));
                    tracks.get(i).prediction = tracks.get(i).ekf.getPrediction();

                    /*System.out.println("predict: " + tracks.get(i).prediction);
                    System.out.println("groundt: Lost object");
                    System.out.println("-----------------------------------------");*/
                }
            }


            if (tracks.get(i).trace.size() > max_trace_length) {
                for (int j = 0; j < tracks.get(i).trace.size() - max_trace_length; j++) {
                    tracks.get(i).trace.remove(j);
                }
            }

            tracks.get(i).trace.add(tracks.get(i).prediction);

            //tracks.get(i).ekf.setLastResult(tracks.get(i).prediction);
        }
    }
}

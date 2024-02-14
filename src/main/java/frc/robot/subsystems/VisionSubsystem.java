// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//@PreethamY
package frc.robot.subsystems;

//import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
//import java.util.Random;
//import javax.swing.JFrame;
//import javax.swing.JLabel;

import org.opencv.core.Core;
//import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
//import org.opencv.core.MatOfPoint2f;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
//import org.opencv.objdetect.FaceDetectorYN;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  static Thread m_visionThread;

  public void imageRunner() {
    m_visionThread = new Thread(
        () -> {

          Scalar upperY;
          Scalar lowerY;
          upperY = new Scalar(252, 127, 3);
          lowerY = new Scalar(252, 177, 3);

          // Get the UsbCamera from CameraServer
          UsbCamera camera = CameraServer.startAutomaticCapture();

          // Set the resolution
          camera.setResolution(640, 480);

          // Get a CvSink. This will capture Mats from the camera
          CvSink cvSink = CameraServer.getVideo();
          // Setup a CvSource. This will send images back to the Dashboard
          CvSource outputStream = CameraServer.putVideo("Yellow Object Detection", 640, 480);

          // Mats are very memory expensive. Lets reuse this Mat.
          Mat mat = new Mat();
          Mat keegan = new Mat();
          List<MatOfPoint> points = new ArrayList<>();
          // This cannot be 'true'. The program will never exit if it is. This
          // lets the robot stop this thread when restarting robot code or
          // deploying.
          while (!Thread.interrupted()) {
            // Tell the CvSink to grab a frame from the camera and put it
            // in the source mat. If there is an error notify the output.
            if (cvSink.grabFrame(mat) == 0) {
              // Send the output the error.
              outputStream.notifyError(cvSink.getError());
              // skip the rest of the current iteration
              continue;
            }
            Core.inRange(mat, lowerY, upperY, keegan);
            Imgproc.findContours(mat, points, keegan, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(mat, points, -1, new Scalar(0, 255, 0), 4);
            int maxIndex = -1;
            double maxSize = 0;
            for (int i = 0; i < points.size(); i++) {
              if (Imgproc.contourArea(points.get(i)) > maxSize) {
                maxSize = Imgproc.contourArea(points.get(i));
                maxIndex = i;
              }
            }
            Imgproc.drawContours(mat, points, maxIndex, new Scalar(0, 255, 0), 4);
            
            outputStream.putFrame(mat);
            points.clear();
          }
        });
    m_visionThread.setDaemon(true);
    m_visionThread.start();
  }
}

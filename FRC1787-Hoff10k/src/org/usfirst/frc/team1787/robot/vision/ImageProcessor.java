package org.usfirst.frc.team1787.robot.vision;

import java.util.ArrayList;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ImageProcessor {
  
  // A "Mat" is the dataformat that OpenCv stores images in.
  // Here 2 different Mats are used:
  // originalFrame stores the raw image from the camera
  // (though sometimes it's drawn on to overlay information)
  // processedFrame stores the filtered/processed image
  private CameraController camController = CameraController.getInstance();
  private Mat originalFrame = new Mat();
  private Mat processedFrame = new Mat();

  // HSV Bounds...................................new Scalar(H, S, V);
  public final Scalar DEFAULT_HSV_LOWER_BOUNDS = new Scalar(75, 200, 30);
  public final Scalar DEFAULT_HSV_UPPER_BOUNDS = new Scalar(90, 255, 150);
  
  // Shape Filtering Parameters
  private final double defaultMinArea = 50;
  private final double defaultMinScore = 0.8;
  private final double defaultMaxScore = 2.0;
  
  // Colors used to draw contours..........new Scalar(B, G, R);
  public static final Scalar COLOR_BLACK = new Scalar(0, 0, 0);
  public static final Scalar COLOR_WHITE = new Scalar(255, 255, 255);
  public static final Scalar COLOR_BLUE = new Scalar(255, 0, 0);
  public static final Scalar COLOR_GREEN = new Scalar(0, 255, 0);
  public static final Scalar COLOR_RED = new Scalar(0, 0, 255);
  public static final Scalar COLOR_YELLOW = new Scalar(0, 255, 255);
  public static final Scalar COLOR_PURPLE = new Scalar(255, 0, 255);
  public static final Scalar COLOR_CYAN = new Scalar(255, 255, 0);
  private final Scalar[] COLORS = {COLOR_RED, COLOR_YELLOW, COLOR_CYAN, 
                                   COLOR_GREEN, COLOR_PURPLE, COLOR_BLUE};
  
  // Current Target
  private Target currentTarget = new Target(null);
  
  // Singleton Instance
  private static final ImageProcessor instance = new ImageProcessor();

  private ImageProcessor() {
    // initialization intentionally left blank.
  }
  
  /** This is the main vision pipeline. Call this method periodically to run vision! Woot! */
  public void runVisionProcessing() {
    /* Perform an HSV filter on the originalFrame to get a binary image, which is stored in processedFrame */
    getHSVFilter(DEFAULT_HSV_LOWER_BOUNDS, DEFAULT_HSV_UPPER_BOUNDS);
    
    /* Search that binary image for contours, and store the detected contours in a list 
     * in OpenCv, contours are represented by the "MatOfPoint" type. */
    ArrayList<MatOfPoint> contours = findContours(processedFrame);
    
    /* Sort through the list of contours, measuring different aspects of them to determine 
     * which of them, if any, is most likely the target */
    MatOfPoint bestCandidate = getStrongestCandidate(contours);
    currentTarget = new Target(bestCandidate);
    
    // if a valid target is found, it will be drawn on the orgininalFrame in green
    if (currentTarget.getHorizontalDistance() > 0) {
      Rect box = Imgproc.boundingRect(bestCandidate);
      drawBoundingBox(box, originalFrame, COLOR_GREEN);
      
      Point centroid = this.getContourCenter(bestCandidate);
      drawPoint(centroid, originalFrame, COLOR_GREEN);
    }
    
    // Push the original frame to the smartdash
    camController.pushFrameToDash(originalFrame);
  }
  
  
  
  
  
  /* ----------------------------------------------------------- */
  // Methods For Filtering Contours!
  /* ----------------------------------------------------------- */
  // TODO: explain how the "scoring system" here works.
  /**
   * @param contour
   * @param minScore
   * @param maxScore
   * @return if the given contour has an aspect ratio that is close enough to the aspect ratio
   * of the actual target. What exactly "close enough" means is determined by minScore and maxScore.
   * Ultimately, what is returned is this: (minScore <= (contour aspect ratio / ideal aspect ratio) <= maxScore)
   */
  public boolean passesShapeTest(MatOfPoint contour, double minScore, double maxScore) {
    double aspectRatioScore = this.getEquivalentRectangleAspectRatio(contour) / Target.DESIRED_CONTOUR_ASPECT_RATIO;
    return (minScore <= aspectRatioScore && aspectRatioScore <= maxScore);
  }
  
  /**
   * @param contour
   * @param minArea
   * @return if the area of the given contour is >= the given minimum
   */
  public boolean passesAreaTest(MatOfPoint contour, double minArea) {
    return (Imgproc.contourArea(contour) >= minArea);
  }
  
  /**
   * This method runs all contour filters and finds the contour that's most likely the target.
   * It does not bother to remove contours that are not the target from the given list.
   * @param listOfTargets
   * @return
   */
  public MatOfPoint getStrongestCandidate(ArrayList<MatOfPoint> contours) {
    MatOfPoint bestCandidate = null;
    double maxArea = defaultMinArea;
    
    for (MatOfPoint c : contours) {
      double area = Imgproc.contourArea(c);
      // TODO:
      // don't forget to also pass the area to the shape test, so it doesn't have to be
      // recalculated when getting the equivalent rectangle of the contour.
      if (area > maxArea && passesShapeTest(c, defaultMinScore, defaultMaxScore)) {
        bestCandidate = c;
        maxArea = area;
      }
    }
    
    return bestCandidate;
  }
  
  
  
  
  
  /* ----------------------------------------------------------- */
  // Methods For Finding & Measuring Contours!
  /* ----------------------------------------------------------- */
  
  public ArrayList<MatOfPoint> findContours(Mat frame) {
    ArrayList<MatOfPoint> listOfContours = new ArrayList<MatOfPoint>();
    
    // TODO: Add description of what each of these parts do.
    Mat hierarchy = new Mat();
    int mode = Imgproc.RETR_EXTERNAL;
    int method = Imgproc.CHAIN_APPROX_SIMPLE;
    Imgproc.findContours(frame, listOfContours, hierarchy, mode, method);
    
    return listOfContours;
  }
  
  public Point getContourCenter(MatOfPoint contour) {
    //Rect boundingBox = Imgproc.boundingRect(contour);
    //double centerX = boundingBox.x + (boundingBox.width / 2.0);
    //double centerY = boundingBox.y + (boundingBox.height / 2.0);
    
    // TODO: explain what's going on here
    Moments moments = Imgproc.moments(contour);
    double centerX = moments.get_m10() / moments.get_m00();
    double centerY = moments.get_m01() / moments.get_m00();
    return new Point(centerX, centerY);
  }
  
  private double getContourPerimeter(MatOfPoint contour) {
    //MatOfPoint2f temp = new MatOfPoint2f(contour.toArray());
    
    // TODO: Add a bit more explanation for commented out part above.
    MatOfPoint2f temp = new MatOfPoint2f();
    contour.convertTo(temp, CvType.CV_32F);
    /* The arcLength function requires a "MatOfPoint2f", not just a regular "MatOfPoint"
     * Just from googling, the method above seems to be the best way to deal with this. */
    
    return Imgproc.arcLength(temp, true);
  }
  
  // TODO: Add explanation for what this is, why it's here, and how it's used.
  private double getEquivalentRectangleAspectRatio(MatOfPoint contour) {
    /* Perimeter = 2*W + 2*H
     * Area = W*H
     * 
     * Area / W = H
     * Perimeter = 2*W + 2*(Area / W)
     * Perimeter*W = 2*(W^2) + 2*Area
     * 2*(W^2) - Perimeter*W + 2*Area = 0
     */
    
    double a = 2;
    double b = -1 * getContourPerimeter(contour);
    double c = 2 * Imgproc.contourArea(contour);
    
    double discriminant = (b*b) - (4*a*c);
    double largerResult = 0;
    double smallerResult = 0;
    if (discriminant >= 0) {
      // Quadratic Formula
      largerResult = (-b + Math.sqrt(discriminant)) / (2*a);
      smallerResult = (-b - Math.sqrt(discriminant)) / (2*a);
    }
    
    return (smallerResult > 0) ? (largerResult / smallerResult) : 0;
  }
  
  
  
  
  
  /* ----------------------------------------------------------- */
  // HSV Filtering Functions!
  /* ----------------------------------------------------------- */
  
  /**
   * Grabs the most recent frame from the turretCam, and performs an HSV filter.
   * 
   * An HSV filter is a type of color filter that looks at Hue, Saturation, and Value
   * (value can essentially be thought of as brightness).
   * The result of the filter is a binary image (because it's composed of only black and white pixels),
   * that indicates which pixels were/weren't within the given bounds. 
   * Pixels within the given bounds will be white, and pixel outside of the given bounds will be black.
   * 
   * Valid range for Hue is [0, 180]
   * Valid range for Saturation is [0, 255]
   * Valid range for Value is [0, 255]
   * 
   * @param lowerBounds The minimum values of H, S, and V that pass through the filter
   * @param upperBounds The maximum values of H, S, and V that pass through the filter
   * 
   * @return A Mat containing the result of the filter. Specifically, this is the "processedFrame"
   * Mat that is a member of the ImageProcessor.
   */
  public Mat getHSVFilter(Scalar lowerBounds, Scalar upperBounds) {
    camController.getFrame(originalFrame);
    
    // Images directly from the turretCam are in BGR format, so they 
    // need to be converted to HSV format before the HSV filter is applied
    Imgproc.cvtColor(originalFrame, processedFrame, Imgproc.COLOR_BGR2HSV);
    
    // This is the HSV filter
    Core.inRange(processedFrame, lowerBounds, upperBounds, processedFrame);
    return processedFrame;
  }
  
  
  
  
  
  /* ----------------------------------------------------------- */
  // Drawing Functions!
  /* ----------------------------------------------------------- */
  
  /**
   * Draws the given list of contours on the originalFrame
   * @param overlay
   * if true: draw contours directly on top of the originalFrame.
   * if false: draw contours on a black image.
   * @param listOfContours
   * @return The image with contours drawn on it.
   */
  public Mat drawContours(boolean overlay, ArrayList<MatOfPoint> listOfContours) {
    if (!overlay) {
      Core.bitwise_xor(originalFrame, originalFrame, originalFrame);
      /* Comparing the image to itself using a bitwise exclusive or operator
       * results in a completely black image. Originally, we used
       * "frame.setTo(Constants.COLOR_BLACK);", but that was deemed too
       * inefficient, as it caused the RIO to run out of memory.
       */
    }
    
    for (int i = listOfContours.size()-1; i >= 0; i--) {
      Imgproc.drawContours(originalFrame, listOfContours, i, COLORS[i % COLORS.length]);
    }
    
    return originalFrame;
  }
  
  public void drawBoundingBox(Rect box, Mat frame, Scalar color) {
    Point topLeft = new Point(box.x, box.y);
    Point bottomRight = new Point(box.x + box.width, box.y + box.height);
    int thickness = 1;
    Imgproc.rectangle(frame, topLeft, bottomRight, color, thickness);
  }
  
  public void drawPoint(Point point, Mat frame, Scalar color) {
    int markerSize = 3;
    int thickness = 1;
    Imgproc.drawMarker(frame, point, color, Imgproc.MARKER_CROSS, markerSize, thickness, Imgproc.LINE_8);
  }
  
  
  
  
  
  /* ----------------------------------------------------------- */
  // Getters!
  /* ----------------------------------------------------------- */
  
  public void publishDataToSmartDash() {
    if (currentTarget.getHorizontalDistance() > 0) {
      SmartDashboard.putBoolean("Target Aquired", true);
    } else {
      SmartDashboard.putBoolean("Target Aquired", false);
    }
    SmartDashboard.putNumber("Distance", currentTarget.getHorizontalDistance());
    SmartDashboard.putNumber("targetErrorX", currentTarget.getErrorInDegreesX());
    SmartDashboard.putNumber("targetErrorY", currentTarget.getErrorInDegreesY());
  }
  
  public Target getCurrentTarget() {
    return currentTarget;
  }
  
  public static ImageProcessor getInstance() {
    return instance;
  }
}

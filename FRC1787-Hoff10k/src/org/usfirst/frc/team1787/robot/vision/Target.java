package org.usfirst.frc.team1787.robot.vision;

import java.util.ArrayList;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

public class Target {
  // Known Geometry
  // (X inches) * (0.0254 meters / inch)
  public static final double CENTER_OF_TARGET_HEIGHT_FROM_FLOOR_METERS = ((7 * 12) + 2) * 0.0254;
  public static final double CAM_TO_TARGET_VERTICAL_DISTANCE_METERS = CENTER_OF_TARGET_HEIGHT_FROM_FLOOR_METERS
                                                                       - CameraController.getInstance().getTurretCamHeightFromFloor();
  public static final double TURRET_TO_TARGET_VERTICAL_DISTANCE_METERS = CENTER_OF_TARGET_HEIGHT_FROM_FLOOR_METERS;
  /* The upper band of the target is 4 inches tall, 
   * and the diameter of the cylinder it's wrapped around is 15 inches.
   * This means that, when viewed head on, the target should appear as a 15x4 rectangle. */
  private static final double DESIRED_CONTOUR_ASPECT_RATIO = 15/4.0;
  
  // Target Filter Preferences
  private static double minArea = 50;
  private static double defaultMinScore = 0.8;
  private static double defaultMaxScore = 2.0;
  
  // Target Info (Instance Variables)
  MatOfPoint originalContour;
  // Contour Measurements
  private double area = -1;
  private double perimeter = -1;
  private Rect boundingBox;
  private Point center;
  private double aspectRatioScore = -1;
  // Physical Measurements
  private double errorInDegreesX = 361;
  private double errorInDegreesY = 361;
  private double distance = -1;
  
  public Target() {
    area = 0;
    perimeter = 0;
    boundingBox = new Rect();
    center = new Point();
    aspectRatioScore = 0;
    errorInDegreesX = 0;
    errorInDegreesY = 0;
    distance = 0;
  }
  
  private Target(MatOfPoint contour) {
    originalContour = contour;
  }
  
  /**
   * @return The given value (assumed to be in inches) in meters.
   */
  public static double getMeters(double inches) {
    // inches * (feet / inch) * (meters / foot)
    return inches * (1.0/12) * 0.3048;
  }
  
  /*
   * Methods below this point are used for drawing the contour
   */
  
  public void drawBoundingBox(Mat frame, Scalar color) {
    Rect box = this.getBoundingBox();
    Point topLeft = new Point(box.x, box.y);
    Point bottomRight = new Point(box.x + box.width, box.y + box.height);
    Imgproc.rectangle(frame, topLeft, bottomRight, color, 1);
  }
  
  public void drawCentroid(Mat frame, Scalar color) {
    Point centroid = this.getCenter();
    Imgproc.drawMarker(frame, centroid, color, Imgproc.MARKER_CROSS, 3, 1, Imgproc.LINE_8);
  }
  
  /*
   * Methods below this point are used for filtering contours.
   */
  
  /**
   * This method runs all contour filters and finds the contour that's most likely the target.
   * It does not bother to remove contours that are not the target from the given list.
   * @param listOfContours
   * @return
   */
  public static Target getTarget(ArrayList<MatOfPoint> listOfContours) {
    Target mostLikelyTarget = new Target();
    double maxArea = minArea;
    
    for (MatOfPoint contour : listOfContours) {
      Target potentialTarget = new Target(contour);
      if (potentialTarget.passesAreaTest(maxArea) && potentialTarget.passesShapeTest()) {
        mostLikelyTarget = potentialTarget;
        maxArea = mostLikelyTarget.getArea();
      }
    }
    mostLikelyTarget.calculatePose();
    return mostLikelyTarget;
  }
  
  /**
   * Removes contours from the given list that do not have an
   * aspect ratio that is close enough to the aspect ratio of the target.
   * @param listOfContours The list of contours to filter
   * @param minScore The minimum "score" that a contour must have to remain in the list.
   * @param maxScore The maximum "score" that a contour musn't exceed to remain in the list.
   */
  public static void filterShape(ArrayList<MatOfPoint> listOfContours) {
    for (int i = listOfContours.size()-1; i >= 0; i--) {
      Target temp = new Target(listOfContours.get(i));
      if (!temp.passesShapeTest()) {
        listOfContours.remove(i);
      }
    }
  }
  
  /**
   * Removes contours from the given list that have an area less than the given minimum.
   * @param listOfContours The list of contours to filter.
   * @param minArea The minimum area a contour must have to remain in the list.
   */
  public static void filterArea(ArrayList<MatOfPoint> listOfContours) {
    for (int i = listOfContours.size()-1; i >= 0; i--) {
      Target temp = new Target(listOfContours.get(i));
      if (!temp.passesAreaTest(minArea)) {
        listOfContours.remove(i);
      }
    }
  }
  
  public boolean passesShapeTest() {
    if (aspectRatioScore == -1) {
      aspectRatioScore = this.getEquivalentRectangleAspectRatio() / DESIRED_CONTOUR_ASPECT_RATIO;
    }
    return (defaultMinScore <= aspectRatioScore && aspectRatioScore <= defaultMaxScore);
  }
  
  
  public boolean passesAreaTest(double minContourArea) {
    return (this.getArea() >= minContourArea);
  }
  
  /**
   * Should only be used to test/tune the shape filter
   * @param lowerBound
   * @param upperBound
   */
  public static void setShapeScoreBounds(double lowerBound, double upperBound) {
    defaultMinScore = lowerBound;
    defaultMaxScore = upperBound;
  }
  
  /**
   * Should only be used to test/tune the area filter
   * @param area
   */
  public static void setMinArea(double area) {
    minArea = area;
  }
  
  /*
   * Methods below this point are used to determine a target's physical location
   */
  public void calculatePose() {
    this.getErrorInDegreesX();
    this.getDistance();
  }
  
  public double getErrorInDegreesX() {
    if (errorInDegreesX == 361) {
      double errorInPixels = this.getCenter().x - CameraController.getInstance().getImageCenterX();
      //errorInDegreesX = errorInPixels * CameraController.DEGREES_PER_PIXEL_X;
      errorInDegreesX = Math.toDegrees(Math.atan(errorInPixels / CameraController.getInstance().getFocalLengthX()));
    }
    return errorInDegreesX;
  }
  
  public double getErrorInDegreesY() {
    if (errorInDegreesY == 361) {
      double errorInPixels = CameraController.getInstance().getImageCenterY() - this.getCenter().y;
      //errorInDegreesY = errorInPixels * CameraController.DEGREES_PER_PIXEL_Y;
      errorInDegreesY = Math.toDegrees(Math.atan(errorInPixels / CameraController.getInstance().getFocalLengthY()));
    }
    return errorInDegreesY;
  }
  
  /**
   * Calculates the horizontal distance between the camera and the target
   * (i.e. the distance from the cam to the target, as measured parallel to the floor).
   * @param boundingBox
   * @return the distance in meters
   */
  public double getDistance() {
    if (distance == -1) {
      double totalAngle = this.getErrorInDegreesY() + CameraController.getInstance().getTurretCamAngleFromFloor();
      distance = CAM_TO_TARGET_VERTICAL_DISTANCE_METERS / Math.tan(Math.toRadians(totalAngle));
    }
    return distance;
  }
  
  /*
   * Methods below this point are used to measure different aspects of a contour:
   */
  
  private double getEquivalentRectangleAspectRatio() {
    double contourPerimeter = this.getPerimeter();
    double contourArea = this.getArea();
    
    /* Perimeter = 2*W + 2*H
     * Area = W*H
     * 
     * Area / W = H
     * Perimeter = 2*W + 2*(Area / W)
     * Perimeter*W = 2*(W^2) + 2*Area
     * 2*(W^2) - Perimeter*W + 2*Area = 0
     */
    double a = 2;
    double b = -contourPerimeter;
    double c = 2*contourArea;
    // Quadratic Formula
    double result1 = (-b + Math.sqrt((b*b)-(4*a*c))) / (2*a);
    double result2 = (-b - Math.sqrt((b*b)-(4*a*c))) / (2*a);
    
    return (result1 > result2) ? (result1 / result2) : (result2 / result1);
  }
  
  public double getPerimeter() {
    if (perimeter == -1) {
      //MatOfPoint2f temp = new MatOfPoint2f(contour.toArray());
      MatOfPoint2f temp = new MatOfPoint2f();
      originalContour.convertTo(temp, CvType.CV_32F);
      /* The arcLength function requires a "MatOfPoint2f", not just a regular "MatOfPoint"
       * Just from googling, the method above seems to be the best way to deal with this. */
      perimeter = Imgproc.arcLength(temp, true);
    }
    return perimeter;
  }
  
  public Point getCenter() {
    if (center == null) {
      /*Rect box = this.getBoundingBox();
      int centerX = box.x + (box.width / 2.0);
      int centerY = box.y + (box.height / 2.0);*/
      Moments moments = Imgproc.moments(originalContour);
      int centerX = (int) (moments.get_m10() / moments.get_m00());
      int centerY = (int) (moments.get_m01() / moments.get_m00());
      center = new Point(centerX, centerY);
    }
    return center;
  }
  
  public Rect getBoundingBox() {
    if (boundingBox == null) {
      boundingBox = Imgproc.boundingRect(originalContour);
    }
    return boundingBox;
  }
  
  public double getArea () {
    if (area == -1) {
      area = Imgproc.contourArea(originalContour);
    }
    return area;
  }
}

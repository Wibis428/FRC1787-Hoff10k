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
import org.usfirst.frc.team1787.robot.utils.UnitConverter;

public class Target {
  // Known Geometry
  // Note, all measurements to target are made to the center of the target.
  // All measurements are in SI units unless specified otherwise
  public static final double TARGET_HEIGHT_FROM_FLOOR = UnitConverter.inchesToMeters((7 * 12) + 2);
  public static final double CAM_TO_TARGET_VERTICAL_DISTANCE = TARGET_HEIGHT_FROM_FLOOR
                                                               - CameraController.TURRET_CAM_HEIGHT_FROM_FLOOR;
  public static final double TURRET_TO_TARGET_VERTICAL_DISTANCE = TARGET_HEIGHT_FROM_FLOOR;
  /* The upper band of the target is 4 inches tall, 
   * and the diameter of the cylinder it's wrapped around is 15 inches.
   * This means that, when viewed head on, the target should appear as a 15x4 rectangle. */
  private static final double DESIRED_CONTOUR_ASPECT_RATIO = 15/4.0;
  
  // Known Geometry of Image
  /* The true center of the image isn't at width/2 or height/2 because of 0 indexing.
   * For example, an image with a width of 120 pixels has pixels from 0 to 119 inclusive, 
   * for a total of 120. Therefore the midpoint is at pixel 59.5 = (120 - 1) / 2.0; 
   * */
  private static final double CENTER_PIXEL_X = (CameraController.IMAGE_WIDTH_PIXELS - 1) / 2.0;
  private static final double CENTER_PIXEL_Y = (CameraController.IMAGE_HEIGHT_PIXELS - 1) / 2.0;
  
  // Target Filter Preferences
  private static double minArea = 50;
  private static double defaultMinScore = 0.8;
  private static double defaultMaxScore = 2.0;
  private static boolean usePinholeCameraModel = true;
  
  // Target Info (Instance Variables)
  MatOfPoint originalContour;
  // Contour Measurements
  private double area = -1;
  private double perimeter = -1;
  private Rect boundingBox = null;
  private Point center = null;
  // Physical Measurements
  private double errorInDegreesX = 361;
  private double errorInDegreesY = 361;
  private double distance = -1;
  
  public Target() {
    area = 0;
    perimeter = 0;
    boundingBox = new Rect();
    center = new Point();
    errorInDegreesX = 0;
    errorInDegreesY = 0;
    distance = 0;
  }
  
  public Target(MatOfPoint contour) {
    originalContour = contour;
  }
  
  /* Methods below this point are used to draw contours on an image
   */
  
  public void drawContour(Mat frame, Scalar color) {
    if (originalContour != null) {
      ArrayList<MatOfPoint> temp = new ArrayList<MatOfPoint>();
      temp.add(originalContour);
      Imgproc.drawContours(frame, temp, 0, color);
    }
  }
  
  public void drawBoundingBox(Mat frame, Scalar color) {
    if (originalContour != null) {
      Rect box = this.getBoundingBox();
      Point topLeft = new Point(box.x, box.y);
      Point bottomRight = new Point(box.x + box.width, box.y + box.height);
      Imgproc.rectangle(frame, topLeft, bottomRight, color, 1);
    }
  }
  
  public void drawCentroid(Mat frame, Scalar color) {
    if (originalContour != null) {
      Point centroid = this.getCenter();
      Imgproc.drawMarker(frame, centroid, color, Imgproc.MARKER_CROSS, 3, 1, Imgproc.LINE_8);
    }
  }
  
  /* Methods above this point are used to draw contours on an image
   * 
   * Methods below this point are used for filtering contours.
   */
  
  /**
   * This method runs all contour filters and finds the contour that's most likely the target.
   * It does not bother to remove contours that are not the target from the given list.
   * @param listOfTargets
   * @return
   */
  public static Target getStrongestCandidate(ArrayList<Target> listOfTargets) {
    Target mostLikelyTarget = new Target();
    double maxArea = minArea;
    
    for (Target t : listOfTargets) {
      if (t.passesAreaTest(maxArea) && t.passesShapeTest()) {
        mostLikelyTarget = t;
        maxArea = mostLikelyTarget.getArea();
      }
    }
    return mostLikelyTarget;
  }
  
  /**
   * Removes Targets from the given list that do not have an
   * aspect ratio that is close enough to the aspect ratio of the actual target.
   * @param listOfTargets The list of Targets to filter
   * @param minScore The minimum "score" that a contour must have to remain in the list.
   * @param maxScore The maximum "score" that a contour musn't exceed to remain in the list.
   */
  public static void filterShape(ArrayList<Target> listOfTargets) {
    for (int i = listOfTargets.size()-1; i >= 0; i--) {
      if (!listOfTargets.get(i).passesShapeTest()) {
        listOfTargets.remove(i);
      }
    }
  }
  
  /**
   * Removes Targets from the given list that have an area less than the given minimum.
   * @param listOfTargets The list of Targets to filter.
   * @param minArea The minimum area a contour must have to remain in the list.
   */
  public static void filterArea(ArrayList<Target> listOfTargets) {
    for (int i = listOfTargets.size()-1; i >= 0; i--) {
      if (!listOfTargets.get(i).passesAreaTest(minArea)) {
        listOfTargets.remove(i);
      }
    }
  }
  
  public boolean passesShapeTest() {
    double aspectRatioScore = this.getEquivalentRectangleAspectRatio() / DESIRED_CONTOUR_ASPECT_RATIO;
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
  
  /* Methods above this point are used to determine if a contour is actually the target
   * 
   * Methods below this point are used to determine a target's physical location
   */
  
  /** @return How many degrees off from the center of the camera the target is (horizontal). */
  public double getErrorInDegreesX() {
    if (errorInDegreesX == 361) {
      double errorInPixels = this.getCenter().x - CENTER_PIXEL_X;
      
      if (usePinholeCameraModel) {
        errorInDegreesX = Math.toDegrees(Math.atan(errorInPixels / CameraController.FOCAL_LENGTH_PIXELS_X));
      } else {
        errorInDegreesX = errorInPixels * CameraController.DEGREES_PER_PIXEL_X;
      }
    }
    return errorInDegreesX;
  }
  
  /** @return How many degrees off from the center of the camera the target is (vertical). */
  public double getErrorInDegreesY() {
    if (errorInDegreesY == 361) {
      double errorInPixels = CENTER_PIXEL_Y - this.getCenter().y;
      
      if (usePinholeCameraModel) {
        errorInDegreesY = Math.toDegrees(Math.atan(errorInPixels / CameraController.FOCAL_LENGTH_PIXELS_Y));
      } else {
        errorInDegreesY = errorInPixels * CameraController.DEGREES_PER_PIXEL_Y;
      }
    }
    return errorInDegreesY;
  }
  
  /**
   * Calculates the horizontal distance between the camera and the target
   * (i.e. the distance from the cam to the target, as measured parallel to the floor).
   * @return the distance in meters
   */
  public double getDistance() {
    if (distance == -1) {
      double angleFromHorizontal = this.getErrorInDegreesY() + CameraController.TURRET_CAM_ANGLE_FROM_FLOOR_DEGREES;
      distance = CAM_TO_TARGET_VERTICAL_DISTANCE / Math.tan(Math.toRadians(angleFromHorizontal));
    }
    return distance;
  }
  
  /* Methods above this point are used to determine a target's location relative to the robot.
   * 
   * Methods below this point are used to measure different aspects of a contour.
   */
  
  private double getEquivalentRectangleAspectRatio() {
    /* Perimeter = 2*W + 2*H
     * Area = W*H
     * 
     * Area / W = H
     * Perimeter = 2*W + 2*(Area / W)
     * Perimeter*W = 2*(W^2) + 2*Area
     * 2*(W^2) - Perimeter*W + 2*Area = 0
     */
    double a = 2;
    double b = -1*this.getPerimeter();
    double c = 2*this.getArea();
    
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
  
  public double getArea () {
    if (area == -1) {
      area = Imgproc.contourArea(originalContour);
    }
    return area;
  }
}

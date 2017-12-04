package org.usfirst.frc.team1787.robot.vision;

import org.usfirst.frc.team1787.robot.utils.UnitConverter;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;

public class Target {
  private static CameraController camController = CameraController.getInstance();
  
  // known geometry of the target
  // (measurements are in SI units and made to the center of the target unless specified otherwise).
  public static final double TARGET_HEIGHT_FROM_FLOOR = UnitConverter.inchesToMeters((7 * 12) + 2);
  public static final double CAM_TO_TARGET_VERTICAL_DISTANCE = TARGET_HEIGHT_FROM_FLOOR
                                                               - camController.TURRET_CAM_HEIGHT_FROM_FLOOR;
  public static final double TURRET_TO_TARGET_VERTICAL_DISTANCE = TARGET_HEIGHT_FROM_FLOOR;
  // 4 inch tall target wrapped around 15 inch diameter cylinder = 15/4 aspect ratio when viewed head on.
  public static final double DESIRED_CONTOUR_ASPECT_RATIO = 15/4.0;
  
  // known geometry of the image
  // The true center of the image isn't at width/2 or height/2 because of 0 indexing.
  private static final double CENTER_PIXEL_X = (camController.IMAGE_WIDTH_PIXELS - 1) / 2.0;
  private static final double CENTER_PIXEL_Y = (camController.IMAGE_HEIGHT_PIXELS - 1) / 2.0;
  
  // used to toggle between 2 different methods of calculating error.
  // pinhole camera model is more correct, but requires that the FOV of the turret cam be known.
  private static boolean usePinholeCameraModel = true;

  // member variables
  private double errorInDegreesX = 0;
  private double errorInDegreesY = 0;
  private double distance = 0;
  
  public Target(MatOfPoint contour) {
    if (contour != null) {
      Point centroid = ImageProcessor.getInstance().getContourCenter(contour);
      calculateErrorInDegreesX(centroid.x);
      calculateErrorInDegreesY(centroid.y);
      calculateDistance(errorInDegreesY);
    }
  }
  
  /** @return How many degrees off from the center the target is from the turretCam (horizontal). */
  private void calculateErrorInDegreesX(double contourCenterX) {
    double errorInPixels = contourCenterX - CENTER_PIXEL_X;
      
    if (usePinholeCameraModel) {
      errorInDegreesX = Math.toDegrees(Math.atan(errorInPixels / camController.FOCAL_LENGTH_PIXELS_X));
    } else {
      errorInDegreesX = errorInPixels * camController.DEGREES_PER_PIXEL_X;
    }
  }
  
  /** @return How many degrees off from the center the target is from the turretCam (vertical). */
  private void calculateErrorInDegreesY(double contourCenterY) {
    double errorInPixels = CENTER_PIXEL_Y - contourCenterY;
    
    if (usePinholeCameraModel) {
      errorInDegreesY = Math.toDegrees(Math.atan(errorInPixels / camController.FOCAL_LENGTH_PIXELS_Y));
    } else {
      errorInDegreesY = errorInPixels * camController.DEGREES_PER_PIXEL_Y;
    }
  }
  
  /**
   * Calculates the horizontal distance between the turretCam and the target
   * (i.e. the distance from the cam to the target, as measured parallel to the floor).
   * @return the distance in meters
   */
  private void calculateDistance(double errorY) {
    if (distance == -1) {
      double angleFromHorizontal = errorY + camController.TURRET_CAM_ANGLE_FROM_FLOOR_DEGREES;
      distance = CAM_TO_TARGET_VERTICAL_DISTANCE / Math.tan(Math.toRadians(angleFromHorizontal));
    }
  }
  
  public double getErrorInDegreesX() {
    return errorInDegreesX;
  }
  
  public double getErrorInDegreesY() {
    return errorInDegreesY;
  }
  
  public double getHorizontalDistance() {
    return distance;
  }
}

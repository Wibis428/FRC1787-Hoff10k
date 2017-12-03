package org.usfirst.frc.team1787.robot.utils;

import org.opencv.core.Mat;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;

public class CustomUsbCamera extends UsbCamera {
  
  /* Note: there are ways to get properties of the camera that are already implemented,
   * but they're not quite straight forward for whatever reason.
   * Here is how you can get different properties of the camera:
   * (The method getVideoMode() comes from the VideoSource class, and returns
   * an object of type VideoMode(), which is it's own class)
   * 
   * imageWidth = cam.getVideoMode().width;
   * imageHeight = cam.getVideoMode().height;
   * camFps = cam.getVideoMode().fps;
   * camName = cam.getName();
   */
  
  // Default Values
  private static final int defaultImageWidth = 160;
  private static final int defaultImageHeight = 120;
  private static final double defaultTimeoutLengthSeconds = 3;
  
  // Custom Member Variables
  private int id;
  private double horizontalFovDegrees;
  private double verticalFovDegrees;
  private double focalLengthPixelsX;
  private double focalLengthPixelsY;
  private CvSink frameGrabber;
  
  /* The degrees per pixel values can be used in place of the focal length calculation if the focal length isn't known.
   * This will yield an error that is less correct, but should ultimately get you to the desired position */
  public static final double DEGREES_PER_PIXEL_X = 0.15;
  public static final double DEGREES_PER_PIXEL_Y = 0.15;
  
  public static final double TURRET_CAM_ANGLE_FROM_FLOOR_DEGREES = 36.0;
  public static final double TURRET_CAM_HEIGHT_FROM_FLOOR = UnitConverter.inchesToMeters(57);
  
  
  // Inherited Constructor
  public CustomUsbCamera(String name, int dev) {
    super(name, dev);
    id = dev;
  }
  
  public CustomUsbCamera(String name, int dev, double fovDegreesX, double fovDegreesY) {
    super(name, dev);
    id = dev;
    horizontalFovDegrees = fovDegreesX;
    verticalFovDegrees = fovDegreesY;
  }
  
  /**
   * Configures the camera to be used either for image
   * processing or for regular viewing by humans.
   * @param configForVision if you would like the cam
   * to be configured for vision. true = yes, false = no.
   */
  public void configCam(boolean configForVision) {
    setResolution(defaultImageWidth, defaultImageHeight);
    setFPS(30);
    
    if (configForVision) {
      setExposureManual(0);
      setBrightness(100);
      setWhiteBalanceManual(WhiteBalance.kFixedIndoor);
      frameGrabber = CameraServer.getInstance().getVideo(this);
    } else {
      setExposureAuto();
      setBrightness(50);
      setWhiteBalanceAuto();
    }
  }
  
  /**
   * This version of calculateFocalLength should be used
   * when using an aspect ratio that 16:9. Note that 
   * the focal length in pixels might be different for x and y.
   * @param numOfPixels
   * @param fov
   * @return
   */
  private static double calculateFocalLength(int numOfPixels, double fov) {
    // Note the dimension is just split in half instead of finding the exact center, 
    //because we're working with physical properties, and the 0th pixel is still 1 pixel.
    double numerator = numOfPixels / 2.0;
    double denominator = Math.tan(Math.toRadians(fov/2.0));
    return (denominator == 0) ? 1 : (numerator / denominator);
  }
  
  /**
   * Gets the most recent frame from the camera, 
   * and store it in the given Mat object.
   * @param destination The OpenCv Mat to 
   * store the image in.
   * @return The timestamp of the frame.
   */
  public long getFrame(Mat destination) {
    return frameGrabber.grabFrame(destination, defaultTimeoutLengthSeconds);
  }
  
  public double getFovX() {
    return horizontalFovDegrees;
  }
  
  public double getFovY() {
    return verticalFovDegrees;
  }
  
  public double getFocalLengthX() {
    return focalLengthPixelsX;
  }
  
  public double getFocalLengthY() {
    return focalLengthPixelsY;
  }
  
  public int getId() {
    return id;
  }
}

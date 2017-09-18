package org.usfirst.frc.team1787.robot.vision;

import java.util.ArrayList;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoCamera.WhiteBalance;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CameraController {

  // Cameras
  /* Relevant Equations for the Pinhole Camera Model
   * 
   * Note that these equations work only when the focal length and the 
   * length of the image are measured in the same units. In these examples,
   * as well as in the code, that unit is pixels.
   * 
   * HorizontalFOV = 2*arctan((imageWidthPixels/2) / focalLengthPixels)
   * VerticalFOV = 2*arctan((imageHeightPixels/2) / focalLengthPixels)
   * imageDiagonalPixels = sqrt(imageWidthPixels^2 + imageHeightPixels^2)
   * DiagonalFOV = 2*arctan((imageDiagonalPixels/2) / focalLengthPixels)
   * focalLengthPixels = (imageWidthPixels/2) / Tan(HorizontalFOV/2)
   * focalLengthPixels = (imageHeightPixels/2) / Tan(VerticalFOV/2)
   * focalLengthPixels = (imageDiagonalPixels/2) / Tan(DiagonalFOV/2)
   */
  
  /* The following data is taken from the 
   * Microsoft Lifecam HD 3000 technical data sheet, 
   * which can be found online from Microsoft.
   * 
   * Native Image Width = 1280 pixels   (taken from data sheet)
   * Native Image Height = 720 pixels   (taken from data sheet)
   * Diagonal FOV in degrees = 68.5     (taken from data sheet)
   * Horizontal FOV in degrees = 61.39  (calculated using HorizontalFOV equation)
   * Vertical FOV in degrees = 36.93    (calculated using VerticalFOV equation)
   * Focal Length in pixels (for native resolution only!) = 1078 (calculated using focalLengthPixels equation)
   * 
   * First of all, it can be seen from the native image width and height that
   * the camera is designed to take pictures/video using a 16:9 aspect ratio.
   * However, for reasons unknown to me at the time, it appears that OpenCV doesn't play nice with 16:9 aspect ratios.
   * None of the image processing functions seemed to work on images with that aspect ratio.
   * To overcome this obstacle, we can set the resolution of the camera (using the .setResolution() method)
   * to a resolution with a different aspect ratio. Testing reveals that a 4:3 aspect ratio works,
   * so that's the ratio we're currently using.
   * Unfortunately, this has a significant effect on how easy it is to obtain the camera's focal length, 
   * which is needed to determine the target's location relative to the robot. 
   * This issue is outlined below.
   * 
   * Note: Please read up a bit on "the pinhole camera model" before reading the rest of this comment.
   * It will make more sense if you have some context.
   * 
   * In general, the focal length is a physical/geometric property of the camera and therefore doesn't change.
   * The USB camera used on the robot is no exception.
   * However, it's important to note that the focal length can change if it's being measured in pixels. 
   * This is because the size of 1 pixel in an image doesn't always correspond to the same amount of physical space.
   * For example, consider a picture that is 320 inches wide and 240 inches tall.
   * Let the focal length of the camera used to take the picture be 2 inches.
   * If the resolution of that picture is 320 pixels wide by 240 pixels tall, 
   * then the width and height of each pixel is 1 inch.
   * This means that the focal length, as measured in pixels, would be 2 pixels long.
   * However, if we increase the resolution of the image to 640 by 480 pixels
   * while keeping its physical size the same, each pixel then has a width and height of 0.5 inches.
   * Now the focal length as measured in pixels would be 4 pixels long, 
   * even though it's physical size of 2 inches never changed.
   * 
   * Keeping this in mind, consider the fact that the aspect ratio we're using isn't the same as 
   * the native aspect ratio. This means that when we request a 4:3 image from the camera, 
   * it will have to do something different than what it normally does in order to comply with our request.
   * Here are a few examples:
   * 
   * 1) The camera may take a picture at its native resolution, 
   * then crop the image so that is has a 4:3 aspect ratio.
   * 
   * 2) The camera may only uses a subset of its pixels to 
   * take pictures using a 4:3 aspect ratio.
   * 
   * 3) The camera may squeeze or stretch the image 
   * until it has a 4:3 aspect ratio.
   * 
   * There may also be other possibilities, but the point is that the image is being manipulated even before 
   * it's scanned for targets. Specifically, the image is being manipulated in a way that may change the effective
   * FOV of the camera. This means that the FOV of the camera must be determined manually, 
   * so it can be used to calculate the focal length of the camera in pixels.
   * 
   * (Side note: As far as I know, changing the resolution of the camera
   * but keeping the aspect ratio the same will have no effect on the FOV, 
   * just the crispness of the image. So once you calculate the FOV for any 
   * resolution with a 4:3 aspect ratio, that should be the same FOV for all
   * other resolutions with a 4:3 aspect ratio. This is just based on what 
   * I remember from my time with the robot, it could very well be incorrect.)
   * 
   * If you look at the equations for focal length listed at the beginning of this comment, you will see that
   * it doen't matter which FOV you calculate, whether it be the horizontal FOV, vertical FOV, or Diagonal FOV.
   * Any one of them can be used to calculate the focal length in pixels. However, there's an important assumption 
   * that these equations make, which is that the pixels in the image are square. Though this is normally true, 
   * it doesn't necessarily remain true when you use an aspect ratio that's different
   * than the native aspect ratio of the camera.
   * 
   * Consider the possibility that the camera converts from 16:9 to 4:3 by "re-grouping" its pixels.
   * For example, say we wanted to go from the native resolution of 1280 by 720 to the 4:3 resolution of 320 by 240.
   * The camera sees that we want a width of 320 pixels, so it divides the 1280 pixels it has 
   * into 320 groups of 4 (1280 / 320 = 4). 
   * It also sees that we want a height of 240 pixels, so it divides the 720 pixels it has 
   * into 240 groups of 3 (720 / 240 = 3). 
   * Now the imaging sensor is effectively divided into an array of "super pixels", 
   * each one composed of a 4 by 3 grid of the native pixels. 
   * The camera then simply considers each of these "super pixels" as one pixel when taking a picture.
   * This results in pixels that are wider than they are tall, which makes it even more
   * difficult to measure the focal length in pixels. This is because the "length" of a pixel 
   * isn't consistent between the vertical and horizontal directions. 
   * 
   * Given all of these complications, simply defining 2 different ways of representing the focal length 
   * seems to be the most simple and robust solution to me. These 2 representations would be:
   * 
   * 1) The focal length as measured by the height of pixels, 
   * which can be defined from the vertical FOV, 
   * and will be used to calculate the distance to the target.
   * 
   * 2) The focal length as measured by the width of pixels, 
   * which can be defined from the horizontal FOV, 
   * and will be used to calculate the angle between the target and the turret.
   * 
   * This will also allow for much easier testing.
   * For example, if the turret seems to be working fine but the distance calculation appears to be off, 
   * all we have to do is recalculate the value for the focal length that's defined by the vertical FOV, 
   * and we don't have to worry about that change having any effect on the performance of the turret.
   */
  
  private final int IMAGE_WIDTH_IN_PIXELS = 160;
  private final int IMAGE_HEIGHT_IN_PIXELS = 120;
  /* The true center of the image isn't at width/2 or height/2 because of 0 indexing.
   * For example, an image with a width of 120 pixels has pixels from 0 to 119 inclusive, 
   * for a total of 120. Therefore the midpoint is 59.5 = (120 - 1) / 2.0; 
   * */
  private final double IMAGE_CENTER_X = (IMAGE_WIDTH_IN_PIXELS - 1) / 2.0;
  private final double IMAGE_CENTER_Y = (IMAGE_HEIGHT_IN_PIXELS - 1) / 2.0;
  
  private final String TURRET_CAM_NAME = "turretCam";
  private final String GEAR_CAM_NAME = "gearCam";
  private final int TURRET_CAM_ID = 1;
  private final int GEAR_CAM_ID = 0;
  /* The cameras used to be constructed here, 
   * but there seems to be an issue with passing 
   * a camera that has already been constructed to 
   * "startAutomaticCapture()"
   */
  private UsbCamera turretCam;
  private UsbCamera gearCam;
  
  // Physical Camera Properties
  
  // These FOV values are just a place holders. The actual horizontal and vertical FOV still needs to be calculated.
  private final double HORIZONTAL_FOV_DEGREES = 90;
  private final double VERTICAL_FOV_DEGREES = 90;
  private final double FOCAL_LENGTH_X = this.calculateFocalLength(IMAGE_WIDTH_IN_PIXELS, HORIZONTAL_FOV_DEGREES);
  private final double FOCAL_LENGTH_Y = this.calculateFocalLength(IMAGE_HEIGHT_IN_PIXELS, VERTICAL_FOV_DEGREES);
  /* The degrees per pixel values can be used in place of the focal length calculation if the focal length isn't known.
   * This will yield an error that is less correct, but should ultimately get you to the desired position */
  public static final double DEGREES_PER_PIXEL_X = 0.15;
  public static final double DEGREES_PER_PIXEL_Y = 0.15;
  
  private final double TURRET_CAM_ANGLE_FROM_FLOOR_DEGREES = 36.0;
  // (X inches) * (0.0254 meters / inch)
  private final double TURRET_CAM_HEIGHT_FROM_FLOOR_METERS = 57.0 * 0.0254;
  
  // Servers, Sinks and Mats (images)
  /* The following variables are used to get images for processing,
   * store processed images, and send processed images to the smart dash.
   * Read the comments in the CameraController constructor and look for 
   * other places where these variables are used for more info! 
   * */
  private CameraServer camServer = CameraServer.getInstance();
  private CvSink turretCamFrameGrabber;
  private CvSource outputStream;
  private Mat originalFrame = new Mat();
  private Mat processedFrame = new Mat();

  // HSV Bounds...................new Scalar(H, S, V);
  private Scalar hsvLowerBounds = new Scalar(75, 200, 30);
  private Scalar hsvUpperBounds = new Scalar(90, 255, 150);
  
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
  private Target currentTarget = new Target();
  
  // Singleton Instance
  private static final CameraController instance = new CameraController();

  private CameraController() {
    /* Initializes each camera, and links each camera to it's own Mjpeg Server
     * which automatically pushes regular (non-processed) images to the SmartDash */
    turretCam = camServer.startAutomaticCapture(TURRET_CAM_NAME, TURRET_CAM_ID);
    gearCam = camServer.startAutomaticCapture(GEAR_CAM_NAME, GEAR_CAM_ID);
    
    /* Configure settings like resolution, exposure, white balance, etc. */
    configCamForVision(turretCam);
    configCamForRegularViewing(gearCam);

    // Construct the CvSink which is used to grab frames from the turret cam for processing.
    turretCamFrameGrabber = camServer.getVideo(turretCam);
    // Construct the CvSource which is used to push processed frames to the dashboard for viewing.
    outputStream = camServer.putVideo("OpenCV Stream", IMAGE_WIDTH_IN_PIXELS, IMAGE_HEIGHT_IN_PIXELS);
  }
  
  /**
   * Call this method periodically to run vision! Woot!
   */
  public void runVisionProcessing() {
    /* Get a frame.
     * Previously, "frameGrabber.grabFrame(orginalFrame)" was called here,
     * but an update made it so that this method times out under certain circumstances 
     * which could cause problems. I'm unsure what those circumstances are as of right now, 
     * but I read on chief delphi that using the "no timeout" version works just like before the update.
     * */
    turretCamFrameGrabber.grabFrameNoTimeout(originalFrame);
    
    /* Perform an HSV filter on the originalFrame to get a binary image, which is stored in processedFrame */
    HSVFilter(originalFrame, hsvLowerBounds, hsvUpperBounds, processedFrame);
    
    /* Search that binary image for contours, and store the detected contours in a list */
    ArrayList<MatOfPoint> listOfContours = findExternalContours(processedFrame);
    
    /* Sort through the list of contours, measuring different aspects of them to determine 
     * which of them, if any, is most likely the target */
    currentTarget = Target.getTarget(listOfContours);
    if (currentTarget.getArea() != 0) {
      // if a valid target is found, draw it on the orgininalFrame in green
      currentTarget.drawBoundingBox(originalFrame, COLOR_GREEN);
      currentTarget.drawCentroid(originalFrame, COLOR_GREEN);
    }
    // Push the original frame to the smart dash
    outputStream.putFrame(originalFrame);
  }
  
  /**
   * Performs an HSV filter, finds contours in the resulting binary image,
   * filters those contours, then draws the remaining contours. Intended to be
   * used for turning the contour filter parameters.
   * @param overlay "true" indicates the contours should be drawn
   * directly on the given image. "false" indicates the given image
   * should be made entirely black before drawing the contours.
   */
  public void showContoursfilter(boolean overlay) {
    turretCamFrameGrabber.grabFrameNoTimeout(originalFrame);
    HSVFilter(originalFrame, hsvLowerBounds, hsvUpperBounds, processedFrame);
    ArrayList<MatOfPoint> listOfContours = findExternalContours(processedFrame);
    Target.filterArea(listOfContours);
    Target.filterShape(listOfContours);
    if (!overlay) {
      Core.bitwise_xor(originalFrame, originalFrame, originalFrame);
      /* Comparing the image to itself using a bitwise exclusive or operator
       * results in a completely black image. Originally, we used
       * "frame.setTo(Constants.COLOR_BLACK);", but that was deemed too
       * inefficient, as it caused the RIO to run out of memory.
       */
    }
    drawExternalContours(listOfContours, originalFrame);
    outputStream.putFrame(originalFrame);
  }
  
  /**
   * Just performs an HSV filter and sends the resulting binary image to the
   * dashboard. Intended to be used for tuning the HSV filter.
   */
  public void showHSVFilter() {
    turretCamFrameGrabber.grabFrameNoTimeout(originalFrame);
    HSVFilter(originalFrame, hsvLowerBounds, hsvUpperBounds, processedFrame);
    outputStream.putFrame(processedFrame);
  }
  
  /*
   * The methods above this point each represent different vision pipeliens.
   * A pipleline is just the name given to a series of image processing steps put together
   * 
   * The methods below this point can be used a building blocks to make more vision pipelines
   */
  
  /**
   * @param listOfContours The list of contours to draw.
   * @param frame The image to draw on.
   */
  private void drawExternalContours(ArrayList<MatOfPoint> listOfContours, Mat frame) {
    for (int i = listOfContours.size()-1; i >= 0; i--) {
      Imgproc.drawContours(frame, listOfContours, i, COLORS[i % COLORS.length]);
    }
  }
  
  /**
   * @param frame The binary image to be analyzed
   * @return A list of the contours found in the binary image.
   */
  private ArrayList<MatOfPoint> findExternalContours(Mat frame) {
    ArrayList<MatOfPoint> listOfContours = new ArrayList<MatOfPoint>();
    Mat hierarchy = new Mat();
    int mode = Imgproc.RETR_EXTERNAL;
    int method = Imgproc.CHAIN_APPROX_SIMPLE;
    Imgproc.findContours(frame, listOfContours, hierarchy, mode, method);
    return listOfContours;
  }
  
  /**
   * Takes a BGR image, converts it to HSV, then filters the HSV image.
   * This yields a binary image.
   * @param sourceFrame The BGR image to convert
   * @param lowerBounds The minimum values of H, S, and V that pass through the filter
   * @param upperBounds The maximum values of H, S, and V that pass through the filter
   * @param outputFrame The mat to put the resulting binary image onto.
   */
  private void HSVFilter(Mat sourceFrame, Scalar lowerBounds, Scalar upperBounds, Mat outputFrame) {
    Imgproc.cvtColor(sourceFrame, outputFrame, Imgproc.COLOR_BGR2HSV);
    Core.inRange(outputFrame, lowerBounds, upperBounds, outputFrame);
  }
  
  public void setHSVBounds(double hMin, double sMin, double vMin, double hMax, double sMax, double vMax) {
    hsvLowerBounds = new Scalar(hMin, sMin, vMin);
    hsvUpperBounds = new Scalar(hMax, sMax, vMax);
  }
  
  /*
   * Methods above this point are the basic building blocks that can be used to make vision pipelines
   * 
   * Methods below this point don't really have anything to do with open CV, and are more related to 
   * getting and setting different properties of the camera.
   */
  
  public Target getCurrentTarget() {
    return currentTarget;
  }
  
  public double getImageCenterX() {
    return IMAGE_CENTER_X;
  }
  
  public double getImageCenterY() {
    return IMAGE_CENTER_Y;
  }
  
  /**
   * @return The focal length in terms of pixel width
   */
  public double getFocalLengthX() {
    return FOCAL_LENGTH_X;
  }
  
  /**
   * @return The focal length in terms of pixel height
   */
  public double getFocalLengthY() {
    return FOCAL_LENGTH_Y;
  }
  
  /**
   * @return the angle of the turret cam from the floor
   * in degrees
   */
  public double getTurretCamAngleFromFloor() {
    return TURRET_CAM_ANGLE_FROM_FLOOR_DEGREES;
  }
  
  /**
   * @return The height of the turret cam from the floor
   * in meters
   */
  public double getTurretCamHeightFromFloor() {
    return TURRET_CAM_HEIGHT_FROM_FLOOR_METERS;
  }
  
  /**
   * This version of calculateFocalLength should be used
   * when using an aspect ratio that 16:9. Note that 
   * the focal length in pixels might be different for x and y.
   * @param numOfPixels
   * @param fov
   * @return
   */
  private double calculateFocalLength(int numOfPixels, double fov) {
    // Note the dimension is just split in half instead of finding the exact center, 
    //because we're working with physical properties, and the 0th pixel is still 1 pixel.
    double numerator = numOfPixels / 2.0;
    double denominator = Math.tan(Math.toRadians(fov/2.0));
    if (denominator == 0) {
      return 1;
    }
    return numerator / denominator;
  }
  
  
  public void configCamForVision(UsbCamera cam) {
    cam.setFPS(30);
    cam.setExposureManual(0);
    cam.setBrightness(100);
    cam.setWhiteBalanceManual(WhiteBalance.kFixedIndoor);
    cam.setResolution(IMAGE_WIDTH_IN_PIXELS, IMAGE_HEIGHT_IN_PIXELS);
  }
  
  public void configCamForRegularViewing(UsbCamera cam) {
    cam.setFPS(30);
    cam.setExposureAuto();
    cam.setBrightness(50);
    cam.setWhiteBalanceAuto();
    cam.setResolution(IMAGE_WIDTH_IN_PIXELS, IMAGE_HEIGHT_IN_PIXELS);
  }
  
  /*
   * Methods below this point don't fit into any other categories, and are fairly basic.
   * They don't really have much else to do with the rest of the code and sort of exist on their own.
   */
  
  public void toggleCamStream() {
    if (getCurrentCam().equals(TURRET_CAM_NAME)) {
      setCurrentCam(GEAR_CAM_NAME);
    } else {
      setCurrentCam(TURRET_CAM_NAME);
    }
    /* Note: The "Selected Camera Path" field 
     * in the SmartDash widget must be set to
     * "CameraPublisher/CurrentCam" for this
     * method to work correctly.
     */
    
    /* Unrelated Network Tables Note: 
     * to get the default table, 
     * just use "" as the key for getTable().
     */
  }
  
  public void setCurrentCam(String camName) {
    NetworkTable.getTable("CameraPublisher").putString("CurrentCam", camName);
  }
  
  public String getCurrentCam() {
    return NetworkTable.getTable("CameraPublisher").getString("CurrentCam", 
        "Couldn't find the value called \"CurrentCam\" in the network table called \"CameraPublisher\".");
  }
  
  public void publishDataToSmartDash() {
    if (currentTarget.getArea() == 0) {
      SmartDashboard.putBoolean("Target Aquired", false);
    } else {
      SmartDashboard.putBoolean("Target Aquired", true);
    }
    SmartDashboard.putNumber("Distance", currentTarget.getDistance());
    SmartDashboard.putNumber("targetErrorX", currentTarget.getErrorInDegreesX());
    SmartDashboard.putNumber("targetErrorY", currentTarget.getErrorInDegreesY());
  }
  
  public static CameraController getInstance() {
    return instance;
  }
}

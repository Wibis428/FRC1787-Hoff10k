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
  private final int NATIVE_IMAGE_WIDTH_IN_PIXELS = 1280;
  private final int NATIVE_IMAGE_HEIGHT_IN_PIXELS = 720;
  /* The above data is taken from the Microsoft Lifecam HD 3000 technical data sheet */
  private final int IMAGE_WIDTH_IN_PIXELS = 160;
  private final int IMAGE_HEIGHT_IN_PIXELS = 120;
  /* Note to self: For unknown reasons, image processing isn't working when using a 16:9 aspect ratio, 
   * only 4:3 works. */
  private final String TURRET_CAM_NAME = "turretCam";
  private final String GEAR_CAM_NAME = "gearCam";
  private final int TURRET_CAM_ID = 1;
  private final int GEAR_CAM_ID = 0;
  /* The cameras used to be constructed here, 
   * but there seems to be an issue with passing 
   * a camera that has already been constructed to 
   * "startAutomaticCapture()".
   */
  private UsbCamera turretCam;
  private UsbCamera gearCam;

  /* Servers, Sinks, and Mats (images) 
   * The CameraServer class contains the code for running camera servers.
   * A CvSink is used to get Mats from the camera for use with OpenCV
   * A CvSource is used to push OpenCV images (Mats) to the SmartDash
   * originalFrame is where the initial frame received from a camera is stored,
   * and can be used to overlay information on top of the original image.
   * processedFrame is where a frame that is suitable for processing will be stored.
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
    /* Links each camera to it's own Mjpeg Server, which automatically pushes
     * regular (non-processed) images to the SmartDash */
    turretCam = camServer.startAutomaticCapture(TURRET_CAM_NAME, TURRET_CAM_ID);
    gearCam = camServer.startAutomaticCapture(GEAR_CAM_NAME, GEAR_CAM_ID);
    
    /* Configure settings like exposure, white balance, etc. */
    configCamForVision(turretCam);
    configCamForRegularViewing(gearCam);

    /* Construct the CvSink & CvSource used to get "Mats" for processing in
     * openCV and to push those processed images to the SmartDash, respectively */
    turretCamFrameGrabber = camServer.getVideo(turretCam);
    outputStream = camServer.putVideo("OpenCV Stream", IMAGE_WIDTH_IN_PIXELS, IMAGE_HEIGHT_IN_PIXELS);
    
    /* Gives the Target class info necessary for it's calculations. This might be temporary */
    Target.calculateFocalLength(IMAGE_WIDTH_IN_PIXELS, IMAGE_HEIGHT_IN_PIXELS);
  }
  
  /**
   * Call this method periodically to run vision! Woot!
   */
  public void runVisionProcessing() {
    /* Get a frame.
     * Used to be frameGrabber.grabFrame(orginalFrame),
     * but an update made it so that this method times out under
     * certain circumstances (idk what they are right now), which could cause problems.
     * Changed it to the "NoTimeout" version to ensure it works. */
    turretCamFrameGrabber.grabFrameNoTimeout(originalFrame);
    
    /* Perform an HSV filter to get a binary image. */
    HSVFilter(originalFrame, hsvLowerBounds, hsvUpperBounds, processedFrame);
    
    /* Search that binary image for contours, and store the detected contours in "listOfContours" */
    ArrayList<MatOfPoint> listOfContours = findExternalContours(processedFrame);
    
    /* Sort through the list of contours, measuring different aspects of them to determine if they are actually targets.
     * Get the bounding box of the contour that is most likely a target. */
    Target temp = Target.getTarget(listOfContours);
    if (temp.getArea() != 0) {
      temp.drawBoundingBox(originalFrame, COLOR_GREEN);
      temp.drawCentroid(originalFrame, COLOR_GREEN);
    }
    outputStream.putFrame(originalFrame);
    
    currentTarget = temp;
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
   * The methods above this point each represent pipeliens.
   * 
   * The methods below this point can be used a building blocks to make more vision pipelines
   */
  
  /**
   * 
   * @param listOfContours The list of contours to draw.
   * @param frame The image to draw on.
   */
  private void drawExternalContours(ArrayList<MatOfPoint> listOfContours, Mat frame) {
    for (int i = listOfContours.size()-1; i >= 0; i--) {
      Imgproc.drawContours(frame, listOfContours, i, COLORS[i % COLORS.length]);
    }
  }
  
  /**
   * 
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
   * Methods below this point don't really have anything to do with open CV.
   */
  
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
  
  public Target getCurrentTarget() {
    return currentTarget;
  }
  
  public int getImageWidth() {
    return IMAGE_WIDTH_IN_PIXELS;
  }
  
  public int getImageHeight() {
    return IMAGE_HEIGHT_IN_PIXELS;
  }
  
  public static CameraController getInstance() {
    return instance;
  }
}

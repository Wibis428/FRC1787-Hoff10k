package org.usfirst.frc.team1787.robot.vision;

import org.opencv.core.Mat;
import org.usfirst.frc.team1787.robot.utils.UnitConverter;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoCamera.WhiteBalance;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class CameraController {
  
  // Info about the pinhole camera model, focal length, and FOV:
  
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
  
  // UsbCamera objects are mainly used to config camera settings (resolution, exposure time, etc.)
  private UsbCamera gearCam;
  private UsbCamera turretCam;
  
  // these objects handle all the networking associated with the cameras,
  // as well as getting individual frames from a cam. See 2017 frc control system for more info.
  private CameraServer camServer = CameraServer.getInstance();
  private CvSink turretCamFrameGrabber;
  private CvSource outputStream;
  // The max amount of time that the code will halt while waiting for an image from the turretCam.
  private final double defaultTimeoutLengthSeconds = 3;
  
  // image info
  public final int IMAGE_WIDTH_PIXELS = 160;
  public final int IMAGE_HEIGHT_PIXELS = 120;
  
  // physical properties of the turretCam (used to find position of target)
  public final double TURRET_CAM_ANGLE_FROM_FLOOR_DEGREES = 36.0;
  public final double TURRET_CAM_HEIGHT_FROM_FLOOR = UnitConverter.inchesToMeters(57);
  private final double HORIZONTAL_FOV_DEGREES = 90;  // The current values for FOV are just place holders.
  private final double VERTICAL_FOV_DEGREES = 90;    // The actual values still need to be calculated.
  public final double FOCAL_LENGTH_PIXELS_X = calculateFocalLength(IMAGE_WIDTH_PIXELS, HORIZONTAL_FOV_DEGREES);
  public final double FOCAL_LENGTH_PIXELS_Y = calculateFocalLength(IMAGE_HEIGHT_PIXELS, VERTICAL_FOV_DEGREES);
  
  // used for when the FOV calculation seems off, and we need to get on the field RIGHT NOW! :)
  public final double DEGREES_PER_PIXEL_X = 0.15;
  public final double DEGREES_PER_PIXEL_Y = 0.15;
  
  // Singleton Instance
  private static final CameraController instance = new CameraController();

  private CameraController() {
    /* Note: The cameras themselves used to be constructed on
     * the same line that they're declared,
     * but there seems to be an issue with passing 
     * a camera that has already been constructed to 
     * "startAutomaticCapture()"
     * I currently have no explanation for why this is.
     */
    
    /* Initializes each camera, and links each camera to it's own Mjpeg Server
     * which automatically pushes regular (non-processed) images to the SmartDash */
    turretCam = camServer.startAutomaticCapture("turretCam", 1);
    gearCam = camServer.startAutomaticCapture("gearCam", 0);
    // (gearCam id (0) and turretCam id (1) were empirically determined through testing)
    // the names given are arbitrary.
    
    /* Configure settings like resolution, exposure, white balance, etc. */
    configCam(turretCam, true); // <- "true" indicates cam will be used for image processing
    configCam(gearCam, false);
    
    // used to grab individual frames from turret cam for the ImageProcessor.
    turretCamFrameGrabber = camServer.getVideo(turretCam);
    // used to push processed frames to the dashboard for viewing.
    outputStream = camServer.putVideo("OpenCV Stream", IMAGE_WIDTH_PIXELS, IMAGE_HEIGHT_PIXELS);
  }
  
  public void toggleCamStream() {
    if (getStreamingCamName().equals(turretCam.getName())) {
      setStreamingCam(gearCam.getName());
    } else {
      setStreamingCam(turretCam.getName());
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
  
  /**
   * Sets the streaming cam (i.e. the cam that's sending images to the smartdash)
   * to be the the camera with the given name.
   * @param camName The name of the camera who's stream you want to see.
   */
  public void setStreamingCam(String camName) {
    /* per the 2017 FRC Control System Documentation:
     * "If you're interested in just switching what the driver sees, 
     * and are using SmartDashboard, the SmartDashboard CameraServer Stream Viewer 
     * has an option ("Selected Camera Path") that reads the given NT key 
     * and changes the "Camera Choice" to that value (displaying that camera). 
     * The robot code then just needs to set the NT key to the correct camera name."
     */
    NetworkTable.getTable("CameraPublisher").putString("CurrentCam", camName);
  }
  
  /**
   * @return The name of the camera currently
   * sending images to the smartdash.
   */
  public String getStreamingCamName() {
    String errorMessage = "Couldn't find the value of the field \"CurrentCam\""
                          + " in the network table \"CameraPublisher\".";
    return NetworkTable.getTable("CameraPublisher").getString("CurrentCam", errorMessage);
  }
  
  /**
   * Configures the given camera to be used either for image
   * processing or for regular viewing by humans.
   * @param cam The camera to configure.
   * @param configForVision if you would like the cam
   * to be configured for vision. true = yes, false = no.
   */
  public void configCam(UsbCamera cam, boolean configForVision) {
    cam.setResolution(IMAGE_WIDTH_PIXELS, IMAGE_HEIGHT_PIXELS);
    cam.setFPS(30);
    
    if (configForVision) {
      // these settings make it easiest to see the target
      cam.setExposureManual(0);
      cam.setBrightness(100);
      cam.setWhiteBalanceManual(WhiteBalance.kFixedIndoor);
    } else {
      cam.setExposureAuto();
      cam.setBrightness(50);
      cam.setWhiteBalanceAuto();
    }
  }
  
  /**
   * Method for getting the focal length in terms of pixel size.
   * Because the width and height of a pixel might not be the same, 
   * two different representations of the focal length are needed:
   * A) the focal length in terms of the width of 1 pixel.
   * B) the focal length in terms of the height of 1 pixel.
   * 
   * @param numOfPixels
   * For "A", give the width of the image.
   * For "B", give the height of the image.            
   * @param fov 
   * For "A", give the horizontal fov of the camera.
   * For "B", give the vertical fov of the camera.
   *      
   * @return The focal length in terms of either the width of a pixel, or the height of a pixel.
   * Which of these is returned is dependent on the input parameters as described above.
   */
  private static double calculateFocalLength(int numOfPixels, double fov) {
    // uses the pinhole camera model to calculate focal length
    double numerator = numOfPixels / 2.0;
    double denominator = Math.tan(Math.toRadians(fov/2.0));
    return (denominator == 0) ? 1 : (numerator / denominator);
  }
  
  /**
   * Gets the most recent frame from the turretCam, 
   * and stores it in the given Mat object.
   * @param destination The OpenCv Mat to 
   * store the image in.
   * @return The timestamp of the frame.
   */
  public long getFrame(Mat destination) {
    return turretCamFrameGrabber.grabFrame(destination, defaultTimeoutLengthSeconds);
  }
  
  /**
   * Pushes the given frame to the dashboard
   * on the stream called "OpenCV Stream".
   * You will have to select "OpenCV Stream" on the
   * smart dashboard widget to see this.
   * 
   * @param img the frame to put on the dashboard.
   */
  public void pushFrameToDash(Mat img) {
    outputStream.putFrame(img);
  }
  
  public static CameraController getInstance() {
    return instance;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;
import java.util.stream.Collectors;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.VisionThread;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/*
   JSON format:
   {
       "team": <team number>,
       "ntmode": <"client" or "server", "client" if unspecified>
       "cameras": [
           {
               "name": <camera name>
               "path": <path, e.g. "/dev/video0">
               "pixel format": <"MJPEG", "YUYV", etc>   // optional
               "width": <video mode width>              // optional
               "height": <video mode height>            // optional
               "fps": <video mode fps>                  // optional
               "brightness": <percentage brightness>    // optional
               "white balance": <"auto", "hold", value> // optional
               "exposure": <"auto", "hold", value>      // optional
               "properties": [                          // optional
                   {
                       "name": <property name>
                       "value": <property value>
                   }
               ],
               "stream": {                              // optional
                   "properties": [
                       {
                           "name": <stream property name>
                           "value": <stream property value>
                       }
                   ]
               }
           }
       ]
       "switched cameras": [
           {
               "name": <virtual camera name>
               "key": <network table key used for selection>
               // if NT value is a string, it's treated as a name
               // if NT value is a double, it's treated as an integer index
           }
       ]
   }
 */

public final class Main {
    private static String configFile = "/boot/frc.json";

    private final static CvSource outputStream = new CvSource("ProcessedVideo", PixelFormat.kMJPEG, 160, 120, 30);
    private final static MjpegServer mjpegServer = new MjpegServer("serve_ProcessedVideo", 1184);
    static {
        mjpegServer.setSource(outputStream);
    }

    private final static NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot pose for field positioning */
    private final static NetworkTable table = inst.getTable("Camera");
    private final static DoubleArrayPublisher colPub2 = table.getDoubleArrayTopic("NotePose").publish();

    @SuppressWarnings("MemberName")
    public static class CameraConfig {
        public String name;
        public String path;
        public JsonObject config;
        public JsonElement streamConfig;
    }

    @SuppressWarnings("MemberName")
    public static class SwitchedCameraConfig {
        public String name;
        public String key;
    };

    public static int team;
    public static boolean server;
    public static List<CameraConfig> cameraConfigs = new ArrayList<>();
    public static List<SwitchedCameraConfig> switchedCameraConfigs = new ArrayList<>();
    public static List<VideoSource> cameras = new ArrayList<>();

    private Main() {
    }

    /**
     * Report parse error.
     */
    public static void parseError(String str) {
        System.err.println("config error in '" + configFile + "': " + str);
    }

    /**
     * Read single camera configuration.
     */
    public static boolean readCameraConfig(JsonObject config) {
        CameraConfig cam = new CameraConfig();

        // name
        JsonElement nameElement = config.get("name");
        if (nameElement == null) {
            parseError("could not read camera name");
            return false;
        }
        cam.name = nameElement.getAsString();

        // path
        JsonElement pathElement = config.get("path");
        if (pathElement == null) {
            parseError("camera '" + cam.name + "': could not read path");
            return false;
        }
        cam.path = pathElement.getAsString();

        // stream properties
        cam.streamConfig = config.get("stream");

        cam.config = config;

        cameraConfigs.add(cam);
        return true;
    }

    /**
     * Read single switched camera configuration.
     */
    public static boolean readSwitchedCameraConfig(JsonObject config) {
        SwitchedCameraConfig cam = new SwitchedCameraConfig();

        // name
        JsonElement nameElement = config.get("name");
        if (nameElement == null) {
            parseError("could not read switched camera name");
            return false;
        }
        cam.name = nameElement.getAsString();

        // path
        JsonElement keyElement = config.get("key");
        if (keyElement == null) {
            parseError("switched camera '" + cam.name + "': could not read key");
            return false;
        }
        cam.key = keyElement.getAsString();

        switchedCameraConfigs.add(cam);
        return true;
    }

    /**
     * Read configuration file.
     */
    @SuppressWarnings("PMD.CyclomaticComplexity")
    public static boolean readConfig() {
        // parse file
        JsonElement top;
        try {
            top = new JsonParser().parse(Files.newBufferedReader(Paths.get(configFile)));
        } catch (IOException ex) {
            System.err.println("could not open '" + configFile + "': " + ex);
            return false;
        }

        // top level must be an object
        if (!top.isJsonObject()) {
            parseError("must be JSON object");
            return false;
        }
        JsonObject obj = top.getAsJsonObject();

        // team number
        JsonElement teamElement = obj.get("team");
        if (teamElement == null) {
            parseError("could not read team number");
            return false;
        }
        team = teamElement.getAsInt();

        // ntmode (optional)
        if (obj.has("ntmode")) {
            String str = obj.get("ntmode").getAsString();
            if ("client".equalsIgnoreCase(str)) {
                server = false;
            } else if ("server".equalsIgnoreCase(str)) {
                server = true;
            } else {
                parseError("could not understand ntmode value '" + str + "'");
            }
        }

        // cameras
        JsonElement camerasElement = obj.get("cameras");
        if (camerasElement == null) {
            parseError("could not read cameras");
            return false;
        }
        JsonArray cameras = camerasElement.getAsJsonArray();
        for (JsonElement camera : cameras) {
            if (!readCameraConfig(camera.getAsJsonObject())) {
                return false;
            }
        }

        if (obj.has("switched cameras")) {
            JsonArray switchedCameras = obj.get("switched cameras").getAsJsonArray();
            for (JsonElement camera : switchedCameras) {
                if (!readSwitchedCameraConfig(camera.getAsJsonObject())) {
                    return false;
                }
            }
        }

        return true;
    }

    /**
     * Start running the camera.
     */
    public static VideoSource startCamera(CameraConfig config) {
        System.out.println("Starting camera '" + config.name + "' on " + config.path);
        UsbCamera camera = new UsbCamera(config.name, config.path);
        MjpegServer server = CameraServer.startAutomaticCapture(camera);

        Gson gson = new GsonBuilder().create();

        camera.setConfigJson(gson.toJson(config.config));
        camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

        if (config.streamConfig != null) {
            server.setConfigJson(gson.toJson(config.streamConfig));
        }

        return camera;
    }

    /**
     * Start running the switched camera.
     */
    public static MjpegServer startSwitchedCamera(SwitchedCameraConfig config) {
        System.out.println("Starting switched camera '" + config.name + "' on " + config.key);
        MjpegServer server = CameraServer.addSwitchedCamera(config.name);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.addListener(
                inst.getTopic(config.key),
                EnumSet.of(NetworkTableEvent.Kind.kImmediate, NetworkTableEvent.Kind.kValueAll),
                event -> {
                    if (event.valueData != null) {
                        if (event.valueData.value.isInteger()) {
                            int i = (int) event.valueData.value.getInteger();
                            if (i >= 0 && i < cameras.size()) {
                                server.setSource(cameras.get(i));
                            }
                        } else if (event.valueData.value.isDouble()) {
                            int i = (int) event.valueData.value.getDouble();
                            if (i >= 0 && i < cameras.size()) {
                                server.setSource(cameras.get(i));
                            }
                        } else if (event.valueData.value.isString()) {
                            String str = event.valueData.value.getString();
                            for (int i = 0; i < cameraConfigs.size(); i++) {
                                if (str.equals(cameraConfigs.get(i).name)) {
                                    server.setSource(cameras.get(i));
                                    break;
                                }
                            }
                        }
                    }
                });

        return server;
    }

    public static double detectOrangePercentage(VideoSource camera) {
        // Create a CvSink to grab frames from the camera
        CvSink cvSink = new CvSink("cvSink");
        cvSink.setSource(camera);

        // Create a Mat to hold the image
        Mat image = new Mat();

        // Grab a frame from the camera
        if (cvSink.grabFrame(image) == 0) {
            System.out.println("Empty Image!");
            return 0.0;
        }

        // Convert image to HSV color space
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(image, hsvImage, Imgproc.COLOR_BGR2HSV);

        // Define lower and upper bounds for orange color in HSV
        Scalar lowerOrange = new Scalar(5, 100, 100); // Lower bound (orange)
        Scalar upperOrange = new Scalar(15, 255, 255); // Upper bound (orange)

        // Threshold the HSV image to get only orange pixels
        Mat orangeMask = new Mat();
        Core.inRange(hsvImage, lowerOrange, upperOrange, orangeMask);

        // Count the number of orange pixels
        int orangePixelCount = Core.countNonZero(orangeMask);

        // Calculate the percentage of orange pixels
        double totalPixels = image.rows() * image.cols();
        double orangePercentage = (orangePixelCount / totalPixels) * 100.0;

        return orangePercentage;
    }

    public static double[] locateNote(VideoSource camera, int width, int height, double fov, double ringRadius,
            double camElevation) {
        // Create a CvSink to grab frames from the camera
        CvSink cvSink = new CvSink("cvSink");
        cvSink.setSource(camera);

        // Create a Mat to hold the image
        Mat image = new Mat();

        // Grab a frame from the camera
        if (cvSink.grabFrame(image) == 0) {
            System.out.println("Empty Image!");
            return new double[] { 0.0, 0.0 };
        }

        // Convert image to HSV color space
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(image, hsvImage, Imgproc.COLOR_BGR2HSV);

        // Define the range for orange color in HSV
        Scalar lowerOrange = new Scalar(0, 100, 100);
        Scalar upperOrange = new Scalar(20, 255, 255);

        // Create a binary mask for the orange color
        Mat mask = new Mat();
        Core.inRange(hsvImage, lowerOrange, upperOrange, mask);

        // Apply morphological operations to reduce noise
        Mat morphedImage = new Mat();
        Imgproc.erode(mask, morphedImage, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));
        Imgproc.dilate(morphedImage, morphedImage, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));

        // Find contours in the binary mask
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(morphedImage, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Filter contours based on size (you can adjust the threshold)
        List<MatOfPoint> filteredContours = contours.stream()
                .filter(contour -> Imgproc.contourArea(contour) > 100)
                .collect(Collectors.toList());

        // Now iterate through the filtered contours to find leftmost and rightmost
        // points
        double leftmostX = Double.MAX_VALUE;
        double rightmostX = Double.MIN_VALUE;

        for (MatOfPoint contour : filteredContours) {
            for (Point point : contour.toList()) {
                double x = point.x;

                if (x < leftmostX) {
                    leftmostX = x;
                }

                if (x > rightmostX) {
                    rightmostX = x;
                }
            }
        }

        int leftX = (int) leftmostX;
        int rightX = (int) rightmostX;

        int centerOffset = (leftX - rightX) / 2;

        final Mat rgb = new Mat();
        Imgproc.cvtColor(morphedImage, rgb, Imgproc.COLOR_GRAY2BGR);

        Imgproc.line(rgb, new Point(leftX, 0), new Point(leftX, height), new Scalar(0, 0, 255), 1);
        Imgproc.line(rgb, new Point(rightX, 0), new Point(rightX, height), new Scalar(0, 0, 255), 1);

        Imgproc.line(rgb, new Point(0, (int) height / 2), new Point(width, (int) height / 2), new Scalar(255, 0, 0), 1);
        Imgproc.line(rgb, new Point((int) width / 2, 0), new Point((int) width / 2, height), new Scalar(255, 0, 0), 1);

        Imgproc.line(rgb, new Point(80 - centerOffset, 0), new Point(80 - centerOffset, height), new Scalar(0, 255, 0),
                1);
        Imgproc.line(rgb, new Point(80 + centerOffset, 0), new Point(80 + centerOffset, height), new Scalar(0, 255, 0),
                1);

        // Imgproc.line(rgb, new Point(0, 0), new Point(100, 100), new Scalar(0, 0,
        // 255), 6);

        // Calculate the center or use leftmostX and rightmostX for further processing
        // double centerX = (leftmostX + rightmostX) / 2.0;

        // Mat processedImage = new Mat();
        // // Draw contours on the processed image for visualization
        // Imgproc.drawContours(processedImage, filteredContours, -1, new Scalar(0, 255,
        // 0), 2);

        // // Optional: Show the processed image (you might need to adjust window name)
        // // HighGui.imshow("Processed Image", processedImage);
        // // HighGui.waitKey(0);

        // // Put the processed image on CameraServer

        // if (!processedImage.empty()) {
        // outputStream.putFrame(processedImage);
        // }

        // return (centerX <= width) ? centerX : 0;

        outputStream.putFrame(rgb);

        final double internalAngleOffset = ((((leftmostX + rightmostX) / 2) - leftmostX) * 2 * fov) / width;
        final double hypDistance = ringRadius / Math.tan(internalAngleOffset);

        if (hypDistance <= camElevation) {
            return new double[] { 0.0, 0.0 };
        }

        final double nD = Math.sqrt(Math.pow(hypDistance, 2) - Math.pow(camElevation, 2));
        final double nTheta = (fov / width) * ((leftmostX + rightmostX - width) / 2);

        if (nTheta > 2) {
            return new double[] { 0.0, 0.0 };
        }

        image.release();
        hsvImage.release();
        mask.release();
        morphedImage.release();
        hierarchy.release();
        rgb.release();

        return new double[] { nD, nTheta };

    }

    /**
     * Example pipeline.
     */
    public static class MyPipeline implements VisionPipeline {
        public int val;

        @Override
        public void process(Mat mat) {
            val += 1;
        }
    }

    /**
     * Main.
     */
    public static void main(String... args) {

        Timer timer = new Timer();

        // StringTopic stringTopic = new StringTopic(NetworkTableInstance.getDefault(),
        // team);

        // StringPublisher publisher = stringTopic.publish();

        if (args.length > 0) {
            configFile = args[0];
        }

        // read configuration
        if (!readConfig()) {
            return;
        }

        // start NetworkTables
        NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
        if (server) {
            System.out.println("Setting up NetworkTables server");
            ntinst.startServer();
        } else {
            System.out.println("Setting up NetworkTables client for team " + team);
            ntinst.startClient4("wpilibpi");
            ntinst.setServerTeam(team);
            ntinst.startDSClient();
        }

        // start cameras
        for (CameraConfig config : cameraConfigs) {
            cameras.add(startCamera(config));
        }

        // start switched cameras
        for (SwitchedCameraConfig config : switchedCameraConfigs) {
            startSwitchedCamera(config);
        }

        // start image processing on camera 0 if present
        if (cameras.size() >= 1) {
            VisionThread visionThread = new VisionThread(cameras.get(0),
                    new MyPipeline(), pipeline -> {
                        // do something with pipeline results
                    });
            /*
             * something like this for GRIP:
             * VisionThread visionThread = new VisionThread(cameras.get(0),
             * new GripPipeline(), pipeline -> {
             * ...
             * });
             */
            visionThread.start();
        }

        TimerTask postLoop = new TimerTask() {
            @Override
            public void run() {
                colPub2.set(locateNote(cameras.get(0), 160, 120, 0.9564404, 0.1778, 0.257556));
            }
        };

        // loop forever
        // for (;;) {
        // try {
        // colPub.set("Field2d");
        // // publisher.set(postCameraPixelColor(cameras.get(0), "TopLeftPixelColor"));
        // Thread.sleep(100); // Sleep for a bit to avoid flooding the NetworkTable
        // } catch (InterruptedException ex) {
        // return;
        // }
        // }

        timer.scheduleAtFixedRate(postLoop, 0, 50);

        // Keep the main method running indefinitely
        for (;;) {
            try {
                Thread.sleep(1000); // Sleep for a second to avoid exiting immediately
            } catch (InterruptedException ex) {
                ex.printStackTrace();
            }
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name ="autonomousCamera", group = "Robot")
public class AutonomousCamera extends LinearOpMode{
    OpenCvWebcam webcam;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    OpenCvPipeline currentPipeline;

    boolean arrived = false;
    private int amountBallsPickedUp = 0;

    final int GPP = 21;
    final int PGP = 22;
    final int PPG = 23;

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor LF = null;
    private DcMotor LB = null;
    private DcMotor RF = null;
    private DcMotor RB = null;
    private char[] volgordeToChar;

    double tagsize = 0.166;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    String volgorde = "";

    @Override
    public void runOpMode()
    {
        // de vier motoren
        LF = hardwareMap.get(DcMotor.class, "LF");
        LB  = hardwareMap.get(DcMotor.class, "LB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        RB = hardwareMap.get(DcMotor.class, "RB");

        // richtingen corrigeren
        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        // remmen na stoppen
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //-----------------------------------------------------------//

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        AutonomousCamera.ballDetectionPipeLine colorDetectionPipeline = new AutonomousCamera.ballDetectionPipeLine(webcam);

        currentPipeline = aprilTagDetectionPipeline;
        webcam.setPipeline(currentPipeline);
        webcam.setMillisecondsPermissionTimeout(5000);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Camera Status", "Stream gestart");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Camera Fout", "Code: " + errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while(opModeIsActive())
        {
            if(currentPipeline == aprilTagDetectionPipeline) {
                ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

                // telemetry
                telemetry.addData("Frame Count", webcam.getFrameCount());
                telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
                telemetry.addData("huidige volgorde", volgorde);

                if (detections != null) {
                    telemetry.addData("Aantal tags", detections.size());
                    if (detections.size() > 0) {
                        for (AprilTagDetection tag : detections) {
                            if (tag.id == GPP) volgorde = "GPP";
                            if (tag.id == PGP) volgorde = "PGP";
                            if (tag.id == PPG) volgorde = "PPG";

                            telemetry.addData("tag ID's: ", tag.id);

                            volgordeToChar = volgorde.toCharArray();
                            currentPipeline = colorDetectionPipeline;
                            webcam.setPipeline(currentPipeline);

                        }
                    }
                } else {
                    telemetry.addData("Aantal tags", "Geen nieuwe detecties");
                }
            }

            if(currentPipeline == colorDetectionPipeline) {
                // haal laatste afstanden en posities op
                double greenDist = colorDetectionPipeline.getLastDistanceGreen();
                double purpleDist = colorDetectionPipeline.getLastDistancePurple();
                String greenPos = colorDetectionPipeline.getGreenBallPosition();
                String purplePos = colorDetectionPipeline.getPurpleBallPosition();

                telemetry.addData("Groen: aantal", colorDetectionPipeline.getNumberOfGreenObjects());
                telemetry.addData("Groen: afstand (cm)", String.format("%.2f", greenDist));
                telemetry.addData("Groen: positie", greenPos);
                telemetry.addData("arrived", arrived);
                telemetry.addData("Paars: aantal", colorDetectionPipeline.getNumberOfPurpleObjects());
                telemetry.addData("Paars: afstand (cm)", String.format("%.2f", purpleDist));
                telemetry.addData("Paars: positie", purplePos);


                // aangepaste logica om minder te haperen
                if (!arrived) {
                    boolean purpleSeen = purpleDist > 0;
                    boolean greenSeen = greenDist > 0;

                    if (volgordeToChar != null) {
                        if (volgordeToChar[amountBallsPickedUp] == 'G') {
                            if (greenSeen) {
                                // naar groene bal
                                if (greenDist > 20) {
                                    if (greenPos.equals("BAL RECHTS")) turnRightSmoothly(0.2);
                                    else if (greenPos.equals("BAL LINKS")) turnLeftSmoothly(0.2);
                                    else if (greenPos.equals("BAL IN MIDDEN")) driveForward(0.4);
                                } else {
                                    stopDriving();
                                    arrived = true;
                                }
                            } else {
                                stopDriving();
                            }
                        } else if (volgordeToChar[amountBallsPickedUp] == 'P') {
                            // naar paarse bal
                            if (purpleSeen) {
                                // stuur naar paarse bal
                                if (purpleDist > 20) {
                                    if (purplePos.equals("BAL RECHTS")) turnRightSmoothly(0.2);
                                    else if (purplePos.equals("BAL LINKS")) turnLeftSmoothly(0.2);
                                    else if (purplePos.equals("BAL IN MIDDEN")) driveForward(0.4);
                                } else {
                                    stopDriving();
                                    arrived = true;
                                }
                            } else {
                                stopDriving();
                            }
                        }
                    }
                }
            }


            telemetry.update();

            sleep(50);

        }

        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    // helper functies
    private void driveForward(double power) {
        LF.setPower(power);
        LB.setPower(power);
        RB.setPower(power);
        RF.setPower(power);
    }

    private void turnLeft(double power) {
        LF.setPower(-power);
        LB.setPower(-power);
        RB.setPower(power);
        RF.setPower(power);
    }

    private void turnRight(double power) {
        LF.setPower(power);
        LB.setPower(power);
        RB.setPower(-power);
        RF.setPower(-power);
    }

    private void stopDriving() {
        LF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
        RF.setPower(0);
    }

    private void turnLeftSmoothly(double power) {
        LF.setPower(power * 0.8);
        LB.setPower(power * 0.8);
        RF.setPower(power);
        RB.setPower(power);
    }
    private void turnRightSmoothly(double power) {
        LF.setPower(power);
        LB.setPower(power);
        RF.setPower(power * 0.8);
        RB.setPower(power * 0.8);
    }

    // ballDetectionPipeline blijft hetzelfde als voorheen
    class ballDetectionPipeLine extends OpenCvPipeline
    {
        boolean viewPortPaused;

        private static final int CAMERA_WIDTH = 320;
        private static final int CAMERA_HEIGHT = 240;
        private static final int CENTER_TOLERANCE = 30;

        Mat hsv = new Mat();
        Mat hierarchy = new Mat();

        public Scalar lowerGreen = new Scalar(85, 180, 100);
        public Scalar upperGreen = new Scalar(95, 255, 255);

        public Scalar lowerPurple = new Scalar(133, 90, 0);
        public Scalar upperPurple = new Scalar(145, 255, 255);

        private volatile int numberOfGreenObjects = 0;
        private volatile double lastDistanceGreen = 0;
        private volatile double lastPixelDiameterGreen = 0;
        private volatile Point lastCenterGreen = new Point(-1, -1);

        private volatile int numberOfPurpleObjects = 0;
        private volatile double lastDistancePurple = 0;
        private volatile double lastPixelDiameterPurple = 0;
        private volatile Point lastCenterPurple = new Point(-1, -1);

        private OpenCvWebcam externalWebcam;

        private final double realDiameter = 12.70;   // cm
        private final double focalLength = 301.00;   // berekend uit 30cm @ 150 pixels

        public ballDetectionPipeLine(OpenCvWebcam webcam)
        {
            this.externalWebcam = webcam;
        }

        // Getters
        public int getNumberOfGreenObjects() { return numberOfGreenObjects; }
        public double getLastDistanceGreen() { return lastDistanceGreen; }
        public String getGreenBallPosition() {
            if (numberOfGreenObjects == 0) return "Geen bal gedetecteerd";
            if (Math.abs(lastCenterGreen.x - CAMERA_WIDTH/2.0) <= CENTER_TOLERANCE) return "BAL IN MIDDEN";
            return lastCenterGreen.x < CAMERA_WIDTH/2.0 ? "BAL LINKS" : "BAL RECHTS";
        }

        public int getNumberOfPurpleObjects() { return numberOfPurpleObjects; }
        public double getLastDistancePurple() { return lastDistancePurple; }
        public String getPurpleBallPosition() {
            if (numberOfPurpleObjects == 0) return "Geen bal gedetecteerd";
            if (Math.abs(lastCenterPurple.x - CAMERA_WIDTH/2.0) <= CENTER_TOLERANCE) return "BAL IN MIDDEN";
            return lastCenterPurple.x < CAMERA_WIDTH/2.0 ? "BAL LINKS" : "BAL RECHTS";
        }

        @Override
        public Mat processFrame(Mat input)
        {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Groen masker
            Mat greenMask = new Mat();
            Core.inRange(hsv, lowerGreen, upperGreen, greenMask);
            Imgproc.morphologyEx(greenMask, greenMask, Imgproc.MORPH_CLOSE,
                    Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(20, 20)));
            Imgproc.morphologyEx(greenMask, greenMask, Imgproc.MORPH_OPEN,
                    Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(6, 6)));

            // Paars masker
            Mat purpleMask = new Mat();
            Core.inRange(hsv, lowerPurple, upperPurple, purpleMask);
            Imgproc.morphologyEx(purpleMask, purpleMask, Imgproc.MORPH_CLOSE,
                    Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(20, 20)));
            Imgproc.morphologyEx(purpleMask, purpleMask, Imgproc.MORPH_OPEN,
                    Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(6, 6)));

            // Groene ballen detecteren
            numberOfGreenObjects = 0;
            lastCenterGreen = new Point(-1, -1);
            List<MatOfPoint> greenContours = new ArrayList<>();
            Imgproc.findContours(greenMask, greenContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            for (MatOfPoint contour : greenContours) {
                double area = Imgproc.contourArea(contour);
                if (area < 20) continue;
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                Point center = new Point();
                float[] radius = new float[1];
                Imgproc.minEnclosingCircle(contour2f, center, radius);
                if (radius[0] > 2) {
                    double pixelDiameter = 2 * radius[0];
                    lastDistanceGreen = (realDiameter * focalLength) / pixelDiameter;
                    lastCenterGreen = center;
                    numberOfGreenObjects++;
                    Imgproc.circle(input, center, (int) radius[0], new Scalar(0, 255, 0), 2);
                }
            }

            // Paarse ballen detecteren
            numberOfPurpleObjects = 0;
            lastCenterPurple = new Point(-1, -1);
            List<MatOfPoint> purpleContours = new ArrayList<>();
            Imgproc.findContours(purpleMask, purpleContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            for (MatOfPoint contour : purpleContours) {
                double area = Imgproc.contourArea(contour);
                if (area < 20) continue;
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                Point center = new Point();
                float[] radius = new float[1];
                Imgproc.minEnclosingCircle(contour2f, center, radius);
                if (radius[0] > 2) {
                    double pixelDiameter = 2 * radius[0];
                    lastDistancePurple = (realDiameter * focalLength) / pixelDiameter;
                    lastCenterPurple = center;
                    numberOfPurpleObjects++;
                    Imgproc.circle(input, center, (int) radius[0], new Scalar(255, 0, 255), 2);
                }
            }

            greenMask.release();
            purpleMask.release();

            return input;
        }

        @Override
        public void onViewportTapped()
        {
            viewPortPaused = !viewPortPaused;
            if(viewPortPaused) externalWebcam.pauseViewport();
            else externalWebcam.resumeViewport();
        }
    }
}

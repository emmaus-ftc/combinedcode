// AutonomousCamera met scan + verticale tracking van bal (verbeterde versie)
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name ="autonomousCamera", group = "Robot")
public class AutonomousCamera extends LinearOpMode {
    OpenCvWebcam webcam;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    OpenCvPipeline currentPipeline;

    // Motors / servo
    private DcMotor LF = null;
    private DcMotor LB = null;
    private DcMotor RF = null;
    private DcMotor RB = null;
    private DcMotor Intake = null;
    private Servo CamServo = null;
    //private CRServo IntakeServo;

    private DcMotor ShooterMotor = null;
    private Servo ShooterServo = null;

    // State
    boolean arrived = false;
    private int amountBallsPickedUp = 0;
    final int GPP = 21;
    final int PGP = 22;
    final int PPG = 23;
    private ElapsedTime runtime = new ElapsedTime();
    String volgorde = "";
    private char[] volgordeToChar;

    //Mode the autonomous is currently in:
    private String Mode = "Schieten";

    // Intake lock
    private boolean intakeLockedOn = false;

    // --- CAMERA SCAN / TRACK PARAMETERS ---
    private double camScanDir = 1; // +1 = naar CAM_MAX_POS, -1 = naar CAM_MIN_POS
    private boolean trackingBall = false;
    private static final double CAM_MIN_POS = 0.07; // beginpositie
    private static final double CAM_MAX_POS = 0.40; // maximale kijk-omlaag

    private static final double CAM_SCAN_SPEED = 0.02; // scan snelheid (servo stappen per loop)
    private static final double CAM_TRACK_KP = 0.08;   // smoothing / proportional factor voor servo tijdens tracking

    private static final int CAM_Y_DEADBAND = 8;
    private static final int CAMERA_HEIGHT_PIXELS = 240;

    private static final int LOCK_LOST_THRESHOLD = 6; // aantal iteraties zonder zicht -> release intake

    // Detection tuning (pipeline)
    private static final double MIN_CONTOUR_AREA = 250.0; // verhoogd om ruis te vermijden
    private static final float MIN_RADIUS = 6.0f;         // minimale radius (pixels) voor geldige bol
    private static final double MAX_DETECTION_DISTANCE = 1000.0; // mm (of willekeurige units uit je afstandsformule) - voorkomt extreem verre blobs

    // AprilTag camera intrinsics
    double tagsize = 0.166;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    //Used for Scanning for balls
    private boolean turnedRightOnce = false;
    //start new timer
    ElapsedTime scanTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // --- hardware init ---
        LF = hardwareMap.get(DcMotor.class, "LF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        //IntakeServo = hardwareMap.get(CRServo.class, "IntakeServo");

        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);
        //IntakeServo.setDirection(DcMotor.Direction.FORWARD);

        ShooterMotor = hardwareMap.get(DcMotor.class, "motorShooter");
        ShooterServo = hardwareMap.get(Servo.class, "shooterServo");

        ShooterMotor.setDirection(DcMotor.Direction.REVERSE);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        CamServo = hardwareMap.get(Servo.class, "camServo");

        int cameraMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId
        );

        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        ballDetectionPipeLine colorDetectionPipeline = new ballDetectionPipeLine(webcam);

        currentPipeline = colorDetectionPipeline;
        webcam.setPipeline(currentPipeline);
        webcam.setMillisecondsPermissionTimeout(5000);

        webcam.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override public void onError(int errorCode) { /* handle if needed */ }
        });

        waitForStart();

        // startpositie camera
        CamServo.setPosition(CAM_MIN_POS);

        // lock lost counter
        int lockLostCounter = 0;

        ShooterServo.setPosition(0.6);

        while (opModeIsActive()) {

            // --------------- SCHIETEN ----------------
            if(Mode == "Schieten") {

                ShooterMotor.setPower(1);
                sleep(800);

                driveForward(0.5);
                sleep(400);
                stopDriving();

                ShooterServo.setPosition(1);

                sleep(500);
                ShooterServo.setPosition(0.6);

                Intake.setPower(0.5);
                sleep(1000);

                ShooterServo.setPosition(1);

                sleep(500);
                ShooterServo.setPosition(0.6);

                Intake.setPower(0);
                ShooterMotor.setPower(0);

                Mode = "BalZoeken";
            }




            // ---------------- APRILTAG ----------------
//            if (currentPipeline == aprilTagDetectionPipeline && Mode == "AprilTagVolgorde") {
//                ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
//                if (detections != null && detections.size() > 0) {
//                    for (AprilTagDetection tag : detections) {
//                        if (tag.id == GPP) volgorde = "GPP";
//                        if (tag.id == PGP) volgorde = "PGP";
//                        if (tag.id == PPG) volgorde = "PPG";
//                        volgordeToChar = volgorde.toCharArray();
//                        currentPipeline = colorDetectionPipeline;
//                        webcam.setPipeline(currentPipeline);
//                        Mode = "BalZoeken";
//                        telemetry.addData("AprilTag", "Found id=" + tag.id + " -> switching to ball pipeline");
//                        telemetry.update();
//                    }
//                } else {
//                    turnLeft(0.1);
//                }
//            }


            // ---------------- BALL TRACKING + CAMERA SERVO ----------------
            if (currentPipeline instanceof ballDetectionPipeLine && Mode == "BalZoeken") {
                ballDetectionPipeLine pipe = (ballDetectionPipeLine) currentPipeline;

                double camPos = CamServo.getPosition();
                double greenDist = pipe.getLastDistanceGreen();
                double purpleDist = pipe.getLastDistancePurple();

                // consider a ball "seen" only if we have a positive distance and it's within a sane max range
                boolean greenSeen = (greenDist > 0 && greenDist < MAX_DETECTION_DISTANCE);
                boolean purpleSeen = (purpleDist > 0 && purpleDist < MAX_DETECTION_DISTANCE);

                boolean nearGreen = (greenSeen && greenDist < 20.0);
                boolean nearPurple = (purpleSeen && purpleDist < 20.0);

                // updated intake lock logic
                if (nearGreen || nearPurple) {
                    intakeLockedOn = true;
                    driveForward(0.5);
                    sleep(500);
                    driveForward(0);
                    amountBallsPickedUp++;
                    sleep(100);
                }

                Intake.setPower(intakeLockedOn ? 1.0 : 0.0);

                if (amountBallsPickedUp < 2 && arrived) {
                    arrived = false;
                    lockLostCounter = 0;
                    intakeLockedOn = false;
                }

                // release intake lock after not seeing any ball for some cycles
                boolean anyBallSeen = greenSeen || purpleSeen;
                if (anyBallSeen) {
                    lockLostCounter = 0;
                } else {
                    lockLostCounter++;
                    if (lockLostCounter >= LOCK_LOST_THRESHOLD) {
                        intakeLockedOn = false;
                        lockLostCounter = LOCK_LOST_THRESHOLD; // clamp
                    }
                }
                //IntakeServo.setPower(intakeLockedOn ? 0.5 : 0.0);

                // choose which ball to track (closest)
                double ballY = -1;
                if (anyBallSeen) {
                    if (greenSeen && purpleSeen) {
                        if (greenDist <= purpleDist) {
                            ballY = pipe.getLastCenterGreen().y;
                        } else {
                            ballY = pipe.getLastCenterPurple().y;
                        }
                    } else if (greenSeen) {
                        ballY = pipe.getLastCenterGreen().y;
                    } else {
                        ballY = pipe.getLastCenterPurple().y;
                    }
                }

                // SCAN <-> TRACK
                double targetPos = camPos; // used for telemetry as well
                if (!arrived) {
                    if (lockLostCounter >= 6) {
                        // SCAN mode: move servo slowly up/down

                        trackingBall = false;
                        //camPos += camScanDir * CAM_SCAN_SPEED;
                        if(!turnedRightOnce) {
                            turnRight(0.2);
                            sleep(800);
                            stopDriving();
                            turnedRightOnce = true;
                            scanTimer.reset();
                        }
                        if(scanTimer.seconds() <= 5) {
                            if (scanTimer.seconds() <= 3) {
                                camPos = CAM_MAX_POS - 0.1;
                                turnLeft(0.2);
                            } else if (scanTimer.seconds() <= 6){
                                camPos = CAM_MIN_POS + 0.1;
                                turnRight(0.2);
                            }else scanTimer.reset();
                        }
//                        if (camPos > CAM_MAX_POS) {
//                            camPos = CAM_MAX_POS;
//                            camScanDir = -1;
//                        } else if (camPos < CAM_MIN_POS) {
//                            camPos = CAM_MIN_POS;
//                            camScanDir = 1;
//                        }
//                        targetPos = camPos;
                    } else {
                        // TRACK mode: map ballY to servo position with smoothing
                        trackingBall = true;
                        if (ballY >= 0) {
                            // normalize Y -> 0..1 (top=0, bottom=CAMERA_HEIGHT_PIXELS)
                            double normalizedY = ballY / (double) CAMERA_HEIGHT_PIXELS;
                            normalizedY = Math.max(0.0, Math.min(1.0, normalizedY));
                            double mapped = CAM_MIN_POS + normalizedY * (CAM_MAX_POS - CAM_MIN_POS);
                            // smoothing/proportional approach
                            double delta = mapped - camPos;
                            camPos += delta * CAM_TRACK_KP;
                            targetPos = mapped;
                        } else if(ballY < 0) {
                            double normalizedY = ballY / (double) CAMERA_HEIGHT_PIXELS;
                            normalizedY = Math.max(0.0, Math.min(1.0, normalizedY));
                            double mapped = CAM_MIN_POS + normalizedY * (CAM_MAX_POS - CAM_MIN_POS);
                            // smoothing/proportional approach
                            double delta = mapped - camPos;
                            camPos -= delta * CAM_TRACK_KP;
                            targetPos = mapped;
                        }
                        // ensure clamped
                        camPos = Math.max(CAM_MIN_POS, Math.min(CAM_MAX_POS, camPos));
                    }

                    CamServo.setPosition(camPos);
                }

                // --------------- Drive logic based on volgorde ---------------
                if (!arrived) {
                    //if (volgordeToChar != null && amountBallsPickedUp < volgordeToChar.length) {
                    //char currentTarget = volgordeToChar[amountBallsPickedUp];
                    //if (currentTarget == 'G') {

                    //} else if (currentTarget == 'P') {
                    if (purpleSeen) {
                        if (purpleDist > 20) {
                            String purplePos = pipe.getPurpleBallPosition();
                            if (purplePos.equals("BAL RECHTS")) turnRight(0.2);
                            else if (purplePos.equals("BAL LINKS")) turnLeft(0.2);
                            else if (purplePos.equals("BAL IN MIDDEN")) driveForward(0.4);
                        } else {
                            stopDriving();
                            arrived = true;
                        }
                    }
                    if (greenSeen) {
                        if (greenDist > 20) {
                            String greenPos = pipe.getGreenBallPosition();
                            if (greenPos.equals("BAL RECHTS")) turnRight(0.2);
                            else if (greenPos.equals("BAL LINKS")) turnLeft(0.2);
                            else if (greenPos.equals("BAL IN MIDDEN")) driveForward(0.4);
                        } else {
                            stopDriving();
                            arrived = true;
                        }
                    }
                    //}
                }

                // telemetry - handig om tuning te doen tijdens testen
                telemetry.addData("camPos", "%.3f", CamServo.getPosition());
                telemetry.addData("targetPos(approx)", "%.3f", targetPos);
                telemetry.addData("anyBallSeen", anyBallSeen);
                telemetry.addData("greenSeen/dist", greenSeen ? String.format("%.1f", greenDist) : "no");
                telemetry.addData("purpleSeen/dist", purpleSeen ? String.format("%.1f", purpleDist) : "no");
                telemetry.addData("intakeLockedOn", intakeLockedOn);
                telemetry.addData("trackingBall", trackingBall);
                telemetry.addData("lockLostCounter", lockLostCounter);
                telemetry.update();
            }

            sleep(50);
        }

        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    // ----------------- basic driving helpers -----------------
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

    // ----------------- ball detection pipeline -----------------
    class ballDetectionPipeLine extends OpenCvPipeline {
        boolean viewPortPaused;
        private static final int CAMERA_WIDTH = 320;
        private static final int CAMERA_HEIGHT = 240;
        private static final int CENTER_TOLERANCE = 30;

        Mat hsv = new Mat();
        Mat hierarchy = new Mat();

        // kleurranges (kan je tunen)
        public Scalar lowerGreen = new Scalar(85, 180, 100);
        public Scalar upperGreen = new Scalar(95, 255, 255);
        public Scalar lowerPurple = new Scalar(133, 90, 0);
        public Scalar upperPurple = new Scalar(145, 255, 255);

        private volatile int numberOfGreenObjects = 0;
        private volatile double lastDistanceGreen = -1;
        private volatile Point lastCenterGreen = new Point(-1, -1);

        private volatile int numberOfPurpleObjects = 0;
        private volatile double lastDistancePurple = -1;
        private volatile Point lastCenterPurple = new Point(-1, -1);

        private OpenCvWebcam externalWebcam;
        private final double realDiameter = 12.70;
        private final double focalLength = 301.00;

        public ballDetectionPipeLine(OpenCvWebcam webcam) {
            this.externalWebcam = webcam;
        }

        public int getNumberOfGreenObjects() { return numberOfGreenObjects; }
        public double getLastDistanceGreen() { return lastDistanceGreen; }
        public Point getLastCenterGreen() { return lastCenterGreen; }
        public String getGreenBallPosition() {
            if (numberOfGreenObjects == 0) return "Geen bal gedetecteerd";
            if (Math.abs(lastCenterGreen.x - CAMERA_WIDTH/2.0) <= CENTER_TOLERANCE)
                return "BAL IN MIDDEN";
            return lastCenterGreen.x < CAMERA_WIDTH/2.0 ? "BAL LINKS" : "BAL RECHTS";
        }

        public int getNumberOfPurpleObjects() { return numberOfPurpleObjects; }
        public double getLastDistancePurple() { return lastDistancePurple; }
        public Point getLastCenterPurple() { return lastCenterPurple; }
        public String getPurpleBallPosition() {
            if (numberOfPurpleObjects == 0) return "Geen bal gedetecteerd";
            if (Math.abs(lastCenterPurple.x - CAMERA_WIDTH/2.0) <= CENTER_TOLERANCE)
                return "BAL IN MIDDEN";
            return lastCenterPurple.x < CAMERA_WIDTH/2.0 ? "BAL LINKS" : "BAL RECHTS";
        }

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Mat greenMask = new Mat();
            Core.inRange(hsv, lowerGreen, upperGreen, greenMask);
            Mat purpleMask = new Mat();
            Core.inRange(hsv, lowerPurple, upperPurple, purpleMask);

            // reset
            numberOfGreenObjects = 0;
            lastCenterGreen = new Point(-1, -1);
            lastDistanceGreen = -1;

            List<MatOfPoint> greenContours = new ArrayList<>();
            Imgproc.findContours(greenMask, greenContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : greenContours) {
                double area = Imgproc.contourArea(contour);
                if (area < MIN_CONTOUR_AREA) continue; // filter ruis
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                Point center = new Point();
                float[] radius = new float[1];
                Imgproc.minEnclosingCircle(contour2f, center, radius);
                if (radius[0] > MIN_RADIUS) {
                    double pixelDiameter = 2.0 * radius[0];
                    double distance = (realDiameter * focalLength) / pixelDiameter;
                    // accept only sane distances
                    if (distance > 0 && distance < MAX_DETECTION_DISTANCE) {
                        lastDistanceGreen = distance;
                        lastCenterGreen = center;
                        numberOfGreenObjects++;
                    }
                }
            }

            // purple
            numberOfPurpleObjects = 0;
            lastCenterPurple = new Point(-1, -1);
            lastDistancePurple = -1;

            List<MatOfPoint> purpleContours = new ArrayList<>();
            Imgproc.findContours(purpleMask, purpleContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : purpleContours) {
                double area = Imgproc.contourArea(contour);
                if (area < MIN_CONTOUR_AREA) continue;
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                Point center = new Point();
                float[] radius = new float[1];
                Imgproc.minEnclosingCircle(contour2f, center, radius);
                if (radius[0] > MIN_RADIUS) {
                    double pixelDiameter = 2.0 * radius[0];
                    double distance = (realDiameter * focalLength) / pixelDiameter;
                    if (distance > 0 && distance < MAX_DETECTION_DISTANCE) {
                        lastDistancePurple = distance;
                        lastCenterPurple = center;
                        numberOfPurpleObjects++;
                    }
                }
            }

            greenMask.release();
            purpleMask.release();
            return input;
        }

        @Override
        public void onViewportTapped() {
            viewPortPaused = !viewPortPaused;
            if (viewPortPaused) externalWebcam.pauseViewport();
            else externalWebcam.resumeViewport();
        }
    }
}

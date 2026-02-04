package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "NewDriveCode")
public class NewDriveCode extends LinearOpMode {

    // Timers
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime servoTimer = new ElapsedTime();

    // Drive motors
    private DcMotor LF, LB, RF, RB;

    // Intake & Shooter
    private DcMotor Intake;
    private DcMotorEx ShooterMotor;
    private Servo ShooterServo;
    private Servo CamServo;
    private Servo FeedServo;

    private int intake_power = 0;
    private boolean XWasPressed = false;
    private boolean YWasPressed = false;
    private boolean Ypressed = false;
    
    private boolean BWasPressed = false;
    private boolean AWasPressed = false;



    // Shooter motor toggle
    private boolean shooterOn;

    private int TARGET_VELOCITY = 1100;

    private double velocity = 0;                // Safe ramp-up velocity
    private final int VELOCITY_STEP = 90;       // Velocity increase per loop
    private final int VELOCITY_TOLERANCE = 20;  // Shooter ready tolerance

    //Shooter shoot mode:
    private String shooterMode = "close";

    private boolean Small_child_detected = false;


    @Override
    public void runOpMode() {

        // Hardware map
        LF = hardwareMap.get(DcMotor.class, "LF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        RB = hardwareMap.get(DcMotor.class, "RB");

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        ShooterMotor = hardwareMap.get(DcMotorEx.class, "motorShooter");
        ShooterServo = hardwareMap.get(Servo.class, "shooterServo");
        CamServo = hardwareMap.get(Servo.class, "camServo");
        FeedServo = hardwareMap.get(Servo.class, "feedServo");

        // Directions
        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ShooterMotor.setDirection(DcMotor.Direction.REVERSE);
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        PIDFCoefficients PIDFclose = new PIDFCoefficients(
                0.002,
                0.000,
                0.0000,
                14.5
        );
        PIDFCoefficients PIDFfar = new PIDFCoefficients(
                0.002,
                0.000,
                0.0000,
                15.6
        );
        ShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDFclose);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        ShooterServo.setPosition(0.76);
        CamServo.setPosition(0.2);
        FeedServo.setPosition(0.6);

        while (opModeIsActive()) {

            // ---------- DRIVE ----------
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x * 0.7;

            double driveSpeedMultiplier = 0.75;
            double maximizePower = gamepad1.right_bumper ? 0.25 : 1.0;

            double leftFrontPower = (axial + lateral + yaw) * driveSpeedMultiplier;
            double rightFrontPower = (axial - lateral - yaw) * driveSpeedMultiplier;
            double leftBackPower = (axial - lateral + yaw) * driveSpeedMultiplier;
            double rightBackPower = (axial + lateral - yaw) * driveSpeedMultiplier;

            leftFrontPower = Math.max(-maximizePower, Math.min(maximizePower, leftFrontPower));
            rightFrontPower = Math.max(-maximizePower, Math.min(maximizePower, rightFrontPower));
            leftBackPower = Math.max(-maximizePower, Math.min(maximizePower, leftBackPower));
            rightBackPower = Math.max(-maximizePower, Math.min(maximizePower, rightBackPower));

            LF.setPower(leftFrontPower);
            RF.setPower(rightFrontPower);
            LB.setPower(leftBackPower);
            RB.setPower(rightBackPower);

            // ---------- INTAKE ----------
            Ypressed = gamepad1.y;
            if (gamepad1.dpad_left ) intake_power = -1;

            if (Ypressed && !YWasPressed) {
                intake_power = (intake_power == 0) ? 1 : 0;
            }
            YWasPressed = Ypressed;
            Intake.setPower(intake_power / 2.0);

            // --------- SHOOTER TOGGLE --------
            boolean XPressed = gamepad2.x;
            if (XPressed && !XWasPressed) {
                shooterOn = !shooterOn;        // Toggle motor state
            }
            XWasPressed = XPressed;

            // ----- SLOWLY INCREASE VELOCITY -------
            if (shooterOn) {
                velocity = Math.min(velocity + VELOCITY_STEP, TARGET_VELOCITY);
            } else {
                velocity = Math.max(velocity - VELOCITY_STEP, 0);
            }
            ShooterMotor.setVelocity(velocity);

            //------- START SERVO SEQUENCE ------
            boolean aPressed = gamepad2.a;
            if (aPressed && !AWasPressed) {
                 servoTimer.reset();
            }
            AWasPressed = aPressed;
            
            //------- TOGGLE SPEED SHOOTER - CLOSE <> FAR -------
            boolean bPressed = gamepad2.b;
            if (bPressed && !BWasPressed) {
                if(shooterMode.equals("far")) {
                    ShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDFclose);
                    TARGET_VELOCITY = 1100;
                    shooterMode = "close";
                } else if(shooterMode.equals("close")) {
                    ShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDFfar);
                    TARGET_VELOCITY = 1400;
                    shooterMode = "far";
                }
            }
            BWasPressed = bPressed;
            

            //------- CHECK IF MOTOR RUNNING -------
            if (ShooterMotor.getVelocity() >= TARGET_VELOCITY - VELOCITY_TOLERANCE) {
                if (servoTimer.seconds() >= 0.4) {
                    ShooterServo.setPosition(0.55);
                }

            }

            if (servoTimer.seconds() >= 0.8) {
                ShooterServo.setPosition(0.76);
            }
            //--------- SERVO SEQUENCE ---------
            if (servoTimer.seconds() >= 1.4) {
                FeedServo.setPosition(0.6);
            } else if (servoTimer.seconds() >= 1.1) {
                FeedServo.setPosition(0.45);
            }

            // Manual override Bumper
            if (gamepad2.left_bumper) {
                ShooterServo.setPosition(0.76);
            }


            if (gamepad1.a) {
                Small_child_detected = true;
            }
            if (gamepad1.b) {
                Small_child_detected = false;
            }

            if(Small_child_detected) {
                driveSpeedMultiplier = 0.01;
            }


            telemetry.addData("Shooter Servo Pos: ", ShooterServo.getPosition());
            telemetry.addData("Feed Servo Pos: ", FeedServo.getPosition());
            telemetry.addData("servo timer: ", servoTimer.seconds());
            telemetry.addData("target_velocity: ", TARGET_VELOCITY);
            telemetry.addData("velocity: ", velocity);
            telemetry.addData("shooter ready: ", ShooterMotor.getVelocity() >= TARGET_VELOCITY - VELOCITY_TOLERANCE);
            telemetry.addData("'actual' velocity", ShooterMotor.getVelocity());
            telemetry.addData("Small child", Small_child_detected);
            telemetry.addData("multiplier", driveSpeedMultiplier);
            telemetry.addData("shooter mode: ", shooterMode);
            telemetry.update();
        }
    }
}

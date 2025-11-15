package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "NewDriveCode")
public class NewDriveCode extends LinearOpMode {

    // Timer
    private ElapsedTime runtime = new ElapsedTime();

    // Drive motors
    private DcMotor LF = null;
    private DcMotor LB = null;
    private DcMotor RF = null;
    private DcMotor RB = null;

    private int intake_power = 0;
    private int shooter_power = 0;

    // Mechanisms
    private DcMotor Intake = null;
    private DcMotor ShooterMotor = null;
    private Servo ShooterServo = null;
    private Servo CamServo = null;

    ElapsedTime timer = new ElapsedTime();
    boolean shooterActivated = false;

    @Override
    public void runOpMode() {

        // Drive motors
        LF = hardwareMap.get(DcMotor.class, "LF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        RB = hardwareMap.get(DcMotor.class, "RB");

        // Other mechanisms
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        ShooterMotor = hardwareMap.get(DcMotor.class, "motorShooter"); // From OpenDagShooter
        ShooterServo = hardwareMap.get(Servo.class, "shooterServo");   // From OpenDagShooter
        CamServo = hardwareMap.get(Servo.class, "camServo");   // From OpenDagShooter

        // Set directions
        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.REVERSE);
        ShooterMotor.setDirection(DcMotor.Direction.REVERSE);

        // Motor behaviors
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        ShooterServo.setPosition(0.6);
        CamServo.setPosition(0.6);

        // Main loop
        while (opModeIsActive()) {

//             ----- Drive control -----
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double driveSpeedMultiplier = 0.75;
            double maximizePower = gamepad1.right_bumper ? 0.25 : 1.0;

            double leftFrontPower = (axial + lateral + yaw) * driveSpeedMultiplier;
            double rightFrontPower = (axial - lateral - yaw) * driveSpeedMultiplier;
            double leftBackPower = (axial - lateral + yaw) * driveSpeedMultiplier;
            double rightBackPower = (axial + lateral - yaw) * driveSpeedMultiplier;

            // Clamp power to limits
            leftFrontPower = Math.max(-maximizePower, Math.min(maximizePower, leftFrontPower));
            rightFrontPower = Math.max(-maximizePower, Math.min(maximizePower, rightFrontPower));
            leftBackPower = Math.max(-maximizePower, Math.min(maximizePower, leftBackPower));
            rightBackPower = Math.max(-maximizePower, Math.min(maximizePower, rightBackPower));

            // Apply drive power
            LF.setPower(leftFrontPower);
            RF.setPower(rightFrontPower);
            LB.setPower(leftBackPower);
            RB.setPower(rightBackPower);

            // ----- Intake control (gamepad2) -----
            if (gamepad2.y) {
                if (intake_power == 0) {
                    intake_power = 1;
                } else {
                    intake_power = 0;
                }
            } else if (gamepad2.dpad_left && gamepad2.dpad_right) {
                if (intake_power == 0) {
                    intake_power = -1;
                } else {
                    intake_power = 0;
                }
            }

            // ----- Shooter toggle -----
            if (gamepad2.x && !shooterActivated) {
                if (shooter_power == 0) {
                    shooter_power = 1;
                } else {
                    shooter_power = 0;
                }
                shooterActivated = true;
                timer.reset(); // start 2-second countdown
            }

            if (shooterActivated && timer.seconds() > 1.0) {
                ShooterServo.setPosition(0.8);
                shooterActivated = false; // prevent repeated activation
            }

            Intake.setPower(intake_power);
            ShooterMotor.setPower(shooter_power);

            // ----- Shooter control (gamepad1) -----
            if (gamepad2.a) {
                ShooterServo.setPosition(0.75);
            }
            if (gamepad2.b) {
                ShooterServo.setPosition(0.6);
            }
            if (gamepad2.right_bumper) {
                ShooterServo.setPosition(0.65);
            }
        }
    }
}

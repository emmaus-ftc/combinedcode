package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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

    // Shooter & Intake
    private int intake_power = 0;
    private int shooter_power = 0;
    private boolean XWasPressed = false;
    private boolean XPressed = false;
    private boolean reverseShooterOn = false;

    private DcMotor Intake = null;
    private DcMotor ShooterMotor = null;
    private Servo ShooterServo = null;
    private Servo CamServo = null;
    //private CRServo IntakeServo;

    private ElapsedTime timer = new ElapsedTime();
    private boolean shooterActivated = false;

    @Override
    public void runOpMode() {

        // Drive motors
        LF = hardwareMap.get(DcMotor.class, "LF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        RB = hardwareMap.get(DcMotor.class, "RB");

        // Other mechanisms
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        ShooterMotor = hardwareMap.get(DcMotor.class, "motorShooter");
        ShooterServo = hardwareMap.get(Servo.class, "shooterServo");
        CamServo = hardwareMap.get(Servo.class, "camServo");


        // Set motor directions
        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        // Motor behaviors
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Initialize Motor of Shooter
        ShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterMotor.setDirection(DcMotor.Direction.REVERSE);
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //target velocity in ticks per second
        double TARGET_VELOCITY = 1800;

        //Let Code know Initialization is ready
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        ShooterServo.setPosition(0.6);
        CamServo.setPosition(0.2);

        // Main loop
        while (opModeIsActive()) {

            // ----- Drive control -----
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

            // ----- Intake control -----
            if (gamepad2.y) {
                intake_power = (intake_power == 0) ? 1 : 0;
            } else if (gamepad2.dpad_left && gamepad2.dpad_right) {
                intake_power = (intake_power == 0) ? -1 : 0;
            }
            Intake.setPower((double) intake_power /2);
            //IntakeServo.setPower(intake_power);

            // ----- Shooter motor toggle -----
            XPressed = gamepad2.x;

            if (XPressed && !XWasPressed) {
                shooter_power = (shooter_power == 0) ? 1 : 0; // toggle motor

                if (shooter_power == 1) {
                    shooterActivated = true; // start servo feed timer
                    timer.reset();
                }
            }
            XWasPressed = XPressed;
            if(gamepad2.left_trigger > 0.5) {
                ShooterMotor.setPower(-1);
            } else {
                ShooterMotor.setPower(shooter_power);
            }

            if(gamepad2.right_trigger > 0.5) {
                ShooterMotor.setPower(1);
            } else {
                ShooterMotor.setPower(shooter_power);
            }

            // ----- Shooter servo feed with spin-up delay -----
            if (shooterActivated) {
                if (timer.seconds() > 1.0 && timer.seconds() < 1.5) {
                    // Wait 1 second for motor spin-up, then feed ball
                    ShooterServo.setPosition(1);
                } else if (timer.seconds() >= 1.5) {
                    ShooterServo.setPosition(0.6);
                    shooterActivated = false; // done feeding
                }
            }

            // ----- Manual servo adjustments -----
            if (gamepad2.a) {
                ShooterServo.setPosition(1);
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

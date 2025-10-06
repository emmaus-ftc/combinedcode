package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name="CombinedCode")
public class CombinedCode extends LinearOpMode {
    //Establish start time (not necessary)
    private ElapsedTime runtime = new ElapsedTime();
    //declare all motors and servo's
    private DcMotor LF = null;
    private DcMotor LB = null;
    private DcMotor RF = null;
    private DcMotor RB = null;

    private DcMotor CascadeLift;

    private CRServo slurp;
    private Servo pols;

    //declare and define all variables
    boolean SlideIsOn = false;
    boolean SlideIsReversed = false;
    boolean aDown = false;
    boolean yDown = false;
    boolean xDown = false;
    boolean yPressed = false;
    boolean aPressed = false;
    
    boolean leftBumperWasPressed = false;
    boolean rightBumperWasPressed = false;
    double power = 1;

@Override 
public void runOpMode() {

    //define all motors and servo's
    LF  = hardwareMap.get(DcMotor.class, "LF");
    LB  = hardwareMap.get(DcMotor.class, "LB");
    RF = hardwareMap.get(DcMotor.class, "RF");
    RB = hardwareMap.get(DcMotor.class, "RB");
    
    CascadeLift = hardwareMap.get(DcMotor.class, "CascadeLift");
    
    slurp = hardwareMap.get(CRServo.class, "slurp");
    pols = hardwareMap.get(Servo.class, "pols");
    
    // //initialize servo positions
    // pols.setPosition(0.8);
    // bakje.setPosition(0.37);

    //initialize motor directions
    LF.setDirection(DcMotor.Direction.FORWARD);
    LB.setDirection(DcMotor.Direction.REVERSE);
    RF.setDirection(DcMotor.Direction.FORWARD);
    RB.setDirection(DcMotor.Direction.FORWARD);
    
    //set motor behavior to brake for smoother driving
    LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    double leftFrontPower = 0;
    double rightFrontPower = 0;
    double leftBackPower   = 0;
    double rightBackPower  = 0;
    double driveSpeedMultiplier = 0.75;
    double maximizePower = 1;

    // change telemetry status to initialized
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    waitForStart();
    runtime.reset();

    // run until the end of the match
    while (opModeIsActive()) {

        // declare and define variable each runtime tick
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        //declare, define variable -> calculate power for each wheel
        leftFrontPower  = (axial + lateral + yaw) * driveSpeedMultiplier;
        rightFrontPower = (axial - lateral - yaw) * driveSpeedMultiplier;
        leftBackPower   = (axial - lateral + yaw) * driveSpeedMultiplier;
        rightBackPower  = (axial + lateral - yaw) * driveSpeedMultiplier;
        if(gamepad1.right_bumper) {
            maximizePower = 0.25;
        }else{
            maximizePower = 1;
        }
        leftFrontPower = Math.min(leftFrontPower, maximizePower);
        rightFrontPower = Math.min(rightFrontPower, maximizePower);
        leftBackPower = Math.min(leftBackPower, maximizePower);
        rightBackPower = Math.min(rightBackPower, maximizePower);
        leftFrontPower = Math.max(leftFrontPower, -maximizePower);
        rightFrontPower = Math.max(rightFrontPower, -maximizePower);
        leftBackPower = Math.max(leftBackPower, -maximizePower);
        rightBackPower = Math.max(rightBackPower, -maximizePower);

        //set the max value for wheel power to 1
        
        
        // set power per wheel based on previous calculated variable
        LF.setPower(leftFrontPower);
        RF.setPower(rightFrontPower);
        LB.setPower(leftBackPower);
        RB.setPower(rightBackPower);
        
        if (gamepad1.y) {
            CascadeLift.setTargetPosition(-7300);  // Set the target position for lift
            //2500 is closest, 2000 is used for debugging
            CascadeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);  // Move to target position
            CascadeLift.setPower(1);  // Set motor power to move towards target
        }
            
            // Control CascadeLift with the A button (move to home position)
        if (gamepad1.a) {
            CascadeLift.setTargetPosition(0);  // Home position
            CascadeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);  // Move to target position
            CascadeLift.setPower(1);  // Set motor power to move towards target
        }
            
            // Stop CascadeLift when it reaches the target position
        if (!CascadeLift.isBusy()) {
            CascadeLift.setPower(0);  // Stop motor once target position is reached
        }

        if (gamepad2.left_trigger > 0 && gamepad2.right_trigger == 0) {
            pols.setPosition(0.6);
        }

        if (gamepad2.left_bumper && !gamepad2.right_bumper) {
            slurp.setPower(-0.9);
        }
 
        if (gamepad2.right_trigger > 0 && !gamepad2.right_bumper && gamepad2.left_trigger == 0) {
            pols.setPosition(0.86);
            slurp.setPower(0.9);
        }
        
        if (gamepad2.right_trigger == 0 && gamepad2.right_bumper) {
            slurp.setPower(0.9);
        }
        
        if (gamepad2.right_trigger == 0 && !gamepad2.right_bumper && !gamepad2.left_bumper) {
            slurp.setPower(0);
        }
        
        if(gamepad2.x) {
            pols.setPosition(0.78);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.update();
    }
}}

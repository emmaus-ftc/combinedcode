package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class Drive extends LinearOpMode {

    private DcMotor left;
    private DcMotor right;

    // --- Toegevoegde servo ---
    private Servo servo;

    @Override
    public void runOpMode() {

        left = hardwareMap.get(DcMotor.class, "Left");
        right = hardwareMap.get(DcMotor.class, "Right");

        // Servo hardware mapping
        servo = hardwareMap.get(Servo.class, "grijper");

        right.setDirection(DcMotor.Direction.REVERSE);
        left.setDirection(DcMotor.Direction.FORWARD);

        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servo.setPosition(0.6);
        waitForStart();

        while (opModeIsActive()) {
       

            double left_x = gamepad1.left_stick_x;
            double left_y = gamepad1.left_stick_y;
            double right_x = gamepad1.right_stick_x;
            double right_y = gamepad1.right_stick_y;

            // Drive
            left.setPower(left_y - right_x);
            right.setPower(left_y + right_x);

            // --- Servo bediening ---
            if (gamepad1.x) {
                servo.setPosition(0.6);
            }
            if (gamepad1.y) {
                servo.setPosition(1);
            }

            // Telemetry (optioneel)
            telemetry.addData("Servo pos", servo.getPosition());
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "OriginalTeleOp", group = "Iterative Opmode")
public class MecanumRobotTeleOpIterative extends LumenBaseOpMode {

    @Override
    public void loop() {
        // Handle driving
        driveMecanum();

        // Handle arm movement
        controlArm();

        //Handle wrist movement

        controlWrist();

        // Handle claw operation
        controlClaw();

        // Handle intake system
        controlIntake();

        // Update telemetry data
        telemetry.update();
    }

    /**
     * Controls the robot's movement using mecanum drive calculations
     */
    private void driveMecanum() {
        // Retrieve joystick values
        double y = -gamepad1.left_stick_y; // Forward/backward (inverted)
        double x = gamepad1.left_stick_x * 1.1; // Strafing (adjusted for imperfect strafing)
        double rotation = gamepad1.right_stick_x; // Rotation

        double slowDownFactor = 0.6; // Factor to slow down robot movements

        // Calculate power for each wheel
        double frontLeftPower = y + x + rotation;
        double backLeftPower = y - x + rotation;
        double frontRightPower = y - x - rotation;
        double backRightPower = y + x - rotation;

        // Apply power to the wheels
        frontLeftMotor.setPower(frontLeftPower * slowDownFactor);
        backLeftMotor.setPower(backLeftPower * slowDownFactor);
        frontRightMotor.setPower(frontRightPower * slowDownFactor);
        backRightMotor.setPower(backRightPower * slowDownFactor);

        // Telemetry for debugging
        telemetry.addData("Front Left Power", frontLeftPower);
        telemetry.addData("Back Left Power", backLeftPower);
        telemetry.addData("Front Right Power", frontRightPower);
        telemetry.addData("Back Right Power", backRightPower);
    }
}
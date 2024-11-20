package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Move Sample", group = "Autonomous")
public class MoveSamples extends LumenBaseLinearOpMode {

    @Override
    public void runOpMode() {
        initHardware();
        // Wait for the start button
        waitForStart();
        forward(600);
        left(2200);
        forward(2500);
        left(600);
        backward(3000);
        forward(3400);
        left(600);
        backward(3400);
        forward(600);
        stop();
    }

    public void left(int duration) {
        frontLeftMotor.setPower(-1);
        frontRightMotor.setPower(-1);
        backLeftMotor.setPower(1);
        backRightMotor.setPower(1);
        sleep(duration);
    }

    public void right(int duration) {
        frontLeftMotor.setPower(1);
        frontRightMotor.setPower(1);
        backLeftMotor.setPower(-1);
        backRightMotor.setPower(-1);
        sleep(duration);
    }

    public void forward(int duration) {
        frontLeftMotor.setPower(-1);
        frontRightMotor.setPower(1);
        backLeftMotor.setPower(-1);
        backRightMotor.setPower(1);
        sleep(duration);
    }

    public void backward(int duration) {
        frontLeftMotor.setPower(1);
        frontRightMotor.setPower(-1);
        backLeftMotor.setPower(1);
        backRightMotor.setPower(-1);
        sleep(duration);
    }
}

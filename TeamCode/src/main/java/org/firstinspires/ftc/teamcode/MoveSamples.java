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
}

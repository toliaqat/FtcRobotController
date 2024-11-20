package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public abstract class LumenBaseOpMode extends OpMode {
    protected static final double CLAW_OPEN_POSITION = 0.55;
    protected static final double CLAW_CLOSED_POSITION = 0.7;

    // Drive motors
    protected DcMotor frontLeftMotor;
    protected DcMotor frontRightMotor;
    protected DcMotor backLeftMotor;
    protected DcMotor backRightMotor;

    // Arm motor
    protected DcMotor armMotor;

    // Wrist moto
    protected DcMotor wristMotor;

    // Servos
    protected Servo clawServo;
    protected Servo intakeServo;

    // IMU
    protected IMU imu;

    protected double previousArmPower = 0.0d;
    protected static final double ARM_POWER_STEP = 0.1;

    @Override
    public void init() {
        // Initialize hardware components
        initHardware();

        // Telemetry for debugging
        telemetry.addData("Status", "Initialized");
    }

    /**
     * Initialize all hardware components
     */
    private void initHardware() {
        // Initialize drive motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRight");

        // Initialize arm motor
        armMotor = hardwareMap.get(DcMotor.class, "arm");

        wristMotor = hardwareMap.get(DcMotor.class, "wrist");

        // Initialize servos
        clawServo = hardwareMap.get(Servo.class, "claw");
        intakeServo = hardwareMap.get(Servo.class, "intake");

        // Set motor directions (may need adjustment based on wiring)
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        stopAllMotors();

        // Set motors to run without encoders
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set initial servo positions
        clawServo.setPosition(CLAW_CLOSED_POSITION);   // Neutral position
        intakeServo.setPosition(0.5); // Neutral position
        //Set zero power behavior
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Init IMU
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }

    /**
     * Stop all motors
     */
    private void stopAllMotors() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        armMotor.setPower(0);
        wristMotor.setPower(0);
    }

    /**
     * Set motor modes for all drive motors
     */
    private void setMotorModes(DcMotor.RunMode mode) {
        frontLeftMotor.setMode(mode);
        backLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        backRightMotor.setMode(mode);
        armMotor.setMode(mode);
        wristMotor.setMode(mode);
    }

    /**
     * Controls the arm movement using the triggers
     */
    protected void controlArm() {
        // Calculate arm power from triggers
        double armPower = 0.0d;
        //ARM UP
        if (gamepad1.right_trigger > 0) {
            armPower = Math.max(previousArmPower + ARM_POWER_STEP, 1.0);
        } else if (gamepad1.left_trigger > 0) {
            armPower = Math.min(previousArmPower - ARM_POWER_STEP, 0.0);
        }
        previousArmPower = armPower;
        // Apply power to the arm motor
        armMotor.setPower(armPower);
        // Telemetry for debugging
        telemetry.addData("Arm Power", armPower);
    }

    /**
     * Controls the arm movement using the triggers
     */
    protected void controlWrist() {
        double wristPower = 0;

        // Control wrist motor with left bumper and right bumper
        if (gamepad1.dpad_up) {
            wristPower = 1.0; // Move wrist up
        } else if (gamepad1.dpad_down) {
            wristPower = -1.0; // Move wrist down
        }

        wristMotor.setPower(wristPower);
        telemetry.addData("Wrist Power", wristPower);
    }

    /**
     * Controls the claw using buttons
     */
    protected void controlClaw() {
        if (gamepad1.a) {
            // Open claw
            clawServo.setPosition(CLAW_OPEN_POSITION);
            telemetry.addData("Claw", "Opened");
        } else if (gamepad1.b) {
            // Close claw
            clawServo.setPosition(CLAW_CLOSED_POSITION);
            telemetry.addData("Claw", "Closed");
        }
    }

    /**
     * Controls the intake system using buttons
     */
    protected void controlIntake() {
        if (gamepad1.x) {
            // Activate intake system
            intakeServo.setPosition(1.0);
            telemetry.addData("Intake System", "Activated");
        } else if (gamepad1.y) {
            // Deactivate intake system
            intakeServo.setPosition(0.0);
            telemetry.addData("Intake System", "Deactivated");
        } else {
            intakeServo.setPosition(0.5);
            telemetry.addData("Intake System", "Stop");
        }
    }
}

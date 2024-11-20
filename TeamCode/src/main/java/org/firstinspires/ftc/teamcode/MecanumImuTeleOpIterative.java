package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "ImuTeleOp", group = "Imu Iterative Opmode")
public class MecanumImuTeleOpIterative extends OpMode {

    private static final double CLAW_OPEN_POSITION = 0.55;
    private static final double CLAW_CLOSED_POSITION = 0.7;

    // Drive motors
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    // Arm motor
    private DcMotor armMotor;

    // Wrist moto
    private DcMotor wristMotor;

    // Servos
    private Servo clawServo;
    private Servo intakeServo;

    // IMU
    protected IMU imu;

    @Override
    public void init() {
        // Initialize hardware components
        initHardware();

        // Telemetry for debugging
        telemetry.addData("Status", "Initialized");
    }

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
     * Controls the robot's movement using mecanum drive calculations
     */
    private void driveMecanum() {
        // Retrieve joystick values
        double y = -gamepad1.left_stick_y; // Forward/backward (inverted)
        double x = gamepad1.left_stick_x * 1.1; // Strafing (adjusted for imperfect strafing)
        double rx = gamepad1.right_stick_x; // Rotation

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        // Telemetry for debugging
        telemetry.addData("Front Left Power", frontLeftPower);
        telemetry.addData("Back Left Power", backLeftPower);
        telemetry.addData("Front Right Power", frontRightPower);
        telemetry.addData("Back Right Power", backRightPower);
    }

    /**
     * Controls the arm movement using the triggers
     */
    private void controlArm() {
        // Calculate arm power from triggers
        double armUp = gamepad1.right_trigger;
        double armDown = gamepad1.left_trigger;
        double armPower = armUp - armDown;

        // Apply power to the arm motor
        armMotor.setPower(armPower);

        // Telemetry for debugging
        telemetry.addData("Arm Power", armPower);
    }

    /**
     * Controls the arm movement using the triggers
     */
    private void controlWrist() {
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
    private void controlClaw() {
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
    private void controlIntake() {
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
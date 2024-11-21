package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Mecanum Robot TeleOp 5", group = "Iterative Opmode")
public class AleenaManaal extends OpMode {

    // Drive motors
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    // Arm motor
    private DcMotorEx armMotor;

    // Wrist moto
    private DcMotor wristMotor;

    // Servos
    private Servo clawServo;
    private Servo intakeServo;

    private double kP = 0.05;  // Proportional gain
    private double kI = 0.001; // Integral gain
    private double kD = 0.02; // Derivative gain
    private double kF = 0.0;  // Feedforward (optional)
    private double errorThreshold = 10; // Define a threshold in encoder ticks where we stop moving if we're within this range

    private double targetArmPosition = 0;  // The target position for the arm (in encoder ticks or other units)
    private double integral = 0;
    private double previousError = 0;
    private double previousTime = 0;  // Used to calculate deltaTime for derivative

    protected double previousArmPower = 0.0d;
    protected static final double ARM_POWER_STEP = 0.1;

    private static final double ARM_MAX_POWER = 1.0;  // Maximum motor power
    private static final double ARM_MIN_POWER = -1.0; // Minimum motor power
    private static final double ARM_POWER_INCREMENT = 0.01; // Step size for ramping power
    private double armCurrentPower = 0.0; // Current motor power


    private static final double WRIST_MAX_POWER = 1.0;  // Maximum motor power
    private static final double WRIST_MIN_POWER = -1.0; // Minimum motor power
    private static final double WRIST_POWER_INCREMENT = 0.01; // Step size for ramping power
    private double wristCurrentPower = 0.0; // Current motor power

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
        // controlArm();

        //Handle wrist movement

        // controlWrist();

        // Automation Arm / Wrist
        controlArmWrist();

        // Handle claw operation
        controlClaw();

        // Handle intake system
        controlIntake();

        // Update telemetry data
        //telemetry.update();
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
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wristMotor = hardwareMap.get(DcMotor.class, "wrist");
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set initial servo positions
        double clawPosition = 0.4;
        clawServo.setPosition(clawPosition);   // Neutral position
        telemetry.addData("initial clawPosition:", clawPosition);
        intakeServo.setPosition(0.5); // Neutral position
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

    /**
     * Automate the arm and write movement to preset locations
     */
    protected void controlArmWrist() {
        telemetry.addData("arm motor position", armMotor.getCurrentPosition());
        telemetry.addData("wrist motor position", wristMotor.getCurrentPosition());
        armMotor.setTargetPosition(-3363);
        // wristMotor.setTargetPosition(330);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        PIDFCoefficients newPIDF = new PIDFCoefficients(15.0, 0.0, 5.0, 10.0); // Example updated values
        armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, newPIDF);

        armMotor.setVelocity(-100);
        //armMotor.setPower(-1.0);

        PIDFCoefficients currentPIDF = armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

        //wristMotor.setPower(0.01);

        while (armMotor.isBusy()) {
            telemetry.addData("arm motor position", armMotor.getCurrentPosition());
            telemetry.addData("wrist motor position", wristMotor.getCurrentPosition());
            telemetry.addData("Current PIDF", "P: %.2f, I: %.2f, D: %.2f, F: %.2f",
                    currentPIDF.p, currentPIDF.i, currentPIDF.d, currentPIDF.f);
            telemetry.update();
        }

        armMotor.setVelocity(0);
        //armMotor.setPower(0);
        // wristMotor.setPower(0);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Exiting controlArmWrist ######", "Test");
    }

    /**
     * Controls the arm movement using the triggers
     */
    protected void controlArm() {
        // Calculate arm power from triggers
        //ARM UP
        if (gamepad1.right_trigger > 0) {
            // armPower = Math.min(previousArmPower + ARM_POWER_STEP, 1.0);
            if (armCurrentPower < ARM_MAX_POWER) {
                armCurrentPower += ARM_POWER_INCREMENT;
            }
        } else if (gamepad1.left_trigger > 0) {
            if (armCurrentPower > ARM_MIN_POWER) {
                armCurrentPower -= ARM_POWER_INCREMENT;
            }
            //armPower = Math.max(previousArmPower - ARM_POWER_STEP, 0.0);
        } else {
            armCurrentPower = 0;
        }

        // Apply power to the arm motor
        armMotor.setPower(armCurrentPower);
        // Telemetry for debugging
        telemetry.addData("Arm Power", armCurrentPower);
    }

    /**
     * Controls the arm movement using the triggers
     */
    private void controlWrist() {

        // Control wrist motor with left bumper and right bumper
        if (gamepad1.right_bumper) {

            if (wristCurrentPower < WRIST_MAX_POWER) {
                wristCurrentPower += WRIST_POWER_INCREMENT;
            }
        } else if (gamepad1.left_bumper) {
            if (wristCurrentPower > WRIST_MIN_POWER) {
                wristCurrentPower -= WRIST_POWER_INCREMENT;
            }
        } else {
            wristCurrentPower = 0;
        }

        wristMotor.setPower(wristCurrentPower);
        telemetry.addData("Wrist Power", wristCurrentPower);
    }

    /**
     * Controls the claw using buttons
     */
    private void controlClaw() {
        double clawPosition = 0;
        if (gamepad1.b) {
            // Close claw
            clawPosition = 0.42;
            clawServo.setPosition(clawPosition);
            telemetry.addData("Claw", "Close");
            telemetry.addData("close clawPosition:", clawPosition);
        } else if (gamepad1.a) {
            // Open claw
            clawPosition = 0.30;
            clawServo.setPosition(clawPosition);
            telemetry.addData("open clawPosition", clawPosition);
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Mecanum Robot TeleOp 5 Linear", group = "Linear Opmode")
public class AleenaManaal extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    // Arm motor
    private DcMotorEx armMotor;

    // Wrist motor
    private DcMotorEx wristMotor;

    // Servos
    private Servo clawServo;
    private Servo intakeServo;

    // IMU declaration
    private BNO055IMU imu;

    private static final double ARM_MAX_POWER = 1.0;  // Maximum motor power
    private static final double ARM_MIN_POWER = -1.0; // Minimum motor power
    private static final double ARM_POWER_INCREMENT = 0.01; // Step size for ramping power
    private double armCurrentPower = 0.0; // Current motor power


    private static final double WRIST_MAX_POWER = 1.0;  // Maximum motor power
    private static final double WRIST_MIN_POWER = -1.0; // Minimum motor power
    private static final double WRIST_POWER_INCREMENT = 0.01; // Step size for ramping power
    private double wristCurrentPower = 0.0; // Current motor power

    private static final int ARM_DEFAULT_POSITION = -165;
    private static final int ARM_POSITION_SPECIMEN_UP = -1460;
    private static final int ARM_POSITION_SPECIMEN_DOWN = -900;
    private static final int ARM_POSITION_LOWER_BASKET = -1463;
    private static final int ARM_POSITION_UPPER_BASKET = -2303;
    private static final int ARM_POSITION_SUBMERSIBLE_1 = -500;
    private static final int ARM_POSITION_SUBMERSIBLE_2 = -400;
    private static final int WRIST_POSITION_UPPER_BASKET = 350;
    private static final int WRIST_POSITION_LOWER_BASKET = 220;
    private static final int WRIST_POSITION_SUBMERSIBLE_1 = 80;
    private static final int WRIST_POSITION_SUBMERSIBLE_2 = 200;

    @Override
    public void runOpMode() {
        // Initialize hardware components
        initHardware();

        // Telemetry for debugging
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Set arm to default rest position
        setArmToDefaultRestPosition();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Handle driving
            driveMecanum();

            // Handle arm movement
            controlArm();

            //Handle wrist movement
            controlWrist();

            // Automation Arm / Wrist
            controlArmWristAutomations();

            // Handle claw operation
            controlClaw();

            // Handle intake system
            controlIntake();

            // Update telemetry data
            telemetry.update();
        }
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

        wristMotor = hardwareMap.get(DcMotorEx.class, "wrist");
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

        // Initialize the IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Set IMU parameters
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS; // Use radians for calculations
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC; // Acceleration unit
        imuParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // Calibration file
        imuParameters.loggingEnabled = false; // Disable logging
        imu.initialize(imuParameters);

        // Wait for the IMU to finish calibrating (optional but recommended)
        while (!imu.isGyroCalibrated() && opModeIsActive()) {
            telemetry.addData("IMU", "Calibrating...");
            telemetry.update();
            sleep(50); // Small delay to prevent busy looping
        }

        telemetry.addData("IMU", "Calibrated");
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
    }

    /**
     * Controls the robot's movement using mecanum drive calculations
     */
    private void driveMecanum() {
        // Retrieve joystick values
        double y = -gamepad1.left_stick_y; // Forward/backward (inverted)
        double x = gamepad1.left_stick_x * 1.1; // Strafing (adjusted for imperfect strafing)
        double rotation = -gamepad1.right_stick_x; // Rotation

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

    // Function to control mecanum drive in Field Orientation mode
//    private void driveMecanum() {
//        // Retrieve joystick inputs (e.g., from gamepad)
//        double xInput = -gamepad1.left_stick_x; // Strafing (X-axis)
//        double yInput = gamepad1.left_stick_y;  // Forward/backward (Y-axis)
//        double rotation = gamepad1.right_stick_x; // Rotation (Z-axis)
//
//        // Get the robot's current orientation from the IMU
//        double robotAngle = imu.getAngularOrientation().firstAngle;
//
//        // Convert joystick inputs to field-oriented inputs
//        double cosAngle = Math.cos(robotAngle);
//        double sinAngle = Math.sin(robotAngle);
//
//        double fieldX = xInput * cosAngle - yInput * sinAngle;
//        double fieldY = xInput * sinAngle + yInput * cosAngle;
//
//        // Calculate motor powers for mecanum drive
//        double frontLeftPower = fieldY + fieldX + rotation;
//        double frontRightPower = fieldY - fieldX - rotation;
//        double backLeftPower = fieldY - fieldX + rotation;
//        double backRightPower = fieldY + fieldX - rotation;
//
//        // Normalize the motor powers to ensure they are within [-1, 1]
//        double maxPower = Math.max(1.0, Math.abs(frontLeftPower));
//        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
//        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
//        maxPower = Math.max(maxPower, Math.abs(backRightPower));
//
//        frontLeftPower /= maxPower;
//        frontRightPower /= maxPower;
//        backLeftPower /= maxPower;
//        backRightPower /= maxPower;
//
//        // Set motor powers
//        frontLeftMotor.setPower(frontLeftPower);
//        frontRightMotor.setPower(frontRightPower);
//        backLeftMotor.setPower(backLeftPower);
//        backRightMotor.setPower(backRightPower);
//    }

    protected void controlArmWristAutomations() {
        if (gamepad1.dpad_up) {
            // Reach to upper basket
            moveAmrWrist(ARM_POSITION_UPPER_BASKET, WRIST_POSITION_UPPER_BASKET);
        } else if (gamepad1.dpad_down){
            // Reach to lower basket
            moveAmrWrist(ARM_POSITION_LOWER_BASKET, WRIST_POSITION_LOWER_BASKET);
        } else if (gamepad1.dpad_right) {
            // Reach inside submersible
            moveAmrWrist(0, WRIST_POSITION_SUBMERSIBLE_1);
            moveAmrWrist(ARM_POSITION_SUBMERSIBLE_1, WRIST_POSITION_SUBMERSIBLE_2);
            moveAmrWrist(ARM_POSITION_SUBMERSIBLE_2, WRIST_POSITION_SUBMERSIBLE_2);
        } else if (gamepad1.back) {
            moveAmrWrist(ARM_POSITION_SPECIMEN_UP, 0);
        } else if (gamepad1.start) {
            moveAmrWrist(ARM_POSITION_SPECIMEN_DOWN, 0);
            controlClawHelper(true);
        } else if (gamepad1.dpad_left) {
            // Reach to base position
            moveAmrWrist(0, 0);
        }
    }

    protected void setArmToDefaultRestPosition() {
        armMotor.setTargetPosition(ARM_DEFAULT_POSITION);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setVelocity(-5000);
        while (opModeIsActive() && armMotor.isBusy()) {
            telemetry.addData("arm motor position", armMotor.getCurrentPosition());
            telemetry.addData("wrist motor position", wristMotor.getCurrentPosition());
            telemetry.update();
        }
        armMotor.setVelocity(0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setVelocity(-5000);
        while (opModeIsActive() && armMotor.isBusy()) {
            telemetry.addData("arm motor position", armMotor.getCurrentPosition());
            telemetry.addData("wrist motor position", wristMotor.getCurrentPosition());
            telemetry.update();
        }
        armMotor.setVelocity(0);
        controlClawHelper(true);
        telemetry.update();
    }

    /**
     * Position upper basket
     */
    protected void moveAmrWrist(int armPosition, int wristPosition) {
        telemetry.addData("arm motor position", armMotor.getCurrentPosition());
        telemetry.addData("wrist motor position", wristMotor.getCurrentPosition());
        armMotor.setTargetPosition(armPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setVelocity(-5000);

        //wristMotor.setTargetPosition(330);
        wristMotor.setTargetPosition(wristPosition);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wristMotor.setVelocity(500);

        while (opModeIsActive() && armMotor.isBusy()) {
            telemetry.addData("arm motor position", armMotor.getCurrentPosition());
            telemetry.addData("wrist motor position", wristMotor.getCurrentPosition());
            telemetry.update();
        }

        while (opModeIsActive() && wristMotor.isBusy()) {
            telemetry.addData("arm motor position", armMotor.getCurrentPosition());
            telemetry.addData("wrist motor position", wristMotor.getCurrentPosition());
            telemetry.update();
        }

        // armMotor.setVelocity(0);
        // wristMotor.setVelocity(0);

//        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Exiting", "controlArmWrist");
        telemetry.update();
    }

    /**
     * Controls the arm movement using the triggers
     */
    protected void controlArm() {
        // Calculate arm power from triggers
        //ARM UP
        if (gamepad1.right_trigger > 0) {
            if (armCurrentPower < ARM_MAX_POWER) {
                armCurrentPower += ARM_POWER_INCREMENT;
            }
        } else if (gamepad1.left_trigger > 0) {
            if (armCurrentPower > ARM_MIN_POWER) {
                armCurrentPower -= ARM_POWER_INCREMENT;
            }
        } else {
            armCurrentPower = 0;
        }

        // Apply power to the arm motor
        armMotor.setPower(armCurrentPower);
        // Telemetry for debugging
        telemetry.addData("Arm Power", armCurrentPower);
        telemetry.update();
    }

    /**
     * Controls the wrist movement using the bumpers
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
        telemetry.update();
    }

    /**
     * Controls the claw using buttons
     */
    private void controlClaw() {
        double clawPosition = 0;
        if (gamepad1.b) {
            // close claw
            controlClawHelper(false);
        } else if (gamepad1.a) {
            // open claw
            controlClawHelper(true);
        }
        telemetry.update();
    }

    private void controlClawHelper(boolean open) {
        double clawPosition = 0;
        if (!open) {
            // Close claw
            clawPosition = 0.42;
            clawServo.setPosition(clawPosition);
            telemetry.addData("Claw", "Close");
            telemetry.addData("close clawPosition:", clawPosition);
        } else {
            // Open claw
            clawPosition = 0.30;
            clawServo.setPosition(clawPosition);
            telemetry.addData("open clawPosition", clawPosition);
        }
        telemetry.update();
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
        telemetry.update();
    }
}

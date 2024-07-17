package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//TODO: Add stick dead zone

@TeleOp()
public class GNRTCh1 extends OpMode {

    DcMotor doorMotor; // Hex motor for controlling the door mechanism

    private final int ENCODER_COUNT_PER_REVOLUTION = 288; // Encoder counts per revolution for door motor

    DcMotor leftMotor; // Motor for left side drive
    DcMotor rightMotor; // Motor for right side drive

    @Override
    public void init() {
        // Initialize left and right motors from the hardware map
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE); // Reverse right motor direction for proper movement

        // Initialize door motor from the hardware map
        doorMotor = hardwareMap.get(DcMotor.class, "door_motor");

        // Reset encoder for door motor to start from a known position
        doorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        // Retrieve joystick values for driving control
        double leftStickX = -gamepad1.left_stick_x; // Negative for reversing direction
        double leftStickY = gamepad1.left_stick_y;

        // Limit motor power to a range between -0.7 and 0.7 for smoother control
        double motorPowerY = ueInRangeOf(-0.7, 0.7, leftStickY);
        double motorPowerX = keepValueInRangeOf(-0.7, 0.7, leftStickX);

        // Set motor power for left and right motors to achieve desired movement
        leftMotor.setPower(motorPowerY - motorPowerX);
        rightMotor.setPower(motorPowerY + motorPowerX);

        // Control door motor based on gamepad button inputs
        if (gamepad1.a) {
            // Move door motor to half a revolution position
            doorMotor.setTargetPosition(ENCODER_COUNT_PER_REVOLUTION / 2);
            doorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (gamepad1.b) {
            // Reset door motor to the initial position
            doorMotor.setTargetPosition(0);
            doorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            doorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // Allow manual control of door motor using right joystick if left bumper is pressed
        if (gamepad1.left_bumper) {
            doorMotor.setPower(gamepad1.right_stick_y);
        }
    }

    // Method to constrain an input value within a specified range
    public double keepValueInRangeOf(double min, double max, double input) {
        if (input > max) {
            return max;
        } else {
            return Math.max(input, min);
        }
    }
}

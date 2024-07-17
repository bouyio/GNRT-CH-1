package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp()
public class GNRTCh1P3 extends OpMode {

    private double accelerationMultiplier;
    private double timeDiff;
    private boolean isTimeDiffSet = false;
    private boolean isStartingAccelerationTimeSet = false;
    private double startingAccelerationTime;

    boolean intakeToggle = false;
    DcMotor intakeMotor; // Hex motor for controlling the door mechanism

    // TODO: Measure the optimal dead zone
    private final double ANALOG_STICK_DEAD_ZONE = 0.15;

    DcMotor leftMotor; // Motor for left side drive
    DcMotor rightMotor; // Motor for right side drive

    @Override
    public void init() {
        // Initialize left and right motors from the hardware map
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE); // Reverse right motor direction for proper movement

        // Initialize door motor from the hardware map
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");

    }

    @Override
    public void loop() {

        // Retrieve joystick values for driving control
        double leftStickX = -gamepad1.left_stick_x; // Negative for reversing direction
        double leftStickY = -gamepad1.left_stick_y;

        // Keep the joystick values only if they are greater the previously set dead zone
        leftStickX = Math.abs(leftStickX) > ANALOG_STICK_DEAD_ZONE ? leftStickX : 0;
        leftStickY = Math.abs(leftStickY) > ANALOG_STICK_DEAD_ZONE ? leftStickY : 0;

        // Limit motor power to a range between -0.7 and 0.7 for smoother control
        double motorPowerY = keepValueInRangeOf(-0.7, 0.7, leftStickY);
        double motorPowerX = keepValueInRangeOf(-0.7, 0.7, leftStickX);

        if(gamepad1.left_bumper) {
            leftMotor.setPower(motorPowerY - motorPowerX);
            rightMotor.setPower(motorPowerY + motorPowerX);
        } else {
            accelerate(motorPowerY, motorPowerX);
        }
        if (gamepad1.a){
            intakeToggle = !intakeToggle;
        }

        if(intakeToggle){
            intakeMotor.setPower(-1);
        } else {
            intakeMotor.setPower(0);
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


    private void accelerate(double LStickY, double LStickX){
        if(!isStartingAccelerationTimeSet){
            startingAccelerationTime = (double) ElapsedTime.SECOND_IN_NANO / 1000000000;
            isStartingAccelerationTimeSet = true;
        }

        if(!isTimeDiffSet){
            timeDiff = (double) ElapsedTime.MILLIS_IN_NANO / 1000000000 - startingAccelerationTime;
            if(timeDiff >= 2){
                isTimeDiffSet = true;
            }
        }


        accelerationMultiplier = (double) timeDiff / 3;

        if(LStickY == 0){
            rightMotor.setPower(0.00);
            leftMotor.setPower(0.00);
            accelerationMultiplier = 0;
            isStartingAccelerationTimeSet = false;
            isTimeDiffSet = false;

            return;
        }

        rightMotor.setPower((LStickY * accelerationMultiplier) + LStickX);
        leftMotor.setPower((LStickY * accelerationMultiplier) - LStickX);
    }
}
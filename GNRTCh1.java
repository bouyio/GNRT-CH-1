package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//TODO: Add stick dead zone

public class GNRTCh1 extends OpMode {

    DcMotor doorMotor;

    //private final double STICK_DEADZONE = 0.01;

    private final int ENCODER_COUNT_PER_REVOLUTION = 288;

    DcMotor leftMotor;

    DcMotor rightMotor;

    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        doorMotor = hardwareMap.get(DcMotor.class, "door_motor");

        doorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        double leftStickX = -gamepad1.left_stick_x;
        double leftStickY = gamepad1.left_stick_y;

        double motorPowerY = keepValueInRangeOf(-0.7, 0.7, leftStickY);
        double motorPowerX = keepValueInRangeOf(-0.7, 0.7, leftStickX);

        leftMotor.setPower(motorPowerY - motorPowerX);
        rightMotor.setPower(motorPowerY + motorPowerX);

        if (gamepad1.a){
            doorMotor.setTargetPosition(ENCODER_COUNT_PER_REVOLUTION / 2);

            doorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if(gamepad1.b){
            doorMotor.setTargetPosition(0);

            doorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            doorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (gamepad1.left_bumper){
            doorMotor.setPower(gamepad1.right_stick_y);
        }

    }

    public double keepValueInRangeOf(double min, double max, double input){
        if (input > max){
            return max;
        } else return Math.max(input, min);
    }
}

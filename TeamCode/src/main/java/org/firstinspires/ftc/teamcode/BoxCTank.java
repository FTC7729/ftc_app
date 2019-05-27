package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Locale;

@TeleOp
public class BoxCTank extends OpMode {
    //
    // Wheel motors
    //
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;

    public void init() {
        //
        // Map motors to phone definitions
        //
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");
    }

    public void loop() {
        float left;
        float right;

        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;

        //
        // Simple tank mode
        //
        leftFront.setPower(left);
        leftRear.setPower(left);
        rightFront.setPower(-right);
        rightRear.setPower(-right);
    }
}
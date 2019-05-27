package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

import static java.lang.Thread.sleep;

@TeleOp
public class BoxCHolonomic extends OpMode {

    // Motor objects
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;

    // The IMU sensor object
    BNO055IMU imu;

    public void init() {
        //
        // Configure IMU options
        //
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode           = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit      = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit      = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        //
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        //
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        //
        // Map motors to the names defined on the phone
        //
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");

        //
        // Reverse the motors on the right side
        //
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
    }


    public void loop() {
        double relAngle;
        double dirStickX;
        double dirStickY;
        double dirRot;
        double theta;
        double thetaRot;
        double powX;
        double powY;
        double powRot;
        //
        Orientation angles;

        //
        // Get the current relative heading of the robot
        //
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        relAngle = (double) angles.firstAngle;
        telemetry.addData("Angle: ", relAngle);

        //
        // Get the control inputs for the robot
        // Gamepad 1, left  stick x: Control rotation
        // Gamepad 1, right stick x: Control left/right motion
        // Gamepad 1, right stick y: Control forward reverse motion
        //
        dirRot    = gamepad1.left_stick_x;
        dirStickX = gamepad1.right_stick_x;
        dirStickY = gamepad1.right_stick_y;

        // Compute rotational power based on joystick input
        powRot = dirRot;

        //
        // Modify X & Y inputs to account for the power required for rotation
        //
        dirStickX = dirStickX * Math.sqrt(Math.pow(1 - Math.abs(powRot), 2) / 2);
        dirStickY = dirStickY * Math.sqrt(Math.pow(1 - Math.abs(powRot), 2) / 2);

        //
        // Compute the angle of the right joystick
        //
        theta    = (Math.atan2(dirStickY, dirStickX));
        // Add the current relative angle reported by the IMU
        thetaRot = theta + relAngle;
        telemetry.addData("Theta: ", theta);
        telemetry.addData("Rot: ", thetaRot);

        //
        // Calculate wheel power
        //
        powX = Math.sqrt(Math.pow(dirStickX, 2) + Math.pow(dirStickY, 2)) * (Math.sin(thetaRot + Math.PI / 4));
        powY = Math.sqrt(Math.pow(dirStickX, 2) + Math.pow(dirStickY, 2)) * (Math.sin(thetaRot - Math.PI / 4));

        //
        // Drive wheels
        //
        leftFront.setPower(powY - powRot);
        leftRear.setPower(powX - powRot);
        rightRear.setPower(powY + powRot);
        rightFront.setPower(powX + powRot);

        //
        // Telemetry
        //
        telemetry.update();

    }

 }
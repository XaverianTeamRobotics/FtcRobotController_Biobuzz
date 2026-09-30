package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Math.abs;
import static java.lang.Math.max;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Mecanum Bot", group="debug")
public class mecanumBot extends OpMode {
    DcMotor FL, BL, FR, BR;
    public double powerScale = 1.0;
    public double rotScale = 1.0;
    public double y, x, r, denominator;

    @Override
    public void init() {
        FL = hardwareMap.get(DcMotor.class, "motor0");
        BL = hardwareMap.get(DcMotor.class, "motor1");
        FR = hardwareMap.get(DcMotor.class, "motor2");
        BR = hardwareMap.get(DcMotor.class, "motor3");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {
        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        r = gamepad1.right_stick_x * rotScale;

        denominator = max(abs(y) + abs(x) + abs(r), 1.0);

        FL.setPower((y + x + r) / denominator * powerScale);
        BL.setPower((y - x + r) / denominator * powerScale);
        FR.setPower(-1*(y - x - r) / denominator * powerScale);
        BR.setPower(-1*(y + x - r) / denominator * powerScale);

    }
}

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotPart;

@TeleOp(name = "Arm Test - show ticks")
public class ArmTestOpMode extends OpMode {
    DcMotor arm1Motor = null;

    @Override
    public void init() {
        arm1Motor = hardwareMap.dcMotor.get(RobotPart.ARM_1_MOTOR);
        arm1Motor.setDirection(DcMotor.Direction.REVERSE);
        arm1Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addLine("Use gamepad1.x and gamepad1.y to lower or raise latch arm");
    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            arm1Motor.setPower(-0.5);

        } else if (gamepad1.y) {
            arm1Motor.setPower(0.5);

        } else {
            arm1Motor.setPower(0);
        }


        int currentPosition = arm1Motor.getCurrentPosition();
        telemetry.addData("current position", currentPosition);

    }


}

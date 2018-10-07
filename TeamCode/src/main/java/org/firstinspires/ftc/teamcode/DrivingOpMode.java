package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="DriveOpMode1")
public class DrivingOpMode extends OpMode {
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;

    @Override
    public void init() {
         leftMotor = hardwareMap.dcMotor.get("leftMotor");
         leftMotor.setDirection(DcMotor.Direction.FORWARD);
         telemetry.addLine("Left Motor ready");
         //Because the outputs of the controller are inverse of what they should be

         rightMotor = hardwareMap.dcMotor.get("rightMotor");
         rightMotor.setDirection(DcMotor.Direction.REVERSE);
         telemetry.addLine("Right Motor ready");
        //Because the outputs of the controller are inverse of what they should be
        //They are different because when you flip a motor it would spin backwards to drive forwards
        telemetry.addLine("hit the button already");
    }
    @Override
    public void loop() {
        rightMotor.setPower(getPower(gamepad1.right_stick_y));
        leftMotor.setPower(getPower(gamepad1.left_stick_y));

        telemetry.clearAll();
        telemetry.addLine(("leftmotor power="+gamepad1.left_stick_y));
        telemetry.addLine(("rightmotor power="+gamepad1.right_stick_y));

    }

    private float getPower(float stickPosition) {
        return Math.abs(stickPosition) * stickPosition;
    }
}

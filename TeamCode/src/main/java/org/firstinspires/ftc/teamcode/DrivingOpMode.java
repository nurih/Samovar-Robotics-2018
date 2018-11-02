package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Driving OpMode")
public class DrivingOpMode extends OpMode {
    public static final double LATCH_ARM_POWER = 0.5;
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor arm1Motor = null;

    DcMotor collectorArm = null;

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get(RobotPart.LEFT_MOTOR);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addLine("Left Motor ready");
        //Because the outputs of the controller are inverse of what they should be

        rightMotor = hardwareMap.dcMotor.get(RobotPart.RIGHT_MOTOR);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addLine("Right Motor ready");
        //Because the outputs of the controller are inverse of what they should be
        //They are different because when you flip a motor it would spin backwards to drive forwards
        arm1Motor = hardwareMap.dcMotor.get(RobotPart.ARM_1_MOTOR);
        arm1Motor.setDirection(DcMotor.Direction.FORWARD);
        arm1Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        collectorArm = hardwareMap.dcMotor.get(RobotPart.COLLECTOR_ARM_MOTOR);
        collectorArm.setDirection(DcMotor.Direction.FORWARD);
        collectorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addLine("hit the button already");
    }

    @Override
    public void loop() {
        rightMotor.setPower(DrivePowerCurve.valueSquared(gamepad1.right_stick_y));
        leftMotor.setPower(DrivePowerCurve.valueSquared(gamepad1.left_stick_y));

        telemetry.addLine(("leftmotor power=" + gamepad1.left_stick_y));
        telemetry.addLine(("rightmotor power=" + gamepad1.right_stick_y));

        operateLatchArm();

        operateCollectorArm();
    }

    private void operateLatchArm() {
        if (gamepad1.x) {
            arm1Motor.setPower(-LATCH_ARM_POWER);

        } else if (gamepad1.y) {
            arm1Motor.setPower(LATCH_ARM_POWER);

        } else {
            arm1Motor.setPower(0);
        }
        telemetry.addData("current position", arm1Motor.getCurrentPosition());
    }

    private void operateCollectorArm() {
        collectorArm.setPower(gamepad2.right_stick_y / 3.0);
    }

}

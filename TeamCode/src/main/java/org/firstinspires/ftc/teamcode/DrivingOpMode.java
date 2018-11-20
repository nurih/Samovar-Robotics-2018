package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Driving OpMode")
public class DrivingOpMode extends OpMode {
    public static final double LATCH_ARM_POWER = 0.5;
    public static final double COLLECTOR_ARM_POWER = 0.25;
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor arm1Motor = null;

    DcMotor collectorArm = null;
    private float drivingPowerFactor = (float) .5;


    CollectorExtenderServo collectorExtenderServo;

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

        collectorExtenderServo = new CollectorExtenderServo(hardwareMap,telemetry);
    }

    @Override
    public void loop() {
        rightMotor.setPower(getDrivePower(gamepad1.right_stick_y));
        leftMotor.setPower(getDrivePower(gamepad1.left_stick_y));

        telemetry.addLine(("leftmotor power=" + gamepad1.left_stick_y));
        telemetry.addLine(("rightmotor power=" + gamepad1.right_stick_y));


        changePower();

        operateLatchArm();

        catchMinerals();

        extendCollectorArm();
    }

    private void changePower() {
        if(gamepad1.x) {
        drivingPowerFactor = 0.75f;
        telemetry.addLine("FASTEST");
    } else if(gamepad1.y) {
        drivingPowerFactor = 0.5f;
        telemetry.addLine("FAST");
    } else if(gamepad1.b) {
        drivingPowerFactor = 0.25f;
        telemetry.addLine("SLOW");
    } else if(gamepad1.a) {
        drivingPowerFactor = 0.1f;
        telemetry.addLine("SLOWEST");
    }
    }

    private float getDrivePower(float stickPosition) {
        // tame total power so it's slower
        return DrivePowerCurve.linear(stickPosition * drivingPowerFactor);
    }

    private void operateLatchArm() {
        if (gamepad2.right_bumper) {
            arm1Motor.setPower(-LATCH_ARM_POWER);

        } else if (gamepad2.left_bumper) {
            arm1Motor.setPower(LATCH_ARM_POWER);

        } else {
            arm1Motor.setPower(0);
        }
        telemetry.addData("current position", arm1Motor.getCurrentPosition());
    }

    private void extendCollectorArm(){
        if(gamepad2.left_stick_y != 0){
            collectorExtenderServo.rotate(gamepad2.left_stick_y);
        }
        else {
            collectorExtenderServo.stop();
        }
    }
    private void catchMinerals() {
        collectorArm.setPower(DrivePowerCurve.valueSquared(gamepad2.right_stick_y) * COLLECTOR_ARM_POWER);
    }



}

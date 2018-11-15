package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@Autonomous(name = "autonamouse v1")
public class Autonomous2OpMode extends LinearOpMode{
    private static final double DRIVE_MOTOR_POWER = 1;
    private static final double ARM_MOTOR_POWER = 1;
    private static final int TURN_ANGLE_90_DEGREES = 90;
    private static final int TURN_ANGLE_45_DEGREES = 45;
    private static final double DRIVE_TO_WALL = 48;
    private static final double DRIVE_FROM_LATCH = 5;

    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor arm1Motor = null;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;


    @Override
    public void runOpMode() throws InterruptedException {
        while(opModeIsActive()) {
            telementary();
            double angleInDegrees = getDegrees(angles.firstAngle);
            if (angleInDegrees < 0){
                rightMotor.setPower(.1);
                leftMotor.setPower(-.1);
            }
            if(angleInDegrees > 0){
                rightMotor.setPower(-.1);
                leftMotor.setPower(.1);
            }
        }
    }

    double getDegrees(double angle){
       return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angle);
    }
    private void telementary() {
        telemetry.addLine()
        .addData("heading", new Func<String>() {
            @Override public String value() {
                return formatAngle(angles.angleUnit, angles.firstAngle);
            }
            });
    }
    private void initializeHardware() {
        leftMotor = initializeDriveMotor(DRIVE_MOTOR_POWER, RobotPart.LEFT_MOTOR, DcMotor.Direction.REVERSE);
        rightMotor = initializeDriveMotor(DRIVE_MOTOR_POWER, RobotPart.RIGHT_MOTOR, DcMotor.Direction.FORWARD);
    }


    private DcMotor initializeDriveMotor(double driveMotorPower, String motorName, DcMotorSimple.Direction direction) {
        DcMotor motor = hardwareMap.dcMotor.get(motorName);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(driveMotorPower);
        telemetry.addLine(motorName + " motor ready");
        return motor;
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}

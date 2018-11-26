package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@Autonomous(name = "Test IMU orientation")
@Disabled
public class TestIMUOpMode extends LinearOpMode {
    private static final double DRIVE_MOTOR_POWER = 1;


    DcMotor leftMotor = null;
    DcMotor rightMotor = null;


    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;


    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        telemetry.addLine("hardware initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double angleInDegrees = getDegrees(angles.firstAngle);
            telemetry.addData("AngleInDegrees: ",angleInDegrees);
            if (angleInDegrees < -1) {
                rightMotor.setPower(.2);
                leftMotor.setPower(-.2);
            }
            else if (angleInDegrees > 1) {
                rightMotor.setPower(-.2);
                leftMotor.setPower(.2);
            }
            else{
                rightMotor.setPower(0);
                leftMotor.setPower(0);
            }
            sendTelementary();
        }
    }

    double getDegrees(double angle) {
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angle);
    }

    private void sendTelementary() {
        telemetry.addData("heading", formatAngle(angles.angleUnit, angles.firstAngle));
        telemetry.update();
    }

    private void initializeHardware() {

        leftMotor = initializeDriveMotor(DRIVE_MOTOR_POWER, RobotPart.LEFT_MOTOR, DcMotor.Direction.REVERSE);
        rightMotor = initializeDriveMotor(DRIVE_MOTOR_POWER, RobotPart.RIGHT_MOTOR, DcMotor.Direction.FORWARD);
        imu = initializeIMU();
    }

    private BNO055IMU initializeIMU() {
        imu = hardwareMap.get(BNO055IMU.class, RobotPart.IMU);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        return imu;
    }


    private DcMotor initializeDriveMotor(double driveMotorPower, String motorName, DcMotorSimple.Direction direction) {
        DcMotor motor = hardwareMap.dcMotor.get(motorName);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
        telemetry.addLine(motorName + " motor ready");
        return motor;
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}

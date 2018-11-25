package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Autonomous 2 (Ben)")
public class Autonomous2OpMode extends LinearOpMode {
    private static final double DRIVE_MOTOR_POWER = 1;

    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor arm1Motor = null;


    private static final double ARM_MOTOR_POWER = 1;
    public static final int DETACH_FROM_LANDER_TICKS = 8700;

    IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();

        waitForStart();

        telemetry.addLine("Started");
        telemetry.update();

        turn(90);
        //waitForDriveDone();
        telemetry.addLine("turned!!!!!!!!!!");
        sleep(200);

        goStraight(10);
        waitForDriveDone();
    }

    private void turn(int TurnAmount) {
        boolean turned = false;

        double angleInDegrees = imu.getDegrees();

        while (angleInDegrees < TurnAmount) {
            angleInDegrees = imu.getDegrees();
            rightMotor.setPower(.4);
            leftMotor.setPower(-.4);
            telemetry.addLine("U NEVER LEFT THE MATRIX BROOO, THERE IS NO ESCAPE");
            telemetry.update();
        }
        telemetry.addLine("WOAH TRIPPY MAN YOU JUST DID A 4D DRIFT");
        telemetry.update();
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }


    private DcMotor initializeDriveMotor(double driveMotorPower, String motorName, DcMotorSimple.Direction direction) {
        DcMotor motor = hardwareMap.dcMotor.get(motorName);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motor.setPower(driveMotorPower);
        telemetry.addLine(motorName + " motor ready");
        return motor;
    }


    private DcMotor initializeArmMotor(double power) {
        DcMotor arm1Motor = hardwareMap.dcMotor.get(RobotPart.ARM_1_MOTOR);
        arm1Motor.setDirection(DcMotor.Direction.REVERSE);
        arm1Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(750);
        arm1Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1Motor.setPower(power);
        return arm1Motor;
    }

    private void initializeHardware() {


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = new IMU(hardwareMap, telemetry);

        arm1Motor = initializeArmMotor(ARM_MOTOR_POWER);
        leftMotor = initializeDriveMotor(DRIVE_MOTOR_POWER, RobotPart.LEFT_MOTOR, DcMotor.Direction.REVERSE);
        rightMotor = initializeDriveMotor(DRIVE_MOTOR_POWER, RobotPart.RIGHT_MOTOR, DcMotor.Direction.FORWARD);
    }

    private void waitForDriveDone() {
        while (leftMotor.isBusy() || rightMotor.isBusy()) {
            // spin wait.
        }
    }

    private void goStraight(double distanceInches) {
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setPower(.5);
        rightMotor.setPower(.5);
        int distanceTicks = inchToTicks(distanceInches);

        int currentPositionLeft = leftMotor.getCurrentPosition();
        int currentPositionRight = rightMotor.getCurrentPosition();
        telemetry.addData("PositionL", currentPositionLeft);
        telemetry.addData("PositionR", currentPositionRight);


        leftMotor.setTargetPosition(distanceTicks + currentPositionLeft);
        rightMotor.setTargetPosition(distanceTicks + currentPositionRight);

        telemetry.addData("PositionL", leftMotor.getCurrentPosition());
        telemetry.addData("PositionR", rightMotor.getCurrentPosition());

        telemetry.update();
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private static int inchToTicks(double inches) {
        return (int) (inches * (26.1));

    }

}

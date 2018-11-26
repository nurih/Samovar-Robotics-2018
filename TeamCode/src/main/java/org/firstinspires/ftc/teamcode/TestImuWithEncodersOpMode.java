package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Test IMU with encoders")
@Disabled
public class TestImuWithEncodersOpMode extends Teapot {


    ImuDrive imuDrive;


    @Override
    public void runOpMode() throws InterruptedException {
        imuDrive = new ImuDrive(this);

        telemetry.addLine("Hardware initialized");
        telemetry.update();
        waitForStart();

        sleep(2000);
        imuDrive.straight(-6);

        sleep(2000);
        double currentAngle = imuDrive.turn(45);
        telemetry.addData("Current angle is ", currentAngle);

        sleep(2000);
        imuDrive.straight(24);
        waitForDriveDone(imuDrive.leftMotor, imuDrive.rightMotor);

        telemetry.addLine("Done");
        telemetry.update();
    }

}


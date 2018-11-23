package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.DriveByCamera.TargetImage;


@Autonomous(name = "Drive To Images", group = "Test")
@Disabled
public class DriveToImageOpMode extends LinearOpMode {


    @Override
    public void runOpMode() {


        DcMotor rightMotor = hardwareMap.dcMotor.get(RobotPart.RIGHT_MOTOR);
        DcMotor leftMotor = hardwareMap.dcMotor.get(RobotPart.LEFT_MOTOR);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DriveByCamera driveByCamera = new DriveByCamera(rightMotor, leftMotor, telemetry, hardwareMap);


        telemetry.addData(">", "Press Play to start");

        telemetry.update();

        waitForStart();

        driveByCamera.setTargetImage( TargetImage.BLUE_ROVER);
        //Entering Loop
        while (opModeIsActive()) {
            int distanceGoal = -200;
            int translationGoal = -250;
            driveByCamera.driveToImage( distanceGoal, translationGoal);
        }
    }


}
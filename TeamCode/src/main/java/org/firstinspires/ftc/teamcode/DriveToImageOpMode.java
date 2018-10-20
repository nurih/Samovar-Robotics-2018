package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.DriveByCamera;
import org.firstinspires.ftc.teamcode.RobotPart;


@Autonomous(name = "Drive To Images", group = "Test")
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


        //Entering Loop
        while (opModeIsActive()) {

            int distanceGoal = -200;
            int translationGoal = -250;
            driveByCamera.driveToImage(TargetImage.BLUE_ROVER, distanceGoal, translationGoal);
        }
    }


}
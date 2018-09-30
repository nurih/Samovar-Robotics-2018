package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "armV1")
public class armV1 extends LinearOpMode {
    DcMotor arm1motor = null;

    @Override
    public void runOpMode() throws InterruptedException {

        arm1motor = hardwareMap.dcMotor.get("arm1motor");
        arm1motor.setDirection(DcMotor.Direction.FORWARD);
        arm1motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1motor.setPower(0.5);

        //arm1motor.setPower(0.5);

        telemetry.addLine("arm Motor ready");

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();

        telemetry.addLine("Started");
        telemetry.update();
        sleep(3000);

        //get current position
        int currentPosition = arm1motor.getCurrentPosition();

        //determine next position
        int wantedMovement = 1000;

        int determinedPosition = wantedMovement + currentPosition;

        telemetry.addData("Current is", currentPosition);
        telemetry.addData("Desired is", determinedPosition);
        telemetry.update();
        sleep(3000);

        //Move motor
        arm1motor.setTargetPosition(determinedPosition);
        while (arm1motor.isBusy()) {
            telemetry.addData("Position", arm1motor.getCurrentPosition());
            telemetry.update();
        }
        arm1motor.setPower(0);

        telemetry.addData("Position", arm1motor.getCurrentPosition());
        telemetry.addLine("Done");
        telemetry.update();

        sleep(3000);
    }
}

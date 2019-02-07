package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class Teapot extends LinearOpMode {

    public void waitForDriveDone(DcMotor m1, DcMotor m2) {
        while (opModeIsActive()) {
            telemetry.addData("Motor 1", m1.getCurrentPosition());
            telemetry.addData("Motor 2", m2.getCurrentPosition());
            telemetry.update();
            if (!m1.isBusy() && !m2.isBusy()) {
                break;
            }
        }
    }

    static int degreesToTicks(double degrees) {
        double circumference = 2 * Math.PI * 13.5;
        double arcLength = (circumference / 360) * degrees;
        return inchToTicks(arcLength);
    }

    public static int inchToTicks(double inches) {
        return (int) (inches * (1120 / 46));

    }

    public void Say(String prompt, Object value){
        telemetry.addData(prompt,value);
        telemetry.update();
    }

    public void Say(String something){
        telemetry.addLine(something);
        telemetry.update();
    }


}

package org.firstinspires.ftc.teamcode;

import android.support.annotation.NonNull;
import android.support.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

enum BotDirection {
    LEFT,
    RIGHT,
    CENTER

}

public class DriveByCamera {
    // camera location on the robot
    final int CAMERA_FORWARD_DISPLACEMENT = 177;   // mm in front of robot center
    final int CAMERA_VERTICAL_DISPLACEMENT = 150;   // mm above ground
    final int CAMERA_LEFT_DISPLACEMENT = 0;   // Camera is ON the robot's center line

    public static final String VISION_KEY = "AbQajKn/////AAAAGU7i3QBuOUjSp+FIwylUn4R1t6lifGcZsFjRQgSVrzy6o0q2+awGv2OiUTS+JJDAP1cPjzy8Qqaa+W0Kp1y+wDyNNJzXPKTk9zpoeA6tCnaH1N7xsfUz8DxBRZmipkzHUWSCwCkslVlvf71X4HXh3tqIJetRchP55t26A3yfgQHBZN6aMMGXR/DWLNv1zI8+t7O4dml5kmHkZLG8yLOr9G8jWUUt7A7e4eoWLxkFm7JE+DTBdIH3dSekVfcSx4tZ09/bDL4fATsN6oom4YzDeWDUaC9M4C+/7MBDLaG7dhtSs6aXhhcSfD3GF1mb1KMju4nO9xuM9ehbCTNJlyt/uHAihVmRtVu08IeVSTO+XsvS";

    public static final double MOTOR_FULL_POWER = .3;
    public static final double MOTOR_MEDIUM_POWER = 0;
    public static final int MOTOR_STOP = 0;

    private final HardwareMap hardwareMap;

    public double xAxisTranslation;
    public double yAxisTranslation;
    public double zAxisTranslation;

    public double xAxisRotation;
    public double yAxisRotation;
    public double zAxisRotation;

    DcMotor rightMotor = null;
    DcMotor leftMotor = null;
    Telemetry telemetry;

    VuforiaTrackables vuforiaTrackables;

    public DriveByCamera(DcMotor rightMotor, DcMotor leftMotor, Telemetry telemetry, HardwareMap hardwareMap) {

        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        vuforiaTrackables = initializeVuforia();
    }

    public BotDirection getBotDirection(double translationX, int translationGoal) {
        BotDirection botDirection;
        if (translationX > translationGoal + 10) {
            botDirection = BotDirection.LEFT;
        } else if
                (translationX < translationGoal - 10) {
            botDirection = BotDirection.RIGHT;
        } else {
            botDirection = BotDirection.CENTER;
        }
        telemetry.addData("Target displacement", botDirection.toString());
        return botDirection;
    }

    public static String formatMatrixString(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    @NonNull
    public static VuforiaLocalizer getVuforiaLocalizer(VuforiaLocalizer.Parameters parameters) {
        parameters.vuforiaLicenseKey = VISION_KEY;

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        return ClassFactory.getInstance().createVuforia(parameters);
    }

    private VuforiaTrackables initializeVuforia() {

        VuforiaLocalizer.Parameters parameters = getVuforiaParameters();
        VuforiaLocalizer vuforia = DriveByCamera.getVuforiaLocalizer(parameters);

        VuforiaTrackables trackables = vuforia.loadTrackablesFromAsset("RoverRuckus");

        VuforiaTrackable blueRover = trackables.get(0);
        blueRover.setName(TargetImage.BLUE_ROVER);
        VuforiaTrackable redFootprint = trackables.get(1);
        redFootprint.setName(TargetImage.RED_FOOTPRINT);
        VuforiaTrackable frontCraters = trackables.get(2);
        frontCraters.setName(TargetImage.FRONT_CRATERS);
        VuforiaTrackable backSpace = trackables.get(3);
        backSpace.setName(TargetImage.BACK_SPACE);


        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        parameters.cameraDirection == FRONT ? 90 : -90, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : trackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        trackables.activate();
        return trackables;
    }

    @NonNull
    public VuforiaLocalizer.Parameters getVuforiaParameters() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        return new VuforiaLocalizer.Parameters(cameraMonitorViewId);
    }

    public void driveToImage(String selectedImage, int distanceGoal, int translationGoal) {

        //TODO: Use the targetName to find the desired target

        VuforiaTrackable selectedTarget = vuforiaTrackables.get(0);

        if (selectedTarget != null) {

            OpenGLMatrix pose = determinePose(selectedTarget);

            if (pose != null) {
                VectorF translation = pose.getTranslation();
                Orientation rotation = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                getTranslationValues(translation);
                getRotationValues(rotation);

                BotDirection botDirection = getBotDirection(xAxisTranslation, translationGoal);

                boolean isDistanceGoalMet = getIfDistanceGoalMet(distanceGoal, zAxisTranslation);

                if (isDistanceGoalMet) {
                    telemetry.addLine("Got there!!!");
                    stopMotors();
                } else {
                    telemetry.addLine("Still Going...");
                    switch (botDirection) {
                        case CENTER:
                            // drive straight
                            leftMotor.setPower(MOTOR_FULL_POWER);
                            rightMotor.setPower(MOTOR_FULL_POWER);
                            break;
                        case RIGHT:
                            leftMotor.setPower(MOTOR_MEDIUM_POWER);
                            rightMotor.setPower(MOTOR_FULL_POWER);
                            // turn left;
                            break;
                        case LEFT:
                            leftMotor.setPower(MOTOR_FULL_POWER);
                            rightMotor.setPower(MOTOR_MEDIUM_POWER);
                            // turn right
                            break;
                    }
                    telemetry.addData("Right motor power", rightMotor.getPower());
                    telemetry.addData("Left motor power", leftMotor.getPower());

                }
            }
        } else {
            telemetry.addData("VuMark", "not visible");
            stopMotors();
        }

        telemetry.update();
    }

    private boolean getIfDistanceGoalMet(int distanceGoal, double currentTranslation) {
        return (currentTranslation > distanceGoal);
    }

    @Nullable
    private OpenGLMatrix determinePose(VuforiaTrackable vuforiaTrackable) {
        telemetry.addData("VuMark", "%s visible", vuforiaTrackable);

        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) vuforiaTrackable.getListener()).getPose();
        telemetry.addData("Pose", DriveByCamera.formatMatrixString(pose));
        return pose;
    }

    private void stopMotors() {
        leftMotor.setPower(MOTOR_STOP);
        rightMotor.setPower(MOTOR_STOP);
    }

    private void getRotationValues(Orientation rot) {
        // Extract the rotational components of the target relative to the robot
        xAxisRotation = Math.round(rot.firstAngle);
        yAxisRotation = Math.round(rot.secondAngle);
        zAxisRotation = Math.round(rot.thirdAngle);

        telemetry.addData("rotation X", xAxisRotation);
        telemetry.addData("rotation Y", yAxisRotation);
        telemetry.addData("rotation Z", zAxisRotation);
    }

    private void getTranslationValues(VectorF trans) {
        // Extract the X, Y, and Z components of the offset of the target relative to the robot
        xAxisTranslation = Math.round(trans.get(0));
        yAxisTranslation = Math.round(trans.get(1));
        zAxisTranslation = Math.round(trans.get(2));

        telemetry.addData("translation X", xAxisTranslation);
        telemetry.addData("translation Y", yAxisTranslation);
        telemetry.addData("translation Z", zAxisTranslation);
    }

}
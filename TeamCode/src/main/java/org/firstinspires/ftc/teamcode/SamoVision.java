package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class SamoVision {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    public static final String VISION_KEY = "AbQajKn/////AAAAGU7i3QBuOUjSp+FIwylUn4R1t6lifGcZsFjRQgSVrzy6o0q2+awGv2OiUTS+JJDAP1cPjzy8Qqaa+W0Kp1y+wDyNNJzXPKTk9zpoeA6tCnaH1N7xsfUz8DxBRZmipkzHUWSCwCkslVlvf71X4HXh3tqIJetRchP55t26A3yfgQHBZN6aMMGXR/DWLNv1zI8+t7O4dml5kmHkZLG8yLOr9G8jWUUt7A7e4eoWLxkFm7JE+DTBdIH3dSekVfcSx4tZ09/bDL4fATsN6oom4YzDeWDUaC9M4C+/7MBDLaG7dhtSs6aXhhcSfD3GF1mb1KMju4nO9xuM9ehbCTNJlyt/uHAihVmRtVu08IeVSTO+XsvS";

    private TFObjectDetector tensorFlowObjectDetector;
    private HardwareMap hardwareMap;

    public SamoVision(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.tensorFlowObjectDetector = initializeTensorFlowObjectDetector();
        tensorFlowObjectDetector.activate();
    }


    private VuforiaLocalizer initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VISION_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        return ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }


    private TFObjectDetector initializeTensorFlowObjectDetector() {
        VuforiaLocalizer vuforia = initVuforia();
        TFObjectDetector result;
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        result = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        result.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        return result;
    }


    Recognition seenItem;

    public double getEstimatedAngle() {
        return seenItem.estimateAngleToObject(AngleUnit.DEGREES);
    }

    public boolean trySeeGoldThing() {
        this.seenItem = null;
        List<Recognition> itemsSeen = tensorFlowObjectDetector.getUpdatedRecognitions();
        if (itemsSeen == null) {
            return false;
        }

        for (Recognition item : itemsSeen) {
            if (item.getLabel() == LABEL_GOLD_MINERAL) {
                this.seenItem = item;
            }
        }

        return this.seenItem != null;
    }
}

/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@TeleOp(name = "Detect Silver Mineral", group = "Concept")
public class TensorFlowDetectSilverMineralOpMode extends OpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    public static final String VISION_KEY = "AbQajKn/////AAAAGU7i3QBuOUjSp+FIwylUn4R1t6lifGcZsFjRQgSVrzy6o0q2+awGv2OiUTS+JJDAP1cPjzy8Qqaa+W0Kp1y+wDyNNJzXPKTk9zpoeA6tCnaH1N7xsfUz8DxBRZmipkzHUWSCwCkslVlvf71X4HXh3tqIJetRchP55t26A3yfgQHBZN6aMMGXR/DWLNv1zI8+t7O4dml5kmHkZLG8yLOr9G8jWUUt7A7e4eoWLxkFm7JE+DTBdIH3dSekVfcSx4tZ09/bDL4fATsN6oom4YzDeWDUaC9M4C+/7MBDLaG7dhtSs6aXhhcSfD3GF1mb1KMju4nO9xuM9ehbCTNJlyt/uHAihVmRtVu08IeVSTO+XsvS";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tensorFlowObjectDetector} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tensorFlowObjectDetector;

    private float getHorizontalCenter(Recognition recognizedItem) {
        return recognizedItem.getLeft() + recognizedItem.getWidth();
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VISION_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tensorFlowObjectDetector = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tensorFlowObjectDetector.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    @Override
    public void stop() {
        if (tensorFlowObjectDetector != null) {
            tensorFlowObjectDetector.shutdown();
        }
        super.stop();
    }

    @Override
    public void init() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        /** Activate Tensor Flow Object Detection. */
        if (tensorFlowObjectDetector != null) {
            tensorFlowObjectDetector.activate();
        }

    }


    @Override
    public void loop() {
        if (tensorFlowObjectDetector != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> itemsSeen = tensorFlowObjectDetector.getUpdatedRecognitions();
            if (itemsSeen == null) {
                return;
            }
            telemetry.addData("Number of things seen: ", itemsSeen.size());

            for (Recognition item : itemsSeen) {
                reportItem(item);
            }
//            if (itemsSeen.size() == 3) {
//                int goldMineralX = -1;
//                int silverMineral1X = -1;
//                int silverMineral2X = -1;
//                for (Recognition recognition : itemsSeen) {
//                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
//                        goldMineralX = (int) recognition.getLeft();
//                    } else if (silverMineral1X == -1) {
//                        silverMineral1X = (int) recognition.getLeft();
//                    } else {
//                        silverMineral2X = (int) recognition.getLeft();
//                    }
//                }
//                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
//                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
//                        telemetry.addData("Gold Mineral Position", "Left");
//                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
//                        telemetry.addData("Gold Mineral Position", "Right");
//                    } else {
//                        telemetry.addData("Gold Mineral Position", "Center");
//                    }
//                }
//            }
        }
    }

    public void reportItem(Recognition item) {
        float itemCenter = getHorizontalCenter(item);
        telemetry.addLine(item.getLabel());
        telemetry.addData("image width", item.getImageWidth());
        telemetry.addData("item left", item.getLeft());
        telemetry.addData("item right", item.getRight());
        telemetry.addData("center ", itemCenter);
        telemetry.addData("angle", item.estimateAngleToObject(AngleUnit.DEGREES));
        if (itemCenter > Math.floor(item.getImageWidth()/2)) {
            telemetry.addLine("turn right");
        } else if (itemCenter < Math.floor(item.getImageWidth()/2)) {
            telemetry.addLine("turn left");
        } else {
            telemetry.addLine("dead center");
        }
    }
}

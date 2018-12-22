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


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@TeleOp(name = "Detect Gold Mineral", group = "Test")
public class TensorFlowDetectSilverMineralOpMode extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    public static final String VISION_KEY = "AbQajKn/////AAAAGU7i3QBuOUjSp+FIwylUn4R1t6lifGcZsFjRQgSVrzy6o0q2+awGv2OiUTS+JJDAP1cPjzy8Qqaa+W0Kp1y+wDyNNJzXPKTk9zpoeA6tCnaH1N7xsfUz8DxBRZmipkzHUWSCwCkslVlvf71X4HXh3tqIJetRchP55t26A3yfgQHBZN6aMMGXR/DWLNv1zI8+t7O4dml5kmHkZLG8yLOr9G8jWUUt7A7e4eoWLxkFm7JE+DTBdIH3dSekVfcSx4tZ09/bDL4fATsN6oom4YzDeWDUaC9M4C+/7MBDLaG7dhtSs6aXhhcSfD3GF1mb1KMju4nO9xuM9ehbCTNJlyt/uHAihVmRtVu08IeVSTO+XsvS";

    private TFObjectDetector tensorFlowObjectDetector;

    private float getHorizontalCenter(Recognition recognizedItem) {
        return (float) (recognizedItem.getLeft() + Math.floor(recognizedItem.getWidth() / 2));
    }

    private VuforiaLocalizer initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VISION_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

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


    public void cleanup() {
        if (tensorFlowObjectDetector != null) {
            tensorFlowObjectDetector.shutdown();
        }
        super.stop();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing");
        initialize();

        telemetry.addLine("Waiting for start");

        this.waitForStart();

        run();

        cleanup();
    }

    public void initialize() {
        tensorFlowObjectDetector = initializeTensorFlowObjectDetector();

        if (tensorFlowObjectDetector != null) {
            tensorFlowObjectDetector.activate();
        }
    }


    public void run() {
        if (tensorFlowObjectDetector == null) {
            return;
        }

        while (!this.isStopRequested()) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> itemsSeen = tensorFlowObjectDetector.getUpdatedRecognitions();
            if (itemsSeen == null) {
                continue;
            }

            telemetry.addData("Number of things seen: ", itemsSeen.size());

            for (Recognition item : itemsSeen) {
                if (item.getLabel() == LABEL_GOLD_MINERAL) {
                    reportItem(item);
                    telemetry.update();
                }
            }

        }
    }

    public void reportItem(Recognition item) {
        float itemCenter = getHorizontalCenter(item);
        int itemImageWidth = item.getImageWidth();

        telemetry.addData("item name", item.getLabel());
        telemetry.addData("image width", itemImageWidth);
        telemetry.addData("item left", item.getLeft());
        telemetry.addData("item right", item.getRight());
        telemetry.addData("center ", itemCenter);
        telemetry.addData("angle", item.estimateAngleToObject(AngleUnit.DEGREES));
        if (itemCenter > Math.floor(itemImageWidth / 2)) {
            telemetry.addLine("turn right");
        } else if (itemCenter < Math.floor(itemImageWidth / 2)) {
            telemetry.addLine("turn left");
        } else {
            telemetry.addLine("dead center");
        }
    }
}

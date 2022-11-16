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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "Blue Cup DuckDuckDuck", group = "Zhou")
@Disabled
public class BlueCupDuck extends LinearOpMode {

    Hardware8295_3 robot   = new Hardware8295_3();   // Use the 3rd hardware
    private ElapsedTime runtime = new ElapsedTime();
    private int position = 1;  //default to cup not seen/left position

    /* Note: This sample uses the all-objects Tensor Flow model (cup.tflite), which contains
     * the following 1 detectable object
     *  0: Cup
     */
    private static final String TFOD_MODEL_ASSET = "cup.tflite";
    private static final String[] LABELS = {
            "Cup",
    };


    static final double     H_SPEED = 0.8;
    static final double     M_SPEED = 0.5;
    static final double     S_SPEED = 0.3;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "ASIBUQn/////AAABmRyaZGLR1UDVlE2D1BNfYhYJjXZ7y3XXZ94fs0//LejYcci1bQhrI2N6aNP3r4/SUv/uirVIH2D9V7y480pPj7yp2fExBwZyNz3jN3J2/Zg1j3hLh/TqkFJE8gu/n6IMmDsELuZpY6WI14QgBnaG6nISbqTD8pdJj6q9V/3s2C2jc4kRQ20AxfG9dfw19gkNIDiM4uU12cv0lAxk0SfhHJgtipLRDB//tYU/YVIv+mBi1PIGM6QwNacMfKT1HF2yZOYaN2we5qEWc8Eso1KflOf5igZJulH3jhcR0t0EHmTCGBoFKPZ3D7k1dYJmKJ0cVEZjpDiCGLK/VMPGknEwMcEI168VeAL0vHwHGjOzDpZT";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //set motors on encoder
            robot.reset();
            robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // 1. Find cup
            position = findCup();
            sleep(500);
            // set lift height and forward distances based on the cup's position
            double forwardInchValue = 0;

            if (position==-1) {//cup on right
                robot.liftUp(1,11,3);
                forwardInchValue = 23;
            }
            else if (position==0) {//cup on middle
                robot.liftUp(1,6,3);
                forwardInchValue=21.5;
            }
            else {//cup on left
                robot.liftUp(1,0,1);
                forwardInchValue=20.5;
            }

            // 2. Once the lift is lifted, navigate to the hub
            sleep(500);
            //red side needs extra distance traveled, calculation yields 9 inches
            robot.rightInches(M_SPEED,-23, 3);
            robot.forwardInches(S_SPEED,forwardInchValue,3);
            sleep(1500);

            //3. Next, push cube out and flip up arm
            robot.rollerSpeed(-1,2);
            robot.arm.setPosition(0.7);
            sleep(500);

            //4. shift to the wall then back into carousel
            robot.forwardInches(1,-6,4);
            robot.rightInches(H_SPEED,41, 5);
            //robot.rightInches(M_SPEED,5,3);

            // 5. spin the duck
            robot.spinnerR.setPower(.7);
            robot.forwardInches(0.2,-1*(forwardInchValue-9),3); //adjust for distance
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 4.0)) {
                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            robot.spinnerR.setPower(0);
            sleep(500);

            //6. forward to park
            robot.forwardInches(M_SPEED, 18, 3);
            robot.rightInches(M_SPEED,5,3);
             /*Needs adjusting*/
            stop();

        }
    }

    public int findCup() {
        int location = 1;
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    i++;
                    // Change "Duck" to "Cup"
                    if (recognition.getLabel() == "Cup") {
                        if (((recognition.getLeft() + recognition.getRight()) / 2) < 350) {
                            location = 0;
                            telemetry.addData("middle", location);
                        }
                        else if (((recognition.getLeft() + recognition.getRight()) / 2) > 350) {
                            location = -1;
                            telemetry.addData("right", location);
                        }
                    }
                }
                telemetry.update();
            }
        }
        return location;
    }

    public double findDistance()
    {
        ModernRoboticsI2cRangeSensor rangeSensor;
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
        while (opModeIsActive()) {
            telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
            telemetry.addData("raw optical", rangeSensor.rawOptical());
            telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
            telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        return rangeSensor.getDistance(DistanceUnit.CM);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

}

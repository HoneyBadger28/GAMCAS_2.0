/*
Developed By FTC Team 'Work In Progress'
*/

package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Objects;

@Autonomous(name="BaseBotAutoLeft", group="BaseBotAutoLeft")

@Disabled
public class BaseBotAutoLeft extends LinearOpMode {
    //StartUp
    HardwareBaseBot robot = new HardwareBaseBot();

    //Setup Variables
    double BaseDiv = 50;


    double WheelDiameter = 9.6; //CentiMeters

    double NewRotVal = (PI*WheelDiameter);

    //Basic Vuforia Assets
    public static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    public static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;
    //Initialize Phases

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0 / 9.0);
        }
        String Recognize = null;
        while (!isStarted()) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                double Confidence = 0.5;
                telemetry.addData("# Objects Detected", updatedRecognitions.size());
                for (Recognition recognition : updatedRecognitions) {
                    double col = (recognition.getLeft() + recognition.getRight()) / 2;
                    double row = (recognition.getTop() + recognition.getBottom()) / 2;
                    double width = Math.abs(recognition.getRight() - recognition.getLeft());
                    double height = Math.abs(recognition.getTop() - recognition.getBottom());
                    telemetry.addData("", " ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                    telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                    telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", Recognize, Confidence * 100);
                    telemetry.update();
                    if (Confidence < recognition.getConfidence()) {
                        Recognize = recognition.getLabel();
                        Confidence = recognition.getConfidence();
                    }
                }

            }
        }

        //Start Autonomous
        MoveMod(530, 0, 0.01, 500, 0);
        Move(6, 0, 0, 0.25, 15, BaseDiv);
        MoveMod(30, 0, 0.01, 500, 0);
        MoveMod(30, 1, 0.01, 500, 0.5);
        MoveMod(240, 1, 0.01, 500, 0);
        Move(0, -12, 0, 0.40, 45, BaseDiv);
        Move(147, 0, 0, 0.40, 45, BaseDiv);
        Move(-50, 0, 0, 0.40, 45, BaseDiv);
        Move(0, 0, 45, 0.40, 45, BaseDiv);
        Move(0, -7, 0, 0.40, 45, BaseDiv);
        MoveMod(3000, 1, 0.01, 500, 0);
        Move(24, 0, 0, 0.40, 45, BaseDiv);
        MoveMod(3000, 0, 0.01, 500, 0.2);
        MoveMod(3300, 0, 0.01, 500, 0);
        Move(-25, 0, 0, 0.40, 45, BaseDiv);
        MoveMod(0, 0, 0.01, 500, 0);
        Move(0, 0, -45, 0.40, 45, BaseDiv);


        if (Objects.equals(Recognize, "1 Bolt")) {
            Move(0, -100, 0, 0.40, 25, BaseDiv);
        } else if (Objects.equals(Recognize, "2 Bulb")) {
            Move(0, 0, 0, 0.40, 25, BaseDiv);
        } else if (Objects.equals(Recognize, "3 Panel")) {
            Move(0, 100, 0, 0.40, 25, BaseDiv);
        }
        Move(10, 0, 0, 0.40, 25, BaseDiv);
        }
















    //Custom Stuff (You don't need to understand this)
    public double SafeZone(double value, double safeZone) {
        if (abs(value) >= safeZone) {
            return(value);
        } else {
            return(0);
        }
    }
    public void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = robot.VuforiaKey;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
    public void Move(double FB,double LR , double R, double Speed, double Zone, double Div) {
        R = (R/90)*80;
        int Check = 0;
        double G;
        double OldRB = robot.BR.getCurrentPosition();
        double OldRF = robot.FR.getCurrentPosition();
        double OldLB = robot.BL.getCurrentPosition();
        double OldLF = robot.FL.getCurrentPosition();

        while(Check < 4 && opModeIsActive()){

            Modules();
            Check = 0;

            //RightBack(FB - LR - R)
            G = ((FB + LR - R) / NewRotVal) * 360 + OldRB;
            G = (G - robot.BR.getCurrentPosition());
            if(SafeZone(G, Zone) != 0){
                G = G/Div;
                if(abs(G) > Speed){
                    G = (G / abs(G)) * Speed;
                }
                robot.BR.setPower(G);
            } else{
                robot.BR.setPower(0);
                Check += 1;
            }

            //RightFront(FB + LR - R)
            G = ((FB - LR - R) / NewRotVal) * 360  + OldRF;
            G = (G - robot.FR.getCurrentPosition());
            if(SafeZone(G, Zone) != 0){
                G = G/Div;
                if(abs(G) > Speed){
                    G = (G / abs(G)) * Speed;
                }
                robot.FR.setPower(G);
            } else{
                robot.FR.setPower(0);
                Check += 1;
            }

            //LeftBack(FB + LR + R)
            G = ((FB - LR + R) / NewRotVal) * 360 + OldLB;
            G = (G - robot.BL.getCurrentPosition());
            if(SafeZone(G, Zone) != 0){
                G = G/Div;
                if(abs(G) > Speed){
                    G = (G / abs(G)) * Speed;
                }
                robot.BL.setPower(G);
            } else{
                robot.BL.setPower(0);
                Check += 1;
            }

            //LeftFront(FB - LR + R)
            G = ((FB + LR + R) / NewRotVal) * 360 + OldLF;
            G = (G - robot.FL.getCurrentPosition());
            if(SafeZone(G, Zone) != 0){
                G = G/Div;
                if(abs(G) > Speed){
                    G = ((G / abs(G)) * Speed);
                }
                robot.FL.setPower(G);
            } else{
                robot.FL.setPower(0);
                Check += 1;
            }
        }
    }
    public void MoveMod(double ViperSlide, double GrabberV, double Zone, double Div, double Wait) {

    }
    public void Modules(){

    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class HardwareBaseBot {
    //Devices / Motors
    public DcMotorEx  BL    = null;
    public DcMotorEx  FL   = null;
    public DcMotorEx  BR   = null;
    public DcMotorEx  FR  = null;
    public DcMotorEx  SL  = null;
    public DcMotorEx  SR  = null;
    public DcMotor Fan = null;
    public AnalogInput Temp = null;
    public BNO055IMU IMU1         = null;
    public BNO055IMU IMU2         = null;
    public String VuforiaKey      = ("Ad0uvlv/////AAABme2lR6q0OEbHnomjejaBvl9rSG2Or8YMU8cxlie2hjZ9Wc/lUmjSCVEH8L0VeNDNRpkR6pg9XHt+QiL+TeIRrq8ARvSOU2pPvPpjmr/vQyaYqab3SLW1OfeTGK1c10TDeLful6XIOuGcD69urFr3rB009khOiYibiJYc7sQt7UZfPpR06+hM2bnyYmKkuj4cdR6dfhMYZLY0agxWHWyXMLZL/tJv+hEW2mDx1UaEzzx6qsPsn9OgitILdlKS1NJU0nDQrnwy6NyNPx3/jeXuTgg9CjufEV2ifbCusNoeyIovleijIXkG9tzF1p7mOIPIdulhDrdErQcMR4aQVvRXAumSFWwU1xuoQ6ciENgDNfAN");
    HardwareMap hwMap             = null;

    public HardwareBaseBot(){
    }
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        //Camera
        hwMap.get(WebcamName.class, "Webcam");
        // Define and Initialize Motors
        BL    = hwMap.get(DcMotorEx.class, "BL");
        FL   = hwMap.get(DcMotorEx.class, "FL");
        BR   = hwMap.get(DcMotorEx.class, "BR");
        FR  = hwMap.get(DcMotorEx.class, "FR");
        SL = hwMap.get(DcMotorEx.class, "SL");
        SR = hwMap.get(DcMotorEx.class, "SR");
        Fan = hwMap.get(DcMotor.class, "Fan");
        Temp = hwMap.get(AnalogInput.class, "Temp");

        //Define Motor Direction
        BL.setDirection(DcMotorEx.Direction.FORWARD);
        FL.setDirection(DcMotorEx.Direction.FORWARD);
        BR.setDirection(DcMotorEx.Direction.REVERSE);
        FR.setDirection(DcMotorEx.Direction.REVERSE);
        SL.setDirection(DcMotorEx.Direction.FORWARD);
        SR.setDirection(DcMotorEx.Direction.FORWARD);

        // Set all motors to zero power
        BL.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        FR.setPower(0);
        SL.setPower(0);
        SR.setPower(0);

        // Set all motors to run with encoders.
        BL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        SL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        SR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        BL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        SL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        SR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        IMU1 = hwMap.get(BNO055IMU.class, "IMU1");
        IMU1.initialize(parameters);
        IMU2 = hwMap.get(BNO055IMU.class, "IMU2");
        IMU2.initialize(parameters);
    }
}
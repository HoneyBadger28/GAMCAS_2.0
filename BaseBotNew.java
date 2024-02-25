/*
Developed By FTC Team 'Work In Progress'
*/
package org.firstinspires.ftc.teamcode;
import static java.lang.Math.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="BaseBotNew", group="BaseBotNew")
//@Disabled
public class BaseBotNew extends OpMode {
    @Override
    //StartUp
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Console", "Robot Running");
    }


    HardwareBaseBot robot = new HardwareBaseBot();
    Orientation LastGyroAngle = new Orientation();
    ElapsedTime time = new ElapsedTime();

    //Variables
    double CurrentGyroAngle;
    double Gyro;
    double OldTime;
    double SuspensionL;
    double SuspensionR;
    double AngleL;
    double AngleR;
    double DriftL;
    double DriftR;
    double BrakeBR;
    double BrakeFR;
    double BrakeBL;
    double BrakeFL;
    double MenuSelect;
    double ArmLength = 19.2;
    Boolean MenuPress;
    double MenuID;
    double WheelDiameter = 9.6;
    double WheelRotVal = WheelDiameter*PI;
    String Drivemode = "Drive";
    String Subdrivemode = "Gyro";
    String Text;
    double R;
    int Menu;
    double ButtonDown;
    double Temp;
    double AirTemp;
    @Override
    public void loop() {
        telemetry.clearAll();
        Fan(15,50, 0.1,2, 1);
        telemetry.addLine("Temp: " + round(AirTemp*100)/100 + "C");
        telemetry.addLine("CPUTemp: " + round(Temp*100)/100 + "C");
        MenuPress = (gamepad1.back);
        Text = Drivemode;
        if (!Subdrivemode.equals("")) {
            Text = Text + Subdrivemode;
        }
        telemetry.addLine(Text);
        //Menu Button
        MenuPress = (gamepad1.back);
        //Menu
        if (MenuPress) {
            if (ButtonDown == 0) {
                Menu += 1;
            }
            ButtonDown = 1;
        }
        Menu = Menu % 2;

        if (Menu == 1) {
            resetAngleGyro();
            if ((gamepad1.dpad_up | gamepad1.dpad_down)) {
                if (ButtonDown == 0) {
                    if (gamepad1.dpad_up) {
                        MenuSelect -= 1;
                    }
                    if (gamepad1.dpad_down) {
                        MenuSelect += 1;
                    }
                }
                ButtonDown = 1;
            }
            MenuID = 0;
            AddMenuItem("Gyro Correction Drive", "");
            AddMenuItem("Gyro Correction Drive", "");
            if (MenuSelect < 0) {
                MenuSelect = MenuID - 1;
            }
            MenuSelect = MenuSelect % MenuID;
        } else if (Drivemode.equals("Gyro Drive")) {
            double Speed = 0.3;
            double FB = -gamepad1.left_stick_y;
            double LR = -gamepad1.left_stick_x;
            double R  = -gamepad1.right_stick_x;
            Suspension(0, 90);
            FB = SafeZone(FB, 0.01);
            LR = SafeZone(LR, 0.01);
            R = SafeZone(R, 0.01);

            DriveWheels(
                    ((FB - LR + R) * Speed),
                    ((FB + LR + R) * Speed),
                    ((FB + LR - R) * Speed),
                    ((FB - LR - R) * Speed),
                    80);
        }
        if (gamepad1.dpad_up | gamepad1.dpad_down | MenuPress) {
            ButtonDown = 0;
        }
    }
    public void Fan(double Min, double Max, double mid, double Sensitivity, double Speed) {
        double Mid = ((Max-Min)*mid)+Min;
        Temp = (robot.Temp.getVoltage()-0.5)*100;
        AirTemp = Temp;
        Temp = ((Temp-Mid)*Sensitivity)+Mid;
        double Fan = 0;
        if (Temp > Min) {
            Fan = (1-(cos(((Temp - Min)/(Max-Min))*(PI))))/2;
        }
        if (Temp > Max) {
            Fan = 1;
        }
        Fan = Fan*Speed;
        telemetry.addLine("Fan Power:  " + Fan);
        robot.Fan.setPower(Fan);
    }
    public void AddMenuItem(String Name, String Subtype) {
        if (MenuSelect == MenuID){
            Text = "> ";
        } else {
            Text = "   ";
        }
        Text = Text + Name;
        Text = Text + Subtype;
        telemetry.addLine(Text);
        if (MenuSelect == MenuID){
            Drivemode = Name;
            Subdrivemode = Subtype;
        }
        MenuID += 1;
    }

    public void DriveWheels(double BR, double FR, double BL, double FL, double GyroSoft) {
        Gyro = 0;
        if (R != 0) {
            resetAngleGyro();
            OldTime = time.seconds();
            Gyro = 0;
        } else {
            if (time.seconds() - OldTime >= 0.35) {
                Gyro = SafeZone(getAngleGyro(), 1.51) / GyroSoft;
            } else {
                resetAngleGyro();
            }
        }
        BR = (BR - Gyro);
        FR = (FR - Gyro);
        BL = (BL + Gyro);
        FL = (FL + Gyro);
        if (abs(BR)+abs(FR)+abs(BL)+abs(FL) > 0) {
            robot.BR.setPower(BR);
            robot.FR.setPower(FR);
            robot.BL.setPower(BL);
            robot.FL.setPower(FL);
            BrakeBR = robot.BR.getCurrentPosition();
            BrakeFR = robot.FR.getCurrentPosition();
            BrakeBL = robot.BL.getCurrentPosition();
            BrakeFL = robot.FL.getCurrentPosition();
        } else {
            robot.BR.setPower(SafeZone((BrakeBR - robot.BR.getCurrentPosition()) / 100, 0.01));
            robot.FR.setPower(SafeZone((BrakeFR - robot.FR.getCurrentPosition()) / 100, 0.01));
            robot.BL.setPower(SafeZone((BrakeBL - robot.BL.getCurrentPosition()) / 100, 0.01));
            robot.FL.setPower(SafeZone((BrakeFL - robot.FL.getCurrentPosition()) / 100, 0.01));
        }
    }
    public void Suspension(double Tension, double Angle){
        AngleL = robot.SL.getCurrentPosition();
        AngleR = robot.SR.getCurrentPosition();
        SuspensionL = ((AngleL-Angle)-SuspensionL)/(100/(Tension/100));
        SuspensionR = ((AngleR-Angle)-SuspensionR)/(100/(Tension/100));
        robot.SL.setPower(-SuspensionL);
        robot.SR.setPower(-SuspensionR);
        DriftL = (sin(AngleL)*ArmLength)/WheelRotVal;
        DriftR = (sin(AngleR)*ArmLength)/WheelRotVal;
    }
    public void resetAngleGyro() {
        LastGyroAngle = robot.IMU1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        CurrentGyroAngle = 0;
    }
    public double getAngleGyro() {
        Orientation orientation = robot.IMU1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = orientation.firstAngle - LastGyroAngle.firstAngle;
        if (deltaAngle > 180) {
            deltaAngle -= 360;
        } else if (deltaAngle <= -180) {
            deltaAngle += 360;
        }
        CurrentGyroAngle += deltaAngle;
        LastGyroAngle = orientation;
        return CurrentGyroAngle;
    }
    public double SafeZone(double value, double safeZone) {
        if (abs(value) >= safeZone) {
            return(value);
        } else {
            return(0);
        }
    }
}
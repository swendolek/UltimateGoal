package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.vuforia.CameraDevice;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class SolidRobotRevamped {

    HardwareMap hardwareMap;

    DcMotor FLW, BLW, BRW, FRW; //Drive
    double frontLeftWheel = 0.0, backLeftWheel = 0.0, backRightWheel = 0.0, frontRightWheel = 0.0;

    DcMotor LI, RI; //Intake
    double leftIntake = 0.0, rightIntake = 0.0;

    public SolidRobotRevamped(HardwareMap hardwareMap, boolean auto){

        this.hardwareMap = hardwareMap;

        /*FLW = hardwareMap.dcMotor.get("FLW"); //Drive
        BLW = hardwareMap.dcMotor.get("BLW");
        BRW = hardwareMap.dcMotor.get("BRW");
        FRW = hardwareMap.dcMotor.get("FRW");*/

        LI = hardwareMap.dcMotor.get("LI");
        RI = hardwareMap.dcMotor.get("RI");

        //FLW.setDirection(DcMotorSimple.Direction.REVERSE);
        //BLW.setDirection(DcMotorSimple.Direction.REVERSE);

        RI.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void setComponents(){
        /*FLW.setPower(frontLeftWheel);
        BLW.setPower(backLeftWheel);
        BRW.setPower(backRightWheel);
        FRW.setPower(frontRightWheel);*/

        LI.setPower(leftIntake);
        RI.setPower(rightIntake);
    }

    void sleep( int millis )
    {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException ex) {

        }

    }

}

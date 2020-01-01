package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpRevamped", group = "TeleOp")
public class TeleOpRevamped extends OpMode{

    SolidRobotRevamped robot;

    public void init(){
        robot = new SolidRobotRevamped(hardwareMap, false);



    }

    public void init_loop(){
        robot.sleep(50);
    }

    public void start(){

    }

    public void loop(){
        robot.backLeftWheel = gamepad1.left_stick_y;
        robot.frontLeftWheel = gamepad1.left_stick_y;
        robot.backRightWheel = gamepad1.right_stick_y;
        robot.frontRightWheel = gamepad1.right_stick_y;

        if(gamepad1.right_trigger > 0.2){
            robot.rightIntake = gamepad1.right_trigger;
            robot.leftIntake = gamepad1.right_trigger;
        }
        else if(gamepad1.left_trigger > 0.2){
            robot.rightIntake = -gamepad1.left_trigger;
            robot.leftIntake = -gamepad1.left_trigger;
        }
        else{
            robot.rightIntake = 0.0;
            robot.leftIntake = 0.0;
        }

        robot.setComponents();
    }

}

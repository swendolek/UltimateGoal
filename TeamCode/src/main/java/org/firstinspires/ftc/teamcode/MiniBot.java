package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "MiniBot", group = "TeleOp")
public class MiniBot extends OpMode {

    int driveDiv = 2;

    DcMotor leftWheel;
    DcMotor rightWheel;

    public void init(){

        rightWheel = hardwareMap.dcMotor.get("LeftWheel");
        leftWheel = hardwareMap.dcMotor.get("RightWheel");

        leftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void init_loop(){

    }

    public void start(){

    }

    public void loop(){

        leftWheel.setPower(gamepad1.left_stick_y);
        rightWheel.setPower(gamepad1.right_stick_y);
    }

}

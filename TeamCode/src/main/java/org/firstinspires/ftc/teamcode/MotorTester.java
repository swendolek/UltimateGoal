package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Motor Test", group = "TeleOp")
public class MotorTester extends OpMode{

    DcMotor testMotor;

    public void init(){
        testMotor = hardwareMap.dcMotor.get("Motor");
    }

    public void loop(){
        testMotor.setPower(gamepad1.left_stick_y);
    }

}

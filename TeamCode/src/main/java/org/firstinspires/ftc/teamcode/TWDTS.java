package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PlazaDrive", group = "TeleOp")
public class TWDTS extends OpMode{

    double clawPos = 0.0;
    int driveDiv = 4;
    boolean clawButtonReleased = true;

    DcMotor RW, LW, LI, RI;

    /**
     * init - run once after init is pressed
     */
    @Override
    public void init(){
        RW = hardwareMap.dcMotor.get("RW");
        LW = hardwareMap.dcMotor.get("LW");

        RI = hardwareMap.dcMotor.get("RI");
        LI = hardwareMap.dcMotor.get("LI");

        RW.setDirection(DcMotorSimple.Direction.REVERSE);
        RI.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    /**
     * init_loop - run continuously after init is pressed
     */
    @Override
    public void init_loop(){


    }

    /**
     * start - run once after play is pressed
     */
    @Override
    public void start(){
    }

    /**
     * loop - run continuously after play is pressed
     */
    @Override
    public void loop(){

        if(gamepad1.right_bumper){
            driveDiv = 1;
        }
        else{
            driveDiv = 2;
        }

        RW.setPower(gamepad1.right_stick_y / driveDiv);
        LW.setPower(gamepad1.left_stick_y / driveDiv);

        if(gamepad1.a){
            RI.setPower(1.0);
            LI.setPower(1.0);
        }
        else if(gamepad1.b){
            RI.setPower(-1.0);
            LI.setPower(-1.0);
        }
        else{
            RI.setPower(0.0);
            LI.setPower(0.0);
        }
    }
}

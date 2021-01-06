package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TestAuto", group = "Autonomous")
public class TestAuto extends OpMode {

    Thread auto = new Thread(){
        public void run(){
            robot.drive(12, 0.5, 0.25,5);
        }
    };

    Robot robot;

    public void init(){
        robot = new Robot(hardwareMap);



    }

    public void start(){
        auto.start();
    }

    public void loop(){
        robot.update();
    }

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;

public class SolidMotor{

    final DcMotor.RunMode mainMode = DcMotor.RunMode.RUN_USING_ENCODER;

    DcMotor motor;
    double power = 0.0;

    public SolidMotor(HardwareMap hardwareMap, String hardwareName){

        motor = hardwareMap.dcMotor.get(hardwareName);
        motor.setMode(mainMode);

    }

    public void setPower(double power){
        this.power = power;
    }

    public void stop(){
        power = 0.0;
    }

    public void update(){
        motor.setPower(power);
    }

    public long getPos(){
        return motor.getCurrentPosition();
    }

    public void setResetMode(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setMainMode(){
        motor.setMode(mainMode);
    }

}

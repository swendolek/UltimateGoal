package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Created by Sam W on 9/22/2020
 */

public class Robot {


    final double COUNTS_PER_MOTOR_REV = 145.6;
    final double DRIVE_GEAR_REDUCTION = 2.0;
    final double WHEEL_DIAMETER_INCHES = 4.0;
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415); //Test l8r

    final double ACCELERATION_RATIO = 0.25; //TODO
    final double MINIMUM_DRIVE_POWER = 0.2; //TODO


    HardwareMap hardwareMap;

    ArrayList<Object> hardwareComponents = new ArrayList<Object>();

    ArrayList<SolidMotor> motorList = new ArrayList<SolidMotor>();

    ArrayList<SolidMotor> driveMotorList = new ArrayList<SolidMotor>();
    SolidMotor FLW, FRW, BRW, BLW;



    public Robot(HardwareMap hardwareMap){

        FLW = new SolidMotor(hardwareMap, "FLW");
        FRW = new SolidMotor(hardwareMap, "FRW");
        BRW = new SolidMotor(hardwareMap, "BRW");
        FRW = new SolidMotor(hardwareMap, "FRW");
        driveMotorList.addAll(Arrays.asList(FLW, FRW, BRW, BLW));


        motorList.addAll(driveMotorList);
        hardwareComponents.addAll(motorList);
    }

    public void sleep(long millis){
        try{
            Thread.sleep(millis);
        } catch(InterruptedException ex){
            //if you get to this, it is an official bruh moment
        }
    }

    public void setDrivePower(double power){
        for(SolidMotor motor : motorList){
            motor.setPower(power);
        }
    }

    public void resetDriveEncoders(){
        for(SolidMotor m : driveMotorList){
            m.setResetMode();
        }
        sleep(10); //TODO
        for(SolidMotor m : driveMotorList){
            m.setMainMode();
        }
    }

    /*
    Forward: ++
    Backward: +-
    Function adapted from team 6955 (I have no idea who they are or where they are from)
     */
    public void drive(double inches, double maxPower, double rampUpTime, double timeOut){

        resetDriveEncoders();

        long dT = (long) (inches * COUNTS_PER_INCH);
        if(maxPower < 0) dT *= -1;

        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() < startTime + (timeOut * 1000)
                && (Math.abs(BLW.getPos()) < dT || Math.abs(BRW.getPos()) < dT)) {

            double power = maxPower;

            /*//Linear vel increase using time, can be adapted for ramp down as well
            double rampUpRatio = System.currentTimeMillis() / (startTime + rampUpTime);
            if(rampUpRatio < 1){
                power = MINIMUM_DRIVE_POWER + ((maxPower - MINIMUM_DRIVE_POWER) * rampUpRatio);
            }*/

            //If ramping up
            if(BLW.getPos() < dT * ACCELERATION_RATIO){

                double decimalWayThroughRampUp = BLW.getPos() / (dT * ACCELERATION_RATIO); // 50 / (1000 * 0.25) = 0.2

                power = MINIMUM_DRIVE_POWER + ((maxPower - MINIMUM_DRIVE_POWER)) * decimalWayThroughRampUp;
            }

            //If ramping down
            //This is probably quadratic if I had to guess, if we want linear we have to use time
            if(BLW.getPos() > dT - (dT * ACCELERATION_RATIO)){

                double ticksAboveRampDownThreshold = (BLW.getPos() - (dT - (dT * ACCELERATION_RATIO))); //800 - 750
                double decimalWayThroughRampDown = ticksAboveRampDownThreshold / (dT * ACCELERATION_RATIO); //50 / (1000 * 0.25) = 0.2

                power = MINIMUM_DRIVE_POWER + ((maxPower - MINIMUM_DRIVE_POWER)) * (1 - decimalWayThroughRampDown);
            }

            double leftPower = power, rightPower = power;
            if(true){
                //this is where we diverge left vs right if needed
            }

            FLW.setPower(leftPower);
            BLW.setPower(leftPower);
            BRW.setPower(rightPower);
            FRW.setPower(rightPower);
        }

        setDrivePower(0.0);

    }




    public void update(){
        for(SolidMotor motor : motorList){
            motor.update();
        }



    }

}

package org.firstinspires.ftc.teamcode;

public class FeedForwardController{
    double ks;
    double kv;
    double ka;
    double setpoint;
    double speed;
    double acceleration;

    public FeedForwardController(){}

    public void calibrate(double Ks, double Kv, double Ka){
        ks = Ks;
        kv = Kv;
        ka = Ka;
    }

    public void changeData(double setpoint, double speed, double acceleration){
        this.setpoint = setpoint;
        this.speed = speed;
        this.acceleration = acceleration;
    }

    public double update(){
        return ks + kv * speed + ka * acceleration;
    }
}

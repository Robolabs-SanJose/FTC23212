package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double k_p;
    private double k_i;
    private double k_d;

    private double setpoint;
    private double sum;
    private double t_prev;
    private double e_prev;
    private ElapsedTime runTime = new ElapsedTime();
    private boolean initialized = false;

    public PIDController(double k_p, double k_i, double k_d){
        this.k_p = k_p;
        this.k_i = k_i;
        this.k_d = k_d;
    }

    public void setSetpoint(double setpoint){
        if (this.setpoint != setpoint) {
            this.setpoint = setpoint;
            runTime.reset();
            sum = 0;
            e_prev =  0;
        }
        initialized = true;
    }

    public double getSetpoint(){
        if (initialized) {
            return setpoint;
        }else{
            return 0;
        }
    }

    public void resetTuning(double k_p, double k_i, double k_d, boolean keepSetpoint){
        this.k_p = k_p;
        this.k_i = k_i;
        this.k_d = k_d;
        if (!keepSetpoint) {
            initialized = false;
        }
    }

    public double getError(double current){
        if (initialized){
            return setpoint - current;
        }else{
            return 0;
        }
    }

    public double update(double current, boolean useGyro){
        if (initialized) {
            double t_cur = runTime.nanoseconds() * 0.000000001;
            double error;
            if (useGyro) {
                error = getErrorGyro(current);
            }else{
                error = getError(current);
            }
            double dt = (t_cur) - t_prev;
            sum += error * dt;
            double dedt = (error - e_prev) / dt;
            e_prev = error;
            t_prev = t_cur;
            return (k_p * error) + (k_i * sum) + (k_d * dedt);
        }else {
            return 0;
        }
    }

    public double getErrorGyro(double current){
        if (initialized){
            double sc =  setpoint - current;
            if ((-180 < sc) && (sc < 180)){
                return sc;
            }else if ((sc <  -180)){
                return sc + 360;
            }else if (180 < sc){
                return sc - 180;
            }else{
                return 0;
            }
        }else{
            return 0;
        }
    }
}
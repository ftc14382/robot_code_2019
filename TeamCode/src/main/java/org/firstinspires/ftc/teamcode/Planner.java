package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;

public class Planner {
    double yIntercept = 0.145;//Was 0.1
    double yMax = 1;
    double target;
    double a = 0.1;
    double b = -0.6;
    double x1;
    double x2;
    double offset;


    public Planner(double startX, double t, double power) {
        offset = startX;//Only operate greater than or equal to 0
        target = Math.abs(t-offset);
        yMax = power;
        x1 = (yMax-yIntercept)/a;
        x2 = yMax/b+target;
        if(x1>x2) {
            x1 = (-b * target - yIntercept) / (a - b);
            x2 = x1;
        }
    }

    public double getPower(double x) {
        double returnVal;
        x= Math.abs(x-offset);
        if(x<x1){
            returnVal = x*a+yIntercept;
        } else if(x>x2) {
            returnVal = b*(x-target);
        } else {
            returnVal = yMax;
        }

        if(returnVal < 0.14) {
            returnVal = 0.14;
        }
        return(returnVal);
    }
}

package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;

public class LookUpTableAngleLimits {
    ArrayList<Integer> speeds = new ArrayList<Integer>();    
    ArrayList<Integer> limits = new ArrayList<Integer>();    

    public LookUpTableAngleLimits(){
        speeds.add(0);
        limits.add(0);
        speeds.add(4);
        limits.add(15);
        speeds.add(6);
        limits.add(5);
        speeds.add(8);
        limits.add(3);
    }

    public double lookUpLimit(double speed){
        //Variable Initilization
        double limit = 0;
        double error = Math.abs(speeds.get(0)-speed);
        int index1 =0;
        int index2 = 0;
        double percentage =0;
        //Searches for point closest to our distance
        for(int i =0; i<speeds.size(); i++){
            if(error>Math.abs(speeds.get(i)-speed)){
                error = Math.abs(speeds.get(i)-speed);
                index1 = i;
            }
        }
        //Searches for point on the other end of the range our distance falls into
        //Always makes index1 smaller and index2 larger
        if(speed>speeds.get(index1)){
            if(index1<speeds.size())
            index2 = index1+1;
            else
            return limits.get(index1);
        }
        else if(speed<speeds.get(index1)){
            if(index1>0){
            index2 = index1;
            index1=index2-1;
            }
            else
            return limits.get(0);
        }
        //calculates what percent of the way it is between the two distance bounds 
        //and then gets that percent of the way through the corresponding angle bounds
        percentage =Math.abs( (speed-speeds.get(index1))/(speeds.get(index2)-speeds.get(index1)));
        limit = limits.get(index1)+((limits.get(index2)-limits.get(index1))*percentage);
        return limit;
    }
 
}

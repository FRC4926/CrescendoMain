package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;

public class LookUpTableShooterAngles {
    ArrayList<Integer> distances = new ArrayList<Integer>();    
    ArrayList<Integer> angles = new ArrayList<Integer>();    

    public LookUpTableShooterAngles(){
        distances.add(2);
        angles.add(10);
        distances.add(4);
        angles.add(15);
        distances.add(6);
        angles.add(5);
        distances.add(8);
        angles.add(3);
    }

    public double lookUpAngle(double distance){
        //Variable Initilization
        double angle = 0;
        double error = Math.abs(distances.get(0)-distance);
        int index1 =0;
        int index2 = 0;
        double percentage =0;
        //Searches for point closest to our distance
        for(int i =0; i<distances.size(); i++){
            if(error>Math.abs(distances.get(i)-distance)){
                error = Math.abs(distances.get(i)-distance);
                index1 = i;
            }
        }
        //Searches for point on the other end of the range our distance falls into
        //Always makes index1 smaller and index2 larger
        if(distance>distances.get(index1)){
            if(index1<distances.size())
            index2 = index1+1;
            else
            return angles.get(index1);
        }
        else if(distance<distances.get(index1)){
            if(index1>0){
            index2 = index1;
            index1=index2-1;
            }
            else
            return angles.get(0);
        }
        //calculates what percent of the way it is between the two distance bounds 
        //and then gets that percent of the way through the corresponding angle bounds
        percentage =Math.abs( (distance-distances.get(index1))/(distances.get(index2)-distances.get(index1)));
        angle = angles.get(index1)+((angles.get(index2)-angles.get(index1))*percentage);
        return angle;
    }
   
}

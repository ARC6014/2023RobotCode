// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6014;

/** Add your docs here. */
public class SuperStructureState {
    private double height;
    private double length;
    private double degree;
    private boolean isRobotSide;

    private double[] kTolerances = {
        1,
        0.5,
        3
    };

    public SuperStructureState(double height, double length, double degree){
        this.height = height;
        this.length = length;
        this.degree = degree;
        if(degree > -5){
            this.isRobotSide = true;
        }else{
            isRobotSide = false;
        }
    }

    public SuperStructureState(){
        this(0,0,0);
    }

    /**
     * Getters
     * 
     */

    public double getHeight(){
        return this.height;
    }

    public double getLength(){
        return this.length;
    }

    public double getDegree(){
        return this.degree;
    }

    /**
     * Setters
     * 
     */
    
     public void setHeight(double height){
        this.height = height;
    }   
    
    public void setLength(double length){
        this.length = length;
    }   

    public void setDegree(double degree){
        this.degree = degree;
    }   

    //Calc

    public double getAbsoluteHeight(){
       return this.height - (this.length * Math.cos(Math.toRadians(this.degree)));
    }

    public boolean isInSameSide(SuperStructureState other){
        return other.isRobotSide == this.isRobotSide;
    }

    public boolean isAtDesiredState(SuperStructureState otherState){
        double[] distances = {
            this.length - otherState.length,
            this.height - otherState.height,
            this.degree - otherState.degree
        };

        for (int i = 0; i <distances.length; i++){
            if(Math.abs(distances[i]) >= kTolerances[i]){ 
                return false;
            }
        }

        return true;

    }

}

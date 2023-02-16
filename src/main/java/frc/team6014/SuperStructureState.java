// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6014;

/** Add your docs here. */
public class SuperStructureState {
    private double height;
    private double length;
    private double degree;

    public SuperStructureState(double height, double length, double degree){
        this.height = height;
        this.length = length;
        this.degree = degree;
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

    //Ğ

}

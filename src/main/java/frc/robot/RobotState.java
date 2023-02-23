package frc.robot;

import frc.team6014.SuperStructureState;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class RobotState implements Loggable {

    private static RobotState mInstance;

    public static RobotState getInstance(){
        if(mInstance == null){
            mInstance = new RobotState();
        }
        
        return mInstance;
    }

    @Log.ToString(name = "Robot Mode")
    private robotState state = robotState.cubeMode;

    private SuperStructureState currenStructureState = new SuperStructureState();

    private enum robotState {
        cubeMode(0), coneMode(1);

        private int numVal;

        robotState(int numVal) {
            this.numVal = numVal;
        }

        public int getNumVal() {
            return numVal;
        }

        public robotState setMode(int numVal) {
            switch (numVal) {
                case 0:
                    numVal = 0;
                    return cubeMode;
                case 1:
                    numVal = 1;
                    return coneMode;
                default:
                    numVal = 0;
                    return cubeMode;
            }
        }
    }

    public void setMode(int numVal) {
        state = state.setMode(numVal);
    }

    public void setMode(robotState state) {
        state = state.setMode(state.getNumVal());
    }

    public robotState getMode() {
        return state;
    }

    public int getModeNum() {
        return state.getNumVal();
    }

    public void updateHeight(double height){
        currenStructureState.setHeight(height);
    }

    public void updateLength(double length){
        currenStructureState.setLength(length);
    }

    public void updateDegree(double degree){
        currenStructureState.setDegree(degree);
    }

    public SuperStructureState getCurrentSuperStructureState(){
        return currenStructureState;
    }

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

enum Actions {
    StrafeDistanceMoveAHDP,RotateDistanceMoveHDP,StrafePowerMoveAP,RotatePowerMoveHP,StrafePointMoveHPXY,RotatePointMovePXY,CurvePointMovePXY,AutoPointMovePXY,AngleRotateAP,HeadingRotateHP,PowerRotateP
}

public class MecanumWheelDriverV2 implements Runnable{
    
    RobotHardware H;
    List<ActionData> actionsList = new ArrayList<>();
    List<Integer> actionRemoveList = new ArrayList<>();
    int listPriority;
    double[] wheelPower = new double[4];
    double[] wheelTarget = new double[4];
    
    MecanumWheelDriverV2(RobotHardware H) {
        
        this.H = H;
        
    }
    
    public void run() {
    
    }
    
    public void setWheelPower() {
    
        // create wheel power values by adding all of the actions in the action list
        for (ActionData action : actionsList) {
        
            Action(action, action.param);
        
        }
        for (int i = actionRemoveList.size()-1; i >= 0; i--) actionsList.remove((int)actionRemoveList.get(i));
        actionRemoveList.clear();
        
        // find the max wheel power for scaling
        double maxWheelPower = Math.max(Math.max(Math.abs(wheelPower[0]), Math.abs(wheelPower[1])), Math.max(Math.abs(wheelPower[2]), Math.abs(wheelPower[3])));
    
        // scale down the power if any of the wheels is over 1 power
        if (maxWheelPower > 1) {
            for (int i = 0; i <= 3; i ++) {
                wheelPower[i] /= maxWheelPower;
            }
        }
    
        // assign all of the power values to the motors and reset wheelPower
        for (int i = 0; i <= 3; i ++) {
            H.driveMotor[i].setPower(wheelPower[i]);
            wheelPower[i] = 0;
        }
        
        // assign all target positions to motors and reset
        if (H.driveMotor[0].getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            for (int i = 0; i <= 3; i++) {
                H.driveMotor[i].setTargetPosition(H.driveMotor[i].getCurrentPosition() + (int)wheelTarget[i]);
                wheelTarget[i] = 0;
            }
        }
    
    }
    
    private boolean priorityCheck(int priority) {
        
        // set priority to current if there are no items in action list
        if (actionsList.isEmpty()) listPriority = priority;
    
        // return false if the current actions are a higher priority
        if (priority > listPriority) {
            return false;
        }
    
        // clear all lower priority actions if current action is higher priority
        if (priority < listPriority) {
            actionsList.clear(); // remove all actions of lower priority
            listPriority = priority; // set the new priority of actions on the action list
        }
        
        return true;
        
    }
    
    double findDegOffset(double DegCurrent, double TargetDeg) {
        
        /**DegCurrent, the current degree of the robot value between 0 and 360
         * TargetDeg, the degree with which to find the offset
         * Finds the angle between current degree and the target degree
         * returns a value between -180 and 180
         * output will be negative if the target degree is right, positive if on the left
         *     x
         * -90   90
         *    180
         */
        
        double offset = TargetDeg - DegCurrent;
        if (offset > 180) {
            offset -= 360;
        } else if (offset < -180) {
            offset += 360;
        }
        return offset;
    }
    
    private void Action(ActionData action, double[] param) {
        
        // for the given action and parameters find required wheel powers to reach the destination
        // and add them to the wheel power array
        switch (action.action) {
            case StrafeDistanceMoveAHDP:
                break;
            case RotateDistanceMoveHDP:
                break;
            case StrafePowerMoveAP:
                
                // assign variables
                param[0] = Math.toRadians(param[0] + 45);
                double cosAngle = Math.cos(param[0]);
                double sinAngle = Math.sin(param[0]);
                param[1] = Range.clip(param[1], 0, 1);
    
                // scale the motor's power so that at least one of them is equal to 1
                if (Math.abs(cosAngle) > Math.abs(sinAngle)) {
                    sinAngle /= Math.abs(cosAngle);
                    cosAngle = Math.signum(cosAngle);
                } else {
                    cosAngle /= Math.abs(sinAngle);
                    sinAngle = Math.signum(sinAngle);
                }
    
                // set all wheel powers
                wheelPower[0] += cosAngle * param[1];
                wheelPower[1] += sinAngle * param[1];
                wheelPower[2] += sinAngle * param[1];
                wheelPower[3] += cosAngle * param[1];
    
                //actionsList.remove(action); // removes this action from action list
                actionRemoveList.add(actionsList.indexOf(action));
    
                break;
            case RotatePowerMoveHP:
                break;
            case StrafePointMoveHPXY:
                break;
            case RotatePointMovePXY:
                break;
            case CurvePointMovePXY:
                break;
            case AutoPointMovePXY:
                break;
            case AngleRotateAP:
                break;
            case HeadingRotateHP:
    
                // assign variables
                param[1] = Range.clip(param[1], 0, 1);
    
                // use different methods of rotating depending on motor runMode
                if (H.driveMotor[0].getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
    
                    final double clicks_per_degree = 9.2222222;
                    final double offset = findDegOffset(H.heading, param[0]) * clicks_per_degree;
    
                    // set all wheel powers and targets
                    for (int i = 0; i <= 1; i++) {
                        wheelPower[2*i] += param[1];
                        wheelPower[2*i+1] -= param[1];
                        wheelTarget[2*i] += offset;
                        wheelTarget[2*i+1] -= offset;
                    }
        
                } else {
    
                    double offset = findDegOffset(H.heading, param[0]);
                    param[1] = Math.signum(offset)* param[1] ;//Range.clip( Math.exp(0.1 * Math.abs(offset))-1, 0.15, param[1]);
                    
                    // remove from list if target has been reached
                    if (Math.abs(offset) > 2.5) {
                        //actionsList.remove(action);
                        actionRemoveList.add(actionsList.indexOf(action));
                        break;
                    }
                    
                    // set all wheel powers
                    for (int i = 0; i <= 1; i++) {
                        wheelPower[2*i] += param[1];
                        wheelPower[2*i+1] -= param[1];
                    }
        
                }
    
                break;
            case PowerRotateP:
                
                // assign variables
                param[0] = Range.clip(param[0], -1, 1);
    
                // set all wheel powers
                for (int i = 0; i <= 1; i++) {
                    wheelPower[2*i] += param[0];
                    wheelPower[2*i+1] -= param[0];
                }
                
                //actionsList.remove(action); // removes this action from action list
                actionRemoveList.add(actionsList.indexOf(action));
                break;
            default:
    
        }
        
    }
    
    ////////    Individual functions for easy auto-fill when programming    ////////
    
    boolean StrafeDistanceMove(double angle, double heading, double distance, double power, int priority) {
    
        // don't execute action and return false if the current actions are a higher priority
        if (!priorityCheck(priority)) return false;
    
        // add the action to the action list
        actionsList.add(new ActionData(Actions.StrafeDistanceMoveAHDP, new double[]{angle, heading, power, distance}));
        return true; // action was added to list
    
    }
    
    boolean RotateDistanceMove(double heading, double distance, double power, int priority) {
    
        // don't execute action and return false if the current actions are a higher priority
        if (!priorityCheck(priority)) return false;
    
        // add the action to the action list
        actionsList.add(new ActionData(Actions.RotateDistanceMoveHDP, new double[]{heading, power, distance}));
        return true; // action was added to list
        
    }
    
    boolean StrafePowerMove(double angle, double power, int priority) {
    
        // don't execute action and return false if the current actions are a higher priority
        if (!priorityCheck(priority)) return false;
    
        // add the action to the action list
        actionsList.add(new ActionData(Actions.StrafePowerMoveAP, new double[]{angle, power}));
        return true; // action was added to list
        
    }
    
    boolean RotatePowerMove(double heading, double power, int priority) {
    
        // don't execute action and return false if the current actions are a higher priority
        if (!priorityCheck(priority)) return false;
    
        // add the action to the action list
        actionsList.add(new ActionData(Actions.RotatePowerMoveHP, new double[]{heading, power}));
        return true; // action was added to list
        
    }
    
    boolean StrafePointMove(double heading, double power, double x, double y, int priority) {
    
        // don't execute action and return false if the current actions are a higher priority
        if (!priorityCheck(priority)) return false;
    
        // add the action to the action list
        actionsList.add(new ActionData(Actions.StrafePointMoveHPXY, new double[]{heading, power, x, y}));
        return true; // action was added to list
        
    }
    
    boolean RotatePointMove(double power, double x, double y, int priority) {
    
        // don't execute action and return false if the current actions are a higher priority
        if (!priorityCheck(priority)) return false;
    
        // add the action to the action list
        actionsList.add(new ActionData(Actions.RotatePowerMoveHP, new double[]{power, x, y}));
        return true; // action was added to list
        
    }
    
    boolean CurvePointMove(double power, double x, double y, int priority) {
    
        // don't execute action and return false if the current actions are a higher priority
        if (!priorityCheck(priority)) return false;
    
        // add the action to the action list
        actionsList.add(new ActionData(Actions.CurvePointMovePXY, new double[]{power, x, y}));
        return true; // action was added to list
        
    }
    
    boolean AutoPointMove(double power, double x, double y, int priority) {
    
        // don't execute action and return false if the current actions are a higher priority
        if (!priorityCheck(priority)) return false;
    
        // add the action to the action list
        actionsList.add(new ActionData(Actions.AutoPointMovePXY, new double[]{power, x, y}));
        return true; // action was added to list
        
    }
    
    boolean AngleRotate(double angle, double power, int priority) {
    
        // don't execute action and return false if the current actions are a higher priority
        if (!priorityCheck(priority)) return false;
    
        // add the action to the action list
        actionsList.add(new ActionData(Actions.AngleRotateAP, new double[]{angle, power}));
        return true; // action was added to list
        
    }
    
    boolean HeadingRotate(double heading, double power, int priority) {
    
        // don't execute action and return false if the current actions are a higher priority
        if (!priorityCheck(priority)) return false;
    
        // add the action to the action list
        actionsList.add(new ActionData(Actions.HeadingRotateHP, new double[]{heading, power}));
        return true; // action was added to list
    
    }
    
    boolean PowerRotate(double power, int priority) {
    
        // don't execute action and return false if the current actions are a higher priority
        if (!priorityCheck(priority)) return false;
    
        // add the action to the action list
        actionsList.add(new ActionData(Actions.PowerRotateP, new double[]{power}));
        return true; // action was added to list
    
    }
    
}

class ActionData {
    
    Actions action;
    double[] param;
    
    ActionData() {
    
    }
    
    ActionData(Actions action, double[] param) {
    
        this.action = action;
        this.param = param;
    
    }
    
}
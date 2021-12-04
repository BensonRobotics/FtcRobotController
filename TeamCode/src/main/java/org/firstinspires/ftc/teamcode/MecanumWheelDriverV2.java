package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;

enum Actions {
    StrafeDistanceMoveHDP,RotateDistanceMoveHDP,StrafePowerMoveAP,RotatePowerMoveHP,StrafePointMovePXY,RotatePointMovePXY,CurvePointMovePXY,AutoPointMovePXY,AngleRotateAP,HeadingRotateHP,PowerRotateP
}

public class MecanumWheelDriverV2 implements Runnable{
    
    RobotHardware H;
    List<ActionData> actionsList = new ArrayList<>();
    List<Integer> actionRemoveList = new ArrayList<>();
    int listPriority;
    double[] wheelPower = new double[4];
    int[] wheelTarget = new int[4];
    boolean runToPosition = false;
    double maxWheelPower;
    
    MecanumWheelDriverV2(RobotHardware H) {
        
        this.H = H;
        
    }
    
    public void run() {
    
    
        // set all wheel target positions
        /*H.driveMotor[0].setTargetPosition(initialPosition[0] + wheelTarget[0]);
        H.driveMotor[1].setTargetPosition(initialPosition[1] + wheelTarget[1]);
        H.driveMotor[2].setTargetPosition(initialPosition[2] + wheelTarget[1]);
        H.driveMotor[3].setTargetPosition(initialPosition[3] + wheelTarget[0]);*/
    
    }
    
    public void useRunToPosition(boolean use) {
        
        for (int i = 0; i < 4; i++) {
            H.driveMotor[i].setPower(0);
            if (use) {
                H.driveMotor[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else {
                H.driveMotor[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
        
        runToPosition = use;
        
    }
    
    public void setWheelPower() {
    
        // create wheel power values by adding all of the actions in the action list
        for (ActionData action : actionsList) {
        
            Action(action, action.param);
        
        }
        for (int i = actionRemoveList.size()-1; i >= 0; i--) actionsList.remove((int)actionRemoveList.get(i));
        actionRemoveList.clear();
        
        // find the max wheel power for scaling
        maxWheelPower = Math.max(Math.max(Math.abs(wheelPower[0]), Math.abs(wheelPower[1])), Math.max(Math.abs(wheelPower[2]), Math.abs(wheelPower[3])));
    
        // scale down the power if any of the wheels is over 1 power
        if (maxWheelPower > 1) {
            for (int i = 0; i <= 3; i ++) {
                wheelPower[i] /= maxWheelPower;
            }
        }
        
        for (int i = 0; i <= 3; i ++) {
            // assign all target positions to motors
            if (runToPosition) {
                H.driveMotor[i].setTargetPosition(wheelTarget[i]);
                H.driveMotor[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
    
            // assign all of the power values to the motors and reset wheelPower
            H.driveMotor[i].setPower(wheelPower[i]);
            wheelPower[i] = 0;
            
        }
    
    }
    
    void waitForMoveDone(){
        
        while ((H.driveMotor[0].isBusy() || H.driveMotor[1].isBusy() || H.driveMotor[2].isBusy() || H.driveMotor[3].isBusy()) && H.opMode.isStopRequested()) {
            H.opMode.idle();
        }
        if (H.opMode.isStopRequested()) {
            for (DcMotor motor : H.driveMotor) {
                motor.setTargetPosition(motor.getCurrentPosition());
                motor.setPower(0);
            }
        }
        
        H.driveMotor[0].  setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        H.driveMotor[1]. setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        H.driveMotor[2].   setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        H.driveMotor[3].  setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            case StrafeDistanceMoveHDP:
                StrafeDistanceMoveHDP(param);
                actionRemoveList.add(actionsList.indexOf(action));
                break;
            case RotateDistanceMoveHDP:
                RotateDistanceMoveHDP(param);
                actionRemoveList.add(actionsList.indexOf(action));
                break;
            case StrafePowerMoveAP:
                StrafePowerMoveAP(param);
                // removes this action from action list
                actionRemoveList.add(actionsList.indexOf(action));
                break;
            case RotatePowerMoveHP:
                RotatePowerMoveHP(param);
                actionRemoveList.add(actionsList.indexOf(action));
                break;
            case StrafePointMovePXY:
                StrafePointMovePXY(param);
                actionRemoveList.add(actionsList.indexOf(action));
                break;
            case RotatePointMovePXY:
                RotatePointMovePXY(param);
                actionRemoveList.add(actionsList.indexOf(action));
                break;
            case CurvePointMovePXY:
                CurvePointMovePXY(param);
                actionRemoveList.add(actionsList.indexOf(action));
                break;
            case AutoPointMovePXY:
                AutoPointMovePXY(param);
                actionRemoveList.add(actionsList.indexOf(action));
                break;
            case AngleRotateAP:
                AngleRotateAP(param);
                actionRemoveList.add(actionsList.indexOf(action));
                break;
            case HeadingRotateHP:
                HeadingRotateHP(action, param);
    
                break;
            case PowerRotateP:
                PowerRotateP(param);
                //actionsList.remove(action); // removes this action from action list
                actionRemoveList.add(actionsList.indexOf(action));
                break;
            default:
    
        }
        
    }
    
    private void ActionMaintain(ActionData action, double[] param) {
        
        // update the powers or distances to stay on track, used on move-to-position moves
        switch (action.action) {
            case StrafeDistanceMoveHDP:
                StrafeDistanceMoveMaintainHDP(param);
                actionRemoveList.add(actionsList.indexOf(action));
                break;
            case RotateDistanceMoveHDP:
                RotateDistanceMoveMaintainHDP(param);
                actionRemoveList.add(actionsList.indexOf(action));
                break;
            case StrafePowerMoveAP:
                StrafePowerMoveMaintainAP(param);
                // removes this action from action list
                actionRemoveList.add(actionsList.indexOf(action));
                break;
            case RotatePowerMoveHP:
                RotatePowerMoveMaintainHP(param);
                actionRemoveList.add(actionsList.indexOf(action));
                break;
            case StrafePointMovePXY:
                StrafePointMoveMaintainPXY(param);
                actionRemoveList.add(actionsList.indexOf(action));
                break;
            case RotatePointMovePXY:
                RotatePointMoveMaintainPXY(param);
                actionRemoveList.add(actionsList.indexOf(action));
                break;
            case CurvePointMovePXY:
                CurvePointMoveMaintainPXY(param);
                actionRemoveList.add(actionsList.indexOf(action));
                break;
            case AutoPointMovePXY:
                AutoPointMoveMaintainPXY(param);
                actionRemoveList.add(actionsList.indexOf(action));
                break;
            case AngleRotateAP:
                AngleRotateMaintainAP(param);
                actionRemoveList.add(actionsList.indexOf(action));
                break;
            case HeadingRotateHP:
                HeadingRotateMaintainHP(param);
                break;
            case PowerRotateP:
                PowerRotateMaintainP(param);
                // removes this action from action list
                actionRemoveList.add(actionsList.indexOf(action));
                break;
            default:
            
        }
        
    }
    
    ///// sets initial power and distance values /////
    private void StrafeDistanceMoveHDP(double[] param) {
        // assign variables
    
        param[0] = Math.toRadians(param[0] + 45);
        double cosAngle = Math.cos(param[0]);
        double sinAngle = Math.sin(param[0]);
        param[2] = Range.clip(param[2], 0, 1);
        
        int[] initialPosition = new int[4];
        /*for (int i = 0; i < 4; i++) {
            initialPosition[i] = H.driveMotor[i].getCurrentPosition();
            H.driveMotor[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }*/
    
        // scale the motor's power so that at least one of them is equal to 1
        if (Math.abs(cosAngle) > Math.abs(sinAngle)) {
            sinAngle /= Math.abs(cosAngle);
            cosAngle = Math.signum(cosAngle);
        } else {
            cosAngle /= Math.abs(sinAngle);
            sinAngle = Math.signum(sinAngle);
        }
        
        // set all wheel target positions
        wheelTarget[0] += (int)(cosAngle * param[1] * H.COUNTS_PER_INCH);
        wheelTarget[1] += (int)(sinAngle * param[1] * H.COUNTS_PER_INCH);
        wheelTarget[2] += (int)(sinAngle * param[1] * H.COUNTS_PER_INCH);
        wheelTarget[3] += (int)(cosAngle * param[1] * H.COUNTS_PER_INCH);
    
        // set all wheel powers
        wheelPower[0] += cosAngle * param[1];
        wheelPower[1] += sinAngle * param[1];
        wheelPower[2] += sinAngle * param[1];
        wheelPower[3] += cosAngle * param[1];
    }
    
    private void RotateDistanceMoveHDP(double[] param) {
    
    }
    
    private void StrafePowerMoveAP(double[] param) {
    
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
    
    }
    
    private void RotatePowerMoveHP(double[] param) {
    
    }
    
    private void StrafePointMovePXY(double[] param) {
    
    }
    
    private void RotatePointMovePXY(double[] param) {
    
    }
    
    private void CurvePointMovePXY(double[] param) {
    
    }
    
    private void AutoPointMovePXY(double[] param) {
    
    }
    
    private void AngleRotateAP(double[] param) {
    
    }
    
    private void HeadingRotateHP(ActionData action, double[] param) {
        // assign variables
        param[1] = Range.clip(param[1], 0, 1);
        double offset;
    
        // use different methods of rotating depending on motor runMode
        if (runToPosition) {
        
            final double clicks_per_degree = 9.2222222;
             offset = findDegOffset(H.heading, param[0]) * clicks_per_degree;
        
            // set all wheel powers and targets
            for (int i = 0; i <= 1; i++) {
                wheelPower[2*i] += param[1];
                wheelPower[2*i+1] -= param[1];
                wheelTarget[2*i] += offset;
                wheelTarget[2*i+1] -= offset;
            }
            actionRemoveList.add(actionsList.indexOf(action));
        } else {
            offset = findDegOffset(H.heading, param[0]);
            // ramp down when near target heading
            double power = Math.signum(offset) * Range.clip( (Math.exp(0.1 * Math.abs(offset))-1)/(param[1] * Math.sqrt(Math.abs(param[2]))), 0.075, param[1]);
            // remove from list if target has been reached
            if (Math.abs(offset) < 1.5) {
                actionRemoveList.add(actionsList.indexOf(action));
            }
        
            // set all wheel powers
            wheelPower[0] -= power;
            wheelPower[1] += power;
            wheelPower[2] -= power;
            wheelPower[3] += power;
        
        }
    }
    
    private void PowerRotateP(double[] param) {
        // assign variables
        param[0] = Range.clip(param[0], -1, 1);
    
        // set all wheel powers
        for (int i = 0; i <= 1; i++) {
            wheelPower[2*i] += param[0];
            wheelPower[2*i+1] -= param[0];
        }
    
    }
    
    
    ///// adjusts values while moving to stay on course /////
    
    private void StrafeDistanceMoveMaintainHDP(double[] param) {
    
    }
    
    private void RotateDistanceMoveMaintainHDP(double[] param) {
    
    }
    
    private void StrafePowerMoveMaintainAP(double[] param) {
    
    }
    
    private void RotatePowerMoveMaintainHP(double[] param) {
    
    }
    
    private void StrafePointMoveMaintainPXY(double[] param) {
    
    }
    
    private void RotatePointMoveMaintainPXY(double[] param) {
    
    }
    
    private void CurvePointMoveMaintainPXY(double[] param) {
    
    }
    
    private void AutoPointMoveMaintainPXY(double[] param) {
    
    }
    
    private void AngleRotateMaintainAP(double[] param) {
    
    }
    
    private void HeadingRotateMaintainHP(double[] param) {
    
    }
    
    private void PowerRotateMaintainP(double[] param) {
    
    }
    
    ////////    Individual functions for easy auto-fill when programming    ////////
    
    boolean addAction(Actions actionName, double param1, double param2, double param3, int priority) {
        
        // don't execute action and return false if the current actions are a higher priority
        if (!priorityCheck(priority)) return false;
    
        // add the action to the action list
        actionsList.add(new ActionData(actionName, new double[]{param1, param2, param3}));
        return true; // action was added to list
        
    }
    
    boolean StrafeDistanceMove(double angle, double distance, double power, int priority) {
    
        // don't execute action and return false if the current actions are a higher priority
        if (!priorityCheck(priority)) return false;
    
        // add the action to the action list
        actionsList.add(new ActionData(Actions.StrafeDistanceMoveHDP, new double[]{angle, distance, power}));
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
    
    boolean StrafePointMove(double power, double x, double y, int priority) {
    
        // don't execute action and return false if the current actions are a higher priority
        if (!priorityCheck(priority)) return false;
    
        // add the action to the action list
        actionsList.add(new ActionData(Actions.StrafePointMovePXY, new double[]{power, x, y}));
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
    
        //find abs of initial offset value
        
        // add the action to the action list
        actionsList.add(new ActionData(Actions.HeadingRotateHP, new double[]{heading, power, findDegOffset(H.heading, heading)}));
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
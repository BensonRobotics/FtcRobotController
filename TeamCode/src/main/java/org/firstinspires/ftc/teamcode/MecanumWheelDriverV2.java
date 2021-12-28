package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

enum Actions {
    StrafeDistanceMoveHDP,RotateDistanceMoveHDP,StrafePowerMoveAP,RotatePowerMoveHP,StrafePointMovePXY,RotatePointMovePXY,CurvePointMovePXY,AutoPointMovePXY,AngleRotateAP,HeadingRotateHP,PowerRotateP
}

public class MecanumWheelDriverV2 implements Runnable{
    
    RobotHardware H;
    List<ActionData> initializationList = new ArrayList<>();
    List<Integer> initializationRemoveList = new ArrayList<>();
    List<ActionData> maintainList = new ArrayList<>();
    List<Integer> maintainRemoveList = new ArrayList<>();
    int listPriority;
    boolean runToPosition = false;
    int[] lastLoopPosition = {0,0,0,0};
    double maxWheelPower;
    
    final long TIMEOUT = 3500;
    final double DISTANCE_MULTIPLIER_LOWER = 1.5;
    final double DISTANCE_MULTIPLIER_UPPER = 1.825;
    final int MOVEMENT_TOLERANCE = 10;
    final double ROTATE_TOLERANCE = 1;
    
    MecanumWheelDriverV2(RobotHardware H) {
        
        this.H = H;
        
    }
    
    public void run() {
    
        // run maintain actions
        for (ActionData action : maintainList) {
        
            ActionMaintain(action);
            
            // remove action if it has timed out
            if (action.startTime + TIMEOUT > H.runtime.now(TimeUnit.MILLISECONDS)) {
                maintainRemoveList.add(maintainList.indexOf(action));
            }
        
        }
        
        // remove all finished actions
        for (int i = maintainRemoveList.size()-1; i >= 0; i--) maintainList.remove((int)maintainRemoveList.get(i));
        maintainRemoveList.clear();
        
        // sum up all of the actions' wheel powers for averaging and figuring power portions
        double[] wheelPower = {0,0,0,0};
        double[] totalAbsPower = {0,0,0,0};
        for (ActionData action : maintainList) {
            for (int i = 0; i < 4; i++) {
                wheelPower[i] += action.wheelPower[i];
                totalAbsPower[i] += Math.abs(action.wheelPower[i]);
            }
        }
        
        // set the power for each wheel
        setWheelPower(wheelPower);
        
        // find how much each action effects the power
        calculatePowerPortions(totalAbsPower);
        
        // read how much each wheel has moved since last loop
        int[] clickDelta = {0,0,0,0};
        for (int i = 0; i < 4; i++) {
            clickDelta[i] = H.driveMotor[i].getCurrentPosition() - lastLoopPosition[i];
            lastLoopPosition[i] = H.driveMotor[i].getCurrentPosition();
        }
        
        // using power portions, find how much farther each action needs to move to be finished
        updateActionTarget(clickDelta);
    
    }
    
    public void startActions() {
    
        // create wheel power values by running all of the actions in the action list
        for (ActionData action : initializationList) {
        
            Action(action);
        
        }
        
        // sum up all of the actions' wheel powers for averaging
        double[] wheelPower = {0,0,0,0};
        for (ActionData action : initializationList) {
            for (int i = 0; i < 4; i++) {
                wheelPower[i] += action.wheelPower[i];
            }
        }
        // set the power for all the wheels
        setWheelPower(wheelPower);
        
        for (int i = initializationRemoveList.size()-1; i >= 0; i--) initializationList.remove((int) initializationRemoveList.get(i));
        initializationRemoveList.clear();
        
        // transfer all unfinished actions to be maintained
        maintainList.addAll(initializationList);
        
        // remove all actions from the initialization list
        initializationList.clear();
    }
    
    private void setWheelPower(double[] wheelPower) {
    
        // find the max wheel power for scaling
        maxWheelPower = Math.max(Math.max(Math.abs(wheelPower[0]), Math.abs(wheelPower[1])), Math.max(Math.abs(wheelPower[2]), Math.abs(wheelPower[3])));
    
        // scale down the power if any of the wheels are over 1 power
        if (maxWheelPower > 1) {
            for (int i = 0; i <= 3; i ++) {
                wheelPower[i] /= maxWheelPower;
            }
        }
    
        for (int i = 0; i <= 3; i ++) {
        
            // assign all of the power values to the motors and reset wheelPower
            H.driveMotor[i].setPower(wheelPower[i]);
            wheelPower[i] = 0;
        
        }
    }
    
    private void calculatePowerPortions(double[] totalAbsPower) {
    
        for (ActionData action : maintainList) {
            
            // find the amount each action effects the final power
            for (int i = 0; i < 4; i++) {
                action.powerPortion[i] = Math.abs(action.wheelPower[i])/totalAbsPower[i];
            }
            
        }
    
    }
    
    private void updateActionTarget(int[] clickDelta) {
        
        int totalClicks = 0;
        double totalPower = 0;
        
        for (ActionData action : maintainList) {
    
            // get the distance that the specified action is responsible for
            for (int i = 0; i < 4; i++) {
                totalClicks += (int)action.powerPortion[i] * clickDelta[i];
                totalPower += action.wheelPower[i];
            }
            
            // remove distance from wheel target after adjusting to be aligned with wheel powers
            for (int i = 0; i < 4; i++) {
                action.wheelTarget[i] -= (int)(totalClicks * action.wheelPower[i]/totalPower);
            }
            
        }
    
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
    
    void waitForMoveDone(){
        
        while ((H.driveMotor[0].isBusy() || H.driveMotor[1].isBusy() || H.driveMotor[2].isBusy() || H.driveMotor[3].isBusy()) && !H.opMode.isStopRequested()) {
            //Log.d(TAG, "MotorPower 0: " + H.driveMotor[0].getPower() + " 1: " + H.driveMotor[1].getPower() + " 2: " + H.driveMotor[2].getPower() + " 3: " + H.driveMotor[3].getPower());
            //Log.d(TAG, "MotorTarget 0: " + H.driveMotor[0].getTargetPosition() + " 1: " + H.driveMotor[1].getTargetPosition() + " 2: " + H.driveMotor[2].getTargetPosition() + " 3: " + H.driveMotor[3].getTargetPosition());
            //Log.d(TAG, "MotorPos 0: " + H.driveMotor[0].getCurrentPosition() + " 1: " + H.driveMotor[1].getCurrentPosition() + " 2: " + H.driveMotor[2].getCurrentPosition() + " 3: " + H.driveMotor[3].getCurrentPosition());
        }
        if (H.opMode.isStopRequested()) {
            for (DcMotor motor : H.driveMotor) {
                motor.setTargetPosition(motor.getCurrentPosition());
                motor.setPower(0);
            }
        }
        
        for (int i = 0; i < 4; i++) {
    
            H.driveMotor[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            H.driveMotor[i].setPower(0);
            
        }
    }
    
    private boolean isLowPriority(int priority) {
        
        // set priority to current if there are no items in action list
        if (initializationList.isEmpty()) listPriority = priority;
    
        // return false if the current actions are a higher priority
        if (priority > listPriority) {
            return true;
        }
    
        // clear all lower priority actions if current action is higher priority
        if (priority < listPriority) {
            initializationList.clear(); // remove all actions of lower priority
            listPriority = priority; // set the new priority of actions on the action list
        }
        
        return false;
        
    }
    
    double findDegOffset(double DegCurrent, double TargetDeg) {
        
        /*DegCurrent, the current degree of the robot value between 0 and 360
         * TargetDeg, the degree with which to find the offset
         * Finds the angle between current degree and the target degree
         * returns a value between -180 and 180
         * output will be negative if the target degree is right, positive if on the left
         *     x
         *  90  -90
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
    
    private double adaptivePowerRamping(double offset, double power, double initialOffset) {
        
        return Range.clip((Math.exp(0.1 * Math.abs(offset))-1)/(power * Math.sqrt(Math.abs(initialOffset))), H.MINIMUM_MOTOR_POWER, power);
    
    }
    
    private void Action(ActionData action) {
        
        // for the given action and parameters find required wheel powers to reach the destination
        // and add them to the wheel power array
        switch (action.action) {
            case StrafeDistanceMoveHDP:
                StrafeDistanceMoveHDP(action);
                initializationRemoveList.add(initializationList.indexOf(action));
                break;
            case RotateDistanceMoveHDP:
                RotateDistanceMoveHDP(action);
                initializationRemoveList.add(initializationList.indexOf(action));
                break;
            case StrafePowerMoveAP:
                StrafePowerMoveAP(action);
                initializationRemoveList.add(initializationList.indexOf(action));
                break;
            case RotatePowerMoveHP:
                RotatePowerMoveHP(action);
                initializationRemoveList.add(initializationList.indexOf(action));
                break;
            case StrafePointMovePXY:
                StrafePointMovePXY(action);
                initializationRemoveList.add(initializationList.indexOf(action));
                break;
            case RotatePointMovePXY:
                RotatePointMovePXY(action);
                initializationRemoveList.add(initializationList.indexOf(action));
                break;
            case CurvePointMovePXY:
                CurvePointMovePXY(action);
                initializationRemoveList.add(initializationList.indexOf(action));
                break;
            case AutoPointMovePXY:
                AutoPointMovePXY(action);
                initializationRemoveList.add(initializationList.indexOf(action));
                break;
            case AngleRotateAP:
                AngleRotateAP(action);
                initializationRemoveList.add(initializationList.indexOf(action));
                break;
            case HeadingRotateHP:
                HeadingRotateHP(action);
    
                break;
            case PowerRotateP:
                PowerRotateP(action);
                //actionsList.remove(action); // removes this action from action list
                initializationRemoveList.add(initializationList.indexOf(action));
                break;
            default:
    
        }
        
    }
    
    private void ActionMaintain(ActionData action) {
        
        // update the powers or distances to stay on track, used on move-to-position moves
        switch (action.action) {
            case StrafeDistanceMoveHDP:
                StrafeDistanceMoveMaintainHDP(action);
                initializationRemoveList.add(initializationList.indexOf(action));
                break;
            case RotateDistanceMoveHDP:
                RotateDistanceMoveMaintainHDP(action);
                initializationRemoveList.add(initializationList.indexOf(action));
                break;
            case StrafePowerMoveAP:
                StrafePowerMoveMaintainAP(action);
                initializationRemoveList.add(initializationList.indexOf(action));
                break;
            case RotatePowerMoveHP:
                RotatePowerMoveMaintainHP(action);
                initializationRemoveList.add(initializationList.indexOf(action));
                break;
            case StrafePointMovePXY:
                StrafePointMoveMaintainPXY(action);
                initializationRemoveList.add(initializationList.indexOf(action));
                break;
            case RotatePointMovePXY:
                RotatePointMoveMaintainPXY(action);
                initializationRemoveList.add(initializationList.indexOf(action));
                break;
            case CurvePointMovePXY:
                CurvePointMoveMaintainPXY(action);
                initializationRemoveList.add(initializationList.indexOf(action));
                break;
            case AutoPointMovePXY:
                AutoPointMoveMaintainPXY(action);
                initializationRemoveList.add(initializationList.indexOf(action));
                break;
            case AngleRotateAP:
                AngleRotateAP(action);
                initializationRemoveList.add(initializationList.indexOf(action));
                break;
            case HeadingRotateHP:
                HeadingRotateHP(action);
                break;
            default:
            
        }
        
    }
    
    ///// sets initial power and distance values /////
    private void StrafeDistanceMoveHDP(ActionData action) {
        // assign variables
        
        // increase distance when moving sideways to counter mecanum wheel inconsistencies
        double distanceMultiplier = (DISTANCE_MULTIPLIER_LOWER - DISTANCE_MULTIPLIER_UPPER)*(Math.abs(Math.abs(action.param[0]/90) - 1)) + DISTANCE_MULTIPLIER_UPPER;
        
        double angle = Math.toRadians(action.param[0] + 45);
        double cosAngle = Math.cos(angle);
        double sinAngle = Math.sin(angle);
        double power = Range.clip(action.param[2], 0, 1);
        
        // set all wheel target positions
        action.initialWheelTarget[0] = (int)(cosAngle * action.param[1] * distanceMultiplier * H.COUNTS_PER_INCH);
        action.initialWheelTarget[1] = (int)(sinAngle * action.param[1] * distanceMultiplier * H.COUNTS_PER_INCH);
        action.initialWheelTarget[2] = (int)(sinAngle * action.param[1] * distanceMultiplier * H.COUNTS_PER_INCH);
        action.initialWheelTarget[3] = (int)(cosAngle * action.param[1] * distanceMultiplier * H.COUNTS_PER_INCH);
    
        action.wheelTarget = action.initialWheelTarget.clone();
    
        // scale the motor's power so that at least one of them is equal to 1
        if (Math.abs(cosAngle) > Math.abs(sinAngle)) {
            sinAngle /= Math.abs(cosAngle);
            cosAngle = Math.signum(cosAngle);
        } else {
            cosAngle /= Math.abs(sinAngle);
            sinAngle = Math.signum(sinAngle);
        }
        
        // set all wheel powers
        action.wheelPower[0] = cosAngle * power;
        action.wheelPower[1] = sinAngle * power;
        action.wheelPower[2] = sinAngle * power;
        action.wheelPower[3] = cosAngle * power;
    }
    
    private void RotateDistanceMoveHDP(ActionData action) {
    
    }
    
    private void StrafePowerMoveAP(ActionData action) {
    
        // assign variables
        double angle = Math.toRadians(action.param[0] + 45);
        double cosAngle = Math.cos(angle);
        double sinAngle = Math.sin(angle);
        double power = Range.clip(action.param[1], 0, 1);
    
        // scale the motor's power so that at least one of them is equal to 1
        if (Math.abs(cosAngle) > Math.abs(sinAngle)) {
            sinAngle /= Math.abs(cosAngle);
            cosAngle = Math.signum(cosAngle);
        } else {
            cosAngle /= Math.abs(sinAngle);
            sinAngle = Math.signum(sinAngle);
        }
        // set all wheel powers
        action.wheelPower[0] += cosAngle * power;
        action.wheelPower[1] += sinAngle * power;
        action.wheelPower[2] += sinAngle * power;
        action.wheelPower[3] += cosAngle * power;
    
    }
    
    private void RotatePowerMoveHP(ActionData action) {
    
    }
    
    private void StrafePointMovePXY(ActionData action) {
    
    }
    
    private void RotatePointMovePXY(ActionData action) {
    
    }
    
    private void CurvePointMovePXY(ActionData action) {
    
    }
    
    private void AutoPointMovePXY(ActionData action) {
    
    }
    
    private void AngleRotateAP(ActionData action) {
        // assign variables
        action.param[1] = Range.clip(action.param[1], 0, 1);
        
        double offset;
        offset = findDegOffset(H.heading - action.param[2], action.param[0]);
        // ramp down when near target heading
        double power = -Math.signum(offset) * adaptivePowerRamping(offset, action.param[1], findDegOffset(action.param[2], action.param[0]));
    
        // remove from list if target has been reached
        if (Math.abs(offset) < ROTATE_TOLERANCE) {
            initializationRemoveList.add(initializationList.indexOf(action));
            return;
        }
    
        // set all wheel powers
        action.wheelPower[0] = power;
        action.wheelPower[1] = -power;
        action.wheelPower[2] = power;
        action.wheelPower[3] = -power;
    }
    
    private void HeadingRotateHP(ActionData action) {
        // assign variables
        action.param[1] = Range.clip(action.param[1], 0, 1);
    
        double offset;
        offset = findDegOffset(H.heading, action.param[0]);
        // ramp down when near target heading
        double power = -Math.signum(offset) * adaptivePowerRamping(offset, action.param[1], action.param[2]);
    
        // remove from list if target has been reached
        if (Math.abs(offset) < ROTATE_TOLERANCE) {
            initializationRemoveList.add(initializationList.indexOf(action));
            return;
        }
    
        // set all wheel powers
        action.wheelPower[0] = power;
        action.wheelPower[1] = -power;
        action.wheelPower[2] = power;
        action.wheelPower[3] = -power;
        
    }
    
    private void PowerRotateP(ActionData action) {
        // assign variables
        double power = Range.clip(action.param[0], -1, 1);
    
        // set all wheel powers
        action.wheelPower[0] = power;
        action.wheelPower[1] = -power;
        action.wheelPower[2] = power;
        action.wheelPower[3] = -power;
    
    }
    
    
    ///// adjusts values while moving to stay on course /////
    
    private void StrafeDistanceMoveMaintainHDP(ActionData action) {
        // assign variables
    
        // increase distance when moving sideways to counter mecanum wheel inconsistencies
        double distanceMultiplier = (DISTANCE_MULTIPLIER_LOWER - DISTANCE_MULTIPLIER_UPPER)*(Math.abs(Math.abs(action.param[0]/90) - 1)) + DISTANCE_MULTIPLIER_UPPER;
    
        double angle = Math.toRadians(action.param[0] + 45);
        double cosAngle = Math.cos(angle);
        double sinAngle = Math.sin(angle);
        double power = Range.clip(action.param[2], 0, 1);
    
        // set all wheel target positions
        action.wheelTarget[0] += (int)(cosAngle * action.param[1] * distanceMultiplier * H.COUNTS_PER_INCH) - action.initialWheelTarget[0];
        action.wheelTarget[1] += (int)(sinAngle * action.param[1] * distanceMultiplier * H.COUNTS_PER_INCH) - action.initialWheelTarget[1];
        action.wheelTarget[2] += (int)(sinAngle * action.param[1] * distanceMultiplier * H.COUNTS_PER_INCH) - action.initialWheelTarget[2];
        action.wheelTarget[3] += (int)(cosAngle * action.param[1] * distanceMultiplier * H.COUNTS_PER_INCH) - action.initialWheelTarget[3];
    
        // remove from list if target has been reached
        if (Math.abs(action.wheelTarget[0]) < MOVEMENT_TOLERANCE && Math.abs(action.wheelTarget[1]) < MOVEMENT_TOLERANCE && Math.abs(action.wheelTarget[2]) < MOVEMENT_TOLERANCE && Math.abs(action.wheelTarget[3]) < MOVEMENT_TOLERANCE) {
            initializationRemoveList.add(initializationList.indexOf(action));
            return;
        }
    
        // scale the motor's power so that at least one of them is equal to 1
        if (Math.abs(cosAngle) > Math.abs(sinAngle)) {
            sinAngle /= Math.abs(cosAngle);
            cosAngle = Math.signum(cosAngle);
        } else {
            cosAngle /= Math.abs(sinAngle);
            sinAngle = Math.signum(sinAngle);
        }
    
        // set all wheel powers
        action.wheelPower[0] = cosAngle * power;
        action.wheelPower[1] = sinAngle * power;
        action.wheelPower[2] = sinAngle * power;
        action.wheelPower[3] = cosAngle * power;
        
        // ramp down power when near destination
        double totalPower = 0;
        for (int i = 0; i < 4; i ++) {
            totalPower += action.wheelPower[i];
        }
        for (int i = 0; i < 4; i++) {
            action.wheelPower[i] = 4/totalPower * adaptivePowerRamping(Math.abs(action.wheelTarget[i]/H.COUNTS_PER_INCH), action.wheelPower[i], action.initialWheelTarget[i]/H.COUNTS_PER_INCH);
        }
        
    }
    
    private void RotateDistanceMoveMaintainHDP(ActionData action) {
    
    }
    
    private void StrafePowerMoveMaintainAP(ActionData action) {
    
    }
    
    private void RotatePowerMoveMaintainHP(ActionData action) {
    
    }
    
    private void StrafePointMoveMaintainPXY(ActionData action) {
    
    }
    
    private void RotatePointMoveMaintainPXY(ActionData action) {
    
    }
    
    private void CurvePointMoveMaintainPXY(ActionData action) {
    
    }
    
    private void AutoPointMoveMaintainPXY(ActionData action) {
    
    }
    
    ////////    Individual functions for easy auto-fill when programming    ////////
    
    boolean addAction(Actions actionName, double param1, double param2, double param3, int priority) {
        
        // don't execute action and return false if the current actions are a higher priority
        if (isLowPriority(priority)) return false;
    
        // add the action to the action list
        initializationList.add(new ActionData(actionName, new double[]{param1, param2, param3}, H.runtime.time(TimeUnit.MILLISECONDS)));
        return true; // action was added to list
        
    }
    
    boolean StrafeDistanceMove(double angle, double distance, double power, int priority) {
    
        // don't execute action and return false if the current actions are a higher priority
        if (isLowPriority(priority)) return false;
    
        // add the action to the action list
        initializationList.add(new ActionData(Actions.StrafeDistanceMoveHDP, new double[]{angle, distance, power}, H.runtime.time(TimeUnit.MILLISECONDS)));
        return true; // action was added to list
    
    }
    
    boolean RotateDistanceMove(double heading, double distance, double power, int priority) {
    
        // don't execute action and return false if the current actions are a higher priority
        if (isLowPriority(priority)) return false;
    
        // add the action to the action list
        initializationList.add(new ActionData(Actions.RotateDistanceMoveHDP, new double[]{heading, power, distance}, H.runtime.time(TimeUnit.MILLISECONDS)));
        return true; // action was added to list
        
    }
    
    boolean StrafePowerMove(double angle, double power, int priority) {
    
        // don't execute action and return false if the current actions are a higher priority
        if (isLowPriority(priority)) return false;
    
        // add the action to the action list
        initializationList.add(new ActionData(Actions.StrafePowerMoveAP, new double[]{angle, power}, H.runtime.time(TimeUnit.MILLISECONDS)));
        return true; // action was added to list
        
    }
    
    boolean RotatePowerMove(double heading, double power, int priority) {
    
        // don't execute action and return false if the current actions are a higher priority
        if (isLowPriority(priority)) return false;
    
        // add the action to the action list
        initializationList.add(new ActionData(Actions.RotatePowerMoveHP, new double[]{heading, power}, H.runtime.time(TimeUnit.MILLISECONDS)));
        return true; // action was added to list
        
    }
    
    boolean StrafePointMove(double power, double x, double y, int priority) {
    
        // don't execute action and return false if the current actions are a higher priority
        if (isLowPriority(priority)) return false;
    
        // add the action to the action list
        initializationList.add(new ActionData(Actions.StrafePointMovePXY, new double[]{power, x, y}, H.runtime.time(TimeUnit.MILLISECONDS)));
        return true; // action was added to list
        
    }
    
    boolean RotatePointMove(double power, double x, double y, int priority) {
    
        // don't execute action and return false if the current actions are a higher priority
        if (isLowPriority(priority)) return false;
    
        // add the action to the action list
        initializationList.add(new ActionData(Actions.RotatePowerMoveHP, new double[]{power, x, y}, H.runtime.time(TimeUnit.MILLISECONDS)));
        return true; // action was added to list
        
    }
    
    boolean CurvePointMove(double power, double x, double y, int priority) {
    
        // don't execute action and return false if the current actions are a higher priority
        if (isLowPriority(priority)) return false;
    
        // add the action to the action list
        initializationList.add(new ActionData(Actions.CurvePointMovePXY, new double[]{power, x, y}, H.runtime.time(TimeUnit.MILLISECONDS)));
        return true; // action was added to list
        
    }
    
    boolean AutoPointMove(double power, double x, double y, int priority) {
    
        // don't execute action and return false if the current actions are a higher priority
        if (isLowPriority(priority)) return false;
    
        // add the action to the action list
        initializationList.add(new ActionData(Actions.AutoPointMovePXY, new double[]{power, x, y}, H.runtime.time(TimeUnit.MILLISECONDS)));
        return true; // action was added to list
        
    }
    
    boolean AngleRotate(double angle, double power, int priority) {
    
        // don't execute action and return false if the current actions are a higher priority
        if (isLowPriority(priority)) return false;
    
        // add the action to the action list
        initializationList.add(new ActionData(Actions.AngleRotateAP, new double[]{angle, power, H.heading}, H.runtime.time(TimeUnit.MILLISECONDS)));
        return true; // action was added to list
        
    }
    
    boolean HeadingRotate(double heading, double power, int priority) {
    
        // don't execute action and return false if the current actions are a higher priority
        if (isLowPriority(priority)) return false;
    
        //find abs of initial offset value
        
        // add the action to the action list
        initializationList.add(new ActionData(Actions.HeadingRotateHP, new double[]{heading, power, findDegOffset(H.heading, heading)}, H.runtime.time(TimeUnit.MILLISECONDS)));
        return true; // action was added to list
    
    }
    
    boolean PowerRotate(double power, int priority) {
    
        // don't execute action and return false if the current actions are a higher priority
        if (isLowPriority(priority)) return false;
    
        // add the action to the action list
        initializationList.add(new ActionData(Actions.PowerRotateP, new double[]{power}, H.runtime.time(TimeUnit.MILLISECONDS)));
        return true; // action was added to list
    
    }
    
}

class ActionData {
    
    Actions action;
    double[] param;
    double[] wheelPower = {0,0,0,0};
    double[] powerPortion = {0,0,0,0};
    int[] wheelTarget = {0,0,0,0};
    int[] initialWheelTarget = {0,0,0,0};
    long startTime = 0;
    
    ActionData() {
    
    }
    
    ActionData(Actions action, double[] param, long startTime) {
    
        this.action = action;
        this.param = param;
        this.startTime = startTime;
    
    }
    
}
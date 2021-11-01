package org.firstinspires.ftc.teamcode;

public class RobotTracker implements Runnable{
    
    public static double[] output = {0, 0};
    
    double oldXEncoder = 0;
    double oldYEncoder = 0;
    double newXEncoder;
    double newYEncoder;
    
    double movementAngle;
    double movementDistance;
    double totalDistance = 0;
    RobotHardware H;
    
    RobotTracker(RobotHardware H) {
        this.H = H;
    }
    
    public void run() {
    
        while (!H.isStopRequested()) {
    
            Iterate();
    
        }
        
    }
    
    double[] Iterate() {
        
        newXEncoder = (H.xEncoder.getCurrentPosition() * Math.PI * 38.62) / (9000 * 4);
        newYEncoder = (H.yEncoder.getCurrentPosition() * Math.PI * 38.62) / (9000 * 4);
    
        double dx = newXEncoder - oldXEncoder;
        double dy = newYEncoder - oldYEncoder;
        
        movementAngle = Math.atan2(dy,dx);
        movementDistance = Math.hypot(dx,dy);
        totalDistance += movementDistance;
        
        output[0] += movementDistance*Math.cos(movementAngle + Math.toRadians(H.heading));
        output[1] += movementDistance*Math.sin(movementAngle + Math.toRadians(H.heading));
        
        oldXEncoder = newXEncoder;
        oldYEncoder = newYEncoder;
        
        return output;
    }
    
    void reset() {
        output[0] = 0;
        output[1] = 0;
    }
    
}

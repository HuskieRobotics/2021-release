/**
 * WASD keys to move joystick, Q & E keys to rotate joystick
 */

package frc.robot;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*; // use javafx libraries once these libraries are deprecated

import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;

final class SimConstants {
    public static final int[] WINDOW_SIZE = {700, 700}; // in pixels
    public static final String TITLE = "Swerve Simulator";
    public static final int DELAY = 40;
    public static final double MAX_LENGTH = 75;
    public static final int JOYSTICK_BASE_SIZE = 30;
    public static final int JOYSTICK_SIZE = 10;
    public static final double JOYSTICK_SPEED = 0.2;
}

public class SwerveSimRunner {
    public static void main(String[] args) {
        java.awt.EventQueue.invokeLater(new Runnable() {
            public void run() {
                SwerveSimRunner frame = new SwerveSimRunner();
            }
        });
    }
    public SwerveSimRunner() {
        JFrame jFrame = new JFrame();
        DrawingComponent drawingComponent = new DrawingComponent();
        drawingComponent.setDoubleBuffered(true); // enables double buffering
        
        jFrame.addKeyListener(drawingComponent);
        jFrame.add(drawingComponent);
        jFrame.setTitle(SimConstants.TITLE);
        jFrame.setSize(SimConstants.WINDOW_SIZE[0], SimConstants.WINDOW_SIZE[1]);
        jFrame.setVisible(true);
        jFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    }
}

class DrawingComponent extends JComponent implements KeyListener {
    private double[] startX = {SimConstants.WINDOW_SIZE[0]*0.5, SimConstants.WINDOW_SIZE[0]*0.75, SimConstants.WINDOW_SIZE[0]*0.5,  SimConstants.WINDOW_SIZE[0]*0.75};
    private double[] startY = {SimConstants.WINDOW_SIZE[1]*0.375, SimConstants.WINDOW_SIZE[1]*0.375, SimConstants.WINDOW_SIZE[1]*0.625, SimConstants.WINDOW_SIZE[1]*0.625};
    private double joystickX = 0;
    private double joystickY = 0;
    private double joystickZ = 0;
    private double joystickSpeedX = 0;
    private double joystickSpeedY = 0;
    private double joystickSpeedZ = 0;
    private final Drivetrain drivetrain = new Drivetrain();
    private SwerveModuleState[] states;
    public DrawingComponent() {
        states = drivetrain.driveSim(joystickX, joystickY, -1*joystickZ);
        //System.out.println(Arrays.toString(states));
        /**
         * Updates joystick values and repaints the 
         */
        Timer timer = new Timer(SimConstants.DELAY, new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                update();
                repaint(); // calls paintComponent method
            }
        });
        timer.start();
    }
    /**
     * paints the joystick and the wheel vectors on the jComponent
     */
    @Override
    protected void paintComponent(Graphics g) {
        Graphics2D g2d = (Graphics2D) g;
        g2d.setColor(Color.RED);
        g2d.drawOval((int) (SimConstants.WINDOW_SIZE[0]*0.25-SimConstants.JOYSTICK_BASE_SIZE), (int) (SimConstants.WINDOW_SIZE[1]*0.5-SimConstants.JOYSTICK_BASE_SIZE), SimConstants.JOYSTICK_BASE_SIZE*2, SimConstants.JOYSTICK_BASE_SIZE*2);
        int joystickXScreen = (int) (SimConstants.WINDOW_SIZE[0]*0.25+joystickX*(SimConstants.JOYSTICK_BASE_SIZE-SimConstants.JOYSTICK_SIZE));
        int joystickYScreen = (int) (SimConstants.WINDOW_SIZE[1]*0.5-joystickY*(SimConstants.JOYSTICK_BASE_SIZE-SimConstants.JOYSTICK_SIZE));
        g2d.drawOval((int) (joystickXScreen-SimConstants.JOYSTICK_SIZE), (int) (joystickYScreen-SimConstants.JOYSTICK_SIZE), SimConstants.JOYSTICK_SIZE*2, SimConstants.JOYSTICK_SIZE*2);
        g2d.drawLine((int) (joystickXScreen), (int) (joystickYScreen), (int) (joystickXScreen+SimConstants.JOYSTICK_SIZE*Math.cos(3.0/4.0*Math.PI*joystickZ-1.0/2.0*Math.PI)), (int) (joystickYScreen+SimConstants.JOYSTICK_SIZE*Math.sin(3.0/4.0*Math.PI*joystickZ-1.0/2.0*Math.PI)));
        //startX.length
        for (int i=0; i<startX.length; i++) {
            drawArrowLine(g2d, startX[i], startY[i], startX[i]+states[i].speedMetersPerSecond*(Math.cos(1/2*Math.PI-states[i].angle.getRadians()))*SimConstants.MAX_LENGTH, startY[i]+states[i].speedMetersPerSecond*(Math.sin(1/2*Math.PI-states[i].angle.getRadians()))*SimConstants.MAX_LENGTH, 10, 10);
        }
        g2d.drawString("Joystick Z: "+joystickZ, 10, 10);
        g2d.dispose();
    }

    /**
     * Draws an arrow line
     */
    private void drawArrowLine(Graphics2D g2d, double x1, double y1, double x2, double y2, double d, double h) {
        double dx = x2 - x1, dy = y2 - y1;
        double D = Math.sqrt(dx*dx + dy*dy);
        double xm = D - d, xn = xm, ym = h, yn = -h, x;
        double sin = dy / D, cos = dx / D;
    
        x = xm*cos - ym*sin + x1;
        ym = xm*sin + ym*cos + y1;
        xm = x;
    
        x = xn*cos - yn*sin + x1;
        yn = xn*sin + yn*cos + y1;
        xn = x;
    
        int[] xpoints = {(int) x2, (int) xm, (int) xn};
        int[] ypoints = {(int) y2, (int) ym, (int) yn};
    
        g2d.drawLine((int) x1, (int) y1, (int) x2, (int) y2);
        g2d.fillPolygon(xpoints, ypoints, 3);
    }

    /**
     * updates joystick X & Y values based on joystick speeds
     */
    public void update()
    {
        // code that runs in timer loop
        SmartDashboard.putNumber("Joystick Z: ", joystickZ);
        states = drivetrain.driveSim(joystickX, joystickY, -1*joystickZ);
        if ((joystickX+joystickSpeedX)*(joystickX+joystickSpeedX)+(joystickY+joystickSpeedY)*(joystickY+joystickSpeedY) < 1) {// false was last parameter.
            joystickX += joystickSpeedX;
            joystickY += joystickSpeedY;
        }
        if (joystickZ+joystickSpeedZ>-1 && joystickZ+joystickSpeedZ<1){
            joystickZ += joystickSpeedZ;
        }
        
    }

    /**
     * updates joystick speeds when WASD keys pressed & Q&E keys pressed
     */
    public void keyPressed(KeyEvent e) {
        //System.out.println(e.getKeyCode());
        switch(e.getKeyCode()) {
            case 87:
                //System.out.println("W");
                joystickSpeedY = SimConstants.JOYSTICK_SPEED;
                break;
            case 65:
                //System.out.println("A");
                joystickSpeedX = -SimConstants.JOYSTICK_SPEED;
                break;
            case 83:
                //System.out.println("S");
                joystickSpeedY = -SimConstants.JOYSTICK_SPEED;
                break;
            case 68:
                //System.out.println("D");
                joystickSpeedX = SimConstants.JOYSTICK_SPEED;
                break;
            case 38:
                //System.out.println("Up Arrow");
                break;
            case 37:
                //System.out.println("Left Arrow");
                break;
            case 40:
                //System.out.println("Down Arrow");
                break;
            case 39:
                //System.out.println("Right Arrow");
                break;
            case 81:
                //System.out.println("Q");
                joystickSpeedZ = -SimConstants.JOYSTICK_SPEED;
                break;
            case 69:
                //System.out.println("E");
                joystickSpeedZ = SimConstants.JOYSTICK_SPEED;
                break;
        }
    }
    public void keyTyped(KeyEvent e) {}
    /**
     * sets speeds to 0 when keys are released
     */
    public void keyReleased(KeyEvent e) {
        switch(e.getKeyCode()) {
            case 87:
                //System.out.println("W");
                joystickSpeedY = 0;
                break;
            case 65:
                //System.out.println("A");
                joystickSpeedX = 0;
                break;
            case 83:
                //System.out.println("S");
                joystickSpeedY = 0;
                break;
            case 68:
                //System.out.println("D");
                joystickSpeedX = 0;
                break;
            case 38:
                //System.out.println("Up Arrow");
                break;
            case 37:
                //System.out.println("Left Arrow");
                break;
            case 40:
                //System.out.println("Down Arrow");
                break;
            case 39:
                //System.out.println("Right Arrow");
                break;
            case 81:
                //System.out.println("Q");
                joystickSpeedZ = 0;
                break;
            case 69:
                //System.out.println("E");
                joystickSpeedZ = 0;
                break;
        }
    }
}
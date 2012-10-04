package gui;

import instruments.BatteryIndicator;
import instruments.Thermometer;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.EventQueue;

import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.border.EmptyBorder;

import org.ros.message.MessageListener;
import org.ros.message.sensor_msgs.Joy;

import controller.BatteryController;
import controller.TemperatureController;

import java.awt.GridLayout;
import java.io.IOException;

/**
 * This is a frame that the Operator uses
 * @author joe
 *
 */
public class VideoFrame extends JFrame {
	public VideoPanel videoPanel;
	public VideoPanel armVideoPanel;
	public VideoPanel driveVideoPanel;
	public Joystickcontroller xboxController = null;
	public TopDownRobotPanel ptzAnglePanel;
	private JPanel panel;

	/**
	 * Create the frame.
	 */
	public VideoFrame() {
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		setBounds(100, 100, 636, 559);
		
		videoPanel = new VideoPanel(true);
		getContentPane().add(videoPanel);
		
		
		try {
			
			panel = new JPanel();
			panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));
			getContentPane().add(panel, BorderLayout.EAST);
			
			armVideoPanel = new VideoPanel(false);
			armVideoPanel.setMaximumSize(new Dimension(232,160));
			panel.add(armVideoPanel);
			
			driveVideoPanel = new VideoPanel(false);
			driveVideoPanel.setMaximumSize(new Dimension(232,160));
			panel.add(driveVideoPanel);
			
			ptzAnglePanel = new TopDownRobotPanel();
			panel.add(ptzAnglePanel);
			
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		xboxController = new Joystickcontroller();
		
	
		
		
	}
	

	public class Joystickcontroller implements MessageListener<Joy>{
		//Axes
		public static final int LEFT_HORIZONTAL_AXIS = 0;
		public static final int LEFT_VERTICAL_AXIS = 1;
		public static final int LEFT_TRIGGER = 2;	
		public static final int RIGHT_HORIZONTAL_AXIS = 3;
		public static final int RIGHT_VERTICAL_AXIS = 4;
		public static final int RIGHT_TRIGGER = 5;
		public static final int LEFT_RIGHT_AXIS = 6;
		public static final int UP_DOWN_AXIS = 7;
		
		//Buttons
		public static final int A_BUTTON = 0;
		public static final int B_BUTTON = 1;
		public static final int X_BUTTON = 2;	
		public static final int Y_BUTTON = 3;
		public static final int LEFT_BUMPER = 4;
		public static final int RIGHT_BUMPER = 5;
		public static final int START_BUTTON = 7;
		public static final int XBOX_BUTTON = 8;
		public static final int UP_BUTTON = 10;
		public static final int DOWN_BUTTON = 11;
		public static final int LEFT_BUTTON = 12;	
		public static final int RIGHT_BUTTON = 13;
		public static final int BACK_BUTTON = 14;
		
		
	
		Joy previousState= new Joy();
		long timeAtLastPress=0;
		@Override
		public void onNewMessage(Joy msg) {
				if(msg.buttons[LEFT_BUMPER] <1 && msg.buttons[RIGHT_BUMPER] <1 && msg.buttons[XBOX_BUTTON] > 0){
					
					if(msg.buttons[X_BUTTON] > 0 && previousState.buttons[X_BUTTON] < 1){
						Process p;
						try {
							System.err.println("Killing Axis");
							p = Runtime.getRuntime().exec("/home/oryx/Desktop/scripts/KillAxis.sh");
							p.waitFor();
						} catch (IOException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						} catch (InterruptedException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					}
					if(msg.buttons[A_BUTTON] > 0 && previousState.buttons[A_BUTTON] < 1){
						Process p;
						try {
							System.err.println("Killing Arm");
							p = Runtime.getRuntime().exec("/home/oryx/Desktop/scripts/KillArmCam.sh");
							p.waitFor();
						} catch (IOException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						} catch (InterruptedException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					}
					if(msg.buttons[B_BUTTON] > 0 && previousState.buttons[B_BUTTON] < 1){
						Process p;
						try {
							System.err.println("Killing Drive");
							p = Runtime.getRuntime().exec("/home/oryx/Desktop/scripts/KillDriveCam.sh");
							p.waitFor();
						} catch (IOException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						} catch (InterruptedException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					}
				}
				previousState = msg;

		}

	}

}

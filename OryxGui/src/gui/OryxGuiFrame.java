package gui;

import instruments.AttitudeIndicator;
import instruments.MotorInfoInstrument;
import instruments.NetworkMonitor;
import instruments.PingIndicatorPanel;

import java.awt.BorderLayout;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.border.EmptyBorder;
import javax.swing.JMenuBar;
import javax.swing.JMenu;
import javax.swing.JTabbedPane;

import java.awt.Dimension;
import java.awt.GridLayout;
import java.awt.Component;
import java.util.ArrayList;

import javax.swing.Box;
import javax.swing.border.EtchedBorder;
import javax.swing.JTextPane;

import org.ros.message.MessageListener;
import org.ros.message.sensor_msgs.Joy;

import controller.BatteryController;
import controller.MotorController;
import controller.TemperatureController;
import java.awt.FlowLayout;
import java.io.IOException;

import javax.swing.BoxLayout;


public class OryxGuiFrame extends JFrame {

	

	private JPanel contentPane;
	public JTabbedPane tabbedPane;
	public VideoPanel videoPanel;
	public VideoPanel subVideoPanel;
	
	public ThermometerPanel thermometerPanel;
	public TopDownRobotPanel topDownPanel;
	public AttitudeIndicatorPanel attitudeIndicatorPanel;
	public BatteryPanel batteryPanel;
	private JTextPane consolePane;
	
	private JPanel instrumentPanel;
	private JPanel userPanel;

	
	public Joystickcontroller xboxController = null;
	public TemperatureController tempController = new TemperatureController();
	public BatteryController batteryController = new BatteryController();
	public MotorController driveMotorController;
	public MotorController armMotorController;
	public MotorController ptzMotorController;
	
	/**
	 * Create the frame.
	 */
	public OryxGuiFrame() {
		initGUI();
	}

	private void initGUI() {
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		setBounds(100, 100, 944, 740);
		this.contentPane = new JPanel();
		this.contentPane.setBorder(new EmptyBorder(5, 5, 5, 5));
		this.contentPane.setLayout(new BorderLayout(0, 0));
		setContentPane(this.contentPane);
		
		////////////////TABBED PANE////////////////////////////
		
		this.tabbedPane = new JTabbedPane(JTabbedPane.TOP);		
		this.contentPane.add(this.tabbedPane, BorderLayout.SOUTH);

		//Drive Motor Information
		driveMotorController = new MotorController("Drive");
		driveMotorController.addMotorInstrument(1, "Left Front Wheel");
		driveMotorController.addMotorInstrument(2, "Right Front Wheel");
		driveMotorController.addMotorInstrument(3, "Right Back Wheel");
		driveMotorController.addMotorInstrument(4, "Left Back Wheel");
		
		JPanel driveMotorPanel = new JPanel();
		driveMotorPanel.setLayout(new GridLayout(1, 0, 0, 0));
		driveMotorPanel.add(driveMotorController.getMotorInstrument(1));
		driveMotorPanel.add(driveMotorController.getMotorInstrument(2));
		driveMotorPanel.add(driveMotorController.getMotorInstrument(3));
		driveMotorPanel.add(driveMotorController.getMotorInstrument(4));
	
		tabbedPane.addTab("Drive Motor Info",driveMotorPanel);
		
		//PTZ Motor Information
		ptzMotorController = new MotorController("Camera");
		ptzMotorController.addMotorInstrument(1, "Boom");
		ptzMotorController.addMotorInstrument(2, "Pan");
		ptzMotorController.addMotorInstrument(3, "Tilt");

		JPanel ptzMotorPanel = new JPanel();
		ptzMotorPanel.setLayout(new GridLayout(1, 0, 0, 0));
		ptzMotorPanel.add(ptzMotorController.getMotorInstrument(1));
		ptzMotorPanel.add(ptzMotorController.getMotorInstrument(2));
		ptzMotorPanel.add(ptzMotorController.getMotorInstrument(3));
		tabbedPane.addTab("PTZ Motor Info",ptzMotorPanel);
		
		//Arm Motor Information
		armMotorController = new MotorController("Arm");
		armMotorController.addMotorInstrument(1, "Turret");
		armMotorController.addMotorInstrument(2, "Shoulder");
		armMotorController.addMotorInstrument(3, "Claw");
		
		JPanel armMotorPanel = new JPanel();
		armMotorPanel.setLayout(new GridLayout(1, 0, 0, 0));
		armMotorPanel.add(armMotorController.getMotorInstrument(1));
		armMotorPanel.add(armMotorController.getMotorInstrument(2));
		armMotorPanel.add(armMotorController.getMotorInstrument(3));
		tabbedPane.addTab("Arm Motor Info",armMotorPanel);

		
			
		//Add Console
		consolePane = new JTextPane();
		consolePane.setEditable(false);
		
		tabbedPane.addTab("Console", null, consolePane, null);
		
		
		///////////////////VIDEO////////////////////////////////////
		this.videoPanel = new VideoPanel(true);
		videoPanel.setBorder(new EtchedBorder(EtchedBorder.LOWERED, null, null));
		this.contentPane.add(this.videoPanel, BorderLayout.CENTER);
		
		
		///////////////////INSTURMENT PANEL/////////////////////////
		instrumentPanel = new JPanel();
		instrumentPanel.setBorder(new EtchedBorder(EtchedBorder.LOWERED, null, null));
		//instrumentPanel.setLayout(new BoxLayout(instrumentPanel, BoxLayout.Y_AXIS));
		instrumentPanel.setLayout(new GridLayout(2, 1));
		
		thermometerPanel = new ThermometerPanel(tempController);
		thermometerPanel.setPreferredSize(new Dimension(232, 350));
		thermometerPanel.setMaximumSize(new Dimension(232, 350));
	    

		instrumentPanel.add(thermometerPanel);
		batteryPanel = new BatteryPanel(batteryController);
		
		instrumentPanel.add(batteryPanel);
		contentPane.add(instrumentPanel, BorderLayout.WEST);
		//////////////////////USER PANEL//////////////////////////////
		userPanel = new JPanel();
		userPanel.setBorder(new EtchedBorder(EtchedBorder.LOWERED, null, null));
		userPanel.setLayout(new BoxLayout(userPanel, BoxLayout.Y_AXIS));

		contentPane.add(userPanel, BorderLayout.EAST);
		
	    subVideoPanel = new VideoPanel(false);
		subVideoPanel.setMaximumSize(new Dimension(232,160));
		userPanel.add(subVideoPanel);
		
		userPanel.add(new PingIndicatorPanel());
		
		NetworkMonitor networkMonitor = new NetworkMonitor();
		networkMonitor.setPreferredSize(new Dimension(232,80));
		networkMonitor.setMaximumSize(new Dimension(232,80));
		userPanel.add(networkMonitor);
		
		try {
			topDownPanel = new TopDownRobotPanel();
			userPanel.add(topDownPanel);
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		try {
			attitudeIndicatorPanel = new AttitudeIndicatorPanel();
			userPanel.add(attitudeIndicatorPanel);
		} catch (IOException e) {
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
				if(msg.buttons[XBOX_BUTTON] < 1 && msg.buttons[A_BUTTON] >0 && previousState.buttons[A_BUTTON] < 1){
					videoPanel.videoSourceSelector.nextSelection();
				}
				if (msg.buttons[Y_BUTTON] > 0 && previousState.buttons[Y_BUTTON] < 1) {
					System.err.println("Y Button Pressed");
					int mainSelectedIndex = videoPanel.videoSourceSelector.getSelectedIndex();
					videoPanel.videoSourceSelector.setSelectedIndex(subVideoPanel.videoSourceSelector.getSelectedIndex());
					subVideoPanel.videoSourceSelector.setSelectedIndex(mainSelectedIndex);
				}
				previousState = msg;

		}

	}
}

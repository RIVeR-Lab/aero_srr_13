package instruments;

import java.awt.Component;
import java.awt.Font;
import java.awt.GridLayout;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingConstants;
import javax.swing.border.EtchedBorder;

import org.ros.message.EposManager.MotorInfo;

import java.awt.Color;
import java.awt.FlowLayout;
import java.awt.BorderLayout;

/**
 * This isntrument displays motor information including name, current, velocity, position, and status
 * @author joe
 *
 */
public class MotorInfoInstrument extends JPanel {
	
	public final static int MOTOR_STATE_DISABLED = 0;
	public final static int MOTOR_STATE_ENABLED = 1;
	public final static int MOTOR_STATE_QUICKSTOP = 2;
	public final static int MOTOR_STATE_FAULT = 3;
	
	private JLabel lblMotorName;
	private JLabel lblMotorNode;
	private JPanel motorGroupPanel;
	private JPanel motorInfoPanel;
	private JLabel motorStatusLabel;
	private JLabel lblStatus;
	private JLabel motorModeLabel;
	private JLabel lblMode;
	private JLabel motorVelocityLabel;
	private JLabel lblVelocity;
	private JLabel motorPositionLabel;
	private JLabel lblPosition;
	private JLabel motorCurrentLabel;
	private JLabel lblCurrent;
	
	public int motorNode;
	public int motorVelocity;
	public int motorPosition;
	public double motorCurrent;
	public String motorMode;
	public int motorState;
	public String motorFaults;
	public String motorName;
	private JPanel errorPanel;
	private JLabel errorLabel;
	private JLabel lblErrors;

/**
 * Creates the motor instrument
 * @param name The name of the mootor
 * @param group The name of the subgroup the motor is a apart of
 * @param nodeId The node ID of the motor within that subgroup
 */
	public MotorInfoInstrument(String name, String group, int nodeId) {
		setBorder(new EtchedBorder(EtchedBorder.LOWERED, null, null));
		setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
	
		
		motorGroupPanel = new JPanel();
		add(motorGroupPanel);
		motorGroupPanel.setLayout(new GridLayout(0, 2, 0, 0));
		
		lblMotorName = new JLabel(name);
		motorGroupPanel.add(lblMotorName);
		lblMotorName.setAlignmentX(Component.CENTER_ALIGNMENT);
		lblMotorName.setFont(new Font("Dialog", Font.BOLD, 14));
		motorName=name;
		
		lblMotorNode = new JLabel(""+nodeId);
		motorNode=nodeId;
		motorGroupPanel.add(lblMotorNode);
		
		motorInfoPanel = new JPanel();
		add(motorInfoPanel);
		motorInfoPanel.setLayout(new GridLayout(0, 2, 0, 0));
		
		motorStatusLabel = new JLabel("Status:");
		motorInfoPanel.add(motorStatusLabel);
		
		lblStatus = new JLabel("Enabled");
		motorInfoPanel.add(lblStatus);
		
		motorModeLabel = new JLabel("Mode:");
		motorInfoPanel.add(motorModeLabel);
		
		lblMode = new JLabel("Velocity");
		motorInfoPanel.add(lblMode);
		
		motorVelocityLabel = new JLabel("Velocity:");
		motorInfoPanel.add(motorVelocityLabel);
		
		lblVelocity = new JLabel("0 RPM");
		motorInfoPanel.add(lblVelocity);
		
		motorPositionLabel = new JLabel("Position:");
		motorInfoPanel.add(motorPositionLabel);
		
		lblPosition = new JLabel("0 Tics");
		motorInfoPanel.add(lblPosition);
		
		motorCurrentLabel = new JLabel("Current:");
		motorInfoPanel.add(motorCurrentLabel);
		
		lblCurrent = new JLabel("0 mA");
		motorInfoPanel.add(lblCurrent);
		
		errorPanel = new JPanel();
		add(errorPanel);
		errorPanel.setLayout(new BorderLayout(0, 0));
		
		errorLabel = new JLabel("Errors:");
		errorLabel.setVerticalAlignment(SwingConstants.BOTTOM);
		errorLabel.setHorizontalAlignment(SwingConstants.LEFT);
		errorPanel.add(errorLabel, BorderLayout.WEST);
		
		lblErrors = new JLabel("");
		lblErrors.setForeground(Color.RED);
		errorPanel.add(lblErrors);

	}
	
	public int getMotorNode() {
		return motorNode;
	}

	public void setMotorNode(int motorNode) {
		this.motorNode = motorNode;
		lblMotorNode.setText(""+this.motorMode);
	}

	public int getMotorVelocity() {
		return motorVelocity;
	}

	public void setMotorVelocity(int motorVelocity) {
		this.motorVelocity = motorVelocity;
		lblVelocity.setText(this.motorVelocity + " RPM");
	}

	public int getMotorPosition() {
		return motorPosition;
	}

	public void setMotorPosition(int motorPosition) {
		this.motorPosition = motorPosition;
		lblPosition.setText(""+this.motorPosition);
	}

	public double getMotorCurrent() {
		return motorCurrent;
	}

	public void setMotorCurrent(double motorCurrent) {
		this.motorCurrent = motorCurrent;
		lblCurrent.setText(this.motorCurrent+ "mA");
	}

	public String getMotorMode() {
		return motorMode;
	}

	public void setMotorMode(String motorMode) {
		this.motorMode = motorMode;
		lblMode.setText(this.motorMode);
	}

	public int getMotorState() {
		return motorState;
	}

	public void setMotorState(int motorState) {
		switch(motorState){
		case MOTOR_STATE_DISABLED:
			lblStatus.setText("Disabled");
			break;
		case MOTOR_STATE_ENABLED:
			lblStatus.setText("Enabled");
			break;	
		case MOTOR_STATE_FAULT:	
			lblStatus.setText("Fault");
			break;
		case MOTOR_STATE_QUICKSTOP:	
			lblStatus.setText("QuickStop");
			break;
		}
		this.motorState = motorState;
	}

	public String getMotorName() {
		return motorName;
	}

	public void setMotorName(String motorName) {
		this.motorName = motorName;
		lblMotorName.setText(this.motorName);
	}
	
	public void setErrors(String errors){
		lblErrors.setText(errors);
	}
	
	public void updateInstrument(MotorInfo msg){
		setMotorState(msg.state);
		setMotorVelocity(msg.motor_velocity);
		setMotorCurrent(msg.motor_current);
		setMotorPosition(msg.motor_position);
		setErrors(msg.faults);
	}
	
	public void setInfo(MotorInfo msg){
		setMotorVelocity(msg.motor_velocity);
		setMotorPosition(msg.motor_position);
		setMotorCurrent(msg.motor_current);
		setMotorState(msg.state);
		setErrors(msg.faults);
	}


}

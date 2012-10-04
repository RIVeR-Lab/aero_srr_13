package gui;

import instruments.AttitudeIndicator;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.io.IOException;

import javax.swing.JPanel;
import java.awt.GridLayout;
import javax.swing.JLabel;

import org.ros.message.MessageListener;
import org.ros.message.geometry_msgs.Quaternion;
import org.ros.message.sensor_msgs.Imu;

import java.awt.Font;

/**
 * @author joe
 *
 */
public class AttitudeIndicatorPanel extends JPanel implements MessageListener<Imu> {
	AttitudeIndicator attitude;
	private JPanel pnlData;
	private JLabel lblPitch;
	private JLabel lblPitchVal;
	private JLabel lblRoll;
	private JLabel lblRollVal;

	
	public AttitudeIndicatorPanel() throws IOException {
		setLayout(new BorderLayout(0, 0));
		attitude = new AttitudeIndicator();
		setMaximumSize(new Dimension(232, 275));
		setPreferredSize(new Dimension(232, 275));
		add(attitude,BorderLayout.CENTER);
		
		pnlData = new JPanel();
		add(pnlData, BorderLayout.SOUTH);
		pnlData.setLayout(new GridLayout(0, 2, 0, 0));
		
		lblPitch = new JLabel("Pitch:");
		lblPitch.setFont(new Font("FreeSans", Font.PLAIN, 16));
		pnlData.add(lblPitch);
		
		lblPitchVal = new JLabel("0.0");
		lblPitchVal.setFont(new Font("FreeSans", Font.PLAIN, 16));
		pnlData.add(lblPitchVal);
		
		lblRoll = new JLabel("Roll:");
		lblRoll.setFont(new Font("FreeSans", Font.PLAIN, 16));
		pnlData.add(lblRoll);
		
		lblRollVal = new JLabel("0.0");
		lblRollVal.setFont(new Font("FreeSans", Font.PLAIN, 16));
		pnlData.add(lblRollVal);
		attitude.setAttitude(0, 0);
	}
	
	public void setAttitude (double pitch, double roll){
		attitude.setAttitude((int)roll,(int) pitch);
		lblRollVal.setText(""+roll);
		lblPitchVal.setText(""+pitch);
	}
	
	public double getPitch(){
		return attitude.getPitch();
	}
	public double getRoll(){
		return attitude.getRoll();
	}
	


	@Override
	public void onNewMessage(Imu imu) {
		Quaternion msg = imu.orientation;
		double roll = Math.toDegrees(Math.atan2(2*(msg.w*msg.x + msg.y*msg.z), 1-2*(msg.x*msg.x+msg.y*msg.y)));
		double pitch = Math.toDegrees(Math.asin(2*(msg.w*msg.y-msg.x*msg.z)));
		
		setAttitude(pitch, roll);
		
	}

	}

package gui;

import instruments.CameraDirectionInstrument;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.io.IOException;

import javax.swing.JPanel;
import javax.swing.JLabel;
import javax.swing.BoxLayout;

import java.awt.Color;
import java.awt.GridLayout;
import java.awt.GridBagLayout;
import java.awt.GridBagConstraints;
import java.awt.Insets;
import javax.swing.JTextField;
import java.awt.Font;
import java.awt.Component;
import javax.swing.Box;
import java.awt.FlowLayout;
import javax.swing.SwingConstants;
import javax.swing.border.EtchedBorder;

import org.ros.message.MessageListener;
import org.ros.message.geometry_msgs.PoseArray;
import org.ros.message.geometry_msgs.Quaternion;
import org.ros.message.geometry_msgs.QuaternionStamped;

/**
 * This panel contains a top down view of the robot, displaying the pan of the camera visually
 * and the pan/tilt of the camera textually
 * @author joe
 *
 */
public class TopDownRobotPanel extends JPanel {
	private JPanel infoPanel;
	private JLabel lblPanAngle;
	private JTextField txtPanAngle;
	private JLabel lblTiltAngle;
	private JTextField txtTiltAngle;
	CameraDirectionInstrument cameraDirectionInstrument;
	public CameraOrientationListener orientationListener = new CameraOrientationListener();
	public WheelHeightListener wheelListener = new WheelHeightListener();
	/**
	 * Create the panel.
	 * @throws IOException 
	 */
	public TopDownRobotPanel() throws IOException {
		setBorder(new EtchedBorder(EtchedBorder.LOWERED, null, null));
		
		setPreferredSize(new Dimension(232, 285));
		setMaximumSize(new Dimension(232, 285));
		setLayout(new BorderLayout(2, 1));
		 cameraDirectionInstrument = new CameraDirectionInstrument();
		this.add(cameraDirectionInstrument);
		
		infoPanel = new JPanel();
		infoPanel.setPreferredSize(new Dimension(237, 30));
		infoPanel.setMaximumSize(new Dimension(237, 30));
		add(infoPanel, BorderLayout.SOUTH);
		infoPanel.setLayout(new GridLayout(0, 2, 0, 0));
		
		lblPanAngle = new JLabel("Pan Angle:");
		lblPanAngle.setFont(new Font("Dialog", Font.BOLD, 14));
		infoPanel.add(lblPanAngle);
		
		txtPanAngle = new JTextField();
		txtPanAngle.setText("0");
		txtPanAngle.setFont(new Font("Dialog", Font.BOLD, 14));
		txtPanAngle.setEditable(false);
		infoPanel.add(txtPanAngle);
		txtPanAngle.setColumns(10);
		
		lblTiltAngle = new JLabel("Tilt Angle:");
		lblTiltAngle.setFont(new Font("Dialog", Font.BOLD, 14));
		infoPanel.add(lblTiltAngle);
		
		txtTiltAngle = new JTextField();
		txtTiltAngle.setText("0");
		txtTiltAngle.setEditable(false);
		txtTiltAngle.setFont(new Font("Dialog", Font.BOLD, 14));
		infoPanel.add(txtTiltAngle);
		txtTiltAngle.setColumns(10);
		
		
		
		
	}
	
	public void setPanTilt(double pan, double tilt){
		if(tilt > 45 || tilt < -45) txtTiltAngle.setForeground(Color.RED);
		else txtTiltAngle.setForeground(Color.BLACK);
		
		if(pan > 140 || pan < -140)txtPanAngle.setForeground(Color.RED);
		else txtPanAngle.setForeground(Color.BLACK);
		txtTiltAngle.setText(Integer.toString((int)tilt));
		txtPanAngle.setText(Integer.toString((int)pan));
		cameraDirectionInstrument.setPanTilt(pan, tilt);
	}
	
	public class CameraOrientationListener implements MessageListener<Quaternion>{

		@Override
		public void onNewMessage(Quaternion msg) {
			double tilt = msg.y;
			double pan = msg.z;
			setPanTilt(pan, tilt);
		}
		
	}
	
	public class WheelHeightListener implements MessageListener<PoseArray>{

		@Override
		public void onNewMessage(PoseArray msg) {
			double fl =  msg.poses.get(0).position.z;
			double fr = msg.poses.get(1).position.z;
			double bl = msg.poses.get(2).position.z;
			double br = msg.poses.get(3).position.z;
			cameraDirectionInstrument.setHeights(fl, fr, bl, br);
			
		}
		
	}
	


}

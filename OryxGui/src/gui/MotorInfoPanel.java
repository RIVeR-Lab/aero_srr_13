package gui;

import instruments.MotorInfoInstrument;

import java.awt.GridLayout;
import java.util.ArrayList;

import javax.swing.JPanel;

import org.ros.message.MessageListener;
import org.ros.message.EposManager.GroupMotorInfo;
import org.ros.message.EposManager.MotorInfo;

public class MotorInfoPanel extends JPanel {
	public ArrayList<MotorInfoInstrument> motorList;
	public MotorInfoListener motorInfoListener;
	
	
	/**
	 * Create the panel.
	 */
	public MotorInfoPanel(ArrayList<MotorInfoInstrument> motorList ) {
		this.motorList= new ArrayList<MotorInfoInstrument>();
		this.motorList.addAll(motorList);
		this.setLayout(new GridLayout(1, 0, 0, 0));
		for (MotorInfoInstrument motor : motorList) {
			this.add(motor);
		}
		motorInfoListener = new MotorInfoListener();
	}
	
	private class MotorInfoListener implements MessageListener<GroupMotorInfo>{

		@Override
		public void onNewMessage(GroupMotorInfo msg) {
			for (MotorInfo motor : msg.motor_group) {
				setMotorInfo(motor);
			}
			
			
		}
		
	}
	
	private void setMotorInfo(MotorInfo msg){
		MotorInfoInstrument motorInstrument=null;
		for (MotorInfoInstrument motor : motorList) {
			if (msg.node_id == motor.getMotorNode()){
				motorInstrument=motor;
				break;
			}			
		}
		if(motorInstrument==null){
			System.out.println("Motor Null");
			return;
		}
		motorInstrument.setMotorVelocity(msg.motor_velocity);
		motorInstrument.setMotorPosition(msg.motor_position);
		motorInstrument.setMotorCurrent(msg.motor_current);
		motorInstrument.setMotorState(msg.state);
		motorInstrument.setErrors(msg.faults);
	}
	

}

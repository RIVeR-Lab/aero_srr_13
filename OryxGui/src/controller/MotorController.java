package controller;

import instruments.BatteryIndicator;
import instruments.MotorInfoInstrument;

import java.util.HashMap;

import org.ros.message.MessageListener;
import org.ros.message.EposManager.GroupMotorInfo;
import org.ros.message.EposManager.MotorInfo;
import org.ros.message.OryxMessages.Temperature;

public class MotorController implements MessageListener<GroupMotorInfo> {
	public HashMap<String, MotorInfoInstrument> motorMap = new HashMap<String,  MotorInfoInstrument>();
	String motorGroup;
	
	public MotorController(String subGroup){
		this.motorGroup = subGroup;
	}
	public MotorInfoInstrument getMotorInstrument(int node){
		return(MotorInfoInstrument) motorMap.get(Integer.toString(node));
	}
	
	public void addMotorInstrument(int nodeId, String name){
		MotorInfoInstrument instrument = new MotorInfoInstrument(name, motorGroup, nodeId);
		motorMap.put(Integer.toString(nodeId), instrument);
	}
	
	@Override
	public void onNewMessage(GroupMotorInfo msg) {	
		for (MotorInfo info : msg.motor_group) {
			if(motorMap.containsKey(Integer.toString(info.node_id)))
				getMotorInstrument(info.node_id).setInfo(info);
		}

	}
}

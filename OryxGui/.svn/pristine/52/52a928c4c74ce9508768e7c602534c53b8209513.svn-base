package controller;

import instruments.BatteryIndicator;
import instruments.Thermometer;

import java.util.HashMap;

import org.ros.message.MessageListener;
import org.ros.message.OryxMessages.Battery;

public class BatteryController implements MessageListener<Battery> {

	public static final int BATTERY_PACK_NODE = 0;
	public static final int BATTERY_CELL_1_NODE = 1;
	public static final int BATTERY_CELL_2_NODE = 2;
	public static final int BATTERY_CELL_3_NODE = 3;
	public static final int BATTERY_CELL_4_NODE = 4;
	public static final int BATTERY_CELL_5_NODE = 5;
	public static final int BATTERY_CELL_6_NODE = 6;
	public static final int BATTERY_CELL_7_NODE = 7;
	
	public HashMap<String, BatteryIndicator> batteryMap = new HashMap<String, BatteryIndicator>(7);
	
	public BatteryController(){
		batteryMap.put(Integer.toString(BATTERY_PACK_NODE), new BatteryIndicator(14, 25.5f, 16.5f, BATTERY_PACK_NODE));
		batteryMap.put(Integer.toString(BATTERY_CELL_1_NODE), new BatteryIndicator(2.0f, 3.65f, 2.4f, BATTERY_CELL_1_NODE));
		batteryMap.put(Integer.toString(BATTERY_CELL_2_NODE), new BatteryIndicator(2.0f, 3.65f, 2.4f, BATTERY_CELL_2_NODE));
		batteryMap.put(Integer.toString(BATTERY_CELL_3_NODE), new BatteryIndicator(2.0f, 3.65f, 2.4f, BATTERY_CELL_3_NODE));
		batteryMap.put(Integer.toString(BATTERY_CELL_4_NODE), new BatteryIndicator(2.0f, 3.65f, 2.4f, BATTERY_CELL_4_NODE));
		batteryMap.put(Integer.toString(BATTERY_CELL_5_NODE), new BatteryIndicator(2.0f, 3.65f, 2.4f, BATTERY_CELL_5_NODE));
		batteryMap.put(Integer.toString(BATTERY_CELL_6_NODE), new BatteryIndicator(2.0f, 3.65f, 2.4f, BATTERY_CELL_6_NODE));
		batteryMap.put(Integer.toString(BATTERY_CELL_7_NODE), new BatteryIndicator(2.0f, 3.65f, 2.4f, BATTERY_CELL_7_NODE));
	}
	
	public BatteryIndicator getIndicator(int node){
		return batteryMap.get(Integer.toString(node));
		
	}
	
	@Override
	public void onNewMessage(Battery msg) {
		if(batteryMap.containsKey(Integer.toString(msg.node)))
			getIndicator(msg.node).setVoltage(msg.voltage);
	}
	
}

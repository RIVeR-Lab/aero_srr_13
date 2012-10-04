package controller;

import instruments.Thermometer;

import java.util.HashMap;
import java.util.Map;

import org.ros.message.MessageListener;
import org.ros.message.OryxMessages.Temperature;

public class TemperatureController implements MessageListener<Temperature>{
	public final int FRONT_LEFT_MOTOR_TEMP_NODE = 1;
	public final int BACK_LEFT_MOTOR_TEMP_NODE = 5;
	public  final int BACK_RIGHT_MOTOR_TEMP_NODE = 7;	
	public  final int FRONT_RIGHT_MOTOR_TEMP_NODE = 3;
	public  final int CORE_1_TEMP_NODE = 11;
	public  final int CORE_2_TEMP_NODE = 12;
	public  final int CORE_3_TEMP_NODE = 13;
	public  final int CORE_4_TEMP_NODE = 14;
	public  final int SOUTHBRIDGE_1_TEMP_NODE = 15;
	public  final int SOUTHBRIDGE_2_TEMP_NODE = 16;
	public  final int BATTERY_CELL_1_TEMP_NODE = 18;
	public final int BATTERY_CELL_2_TEMP_NODE = 19;
	public  final int BATTERY_CELL_3_TEMP_NODE = 20;
	public final int BATTERY_CELL_4_TEMP_NODE = 21;
	public  final int BATTERY_CELL_5_TEMP_NODE = 22;
	public  final int BATTERY_CELL_6_TEMP_NODE = 23;
	public  final int POWER_DIST_1_TEMP_NODE = 2;
	public  final int POWER_DIST_2_TEMP_NODE = 4;
	public  final int POWER_DIST_3_TEMP_NODE = 6;
	public  final int BMS_TEMP_NODE = 17;
	public  final int CHIPSET_TEMP_NODE = 9;
	public  final int CONVECTOR_TEMP_NODE = 8;
	
	public HashMap<String, Thermometer> temperatureMap = new HashMap<String, Thermometer>(22);
	
	public TemperatureController(){
		temperatureMap.put(Integer.toString(FRONT_LEFT_MOTOR_TEMP_NODE), new Thermometer(FRONT_LEFT_MOTOR_TEMP_NODE, "Front Left Motor", 70, 180));
		temperatureMap.put(Integer.toString(FRONT_RIGHT_MOTOR_TEMP_NODE), new Thermometer(FRONT_RIGHT_MOTOR_TEMP_NODE, "Front Right Motor", 70, 180));
		temperatureMap.put(Integer.toString(BACK_LEFT_MOTOR_TEMP_NODE), new Thermometer(BACK_LEFT_MOTOR_TEMP_NODE, "Back Left Motor", 70, 180));
		temperatureMap.put(Integer.toString(BACK_RIGHT_MOTOR_TEMP_NODE), new Thermometer(BACK_RIGHT_MOTOR_TEMP_NODE, "Back Right Motor", 70, 180));
		temperatureMap.put(Integer.toString(CORE_1_TEMP_NODE), new Thermometer(CORE_1_TEMP_NODE, "Core 1", 70, 180));
		temperatureMap.put(Integer.toString(CORE_2_TEMP_NODE), new Thermometer(CORE_2_TEMP_NODE, "Core 2", 70, 180));
		temperatureMap.put(Integer.toString(CORE_3_TEMP_NODE), new Thermometer(CORE_3_TEMP_NODE, "Core 3", 70, 180));
		temperatureMap.put(Integer.toString(CORE_4_TEMP_NODE), new Thermometer(CORE_4_TEMP_NODE, "Core 4", 70, 180));
		temperatureMap.put(Integer.toString(SOUTHBRIDGE_1_TEMP_NODE), new Thermometer(SOUTHBRIDGE_1_TEMP_NODE, "Southbridge 1", 70, 180));
		temperatureMap.put(Integer.toString(SOUTHBRIDGE_2_TEMP_NODE), new Thermometer(SOUTHBRIDGE_2_TEMP_NODE, "Southbridge 2", 70, 180));	
		temperatureMap.put(Integer.toString(BATTERY_CELL_1_TEMP_NODE), new Thermometer(BATTERY_CELL_1_TEMP_NODE, "Battery Cell 1", 70, 130));
		temperatureMap.put(Integer.toString(BATTERY_CELL_2_TEMP_NODE), new Thermometer(BATTERY_CELL_2_TEMP_NODE, "Battery Cell 2", 70, 130));
		temperatureMap.put(Integer.toString(BATTERY_CELL_3_TEMP_NODE), new Thermometer(BATTERY_CELL_3_TEMP_NODE, "Battery Cell 3", 70, 130));
		temperatureMap.put(Integer.toString(BATTERY_CELL_4_TEMP_NODE), new Thermometer(BATTERY_CELL_4_TEMP_NODE, "Battery Cell 4", 70, 130));
		temperatureMap.put(Integer.toString(BATTERY_CELL_5_TEMP_NODE), new Thermometer(BATTERY_CELL_5_TEMP_NODE, "Battery Cell 5", 70, 130));
		temperatureMap.put(Integer.toString(BATTERY_CELL_6_TEMP_NODE), new Thermometer(BATTERY_CELL_6_TEMP_NODE, "Battery Cell 6", 70, 130));
		temperatureMap.put(Integer.toString(POWER_DIST_1_TEMP_NODE), new Thermometer(POWER_DIST_1_TEMP_NODE, "Power Dist 1", 70, 150));
		temperatureMap.put(Integer.toString(POWER_DIST_2_TEMP_NODE), new Thermometer(POWER_DIST_2_TEMP_NODE, "Power Dist 2", 70, 150));
		temperatureMap.put(Integer.toString(POWER_DIST_3_TEMP_NODE), new Thermometer(POWER_DIST_3_TEMP_NODE, "Power Dist 3", 70, 150));
		temperatureMap.put(Integer.toString(BMS_TEMP_NODE), new Thermometer(BMS_TEMP_NODE, "BMS", 70, 150));
		temperatureMap.put(Integer.toString(CHIPSET_TEMP_NODE), new Thermometer(CHIPSET_TEMP_NODE, "Chipset", 70, 170));
		temperatureMap.put(Integer.toString(CONVECTOR_TEMP_NODE), new Thermometer(CONVECTOR_TEMP_NODE, "Convector", 70, 130));
	}
	
	public Thermometer getThermometer(int node){
		return (Thermometer) temperatureMap.get(Integer.toString(node));
	}
	
	@Override
	public void onNewMessage(Temperature msg) {	
		if(temperatureMap.containsKey(Integer.toString(msg.temperature_node)))
			getThermometer(msg.temperature_node).setTemperature(msg.temperature);
	}
	
	
	
}

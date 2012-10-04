package gui;

import instruments.Thermometer;

import java.util.ArrayList;

import javax.swing.JPanel;

import java.awt.Component;
import java.awt.GridLayout;
import javax.swing.JLabel;
import javax.swing.BoxLayout;

import org.ros.message.MessageListener;
import org.ros.message.OryxMessages.Temperature;

import controller.TemperatureController;

import java.awt.CardLayout;

public class ThermometerPanel extends JPanel {
	public ArrayList<Thermometer> thermometerList = new ArrayList<Thermometer>();
	TemperatureController controller;
	/**
	 * Create the panel.
	 */
	public ThermometerPanel(TemperatureController controller) {
		this.controller=controller;
		setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
		add(controller.getThermometer(controller.BACK_LEFT_MOTOR_TEMP_NODE));
		add(controller.getThermometer(controller.BACK_RIGHT_MOTOR_TEMP_NODE));
		add(controller.getThermometer(controller.FRONT_LEFT_MOTOR_TEMP_NODE));
		add(controller.getThermometer(controller.FRONT_RIGHT_MOTOR_TEMP_NODE));
		add(controller.getThermometer(controller.CORE_1_TEMP_NODE));
		add(controller.getThermometer(controller.CORE_2_TEMP_NODE));
		add(controller.getThermometer(controller.CORE_3_TEMP_NODE));
		add(controller.getThermometer(controller.CORE_4_TEMP_NODE));
//		add(controller.getThermometer(controller.BATTERY_CELL_1_TEMP_NODE));
//		add(controller.getThermometer(controller.BATTERY_CELL_2_TEMP_NODE));
//		add(controller.getThermometer(controller.BATTERY_CELL_3_TEMP_NODE));
//		add(controller.getThermometer(controller.BATTERY_CELL_4_TEMP_NODE));
//		add(controller.getThermometer(controller.BATTERY_CELL_5_TEMP_NODE));
//		add(controller.getThermometer(controller.BATTERY_CELL_6_TEMP_NODE));
		add(controller.getThermometer(controller.CHIPSET_TEMP_NODE));
		add(controller.getThermometer(controller.SOUTHBRIDGE_1_TEMP_NODE));
		add(controller.getThermometer(controller.SOUTHBRIDGE_2_TEMP_NODE));
		add(controller.getThermometer(controller.BMS_TEMP_NODE));
		add(controller.getThermometer(controller.CONVECTOR_TEMP_NODE));
		add(controller.getThermometer(controller.CHIPSET_TEMP_NODE));
		add(controller.getThermometer(controller.POWER_DIST_1_TEMP_NODE));
		
//		add(controller.getThermometer(controller.POWER_DIST_2_TEMP_NODE));
//		add(controller.getThermometer(controller.POWER_DIST_3_TEMP_NODE));

	}


}

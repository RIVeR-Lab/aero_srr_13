package gui;

import instruments.BatteryIndicator;

import java.awt.GridLayout;
import java.util.ArrayList;

import javax.swing.JPanel;

import org.ros.message.MessageListener;
import org.ros.message.OryxMessages.Battery;
import org.ros.message.OryxMessages.Temperature;

import controller.BatteryController;
import controller.TemperatureController;

public class BatteryPanel extends JPanel {
	BatteryController batteryController;
	/**
	 * Create the panel.
	 */
	public BatteryPanel(BatteryController batControl) {
		batteryController=batControl;
		setLayout(new GridLayout(0, 1, 1, 0));	
		add(batteryController.getIndicator(BatteryController.BATTERY_PACK_NODE));
		add(batteryController.getIndicator(BatteryController.BATTERY_CELL_1_NODE));
		add(batteryController.getIndicator(BatteryController.BATTERY_CELL_2_NODE));
		add(batteryController.getIndicator(BatteryController.BATTERY_CELL_3_NODE));
		add(batteryController.getIndicator(BatteryController.BATTERY_CELL_4_NODE));
		add(batteryController.getIndicator(BatteryController.BATTERY_CELL_5_NODE));
		add(batteryController.getIndicator(BatteryController.BATTERY_CELL_6_NODE));
		add(batteryController.getIndicator(BatteryController.BATTERY_CELL_7_NODE));
	}
}

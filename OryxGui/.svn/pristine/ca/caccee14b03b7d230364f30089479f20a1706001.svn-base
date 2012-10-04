package gui;

import instruments.BatteryIndicator;
import instruments.Thermometer;

import java.awt.BorderLayout;
import java.awt.EventQueue;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.border.EmptyBorder;

import controller.BatteryController;
import controller.TemperatureController;

import java.awt.GridLayout;

public class TelemetryGuiFrame extends JFrame {
	private JPanel temperaturePanel;
	private JPanel batteryPanel;
	private JPanel contentPane;

	public BatteryController batteryController = new BatteryController();
	public TemperatureController temperatureController = new TemperatureController();
	/**
	 * Create the frame.
	 */
	public TelemetryGuiFrame() {
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		setBounds(100, 100, 636, 559);
		
		contentPane = new JPanel();
		contentPane.setBorder(new EmptyBorder(5, 5, 5, 5));
		contentPane.setLayout(new BorderLayout(0, 0));
		setContentPane(contentPane);
		
		temperaturePanel = new JPanel();
		getContentPane().add(temperaturePanel, BorderLayout.CENTER);
		temperaturePanel.setLayout(new GridLayout(0, 1, 0, 0));
		
		//Add Thermometers
		temperaturePanel.add(temperatureController.getThermometer(temperatureController.BACK_LEFT_MOTOR_TEMP_NODE));
		temperaturePanel.add(temperatureController.getThermometer(temperatureController.BACK_RIGHT_MOTOR_TEMP_NODE));
		temperaturePanel.add(temperatureController.getThermometer(temperatureController.FRONT_LEFT_MOTOR_TEMP_NODE));
		temperaturePanel.add(temperatureController.getThermometer(temperatureController.FRONT_RIGHT_MOTOR_TEMP_NODE));
		temperaturePanel.add(temperatureController.getThermometer(temperatureController.CORE_1_TEMP_NODE));
		temperaturePanel.add(temperatureController.getThermometer(temperatureController.CORE_2_TEMP_NODE));
		temperaturePanel.add(temperatureController.getThermometer(temperatureController.CORE_3_TEMP_NODE));
		temperaturePanel.add(temperatureController.getThermometer(temperatureController.CORE_4_TEMP_NODE));
		temperaturePanel.add(temperatureController.getThermometer(temperatureController.BATTERY_CELL_1_TEMP_NODE));
		temperaturePanel.add(temperatureController.getThermometer(temperatureController.BATTERY_CELL_2_TEMP_NODE));
		temperaturePanel.add(temperatureController.getThermometer(temperatureController.BATTERY_CELL_3_TEMP_NODE));
		temperaturePanel.add(temperatureController.getThermometer(temperatureController.BATTERY_CELL_4_TEMP_NODE));
		temperaturePanel.add(temperatureController.getThermometer(temperatureController.BATTERY_CELL_5_TEMP_NODE));
		temperaturePanel.add(temperatureController.getThermometer(temperatureController.BATTERY_CELL_6_TEMP_NODE));
		temperaturePanel.add(temperatureController.getThermometer(temperatureController.CHIPSET_TEMP_NODE));
		temperaturePanel.add(temperatureController.getThermometer(temperatureController.POWER_DIST_1_TEMP_NODE));
		temperaturePanel.add(temperatureController.getThermometer(temperatureController.POWER_DIST_2_TEMP_NODE));
		temperaturePanel.add(temperatureController.getThermometer(temperatureController.POWER_DIST_3_TEMP_NODE));
		temperaturePanel.add(temperatureController.getThermometer(temperatureController.SOUTHBRIDGE_1_TEMP_NODE));
		temperaturePanel.add(temperatureController.getThermometer(temperatureController.SOUTHBRIDGE_2_TEMP_NODE));
		temperaturePanel.add(temperatureController.getThermometer(temperatureController.BMS_TEMP_NODE));
		temperaturePanel.add(temperatureController.getThermometer(temperatureController.CONVECTOR_TEMP_NODE));
		temperaturePanel.add(temperatureController.getThermometer(temperatureController.CHIPSET_TEMP_NODE));
		temperaturePanel.add(temperatureController.getThermometer(temperatureController.POWER_DIST_1_TEMP_NODE));
		temperaturePanel.add(temperatureController.getThermometer(temperatureController.POWER_DIST_2_TEMP_NODE));
		temperaturePanel.add(temperatureController.getThermometer(temperatureController.POWER_DIST_3_TEMP_NODE));
		
		batteryPanel = new JPanel();
		getContentPane().add(batteryPanel, BorderLayout.EAST);
		batteryPanel.setLayout(new GridLayout(0, 1, 0, 0));
		
		batteryPanel.add(batteryController.getIndicator(BatteryController.BATTERY_PACK_NODE));
		batteryPanel.add(batteryController.getIndicator(BatteryController.BATTERY_CELL_1_NODE));
		batteryPanel.add(batteryController.getIndicator(BatteryController.BATTERY_CELL_2_NODE));
		batteryPanel.add(batteryController.getIndicator(BatteryController.BATTERY_CELL_3_NODE));
		batteryPanel.add(batteryController.getIndicator(BatteryController.BATTERY_CELL_4_NODE));
		batteryPanel.add(batteryController.getIndicator(BatteryController.BATTERY_CELL_5_NODE));
		batteryPanel.add(batteryController.getIndicator(BatteryController.BATTERY_CELL_6_NODE));
		
	}

}

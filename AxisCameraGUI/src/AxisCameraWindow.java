import java.awt.BorderLayout;
import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.lang.Thread.UncaughtExceptionHandler;

import javax.swing.JFrame;
import javax.swing.JOptionPane;

import axiscamera.AxisCamera;

public class AxisCameraWindow extends JFrame {


	public static final String ADMIN_USERNAME = "root";
	public static final String ADMIN_PASSWORD = "oryx2012";

	public static void main(final String[] args) {
		Thread.setDefaultUncaughtExceptionHandler(new UncaughtExceptionHandler() {
			@Override
			public void uncaughtException(Thread t, Throwable e) {
				e.printStackTrace();
				ByteArrayOutputStream os = new ByteArrayOutputStream();
				e.printStackTrace(new PrintStream(os));
				if(args.length<2 || !args[1].equals("simpleError"))
					JOptionPane.showMessageDialog(null, os.toString(), "Error: "
						+ e, JOptionPane.ERROR_MESSAGE);
				else
					JOptionPane.showMessageDialog(null, e.getMessage(), "Error: "
							+ e, JOptionPane.ERROR_MESSAGE);
			}
		});

		String ipAddress = "192.168.0.90";/*
											 * JOptionPane .showInputDialog(
											 * "Enter the IP Address to connect to:"
											 * );
											 */
		try {
			AxisCamera camera = new AxisCamera(ipAddress, ADMIN_USERNAME, ADMIN_PASSWORD);
			AxisCameraWindow window = new AxisCameraWindow(camera);
			window.setVisible(true);
		} catch (Exception e1) {
			e1.printStackTrace();
		}

	}


	public AxisCameraWindow(AxisCamera camera) {
		super("Axis Camera Viewer");
		setDefaultCloseOperation(EXIT_ON_CLOSE);

		add(new ImageDisplay(camera));
		add(new ControlsPanel(camera), BorderLayout.SOUTH);

		setSize(600, 500);
	}
}

import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.JButton;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import axiscamera.AxisCamera;
import axiscamera.ptz.PTZ;

public class ControlsPanel extends JPanel {
	private AxisCamera camera;

	private JButton jogInButton;
	private JButton jogOutButton;
	private JSlider zoomSlider;

	/*private JButton leftButton;
	private JButton rightButton;
	private JButton upButton;
	private JButton downButton;*/

	public ControlsPanel(AxisCamera camera) {
		this.camera = camera;

		setBorder(BorderFactory.createEmptyBorder(0, 5, 15, 5));
		setLayout(new GridBagLayout());
		GridBagConstraints c = new GridBagConstraints();
		c.fill = GridBagConstraints.BOTH;

		/*c.gridx = 1;
		c.gridy = 0;
		add(upButton = new JButton("\u2191"), c);
		upButton.addActionListener(new MoveActionListener("up"));
		c.gridx = 0;
		c.gridy = 1;
		add(leftButton = new JButton("\u2190"), c);
		leftButton.addActionListener(new MoveActionListener("left"));
		c.gridx = 1;
		add(downButton = new JButton("\u2193"), c);
		downButton.addActionListener(new MoveActionListener("down"));
		c.gridx = 2;
		add(rightButton = new JButton("\u2192"), c);
		rightButton.addActionListener(new MoveActionListener("right"));*/

		c.gridy = 1;
		c.gridx = 3;
		add(Box.createHorizontalStrut(50), c);
		c.gridx = 4;
		add(jogOutButton = new JButton("Zoom -"), c);
		jogOutButton.addActionListener(new JogZoomActionListener(-500));
		c.gridx = 5;
		add(jogInButton = new JButton("Zoom +"), c);
		jogInButton.addActionListener(new JogZoomActionListener(500));
		c.gridx = 6;
		c.insets = new Insets(0, 10, 0, 0);
		add(zoomSlider = new JSlider(1, 9999, 1), c);
		zoomSlider.setMajorTickSpacing(1000);
		zoomSlider.setPaintTicks(true);
		zoomSlider.addChangeListener(new ChangeListener() {
			@Override
			public void stateChanged(ChangeEvent e) {
				if (!zoomSlider.getValueIsAdjusting())
					PTZ.zoom(ControlsPanel.this.camera, zoomSlider.getValue());
			}
		});
	}

	private class JogZoomActionListener implements ActionListener {
		private int dist;

		JogZoomActionListener(int dist) {
			this.dist = dist;
		}

		@Override
		public void actionPerformed(ActionEvent e) {
			PTZ.jogZoom(camera, dist);
		}
	}

	/*private class MoveActionListener implements ActionListener {
		private String command;

		MoveActionListener(String command) {
			this.command = command;
		}

		@Override
		public void actionPerformed(ActionEvent e) {
			PTZ.move(camera, command);
		}
	}*/
}

package axiscamera.image;

import java.util.Map;

import axiscamera.AxisCamera;

public class RotationConfig implements ImageConfig{
	private String rotation;

	public RotationConfig(int rotation) {
		this(Integer.toString(rotation));
	}

	public RotationConfig(String rotation) {
		this.rotation = rotation;
	}
	
	public String toString(){
		return rotation;
	}

	public static RotationConfig[] getSupported(AxisCamera camera) {
		String[] supportedRotationNames = camera.listParam("Properties.Image.Rotation").get("Properties.Image.Rotation").split(",");
		RotationConfig[] supportedRotations = new RotationConfig[supportedRotationNames.length];
		for(int i = 0; i<supportedRotationNames.length; ++i){
			supportedRotations[i] = new RotationConfig(supportedRotationNames[i]);
		}
		return supportedRotations;
	}

	public void applyHttp(Map<String, String> args) {
		args.put("rotation", rotation);
	}

	public void applyRstp(Map<String, String> args) {
		args.put("rotation", rotation);
	}
}

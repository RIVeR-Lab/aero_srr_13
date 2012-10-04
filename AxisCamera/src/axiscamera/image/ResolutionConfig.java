package axiscamera.image;

import java.util.Map;

import axiscamera.AxisCamera;

public class ResolutionConfig implements ImageConfig{
	private String res;

	public ResolutionConfig(int width, int height) {
		this(width+"x"+height);
	}

	public ResolutionConfig(String res) {
		this.res = res;
	}
	
	public String toString(){
		return res;
	}

	public static ResolutionConfig[] getSupported(AxisCamera camera) {
		String[] supportedResolutionNames = camera.listParam("Properties.Image.Resolution").get("Properties.Image.Resolution").split(",");
		ResolutionConfig[] supportedResolutions = new ResolutionConfig[supportedResolutionNames.length];
		for(int i = 0; i<supportedResolutionNames.length; ++i){
			supportedResolutions[i] = new ResolutionConfig(supportedResolutionNames[i]);
		}
		return supportedResolutions;
	}

	public void applyHttp(Map<String, String> args) {
		args.put("resolution", res);
	}

	public void applyRstp(Map<String, String> args) {
		args.put("resolution", res);
	}
}

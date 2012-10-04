package axiscamera.image;

import java.util.Map;

import axiscamera.AxisCameraException;

public class CompressionConfig implements ImageConfig{
	private int compression;

	public CompressionConfig(int compression) {
		if(compression>100||compression<0)
			throw new AxisCameraException("Compression must be between 0 and 100 inclusive");
		this.compression = compression;
	}
	
	public String toString(){
		return Integer.toString(compression);
	}

	public void applyHttp(Map<String, String> args) {
		args.put("compression", toString());
	}

	public void applyRstp(Map<String, String> args) {
		args.put("compression", toString());
	}
}

package axiscamera.image;

import java.util.Map;




public class JPEGConfig implements ImageConfig {
	
	private ResolutionConfig resolution;
	private CompressionConfig compression;
	private RotationConfig rotation;
	public JPEGConfig(ResolutionConfig resolution, CompressionConfig compression, RotationConfig rotation){
		this.resolution = resolution;
		this.compression = compression;
		this.rotation = rotation;
	}

	public void applyHttp(Map<String, String> args) {
		if(resolution!=null)
			resolution.applyHttp(args);
		if(compression!=null)
			compression.applyHttp(args);
		if(rotation!=null)
			rotation.applyHttp(args);
	}

	@Override
	public void applyRstp(Map<String, String> args) {
		if(resolution!=null)
			resolution.applyRstp(args);
		if(compression!=null)
			compression.applyRstp(args);
		if(rotation!=null)
			rotation.applyRstp(args);
	}

}

package axiscamera.image;

import java.util.Map;

public class VideoConfig implements ImageConfig {
	
	private int duration;
	private int fps;

	public VideoConfig(int fps){
		this(0, fps);
	}

	public VideoConfig(int duration, int fps){
		this.duration = duration;
		this.fps = fps;
	}

	@Override
	public void applyHttp(Map<String, String> args) {
		if(duration>=0)
			args.put("duration", Integer.toString(duration));
		if(fps>=0)
			args.put("fps", Integer.toString(fps));
	}

	@Override
	public void applyRstp(Map<String, String> args) {
		if(duration>=0)
			args.put("duration", Integer.toString(duration));
		if(fps>=0)
			args.put("fps", Integer.toString(fps));
	}
	
}

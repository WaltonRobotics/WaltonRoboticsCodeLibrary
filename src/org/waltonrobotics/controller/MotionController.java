package org.waltonrobotics.controller;

import java.util.Collections;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.BlockingDeque;
import java.util.concurrent.LinkedBlockingDeque;

import org.waltonrobotics.DriveTrain;
import org.waltonrobotics.motion.Path;

public class MotionController {
	
	private class MotionTask extends TimerTask{
		
		@Override
		public void run() {
			Vector2 vec2 = motions.poll();
			DriveTrain.setSpeeds(vec2.getLeftVelocity(), vec2.getRightVelocity());
		}
		
	}
	
	private BlockingDeque<Vector2> motions = new LinkedBlockingDeque<>();
	private Timer velocityScheduler;
	
	boolean running;
	
	public MotionController(long period) {
		velocityScheduler = new Timer();		
		enableScheduler(period);
	}
	
	public void addPaths(Path... paths) {
		for(Path path: paths)
		{
			Collections.addAll(motions, path.getSpeedVectors());
		}
	}
	
	public void stopScheduler() {
		velocityScheduler.cancel();
		running = false;
	}
	
	public void enableScheduler(long period) {
		if(!running) {
		velocityScheduler.scheduleAtFixedRate(new MotionTask(), 0L, period);
		running = true;
		}
	}
	
	public void clearMotions()
	{
		motions.clear();
	}
	
	public boolean isRunning() {
		return running;
	}
}

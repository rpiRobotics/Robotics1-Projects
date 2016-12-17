package BoundingClasses;
import root.Main;

public class BoundedIntFrameWidth{
	public int value;
	public static final int minimum = 0;
	public static final int maximum = Main.x_resolution - 1;
	
	public BoundedIntFrameWidth(int value){
		init(value);
	}
	
	public BoundedIntFrameWidth(){
		init(0);
	}
	
	public void init(int value){
		this.value = value;
	}
	
}
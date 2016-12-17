package BoundingClasses;

import root.Main;

public class BoundedIntFrameHeight {
	public int value;
	public static final int minimum = 0;
	public static final int maximum = Main.y_resolution - 1;

	public BoundedIntFrameHeight(int value) {
		init(value);
	}

	public BoundedIntFrameHeight() {
		init(0);
	}

	public void init(int value) {
		this.value = value;
	}

	public String getPython(String varName) {
		String pythonText = varName + " = BoundedIntFrameHeight(" + String.valueOf(value) + ")\n";
		pythonText += "print(" + varName + ".value)\n";
		return pythonText;
	}

}
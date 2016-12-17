package root;
/**
 * The Pixel class is a struct type class that hold the values for red/green/blue for a pixel.
 * This class supports individual pixels of color depth up to the max value for an int, not an unsigned int. 
 * @author Ryan Feldhausen
 */
public class Pixel {
	/**
	 * The red color of this pixel. Ex somewhere between 0-255
	 */
	private int  red;
	/**
	 * The green color of this pixel. Ex somewhere between 0-255
	 */
	private int green;
	/**
	 * The blue color of this pixel. Ex somewhere between 0-255
	 */
	private int blue;
	/**
	 * The maximum value of each color Ex (0 - maxValue) or (0 - 255)
	 */
	private final int maxValue;
	
	/**
	 * Initialize a pixel with given red, green, and blue colors.
	 * @param red int value for red
	 * @param green int value for green
	 * @param blue int value for blue
	 * @param maxValue an integer representing the range (0 - maxValue) for each color.
	 * @throws Exception Pixel Component Out of Bounds
	 */
	public Pixel(int red, int green, int blue, int maxValue) throws Exception{
		// maxValue must be set before red, green, and blue because of bounds checking.
		this.maxValue = maxValue;
		this.setRed(red);
		this.setGreen(green);
		this.setBlue(blue);
	}

	/**
	 * Gets the red value for this pixel.
	 * @return int The value of the pixel from range 0 - maxValue 
	 */
	public int getRed() {
		return red;
	}

	/**
	 * Sets the red value for this pixel.
	 * @param red int limited by the maxValue variable
	 * @throws Exception Pixel Component Out of Bounds
	 */
	public void setRed(int red) throws Exception {
		if (red > maxValue){
			throw new Exception("Pixel Component Out of Bounds (red > maxValue)\n");
		}
		if (red < 0){
			throw new Exception("Pixel Component Out of Bounds (red < 0)\n");
		}
		this.red = red;
	}

	/**
	 * Gets the green value for this pixel.
	 * @return int The value of the pixel from range 0 - maxValue 
	 */
	public int getGreen() {
		return green;
	}

	/**
	 * Sets the green value for this pixel.
	 * @param green int limited by the maxValue variable
	 * @throws Exception Pixel Component Out of Bounds
	 */
	public void setGreen(int green) throws Exception {
		if (green > maxValue){
			throw new Exception("Pixel Component Out of Bounds (red > maxValue)\n");
		}
		if (green < 0){
			throw new Exception("Pixel Component Out of Bounds (red < 0)\n");
		}
		this.green = green;
	}
	
	/**
	 * Gets the blue value for this pixel.
	 * @return int The value of the pixel from range 0 - maxValue 
	 */
	public int getBlue() {
		return blue;
	}

	/**
	 * Sets the blue value for this pixel.
	 * @param blue int limited by the maxValue variable
	 * @throws Exception Pixel Component Out of Bounds
	 */
	public void setBlue(int blue) throws Exception {
		if (blue > maxValue){
			throw new Exception("Pixel Component Out of Bounds (blue > maxValue)\n");
		}
		if (blue < 0){
			throw new Exception("Pixel Component Out of Bounds (blue < 0)\n");
		}
		this.blue = blue;
	}
	
	/**
	 * Returns the maximum integer value for this pixel.
	 * Range (0 - maxValue)
	 * @return int the maximum permissible value for a pixel's individual color component.
	 */
	public int getMaxValue(){
		return maxValue;
	}
}

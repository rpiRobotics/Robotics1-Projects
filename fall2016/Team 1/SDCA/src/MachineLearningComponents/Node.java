package MachineLearningComponents;
import java.util.ArrayList;

import MachineLearningFramework.AbstractMethod;
import MachineLearningFramework.ReflectionHelper;
import root.Pixel;

/**
 * A node is a group of pixels. Nodes contain a set of group-values that can be 
 * accessed by the machine learning algorithm.
 * @author Ryan Feldhausen
 *
 */
public class Node {
	/**
	 * prefix of the methods that the genetic algorithm will call.
	 */
	private static final String reflectionPrefix = "determine";
	/**
	 * A list of the pixels inside this node
	 */
	private ArrayList<Pixel> pixelsInNode = null;
	/**
	 * This class abstracts away the maintenance for adding/removing operations for the 
	 * genetic algorithm.
	 */
	private ReflectionHelper nodeReflectionHelper;
	
	private int reflectionMethodCount;
	
	/**
	 * Initialize a node with a set pixels.
	 * This is the only way to set the pixels inside the node. 
	 * Setting a node is limited to initialization because the machine
	 * learning algorithm will not modify the contents of a node.
	 * @param pixels
	 * @throws Exception Node Pixel Types
	 * @throws Exception Node Given 0 Pixels
	 */
	public Node(ArrayList<Pixel> pixels) throws Exception{
		this.pixelsInNode = pixels;
		// Ensure that this node has at least one pixel.
		if (pixels.size() < 1){
			throw new Exception("Node Given 0 Pixels\n");
		}
		// Ensure that the pixels all have the same range.
		int initialRange = pixels.get(0).getMaxValue();
		for (Pixel pixel : pixels){
			if (pixel.getMaxValue() != initialRange){
				throw new Exception("Node Pixel Types (maxValues are not equal)\n");
			}
		}
		
		ReflectionHelper nodeReflectionHelper = new ReflectionHelper(this, reflectionPrefix);
		this.nodeReflectionHelper = nodeReflectionHelper;
		this.reflectionMethodCount = nodeReflectionHelper.getMethodCount();
	}

	/**
	 * This will execute one of the "Node.determineXXXXX()" methods below.
	 * We don't care which one is executed, as long as it is consistent. The machine will
	 * do the learning, not us. 
	 * @param x The method to execute. A static X will always access the same method.
	 * @return The result from the method that was invoked.
	 */
	public int executeMethodX(int x){
		return (int) nodeReflectionHelper.executeMethodX(x);
	}
	
	public int getReflectionMethodCount(){
		return reflectionMethodCount;
	}
	
	public AbstractMethod characterizeMethodX(int x){
		AbstractMethod characterizedMethod = nodeReflectionHelper.characterizeMethodX(x);
		return characterizedMethod;
	}
	
	//----------------------------------------------------------
	//------------MACHINE LEARNING METHODS----------------------
	//----------------------------------------------------------
	
	// NOTE: As long as the code above is not changed. methods
	// here can be added and removed at will as long as they obey
	// the following rules. The rest of the program will automatically
	// handle new methods added here.
	// A: start with "get"
	// B: return an integer
	// C: have no arguments
	// Whatever goes on inside the method is fair game.
	
	/**
	 * Get the average red value of all pixels inside this node.
	 * @return int
	 */
	public int determineAverageRed(){
		int sum = 0;
		for (Pixel pixel : pixelsInNode){
			sum += pixel.getRed();
		}
		int average = sum / pixelsInNode.size();
		return average;
	}

	/**
	 * Get the average green value of all pixels inside this node
	 * @return int
	 */
	public int determineAverageGreen(){
		int sum = 0;
		for (Pixel pixel : pixelsInNode){
			sum += pixel.getGreen();
		}
		int average = sum / pixelsInNode.size();
		return average;
	}

	/**
	 * Get the average blue value of all pixels inside this node
	 * @return int
	 */
	public int determineAverageBlue(){
		int sum = 0;
		for (Pixel pixel : pixelsInNode){
			sum += pixel.getBlue();
		}
		int average = sum / pixelsInNode.size();
		return average;
	}
}

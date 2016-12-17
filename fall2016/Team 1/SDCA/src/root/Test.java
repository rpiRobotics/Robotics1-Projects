package root;

import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Random;

import MachineLearningComponents.Node;
import MachineLearningComponents.NodeGenerator;
import MachineLearningComponents.Operation;
import MachineLearningFramework.AbstractMethod;
import MachineLearningFramework.Organism;

/**
 * Test.java is essentially a container for a number of self contained tests.
 * These tests will test/demo the correct operation of other classes in this
 * program.
 * 
 * Due to the abstraction of the program. A crash-less run = a successful test.
 * 
 * @author Ryan
 *
 */
public class Test {

	public Test() {
		// Do nothing here
	}

	/**
	 * This function will test out the operation class by calling all methods
	 * with sample integers and ensuring that nothing crashes during normal
	 * operation
	 * 
	 * @param printOutput
	 *            If true, the test will print its operation to the command
	 *            line.
	 * @throws InvocationTargetException
	 * @throws IllegalArgumentException
	 * @throws IllegalAccessException
	 */
	public void testOperation(boolean printOutput)
			throws IllegalAccessException, IllegalArgumentException, InvocationTargetException {
		// Instantiate the operation class
		Operation operationTestInstance = new Operation();
		// Obtain the number of machine-learning methods for the operation class
		int operationMethodCount = operationTestInstance.getReflectionMethodCount();
		if (printOutput) {
			System.out.printf("testOperation : Methods Found = %d\n", operationMethodCount);
		}
		// Pick some random integers to test.
		int a = 17;
		int b = 7;
		// Loop through all of the found methods, extract their names and call
		// them with the parameters a and b.
		for (int x = 0; x < operationMethodCount; x++) {
			// Characterize the method (i.e. get the method's metadata).
			AbstractMethod methodX = operationTestInstance.characterizeMethodX(x);
			// Pull out the method name
			String methodXName = methodX.method.getName();
			// Invoke the method
			int c = (int) methodX.method.invoke(methodX.owner, a, b);
			// Print output if requested
			if (printOutput) {
				// Padding in order to have the debug ouput look pretty.
				int paddingToAdd = 18 - methodXName.length();
				StringBuffer paddingBuffer = new StringBuffer(paddingToAdd);
				for (int y = 0; y < paddingToAdd; y++) {
					paddingBuffer.append(" ");
				}

				System.out.printf("testOperation : %s (%d) %s Arguments (%d, %d) returned %d\n", methodXName, x,
						paddingBuffer.toString(), a, b, c);
			}
		}
	}

	/**
	 * This function will test out the node class by calling all methods
	 * and ensuring that nothing crashes during normal operation
	 * 
	 * @param printOutput
	 *            If true, the test will print its operation to the command
	 *            line.
	 * @throws Exception
	 */
	public void testNode(boolean printOutput) throws Exception {
		// Create two pixels
		Pixel testPixelA = new Pixel(100, 125, 150, 255);
		Pixel testPixelB = new Pixel(115, 130, 145, 255);

		// Create a node with these two pixels
		ArrayList<Pixel> testPixelArray = new ArrayList<Pixel>();
		testPixelArray.add(testPixelA);
		testPixelArray.add(testPixelB);
		Node testNode = new Node(testPixelArray);

		int nodeMethodCount = testNode.getReflectionMethodCount();
		if (printOutput) {
			System.out.printf("testNode : Methods Found = %d\n", nodeMethodCount);
		}

		// Loop through all of the found methods, extract their names and call
		// them with the parameters a and b.
		for (int x = 0; x < nodeMethodCount; x++) {
			// Characterize the method (i.e. get the method's metadata).
			AbstractMethod methodX = testNode.characterizeMethodX(x);
			// Pull out the method name
			String methodXName = methodX.method.getName();
			// Invoke the method
			int c = (int) methodX.method.invoke(methodX.owner);
			// Print output if requested
			if (printOutput) {
				// Padding in order to have the debug ouput look pretty.
				int paddingToAdd = 25 - methodXName.length();
				StringBuffer paddingBuffer = new StringBuffer(paddingToAdd);
				for (int y = 0; y < paddingToAdd; y++) {
					paddingBuffer.append(" ");
				}

				System.out.printf("testNode : %s (%d) %s Arguments (None) returned %d\n", methodXName, x,
						paddingBuffer.toString(), c);
			}
		}
	}

	public void testNodeGenerator(boolean printOutput) throws Exception {
		// Initialize a random number generator. This is used to create random
		// arguments for the NodeGenerator methods.
		Random rng = new Random();

		// Initialize a frame by creating a nested array of pixels.
		ArrayList<ArrayList<Pixel>> image = new ArrayList<ArrayList<Pixel>>();
		for (int x = 0; x < Main.x_resolution; x++) {
			ArrayList<Pixel> rowOfPixels = new ArrayList<Pixel>();
			for (int y = 0; y < Main.y_resolution; y++) {
				Pixel newPixel = new Pixel(100, 200, 155, 255);
				rowOfPixels.add(newPixel);
			}
			image.add(rowOfPixels);
		}
		Frame testFrame = new Frame(image);

		// Initialize the node generator
		NodeGenerator testNodeGenerator = new NodeGenerator(testFrame);

		// Count the node generator methods
		int nodeGeneratorMethodCount = testNodeGenerator.getReflectionMethodCount();
		if (printOutput) {
			System.out.printf("testNodeGenerator : Methods Found = %d\n", nodeGeneratorMethodCount);
		}

		// Loop through each method of the node generator...
		for (int x = 0; x < nodeGeneratorMethodCount; x++) {

			// Characterize a node to find out out its parameter count and
			// parameter types
			AbstractMethod characterizedMethod = testNodeGenerator.characterizeMethodX(x);
			int parameterCount = characterizedMethod.parameterCount;
			Class<?>[] parameterTypes = characterizedMethod.parameterTypes;
			Class<?> returnType = characterizedMethod.returnType;

			// Pull out the method name
			String methodXName = characterizedMethod.method.getName();

			if (printOutput) {
				System.out.printf("testNodeGenerator : %s parameter count = %d\n", methodXName, parameterCount);
				System.out.printf("testNodeGenerator : %s return type = %s\n", methodXName,
						returnType.getCanonicalName());
			}

			// Initialize an array of objects that will contain the function
			// arguments.
			Object[] functionArgs = new Object[parameterCount];

			// Characterize each parameter by finding its class name and its
			// min/max
			// Min/Max and the type of Min/Max is PRE-PROGRAMED
			// The bounds of a node are not abstracted away. Take note.
			for (int y = 0; y < parameterCount; y++) {
				// Get the types of this specific function argument.
				Class<?> parameter = parameterTypes[y];

				if (printOutput) {
					String parameterName = parameter.getSimpleName();
					System.out.printf("testNodeGenerator : %s Parameter %d is %s\n", methodXName, y, parameterName);
				}

				// Get the minimum and maximum of the argument (we know it is an
				// integer)
				int minimum = (int) parameter.getDeclaredField("minimum").get(Integer.class);
				int maximum = (int) parameter.getDeclaredField("maximum").get(Integer.class);

				if (printOutput) {
					System.out.printf("testNodeGenerator : %s Parameter %d Minimum = %d Maximum = %d\n", methodXName, y,
							minimum, maximum);
				}

				// Instantiate a new instance of this argument's type.
				Object argumentObject = parameter.newInstance();

				// Pick a random value that obeys the Min/Max constraints
				int value = rng.nextInt(maximum - minimum) + minimum;

				// Set the argument's "value" field. (Hard-coded)
				argumentObject.getClass().getDeclaredField("value").setInt(argumentObject, value);

				// Add this argument to the array of arguments.
				functionArgs[y] = argumentObject;
			}

			// Invoke the method and grab the return node (Return value is hard
			// coded)
			Node returnNode = (Node) characterizedMethod.method.invoke(characterizedMethod.owner, functionArgs);

			// The returnNode can be null if we randomly provided bad inputs.
			// This can happen even with a set minimum/maximum because the
			// min/max is set to be "generally accurate"
			if (returnNode != null) {
				int returnNodeMethods = returnNode.getReflectionMethodCount();

				for (int y = 0; y < returnNodeMethods; y++) {
					int returnNodeReturnValue = returnNode.executeMethodX(y);
					String methodYName = returnNode.characterizeMethodX(y).method.getName();

					if (printOutput) {
						System.out.printf("testNodeGenerator : %s %s returned %d\n", methodXName, methodYName,
								returnNodeReturnValue);
					}
				}
			} else {
				if (printOutput) {
					System.out.printf("testNodeGenerator : %s was null\n", methodXName);
				}
			}

		}
	}

	public void testOrganism(boolean printOutput) throws Exception {
		Organism testOrganism = new Organism(1334);
		if (printOutput) {
			System.out.println("-----Test Organism-----");
		}
		testOrganism.define();
		if (printOutput) {
			System.out.print(testOrganism.getPython());
		}
	}
}

package MachineLearningFramework;

import java.util.ArrayList;
import java.util.Random;

import MachineLearningComponents.Node;
import MachineLearningComponents.NodeGenerator;
import MachineLearningComponents.Operation;
import OrganismInstructions.GenerateCoordinate;
import OrganismInstructions.GenerateNode;
import OrganismInstructions.GenerateOperation;
import OrganismInstructions.GenerateVariable;
import OrganismInstructions.GenericInstruction;
import OrganismInstructions.InitializeNodeGenerator;
import OrganismInstructions.InitializeOperationMachine;
import root.Pixel;

/**
 * Organisms are individual complete algorithms.
 * 
 * @author feldh
 *
 */
public class Organism {
	private final int seed;

	private enum operation_return {
		VARIABLE, COORDINATE
	}

	private enum operation_param {
		VARIABLE, COORDINATE, NODE
	}

	private enum new_old {
		NEW, OLD
	}

	private static final int INITIAL_INSTRUCTION_COUNT = 25;
	private static final int VARIABLE_INITIALIZATION_MINIMUM = -100;
	private static final int VARIABLE_INITIALIZATION_MAXIMUM = 100;
	private static final String node_generator_argument = "frame";

	private ArrayList<GenericInstruction> instructions;
	private ArrayList<GenerateNode> nodes;
	private ArrayList<GenerateVariable> variables;
	private ArrayList<GenerateCoordinate> coordinates;

	private Random rng;
	private int generateNodeMethodCount;
	private int instructionCount;

	public Organism(int seed) {
		this.seed = seed;
		init();
	}

	public void init() {
		this.rng = new Random(seed);
		this.instructions = new ArrayList<GenericInstruction>();
		this.nodes = new ArrayList<GenerateNode>();
		this.variables = new ArrayList<GenerateVariable>();
		this.coordinates = new ArrayList<GenerateCoordinate>();

		NodeGenerator temp = new NodeGenerator(null);
		this.generateNodeMethodCount = temp.getReflectionMethodCount();

		this.instructionCount = 0;
	}

	/**
	 * This function will construct the overall data of the organism.
	 * It defines every variable etc in a format that is easily converted
	 * into the executable python representation.
	 * 
	 * @throws Exception
	 */
	public void define() throws Exception {
		// Initialize the node generator
		instructionCount = initializeNodeGenerator(instructionCount);

		// Initialize the operation "machine?"
		instructionCount = initializeOperationMachine(instructionCount);

		// Initialize the coordinates (Do twice because for loop is longer)
		instructionCount = initializeCoordinate(instructionCount);
		instructionCount = initializeCoordinate(instructionCount);

		// Add some operations
		for (int x = 0; x < INITIAL_INSTRUCTION_COUNT; x++) {
			addRandomOperation();
		}
	}

	public String getPython() {
		String python = "";
		for (int x = 0; x < instructions.size(); x++) {
			python = python + instructions.get(x).getPythonInstruction();
		}
		return python;
	}

	public void addRandomOperation() throws Exception {
		String paramA;
		String paramB;
		String retVal;

		// Choose parameter type 1 / 2
		// Choose a random int that represents a random enumeration value
		int paramEnumIndexA = rng.nextInt(operation_param.values().length);
		int paramEnumIndexB = rng.nextInt(operation_param.values().length);
		// Take the ranom int and change it to the enumeration
		operation_param paramEnumA = operation_param.values()[paramEnumIndexA];
		operation_param paramEnumB = operation_param.values()[paramEnumIndexB];

		// Choose return type
		int returnEnumIndex = rng.nextInt(operation_return.values().length);
		operation_return returnEnum = operation_return.values()[returnEnumIndex];

		// Send the enumeration into the getParam function.
		// This function will return a different string result based on
		// (RNG/Enumeration Type)
		// See the getParam function for details.
		paramA = getParam(paramEnumA);
		paramB = getParam(paramEnumB);

		// Send the retVal enumeration into the getRetVal function.
		// This function will return a different string result based on
		// (RNG/Enumeration Type)
		// See the getRetVal function for details
		retVal = getRetVal(returnEnum);

		// Create a random operation and feed it retVal, paramA and paramB
		Operation tempOperation = new Operation();
		int methodCount = tempOperation.getReflectionMethodCount();
		int methodId = rng.nextInt(methodCount);
		instructionCount = initializeOperation(instructionCount, retVal, paramA, paramB, 1, methodId);
	}

	private int initializeNodeGenerator(int instructionCount) {
		InitializeNodeGenerator nodeGen = new InitializeNodeGenerator(instructionCount, node_generator_argument);
		instructions.add(nodeGen);
		// System.out.print(nodeGen.getPythonInstruction());
		return instructionCount + 1;
	}

	private int initializeOperationMachine(int instructionCount) {
		InitializeOperationMachine operationMachine = new InitializeOperationMachine(instructionCount);
		instructions.add(operationMachine);
		// System.out.print(operationMachine.getPythonInstruction());
		return instructionCount + 1;
	}

	private int initializeNode(int instructionCount, int nodeIdentifier) throws Exception {
		ArrayList<Integer> arguments = new ArrayList<Integer>();

		// Create a test node to pull reflection data from.
		NodeGenerator tempNode = new NodeGenerator(null);

		// Pull out method count and pick a random method.
		int methodCount = tempNode.getReflectionMethodCount();
		int randomMethod = rng.nextInt(methodCount);

		// Characterize method to obtain parameter count/types
		AbstractMethod methodX = tempNode.characterizeMethodX(randomMethod);
		int parameterCount = methodX.parameterCount;
		Class<?>[] parameterTypes = methodX.parameterTypes;

		// Loop through each parameter, get min/max.
		for (int x = 0; x < parameterCount; x++) {
			int minimum = (int) parameterTypes[x].getDeclaredField("minimum").get(Integer.class);
			int maximum = (int) parameterTypes[x].getDeclaredField("maximum").get(Integer.class);

			// Pick a random value within the minimum/maximum
			int value = rng.nextInt(maximum - minimum) + minimum;

			// Set this random value to the parameter value
			arguments.add(value);
		}

		GenerateNode node = new GenerateNode(instructionCount, nodeIdentifier, randomMethod, arguments);
		instructions.add(node);
		nodes.add(node);
		// System.out.print(node.getPythonInstruction());
		return instructionCount + 1;
	}

	private int initializeVariable(int instructionCount) {
		// Pick a value in the configured initialization range
		int value = rng.nextInt(VARIABLE_INITIALIZATION_MAXIMUM - VARIABLE_INITIALIZATION_MINIMUM)
				+ VARIABLE_INITIALIZATION_MINIMUM;
		GenerateVariable variable = new GenerateVariable(instructionCount, value);
		instructions.add(variable);
		variables.add(variable);
		// System.out.print(variable.getPythonInstruction());
		return instructionCount + 1;
	}

	private int initializeCoordinate(int instructionCount) {
		GenerateCoordinate coordinate = new GenerateCoordinate(instructionCount);
		instructions.add(coordinate);
		coordinates.add(coordinate);
		// System.out.print(coordinate.getPythonInstruction());
		return instructionCount + 1;
	}

	private int initializeOperation(int instructionCount, String retVal, String paramA, String paramB,
			int operationMachineIdentifier, int methodId) {
		GenerateOperation operation = new GenerateOperation(instructionCount, retVal, paramA, paramB,
				operationMachineIdentifier, methodId);
		instructions.add(operation);
		return instructionCount + 1;
	}

	private String getRetVal(operation_return returnEnum) {
		String retVal = "";
		int new_old_index;
		new_old new_old_value;

		switch (returnEnum) {
		case VARIABLE:
			// Choose new or old variale
			if (variables.size() > 0) {
				new_old_index = rng.nextInt(new_old.values().length);
				new_old_value = new_old.values()[new_old_index];
			} else {
				new_old_value = new_old.NEW;
			}
			switch (new_old_value) {
			case NEW:
				// Initialize a new variable
				retVal = "i" + String.valueOf(instructionCount);
				instructionCount = initializeVariable(instructionCount);
				break;
			case OLD:
				// Choose which old variable to use
				int variableIndex = rng.nextInt(variables.size());
				GenerateVariable variable = variables.get(variableIndex);
				retVal = "i" + String.valueOf(variable.getIdentifier());
				break;
			}
			break;
		case COORDINATE:
			// Choose which coordinate to use
			int coordinateIndex = rng.nextInt(coordinates.size());
			GenerateCoordinate coordinate = coordinates.get(coordinateIndex);
			retVal = "c" + String.valueOf(coordinate.getIdentifier());
			break;
		}
		return retVal;
	}

	private String getParam(operation_param paramEnum) throws Exception {
		String param = "";
		int new_old_index;
		new_old new_old_value;

		switch (paramEnum) {
		case VARIABLE:
			// Choose new or old variable
			if (variables.size() > 0) {
				new_old_index = rng.nextInt(new_old.values().length);
				new_old_value = new_old.values()[new_old_index];
			} else {
				new_old_value = new_old.NEW;
			}
			switch (new_old_value) {
			case NEW:
				// Create a new variable and set it to this parameter
				param = "i" + String.valueOf(instructionCount);
				instructionCount = initializeVariable(instructionCount);
				break;
			case OLD:
				// Choose which old variable to use
				int variableIndex = rng.nextInt(variables.size());
				GenerateVariable variable = variables.get(variableIndex);
				param = "i" + String.valueOf(variable.getIdentifier());
				break;
			}
			break;
		case COORDINATE:
			// Choose which coordinate to use
			int coordinateIndex = rng.nextInt(coordinates.size());
			GenerateCoordinate coordinate = coordinates.get(coordinateIndex);
			param = "c" + String.valueOf(coordinate.getIdentifier());
			break;
		case NODE:
			// Choose new or old node
			if (nodes.size() > 0) {
				new_old_index = rng.nextInt(new_old.values().length);
				new_old_value = new_old.values()[new_old_index];
			} else {
				new_old_value = new_old.NEW;
			}
			String nodeId;
			String nodeMethodName;
			Node tempNode;
			int nodeMethodCount;
			int nodeMethod;
			Pixel tempPixel;
			ArrayList<Pixel> tempNodeContent;
			AbstractMethod methodX;
			switch (new_old_value) {
			case NEW:
				// Create a new node and choose what to use from the node
				nodeId = "i" + String.valueOf(instructionCount);
				instructionCount = initializeNode(instructionCount, 0);
				// Choose which method to use
				tempPixel = new Pixel(0, 0, 0, 10);
				tempNodeContent = new ArrayList<Pixel>();
				tempNodeContent.add(tempPixel);
				tempNode = new Node(tempNodeContent);
				nodeMethodCount = tempNode.getReflectionMethodCount();
				nodeMethod = rng.nextInt(nodeMethodCount);
				methodX = tempNode.characterizeMethodX(nodeMethod);
				nodeMethodName = methodX.method.getName();
				param = nodeId + "." + nodeMethodName + "()";
				break;
			case OLD:
				// Choose which old node to use
				int nodeIndex = rng.nextInt(nodes.size());
				GenerateNode node = nodes.get(nodeIndex);
				nodeId = "i" + String.valueOf(node.getIdentifier());
				// Choose which method to use
				tempPixel = new Pixel(0, 0, 0, 10);
				tempNodeContent = new ArrayList<Pixel>();
				tempNodeContent.add(tempPixel);
				tempNode = new Node(tempNodeContent);
				nodeMethodCount = tempNode.getReflectionMethodCount();
				nodeMethod = rng.nextInt(nodeMethodCount);
				methodX = tempNode.characterizeMethodX(nodeMethod);
				nodeMethodName = methodX.method.getName();
				param = nodeId + "." + nodeMethodName + "()";
				break;
			}
			break;
		}
		return param;
	}
}

package OrganismInstructions;

import java.util.ArrayList;

import MachineLearningComponents.NodeGenerator;
import MachineLearningFramework.AbstractMethod;

public class GenerateNode extends GenericInstruction {
	private int nodeIdentifier;
	private int methodId;
	private ArrayList<Integer> arguments;

	public GenerateNode(int identifier, int nodeIdentifier, int methodId, ArrayList<Integer> arguments) {
		this.identifier = identifier;
		this.nodeIdentifier = nodeIdentifier;
		this.methodId = methodId;
		this.arguments = arguments;
	}

	@Override
	public String getPythonInstruction() {
		// y = n.f(a)
		String y = "i" + String.valueOf(identifier);
		String n = "i" + String.valueOf(nodeIdentifier);
		NodeGenerator temp = new NodeGenerator(null);
		AbstractMethod methodX = temp.characterizeMethodX(methodId);
		String methodName = methodX.method.getName();
		String f = methodName;
		String args = "";
		for (int x = 0; x < arguments.size(); x++) {
			args = args + "," + arguments.get(x);
		}
		String a = args.substring(1);

		return y + " = " + n + "." + f + "(" + a + ")\n";
	}

}

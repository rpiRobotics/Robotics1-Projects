package OrganismInstructions;

public class GenerateVariable extends GenericInstruction {
	private int initialValue;

	public GenerateVariable(int identifier, int initialValue) {
		this.identifier = identifier;
		this.initialValue = initialValue;
	}

	@Override
	public String getPythonInstruction() {
		// y = x
		String y = "i" + String.valueOf(identifier);
		String x = String.valueOf(initialValue);
		return y + " = " + x + "\n";
	}

}

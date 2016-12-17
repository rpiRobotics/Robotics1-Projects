package OrganismInstructions;

public class GenerateCoordinate extends GenericInstruction {

	public GenerateCoordinate(int identifier) {
		this.identifier = identifier;
	}

	@Override
	public String getPythonInstruction() {
		// y = x
		String y = "c" + String.valueOf(identifier);
		String x = String.valueOf(0);
		return y + " = " + x + "\n";
	}

}

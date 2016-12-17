package OrganismInstructions;

public class InitializeNodeGenerator extends GenericInstruction {
	private int identifier;
	private String frameName;

	public InitializeNodeGenerator(int identifier, String frameName) {
		this.identifier = identifier;
		this.frameName = frameName;
	}

	public String getPythonInstruction() {
		// x = NodeGenerator( y )
		String y = "i" + String.valueOf(identifier);
		String x = frameName;
		return y + " = NodeGenerator(" + x + ")\n";
	}

}

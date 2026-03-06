load(
  scripting.getScriptsDirectory().toString() + "/Examples/JavaScript/Utility.js"
);

var imports = new JavaImporter(org.openpnp.model, org.openpnp.util);

with (imports) {
  var ledPowerActuator = machine.getActuator("ACT1605385237291");

  function turnRed() {
    print("Turning LED on.");
    try {
      ledPowerActuator.actuate("Red");
    } catch (e) {
      print("Failed to turn LED on.", e);
    }
  }

  task(function () {
    turnRed();
  });
}

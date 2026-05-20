load(
  scripting.getScriptsDirectory().toString() + "/Examples/JavaScript/Utility.js"
);

var imports = new JavaImporter(org.openpnp.model, org.openpnp.util);

with (imports) {
  var ledPowerActuator = machine.getActuator("ACT1605385237291");

  function turnOff() {
    print("Turning LED off.");
    try {
      ledPowerActuator.actuate(false);
    } catch (e) {
      print("Failed to turn LED off.", e);
    }
  }

  task(function () {
    turnOff();
  });
}

load(
  scripting.getScriptsDirectory().toString() + "/Examples/JavaScript/Utility.js"
);

var imports = new JavaImporter(
  java.lang.Exception,
  org.openpnp.model,
  org.openpnp.util
);

with (imports) {
  task(function () {
    var head = machine.getDefaultHead();

    var units = LengthUnit.Millimeters;

    var speed = machine.getSpeed();

    //1 is maximum
    print(`the machine speed is ${speed}.`);
  });
}

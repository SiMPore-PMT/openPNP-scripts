load(
  scripting.getScriptsDirectory().toString() + "/Examples/JavaScript/Utility.js"
);

var imports = new JavaImporter(
  java.lang.Exception,
  org.openpnp.model,
  org.openpnp.util,
  org.openpnp.vision.pipeline.CvPipeline,
  org.openpnp.vision.pipeline.CvStage,
  org.opencv.core.RotatedRect
);

var coldStartX = 269.939;
var coldStarty = 146.762;
var safeHeight = 33;

var NOZZLE_NAME = "N2";

with (imports) {
  task(function () {
    var head = machine.getDefaultHead();
    var nozzle = head.getNozzleByName(NOZZLE_NAME);

    var units = LengthUnit.Millimeters;
    var rotationAtStartOfSpin = nozzle.getLocation().getRotation();
    var rotation = rotationAtStartOfSpin;

    var initY = nozzle.getLocation().getY();
    var initX = nozzle.getLocation().getX();

    nozzle.moveTo(new Location(units, initX, initY, safeHeight, rotation));

    nozzle.moveTo(
      new Location(units, coldStartX, coldStarty, safeHeight, rotation)
    );
  });
}

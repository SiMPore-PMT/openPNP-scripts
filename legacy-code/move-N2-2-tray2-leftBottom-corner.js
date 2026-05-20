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

var coldStartX = 269.813;
var coldStarty = 248.814;
var safeHeight = 33;

var NOZZLE_NAME = "N2";

with (imports) {
  task(function () {
    var head = machine.getDefaultHead();
    var nozzle = head.getNozzleByName(NOZZLE_NAME);

    var initX = nozzle.getLocation().getX();
    var initY = nozzle.getLocation().getY();

    var units = LengthUnit.Millimeters;
    var rotationAtStartOfSpin = nozzle.getLocation().getRotation();
    var rotation = rotationAtStartOfSpin;

    nozzle.moveTo(new Location(units, initX, initY, safeHeight, rotation));

    nozzle.moveTo(
      new Location(units, coldStartX, coldStarty, safeHeight, rotation)
    );
  });
}

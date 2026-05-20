load(
  scripting.getScriptsDirectory().toString() + "/Examples/JavaScript/Utility.js"
);
load(scripting.getScriptsDirectory().toString() + "/dispense.js");
load(scripting.getScriptsDirectory().toString() + "/getPlacement.js");

var imports = new JavaImporter(
  java.lang.Exception,
  java.util.ArrayList,
  org.openpnp.model,
  org.openpnp.util,
  org.openpnp.vision.pipeline.CvPipeline,
  org.openpnp.vision.pipeline.CvStage,
  org.opencv.core.RotatedRect,
  org.opencv.core.Point,
  org.opencv.core.Size,
  java.lang.Math,

  org.openpnp.vision.pipeline.stages.DrawRotatedRects,
  java.awt.Color,

  java.net.HttpURLConnection,
  java.net.URL,
  java.io.OutputStream,
  java.io.DataOutputStream,
  java.nio.charset.StandardCharsets,
  org.opencv.core.Mat,
  org.opencv.core.MatOfByte,
  org.opencv.imgcodecs.Imgcodecs,
  java.text.SimpleDateFormat,
  java.util.Date,

  org.opencv.core.MatOfPoint,
  org.opencv.imgproc.Imgproc,
  org.opencv.core.Scalar,
  javax.swing.SwingUtilities,

  // Classes for reading server responses
  java.io.BufferedReader,
  java.io.InputStreamReader
);

with (imports) {
  var jobPanel = gui.jobTab;
  task(function () {
    // getPlacement();
    // dispense();
    jobPanel.jobStart();
  });
}

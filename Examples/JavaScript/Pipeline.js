// Small example performing image processing using a CvPipeline

load(scripting.getScriptsDirectory().toString() + '/Examples/JavaScript/Utility.js');

// Import some OpenPnP classes we'll use
var imports = new JavaImporter(org.openpnp.model, org.openpnp.util, org.openpnp.vision.pipeline.CvPipeline, org.openpnp.vision.pipeline.CvStage);

// Using the imports from above, do some work.
with (imports) {
	task(function () {
		var xml =
  '<cv-pipeline>' +
  '<stages>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.ImageCapture" name="capture" enabled="true" default-light="true" settle-option="Settle" count="1"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.MaskRectangle" name="0" enabled="true" width="600" height="600"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.ConvertColor" name="gray" enabled="true" conversion="Bgr2Gray"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.Threshold" name="highlights" enabled="false" threshold="200" auto="false" invert="false"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.ThresholdAdaptive" name="1" enabled="true" adaptive-method="Mean" invert="false" block-size="127" c-parm="2"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.ImageRecall" name="recall1" enabled="false" image-stage-name="gray"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.Threshold" name="lowlights" enabled="false" threshold="120" auto="false" invert="true"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.Add" name="combined" enabled="false" first-stage-name="highlights" second-stage-name="lowlights" first-scalar="1.0" second-scalar="1.0"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.BlurMedian" name="merged" enabled="false" kernel-size="9"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.FindContours" name="contours" enabled="true" retrieval-mode="List" approximation-method="Simple"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.FilterContours" name="filtered_contours" enabled="true" contours-stage-name="contours" min-area="100000.0" max-area="200000.0" property-name="FilterContours"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.MinAreaRectContours" name="rects" enabled="true" contours-stage-name="filtered_contours"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.MinAreaRect" name="2" enabled="false" threshold-min="0" threshold-max="0" expected-angle="0.0" search-angle="45.0" left-edge="true" right-edge="true" top-edge="true" bottom-edge="true" diagnostics="false" property-name="MinAreaRect"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.OrientRotatedRects" name="oriented_rects" enabled="true" rotated-rects-stage-name="rects" orientation="Landscape" negate-angle="false" snap-angle="0"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.OrientRotatedRects" name="results" enabled="true" rotated-rects-stage-name="rects" orientation="Landscape" negate-angle="true" snap-angle="0"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.ImageRecall" name="recall2" enabled="true" image-stage-name="capture"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.DrawContours" name="draw_contours" enabled="true" contours-stage-name="filtered_contours" thickness="2" index="-1"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.DrawRotatedRects" name="draw_results" enabled="true" rotated-rects-stage-name="oriented_rects" thickness="2" draw-rect-center="true" rect-center-radius="3" show-orientation="true">' +
  '</cv-stage>' +
  '</stages>' +
  '</cv-pipeline>';

		var pipeline = new CvPipeline(xml);

		var camera = machine.getDefaultHead().getDefaultCamera();

		pipeline.setProperty("camera", camera);

		pipeline.process();

		gui.getCameraViews().getCameraView(camera).showFilteredImage(OpenCvUtils.toBufferedImage(pipeline.getWorkingImage()),1500);

		var result = pipeline.getResult("results");

	});
}

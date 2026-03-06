/**
 * OpenPnP Script: Move camera to all placement locations.
 * 
 * This script loops through all placement locations in the current job
 * and moves the camera to each one.
 */

load(
  scripting.getScriptsDirectory().toString() + "/Examples/JavaScript/Utility.js"
);

var imports = new JavaImporter(java.lang.Exception,java.util.ArrayList, org.openpnp.model,org.openpnp.util, org.openpnp.vision.pipeline.CvPipeline, org.openpnp.vision.pipeline.CvStage, org.opencv.core.RotatedRect,org.opencv.core.Point, org.opencv.core.Size, java.lang.Math, javax.swing.SwingUtilities);

var placeHeight = 0;


with (imports) {
  
  task(function () {

    var head = machine.getDefaultHead();
    var job = gui.jobTab.job;
    var camera = head.getDefaultCamera();
    var units = LengthUnit.Millimeters;

    var xml =   '<cv-pipeline>' +
    '<stages>' +
    '<cv-stage class="org.openpnp.vision.pipeline.stages.ImageCapture" name="capture" enabled="true" default-light="true" settle-option="Settle" count="1"/>' +
    '<cv-stage class="org.openpnp.vision.pipeline.stages.MaskRectangle" name="0" enabled="true" width="1000" height="1000"/>' +
    '<cv-stage class="org.openpnp.vision.pipeline.stages.ConvertColor" name="gray" enabled="true" conversion="Bgr2Gray"/>' +
    '<cv-stage class="org.openpnp.vision.pipeline.stages.Threshold" name="highlights" enabled="true" threshold="35" auto="false" invert="false"/>' +
    '<cv-stage class="org.openpnp.vision.pipeline.stages.BlurMedian" name="merged" enabled="true" kernel-size="13"/>' +
    '<cv-stage class="org.openpnp.vision.pipeline.stages.FindContours" name="contours" enabled="true" retrieval-mode="List" approximation-method="Simple"/>' +
    '<cv-stage class="org.openpnp.vision.pipeline.stages.FilterContours" name="filtered_contours" enabled="true" contours-stage-name="contours" min-area="100000.0" max-area="200000.0" property-name="FilterContours"/>' +
    '<cv-stage class="org.openpnp.vision.pipeline.stages.MinAreaRectContours" name="rects" enabled="true" contours-stage-name="filtered_contours"/>' +
    '<cv-stage class="org.openpnp.vision.pipeline.stages.OrientRotatedRects" name="orient" enabled="true" rotated-rects-stage-name="rects" orientation="Landscape" negate-angle="true" snap-angle="0"/>' +
    '<cv-stage class="org.openpnp.vision.pipeline.stages.ScriptRun" name="results" enabled="true" file="/home/engineering-simpore/.openpnp2/scripts/return-always-north-angle-rect.bsh" args=""/>' +
    '<cv-stage class="org.openpnp.vision.pipeline.stages.ImageRecall" name="recall2" enabled="true" image-stage-name="capture"/>' +
    '<cv-stage class="org.openpnp.vision.pipeline.stages.DrawContours" name="draw_contours" enabled="true" contours-stage-name="filtered_contours" thickness="2" index="-1"/>' +
    '<cv-stage class="org.openpnp.vision.pipeline.stages.DrawRotatedRects" name="final_image" enabled="true" rotated-rects-stage-name="results" thickness="2" draw-rect-center="true" rect-center-radius="3" show-orientation="true">' +
    '<color r="51" g="255" b="51" a="255"/>' +
    '</cv-stage>' +
    '</stages>' +
    '</cv-pipeline>';



    var pipeline = new CvPipeline(xml);

    pipeline.setProperty("camera", camera);


    // --- Initial Sanity Checks ---
    try { 
      
    if (!job) {
      javax.swing.JOptionPane.showMessageDialog(
        null,
        "No job is currently loaded."
      );
      return;
    }

    if (job.getBoardLocations().isEmpty()) {
      javax.swing.JOptionPane.showMessageDialog(
        null,
        "The current job has no boards defined."
      );
      return;
    }


    for each (var boardLocation in job.getBoardLocations()) {
      var boardDefinition = boardLocation.getBoard();

      if (!boardDefinition) {
        print(
          "WARNING: BoardLocation " +
            boardLocation.getId() +
            " has no board definition."
        );
        continue;
      }

      var placements = boardDefinition.getPlacements();

      //update the initial location only
      var baseLocation = placements.get(0).getLocation();
      var currentInspectionLocation = baseLocation;
      var counter = 1;

      for each (var placement in placements) {
        
        // Skip placements that are not enabled.
        if (!placement.isEnabled()) {
          counter++;
          continue;
        }

        // Skip fiducials
        var partId = placement.getPart() ? placement.getPart().getId() : "";
        if (partId.contains("Fiducial")) {
          continue;
        }

        // For every placement after the first, calculate the new inspection location.
        if (counter > 1) {
          var currentX = currentInspectionLocation.getX();
          var currentY = currentInspectionLocation.getY();
          var units = currentInspectionLocation.getUnits();
          var newX = currentX;
          var newY = currentY;

          // Conditions for moving right (X+)
          if (
            (counter >= 2 && counter <= 6) ||
            (counter >= 14 && counter <= 18) ||
            (counter >= 26 && counter <= 30) ||
            (counter >= 38 && counter <= 42) ||
            (counter >= 50 && counter <= 54) ||
            (counter >= 62 && counter <= 66) ||
            (counter >= 74 && counter <= 78) ||
            (counter >= 86 && counter <= 90) ||
            (counter >= 98 && counter <= 102) 
          ) {
            newX += 28;
          }
          // Conditions for moving left (X-)
          else if (
            (counter >= 8 && counter <= 12) ||
            (counter >= 20 && counter <= 24) ||
            (counter >= 32 && counter <= 36) || 
            (counter >= 44 && counter <= 48) ||
            (counter >= 56 && counter <= 60) ||
            (counter >= 68 && counter <= 72) ||
            (counter >= 80 && counter <= 84) ||
            (counter >= 92 && counter <= 96) ||
            (counter >= 104 && counter <= 108) 
          ) {
            newX -= 28;
          }
          // Conditions for moving up (Y+)
          else if (
            counter == 7 ||
            counter == 13 ||
            counter == 19 ||
            counter == 25 ||
            counter == 31 ||
            counter == 43 ||
            counter == 49 ||
            counter == 55 ||
            counter == 61 ||
            counter == 67 ||
            counter == 79 ||
            counter == 85 ||
            counter == 91 ||
            counter == 97 ||
            counter == 103 
          ) {
            newY += 28;
          }

          else if (
            counter == 37){
              newX = placement.getLocation().getX();
              newY = placement.getLocation().getY();
            }

          else if (
            counter == 73){
              newX = placement.getLocation().getX();
              newY = placement.getLocation().getY();
            }
          
          

          // Create a new Location object for the inspection.
          // This uses the new X/Y but preserves the original Z and Rotation.
          currentInspectionLocation = new Location(
            units,
            newX,
            newY,
            baseLocation.getZ(),
            baseLocation.getRotation()
          );
          
        }

        

        // move camera to the placement location 
        camera.moveTo(currentInspectionLocation);


        // execute visin pipeline 
        for (var i = 0; i < 2; i++) {
          pipeline.process();
        }
        
        var visionResults = pipeline.getResult("results");

          if (visionResults == null) {
            print("WARNING: Vision pipeline failed to produce a result for " + partId);
            counter++;
            continue;
          }

           var rects;
            try {
              rects = visionResults.getExpectedListModel(
                RotatedRect.class,
                new Exception("Vision returned no results")
              );
            } catch (e) {
              print("WARNING: Vision ran, but no part was found for " + partId);
              counter++;
              continue; // Skip to the next placement
            }

          // CHANGE: Check if the list is empty before trying to access an element.
          if (rects.isEmpty()) {
            print("WARNING: Vision ran, but no part was found for " + partId);
            counter++;
            continue; // Skip to the next placement
          }

          // We have at least one result, so get the first one.
          var result = rects.get(0);

          var centerX = result.center.x;
          var centerY = result.center.y;
          var angle = result.angle;

          var finalLocation1 = VisionUtils.getPixelLocation(camera, centerX, centerY);
          var finalLocation = new Location(
            units,
            finalLocation1.getX(),
            finalLocation1.getY(),
            placeHeight,
            angle
          );

          var outputString =
            "{ {" +
            finalLocation.getX().toFixed(4) +
            ", " +
            finalLocation.getY().toFixed(4) +
            "} * " +
            finalLocation.getRotation().toFixed(4) +
            " }";
 
          print(" Vision Offset: Angle " + angle.toFixed(4));
          print(" Final Location: " + outputString);

          placement.setLocation(finalLocation);
          counter++;
      }
    }
  }finally {
      // This block will always run when the task is finished or an error occurs.
      print("Script finished. Releasing pipeline resources.");
      if (pipeline != null) {
        pipeline.close();
      }
      SwingUtilities.invokeLater(function() {
          gui.jobTab.refresh();
          javax.swing.JOptionPane.showMessageDialog(null, "Post placements uploaded.");
      });
    }
  });
};
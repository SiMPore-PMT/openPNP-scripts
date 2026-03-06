/**
 * Script for board localization with a FIXED TOP CAMERA, using a custom CvPipeline.
 * Bypasses FiducialLocator. Uses JavaImporter for class resolution.
 */

load(
  scripting.getScriptsDirectory().toString() + "/Examples/JavaScript/Utility.js"
);

// Import OpenPnP classes using JavaImporter
var imports = new JavaImporter(
  org.openpnp.model.Location, // Specific class for clarity
  org.openpnp.model.LengthUnit, // Specific class
  org.openpnp.vision.pipeline.CvPipeline, // Specific class
  org.openpnp.util.Utils2D, // Specific class
  java.awt.geom.Point2D, // Specific class (Point2D.Double will be used)
  org.openpnp.util.AngleUtils // Specific class
  // org.openpnp.opencv.OpenCvUtils // If you need to show images via gui
);

// --- Script Configuration ---
var NOZZLE_NAME = "N2";
var CAMERA_NAME = null;
var SAFE_Z_HEIGHT = 30.0;

var EXPECTED_BOARD_FIDUCIALS = [
  { id: "FID1_Board", x: 76.16, y: -33.986 },
  { id: "FID2_Board", x: 166.261, y: -33.213 },
  { id: "FID3_Board", x: 166.028, y: 55.867 },
  { id: "FID4_Board", x: 75.837, y: 56.234 },
];
var INITIAL_BOARD_ORIGIN_GUESS_MACHINE = { x: 27.888, y: 121.786};
var FIDUCIAL_CV_PIPELINE_XML =
  '<cv-pipeline>' +
  '<stages>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.ImageCapture" name="image" enabled="true" default-light="true" settle-option="Settle" count="1"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.MaskCircle" name="mask" enabled="true" diameter="500" property-name="MaskCircle"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.DetectCircularSymmetry" name="cir" enabled="true" min-diameter="10" max-diameter="150" max-distance="250" search-width="0" search-height="0" max-target-count="1" min-symmetry="1.2" corr-symmetry="0.0" outer-margin="0.2" inner-margin="0.4" sub-sampling="8" super-sampling="1" symmetry-score="OverallVarianceVsRingVarianceSum" property-name="" diagnostics="true" heat-map="true"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.ConvertModelToKeyPoints" name="results" enabled="true" model-stage-name="cir"/>' +
  '</stages>' +
  '</cv-pipeline>';
var BOARD_PLACEMENTS = [
  { id: "C1", x: 44.724, y: 15.297, rotation: 0.0 },
  { id: "R1", x: 74.952, y: 15.434, rotation: 0.0 },
  { id: "U1", x: 135.045, y: 15.9, rotation: 0.0 },
];
// --- End of Configuration ---

// Using the imports from above, do some work.
with (imports) {
  task(function () {
    var head = machine.getDefaultHead();
    if (!head) {
      print("ERROR: Default head not found.");
      return;
    }
    var nozzle = head.getNozzleByName(NOZZLE_NAME);
    if (!nozzle) {
      print("ERROR: Nozzle '" + NOZZLE_NAME + "' not found.");
      return;
    }
    var cameraToUse;
    if (CAMERA_NAME) {
      cameraToUse = head.getCameraByName(CAMERA_NAME);
    } else {
      cameraToUse = head.getDefaultCamera();
    }
    if (!cameraToUse) {
      print("ERROR: Camera not found.");
      return;
    }
    print("INFO: Using Fixed Camera: " + cameraToUse.getName());

    var foundMachineFiducials = [];
    var expectedBoardFiducialPoints = [];
    var currentNozzleLoc = nozzle.getLocation();

    // Initial move to safe height
    try {
        print("INFO: Performing initial move to safe Z.");
        nozzle.moveTo(
            new Location( // Location is now directly available due to 'with (imports)'
            currentNozzleLoc.getUnits(),
            currentNozzleLoc.getX(),
            currentNozzleLoc.getY(),
            SAFE_Z_HEIGHT,
            currentNozzleLoc.getRotation()
            )
        );
        print("INFO: Initial move to safe Z complete.");
    } catch (e) {
        print("ERROR during initial nozzle.moveTo: " + e.message);
        var sw = new java.io.StringWriter(); e.printStackTrace(new java.io.PrintWriter(sw)); print(sw.toString());
        return;
    }


    print("INFO: Starting fiducial search with script-defined CvPipeline (Fixed Camera)...");
    var workingBoardOriginGuessMachine = {
      x: INITIAL_BOARD_ORIGIN_GUESS_MACHINE.x,
      y: INITIAL_BOARD_ORIGIN_GUESS_MACHINE.y,
    };

    var fiducialFindingPipeline;
    try {
      print("INFO: Attempting to create CvPipeline instance.");
      // CvPipeline is now directly available due to 'with (imports)'
      fiducialFindingPipeline = new CvPipeline(FIDUCIAL_CV_PIPELINE_XML);
      print("INFO: CvPipeline instance created successfully.");
      fiducialFindingPipeline.setProperty("camera", cameraToUse);
      print("INFO: Camera property set on CvPipeline.");
    } catch (e) {
      print("ERROR creating or configuring CvPipeline: " + e.message);
      var sw = new java.io.StringWriter();
      e.printStackTrace(new java.io.PrintWriter(sw));
      print(sw.toString());
      return; // Stop if pipeline can't be created
    }

    for (var i = 0; i < EXPECTED_BOARD_FIDUCIALS.length; i++) {
      var expectedBoardFid = EXPECTED_BOARD_FIDUCIALS[i];
      print("LOOP START: Processing fiducial " + (i + 1) + ": " + expectedBoardFid.id);

      try { // Added try-catch around the entire loop iteration
        var approxMachineX =
          workingBoardOriginGuessMachine.x + expectedBoardFid.x;
        var approxMachineY =
          workingBoardOriginGuessMachine.y + expectedBoardFid.y;

        var visionTargetLocation = new Location( // Location is directly available
          LengthUnit.Millimeters, // LengthUnit is directly available
          approxMachineX,
          approxMachineY,
          SAFE_Z_HEIGHT,
          0.0
        );
        print("  Moving nozzle to: " + visionTargetLocation);
        nozzle.moveTo(visionTargetLocation);
        print("  Nozzle move complete. Settling camera...");
        java.lang.Thread.sleep(cameraToUse.getSettleTimeMs() + 250);
        print("  Camera settled. Processing pipeline...");

        fiducialFindingPipeline.process();
        print("  Pipeline processed.");
        var rawPipelineOutput = fiducialFindingPipeline.getResult("results");
        print("    Raw output from 'results' stage: " + rawPipelineOutput);

        var keyPointToUse = null; // This will hold the single KeyPoint we want to use

      if (rawPipelineOutput) {
        var modelData = null;
        var rawOutputClassName = "Unknown";
        try {
            rawOutputClassName = rawPipelineOutput.getClass().getName();
            print("    Type of rawPipelineOutput: " + rawOutputClassName);
        } catch (e) { print("    Could not get class name of rawPipelineOutput."); }


        // Check if rawPipelineOutput is a CvStage.Result itself
        if (typeof rawPipelineOutput.getModel === 'function') {
            print("    rawPipelineOutput appears to be a CvStage.Result. Getting model from it.");
            modelData = rawPipelineOutput.getModel(); // This should be the actual data
            print("    Model data from CvStage.Result: " + modelData);
            if (modelData) {
                try {
                    print("    Type of modelData (from CvStage.Result.getModel()): " + modelData.getClass().getName());
                } catch (e) { print("    Could not get class name of modelData.");}
            }
        } else {
            // Assume rawPipelineOutput IS the model data directly
            print("    rawPipelineOutput is NOT a CvStage.Result. Assuming it's the model data directly.");
            modelData = rawPipelineOutput;
        }

        // Now, figure out what modelData is and extract a single KeyPoint
        if (modelData) {
            // Check if modelData is a single KeyPoint (duck-typing by checking for 'pt' property)
            if (typeof modelData.pt !== 'undefined' && modelData.pt !== null && typeof modelData.pt.x !== 'undefined') {
                print("    Model data appears to be a single KeyPoint object.");
                keyPointToUse = modelData;
            }
            // Check if modelData is a Java List
            else if (modelData instanceof java.util.List) {
                print("    Model data is a java.util.List. Size: " + modelData.size());
                if (modelData.size() > 0) {
                    var firstElement = modelData.get(0);
                    // Check if the first element is a KeyPoint
                    if (typeof firstElement.pt !== 'undefined' && firstElement.pt !== null && typeof firstElement.pt.x !== 'undefined') {
                        keyPointToUse = firstElement;
                        print("    Took first KeyPoint from the List.");
                    } else {
                        print("    WARN: First element in List is not a KeyPoint: " + firstElement);
                    }
                } else {
                    print("    Model data List is empty.");
                }
            }
            // Check if modelData behaves like an OpenCV MatOfKeyPoint
            else if (typeof modelData.empty === 'function' && typeof modelData.toArray === 'function') {
                print("    Model data behaves like a MatOfKeyPoint.");
                if (!modelData.empty()) {
                    var keyPointListFromMat = modelData.toList(); // Convert MatOfKeyPoint to List<KeyPoint>
                    print("    Converted MatOfKeyPoint to List. Size: " + keyPointListFromMat.size());
                    if (keyPointListFromMat.size() > 0) {
                        keyPointToUse = keyPointListFromMat.get(0);
                        print("    Took first KeyPoint from MatOfKeyPoint (after converting to list).");
                    } else {
                         print("    List converted from MatOfKeyPoint is empty.");
                    }
                } else {
                    print("    MatOfKeyPoint is empty.");
                }
            } else {
                print("    WARN: Model data is not a recognized KeyPoint, List, or MatOfKeyPoint. Type was: " + (modelData.getClass ? modelData.getClass().getName() : "Unknown"));
            }
        } else {
            print("    Model data is null.");
        }
      } else {
          print("    Raw pipeline output for 'results' is null.");
      }

      // Proceed if we successfully extracted a KeyPoint
      if (keyPointToUse) {
        print("    Successfully extracted KeyPoint to use: X=" + keyPointToUse.pt.x + ", Y=" + keyPointToUse.pt.y);

        var fiducialPixelLocation = new Location( // Location is directly available
          LengthUnit.Pixels, // LengthUnit is directly available
          keyPointToUse.pt.x,
          keyPointToUse.pt.y,
          0,0
        );
        var fiducialMachineLocation = cameraToUse.getLocation(fiducialPixelLocation);
        print(
          "  Found " + expectedBoardFid.id + " at Machine: " +
          fiducialMachineLocation.getX().toFixed(3) + ", " +
          fiducialMachineLocation.getY().toFixed(3)
        );
        foundMachineFiducials.push(fiducialMachineLocation);
        expectedBoardFiducialPoints.push(
          new java.awt.geom.Point2D.Double(expectedBoardFid.x, expectedBoardFid.y)
        );
        if (foundMachineFiducials.length === 1) {
          workingBoardOriginGuessMachine.x =
            fiducialMachineLocation.getX() - expectedBoardFid.x;
          workingBoardOriginGuessMachine.y =
            fiducialMachineLocation.getY() - expectedBoardFid.y;
          print(
            "  Updated board origin guess to Machine: " +
            workingBoardOriginGuessMachine.x.toFixed(3) + ", " +
            workingBoardOriginGuessMachine.y.toFixed(3)
          );
        }
      } else {
        print("  WARN: No valid KeyPoint ultimately found for " + expectedBoardFid.id +
              " at approx. Machine X: " + approxMachineX.toFixed(3) +
              ", Y: " + approxMachineY.toFixed(3));
      }
      } catch (e_loop) {
        print("ERROR in fiducial processing loop for " + expectedBoardFid.id + ": " + e_loop.message);
        var sw_loop = new java.io.StringWriter();
        e_loop.printStackTrace(new java.io.PrintWriter(sw_loop));
        print(sw_loop.toString());
        // Decide if you want to 'return;' or 'continue;'
        // For now, let's continue to try other fiducials if possible,
        // but the transform will likely fail if not enough are found.
      }
      print("LOOP END: Finished processing for " + expectedBoardFid.id);
    } // End fiducial search loop

    if (foundMachineFiducials.length < 2) {
      print(
        "ERROR: Found only " + foundMachineFiducials.length +
        " fiducials. Need at least 2 for transform."
      );
      return;
    }
    // ... (rest of the transformation and placement logic - ensure classes like Utils2D, AngleUtils are used directly)

    var actualMachineFiducialPoints = [];
    for (var i = 0; i < foundMachineFiducials.length; i++) {
        actualMachineFiducialPoints.push(
        new java.awt.geom.Point2D.Double( // Use full path if Point2D.Double isn't resolved by 'with'
            foundMachineFiducials[i].getX(),
            foundMachineFiducials[i].getY()
        )
        );
    }

    // Utils2D is directly available
    var boardToMachineTransform = Utils2D.deriveAffineTransform(
        expectedBoardFiducialPoints.slice(0, actualMachineFiducialPoints.length),
        actualMachineFiducialPoints
    );

    if (!boardToMachineTransform) {
        print("ERROR: Failed to calculate board transformation.");
        return;
    }
    print("INFO: Board transformation calculated successfully.");
    // ... (matrix printout) ...

    print("\nINFO: Moving to board placements using calculated transform...");
    for (var i = 0; i < BOARD_PLACEMENTS.length; i++) {
        var boardPlacement = BOARD_PLACEMENTS[i];
        var boardPoint = new java.awt.geom.Point2D.Double(boardPlacement.x, boardPlacement.y);
        var machinePoint = new java.awt.geom.Point2D.Double();
        boardToMachineTransform.transform(boardPoint, machinePoint);

        var boardRotationDegrees = (Math.atan2(matrix[1], matrix[0]) * 180.0) / Math.PI;
        var finalMachineRotation = boardPlacement.rotation + boardRotationDegrees;
        finalMachineRotation = AngleUtils.normalizeDegrees(finalMachineRotation); // AngleUtils directly available

        print( /* ... placement info ... */ );
        var placementMachineLocation = new Location( /* ... */ );
        nozzle.moveTo(placementMachineLocation);
        java.lang.Thread.sleep(1500);
    }
    print("\nINFO: All placements processed.");

  }); // End task
} // End with(imports)

load(
  scripting.getScriptsDirectory().toString() + "/Examples/JavaScript/Utility.js"
);

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
    
    // Classes for programmatic drawing
    org.openpnp.vision.pipeline.stages.DrawRotatedRects,
    java.awt.Color,

    // Classes for networking and image handling
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

var placeHeight = 0;

var PIPELINE_RUNS = 10; 
var CLUSTER_DISTANCE_THRESHOLD = 5.0; 
var CLUSTER_ANGLE_THRESHOLD = 0.05; 
var API_URL = "http://localhost:3000/api/upload-image"; 
var API_START = "http://localhost:3000/api/runs/start";
var UPLOAD_FOLDER = "Pre-Placements"; 


function averageRotatedRects(rects) {
    with (imports) {
        if (rects == null || rects.isEmpty()) {
            return null;
        }

        var avgX = 0.0;
        var avgY = 0.0;
        var avgWidth = 0.0;
        var avgHeight = 0.0;
        var avgAngleX = 0.0; // For averaging angles as vectors
        var avgAngleY = 0.0;

        var count = rects.size();

        for (var i = 0; i < count; i++) {
            var rect = rects.get(i);
            avgX += rect.center.x;
            avgY += rect.center.y;
            avgWidth += rect.size.width;
            avgHeight += rect.size.height;

            // Convert angle to a vector, add it to the sum.
            var angleRad = Math.toRadians(rect.angle);
            avgAngleX += Math.cos(angleRad);
            avgAngleY += Math.sin(angleRad);
        }

        avgX /= count;
        avgY /= count;
        avgWidth /= count;
        avgHeight /= count;

        // Convert the average vector back to an angle.
        var finalAngleRad = Math.atan2(avgAngleY, avgAngleX);
        var finalAngleDeg = Math.toDegrees(finalAngleRad);

        return new RotatedRect(
            new Point(avgX, avgY),
            new Size(avgWidth, avgHeight),
            finalAngleDeg
        );
    }
}


function startRunSession(apiUrl) {
    with (imports) {
        try {
            print("Requesting new run number from server...");
            var url = new URL(apiUrl);
            var conn = url.openConnection();
            conn.setRequestMethod("POST");
            conn.setDoOutput(true); // Required for POST to send a body, even if empty

            var responseCode = conn.getResponseCode();
            if (responseCode == HttpURLConnection.HTTP_OK) {
                var inReader = new BufferedReader(new InputStreamReader(conn.getInputStream()));
                var response = inReader.readLine(); // Read the JSON response line
                inReader.close();
                
                // Use regex to safely extract the run number from the JSON string
                var match = response.match(/"runNumber":(\d+)/);
                if (match && match[1]) {
                    var runNumber = parseInt(match[1]);
                    print("Successfully started Run #" + runNumber);
                    return runNumber;
                } else {
                    throw new Exception("Could not parse runNumber from server response: " + response);
                }
            } else {
                throw new Exception("Server responded with error code: " + responseCode);
            }
        } catch (e) {
            print("FATAL: Could not start run session. " + e);
            // Show a popup to the user
            javax.swing.JOptionPane.showMessageDialog(null, "FATAL: Could not start run session with the server.\n" + e, "Network Error", javax.swing.JOptionPane.ERROR_MESSAGE);
            return null;
        }
    }
}

function uploadImageToServer(pipeline, result, folder, filename, apiUrl, runNumber) {
    with (imports) {
        try {
            print("Attempting to upload image: " + filename);
            
            var mat = pipeline.getWorkingImage();

            if (mat != null && !mat.empty()) {
                var vertices = [];
                var tempPoints = new (Java.type("org.opencv.core.Point[]"))(4);
                result.points(tempPoints); 
                
                // Convert the Point array to the format Imgproc expects
                for (var i = 0; i < tempPoints.length; i++) {
                    vertices.push(new Point(tempPoints[i].x, tempPoints[i].y));
                }
                
                // Create MatOfPoint from the vertices
                var matOfPoint = new MatOfPoint();
                matOfPoint.fromArray(vertices);
                
                var contoursToDraw = new ArrayList();
                contoursToDraw.add(matOfPoint);

                // Draw the rectangle outline
                Imgproc.drawContours(
                    mat,
                    contoursToDraw,
                    -1,
                    new Scalar(51, 255, 51), // Green color 
                    1,
                    Imgproc.LINE_AA
                );

                // Draw the center point
                Imgproc.circle(
                    mat,
                    result.center,
                    2,
                    new Scalar(0, 255, 255), // Yellow color 
                    -1
                );

                var angleRad = Math.toRadians(result.angle);
                
                var lineLength = result.size.width / 2.0;
                
                var endPoint = new Point(
                    result.center.x + lineLength * Math.cos(angleRad),
                    result.center.y + lineLength * Math.sin(angleRad)
                );
                
                // Draw the orientation line
                Imgproc.line(
                    mat,
                    result.center, 
                    endPoint,      
                    new Scalar(0, 255, 255), 
                    1,
                    Imgproc.LINE_AA        
                );
            }

            if (mat == null || mat.empty()) {
                print("Error: Image not found or is empty.");
                return;
            }
            
            var matOfByte = new MatOfByte();
            Imgcodecs.imencode(".png", mat, matOfByte);
            var imageBytes = matOfByte.toArray();
            var boundary = "===" + java.lang.System.currentTimeMillis() + "===";
            var url = new URL(apiUrl);
            var conn = url.openConnection();
            conn.setDoOutput(true);
            conn.setRequestMethod("POST");
            conn.setRequestProperty("Content-Type", "multipart/form-data; boundary=" + boundary);
            var dos = new DataOutputStream(conn.getOutputStream());

            //folder
            dos.writeBytes("--" + boundary + "\r\n");
            dos.writeBytes('Content-Disposition: form-data; name="folder"\r\n\r\n');
            dos.writeBytes(folder + "\r\n");

            //runNumber
            dos.writeBytes("--" + boundary + "\r\n");
            dos.writeBytes('Content-Disposition: form-data; name="runNumber"\r\n\r\n');
            dos.writeBytes(runNumber.toString() + "\r\n");

            //file
            dos.writeBytes("--" + boundary + "\r\n");
            dos.writeBytes('Content-Disposition: form-data; name="file"; filename="' + filename + '"\r\n');
            dos.writeBytes("Content-Type: image/png\r\n\r\n");
            dos.write(imageBytes);
            dos.writeBytes("\r\n");
            dos.writeBytes("--" + boundary + "--\r\n");


            dos.flush();
            dos.close();
            var responseCode = conn.getResponseCode();
            if (responseCode == HttpURLConnection.HTTP_OK) {
                print("Image uploaded successfully! Server responded with code: " + responseCode);
            } else {
                print("Upload failed. Server responded with code: " + responseCode + ", " + conn.getResponseMessage());
            }
            conn.disconnect();
        } catch (e) {
            print("An exception occurred during upload: " + e);
            e.printStackTrace();
        }
    }
}



with (imports) {

  task(function () {

    var head = machine.getDefaultHead();
    var job = gui.jobTab.job;
    var camera = head.getDefaultCamera();
    var units = LengthUnit.Millimeters;

    var xml =   '<cv-pipeline>' +
    '<stages>' +
    '<cv-stage class="org.openpnp.vision.pipeline.stages.ImageCapture" name="capture" enabled="true" default-light="true" settle-option="Settle" count="1"/>' +
    '<cv-stage class="org.openpnp.vision.pipeline.stages.MaskRectangle" name="0" enabled="true" width="500" height="500"/>' +
    '<cv-stage class="org.openpnp.vision.pipeline.stages.ConvertColor" name="gray" enabled="true" conversion="Bgr2Gray"/>' +
    '<cv-stage class="org.openpnp.vision.pipeline.stages.Threshold" name="highlights" enabled="true" threshold="35" auto="false" invert="false"/>' +
    '<cv-stage class="org.openpnp.vision.pipeline.stages.BlurMedian" name="merged" enabled="true" kernel-size="13"/>' +
    '<cv-stage class="org.openpnp.vision.pipeline.stages.FindContours" name="contours" enabled="true" retrieval-mode="List" approximation-method="Simple"/>' +
    '<cv-stage class="org.openpnp.vision.pipeline.stages.FilterContours" name="filtered_contours" enabled="true" contours-stage-name="contours" min-area="100000.0" max-area="200000.0" property-name="FilterContours"/>' +
    '<cv-stage class="org.openpnp.vision.pipeline.stages.MinAreaRectContours" name="rects" enabled="true" contours-stage-name="filtered_contours"/>' +
    '<cv-stage class="org.openpnp.vision.pipeline.stages.OrientRotatedRects" name="orient" enabled="true" rotated-rects-stage-name="rects" orientation="Landscape" negate-angle="true" snap-angle="0"/>' +
    '<cv-stage class="org.openpnp.vision.pipeline.stages.ScriptRun" name="results" enabled="true" file="/home/engineering-simpore/.openpnp2/scripts/return-always-north-angle-rect.bsh" args=""/>' +
    '<cv-stage class="org.openpnp.vision.pipeline.stages.ImageRecall" name="recall2" enabled="true" image-stage-name="capture"/>' +
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

    var runNumber = startRunSession(API_START);
      
      // If we couldn't get a run number, abort the entire script.
      if (runNumber == null) {
          print("Aborting script because a run number could not be obtained.");
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

      counter = 1;

      for each (var placement in placements) {
        

        // Skip placements that are not enabled.
        if (!placement.isEnabled()) {
          continue;
        }

        // Skip fiducials
        var partId = placement.getPart() ? placement.getPart().getId() : "";
        if (partId.contains("Fiducial")) {
          continue;
        }

        location = placement.getLocation();

        // move camera to the placement location 
        camera.moveTo(location);


        var allResults = new ArrayList();
        for (var i = 0; i < PIPELINE_RUNS; i++) {
          pipeline.process();
          var visionResults = pipeline.getResult("results");

          var rects;

          rects = visionResults.getExpectedListModel(
                RotatedRect.class,
                new Exception("Vision returned no results")
              );

          // Ignore single runs that fail or produce no rects.
          if (rects.isEmpty()) {
            counter++;
            continue;
          }

          try {
            // Add all found rectangles from this run to our master list.
            allResults.addAll(rects);
          } catch (e) {
            // This run didn't find anything
            counter++;
            continue;
          }
        }

          if (allResults == null) {
            print("WARNING: Vision pipeline failed to produce a result for " + partId);
            counter++;
            continue;
          }


          print(
              "Vision found " +
              allResults.size() +
              " total potential rects for " +
              partId
            );



          var bestCluster = new ArrayList();

          for each (var rectA in allResults) {
            var currentCluster = new ArrayList();
            currentCluster.add(rectA);

            if (rectA == undefined) {
              print("WARNING: Found an undefined rectangle in the results for " + partId);
              continue; // Skip this rectangle if it's undefined.
            }

            for each (var rectB in allResults) {

              if (rectB == undefined) {
              print("WARNING: Found an undefined rectangle in the results for " + partId);
              continue; // Skip this rectangle if it's undefined.
            }

              if (rectA == rectB) continue; // Don't compare a rect to itself.

              // Manually calculate the distance between the centers of the two rectangles
              // using the distance formula: sqrt((x2-x1)^2 + (y2-y1)^2)
              var deltaX = rectA.center.x - rectB.center.x;
              var deltaY = rectA.center.y - rectB.center.y;
              var distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

              if (distance < CLUSTER_DISTANCE_THRESHOLD) {
                // Calculate the absolute difference in angle between the two rectangles.
                var angleDifference = Math.abs(rectA.angle - rectB.angle);

                if (angleDifference < CLUSTER_ANGLE_THRESHOLD) {
                  currentCluster.add(rectB);
                }
              }
            }
            // If this is the largest cluster we've found so far, save it.
            if (currentCluster.size() > bestCluster.size()) {
              bestCluster = currentCluster;
            }
          }

          print("New bestCluster found: " + bestCluster);

          var newRect = averageRotatedRects(bestCluster);
          if (newRect == null) {
            print("WARNING: No valid rects found in the best cluster for " + partId);
            counter++;
            continue; // Skip to the next placement
          }
        


        //  Vision was successful, upload the image 
        print("Vision successful for " + partId + ". Uploading verification image.");
        var timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
        var filename = counter + "_" + timestamp + ".png";
        
        var result = newRect;

        uploadImageToServer(
            pipeline,
            result,
            UPLOAD_FOLDER,
            filename,
            API_URL,
            runNumber
        );
        // --- END upload---

        
        var centerX = result.center.x;
        var centerY = result.center.y;
        var angle = result.angle;
        var finalLocation1 = VisionUtils.getPixelLocation(camera, centerX, centerY);
        var finalLocation = new Location(units, finalLocation1.getX(), finalLocation1.getY(), placeHeight, angle);
        var outputString = "{ {" + finalLocation.getX().toFixed(4) + ", " + finalLocation.getY().toFixed(4) + "} * " + finalLocation.getRotation().toFixed(4) + " }";
        print("  Vision Offset: Angle " + angle.toFixed(4));
        print("  Final Location: " + outputString);
        placement.setLocation(finalLocation);
        counter++;
      }
    }
  }finally {
      print("Script finished. Releasing pipeline resources.");
      if (pipeline != null) {
        pipeline.close();
      }
      SwingUtilities.invokeLater(function() {
          gui.jobTab.refresh();
          javax.swing.JOptionPane.showMessageDialog(null, "The placements are updated.");
      });
    }
  });
};
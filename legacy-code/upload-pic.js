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
    javax.swing.SwingUtilities  
);


var placeHeight = 0;


var PIPELINE_RUNS = 10; 
var CLUSTER_DISTANCE_THRESHOLD = 5.0; 
var CLUSTER_ANGLE_THRESHOLD = 0.05; 
var API_URL = "http://localhost:3000/api/upload-image"; 
var UPLOAD_FOLDER = "Initial-Placements"; 


// function averageRotatedRects(rects) {
//   with (imports) {
//         if (rects == null || rects.isEmpty()) {
//           return null;
//         }

//         var avgX = 0.0;
//         var avgY = 0.0;
//         var avgAngle= 0;
//         var avgWidth = 0.0;
//         var avgHeight = 0.0;
        

//         var count = rects.size();

//         for (var i = 0; i < count; i++) {
//           var rect = rects.get(i);
//           avgX += rect.center.x;
//           avgY += rect.center.y;
//           avgAngle += rect.angle; 
//           avgWidth += rect.size.width;
//           avgHeight += rect.size.height;
//         }

//         avgX /= count;
//         avgY /= count;
//         avgAngle /= count;
//         avgWidth /= count;
//         avgHeight /= count;

//         return new RotatedRect(
//           new Point(avgX, avgY),
//           new Size(avgWidth, avgHeight),
//           avgAngle
//         );
//       }
//     };


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

function uploadImageToServer(pipeline, result, folder, filename, apiUrl) {
    with (imports) {
        try {
            print("Attempting to upload image: " + filename);
            
            // Get the clean image from the capture stage
            var mat = pipeline.getWorkingImage();

            if (mat != null && !mat.empty()) {
                // CORRECTED: Proper way to get points from RotatedRect
                var vertices = [];
                var tempPoints = new (Java.type("org.opencv.core.Point[]"))(4);
                result.points(tempPoints); // This fills tempPoints with an array of Point objects
                
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
            dos.writeBytes("--" + boundary + "\r\n");
            dos.writeBytes('Content-Disposition: form-data; name="folder"\r\n\r\n');
            dos.writeBytes(folder + "\r\n");
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
    '<cv-stage class="org.openpnp.vision.pipeline.stages.MaskRectangle" name="0" enabled="true" width="1000" height="1000"/>' +
    '<cv-stage class="org.openpnp.vision.pipeline.stages.ConvertColor" name="gray" enabled="true" conversion="Bgr2Gray"/>' +
    '<cv-stage class="org.openpnp.vision.pipeline.stages.Threshold" name="highlights" enabled="true" threshold="45" auto="false" invert="false"/>' +
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
      javax.swing.JOptionPane.showMessageDialog(null, "No job is currently loaded.");
      return;
    }
    if (job.getBoardLocations().isEmpty()) {
      javax.swing.JOptionPane.showMessageDialog(null, "The current job has no boards defined.");
      return;
    }

    for each (var boardLocation in job.getBoardLocations()) {
      var boardDefinition = boardLocation.getBoard();
      if (!boardDefinition) {
        print("WARNING: BoardLocation " + boardLocation.getId() + " has no board definition.");
        continue;
      }
      var placements = boardDefinition.getPlacements();
      var baseLocation = placements.get(0).getLocation();
      var currentInspectionLocation = baseLocation;
      var counter = 1;

      for each (var placement in placements) {
        if (!placement.isEnabled()) {
          counter++;
          continue;
        }
        var partId = placement.getPart() ? placement.getPart().getId() : "";
        if (partId.contains("Fiducial")) {
          continue;
        }
        if (counter > 1) {
          var currentX = currentInspectionLocation.getX();
          var currentY = currentInspectionLocation.getY();
          var units = currentInspectionLocation.getUnits();
          var newX = currentX;
          var newY = currentY;
          if ((counter >= 2 && counter <= 6) || (counter >= 14 && counter <= 18) || (counter >= 26 && counter <= 30) || (counter >= 38 && counter <= 42) || (counter >= 50 && counter <= 54) || (counter >= 62 && counter <= 66) || (counter >= 74 && counter <= 78) || (counter >= 86 && counter <= 90) || (counter >= 98 && counter <= 102) ) { newX += 28; }
          else if ((counter >= 8 && counter <= 12) || (counter >= 20 && counter <= 24) || (counter >= 32 && counter <= 36) || (counter >= 44 && counter <= 48) || (counter >= 56 && counter <= 60) || (counter >= 68 && counter <= 72) || (counter >= 80 && counter <= 84) || (counter >= 92 && counter <= 96) || (counter >= 104 && counter <= 108) ) { newX -= 28; }
          else if (counter == 7 || counter == 13 || counter == 19 || counter == 25 || counter == 31 || counter == 43 || counter == 49 || counter == 55 || counter == 61 || counter == 67 || counter == 79 || counter == 85 || counter == 91 || counter == 97 || counter == 103 ) { newY += 28; }
          else if (counter == 37){ newX = placement.getLocation().getX(); newY = placement.getLocation().getY(); }
          else if (counter == 73){ newX = placement.getLocation().getX(); newY = placement.getLocation().getY(); }
          currentInspectionLocation = new Location(units, newX, newY, baseLocation.getZ(), baseLocation.getRotation());
        }
        
        camera.moveTo(currentInspectionLocation);

        
        
        
        // 1. COLLECT RESULTS FROM MULTIPLE PIPELINE RUNS
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
            continue;
          }

          try {
            // Add all found rectangles from this run to our master list.
            allResults.addAll(rects);
          } catch (e) {
            // This run didn't find anything
            continue;
          }
        }

          if (allResults == null) {
            print("WARNING: Vision pipeline failed to produce a result for " + partId);
            continue;
          }


          print(
              "Vision found " +
              allResults.size() +
              " total potential rects for " +
              partId
            );



          // 2. FIND THE LARGEST CLUSTER OF RECTANGLES
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

          // 3. AVERAGE THE RECTANGLES IN THE BEST CLUSTER
          var newRect = averageRotatedRects(bestCluster);
          if (newRect == null) {
            print("WARNING: No valid rects found in the best cluster for " + partId);
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
            API_URL
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
  } finally {
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
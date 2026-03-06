load(
  scripting.getScriptsDirectory().toString() + "/Examples/JavaScript/Utility.js"
);

var imports = new JavaImporter(
  java.net.HttpURLConnection,
  java.net.URL,
  java.io.DataOutputStream,
  org.opencv.core.MatOfByte,
  org.opencv.imgcodecs.Imgcodecs,
  javax.swing.SwingUtilities,
  java.lang.Exception,
  org.openpnp.model,
  org.openpnp.util,
  java.text.SimpleDateFormat,
  java.util.Date,
  org.openpnp.util.OpenCvUtils,
  java.io.BufferedReader,
  java.io.InputStreamReader
);

var API_URL = "https://lumen-image-storage.vercel.app/api/upload-image";
var UPLOAD_FOLDER = "Post-Placements";
var RUN_NUMBER_URL = "https://lumen-image-storage.vercel.app/api/runs/get";

function getRunSession(apiUrl) {
  with (imports) {
    try {
      print("Requesting new run number from server...");
      var url = new URL(apiUrl);
      var conn = url.openConnection();
      conn.setRequestMethod("POST");
      conn.setDoOutput(true); // Required for POST to send a body, even if empty

      var responseCode = conn.getResponseCode();
      if (responseCode == HttpURLConnection.HTTP_OK) {
        var inReader = new BufferedReader(
          new InputStreamReader(conn.getInputStream())
        );
        var response = inReader.readLine(); // Read the JSON response line
        inReader.close();

        // Use regex to safely extract the run number from the JSON string
        var match = response.match(/"runNumber":(\d+)/);
        if (match && match[1]) {
          var runNumber = parseInt(match[1]);
          print("Successfully in Run #" + runNumber);
          return runNumber;
        } else {
          throw new Exception(
            "Could not parse runNumber from server response: " + response
          );
        }
      } else {
        throw new Exception(
          "Server responded with error code: " + responseCode
        );
      }
    } catch (e) {
      print("FATAL: Could not start run session. " + e);
      // Show a popup to the user
      javax.swing.JOptionPane.showMessageDialog(
        null,
        "FATAL: Could not start run session with the server.\n" + e,
        "Network Error",
        javax.swing.JOptionPane.ERROR_MESSAGE
      );
      return null;
    }
  }
}

function uploadImageToServer(image, folder, filename, apiUrl, runNumber) {
  with (imports) {
    try {
      print("Attempting to upload image: " + filename);

      var mat = OpenCvUtils.toMat(image);

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
      conn.setRequestProperty(
        "Content-Type",
        "multipart/form-data; boundary=" + boundary
      );
      var dos = new DataOutputStream(conn.getOutputStream());

      //folder
      dos.writeBytes("--" + boundary + "\r\n");
      dos.writeBytes('Content-Disposition: form-data; name="folder"\r\n\r\n');
      dos.writeBytes(folder + "\r\n");

      //runNumber
      dos.writeBytes("--" + boundary + "\r\n");
      dos.writeBytes(
        'Content-Disposition: form-data; name="runNumber"\r\n\r\n'
      );
      dos.writeBytes(runNumber.toString() + "\r\n");

      //file
      dos.writeBytes("--" + boundary + "\r\n");
      dos.writeBytes(
        'Content-Disposition: form-data; name="file"; filename="' +
          filename +
          '"\r\n'
      );
      dos.writeBytes("Content-Type: image/png\r\n\r\n");
      dos.write(imageBytes);
      dos.writeBytes("\r\n");
      dos.writeBytes("--" + boundary + "--\r\n");

      dos.flush();
      dos.close();
      var responseCode = conn.getResponseCode();
      if (responseCode == HttpURLConnection.HTTP_OK) {
        print(
          "Image uploaded successfully! Server responded with code: " +
            responseCode
        );
      } else {
        print(
          "Upload failed. Server responded with code: " +
            responseCode +
            ", " +
            conn.getResponseMessage()
        );
      }
      conn.disconnect();
    } catch (e) {
      print("An exception occurred during upload: " + e);
      e.printStackTrace();
    }
  }
}

with (imports) {
  // var coldStartX = 97.629;
  // var coldStarty = 188.174;
  // var coldStartX2 = 90.629;
  // var coldStartZ = 5.4;
  // var safeHeight = 33;
  // var NOZZLE_NAME = "N2";
  // task(function () {
  //   var head = machine.getDefaultHead();
  //   var job = gui.jobTab.job;
  //   var camera = head.getDefaultCamera();
  //   var units = LengthUnit.Millimeters;
  //   var nozzle = head.getNozzleByName(NOZZLE_NAME);
  //   var rotationAtStartOfSpin = nozzle.getLocation().getRotation();
  //   var rotation = rotationAtStartOfSpin;
  //   // --- Initial Sanity Checks ---
  //   try {
  //   if (!job) {
  //     javax.swing.JOptionPane.showMessageDialog(
  //       null,
  //       "No job is currently loaded."
  //     );
  //     return;
  //   }
  //   if (job.getBoardLocations().isEmpty()) {
  //     javax.swing.JOptionPane.showMessageDialog(
  //       null,
  //       "The current job has no boards defined."
  //     );
  //     return;
  //   }
  //   for each (var boardLocation in job.getBoardLocations()) {
  //     var boardDefinition = boardLocation.getBoard();
  //     if (!boardDefinition) {
  //       print(
  //         "WARNING: BoardLocation " +
  //           boardLocation.getId() +
  //           " has no board definition."
  //       );
  //       continue;
  //     }
  //     var placements = boardDefinition.getPlacements();
  //     var counter = 1;
  //     for each (var placement in placements) {
  //       // Skip placements that are not enabled.
  //       if (!placement.isEnabled()) {
  //         continue;
  //       }
  //       // Skip fiducials
  //       var partId = placement.getPart() ? placement.getPart().getId() : "";
  //       if (partId.contains("Fiducial")) {
  //         continue;
  //       }
  //       location = placement.getLocation();
  //       // move camera to the placement location
  //       camera.moveTo(location);
  //       // Wait for the camera to stabilize
  //       java.lang.Thread.sleep(2000);
  //       //Capture the image
  //       var bufferedImage = camera.capture();
  //       java.lang.Thread.sleep(2000);
  //       var timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
  //       var filename = counter + "_" + timestamp + ".png";
  //       // Get the run number from the server
  //       var runNumber = getRunSession(RUN_NUMBER_URL);
  //       uploadImageToServer(
  //           bufferedImage,
  //           UPLOAD_FOLDER,
  //           filename,
  //           API_URL,
  //           runNumber
  //       );
  //       counter++;
  //     }
  //   }
  //   nozzle.moveTo(new Location(units, coldStartX, coldStarty, safeHeight,rotation));
  // }finally {
  //     print("Script finished. Releasing pipeline resources.");
  //     SwingUtilities.invokeLater(function() {
  //         gui.jobTab.refresh();
  //         javax.swing.JOptionPane.showMessageDialog(null, "Post placements uploaded.");
  //     });
  //   }
  // });
}

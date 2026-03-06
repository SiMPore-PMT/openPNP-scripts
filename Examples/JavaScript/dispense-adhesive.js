/**
 * Moves nozzle "N2" to a series of specified locations and initial rotations
 * at a defined safe Z height. After each positioning, performs an additional
 * 360-degree clockwise rotation.
 * Based on the OpenPnP MoveInSquare.js example structure.
 */

// Load Examples/Utility.js functions into our scope. This should define the 'task' function.
// Make sure Utility.js is present at this path relative to your main scripts directory.
load(scripting.getScriptsDirectory().toString() + '/Examples/JavaScript/Utility.js');

// Import some OpenPnP classes we'll use
var imports = new JavaImporter(org.openpnp.model, org.openpnp.util);

// --- Script Configuration ---

// 1. Specify the name of the nozzle to use.
var nozzleNameToUse = "N2"; 

// 2. Define the Z height for safe travel and for positioning at target points.
var safeZHeight = 30.0; // In Millimeters

var dispenseHeight = 15.0;

// 3. Define the locations and initial rotations.
//    Each item: { x: X_COORD, y: Y_COORD, rotation: INITIAL_ANGLE_DEGREES }
var locationsAndRotations = [
  { x: 70.255, y: 135.5, rotation: 0.0 },
  { x: 70.255, y: 136.5, rotation: 0.0 },
  { x: 70.255, y: 137.5, rotation: 0.0 },
  { x: 70.255, y: 138.5, rotation: 0.0 },
  { x: 70.255, y: 139.5, rotation: 0.0 },
  { x: 70.255, y: 140.5, rotation: 0.0 },
  { x: 71.255, y: 140.5, rotation: 0.0 },
  { x: 72.255, y: 140.5, rotation: 0.0 },
  { x: 73.255, y: 140.5, rotation: 0.0 },
  { x: 74.255, y: 140.5, rotation: 0.0 },
  { x: 75.255, y: 140.5, rotation: 0.0 },
  { x: 75.255, y: 139.5, rotation: 0.0 },
  { x: 75.255, y: 138.5, rotation: 0.0 },
  { x: 75.255, y: 137.5, rotation: 0.0 },
  { x: 75.255, y: 136.5, rotation: 0.0 },
  { x: 75.255, y: 135.5, rotation: 0.0 },
  { x: 74.255, y: 135.5, rotation: 0.0 },
  { x: 73.255, y: 135.5, rotation: 0.0 },
  { x: 72.255, y: 135.5, rotation: 0.0 },
  { x: 71.55, y: 135.5, rotation: 0.0 },
];



// --- End of Configuration ---

// Using the imports from above, do some work.
with (imports) {

  function travel2SafeHeight (units,nozzle,safeZHeight){
  var currentNozzleLocation = nozzle.getLocation();
        print("INFO: Initial nozzle location: X=" + currentNozzleLocation.getX().toFixed(3) +
              " Y=" + currentNozzleLocation.getY().toFixed(3) + " Z=" + currentNozzleLocation.getZ().toFixed(3) +
              " R=" + currentNozzleLocation.getRotation().toFixed(3) + " " + units);

        //to travel between different locations use safeheight
        if (currentNozzleLocation.getZ() !== safeZHeight) {
            print("INFO: Moving nozzle to safe Z (" + safeZHeight.toFixed(3) + " " + units +
                  ") at current X,Y,R before starting sequence.");
            var liftLoc = new Location(units,
                                       currentNozzleLocation.getX(),
                                       currentNozzleLocation.getY(),
                                       safeZHeight,
                                       currentNozzleLocation.getRotation());
            nozzle.moveTo(liftLoc);
        }
}


	task(function() { // The 'task' function is loaded from Utility.js
		var head = machine.defaultHead;
		if (!head) {
			print("ERROR: Default head not found. Check machine configuration.");
			return;
		}

		var nozzle = head.getNozzleByName(nozzleNameToUse);
		if (!nozzle) {
			print("ERROR: Nozzle '" + nozzleNameToUse + "' not found on head '" + head.getName() + "'.");
			return;
		}
		print("INFO: Using nozzle '" + nozzle.getName() + "' on head '" + head.getName() + "'.");

		var units = LengthUnit.Millimeters; // Assuming Millimeters

        // var currentNozzleLocation = nozzle.getLocation();
        // print("INFO: Initial nozzle location: X=" + currentNozzleLocation.getX().toFixed(3) +
        //       " Y=" + currentNozzleLocation.getY().toFixed(3) + " Z=" + currentNozzleLocation.getZ().toFixed(3) +
        //       " R=" + currentNozzleLocation.getRotation().toFixed(3) + " " + units);

        // //to travel between different locations use safeheight, before dispense go to dispenseheight
        // if (currentNozzleLocation.getZ() !== safeZHeight) {
        //     print("INFO: Moving nozzle to safe Z (" + safeZHeight.toFixed(3) + " " + units +
        //           ") at current X,Y,R before starting sequence.");
        //     var liftLoc = new Location(units,
        //                                currentNozzleLocation.getX(),
        //                                currentNozzleLocation.getY(),
        //                                safeZHeight,
        //                                currentNozzleLocation.getRotation());
        //     nozzle.moveTo(liftLoc);
        // }
        //

    travel2SafeHeight(units,nozzle,safeZHeight);

		if (locationsAndRotations.length === 0) {
			print("INFO: No locations defined. Exiting script.");
			return;
		}

		print("INFO: Starting routine for " + locationsAndRotations.length + " locations.");

    // every 
		for (var i = 0; i < locationsAndRotations.length; i++) {
			var point = locationsAndRotations[i];
			print(
			  "INFO: Point " + (i + 1) + ": Moving to X=" + point.x.toFixed(3) +
			  ", Y=" + point.y.toFixed(3) + ", Initial Rotation=" + point.rotation.toFixed(3) +
			  " (at Z=" + safeZHeight.toFixed(3) + " " + units + ")"
			);

			// 1. Move to the target X, Y, Z, and initial rotation
			var targetLocation = new Location(units,
											  point.x,
											  point.y,
											  safeZHeight,
											  point.rotation);
			nozzle.moveTo(targetLocation);

      

            // Get current X, Y, and actual rotation after the move
            var currentX = point.x; // We are at the target X
            var currentY = point.y; // We are at the target Y
            // It's good practice to get the actual rotation from the nozzle after a move
            var rotationAtStartOfSpin = nozzle.getLocation().getRotation();
            print("INFO: Arrived at initial rotation: " + rotationAtStartOfSpin.toFixed(3) + " degrees.");

            // 2. Perform a 360-degree clockwise spin in 3 steps
            print("INFO: Performing 360-degree clockwise spin...");

            var spinAngle1 = rotationAtStartOfSpin + 120.0;
            var spinLocation1 = new Location(units, currentX, currentY, dispenseHeight, spinAngle1);
            print("INFO: Spin step 1/3 to " + spinAngle1.toFixed(3) + " degrees.");
            nozzle.moveTo(spinLocation1);

            var spinAngle2 = rotationAtStartOfSpin + 240.0;
            var spinLocation2 = new Location(units, currentX, currentY, dispenseHeight, spinAngle2);
            print("INFO: Spin step 2/3 to " + spinAngle2.toFixed(3) + " degrees.");
            nozzle.moveTo(spinLocation2);

            var spinAngle3 = rotationAtStartOfSpin + 360.0; // This will effectively be the same orientation as rotationAtStartOfSpin
            var spinLocation3 = new Location(units, currentX, currentY, dispenseHeight, spinAngle3);
            print("INFO: Spin step 3/3 to " + spinAngle3.toFixed(3) + " degrees (completing full circle).");
            nozzle.moveTo(spinLocation3);

            print("INFO: 360-degree spin complete for point " + (i + 1) + ".");
      nozzle.moveTo(targetLocation);
      
		}

		print("INFO: All points and spins processed successfully.");
        var finalNozzleLocation = nozzle.getLocation();
        print("INFO: Final nozzle location: X=" + finalNozzleLocation.getX().toFixed(3) +
              " Y=" + finalNozzleLocation.getY().toFixed(3) + " Z=" + finalNozzleLocation.getZ().toFixed(3) +
              " R=" + finalNozzleLocation.getRotation().toFixed(3) + " " + units);
		print("INFO: Script finished.");
	});
}


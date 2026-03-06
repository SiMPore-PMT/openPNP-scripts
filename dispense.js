
var imports = new JavaImporter(java.lang.Exception,org.openpnp.model,org.openpnp.util, org.openpnp.vision.pipeline.CvPipeline, org.openpnp.vision.pipeline.CvStage, org.opencv.core.RotatedRect);

var NOZZLE_NAME = "N2";
var safeHeight = 33;
var safeH2 = 11.5;
var dispenseHeight = 10.9;
var angle = 2;

var coldStartX = 97.629;
var coldStarty = 188.174;
var coldStartX2 = 90.629;
var coldStartZ = 5.4;
var coldStartAngle = 60;

//placement  offsets
x1= -2.0;
y1= +2.0;
x2= +2.0;
y2= -2.0;



var xml =
  '<cv-pipeline>' +
  '<stages>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.ImageCapture" name="capture" enabled="true" default-light="true" settle-option="Settle" count="1"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.MaskRectangle" name="0" enabled="true" width="600" height="600"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.ConvertColor" name="gray" enabled="true" conversion="Bgr2Gray"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.Threshold" name="highlights" enabled="true" threshold="70" auto="false" invert="false"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.BlurMedian" name="merged" enabled="true" kernel-size="13"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.FindContours" name="contours" enabled="true" retrieval-mode="List" approximation-method="Simple"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.FilterContours" name="filtered_contours" enabled="true" contours-stage-name="contours" min-area="100000.0" max-area="200000.0" property-name="FilterContours"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.MinAreaRectContours" name="rects" enabled="true" contours-stage-name="filtered_contours"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.OrientRotatedRects" name="orient" enabled="true" rotated-rects-stage-name="rects" orientation="Landscape" negate-angle="true" snap-angle="0"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.ScriptRun" name="results" enabled="true" file="/home/engineering-simpore/.openpnp2/scripts/return-always-north-angle-rect.bsh" args=""/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.ImageRecall" name="recall2" enabled="true" image-stage-name="capture"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.DrawContours" name="draw_contours" enabled="true" contours-stage-name="filtered_contours" thickness="2" index="-1"/>' +
  '<cv-stage class="org.openpnp.vision.pipeline.stages.DrawRotatedRects" name="draw_results" enabled="true" rotated-rects-stage-name="results" thickness="2" draw-rect-center="true" rect-center-radius="3" show-orientation="true">' +
  '<color r="51" g="255" b="51" a="255"/>' +
  '</cv-stage>' +
  '</stages>' +
  '</cv-pipeline>';



    function dispense() {

        with (imports) {

        


        var head = machine.getDefaultHead();
        var job = gui.jobTab.job;
        var camera = head.getDefaultCamera();
        var nozzle = head.getNozzleByName(NOZZLE_NAME);



        var pipeline = new CvPipeline(xml);


        pipeline.setProperty("camera", camera);




        if (job == null) {
            javax.swing.JOptionPane.showMessageDialog(
                null,
                "No job is currently loaded.",
                "Job Placements",
                javax.swing.JOptionPane.INFORMATION_MESSAGE
            );
            return;
        }

        if (job.getBoardLocations().isEmpty()) {
            javax.swing.JOptionPane.showMessageDialog(
                null,
                "The current job has no boards defined.",
                "Job Placements",
                javax.swing.JOptionPane.INFORMATION_MESSAGE
            );
            return;
        }

        var allBoardStrings = [];
        var boardInstanceCount = 0;

        var placementArray = [];

        for each (var boardLocation in job.getBoardLocations()) {
            boardInstanceCount++;
            var boardDefinition = boardLocation.getBoard();
            var currentBoardOutput = "";

            if (boardDefinition == null) {
                currentBoardOutput = "Board Instance " + boardInstanceCount +
                                     " (ID: " + boardLocation.getId() +
                                     ") has no associated board definition.";
                allBoardStrings.push(currentBoardOutput);
                continue;
            }

            currentBoardOutput += "Board Instance " + boardInstanceCount + ": " +
                                  boardDefinition.getName() +
                                  " (BoardLocation ID: " + boardLocation.getId() + ")\n";
            currentBoardOutput += "  Board Location: " + boardLocation.getLocation().toString() + "\n";


            var placements = boardDefinition.getPlacements();

            if (placements.isEmpty()) {
                currentBoardOutput += "  This board definition has no placements.\n";
                allBoardStrings.push(currentBoardOutput);
                continue;
            }

            var enabledPlacementDetails = "";
            var enabledPlacementCountOnBoard = 0;
            
            for each (var placement in placements) {

                if(!placement.isEnabled()){
                    continue;
                }

                enabledPlacementCountOnBoard++;
                var part = placement.getPart();
                var partId = (part != null) ? part.getId() : "N/A (No Part)";
                var placementId = placement.getId();

                if (partId === "Fiducial_1mm_Mask3mm-Fiducial"){
                    enabledPlacementCountOnBoard = enabledPlacementCountOnBoard - 1;
                    continue;
                }

                var absolutePlacementLocation = Utils2D.calculateBoardPlacementLocation(
                    boardLocation,        
                    placement.getLocation() 
                );

                placementArray.push(absolutePlacementLocation);

                enabledPlacementDetails += "    - ID: " + placementId +
                                         ", Part: " + partId +
                                         ", BoardRelLoc: " + placement.getLocation().toString() +
                                         ", JobAbsLoc: " + absolutePlacementLocation.toString() + "\n";
            }

            if (enabledPlacementCountOnBoard > 0) {
                currentBoardOutput += " Placements (" + enabledPlacementCountOnBoard + "):\n" +
                                      enabledPlacementDetails;
            } else {
                currentBoardOutput += "  No placements found on this board instance.\n";
            }
            allBoardStrings.push(currentBoardOutput);
        }

        var finalDialogMessage;
        if (allBoardStrings.length > 0) {
            // Join the information for each board location, separated by a comma and newlines
            finalDialogMessage = allBoardStrings.join("\n,\n");
        } else {
            // This case should ideally be caught by earlier checks (no boards in job)
            finalDialogMessage = "No board information to display for the current job.";
        }

        var x = 0;
        var y =0;
        var units = LengthUnit.Millimeters;
        var rotationAtStartOfSpin = nozzle.getLocation().getRotation();
        var rotation = rotationAtStartOfSpin;
        
        
        rotation -= coldStartAngle;
        nozzle.moveTo(new Location(units, coldStartX, coldStarty, safeHeight,rotation));
        nozzle.moveTo(new Location(units, coldStartX, coldStarty, coldStartZ,rotation));
        java.lang.Thread.sleep(5000);
        nozzle.moveTo(new Location(units, coldStartX, coldStarty, safeHeight,rotation))

        java.lang.Thread.sleep(5000);
        rotation -= coldStartAngle;
        nozzle.moveTo(new Location(units, coldStartX2, coldStarty, coldStartZ,rotation));
        java.lang.Thread.sleep(5000);
        nozzle.moveTo(new Location(units, coldStartX2, coldStarty, safeHeight,rotation));
        

        for (placement in placementArray){
            print(placementArray[placement].toString());    
            x = placementArray[placement].getX();
            y = placementArray[placement].getY()

            //first dispense
            nozzle.moveTo(new Location( units,x,y,safeHeight,rotation));
            nozzle.moveTo(new Location( units,x+x1,y,dispenseHeight,rotation));
            rotation -= angle ;
            nozzle.moveTo(new Location(units,x+x1,y,dispenseHeight,rotation));


            //second dispense
            nozzle.moveTo(new Location(units,x+x1,y+(y1/2),safeH2,rotation));
            nozzle.moveTo(new Location( units,x+x1,y+(y1/2),dispenseHeight,rotation));
            rotation -= angle;
            nozzle.moveTo(new Location(units,x+x1,y+(y1/2),dispenseHeight,rotation));


            //third dispense
            nozzle.moveTo(new Location( units,x+x1,y+y1,safeH2,rotation));
            nozzle.moveTo(new Location( units,x+x1,y+y1,dispenseHeight,rotation));
            rotation -= angle;         
            nozzle.moveTo(new Location( units,x+x1,y+y1,dispenseHeight,rotation));


            //fourth dispense
            nozzle.moveTo(new Location( units,x+(x1/2),y+y1,safeH2,rotation));
            nozzle.moveTo(new Location( units,x+(x1/2),y+y1,dispenseHeight,rotation));
            rotation -= angle;
            nozzle.moveTo(new Location( units,x+(x1/2),y+y1,dispenseHeight,rotation));


            //fifth dispense
            nozzle.moveTo(new Location( units,x,y+y1,safeH2,rotation));
            nozzle.moveTo(new Location( units,x,y+y1,dispenseHeight,rotation));
            rotation -= angle;
            nozzle.moveTo(new Location( units,x,y+y1,dispenseHeight,rotation));


            //sixth dispense
            nozzle.moveTo(new Location( units,x+(x2/2),y+y1,safeH2,rotation));
            nozzle.moveTo(new Location( units,x+(x2/2),y+y1,dispenseHeight,rotation));
            rotation -= angle;
            nozzle.moveTo(new Location( units,x+(x2/2),y+y1,dispenseHeight,rotation));

            //seventh dispense
            nozzle.moveTo(new Location( units,x+x2,y+y1,safeH2,rotation));
            nozzle.moveTo(new Location( units,x+x2,y+y1,dispenseHeight,rotation));
            rotation -= angle;
            nozzle.moveTo(new Location( units,x+x2,y+y1,dispenseHeight,rotation));

            //eighth dispense
            nozzle.moveTo(new Location( units,x+x2,y+(y1/2),safeH2,rotation));
            nozzle.moveTo(new Location( units,x+x2,y+(y1/2),dispenseHeight,rotation));
            rotation -= angle;
            nozzle.moveTo(new Location( units,x+x2,y+(y1/2),dispenseHeight,rotation));


            //ninth dispense
            nozzle.moveTo(new Location( units,x+x2,y,safeH2,rotation));
            nozzle.moveTo(new Location( units,x+x2,y,dispenseHeight,rotation));
            rotation -= angle;
            nozzle.moveTo(new Location( units,x+x2,y,dispenseHeight,rotation));



            //tenth dispense
            nozzle.moveTo(new Location( units,x+x2,y+(y2/2),safeH2,rotation));
            nozzle.moveTo(new Location( units,x+x2,y+(y2/2),dispenseHeight,rotation));
            rotation -= angle;
            nozzle.moveTo(new Location( units,x+x2,y+(y2/2),dispenseHeight,rotation));



            //eleventh dispense
            nozzle.moveTo(new Location( units,x+x2,y+y2,safeH2,rotation));
            nozzle.moveTo(new Location( units,x+x2,y+y2,dispenseHeight,rotation));
            rotation -= angle;
            nozzle.moveTo(new Location( units,x+x2,y+y2,dispenseHeight,rotation));



            //twelfth dispense
            nozzle.moveTo(new Location( units,x+(x2/2),y+y2,safeH2,rotation));
            nozzle.moveTo(new Location( units,x+(x2/2),y+y2,dispenseHeight,rotation));
            rotation -= angle;
            nozzle.moveTo(new Location( units,x+(x2/2),y+y2,dispenseHeight,rotation));


            //thirteenth dispense
            nozzle.moveTo(new Location( units,x,y+y2,safeH2,rotation));
            nozzle.moveTo(new Location( units,x,y+y2,dispenseHeight,rotation));
            rotation -= angle;
            nozzle.moveTo(new Location( units,x,y+y2,dispenseHeight,rotation));


            //fourteenth dispense
            nozzle.moveTo(new Location( units,x+(x1/2),y+y2,safeH2,rotation));
            nozzle.moveTo(new Location( units,x+(x1/2),y+y2,dispenseHeight,rotation));
            rotation -= angle;
            nozzle.moveTo(new Location( units,x+(x1/2),y+y2,dispenseHeight,rotation));



            //fifteenth dispense
            nozzle.moveTo(new Location( units,x+x1,y+y2,safeH2,rotation));
            nozzle.moveTo(new Location( units,x+x1,y+y2,dispenseHeight,rotation));
            rotation -= angle;
            nozzle.moveTo(new Location( units,x+x1,y+y2,dispenseHeight,rotation));


            //sixteenth dispense
            nozzle.moveTo(new Location( units,x+x1,y+(y2/2),safeH2,rotation));
            nozzle.moveTo(new Location( units,x+x1,y+(y2/2),dispenseHeight,rotation));
            rotation -= angle;
            nozzle.moveTo(new Location( units,x+x1,y+(y2/2),dispenseHeight,rotation));


            //seventeenth dispense
            nozzle.moveTo(new Location( units,x+x1,y,safeH2,rotation));
            nozzle.moveTo(new Location( units,x+x1,y,dispenseHeight,rotation));
            rotation -= angle;
            nozzle.moveTo(new Location( units,x+x1,y,dispenseHeight,rotation));



            nozzle.moveTo(new Location( units,x+x1,y,safeHeight,rotation));

    }

     nozzle.moveTo(new Location(units, coldStartX, coldStarty, safeHeight,rotation));
     
        java.lang.Thread.sleep(5000);
        nozzle.moveTo(new Location(units, coldStartX, coldStarty, coldStartZ,rotation));
        java.lang.Thread.sleep(5000);
        nozzle.moveTo(new Location(units, coldStartX, coldStarty, safeHeight,rotation));
        java.lang.Thread.sleep(20000);
        nozzle.moveTo(new Location(units, (coldStartX-3), coldStarty, coldStartZ,rotation));
        java.lang.Thread.sleep(5000);
        nozzle.moveTo(new Location(units, (coldStartX-3), coldStarty, safeHeight,rotation));
        java.lang.Thread.sleep(20000);
        nozzle.moveTo(new Location(units, (coldStartX-4), coldStarty, coldStartZ,rotation));
        java.lang.Thread.sleep(5000);
        nozzle.moveTo(new Location(units, (coldStartX-4), coldStarty, safeHeight,rotation));

    };
}
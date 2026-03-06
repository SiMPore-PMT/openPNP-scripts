from org.openpnp.gui import MainFrame
from org.openpnp.model import Placement
from org.openpnp.machine.reference.feeder import ReferenceLoosePartFeeder

job = gui.getJobTab().getJob()

if job is None:
    print "ERROR: No job is currently loaded in OpenPnP."
    print "Please load the job you want to modify and then run this script again."
else:
    print "Successfully accessed loaded job"
    placements = job.getPlacements()
    
    if not placements:
        print "The loaded job has no placements to modify."
    else:
        all_feeders = MainFrame.get().getmachine().getFeeders()
        loose_part_feeders = []
        for feeder in all_feeders:
            if isinstance(feeder, ReferenceLoosePartFeeder) and feeder.isEnabled():
                loose_part_feeders.apend(feeder)
        
        if not loose_part_feeders:
            print "ERROR: No enabled ReferenceLoosePartFeeders found"
        else:
            print "Found {} enabled loose part feeders.".format(len(loose_part_feeders))
            print "Found {} placements in the job.".format(len(placements))
            
            if len(loose_part_feeders) < len(placements):
                print "WARNING: There are fewer loose part feeders ({}) than placements ({}).".format(len(loose_part_feeders),len(placements))
                print "Not all placements will be assigned a unique feeder from this list."
                
            modified_count = 0
            feeder_index = 0
            
            for p_index, placement in enumerate(placements):
                original_feeder_id = placement.getFeederId()
                new_feeder_id = original_feeder_id
                
                if feeder_index < len(loose_part_feeders):
                    current_loose_feeder = loose_part_feeders[feeder_index]
                    new_feeder_id = current_loose_feeder.getId()
                    
                    if current_loose_feeder.getPartId() != placement.getPartId():
                        print "WARNING: Placement '{}' (Part: {}) is being assigned Feeder '{}' (Part: {}). Part IDs do not match!".format(placement.getId(), placement.getPartId(), new_feeder_id, current_loose_feeder.getPartId())
                        
                    feeder_index += 1
                    
                else:
                    print"Info: Ran out of unique loose part feeders."
                    
                    
                if new_feeder_id != original_feeder_id:
                    placement.setFeederId(new_feeder_id)
                    print "Placement '{}' (Part: {}): Feeder assigned to '{}'".format(placement.getId(), placement.getPartId(), new_feeder_id)
                    modified_count += 1
                    
            if modified_count > 0:
                job.fireJobUpdated()
                print "Finished processing. {} placements had their feeder IDs updated.".format(modified_count)
            else: 
                print "Finished Processing. No feeder IDs were changed."
            
        
        
    

import numpy as np
import json
import argparse
import glob
import os


parser = argparse.ArgumentParser()
parser.add_argument("--full_labels_json", help="json file with full gt info (pose, camera coordinates, etc.)")
parser.add_argument("--save_file", help="file to save challenge gt to")
args = parser.parse_args()

if __name__ == "__main__":
    # create dictionary that will hold all ground-truth bounding boxes
    # Format: {<object_name>: {"class": <object_class>, "centroid": [cx, cy, cz], "extent": [ex, ey, ez]}}
    # Where cx, cy, and cz are the centroid coordinates of the object in world coordinates,
    # and ex, ey, and ez are the extent of the object in the world axis directions.
    challenge_gt_dict = {}

    # Go through all files in sequence folder and update challenge_gt_dict accordingly
    # Note that we overwrite any object that is seen in more than one frame but this doesn't change the actual values.
    # TODO check if I can define leading zeros for neatness
    with open(args.full_labels_json, 'r') as f:
      original_gts = json.load(f)
    
    for original_frame_gts in original_gts.values():
      # Go through all objects within the frame's gt file and add them to the challenge_gt_dict
      for gt_id, original_gt_dict in original_frame_gts.items():
        if gt_id in ["_metadata", "pose_rotation", "pose_translation", "pose"]:
          continue
        
        gt_name = original_gt_dict['ID_name']
        challenge_gt_dict[gt_name] = {"class": original_gt_dict["class"],
                                      "centroid": original_gt_dict["origin_in_world"],
                                      "extent": original_gt_dict["extent_in_world"],
                                      "ID_name": original_gt_dict['ID_name']}
        # Update from centimetres to metres
        challenge_gt_dict[gt_name]["extent"][0] /= 100.
        challenge_gt_dict[gt_name]["extent"][1] /= 100.
        challenge_gt_dict[gt_name]["extent"][2] /= 100.
        challenge_gt_dict[gt_name]["centroid"][0] /= 100.
        challenge_gt_dict[gt_name]["centroid"][1] /= 100.
        challenge_gt_dict[gt_name]["centroid"][2] /= 100.

        # Update centroid location to fit RH coordinate system rather than original LH coordinates
        challenge_gt_dict[gt_name]["centroid"][1] *= -1
    
    with open(args.save_file, 'w') as f:
      json.dump(challenge_gt_dict, f)

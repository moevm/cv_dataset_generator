#!/bin/sh

while read pos; do
    rosrun camera_controls move.py camera $(echo "$pos")
    rosrun camera_controls save_image.py camera
done < "$1";
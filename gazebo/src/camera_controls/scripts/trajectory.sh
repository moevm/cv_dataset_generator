#!/bin/sh

i=0;
while read pos; do
    rosrun camera_controls move.py camera $(echo "$pos")
    rosrun camera_controls save_image.py camera $(printf "%03d_" "$i")
    i=$((i + 1))
done < "$1";
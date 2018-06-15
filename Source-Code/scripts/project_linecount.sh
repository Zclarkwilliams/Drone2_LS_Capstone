#!/bin/bash

SRC_DIR="../Drone2/impl1/source"
TOTAL_LC=0
HEADER_COMMENT_LC=40

for file in $SRC_DIR/*.v;
do
	let "TOTAL_LC += $(wc -l "$file" | awk '{print $1}') - HEADER_COMMENT_LC"
done

echo "Estimated Line Count: $TOTAL_LC"

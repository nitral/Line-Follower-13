# Line Follower 2013 | Source Code Condensation Script.
# Script to Condense Entire Source Code into a single File.

# Source Code Condensation Script Begins.

echo "Source Code Condensation - Line Follower 2013"
echo "---------------------------------------------"

echo "Starting Source Code Condensing Process..."
echo ""

echo "Merging Header Files..."
echo "/* ****************************" > inc/condensedHeader.h
echo "Header Files for Line Follower." >> inc/condensedHeader.h
echo "**************************** */" >> inc/condensedHeader.h
echo "" >> inc/condensedHeader.h
cat inc/lineFollowerMain.h >> inc/condensedHeader.h
echo "" >> inc/condensedHeader.h
echo "" >> inc/condensedHeader.h
cat inc/adci.h >> inc/condensedHeader.h
echo "" >> inc/condensedHeader.h
echo "" >> inc/condensedHeader.h
cat inc/normalizeAlgorithm.h >> inc/condensedHeader.h
echo "" >> inc/condensedHeader.h
echo "" >> inc/condensedHeader.h
cat inc/moveAlgorithm.h >> inc/condensedHeader.h
echo "" >> inc/condensedHeader.h
echo "" >> inc/condensedHeader.h
echo "Header Files Merged."
echo ""

echo "Merging Program Source Files..."
echo "/* ***********************************" > src/lineFollowerSourceCondensed.c
echo "Source Code For Functions Begins Here." >> src/lineFollowerSourceCondensed.c
echo "*********************************** */" >> src/lineFollowerSourceCondensed.c
sed '/^\#/d' src/lineFollower.c >> src/lineFollowerSourceCondensed.c
sed '/^\#/d' src/initializeAll.c >> src/lineFollowerSourceCondensed.c
sed '/^\#/d' src/inputSensorArray.c >> src/lineFollowerSourceCondensed.c
sed '/^\#/d' src/isRunningFeasible.c >> src/lineFollowerSourceCondensed.c
sed '/^\#/d' src/normalizeSensorArray.c >> src/lineFollowerSourceCondensed.c
sed '/^\#/d' src/moveLineFollower.c >> src/lineFollowerSourceCondensed.c
echo "Program Source Files Merged."
echo ""

echo "Creating Condensed Source Code File..."
echo "/* ********************************************" > lineFollowerCondensed.c
echo "Line Follower 2013 | Condensed Source Code File" >> lineFollowerCondensed.c
echo "******************************************** */" >> lineFollowerCondensed.c
echo "" >> lineFollowerCondensed.c
echo "Condensed Source Code File Created Successfully."
echo ""

echo "Including Library Header Files..."
echo "// Library Header Files" >> lineFollowerCondensed.c
echo "" >> lineFollowerCondensed.c
echo "#include <avr/io.h>" >> lineFollowerCondensed.c
echo "#include <util/delay.h>" >> lineFollowerCondensed.c
echo "#include <math.h>" >> lineFollowerCondensed.c
echo "#include <stdlib.h>" >> lineFollowerCondensed.c
echo "" >> lineFollowerCondensed.c
echo "Library Header Files Included."
echo ""

echo "Merging Everything..."
cat inc/condensedHeader.h src/lineFollowerSourceCondensed.c >> lineFollowerCondensed.c
echo "Merged Everything."
echo ""

echo "Source Code has been Condensed Successfully."
echo ""

echo "Cleaning Up..."
rm inc/condensedHeader.h src/lineFollowerSourceCondensed.c
echo "Cleanup Done."

echo "---------------------------------------------"

# End of Source Code Condensation Script.
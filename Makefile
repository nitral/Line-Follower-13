COMPILER = avr-gcc
CFLAG = -c -Wall -g
OFLAG = -o
LIB = -lm
ARCHIVER = ar
AROP = cr

binaries:					bin/lineFollower

bin/lineFollower: 			obj/initializeAll.o obj/inputSensorArray.o obj/isRunningFeasible.o obj/lineFollower.o lib/lineFollower.so obj/moveLineFollower.o obj/normalizeSensorArray.o
							$(COMPILER) obj/lineFollower.o lib/lineFollower.so $(LIB) $(OFLAG) bin/lineFollower

obj/lineFollower.o:			src/lineFollower.c inc/lineFollowerMain.h
							$(COMPILER) $(CFLAG) src/lineFollower.c $(OFLAG) obj/lineFollower.o

obj/initializeAll.o:		src/initializeAll.c inc/lineFollowerMain.h
							$(COMPILER) $(CFLAG) src/initializeAll.c $(OFLAG) obj/initializeAll.o

obj/inputSensorArray.o:		src/inputSensorArray.c inc/lineFollowerMain.h inc/adci.h
							$(COMPILER) $(CFLAG) src/inputSensorArray.c $(OFLAG) obj/inputSensorArray.o

obj/isRunningFeasible.o:	src/isRunningFeasible.c inc/lineFollowerMain.h
							$(COMPILER) $(CFLAG) src/isRunningFeasible.c $(OFLAG) obj/isRunningFeasible.o

obj/normalizeSensorArray.o: src/normalizeSensorArray.c inc/normalizeAlgorithm.h
							$(COMPILER) $(CFLAG) src/normalizeSensorArray.c $(OFLAG) obj/normalizeSensorArray.o
							
obj/moveLineFollower.o:		src/moveLineFollower.c inc/lineFollowerMain.h inc/moveAlgorithm.h
							$(COMPILER) $(CFLAG) src/moveLineFollower.c $(LIB) $(OFLAG) obj/moveLineFollower.o

lib/lineFollower.so:		obj/initializeAll.o obj/inputSensorArray.o obj/isRunningFeasible.o obj/normalizeSensorArray.o obj/moveLineFollower.o
							$(ARCHIVER) $(AROP) lib/lineFollower.so obj/initializeAll.o obj/inputSensorArray.o obj/isRunningFeasible.o obj/normalizeSensorArray.o obj/moveLineFollower.o
	
cleanall:
							rm -f obj/* bin/* lib/*

clean:
							rm -f obj/*
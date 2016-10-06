#############################################
# Simple makefile for execution of project2
# set path = ($path /cygdrive/c/cygwin64/bin)
#############################################
all: raycast.c
	gcc raycast.c -o raycast

clean:
	rm raycast.exe

# assuming VERBOSE is set to 1 in raycast.c
verbose: raycast.c
	gcc raycast.c -o raycast_v

#############################################
# targets for testing quadrics
#############################################
ellipsoid:
	gcc raycast.c -o raycast
	./raycast 600 600 ellipsoid.json ellipsoid.ppm

cylinder:
	gcc raycast.c -o raycast
	./raycast 600 600 cylinder.json cylinder.ppm

cone:
	gcc raycast.c -o raycast
	./raycast 600 600 cone.json cone.ppm

#############################################
# targets for testing good and bad json files
#############################################
testgood.%:
	gcc raycast.c -o raycast
	./raycast 600 600 test_good_$*.json test.ppm
	emacs test.ppm

testbad.%:
	gcc raycast.c -o raycast
	./raycast 600 600 test_bad_$*.json test.ppm

#############################################
# targets for simple test configs
#############################################
test1:
	gcc raycast.c -o raycast
	./raycast 60 30 test_good_01.json test.ppm
	emacs test.ppm

test2:
	gcc raycast.c -o raycast
	./raycast 1000 800 test_good_02.json test.ppm
	emacs test.ppm

test3:
	gcc raycast.c -o raycast
	./raycast 600 500 test_good_03.json test.ppm

test4:
	gcc raycast.c -o raycast
	./raycast 600 500 test_good_04.json test.ppm

test5:
	gcc raycast.c -o raycast
	./raycast 1000 800 test_good_05.json test.ppm
	emacs test.ppm

test6:
	gcc raycast.c -o raycast
	./raycast 1000 800 test_good_06.json test.ppm
	emacs -geometry 200x60 test.ppm

testexample:
	gcc raycast.c -o raycast
	./raycast 1000 800 example.json test.ppm
	emacs -geometry 200x60 test.ppm

# trafficLightRecognition

Features:

Various color classifiers:

1. gaussian color classifiers for segmenting colors

2. histogram backprojection for segmenting colors(colorSegmentation/backproject.cpp)

Falsecandidate reduction:

1. Basic BLOB analysis(blobAnalysis/blobAnalysis.cpp)

2. Finding shape(shapeSegmentation/edgeSegmentation.cpp)

3. Integrate stereo

  a. reject regions by looking at region’s cluster size and shape against the distance

  b. look at the volume around the candidate and determine if it looks right

4. tracking of found traffic signals

Light state classification:

1. Determine state using structural information

2. State machine

Tested on OSX, build using cmake 3.10. Compile instructions follows:

cmake .

make

./main <path to video clip>

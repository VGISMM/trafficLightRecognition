#include "evaluation.h"
Evaluation::Evaluation(){

}

void Evaluation::init(string type){
    typeLabel = type;
    evalFileScoreStream << "../Data/output/" << type << "Scores.txt";
    scoresPath = evalFileScoreStream.str();
    scoreOutputStream.open(scoresPath);

    evalFileLabelStream << "../Data/output/" << type << "Target.txt";
    labelPath = evalFileLabelStream.str();
    labelOutputStream.open(labelPath);

    resultsOutputStreamString << "../Data/output/" << type << "Results.csv";
    resultsPath = resultsOutputStreamString.str();
    resultsOutputStreamFile.open(resultsPath);
}

void Evaluation::endInputStream(){
  GTInputStream->close();
}

void Evaluation::endOutputStream(){
  scoreOutputStream.close();
  labelOutputStream.close();

  resultsOutputStreamFile.close();
}

void Evaluation::loadGT(){
    

    string str;
    string delimframeNumber = "\t";

    while (std::getline(*GTInputStream, str)) 
    {
        int delimCounter=1;
        expandedAnnotationVector.push_back(str);
        size_t pos = 0;
        string token;
        while (( (pos = str.find(delimframeNumber)) != std::string::npos) && (delimCounter<=1)) 
        {
            token = str.substr(0, pos);
            compressedAnnotationVector.push_back(token);
            str.erase(0, pos + delimframeNumber.length());
            delimCounter++;
        }
    }
}

void Evaluation::readFrameAnnotations(int frameNumber){
    frameNumberIndex = frameNumber;
    GTrects.clear();
    for (unsigned n=0; n<compressedAnnotationVector.size(); ++n) 
    {
      int annotationNumber = atoi(compressedAnnotationVector.at( n ).c_str());
      if (frameNumber == annotationNumber)
      {
        std::vector<std::string> x = split(expandedAnnotationVector.at( n ), '\t');
        int amountAnnotedObjects= atoi(x.at( 1 ).c_str());
        int annotationIndex = 1;
        int localX1 = 2;
        int localX2 = 3;
        int localX3 = 4;
        int localX4 = 5;

        while (annotationIndex <= amountAnnotedObjects)
        { 
          int P0x = atoi(x.at( localX1 ).c_str());
          int P0y = atoi(x.at( localX2 ).c_str());
          int widthValue = atoi(x.at( localX3 ).c_str());
          int heightValue =  atoi(x.at( localX4 ).c_str());
          int PCx = P0x+widthValue/2;
          int PCy = P0y+heightValue/2;
          GTrects.push_back(cv::Rect(P0x,P0y,widthValue,heightValue));

          localX1 = localX1 + 4;
          localX2 = localX2 + 4;
          localX3 = localX3 + 4;
          localX4 = localX4 + 4;
          annotationIndex  = annotationIndex + 1;
        }
      }
    } 
}

void Evaluation::cleanUpCandidates(){
    trafficSignalCandidates.clear();
    /*
    for (int rectIndex1 = 0; rectIndex1<preTrafficSignalCandidates.size(); rectIndex1++)
    {
        trafficSignalCandidates.push_back(preTrafficSignalCandidates[rectIndex1]);
    }
    */


    for (int rectIndex1 = 0; rectIndex1<preTrafficSignalCandidates.size(); rectIndex1++)
    {
        bool Overlap = false;
        bool OverlapWinner = false;
        bool OverlapLooser = false;
        for (int rectIndex2 = 0; rectIndex2<preTrafficSignalCandidates.size(); rectIndex2++)
        {
            if(rectIndex2 != rectIndex1)
            {
                cv::Rect2d Rinter = preTrafficSignalCandidates[rectIndex1].rect2d & preTrafficSignalCandidates[rectIndex2].rect2d;
                //cv::Rect2d Runion = preTrafficSignalCandidates[rectIndex1].rect2d | preTrafficSignalCandidates[rectIndex2].rect2d;
                float interesectArea = Rinter.width*Rinter.height;
                float unionArea = Rinter.width*Rinter.height+preTrafficSignalCandidates[rectIndex1].rect2d.width*preTrafficSignalCandidates[rectIndex1].rect2d.height-interesectArea;
                
                if (Rinter.area() > 0 && interesectArea/unionArea > 0.5)
                {
                    Overlap = true;
                    if(preTrafficSignalCandidates[rectIndex1].TLConfidence >= preTrafficSignalCandidates[rectIndex2].TLConfidence)
                    {
                        OverlapWinner = true;
                        //cout << "OverlapWinner" << " index: " << rectIndex1 << endl;
                          // erase the 6th element
                        //myvector.erase (myvector.begin()+5);
                    }
                    else
                    {
                        OverlapLooser = true;
                    }
                }    
                else
                {
                 //cout << "Non-overlapping Rectangles" << endl;
                    Overlap = false;
                    //trafficSignalCandidates.push_back(preTrafficSignalCandidates[rectIndex1]);
                }
            }
        }
        if(!Overlap && !OverlapLooser)
        {
            trafficSignalCandidates.push_back(preTrafficSignalCandidates[rectIndex1]);
        }
        else if(OverlapWinner && !OverlapLooser)
        {   
            trafficSignalCandidates.push_back(preTrafficSignalCandidates[rectIndex1]);
        }
    }

    
}

void Evaluation::evaluateVIVA(){
    



}


void Evaluation::evaluateStandard(string frameName){
    cleanUpCandidates();

        // write viva evaluation
    for( int i = 0; i< trafficSignalCandidates.size(); i++ ){   
                
        resultsOutputStreamFile << frameName << ";" << trafficSignalCandidates[i].rect2d.x << ";" << trafficSignalCandidates[i].rect2d.y << ";" << trafficSignalCandidates[i].rect2d.x+trafficSignalCandidates[i].rect2d.width << ";" << trafficSignalCandidates[i].rect2d.y+trafficSignalCandidates[i].rect2d.height << ";" << trafficSignalCandidates[i].TLConfidence << "\r\n"; 
        
    }
    //cout << "E size " << trafficSignalCandidates.size() << endl;
    bool intersectBool = false;

    FNframe = 0;
    FPframe = 0;
    TPframe = 0;

    // Draw Annotations cout << "GT size " << GTrects.size() << endl;
    for (int rectIndex = 0; rectIndex<GTrects.size(); rectIndex++)
    {
        if(GTrects[rectIndex].x > 0 && GTrects[rectIndex].y > 0 && GTrects[rectIndex].x+GTrects[rectIndex].width < 1280 && GTrects[rectIndex].y+GTrects[rectIndex].height < 960)
        {
            cv::rectangle(annotationImage, GTrects[rectIndex], cv::Scalar(255, 255, 255),2,1); //GT
            //cv::rectangle(imgPressentationColor, cv::Point(GTrects[rectIndex].x, GTrects[rectIndex].y) ,cv::Point(GTrects[rectIndex].x+GTrects[rectIndex].width, GTrects[rectIndex].y+GTrects[rectIndex].height), cv::Scalar(255, 255, 255),2,1); //GT
        }
    }

    // Draw detections
    for( int i = 0; i< trafficSignalCandidates.size(); i++ ){   
        rectangle(annotationImage, trafficSignalCandidates[i].rect2d, cv::Scalar( 0, 0, 255 ), 2, 4 ); // All detection
        for (int y = 0; y<GTrects.size(); y++)
        {             
            //rectangle(imgPressentationSpotlight, trafficSignalCandidates[i].rect2d, cv::Scalar( 255, 0, 0 ), 2, 4 ); // FP detection
            cv::Rect interesect  = GTrects[y] & trafficSignalCandidates[i].rect2d;
            if(interesect != cv::Rect())
            {
                float interesectArea = interesect.width*interesect.height;
                float unionArea = GTrects[y].width*GTrects[y].height+trafficSignalCandidates[i].rect2d.width*trafficSignalCandidates[i].rect2d.height-interesectArea;
                //cout << "area " << interesectArea/unionArea << endl;
                if(interesectArea/unionArea > 0.5)
                {
                    intersectBool = true;
                    trafficSignalCandidates[i].TP = true;
                    trafficSignalCandidates[i].FP = false;
                    trafficSignalCandidates[i].TN = false;
                }
            }
        }
        if(intersectBool)
        {
            rectangle(annotationImage, trafficSignalCandidates[i].rect2d, cv::Scalar( 0, 255, 0 ), 2, 4 ); // TP detection
            intersectBool = false;
            TPframe++;
        }
    }

    FNframe = GTrects.size()-TPframe;
    FPframe = trafficSignalCandidates.size() - TPframe;
    
    char auhaSpot[40];
    for( int e = 0; e< trafficSignalCandidates.size(); e++ ){
      
      if(trafficSignalCandidates[e].TP && !trafficSignalCandidates[e].FP)
      {
        sprintf(auhaSpot,"%f\t",trafficSignalCandidates[e].TLConfidence);
        scoreOutputStream << auhaSpot << endl;
        labelOutputStream << 1 << endl;
      }
      else if(trafficSignalCandidates[e].FP && !trafficSignalCandidates[e].TP)
      {
        sprintf(auhaSpot,"%f\t",trafficSignalCandidates[e].TLConfidence);
        scoreOutputStream << auhaSpot << endl;
        labelOutputStream << 0 << endl;
      }
    }    
    for(int index = 0; index < FNframe; index++)
    {
        sprintf(auhaSpot,"NaN");
        scoreOutputStream << auhaSpot << endl;
        labelOutputStream << 1 << endl;
    }

    FNtotal += FNframe;
    FPtotal += FPframe;
    TPtotal += TPframe;
    cout << "frame: " << frameNumberIndex << " Detector: " << typeLabel  << " FNtotal: " << FNtotal << " FPtotal: " << FPtotal << " TPtotal: " << TPtotal << endl;  
}



std::vector<string> Evaluation::split(const string &s, char delim) {
    std::vector<string> elems;
    stringstream ss(s);
    string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}
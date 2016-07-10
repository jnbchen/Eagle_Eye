#include "ConeDetection.h"


//======== function to detect edge in region of interest defined by corner coordinates (row,col) of the cone =====================

void ConeDetection::detectEdge(int row, int left_right, char cam, EdgeData& EdgeData_res){     //left edge (col=beg) => left_right=-1 <->  right edge (col=end) => left_right=1



  int col= EdgeData_res.start; //set column number: start point of edge (=begin or end point of cone top side)

  //initialize counter
  EdgeData_res.sum_u=0;
  EdgeData_res.sum_v=0;
  EdgeData_res.n=0;
  
  //declare pointers
  const uchar* EdgeH; 
  const uchar* EdgeS;
  const uchar* EdgeV;
  uchar* EdgeOut;
  

  //----------iterate through rows------------------------------------------
  for(int iE=row; iE<std::min(img[0].rows, row+height);iE++){ 

    if(cam=='L'){
      EdgeH=img[0].ptr<uchar>(iE); //pointer to i-th row
      EdgeS=img[1].ptr<uchar>(iE);
      EdgeV=img[2].ptr<uchar>(iE);
      EdgeOut=out0.ptr<uchar>(iE);
    }
    else{
      EdgeH=imgR[0].ptr<uchar>(iE); //pointer to i-th row
      EdgeS=imgR[1].ptr<uchar>(iE);
      EdgeV=imgR[2].ptr<uchar>(iE);
      EdgeOut=out0R.ptr<uchar>(iE);   //define channel where to plot ROI and detected pixels is channel 0
    }


 

    int jEdge=col+left_right*(iE-row)*1/m_cone;      //ROI center: column number dependent on row number ( line starting in col (col=left or right cone top side vertex)).

    int jE0=std::max(1,jEdge-ROI_half_width);        //where to begin (a few  points in front of the ROI center)
    EdgeOut+=jE0;                                    //move pointer from first column in the current row to ROI begin
    
    //--------------iterate through columns------------------------------------------    
    for(int jE=jE0; jE<std::min(jEdge+ROI_half_width, img[0].cols-20); jE++){ //consider filter mask width in what to choose as minimum column number
      //std::cout<<"jE "<<jE<<std::endl;
      
      *EdgeOut+=50;                                  //PLOT all considered pixels (=region of interest) in Edge Channel



      if(
	 //left edge and filter mask [-2,1] with threshold 40
	 (left_right==-1
	   &&      std::max( 0,  diff_to_Orange(EdgeH[jE],EdgeS[jE],EdgeV[jE])- 2*diff_to_Orange(EdgeH[jE-1],EdgeS[jE-1],EdgeV[jE-1]) )     >40  )
	  ||
	 //or right edge and filter mask [1,-2] with threshold 40
	  (left_right==+1
	   &&      std::max( 0,  diff_to_Orange(EdgeH[jE],EdgeS[jE],EdgeV[jE])- 2*diff_to_Orange(EdgeH[jE+1],EdgeS[jE+1],EdgeV[jE+1]) )     >40   )  
	  ){
	EdgeData_res.sum_u+=jE;
	EdgeData_res.sum_v+=iE;
	EdgeData_res.n++;

	EdgeData_res.c_list.push_back(iE+left_right*m_cone*jE); //y-offset of line with m_cone going through point u=jE, v=iE; add to list (only needed if median should be calculated)
	
	*EdgeOut=255; //PLOT detected edge pixels
      }

      
      // :::::::::::: or no edge point ::::::::::::::::::::::
      // nothing to do
    
     
      // move pointer to next column
      EdgeOut++;
      
    }
	       
  }
}







//////////////////////////////////////////////////////////////////////////////

// image preparation for edge search


 /*
    Mat getThresholded(const Mat& input) {
        // Region of Interest
        Rect roi(0, 0, inout.rows, input.cols);
        Mat im_roi = input(roi);


        // Blur and HSV
        int median_filter_size = 3;
        medianBlur(im_roi, im_roi, median_filter_size);
        Mat im_hsv;
        cvtColor(im_roi, im_hsv, COLOR_BGR2HSV);


        // Color thresholing
        Mat hue_range;
        int hue_low = 0
        int hue_high = 20;
        int sat_low = 25;
        int sat_high = 255;
        int val_low = 90;
        int val_high = 255;

        inRange(im_hsv, cv::Scalar(hue_low, sat_low, val_low),
                cv::Scalar(hue_high, sat_high, val_high), hue_range);


        // Opening/Closing
        int erode_size = 2;
        int dilate_size = 2;
        Mat element_erode = getStructuringElement(MORPH_ELLIPSE,
                                                  Size(2*erode_size + 1, 2*erode_size+1),
                                                  Point(-1, -1));
        Mat element_dilate = getStructuringElement(MORPH_ELLIPSE,
                                                   Size(2*dilate_size + 1, 2*dilate_size+1),
                                                   Point(-1, -1));
        // Opening
        erode(hue_range, hue_range, element_erode);
        dilate(hue_range, hue_range, element_dilate);

        return hue_range;
    }
    */

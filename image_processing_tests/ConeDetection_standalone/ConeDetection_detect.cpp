#include "ConeDetection.h"


//=====core function =======================================================================

void ConeDetection::detect (char camera){
  if (camera=='S'){
    current_camera='L';
  }
  else{
    current_camera=camera; //set global variable to current camera left (L) or right(R)
  }
  
  //------local variables---------------------------------------------------------------
  int beg, end;                                 // begin and end of current truncated cone top side
  int delta;                                    // critical pixel distance, when to assume that pixels ahead would no more belong to the current cone
  int width_min;                                // width minus tolerance
  int width_max;                                // width plus tolerance
  std::vector<int> hidden_beg, hidden_end, hidden_row;     //vectors with left point of cone top side, with corresponding right point and with the row number of the detected cone top side
  EdgeData EDl, EDr, EDlR,EDrR; // e.g. right edge (r) in right camera (R)
  bool valid_right_cone;

  int n_min; // min number of edge pixels to accept edge
  int accepted_v_diff; //accepted difference between v-coordinate of apex estimation

  const uchar* currentH;
  const uchar* currentS;
  const uchar* currentV;  //pointer to i-th row of ...-channel
  uchar* output0;
  uchar* output1;
  uchar* output2;

  // plot horizont
  cv::line(out2, cv::Point(0,v0),cv::Point(img[0].cols,v0),100); //PLOT line in channel 2

  //------iterate through rows----------------------------------------------------------
  for(int i=v_search_beg; i<v_search_end;i++){ 

    
    //set pointers to i-th row
    if(camera=='L' || camera=='S'){
      currentH=img[0].ptr<uchar>(i); //pointer to i-th row of H-channel
      currentS=img[1].ptr<uchar>(i); //pointer to i-th row of S-channel
      currentV=img[2].ptr<uchar>(i); //pointer to i-th row of V-channel
      output0=out0.ptr<uchar>(i);            //pointer to i-th row of first out-channel
      output1=out1.ptr<uchar>(i);            //pointer to i-th row of second out-channel
      output2=out2.ptr<uchar>(i);            //pointer to i-th row of third out-channel
    }
    else{
      currentH=imgR[0].ptr<uchar>(i); //pointer to i-th row of H-channel
      currentS=imgR[1].ptr<uchar>(i); //pointer to i-th row of S-channel
      currentV=imgR[2].ptr<uchar>(i); //pointer to i-th row of V-channel
      output0=out0R.ptr<uchar>(i);            //pointer to i-th row of first out-channel
      output1=out1R.ptr<uchar>(i);            //pointer to i-th row of second out-channel
      output2=out2R.ptr<uchar>(i);            //pointer to i-th row of third out-channel
    }


    //optional: left rand not to scan (because not in right stereo image available, or because of edges from rectification)
    u_search_beg=30;
    output0+=u_search_beg;                       //move pointer to column
    output1+=u_search_beg;
    output2+=u_search_beg;

    //initialize beg and end counter
    beg=-2;                            // -2 means that no cone begin was detected, and there must be a non-orange pixel in front of a valid cone (in order to have no cone begin next to the left rand or next to a hidden region
    end=-2;                            // no active cone

    //evaluate row-dependent parameters
    width=meter2pixel(w,h,v0-i);  // width of top side if truncated cone top side  would be in this row.
    width_min= width_min_prop_to_width*width+width_min_const;
    width_max=width_max_prop_to_width*width+width_max_const; //width_max=1.3*width+2;
    delta= delta_prop_to_width*width;        //critical distance to last orange pixel
      
    height=meter2pixel(H,h,v0-i); //// height of region of interest, dependent on row, global variable !!!

    
    //----iterate through columns---------------------------------------------------------------------
    for (int j=u_search_beg; j<img[0].cols-1; j++){
      
      if(isHidden(j,i,hidden_beg,hidden_end,hidden_row)){*output0+=100;} //PLOT hidden regions in Channel 0

      //......either pixel is orange...................................................................
      if( isOrange(currentH[j],currentS[j],currentV[j]) ){

	//either  new cone
	if(beg==-1){
	  beg=j;
	  end=j;
	}

	//or not relevant: cone with top side here would be hidden by already detected cone
	else if(isHidden(j,i,hidden_beg,hidden_end,hidden_row) ){
	  beg=-2;
	  end=-2;
	}
	
	//no active cone, and there was no non-orange pixel between the left rand or a hidden region and the current pixel => nothing to do
	else if(beg==-2 || end==-2){
	  ;
	}

	//or  pixel belongs to current cone
	else{
	  end=j;
	}
	*output1=100;                                                  //PLOT all orange Pixels in ROI in Channel 1
      }

      //........or pixel is not orange...............................................................
      else {

	//******* critical distance to last orange pixel  => cone finished **************************
	if (j-end==delta){ 

	  //::::::::::::: either suitable width of detected cone => VALID CONE DETECTED:::::::::::::::
	  if(end-beg>=width_min && end-beg<=width_max && beg!=u_search_beg){ 

	    for (int z=beg; z<=end; z++){*(output1-j+z)+=150;} //plot line from cone begin to cone end

	    // define hidden region
	    hidden_beg.push_back(beg-10);  //10
	    hidden_end.push_back(end+10);
	    hidden_row.push_back(i);

	    EDl.start=beg;
	    EDr.start=end;

	    //detect edges
	    detectEdge(i,-1,current_camera, EDl);      //detect left edge
	    detectEdge(i,+1,current_camera, EDr);      //detect right edge


	    n_min= n_min_prop_to_height *height + n_min_const ; //min. number of edge pixels to accept as valid edge

	    if(EDl.n>n_min && EDr.n>n_min){
	       
	      //estimate cone apex from left or right camera image
	      if(camera!='S'){
		estimateApexMono(current_camera,EDl,EDr);
		// visualize expected disparity
		visualizeDispExpectation(beg, end, i);
	     
	      }



	      //if camera mode is 'S' (Stereo): estimate cone apex and disparity from both images
	      if(camera=='S'){

      
		// search for cone in right stereo image based on the current result in left camera image
		valid_right_cone=0;
		searchRightImage(beg, end, i, EDlR, EDrR, valid_right_cone);

		//if corresponding right cone top side found
		if(valid_right_cone ==1){

		  //search for left and right cone edge
		  detectEdge(i,-1,'R', EDlR);
		  detectEdge(i,1,'R', EDrR);

		  //if enough edge pixels found
		  if( EDlR.n>n_min && EDrR.n>n_min){

		    //estimate apex in left and right image
		    estimateApexStereo(EDl,EDr,EDlR,EDrR);

		    //if not too large difference in apex height: calculate x,y (vehicle-fixed coordinate system) coordinates
		    accepted_v_diff = ac_v_diff_prop_to_height * height+ ac_v_diff_const;
		    if(std::abs( EDl.apex_v-EDlR.apex_v)<accepted_v_diff){
		      cv::line(out2, cv::Point(EDl.apex_u,EDl.apex_v),cv::Point(EDlR.apex_u,EDlR.apex_v),100);
		      determinePosition(EDl);
		    }
		    // else{
		    // 	std::cout<<"vertical distance of apex estimations: "<< std::abs( EDl.apex_v-EDlR.apex_v)<<" > accepted distance "<< accepted_v_diff <<std::endl;
		    // }

	      
		  }
		  // else{
		  // 		std::cout<<"not enough edge pixel found in right image: n_min= "<<n_min<< " [right cam - left flank]: "<< EDlR.n<<" [right cam - right flank]: "<<EDrR.n<<std::endl;
		  // }



		}
	    
	      }

	    }
	    // else{
	    // 	std::cout<<"not enough edge pixel found in left image: n_min= "<<n_min<< " [left cam,  left flank]: "<< EDl.n<<" [left cam, right flank]: "<<EDr.n<<std::endl;
	    // }

	  }

	  
	  //::::::::::: or no suitable width  ::::::::::::::::::::::::::::::::::::::::::::::::::::::::
	  //   => nothing to do 

	  
	  //cone finished => reset beg and end counter to not active (-1)
	  beg=-1; 
	  end=-1;
	}
	
	//*************else:  critical distance to last orange pixel not reached *********************
	// non-orange pixel detected => can remove "-2" blocker, because there has been a non orange pixel after the rand or hidden region.
	if (beg==-2 || end==-2){
	  beg=-1;
	  end=-1;
	}


	
      }

      if(j==200){ *output1=100;	*(output1-width)=100;      }  //plot width

      output0++; //move pointer to next column
      output1++;
      output2++;
    }
  }
}


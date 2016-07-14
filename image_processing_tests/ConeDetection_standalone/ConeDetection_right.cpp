#include "ConeDetection.h"



// //========== search in right stereo image for corresponding cone
void ConeDetection::searchRightImage(int beg, int end, int i, EdgeData &EDlR, EdgeData & EDrR, bool & valid_right_cone){
  //search only in current row; begin in beg-disp-tolerance (see visualizeDispExpectation() ), search for orange region with suitable with (see detect() )
  const uchar* currentH; //pointer to i-th row of H-channel
  const uchar* currentS; //pointer to i-th row of S-channel
  const uchar* currentV; //pointer to i-th row of V-channel
  uchar* output2R;            //pointer to i-th row of third out-channel
  uchar* output1R;            //pointer to i-th row of third out-channel
  uchar* output0R;            //pointer to i-th row of third out-channel
  int widthR=end-beg;
  int disp=meter2pixel(b,h,v0-i);        //expected disparity
  int j0=beg-disp;
  
  //int delta=0.5*widthR;
  
  int tol=2*widthR+30; // half width of region where to search
  // int width_min=0.5*(end-beg)-1;
  //int width_max=1.5*(end-beg)+1;

  //evaluate row-dependent parameters
  int  width=meter2pixel(w,h,v0-i);  // width of top side if truncated cone top side  would be in this row.
  int width_min=width; //0.7*width-2; 
  int  width_max=1.3*width+2;
  int  delta=0.5*width;        //critical distance to last orange pixel

    
  //j0>0

  bool done=0;
  int v_cur=i-2;

  while(done==0 && v_cur-i<20){
    currentH=imgR[0].ptr<uchar>(v_cur); //pointer to i-th row of H-channel
    currentS=imgR[1].ptr<uchar>(v_cur); //pointer to i-th row of S-channel
    currentV=imgR[2].ptr<uchar>(v_cur); //pointer to i-th row of V-channel
    output2R=out2R.ptr<uchar>(v_cur);            //pointer to i-th row of third out-channel
    output1R=out1R.ptr<uchar>(v_cur);            //pointer to i-th row of third out-channel
    output0R=out0R.ptr<uchar>(v_cur);            //pointer to i-th row of third out-channel

    v_cur++;


    int beg_r=-3; // -3 means, that the current pixel could be a inner pixel of the cone
    int beg_l=-2;
    int end_r=-2;
    int end_l=-2;
    int j_r=j0;
    int j_l=j0;



  
    for (int j_rel=0;j_rel<tol;j_rel++) {
      j_r=j0+j_rel;

      *(output0R+j_r)+=100;  //PLOT

      //right
    
      if( isOrange(currentH[j_r],currentS[j_r],currentV[j_r]) ){
      
	// either inner point ->  first search for left flank of cone, save as beg_r
	if(beg_r==-3){
	  while(1){
	    //*(output1R+j_l)+=100; //PLOT

	    //std::cout<<"begr "<<beg_r<<" j_l "<<j_l<<std::endl;
	  
	    if( isOrange(currentH[j_l],currentS[j_l],currentV[j_l]) ){
	      beg_r=j_l;
	    }

	    else if(beg_r-j_l > delta){
	      break;
	    }
	    if(j0-j_l>tol || j_l==1){
	      beg_r=-2; //
	      break;
	    }
	    j_l--;
	  }
	}

      
	//or  new cone
	if(beg_r==-1){
	  beg_r=j_r;
	  end_r=j_r;
	}

	//or  pixel belongs to current cone
	else{
	  end_r=j_r;
	}
	//*(output1R+j_r)=100;                                                  //PLOT all orange Pixels in ROI in Channel 2R
      }

      //........or pixel is not orange...............................................................
      else {
      


	// std::cout<<"j0 "<<j0<<" jr "<<j_r<<" begr "<<beg_r<<" endr "<<end_r<<std::endl;
	// *(output1R+j_r)=100;    
	//******* critical distance to last orange pixel  => cone finished **************************
	if (j_r-end_r==delta && end_r>0){



	  //width>width_max => probably hidden
	  if(end_r-beg_r>width_max){
	    done=1;
	    std::cout<<"R: width>width_max => probably hidden"<<std::endl;
	    break;
	  }
	  //::::::::::::: or suitable width of detected cone => VALID CONE DETECTED:::::::::::::::
	  else if(end_r-beg_r>=width_min && end_r-beg_r<=width_max){ 

	    //   for (int z=beg_r; z<=end_r; z++){*(output1R+z)+=150;}


	    //std::cout<<"RESULT:"<<beg_r<<", "<<end_r<<std::endl;
	    for (int z=beg_r; z<=end_r; z++){*(output0R+z)=255;*(output1R+z)=255;}//*(output2R+z)=255;}
	    done=1;
	    EDlR.start=beg_r;
	    EDrR.start=end_r;
	    valid_right_cone=1;
	    break;
	  
	  }
      
	  beg_r=-1;
	  end_r=-1;
	}

	if(beg_r==-3 || beg_r==-2){
	  beg_r=-1;
	  end_r=-1;
	}
      }








      //left
      if (j_l>j0-j_rel){
	j_l=j0-j_rel;
	*(output1R+j_l)+=100;  //PLOT

      
	if( isOrange(currentH[j_l],currentS[j_l],currentV[j_l]) ){
            
	  //either new cone
	  if(beg_l==-1){
	    beg_l=j_l;
	    end_l=j_l;
	  }

	  //or  pixel belongs to current cone
	  else{
	    beg_l=j_l;
	  }
	  //*(output1R+j_l)=100;                                                  //PLOT all orange Pixels in ROI in Channel 2R
	}

	//........or pixel is not orange...............................................................
	else {
       
	  //******* critical distance to last orange pixel  => cone finished **************************
	  if (beg_l-j_l==delta && beg_l>0){

	    // std::cout<<" endl "<<end_l<<" begl "<<beg_l<<"breite "<<end_l-beg_l<<" min "<< width_min<<" max "<<width_max<<std::endl;


	    //width>width_max => probably hidden
	    if(end_l-beg_l>width_max){
	      done=1;
	      std::cout<<"R: width>width_max => probably hidden"<<std::endl;
	      break;
	    }
	    //::::::::::::: or suitable width of detected cone => VALID CONE DETECTED:::::::::::::::

	    else if(end_l-beg_l>=width_min && end_l-beg_l<=width_max){ 

	      //   for (int z=beg_l; z<=end_l; z++){*(output1R+z)+=150;}


	      //std::cout<<"RESULT:"<<beg_l<<", "<<end_l<<std::endl;
	      for (int z=beg_l; z<=end_l; z++){*(output0R+z)=255;*(output1R+z)=255;}//*(output2R+z)=255;}
	      done=1;
	      EDlR.start=beg_l;
	      EDrR.start=end_l;
	      valid_right_cone=1;
	      break;
	  
	    }
      
	    beg_l=-1;
	    end_l=-1;
	  }

	  if( beg_l==-2){
	    beg_l=-1;
	    end_l=-1;
	  }
	}

      }



    }
  }
}




#include <iostream>
#include <fstream>
#include <array>  // needs  g++ segments.cpp -std=c++11
#include <vector>

int main (){

  std::ofstream outfile1;
  outfile1.open("auto_segments.svg"); // svg-NAME
  
  std::ifstream infile_beg;
  infile_beg.open("seg_beg");
  std::string prototype_line;
  while(std::getline(infile_beg,prototype_line)){ outfile1<<prototype_line<<"\n";  }    //zeilenweise lesen und in outfile1 schreiben 
  infile_beg.close();

  //====== Punkte definieren===============================
  std::array<  std::array<  std::vector<  std::array<int,2>> ,3>   ,6>  P;
  // P [Hauptpunkt-ID] [0(Punkte davor) oder 1(Hauptpunkt) oder 2(Punkte danach)] [Punkt-Index (bei Hauptpunkt nur ein Eintag =>muss Index 0)] [0 fuer x oder 1 fuer y-Koordinate]
  
  P[1][0]={{{2750,3900},{1450,2200}}};//Punkte, die immer vor Hauptpunkt mit ID 1 liegen muessen
  P[1][1]={{{3800,200}}};             //Hauptpunkt mit ID 1
  P[1][2]={{{5500,700}}};             //Punkte, die immer nach Hauptpunkt mit ID 1 liegen muessen

  P[2][0]={{{5300,1200}}};
  P[2][1]={{{3800,750}}};
  P[2][2]={{{2300,1100},{2000,2100},{2300,3100},{3800,3400}}};

  P[3][0]={{{7400,5000}}};
  P[3][1]={{{9000,5550}}};
  P[3][2]={{{11000,4000},{10200,2500}}};
  
  P[4][0]={{{9500,1800},{11450,3300}}};
  P[4][1]={{{8500,6050}}};
  P[4][2]={{{7000,5500}}};

  P[5][0]={{{10100,1300}}};
  P[5][1]={{{11800,1000}}};
  P[5][2]={{}};
  
  //Uebergangspunkte
  P[0][0]={{{7000,2300}}}; //fuer 1->4 &  1->5 & 3->2
  P[0][1]={{{6200,4200}}}; //fuer 2->3 & 4->1
  P[0][2]={{{11000,3650}}}; //fuer 3->5 (dann Punkte nach 3 weglassen)

  //==========================================================
  

  std::string ink_nodetypes="ca";
  int l=-1;
  std::array<std::string,5> col ={"","rgb(255,0,0)","rgb(255,255,0)","rgb(100,255,255)","rgb(0,0,255)"};

  
  //Punkte schreiben
  outfile1<<"\t\t<g id=\"l_pts_info\"  inkscape:groupmode=\"layer\" inkscape:label=\"pts_info\"> \n"
	  <<"\t\t\t <polyline points=\" ";
  for(int i=1;i<=4;i++){
    for(int j=0;j<=5;j++){
      if(P[i][j].empty()==0){
	for(int k=0;k<P[i][j].size();k++){
	  if(P[i][j][k].empty()==0){
	    outfile1<<P[i][j][k][0]<<","<<P[i][j][k][1]<<" ";
	  }}}}}
  outfile1<<"\" style=\"fill:none\" stroke-opacity=\"0\" stroke=\"black\" stroke-width=\"50\" marker-mid=\"url(#M_circ)\" marker-start=\"url(#M_circ)\" marker-end=\"url(#M_circ)\" /> \n \t\t \n </g>\n"<<std::endl;

  
  //Segmente schreiben
  
  for(int i=1; i<=4; i++){
    
    //Ueberebene  l_segiX
    outfile1<<"\t\t<g id=\"segment"<<i<<"X\" inkscape:groupmode=\"layer\" inkscape:label=\"segment"<<i<<"X\">"<<std::endl;
    
    for(int j=1;j<=5;j++){
      
      if( (i==1 && j==2) || (i==2 && j==1) || (i==3 && j==4) || (i==4 && j==3) ){;}      // keine Segmente 12, 21, 34, 43
      else{
	//Ebene l_segij
	outfile1<<"\t\t\t<g id=\"segment"<<i<<j<<"\" inkscape:groupmode=\"layer\" inkscape:label=\"segment"<<i<<j<<"\"> \n"
        //Pfad segij mit Pfad-Effekt Spiro-Spline
		<<"\t\t\t\t <path style=\"fill:none\" stroke=\""<<col[i]<<"\" stroke-opacity=\"1\" stroke-width=\"10\" marker-mid=\"url(#M_dot)\"  \n"  
		<<"\t\t\t\t\t d=\"\" id=\"seg"<<i<<j<<"\" inkscape:path-effect=\"#path-effect1\" \n" << "\t\t\t\t\t inkscape:original-d=\"  "
	//Startpunkt (Hilfspunkt --Gerade--> Startpunkt -> erster Kontrollpunkt )
	             <<"  M  " << P[i][1][0][0]-(2*(i%2)-1)*1000 <<","<<P[i][1][0][1]<<"    "<<P[i][1][0][0]<<","<<P[i][1][0][1]<< "  C  " << P[i][1][0][0]+(2*(i%2)-1)*1000<<","<<P[i][1][0][1];
	    ink_nodetypes="cs";
	    
        //ggf. Punkte nach Startpunkt (Kontrollpunkt -> Punkt -> Kontrollpunkt)
	if(P[i][2].empty()==0 && !(i==3 && j==5)){
	  for(int k=0;k<P[i][2].size();k++){
	    outfile1<<"     " << P[i][2][k][0]-1000             <<","<<P[i][2][k][1]<<"    "<<P[i][2][k][0]<<","<<P[i][2][k][1]<< "     " << P[i][2][k][0]+1000<<","<<P[i][2][k][1];
	    ink_nodetypes+="s";}}
	
	//ggf. Uebergangspunkte
	if((i==1 && j==4) || (i==1 && j==5) || (i==3 && j==2)) {l=0;}
	else {l=-1;}
	if((i==2 && j==3) || (i==4 && j==1)) {l=1;}
	if( i==3 && j==5 ){l=2;}
	if(l!=-1){
	  for(int k=0;k<P[0][l].size();k++){
	    outfile1<<"     " << P[0][l][k][0]-1000             <<","<<P[0][l][k][1]<<"    "<<P[0][l][k][0]<<","<<P[0][l][k][1]<< "     " << P[0][l][k][0]+1000<<","<<P[0][l][k][1];
	    ink_nodetypes+="s";}}
	
	//ggf. Punkte vor Endpunkt
	if(P[j][0].empty()==0){
	  for(int k=0;k<P[j][0].size();k++){
	    outfile1<<"     " << P[j][0][k][0]-1000             <<","<<P[j][0][k][1]<<"    "<<P[j][0][k][0]<<","<<P[j][0][k][1]<< "     " << P[j][0][k][0]+1000<<","<<P[j][0][k][1];
	    ink_nodetypes+="s";}}
	
	//Endpunkt (Kontrollpunkt -> Endpunkt --Gerade--> Hilfspunkt)
       	outfile1    <<"     " << P[j][1][0][0]-(2*(j%2)-1)*1000 <<","<<P[j][1][0][1]<<"    "<<P[j][1][0][0]<<","<<P[j][1][0][1]<< "  L  " << P[j][1][0][0]+(2*(j%2)-1)*1000<<","<<P[j][1][0][1]
       		    << " \" \n";
	ink_nodetypes+="sc";
	
	outfile1    <<"\t\t\t\t\t  sodipodi:nodetypes=\""<<ink_nodetypes<<"\" /> \n"
	        <<"\t\t\t</g>"<<std::endl;
      }
      }
    outfile1<<"\t\t</g>"<<std::endl;
    }
  
  // ============
  outfile1<<"	</g> </svg>";
  outfile1.close();




  
  return 0;
}

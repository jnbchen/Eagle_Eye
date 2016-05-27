#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <boost/filesystem.hpp>

#include "tinyxml2.h"


/* logtosvg reads log files and produces an svg file, which visualizes 
 * the data point in the specified log files. logtosvg takes a single 
 * log file, a list of log files or a directory of log files as command 
 * line argument. At least one argument must be passed to it.
 * Examples:
 * ./logtosvg data/log.txt
 * ./logtosvg log1.txt log2.txt
 * ./logtosvg ../data/LogFiles/
 * 
 * The svg-file is constructed from scratch and exisitng svg-files won't
 * be overweritten.
 * Every log file is visualized by one polyline, for which dot size, 
 * color and stroke width can be changed in the parameter section, as 
 * well as file locations and other things.
 * The units are mm in the svg coordinate system, but cm in the Inksacpe
 * coordinate system.
 * 
 * All points are on a locked sublayer called 'Points'.
 * Bezier Curves should be drawn on the sublayer called 'Curves' and 
 * should get their correct name as an identifier!
 */



// function declarations
std::vector<std::string> split_string(const std::string&, 
                                      const std::string&);


int main(int argc, char** argv) {
    if (argc == 1) {
        throw std::runtime_error("logtosvg needs at least one log file "
                                 "as argument");
    }
    
    // parameters
    std::string output_svg = "../path.svg";
    
    const std::string log_delimiter = " ";
    
    const std::string xmlns = "http://www.w3.org/2000/svg";
    const int width = 12000;
    const int height = 6500;
    const int downscale = 10;
    
    const int dot_radius = 1;
    const std::string dot_color = "black";
    const std::string marker_id = "M_dot";
    
    const std::string line_style = "fill:none";
    const std::string line_color = "grey";
    const int line_width = 5;
    
    // check if output file already exist, if so add number
    boost::filesystem::path out(output_svg);
    std::string filepath = out.parent_path().string() + 
                           "/" + out.stem().string();
    std::string ext = out.extension().string();
    for (int i = 2; boost::filesystem::exists(out); ++i) {
        std::stringstream temp;
        temp << filepath << "_" << i << ext;
        output_svg = temp.str();
        out = boost::filesystem::path(output_svg);
        temp.clear();
    }
    std::cout << "Output filename: " << output_svg << std::endl;
    
    // convert arguments of main to list of files
    boost::filesystem::path p(argv[1]);
    std::vector<std::string> file_list;
    
    if (boost::filesystem::is_directory(p)) {
        // argument is directory
        for (boost::filesystem::directory_iterator i = 
             boost::filesystem::directory_iterator(p); 
             i != boost::filesystem::directory_iterator(); ++i) {
            // eliminate sub-directories
            if (!boost::filesystem::is_directory(i->path()))
                file_list.push_back(i->path().string());
            else
                continue;
        }
    }
    else {
        // argument is a file or list of files      
        for (int i = 1; i < argc; ++i) {
            file_list.push_back(argv[i]);
        }
    }
    
    // open empty svg-file, write standard declaration
    tinyxml2::XMLDocument doc;
    doc.InsertEndChild(doc.NewDeclaration());
    
    // create root
    tinyxml2::XMLElement* root = doc.NewElement("svg");
    root->SetAttribute("xmlns", xmlns.c_str());
    root->SetAttribute("width", (double)width/downscale);
    root->SetAttribute("height", (double)height/downscale);
    doc.InsertEndChild(root);
    
    // create marker definition
    tinyxml2::XMLElement* defs = doc.NewElement("defs");
    root->InsertEndChild(defs);
    tinyxml2::XMLElement* marker = doc.NewElement("marker");
    marker->SetAttribute("id", marker_id.c_str());
    marker->SetAttribute("markerWidth", 2*dot_radius);
    marker->SetAttribute("markerHeight", 2*dot_radius);
    marker->SetAttribute("refX", dot_radius);
    marker->SetAttribute("refY", dot_radius);
    defs->InsertEndChild(marker);
    tinyxml2::XMLElement* circle = doc.NewElement("circle");
    circle->SetAttribute("cx", dot_radius);
    circle->SetAttribute("cy", dot_radius);
    circle->SetAttribute("r", dot_radius);
    circle->SetAttribute("fill", dot_color.c_str());
    marker->InsertEndChild(circle);
    
    // build marker link
    std::stringstream marker_link_stream;
    marker_link_stream << "url(#" << marker_id << ")";
    std::string marker_link = marker_link_stream.str();
    
    // build coordinate transformation
    std::stringstream trafo_stream;
    trafo_stream << "translate(0," << (double)height/downscale 
                 << ") scale(" << (double)1/downscale << "," 
                 << (double)-1/downscale << ")";
    std::string coord_trafo = trafo_stream.str();
    
    // root layer, with coordinate transformation
    tinyxml2::XMLElement* root_layer = doc.NewElement("g");
    root_layer->SetAttribute("transform", coord_trafo.c_str());
    root_layer->SetAttribute("inkscape:label", "Root");
    root_layer->SetAttribute("inkscape:groupmode", "layer");
    root_layer->SetAttribute("id", "layer1");
    root->InsertEndChild(root_layer);
    
    // curve layer
    tinyxml2::XMLElement* curve_layer = doc.NewElement("g");
    curve_layer->SetAttribute("inkscape:label", "Curves");
    curve_layer->SetAttribute("inkscape:groupmode", "layer");
    curve_layer->SetAttribute("id", "layer2");
    root_layer->InsertEndChild(curve_layer);
    
    // point layer
    tinyxml2::XMLElement* point_layer = doc.NewElement("g");
    point_layer->SetAttribute("inkscape:label", "Points");
    point_layer->SetAttribute("inkscape:groupmode", "layer");
    point_layer->SetAttribute("id", "layer3");
    point_layer->SetAttribute("sodipodi:insensitive", true);
    root_layer->InsertEndChild(point_layer);
    
    
    /*
    // find "Points" layer
    tinyxml2::XMLElement *node = 
        doc.FirstChildElement("svg")->FirstChildElement("g");
    while (!node->Attribute("inkscape:label", "Points")) {
        node = node->NextSiblingElement("g");
    }
    */
    
    
    // loop over all log-files
    std::cout << "Processing:" << std::endl;
    for (size_t i = 0; i < file_list.size(); ++i) {
        std::string log_file = file_list[i];
        std::cout << log_file << std::endl;
        
        // open log-file and discard first line
        std::string line;
        std::ifstream log(log_file.c_str());
        std::getline(log, line);
        
        // check if log file could be opened
        if (!log.is_open()) {
            std::cout << "WARNING! Could not open log file: " 
                      << log_file << std::endl;
        }
        
        // loop over all lines of the log file, build points string
        // (beginning from second line)
        std::stringstream points_stream;
        while (std::getline(log, line)) {
            
            // get x, y positions of point
            std::vector<std::string> line_content = 
                split_string(line, log_delimiter);

            points_stream << line_content[0] << "," 
                          << line_content[1] << " ";
        }
        std::string points = points_stream.str();
        
        // create polyline node
        tinyxml2::XMLElement* polyline = doc.NewElement("polyline");
        polyline->SetAttribute("style", line_style.c_str());
        polyline->SetAttribute("stroke" , line_color.c_str());
        polyline->SetAttribute("stroke-width", line_width);
        polyline->SetAttribute("marker-start", marker_link.c_str());
        polyline->SetAttribute("marker-mid", marker_link.c_str());
        polyline->SetAttribute("marker-end", marker_link.c_str());
        polyline->SetAttribute("points", points.c_str());
        point_layer->InsertEndChild(polyline);
        
    }
    
    doc.SaveFile(output_svg.c_str());
    std::cout << "Done" << std::endl;
}



// split a string at the characters defined by delimiter
std::vector<std::string> split_string(const std::string& str,
                                      const std::string& delimiter) {
    std::vector<std::string> strings;
 
    std::string::size_type pos = 0;
    std::string::size_type prev = 0;
    while ((pos = str.find(delimiter, prev)) != std::string::npos) {
        strings.push_back(str.substr(prev, pos - prev));
        prev = pos + delimiter.size();
    }
 
    // To get the last substring (or only, if delimiter is not found)
    strings.push_back(str.substr(prev));
 
    return strings;
}

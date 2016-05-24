#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <boost/filesystem.hpp>

#include "tinyxml2.h"


/* logtosvg reads log files and produces an svg file, which contains 
 * circles for every data point in the specified log files.
 * logtosvg takes a single log file, a list of log files or a directory 
 * of log files as command line argument. At least one argument must be 
 * passed to it.
 * Example:
 * ./logtosvg data/log.txt
 * ./logtosvg log1.txt log2.txt
 * ./logtosvg ../data/LogFiles/
 * 
 * The circles are added to an empty svg file at the correct position, 
 * which is then saved.
 * Radius, color, file locations and other things can be changed in the 
 * parameter section.
 * The units of the svg-file are mm.
 * 
 * example circle node in svg file:
 * <circle
 *     id="path0000"
 *     style="fill:#000000;stroke:none"
 *     cx="0.0"
 *     cy="0.0"
 *     r="1.0" />
 * 
 * So far the points in the svg file are displayed up side down!
 * All points are on layer 'Points'.
 */



// function declarations
std::vector<std::string> split_string(const std::string&, 
                                      const std::string&);


int main(int argc, char** argv) {
    if (argc == 1) {
        throw std::runtime_error("Main needs at least one log file "
                                 "as argument");
    }
    
    // parameters
    const std::string empty_svg = "empty.svg";
    const std::string output_svg = "../path.svg";
    
    const std::string log_delimiter = " ";
    
    const std::string width = "12000mm";
    const std::string height = "7000mm";
    const std::string view_box = "0 0 12000 7000";
    
    const std::string radius = "10";
    const std::string color = "#000000";  // black
    
    const std::string id_tag = "path";
    const int id_start = 1111;
    const int id_increment = 2;
    
    
    // convert arguments of main to list of files
    boost::filesystem::path p(argv[1]);
    std::vector<std::string> file_list;
    
    if (boost::filesystem::is_directory(p)) {
        // argument is directory
        for (auto i = boost::filesystem::directory_iterator(p); 
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
    
    // read empty svg-file
    tinyxml2::XMLDocument doc;
    doc.LoadFile(empty_svg.c_str());
    
    // set width, length and viewbox
    doc.FirstChildElement("svg")->SetAttribute("width", width.c_str());
    doc.FirstChildElement("svg")->
        SetAttribute("height", height.c_str());
    doc.FirstChildElement("svg")->
        SetAttribute("viewBox", view_box.c_str());
    
    // find "Points" layer
    tinyxml2::XMLElement *node = 
        doc.FirstChildElement("svg")->FirstChildElement("g");
    while (!node->Attribute("inkscape:label", "Points")) {
        node = node->NextSiblingElement("g");
    }
    
    // build style attribute
    std::stringstream style_stream;
    style_stream << "fill:" << color << ";stroke:none";
    std::string style = style_stream.str();
    
    
    // loop over all log-files
    std::cout << "Processing:" << std::endl;
    int counter = 0;
    for (auto &log_file : file_list) {
        std::cout << log_file << std::endl;
        
        // open log-file and discard first line
        std::string line;
        std::ifstream log(log_file);
        std::getline(log, line);
        
        // check if log file could be opened
        if (!log.is_open()) {
            std::cout << "WARNING: Could not open log file: " 
                      << log_file << std::endl;
        }
        
        // loop over all lines of the log file
        // (beginning from second line)
        while (std::getline(log, line)) {
            
            // get x, y positions of point
            std::vector<std::string> line_content = 
                split_string(line, log_delimiter);

            std::stringstream x_stream;
            x_stream << line_content[0];
            std::string x = x_stream.str();
            
            std::stringstream y_stream;
            y_stream << line_content[1];
            std::string y = y_stream.str();
            
            // build id attribute
            std::stringstream id_stream;
            id_stream << id_tag << id_start + id_increment * counter;
            std::string id = id_stream.str();
            
            // create new circle node and attach attributes
            tinyxml2::XMLElement *circle = doc.NewElement("circle");
            circle->SetAttribute("id", id.c_str());
            circle->SetAttribute("style", style.c_str());
            circle->SetAttribute("cx", x.c_str());
            circle->SetAttribute("cy", y.c_str());
            circle->SetAttribute("r", radius.c_str());
            
            // insert circle node at bottom of g-node
            node->InsertEndChild(circle);
            
            counter++;
        }
        
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

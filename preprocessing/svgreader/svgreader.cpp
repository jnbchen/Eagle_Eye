#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <boost/filesystem.hpp>

#include "tinyxml2.h"


/* svgreader scans svg files for paths which are labled as segments and
 * then writes them to a text file. The svg input file must be passed as
 * commandline argument to svgreader.
 * Example:
 * ./svgreader ../path.svg
 * 
 * Every bezier curve in the svg is written to the ouput on a single 
 * line, containing start, end and control points. Segments are grouped 
 * together with a header, where the name is read from the path id. 
 * The order of the curves is preserved. This output file can then be 
 * read by the TrajectoryGenerator.
 * 
 * Formatting of the output text file:
 * # segment XY
 * start_x,start_y c1_x,c1_y c2_x,c2_y, end_x,end_y
 * ...
 * 
 * The robustness and error checking capabilities are limited!
 * svgreader assumes a well defined svg-file, such is produced by 
 * logtosvg with curves drawn the right way!
 */



// function declarations
std::vector<std::string> split_string(const std::string&, 
                                      const std::string&);


int main(int argc, char** argv) {
    if (argc != 2) {
        throw std::runtime_error("Input svg is missing or to many "
                                 "arguments");
    }    
    
    // default output path
    std::string output_path = "../test_segments.txt";
    
    // check if output file already exist, if so add number
    boost::filesystem::path p(output_path);
    std::string filepath = p.parent_path().string() + 
                           "/" + p.stem().string();
    std::string ext = p.extension().string();
    for (int i = 2; boost::filesystem::exists(p); ++i) {
        std::stringstream temp;
        temp << filepath << "_" << i << ext;
        output_path = temp.str();
        p = boost::filesystem::path(output_path);
        temp.clear();
    }
    std::cout << "Output filename: " << output_path << std::endl;
    
    // open svg file
	tinyxml2::XMLDocument doc;
	doc.LoadFile(argv[1]);
    
    // find curve layer in svg
    tinyxml2::XMLElement *curve_layer = 
        doc.FirstChildElement("svg")->FirstChildElement("g")
                                    ->FirstChildElement("g");
    while (!curve_layer->Attribute("inkscape:label", "Curves")) {
        curve_layer = curve_layer->NextSiblingElement("g");
    }
    
    // open output file
    std::ofstream fout(output_path.c_str());
    if (fout.fail()) {
        throw std::runtime_error("Could not open output file");
    }
    else {
        // loop over all curves, check if they are segments
        const std::string seg = "segment";
        for (tinyxml2::XMLElement* path = 
                 curve_layer->FirstChildElement("path"); 
             path; path = path->NextSiblingElement("path")) 
        {
            std::string id = path->Attribute("id");
            if (id.substr(0, 7) == "segment" && id.size() == 9) {
                // found bezier curve
                fout << "# segment " << id.substr(7, 9) << std::endl;
                
                std::string complete_str = path->Attribute("d");
                std::vector<std::string> s = 
                    split_string(complete_str, " ");
                
                // check for absolute coordinates
                if (s[0] == "m" || 
                    s[2] == "c" || s[3] == "c" || 
                    s[s.size() - 2] == "l") {
                    throw std::runtime_error("svgreader needs svg files" 
                                        " with absolute coordinates");
                }
                
                // check for smooth nodes, ignore end nodes
                std::string nodes = path->Attribute("sodipodi:nodetypes");
                nodes = nodes.substr(1, nodes.size() - 2);
                
                if (nodes.empty() || 
                    nodes.find_first_not_of("s") != std::string::npos) {
                    std::cout << "WARNING! Not all of the path nodes "
                                 "are smooth nodes, " << id << std::endl;
                }
                
                // process looooooong string ...
                
                // get starting point of segment
                std::string start;
                int start_index;
                if (s[0] == "M" && s[2] == "C") {
                    start = s[1];
                    start_index = 3;
                }
                else if (s[0] == "M" && s[3] == "C") {
                    start = s[2];
                    start_index = 4;
                }
                else {
                    throw std::runtime_error("somehting wrong with "
                                             "beginning of path");
                }
                
                // end point of segment
                int end_index;
                if (s[s.size() - 2] == "L")
                    end_index = s.size() - 3;
                else
                    end_index = s.size() - 1;
                
                if ((end_index - start_index + 1) % 3 != 0) {
                    throw std::runtime_error("path has not correct "
                                             "number of points");
                }
                
                // print points to output
                for (int i = start_index; i <= end_index; i+=3) {
                    fout << start << " ";
                    fout << s[i] << " " << s[i+1] << " " << s[i+2];
                    fout << std::endl;
                    start = s[i+2];
                }
                
                // new line for next segment
                fout << std::endl;
                
            }
            else {
                // this is no bezier curve
                continue;
            }
        }
        
        fout.close();
    }
    
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

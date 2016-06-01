#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "sys/stat.h"
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
 * svgreader assumes a well defined svg-file with curves drawn the 
 * right way!
 */



// function declarations
std::vector<std::string> split_string(const std::string&, 
                                      const std::string&);


int main() {
    // default paths
    std::string input_path = "../seg.svg";
    std::string output_path = "../seg.txt";
    
    // ==============================================================
    // check if output file already exist, if so ask user
    // http://stackoverflow.com/questions/12774207/
    // fastest-way-to-check-if-a-file-exist-using-standard-c-c11-c
    struct stat buffer;
    while (stat(output_path.c_str(), &buffer) == 0) {
        std::string decision;
        std::cout << output_path << " already exists" << std::endl;
        std::cout << "Overwrite? (y/n) ";
        std::cin >> decision;
        if (decision == "y") {
            // keep filename
            break;
        }
        else {
            // new filename
            std::cout << "New path and filename: ";
            std::cin >> output_path;
        }
    }
    std::cout << "Output filename: " << output_path << std::endl;
    
    // ==============================================================
    // open svg file
	tinyxml2::XMLDocument doc;
	doc.LoadFile(input_path.c_str());
    
    // find segments layer in svg
    tinyxml2::XMLElement *seg_layer = 
        doc.FirstChildElement("svg")->FirstChildElement("g");
    while (!seg_layer->Attribute("id", "segments") &&
           seg_layer->Attribute("transform") == NULL) {
        seg_layer = seg_layer->NextSiblingElement("g");
    }    
    
    // open output file
    std::ofstream fout(output_path.c_str());
    if (fout.fail()) {
        throw std::runtime_error("Could not open output file");
    }
    
    // lists for storing segment ids and pointer to segment nodes
    std::vector<std::string> segment_ids;
    std::vector<tinyxml2::XMLElement*> segment_nodes;
    
    // ==============================================================
    // loop over all sublayer of segments layer, 
    // containing all segments for one starting point
    for (tinyxml2::XMLElement* superseg = 
             seg_layer->FirstChildElement("g"); 
         superseg; superseg = superseg->NextSiblingElement("g")) 
    {
        std::string superseg_id = superseg->Attribute("id");
        if (superseg_id.substr(0, 7) == "segment" &&
            superseg_id.at(8) == 'X' &&
            superseg_id.size() == 9)
        {
            
            // no additional scaling and translation of coordinates
            if (superseg->Attribute("transform") != NULL) {
                std::cout << "Error in layer " 
                          << superseg_id << std::endl;
                throw std::runtime_error("One layer with segments "
                                         "contains transformed "
                                         "coordinates");
            }
            
            
            // loop over all segments of one starting point
            for (tinyxml2::XMLElement* seg = 
                    superseg->FirstChildElement("g"); 
                 seg; seg = seg->NextSiblingElement("g")) 
            {
                std::string seg_id = seg->Attribute("id");
                if (seg_id.substr(0, 7) == "segment" &&
                    seg_id.size() == 9)
                {
                    // layer contains a segment
                    tinyxml2::XMLElement* path = 
                        seg->FirstChildElement("path");
                    
                    // store pointer to node and segment id
                    segment_ids.push_back(seg_id.substr(7, 9));
                    segment_nodes.push_back(path);                        
                }
            }
        }
        else
            continue;
    }
    
    // sanity check, as many sgement ids as paths
    if (segment_ids.size() != segment_nodes.size()) {
        throw std::runtime_error("Could not read segment/s or id/s");
    }
    
    // ==============================================================
    // go through list of paths,
    // check every path for validity and write them to the output
    for (size_t i = 0; i < segment_ids.size(); ++i) {
        fout << "# segment " << segment_ids[i] << std::endl;
        
        tinyxml2::XMLElement *path = segment_nodes[i];
        
        std::string complete_str = path->Attribute("d");
        std::vector<std::string> s = 
            split_string(complete_str, " ");
        
        // no additional scaling and translation of coordinates
        if (path->Attribute("transform") != NULL) {
            std::cout << "Error in segment " 
                      << segment_ids[i] << std::endl;
            throw std::runtime_error("One segment contains "
                                     "transformed coordinates");
        }
        
        // check for absolute coordinates
        if (s[0] == "m" || 
            s[2] == "c" || s[3] == "c" || 
            s[s.size() - 2] == "l") {
            std::cout << "Error in segment " 
                      << segment_ids[i] << std::endl;
            throw std::runtime_error("svgreader needs svg files" 
                                " with absolute coordinates");
        }
        
        // check for smooth nodes, ignore end nodes
        std::string nodes = path->Attribute("sodipodi:nodetypes");
        nodes = nodes.substr(1, nodes.size() - 2);
        
        if (nodes.empty() || 
            nodes.find_first_not_of("s") != std::string::npos) {
            std::cout << "Error in segment " 
                      << segment_ids[i] << std::endl;
            throw std::runtime_error("WARNING! Not all of the path "
                                     "nodes are smooth nodes");
        }
        
        // ==================================================
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
            std::cout << "Error in segment " 
                      << segment_ids[i] << std::endl;
            throw std::runtime_error("Somehting wrong with "
                                     "beginning of path");
        }
        
        // end point of segment
        int end_index;
        if (s[s.size() - 2] == "L")
            end_index = s.size() - 3;
        else
            end_index = s.size() - 1;
        
        if ((end_index - start_index + 1) % 3 != 0) {
            std::cout << "Error in segment " 
                      << segment_ids[i] << std::endl;
            throw std::runtime_error("Path has not the correct "
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
    
    fout.close();
    
}



// ==============================================================
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

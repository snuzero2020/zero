#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "ros/package.h"
#include "ros/ros.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"

#include "XYToPixel.h"

using namespace std;
using namespace cv;

namespace Zero {
    typedef struct _Tlbr {
        Point tl;
        Point br;
        Point center;
    } Tlbr;

    class StringWithLinenum {
        public:
        string _str;
        int _line;

        StringWithLinenum(string str, int line) {
            _str = str; _line = line;
        }
    };
    
    class Node {
        private:
        double _x = 0;
        double _y = 0;
        int _pixel_x = 0;
        int _pixel_y = 0;

        void _XYToPixel(double x, double y, bool is_kcity) {
            XYToPixel(_pixel_x, _pixel_y, x, y, is_kcity);
        }

        void _PixelToXY(int pixel_x, int pixel_y, bool is_kcity) {
            PixelToXY(_x, _y, pixel_x, pixel_y, is_kcity);
        }

        public:
        double heading;
        int flag;
        int line;
        int index;
        bool selected = false;

        Point2d GetCoordinate() {
            return Point2d(_x, _y);
        }

        Point GetPixelCoordinate() {
            return Point(_pixel_x, _pixel_y);
        }

        void UpdatePixelCoordinate(int pixel_x, int pixel_y, bool is_kcity) {
            _pixel_x = pixel_x; _pixel_y = pixel_y;
            _PixelToXY(_pixel_x, _pixel_y, is_kcity);
        }

        void UpdateCoordinate(double x, double y, bool is_kcity) {
            _x = x; _y = y;
            _XYToPixel(_x, _y, is_kcity);
        }
    };
}

int resolution[2] = {1920, 1080}; // resolution of display, default is 1920x1080
int window_size[2] = {1280, 720}; // default is 16:9 HD

string seperator(80, '-');

atomic<int> zoom(1);
atomic<int> mouse_pos_x, mouse_pos_y;

double default_position[2];
int default_pixel[2];

atomic<bool> dragging(false);
atomic<bool> hovering(false);
Point drag_start, drag, drag_end;
atomic<bool> editing(false);
atomic<bool> highlight_flag(true);
vector<Zero::Node> node_vector_in_roi;

atomic<bool> node_select_mode(false);
atomic<bool> node_select_ready(true);
atomic<bool> show_arrow(false);
Zero::Node node_to_move;
atomic<int> node_to_move_index;

atomic<bool> rbuttondown(false); atomic<bool> lbuttondown(false);

vector<Zero::Node> hovered_node_vector;
vector<Zero::Node> hovered_node_vector_to_draw;
Zero::Node selected_node;

Zero::Tlbr update_tlbr(Point center, Size map_size);
void mouse_callback(int event, int x, int y, int flags, void* param);
void flag_trackbar_callback(int pos, void* param);
void save_as_txt(vector<Zero::Node> node_vector, map<int, string> comment_map, string in_path_string);
void save_as_image(Mat& map, vector<Zero::Node> node_Vector, string out_visual_string);
void draw_heading(Mat& map, vector<Zero::Node> node_vector, Point tl);
void print_nodeinfo(Zero::Node& node);
void info_ui(Mat map, vector<Zero::Node>& node_vector, Point tl);
Mat watermark_mode(Mat& map);
Scalar color_flag(int flag);
void draw_node(Mat& map, Zero::Node& node, Rect view, int zoom, bool inverse = false);
void draw_nodes(Mat& map, vector<Zero::Node>& node_vector, Rect view, int zoom, bool inverse = false);
vector<Zero::Node> filter_nodes(vector<Zero::Node>& node_vector, Rect view);
Mat update_rendering(Mat& map, vector<Zero::Node>& node_vector, Point pos, int zoom, bool include_nodes, bool render = true);

int main(void) {
    int selected_num = 1; // Default is K-City
    cout << "Select the place to edit:\n"\
         << "1. K-City\n"\
         << "2. FMTC\n";
    do { // receive only 1 or 2
        cout << "Enter: ";
        cin >> selected_num;

        if (selected_num != 1 && selected_num != 2) cin.ignore(256, '\n');
        else break;

        if (cin.fail()) {
            cin.clear();
            cin.ignore(256, '\n');
        }
    } while (1);

    cout << seperator << endl;

    stringstream in_path_stream, in_map_stream, out_visual_stream;
    bool is_kcity;

	if (selected_num == 1) { // K-City
		in_path_stream << ros::package::getPath("slam") << "/config/KCity/re_global_path.txt";
		in_map_stream << ros::package::getPath("slam") << "/config/KCity/KCity.png";
    	out_visual_stream << ros::package::getPath("slam")<< "/config/KCity/KCity_global_path_visual.png";
        default_position[0] = 302457.27; default_position[1] = 4123711.98;
        is_kcity = true;
	} else { // FMTC
		in_path_stream << ros::package::getPath("slam") << "/config/FMTC/FMTC_global_path.txt";
        in_map_stream << ros::package::getPath("slam") << "/config/FMTC/FMTC_map.png";
        out_visual_stream << ros::package::getPath("slam")<< "/config/FMTC/FMTC_global_path_visual.png";
        default_position[0] = 298551.982; default_position[1] = 4137857.464;
        is_kcity = false;
	}

    XYToPixel(default_pixel[0], default_pixel[1], default_position[0], default_position[1], is_kcity);
    // load a map
    Mat map = imread(in_map_stream.str(), IMREAD_COLOR);
    if (map.empty()) {
        cerr << "The map file do not exist\n";
        return -1;
    }

    cout << "path file: " << in_path_stream.str() << "\n";
    cout << "map file: " << in_map_stream.str() << endl;

    // load paths
    fstream node_vector_fstream(in_path_stream.str());
    string node_vector_line;
    vector<Zero::Node> node_vector;
    std::map<int, string> comment_map;
    int max_flag = 1;

    int getnode_line_count = 0, getnode_index_count = 0;
    while (getline(node_vector_fstream, node_vector_line)) {
        getnode_line_count++;

        if (node_vector_line.substr(0, 2) == "//") { // comment
            comment_map.insert(pair<int, string>(getnode_line_count, node_vector_line));
            continue; 
        }

        stringstream node_vector_linestream(node_vector_line);
        Zero::Node node;
        double x, y;
        node_vector_linestream >> x >> y >> node.heading >> node.flag;
        node.UpdateCoordinate(x, y, is_kcity);
        if (node.flag > max_flag) max_flag = node.flag; // culculate max flag

        int pixel_node_x, pixel_node_y;
        XYToPixel(pixel_node_x, pixel_node_y, node.GetPixelCoordinate().x, node.GetPixelCoordinate().y, is_kcity);
        
        node.line = getnode_line_count;
        node.index = getnode_index_count++;

        node_vector.push_back(node);
    }

    node_vector_fstream.close();
    cout << "Info: Size of the map: " << map.size() << endl;
    cout << "Info: " << node_vector.size() << " nodes is loaded (max flag: " << max_flag << ")\a" << endl;
    

    if (map.cols < window_size[0] || map.rows < window_size[1]) {
        cerr << "Error: the map size is smaller than that of the window\a\a" << endl;
        return -2;
    }

    // set ROI
    Point center(default_pixel[0], default_pixel[1]);
    Zero::Tlbr temp_tlbr = update_tlbr(center, map.size());
    Point tl = temp_tlbr.tl, br = temp_tlbr.br;
    
    int flag = 5;
    Mat rendering_image_in_roi = update_rendering(map, node_vector, center, zoom, false); // receive a image to render
    Mat map_for_callback = update_rendering(map, node_vector, center, zoom, true); // image including nodes

    namedWindow("Global Path Editor for SNUZERO");
    //createTrackbar("Flag", "Global Path Editor for SNUZERO", &flag, max_flag, flag_trackbar_callback, 0);
    cvSetMouseCallback("Global Path Editor for SNUZERO", mouse_callback, &map_for_callback);

    cout << seperator << endl;

    bool modified = false;

    while (1) { // run until enter 'q'
#ifndef CV_VERSION_EPOCH
			int key_entered = cv::waitKeyEx(20);	// OpenCV 3.x
#else
			int key_entered = cv::waitKey(20);		// OpenCV 2.x
#endif
        //if (key_entered != 255 && key_entered != -1) cout << hex << "0x" << key_entered << dec << endl;

        if (drag_end != Point(0, 0)) { // LMouseButtonUp
            center -= (drag_end - drag_start) / zoom;
            temp_tlbr = update_tlbr(center, map.size());
            tl = temp_tlbr.tl; br = temp_tlbr.br; center = temp_tlbr.center;
            
            rendering_image_in_roi = update_rendering(map, node_vector, center, zoom, false); // receive a image to render
            map_for_callback = update_rendering(map, node_vector, center, zoom, true); // image including nodes

            drag_end = Point(0, 0);
        }

        // hover mode
        if (editing && !dragging) {
            hovering = false;
            bool duplicated = false;

            draw_nodes(map_for_callback, node_vector_in_roi, Rect(tl, br), zoom, false);
            
            if (show_arrow && node_select_mode) draw_heading(map_for_callback, node_vector, tl);
            else map_for_callback = update_rendering(map, node_vector, center, zoom, true, true);

            if (node_select_mode) draw_node(map_for_callback, node_vector[node_to_move_index], Rect(tl, br), zoom, true);

            // find nodes hovered
            hovered_node_vector.clear();
            for (auto node : node_vector_in_roi) {
                if (node.GetPixelCoordinate().inside(Rect(tl + Point(mouse_pos_x - 8, mouse_pos_y - 8) / zoom, Size(9, 9)))) {
                    hovered_node_vector.push_back(node);

                    if (hovering) {
                        duplicated = true;
                    } else hovering = true;
                }
            }

            if (lbuttondown) { // not hovering and try to exit select mode
                if (!hovered_node_vector.empty()) { // draw the last selected node
                    draw_nodes(map_for_callback, node_vector_in_roi, Rect(tl, br), zoom, false);
                    
                    node_to_move = hovered_node_vector.back();
                    node_to_move_index = node_to_move.index;
                    //cout << node_to_move.index << endl;
                    draw_node(map_for_callback, node_to_move, Rect(tl, br), zoom, true);

                    print_nodeinfo(node_to_move);
                }

                map_for_callback = update_rendering(map, node_vector, center, zoom, true, true);

                if (hovering) node_select_mode = true;
                else node_select_mode = false;
                lbuttondown = false;
            } else hovered_node_vector_to_draw = hovered_node_vector; // deep copy for rendering the selected nodes

            if (hovering) draw_nodes(map_for_callback, hovered_node_vector_to_draw, Rect(tl, br), zoom, true);

            imshow("Global Path Editor for SNUZERO", watermark_mode(map_for_callback));
            
            //cout << hovering << ", " << duplicated << endl;
        }

        if (show_arrow & node_select_mode) draw_heading(map_for_callback, node_vector, tl);

        //cout << node_to_move.line << endl;

        // draw info UI for the selected nodes
        if (rbuttondown) {
            rbuttondown = false;
            if (hovering) node_select_mode = true;
            map_for_callback = update_rendering(map, node_vector, center, zoom, true, false);
            Zero::Tlbr temp_tlbr = update_tlbr(center, map.size());
            info_ui(map_for_callback, hovered_node_vector_to_draw, temp_tlbr.tl);
            hovered_node_vector_to_draw.clear();
        }

        

        //cout << mouse_pos_x << "," << mouse_pos_y << endl;
        //cout << tl + Point(mouse_pos_x, mouse_pos_y) / zoom << endl;

        switch (key_entered) {
            case 'E': // edit mode
            case 'e':
            case 0x120000 + 'E':
            case 0x100000 + 'e':
                editing = true;
                zoom = 2;

                temp_tlbr = update_tlbr(center, map.size());
                tl = temp_tlbr.tl; br = temp_tlbr.br; center = temp_tlbr.center;

                map_for_callback = update_rendering(map, node_vector, center, zoom, true); // image including nodes
                break;
            case 'F': // view / edit mode + flag
            case 'f':
            case 0x120000 + 'F':
            case 0x100000 + 'f':
                highlight_flag = !(highlight_flag);
                map_for_callback = update_rendering(map, node_vector, center, zoom, true);
                break;
            case 'V': // view mode
            case 'v':
            case 0x120000 + 'V':
            case 0x100000 + 'v':
                editing = false;
                zoom = 1;

                temp_tlbr = update_tlbr(center, map.size());
                tl = temp_tlbr.tl; br = temp_tlbr.br; center = temp_tlbr.center;

                map_for_callback = update_rendering(map, node_vector, center, zoom, true); // image including nodes
                break;
            case 'H': // view heading of the selected node
            case 'h':
            case 0x120000 + 'H':
            case 0x100000 + 'h':
                show_arrow = !show_arrow;
                break;
            case 'S': // save the path
            case 's':
            case 0x120000 + 'S':
            case 0x100000 + 's':
                save_as_txt(node_vector, comment_map, in_path_stream.str());
                modified = false;
                break;
            case 'I': // save as image
            case 'i':
            case 0x120000 + 'I':
            case 0x100000 + 'i':
                save_as_image(map, node_vector, out_visual_stream.str());
                break;
            case 'Q': // exit
            case 'q':
            case 0x120000 + 'Q':
            case 0x100000 + 'q':
                if (modified) {
                    cerr << "Error: The modified paths isn't yet saved!" << endl;
                    cerr << "Error: Save it or press 'Ctrl + C' to force to exit" << endl;
                    break;
                }
            case 0x140000 + 'C': // Ctrl + C
            case 0x140000 + 'c':
                if (modified) {
                    cerr << "Warn: The modified paths will be not saved!" << endl;
                }
                return 0;
            case 0x10ff51: // left
                if (node_select_mode) {
                    Zero::Node& node_to_move = node_vector[node_to_move_index];
                    //cout << node_to_move.index << endl;
                    node_to_move.UpdatePixelCoordinate(node_to_move.GetPixelCoordinate().x - 1, node_to_move.GetPixelCoordinate().y, is_kcity);
                    //cout << node_to_move.GetPixelCoordinate() << endl;
                    //cout << node_to_move.GetCoordinate() << endl;

                    map_for_callback = update_rendering(map, node_vector, center, zoom, true);
                    draw_node(map_for_callback, node_to_move, Rect(tl, br), zoom, true);

                    print_nodeinfo(node_to_move);
                    modified = true;
                }
                break;
            case 0x10ff52: // up
                if (node_select_mode) {
                    Zero::Node& node_to_move = node_vector[node_to_move_index];
                    node_to_move.UpdatePixelCoordinate(node_to_move.GetPixelCoordinate().x, node_to_move.GetPixelCoordinate().y - 1, is_kcity);

                    map_for_callback = update_rendering(map, node_vector, center, zoom, true);
                    draw_node(map_for_callback, node_to_move, Rect(tl, br), zoom, true);

                    print_nodeinfo(node_to_move);
                    modified = true;
                }
                break;
            case 0x10ff53: // right
                if (node_select_mode) {
                    Zero::Node& node_to_move = node_vector[node_to_move_index];
                    node_to_move.UpdatePixelCoordinate(node_to_move.GetPixelCoordinate().x + 1, node_to_move.GetPixelCoordinate().y, is_kcity);

                    map_for_callback = update_rendering(map, node_vector, center, zoom, true);
                    draw_node(map_for_callback, node_to_move, Rect(tl, br), zoom, true);

                    print_nodeinfo(node_to_move);
                    modified = true;
                }
                break;
            case 0x10ff54: // down
                if (node_select_mode) {
                    Zero::Node& node_to_move = node_vector[node_to_move_index];
                    node_to_move.UpdatePixelCoordinate(node_to_move.GetPixelCoordinate().x, node_to_move.GetPixelCoordinate().y + 1, is_kcity);

                    map_for_callback = update_rendering(map, node_vector, center, zoom, true);
                    draw_node(map_for_callback, node_to_move, Rect(tl, br), zoom, true);

                    print_nodeinfo(node_to_move);
                    modified = true;
                }
                break;
            case '[': // modify the heading counterclockwise
            case 0x100000 + '[':
                if (node_select_mode && show_arrow) {
                    Zero::Node& node_to_move = node_vector[node_to_move_index];
                    node_to_move.heading += 143.0 / 8192 / 2; // ~ 0.5 deg
                    map_for_callback = update_rendering(map, node_vector, center, zoom, true, false);

                    print_nodeinfo(node_to_move);
                    modified = true;
                }
                break;
            case ']': // modify the heading clockwise
            case 0x100000 + ']':
                if (node_select_mode && show_arrow) {
                    Zero::Node& node_to_move = node_vector[node_to_move_index];
                    node_to_move.heading -= 143.0 / 8192 / 2; // ~ 0.5 deg
                    map_for_callback = update_rendering(map, node_vector, center, zoom, true, false);

                    print_nodeinfo(node_to_move);
                    modified = true;
                }
                break;
        }
    }
}

Zero::Tlbr update_tlbr(Point center, Size map_size) {
    Zero::Tlbr temp;
    temp.center.x = center.x; temp.center.y = center.y;
    temp.tl = temp.center - Point(window_size[0], window_size[1]) / 2 / zoom;
    temp.br = temp.tl + Point(window_size[0], window_size[1]) / zoom;

    if (temp.tl.x < 0) temp.center.x = window_size[0] / 2 / zoom;
    else if (temp.br.x >= map_size.width) temp.center.x = map_size.width - window_size[0] / 2 / zoom;
    if (temp.tl.y < 0) temp.center.y = window_size[1] / 2 / zoom;
    else if (temp.br.y >= map_size.height) temp.center.y = map_size.height - window_size[1] / 2 / zoom;
    
    temp.tl = temp.center - Point(window_size[0], window_size[1]) / 2 / zoom;
    temp.br = temp.tl + Point(window_size[0], window_size[1]) / zoom;
    return temp;
}

void mouse_callback(int event, int x, int y, int flags, void* param) {
    Mat temp_image = *(static_cast<Mat*>(param));
    Mat return_image;
    float move_mat_data[6] = {1, 0, 0, 0, 1, 0};
    Mat move_mat(2, 3, CV_32FC1, move_mat_data);

    switch(event) {
        case CV_EVENT_LBUTTONDOWN:
            lbuttondown = true;
            dragging = true;
            drag = drag_start = Point(x, y);
            break;
        case CV_EVENT_RBUTTONDOWN:
            node_select_ready = false;
            rbuttondown = true;
            break;
        case CV_EVENT_MOUSEMOVE:
            if (dragging && !node_select_mode) { // dragging
                Point dragging_direction = Point(x, y) - drag;
                move_mat_data[2] = dragging_direction.x; move_mat_data[5] = dragging_direction.y;
                warpAffine(temp_image, return_image, move_mat, temp_image.size(), 1, 0, Scalar(255, 255, 0));
                
                imshow("Global Path Editor for SNUZERO", watermark_mode(return_image));
            }

            mouse_pos_x = max(x, 0); // position of mouse
            mouse_pos_y = max(y, 0);
            break;
        case CV_EVENT_LBUTTONUP:
            dragging = false;
            drag_end = Point(x, y);

            if (node_select_mode) drag_end = drag_start;
            break;
        case CV_EVENT_RBUTTONUP:
            node_select_ready = true;
            break;
    }
}

void flag_trackbar_callback(int pos, void* param) {
    return;
}

void draw_heading(Mat& map, vector<Zero::Node> node_vector, Point tl) {
    Zero::Node& node_to_move = node_vector[node_to_move_index];
    Point point1 = (node_to_move.GetPixelCoordinate() - tl) * 2;
    Point point2 = point1 + Point(static_cast<int>(70 * cos(node_to_move.heading)), static_cast<int>(-70 * sin(node_to_move.heading)));
    arrowedLine(map, point1, point2, Scalar(64, 64, 64), 3, 8, 0, 0.25);
}

void save_as_txt(vector<Zero::Node> node_vector, map<int, string> comment_map, string in_path_string) {
    cout << "Saving the paths as text file..." << endl;
    int count = 1;
    ofstream node_vector_ofstream(in_path_string);
    node_vector_ofstream.setf(ios::fixed);
    node_vector_ofstream.precision(6);
    for (auto node : node_vector) {
        //cout << node.line << ": ";

        while (1) {
            if (comment_map.find(count) != comment_map.end()) {
                node_vector_ofstream << comment_map[count] << "\n";
                //cout << "comment: " << count << endl;
                count++;
            } else break;
        }

        if (node.line == count) {
            //cout << "node: " << count << endl;
            node_vector_ofstream << node.GetCoordinate().x << " " << node.GetCoordinate().y << " "
                                 << node.heading << " " << node.flag << "\n";
            
        } else cerr << "Error: Nodes of the path file might be missing!" << endl;
        count++;
    }

    while (1) {
        if (comment_map.find(count) != comment_map.end()) {
            node_vector_ofstream << comment_map[count];
            //cout << "comment: " << count << endl;
            count++;
        } else break;
    }

    node_vector_ofstream.close();
    cout << "The path file has saved at \a" << in_path_string << "\n";
    cout << seperator << endl;
}

void save_as_image(Mat& map, vector<Zero::Node> node_vector, string out_visual_string) {
    cout << "Saving the paths as image file...\a" << endl;
    Mat temp_map = map.clone();

    int node_size = 2;
    for (auto& node : node_vector) {
        Point relative_pos = node.GetPixelCoordinate();
        rectangle(temp_map, Rect(relative_pos - Point(node_size, node_size), Size(1, 1) * (2 * node_size + 1)), Scalar(64, 64, 64));
        
        rectangle(temp_map, Rect(relative_pos - Point(node_size - 1, node_size - 1), Size(1, 1) * (2 * node_size - 2 + 1)), Scalar(0xFF, 0xFF, 0xFF) - color_flag(node.flag), -1);
    }
    
    imwrite(out_visual_string, temp_map);
    cout << "The image file has saved at \a" << out_visual_string << "\n";
    cout << seperator << endl;
}

void print_nodeinfo(Zero::Node& node) {
    cout.setf(ios::fixed);
    cout.precision(3);
    cout << "node No." << node.index << " (flag " << node.flag << ", line " << node.line << ")\n"
         << "\n"
         << "real x: " << node.GetCoordinate().x << " [m], real y: " << node.GetCoordinate().y << " [m]\n"
         << "pixel x: " << node.GetPixelCoordinate().x << " [px], pixel y: " << node.GetPixelCoordinate().y << " [px]\n"
         << "heading: " << node.heading << " [rad] (= " << node.heading * 180 / CV_PI << " [deg]) from (1, 0)\n"
         << seperator << endl;
    cout.unsetf(ios::fixed);
}

void info_ui(Mat map, vector<Zero::Node>& node_vector, Point tl) {
    Point tooltip_position;
    int count = 1;
    // find max
    for (auto& node : node_vector) if (node.GetPixelCoordinate().x > tooltip_position.x) tooltip_position = node.GetPixelCoordinate();

    for (auto& node : node_vector) {
        if (node.GetPixelCoordinate().x > tooltip_position.x) tooltip_position = node.GetPixelCoordinate();
        rectangle(map, Rect((tooltip_position - tl) * zoom + Point(7, -7) + Point(0, 149 * (count - 1)), Size(200, 150)), Scalar(256, 256, 0), 1);
        rectangle(map, Rect((tooltip_position - tl) * zoom + Point(8, -6) + Point(0, 149 * (count - 1)), Size(198, 148)), Scalar(0, 0, 0), -1);

        count++;
    }

    imshow("Global Path Editor for SNUZERO", watermark_mode(map));
}

Scalar color_flag(int flag) {
    // red
    const Scalar RED(0x00, 0x00, 0xFF), ORANGE(0x00, 0x8C, 0xFF), DARKGOLDENROD(0x0B, 0x86, 0xB8), DEEPPINK(0x93, 0x14, 0xFF);
    // green
    const Scalar LIMEGREEN(0x32, 0xCD, 0x32), SPRINGGREEN(0x7F, 0xFF, 0x00), DARKCYAN(0x8B, 0x8B, 0x00);
    // blue
    const Scalar STEELBLUE(0xB4, 0x82, 0x46), SKYBLUE(0xEB, 0xCE, 0x87), CYAN(0xFF, 0xFF, 0x00), BLUE(0xFF, 0x00, 0x00), SLATEBLUE(0xCD, 0x5A, 0x6A);

    switch (flag % 12) {
        case 0: return RED; case 1: return LIMEGREEN; case 2: return STEELBLUE; case 3: return ORANGE;
        case 4: return SPRINGGREEN; case 5: return SKYBLUE; case 6: return DARKGOLDENROD; case 7: return DARKCYAN;
        case 8: return CYAN; case 9: return DEEPPINK;case 10: return BLUE; case 11: return STEELBLUE;
    }
}

void draw_node(Mat& map, Zero::Node& node, Rect view, int zoom, bool inverse /* = false */) {
    vector<Zero::Node> node_vector;
    node_vector.push_back(node);
    
    draw_nodes(map, node_vector, view, zoom, inverse);
}

void draw_nodes(Mat& map, vector<Zero::Node>& node_vector, Rect view, int zoom, bool inverse /* = false */) {
    int node_size = 2 * zoom;
    for (auto& node : node_vector) {
        Point relative_pos = (node.GetPixelCoordinate() - view.tl());
        rectangle(map, Rect(relative_pos * zoom - Point(node_size, node_size), Size(1, 1) * (2 * node_size + 1)), Scalar(128, 128, 0));
        if (highlight_flag) {
            if (inverse) {
                rectangle(map, Rect(relative_pos * zoom - Point(node_size - 1, node_size - 1), Size(1, 1) * (2 * node_size - 2 + 1)), color_flag(node.flag), -1);
            } else {
                rectangle(map, Rect(relative_pos * zoom - Point(node_size - 1, node_size - 1), Size(1, 1) * (2 * node_size - 2 + 1)), Scalar(0xFF, 0xFF, 0xFF) - color_flag(node.flag), -1);
            }
        } else {
            rectangle(map, Rect(relative_pos * zoom - Point(node_size - 1, node_size - 1), Size(1, 1) * (2 * node_size - 2 + 1)), Scalar(128, 128, 128), -1);
        }
    } // #008080

    //imshow("Global Path Editor for SNUZERO", map);
}

vector<Zero::Node> filter_nodes(vector<Zero::Node>& node_vector, Rect view) {
    // filter nodes with ROI
    vector<Zero::Node> node_vector_in_roi;
    for (auto& node : node_vector) {
        if (view.contains(node.GetPixelCoordinate())) {
            node_vector_in_roi.push_back(node);
        }
    }
    return node_vector_in_roi;
}

Mat watermark_mode(Mat& map) {
    Mat temp_map = map.clone();
    if (editing) putText(temp_map, "Edit Mode", Point(5, 30), 2, 1, Scalar(0, 128, 256));
    else putText(temp_map, "View Mode", Point(5, 30), 2, 1, Scalar(0, 128, 256));
    return temp_map;
}

Rect getROI(Point tl, int zoom) {
    if (zoom == 1) {
        return Rect(tl, tl + Point(window_size[0], window_size[1]));
    } else if (zoom == 2) {
        return Rect(tl.x + static_cast<int>(window_size[0] / 4 + 0.5), tl.y + static_cast<int>(window_size[1] / 4 + 0.5), window_size[0], window_size[1]);
    }
}

Mat update_rendering(Mat& map, vector<Zero::Node>& node_vector, Point pos, int zoom, bool include_nodes, bool render) {
    Rect view;
    view.x = pos.x - static_cast<int>(window_size[0] / 2 + 0.5);
    view.y = pos.y - static_cast<int>(window_size[1] / 2 + 0.5);
    view.width = window_size[0]; view.height = window_size[1];
    
    // validate
    if (view.tl().x < 0) view.x = 0;
    if (view.br().x >= map.cols) view.x = map.cols - window_size[0];
    if (view.tl().y < 0) view.y = 0;
    if (view.br().y >= map.rows) view.y = map.rows - window_size[1];

    Mat rendering_image_in_roi, return_map_image_in_roi;
    rendering_image_in_roi = map(view).clone();
    return_map_image_in_roi = rendering_image_in_roi.clone();
    
    if (editing) { // edit mode
        resize(rendering_image_in_roi, rendering_image_in_roi, Size(), 2, 2); // zoom 2x
        rendering_image_in_roi = rendering_image_in_roi(Rect(window_size[0] / 2 + 0.5, window_size[1] / 2 + 0.5, window_size[0], window_size[1]));
        Rect roi_2x(getROI(view.tl(), 2));
        node_vector_in_roi.clear();
        node_vector_in_roi = filter_nodes(node_vector, roi_2x); // transfer node vector in roi to the global
        draw_nodes(rendering_image_in_roi, node_vector_in_roi, roi_2x, 2);
    } else { // view mode
        vector<Zero::Node> node_vector_in_roi = filter_nodes(node_vector, view);
        draw_nodes(rendering_image_in_roi, node_vector_in_roi, view, 1);
    }

    if (render) imshow("Global Path Editor for SNUZERO", watermark_mode(rendering_image_in_roi));
    if (include_nodes) return rendering_image_in_roi;
    else return return_map_image_in_roi;
}
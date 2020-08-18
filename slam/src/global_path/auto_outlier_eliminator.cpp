#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <math.h>
#include <sstream>
#include <string>

#include "ros/package.h"
#include "ros/ros.h"


class Outlier_eliminator{
	private:
		std::string save_path_;
		double threshold_error_, threshold_heading_error_;
		const char delimiter_ = ' ';
		bool is_kcity;

	public:
		int current_flag{0};
		double x_avg_delta{0}, y_avg_delta{0}, heading_avg_delta{0};
		double x_inst_delta{0}, y_inst_delta{0}, heading_inst_delta{0};
		std::stringstream in_path_stream, out_path_stream;
		int cnt;//count how many lines were eliminated

		Outlier_eliminator(){
			threshold_error_ = 0.2;
			threshold_heading_error_ = 5.0;
			ros::param::get("/is_kcity", is_kcity);
			if(!is_kcity){
				in_path_stream << ros::package::getPath("slam") << "/config/FMTC/FMTC_global_path.txt";
				out_path_stream << ros::package::getPath("slam") << "/config/FMTC/FMTC_refined_global_path.txt";
			}
			else{
				in_path_stream << ros::package::getPath("slam") << "/config/KCity/global_path.txt";
                out_path_stream << ros::package::getPath("slam") << "/config/KCity/KCity_refined_global_path.txt";
			}
			eliminate_outliers();
		}

		void eliminate_outliers(){
			std::string in_line;
			std::ifstream in(in_path_stream.str());
			int line_count{0};
			double prev_x, prev_y, prev_heading;

			while(getline(in, in_line)){
				std::stringstream ss(in_line);
				std::string token;
				std::vector<std::string> result;

				while(getline(ss, token, delimiter_)){
					result.push_back(token);
				}

				if(line_count == 0){ 
					current_flag = atoi(result.at(3).c_str());
					prev_x = stod(result.at(0));
					prev_y = stod(result.at(1));
					prev_heading = stod(result.at(2));
					line_count++;
				}
				else{
					if(current_flag == atoi(result.at(3).c_str()) && line_count == 1){
						x_avg_delta = stod(result.at(0)) - prev_x;
						y_avg_delta = stod(result.at(1)) - prev_y;
						heading_avg_delta = stod(result.at(2)) - prev_heading;
						prev_x = stod(result.at(0));
                    	prev_y = stod(result.at(1));
                    	prev_heading = stod(result.at(2));
						line_count++;
					}
					else if(current_flag == atoi(result.at(3).c_str()) && line_count > 1){
						if(abs(x_avg_delta) + threshold_error_ < abs(prev_x-stod(result.at(0))) || abs(y_avg_delta) + threshold_error_ < abs(prev_y-stod(result.at(1))) || abs(heading_avg_delta) + threshold_heading_error_ < abs(prev_heading-stod(result.at(2)))){ 
							in_line.replace(in_line.find(token), token.length(), "");
							cnt++;
						}
						else{
							x_avg_delta = (x_avg_delta*(line_count-1)+(stod(result.at(0))-prev_x))/line_count;
							y_avg_delta = (y_avg_delta*(line_count-1)+(stod(result.at(1))-prev_y))/line_count;
							heading_avg_delta = (heading_avg_delta*(line_count-1)+(stod(result.at(2))-prev_heading))/line_count;
							line_count++;
						}
					}
					else line_count == 0;
				}
			}
			std::ofstream out(out_path_stream.str());

			while(getline(in, in_line)){
				out << in_line;
			}

			out.close();
			in.close();
			printf("complete elimination of outliers\n");
		}
};

int main(int argc, char **argv){
	ros::init(argc, argv, "outlier_eliminator");
	Outlier_eliminator outlier_eliminator;
	ros::spinOnce();
	return 0;
}

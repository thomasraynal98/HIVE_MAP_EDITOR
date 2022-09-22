#include <string.h>
#include <iostream>
#include <vector>
#include <bits/stdc++.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

struct Opencv_area
{
    int top_left_col;
    int top_left_row;
    int down_right_col;
    int down_right_row;

    Opencv_area(int a, int b, int c, int d)
        : top_left_col(a)
        , top_left_row(b)
        , down_right_col(c)
        , down_right_row(d)
        {}
    
    bool is_in(int idx_col, int idx_row)
    {
        if(top_left_col < idx_col && idx_col < down_right_col && top_left_row < idx_row && idx_row < down_right_row) return true;

        return false;
    }

    void fullfill(int a, int b, int c, int d){
        top_left_col = a;
        top_left_row = b;
        down_right_col = c;
        down_right_row = d;
    }

    int option_selection(int idx_col, int idx_row)
    {
        if(top_left_col < idx_col && idx_col < top_left_col+30 && top_left_row < idx_row && idx_row < top_left_row+30) return 1;
        if(top_left_col+30 < idx_col && idx_col < top_left_col+60 && top_left_row < idx_row && idx_row < top_left_row+30) return 2;
        if(top_left_col+60 < idx_col && idx_col < top_left_col+90 && top_left_row < idx_row && idx_row < top_left_row+30) return 6;
        if(top_left_col < idx_col && idx_col < top_left_col+30 && top_left_row+30 < idx_row && idx_row < top_left_row+60) return 3;
        if(top_left_col+30 < idx_col && idx_col < top_left_col+60 && top_left_row+30 < idx_row && idx_row < top_left_row+60) return 4;
        if(top_left_col+60 < idx_col && idx_col < top_left_col+90 && top_left_row+30 < idx_row && idx_row < top_left_row+60) return 5;
    }
};

struct Geographic_point
{
    double longitude, latitude;

    Geographic_point(double a, double b)
        : longitude(a)
        , latitude(b)
        {}
};

struct Data_node
{
    Geographic_point point;
    int node_ID;
    double col_idx, row_idx;

    Data_node(int a, double b, double c)
        : point(b,c)
        , node_ID(a)
        , col_idx(0)
        , row_idx(0)
        {}
};

struct Data_road
{
    int road_ID;
    Data_node* A;
    Data_node* B;
    double deg_to_A, deg_to_B;
    double length;
    bool available;
    double max_speed;

    Data_road(int a, Data_node* b, Data_node* c)
        : road_ID(a)
        , A(b)
        , B(c)
        , deg_to_A(0.1)
        , deg_to_B(0.1)
        {init_data_road();}

    long double toRadians(const long double degree)
    {
        long double one_deg = (M_PI) / 180;
        return (one_deg * degree);
    }

    void init_data_road()
    {
        max_speed = 7.001;
        available = true;
        
        length = fget_angular_distance(&A->point, &B->point);
        deg_to_A = fget_bearing(&B->point, &A->point);
        deg_to_B = fget_bearing(&A->point, &B->point);
    }

    double fget_angular_distance(Geographic_point* pointA, Geographic_point* pointB)
    {
        double lat1  = pointA->latitude;
        double long1 = pointA->longitude;
        double lat2  = pointB->latitude;
        double long2 = pointB->longitude;
        
        double R = 6371000;
        double r1 = lat1 * M_PI / 180;
        double r2 = lat2 * M_PI / 180;
        double dl = (lat2 - lat1) * M_PI/180;
        double dd = (long2 - long1) * M_PI/180;

        double a = sin(dl/2) * sin(dl/2) + cos(r1) * cos(r2) * sin(dd/2) * sin(dd/2);
        double c = 2 * atan2(sqrt(a), sqrt(1-a));
        // return c;
        return R * c;
    }

    double fget_bearing(Geographic_point* pointA, Geographic_point* pointB)
    {
        double lat1  = deg_to_rad(pointA->latitude);
        double long1 = deg_to_rad(pointA->longitude);
        double lat2  = deg_to_rad(pointB->latitude);
        double long2 = deg_to_rad(pointB->longitude);

        double y = sin(long2 - long1) * cos(lat2);
        double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(long2 - long1);
        double o = atan2(y, x);

        if(o*180/M_PI < 0.0)
        {
            return 360 + o*180/M_PI;
        }

        return o*180/M_PI;
    }

    long double deg_to_rad(const long double degree)
    {
        long double one_deg = (M_PI) / 180;
        return (one_deg * degree);
    }
};

struct Path_node{
    int index_node;
    Data_node* n;
    bool closet;
    double fscore;
    double gscore;
    std::vector<Path_node*> connection_data;
    std::vector<double> connection_weight;
    Path_node* come_from; 

    Path_node(int id, Data_node* a)
        : n(a)
        , closet(false)
        , fscore(99999)
        , gscore(99999)
        , index_node(id)
        , come_from(NULL)
        {}
};

struct Robot_position
{
    Geographic_point* point;
    double g_longitude, g_latitude, g_hdg;
    double l_x, l_y, l_hdg;
    int64_t g_timestamp, l_timestamp;

    Robot_position()
        : g_longitude(0.0)
        , g_latitude(0.0)
        , g_hdg(0.0)
        , l_x(0.0)
        , l_y(0.0)
        , l_hdg(0.0)
        , g_timestamp(0)
        , l_timestamp(0)
        {point = new Geographic_point(0.0,0.0);}
};

typedef std::tuple<double, Path_node*> TuplePath;

void Read_TXT_file(std::string path, std::vector<Data_node>& vector_node, std::vector<Data_road>& road_vector);
void Write_TXT_file(std::string path, std::vector<Data_node>& node_vector, std::vector<Data_road>& road_vector);
int get_multi_str(std::string str, std::vector<std::string>& vec_str);

void Read_XLSX_file(std::string path, std::vector<Data_node>& vector_node, std::vector<Data_road>& road_vector);
void Read_YAML_file(std::string path, std::vector<Geographic_point>* info_brut);
void Write_XLSX_file(std::string path,std::vector<Data_node>& node_vector, std::vector<Data_road>& road_vector);
void Read_JPG_file(std::string path, cv::Mat& img);
void Init_data_map(cv::Mat& map_current, cv::Mat& map_data);
void Project_all_element(std::vector<Geographic_point>& ref_border, std::vector<Data_node>& node_vector, cv::Mat& map_current, cv::Mat& map_data, std::vector<Data_road>& road_vector, bool speed_view);
void clear_road_vector(std::vector<Data_road>& road_vector);
long double toRadians(const long double degree);
double compute_distance_to_end(Data_node A, Data_node B);
double compute_weight_road(Data_road* road);void fill_path_node(std::vector<Data_node>& vector_node, std::vector<Data_road>& road_vector, std::vector<Path_node>& graph);
void fill_path_node(std::vector<Data_node>& vector_node, std::vector<Data_road>& road_vector, std::vector<Path_node>& graph);
void compute_navigation_path(int idx_start, int idx_endof, std::vector<Path_node>& graph, std::vector<Data_road>& road_vector, std::vector<Data_road*>& path_road_vector);
double get_min_dist_pos_to_road(Data_road* curr_road, Geographic_point* curr_pos, Geographic_point* projec_pos);
int get_road_ID_from_pos(std::vector<Data_road>& vect_road, Geographic_point* curr_pos, Geographic_point* projec_pos);
double get_dist_between_pos(Geographic_point* pos_A, Geographic_point* pos_B);
long double deg_to_rad(const long double degree);
void Project_tempo_point(std::vector<Geographic_point>& ref_border, cv::Mat& map_current, double long1, double lat1, double long2, double lat2);
int get_road_ID_from_pos2(std::vector<Data_road>& vect_road, Geographic_point* curr_pos, Geographic_point* projec_pos);
double get_angular_distance(Geographic_point* pointA, Geographic_point* pointB);
double get_bearing(Geographic_point* pointA, Geographic_point* pointB);
void test_function(Geographic_point* pointA, Geographic_point* pointB, Geographic_point* pointC);
int get_road_ID_from_pos2(std::vector<Data_road>& vect_road, Geographic_point* curr_pos, Geographic_point* projec_pos);
double test_function_deploy(Geographic_point* pointA, Geographic_point* pointB, Geographic_point* pointC, Geographic_point* projec_pos);
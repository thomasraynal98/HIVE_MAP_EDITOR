#include "useful.h"
#include <OpenXLSX.hpp>
#include <bits/stdc++.h>
#include <iostream>
#include <fstream>
#include <string>

void Read_TXT_file(std::string path, std::vector<Data_node>& vector_node, std::vector<Data_road>& road_vector)
{ 
    vector_node.clear();
    road_vector.clear();

    std::ifstream file(path);
    std::string str; 

    std::string data_type;

    while (std::getline(file, str))
    {
        std::vector<std::string> vect_str;

        get_multi_str(str, vect_str);
        if(vect_str.size() == 1) data_type = vect_str[0];
        else
        {
            if(data_type.compare("NODE") == 0)
            {
                Data_node new_data(std::stoi(vect_str[0]), std::stod(vect_str[1]), std::stod(vect_str[2]));
                vector_node.push_back(new_data);
            }

            if(data_type.compare("ROAD") == 0)
            {
                Data_node* tempo_save_A;
                Data_node* tempo_save_B;
                for(int i = 0; i < vector_node.size(); i++)
                {
                    if(vector_node[i].node_ID == std::stoi(vect_str[1]))
                    {
                        tempo_save_A = &vector_node[i];
                    }
                    if(vector_node[i].node_ID == std::stoi(vect_str[2]))
                    {
                        tempo_save_B = &vector_node[i];
                    }
                }
                Data_road new_road(std::stoi(vect_str[0]), tempo_save_A, tempo_save_B);

                if(std::stoi(vect_str[6]) == 1) new_road.available = true;
                else{new_road.available = false;}

                new_road.deg_to_A = std::stod(vect_str[3]);
                new_road.deg_to_B = std::stod(vect_str[4]);
                new_road.length = std::stod(vect_str[5]);
                new_road.max_speed = std::stod(vect_str[7]);
                road_vector.push_back(new_road);
            }
        }
    }
}

int get_multi_str(std::string str, std::vector<std::string>& vec_str)
{
    vec_str.clear();

    std::string T;
    std::stringstream X(str);

    int number_of_data = 0;

    while(std::getline(X, T, '|'))
    {
        vec_str.push_back(T);
        number_of_data++;
    } 

    return number_of_data;
}

void Read_XLSX_file(std::string path, std::vector<Data_node>& vector_node, std::vector<Data_road>& road_vector)
{
    OpenXLSX::XLDocument doc;
    doc.open(path);

    // ADD NODE.
    auto wks = doc.workbook().worksheet("GEO_POINT");

    for(auto& row : wks.rows())
    {
        bool end = false;
        int i = 0;

        int id = -1;
        double longitude = 0;
        double latitude = 0;

        for(auto cell : row.cells(3))
        {
            if(cell.value().type() == OpenXLSX::XLValueType::Empty) break;

            end = true;
            if(i == 0) id        = std::stoi(cell.value());
            if(i == 1) longitude = std::stod(cell.value());
            if(i == 2) latitude  = std::stod(cell.value());
            i++;
        }

        if(!end) break;

        Data_node new_data(id, longitude, latitude);
        vector_node.push_back(new_data);
    }

    // ADD ROAD.
    auto wks2 = doc.workbook().worksheet("PATH");

    for(auto& row : wks2.rows())
    {
        bool end = false;
        int i = 0;

        int id_road = -1;
        int id_pointA = 0;
        int id_pointB = 0;
        double deg_to_A = 0;
        double deg_to_B = 0;
        double length = 0;
        bool available = true;
        double speed = 0;

        for(auto cell : row.cells(8))
        {
            if(cell.value().type() == OpenXLSX::XLValueType::Empty) break;

            end = true;
            if(i == 0) id_road   = cell.value();
            if(i == 1) id_pointA = cell.value();
            if(i == 2) id_pointB = cell.value();
            if(i == 3) deg_to_A  = cell.value();
            if(i == 4) deg_to_B  = cell.value();
            if(i == 5) length    = cell.value();
            if(i == 6)
            {
                int tempo = cell.value();
                if(tempo == 1) available = true;
                else{available = false;}
            }
            if(i == 7) speed     = cell.value();
            i++;
        }

        if(!end) break;

        Data_node* tempo_save;
        for(int i = 0; i < vector_node.size(); i++)
        {
            if(id_pointA == vector_node[i].node_ID) { tempo_save = &vector_node[i]; break;}
        }
        for(int i = 0; i < vector_node.size(); i++)
        {
            if(id_pointB == vector_node[i].node_ID)
            {
                Data_road new_road(id_road, tempo_save, &vector_node[i]);
                new_road.available = available;
                new_road.deg_to_A = deg_to_A;
                new_road.deg_to_B = deg_to_B;
                new_road.length = length;
                new_road.max_speed = speed;
                road_vector.push_back(new_road);
            }
        }
    }
    doc.close();
}

void Read_YAML_file(std::string path, std::vector<Geographic_point>* ref_border)
{
    cv::FileStorage fsSettings(path, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Can't open the file " << path << std::endl;
        return;
    }

    std::string information;
    size_t size = 10;

    Geographic_point p1(0,0);
    fsSettings["p1_longitude"] >> information;
    p1.longitude = std::stod(information, &size);
    fsSettings["p1_latitude"] >> information;
    p1.latitude = std::stod(information);

    Geographic_point p2(0,0);
    fsSettings["p2_longitude"] >> information;
    p2.longitude = std::stod(information);
    fsSettings["p2_latitude"] >> information;
    p2.latitude = std::stod(information);

    ref_border->push_back(p1);
    ref_border->push_back(p2);
}

void Write_TXT_file(std::string path, std::vector<Data_node>& node_vector, std::vector<Data_road>& road_vector)
{
    std::ofstream myfile;
    myfile.open(path);
    myfile << "NODE\n";
    for(auto node : node_vector)
    {
        std::string msg_str;
        msg_str += std::to_string(node.node_ID) + "|" + std::to_string(node.point.longitude) + "|" + std::to_string(node.point.latitude) + "|\n"; 
        myfile << msg_str;
    }

    myfile << "ROAD\n";
    for(auto road : road_vector)
    {
        std::string msg_str;
        msg_str += std::to_string(road.road_ID) + "|" + std::to_string(road.A->node_ID) + "|" + std::to_string(road.B->node_ID) + "|" + std::to_string(road.deg_to_A) + "|" + std::to_string(road.deg_to_B) + "|" + std::to_string(road.length) + "|"; 
        if(road.available) msg_str += "1|";
        else{msg_str += "0|";}
        msg_str += std::to_string(road.max_speed) + "|\n";
        myfile << msg_str;
    }
    myfile.close();
}

void Write_XLSX_file(std::string path, std::vector<Data_node>& node_vector, std::vector<Data_road>& road_vector)
{
    OpenXLSX::XLDocument doc;
    doc.open(path);
    auto wks = doc.workbook().worksheet("PATH");

    // CLEAR THE FILE
    for(auto& row : wks.rows())
    {
        for(auto cell : row.cells(8))
        {
            cell.value().clear();
        }
    }

    // ADD ROAD
    int index = 1;
    for(auto road : road_vector)
    {
        std::string id_cell = "A" + std::to_string(index);
        wks.cell(id_cell).value() = road.road_ID;
        id_cell = "B" + std::to_string(index);
        wks.cell(id_cell).value() = road.A->node_ID;
        id_cell = "C" + std::to_string(index);
        wks.cell(id_cell).value() = road.B->node_ID;
        id_cell = "D" + std::to_string(index);
        wks.cell(id_cell).value() = road.deg_to_A;
        id_cell = "E" + std::to_string(index);
        wks.cell(id_cell).value() = road.deg_to_B;
        id_cell = "F" + std::to_string(index);
        wks.cell(id_cell).value() = road.length;
        id_cell = "G" + std::to_string(index);
        wks.cell(id_cell).value() = (double)(road.available);
        id_cell = "H" + std::to_string(index);
        wks.cell(id_cell).value() = road.max_speed;
        index++;
    }

    doc.save();

    // ADD POINT
    
    doc.close();
}

void Read_JPG_file(std::string path, cv::Mat& img)
{
    img = imread(path, cv::IMREAD_COLOR);
    if(img.empty())
    {
        std::cout << "Could not read the image: " << path << std::endl;
    }
}

void Init_data_map(cv::Mat& map_current, cv::Mat& map_data)
{
    cv::Mat new_map_data(map_current.rows, map_current.cols, CV_16UC1, cv::Scalar(0));
    map_data = new_map_data;
}

void Project_all_element(std::vector<Geographic_point>& ref_border, std::vector<Data_node>& node_vector, cv::Mat& map_current, cv::Mat& map_data, std::vector<Data_road>& road_vector, bool speed_view)
{    
    // Draw road.
    for(auto& road : road_vector)
    { 
        if(speed_view)
        {
            if(road.max_speed < 6) cv::line(map_current, cv::Point((int)(road.A->col_idx),(int)(road.A->row_idx)), cv::Point((int)(road.B->col_idx),(int)(road.B->row_idx)), cv::Scalar(180, 180, 180), 10, cv::LINE_8);
            if(road.max_speed > 6 && road.max_speed < 8) cv::line(map_current, cv::Point((int)(road.A->col_idx),(int)(road.A->row_idx)), cv::Point((int)(road.B->col_idx),(int)(road.B->row_idx)), cv::Scalar(102, 102, 255), 10, cv::LINE_8);
            if(road.max_speed > 8) cv::line(map_current, cv::Point((int)(road.A->col_idx),(int)(road.A->row_idx)), cv::Point((int)(road.B->col_idx),(int)(road.B->row_idx)), cv::Scalar(51, 51, 255), 10, cv::LINE_8);
        }
        if(road.available)
        {
            cv::line(map_current, cv::Point((int)(road.A->col_idx),(int)(road.A->row_idx)), cv::Point((int)(road.B->col_idx),(int)(road.B->row_idx)), cv::Scalar(182,242,176), 4, cv::LINE_8);
        }
        else
        {
            cv::line(map_current, cv::Point((int)(road.A->col_idx),(int)(road.A->row_idx)), cv::Point((int)(road.B->col_idx),(int)(road.B->row_idx)), cv::Scalar(255, 0, 127), 4, cv::LINE_8);
        }
        cv::line(map_data   , cv::Point((int)(road.A->col_idx),(int)(road.A->row_idx)), cv::Point((int)(road.B->col_idx),(int)(road.B->row_idx)), cv::Scalar(road.road_ID), 4, cv::LINE_8);
    }

    // Draw node.
    for(auto& node : node_vector)
    {   
        node.col_idx = ((node.point.longitude - ref_border[0].longitude) * (double)(map_current.cols)) / (ref_border[1].longitude - ref_border[0].longitude);
        node.row_idx = (double)(map_current.rows) - (((node.point.latitude - ref_border[1].latitude) * (double)(map_current.rows)) / (ref_border[0].latitude - ref_border[1].latitude));
        
        cv::circle(map_current, cv::Point((int)(node.col_idx),(int)(node.row_idx)),5,      cv::Scalar(0,0,250), cv::FILLED, 1,0);
        cv::circle(map_data,    cv::Point((int)(node.col_idx),(int)(node.row_idx)),5, cv::Scalar(node.node_ID), cv::FILLED, 1,0);
    }
}

void clear_road_vector(std::vector<Data_road>& road_vector)
{
    road_vector.clear();
}

long double toRadians(const long double degree)
{
    long double one_deg = (M_PI) / 180;
    return (one_deg * degree);
}

double compute_distance_to_end(Data_node A, Data_node B)
{
    // Distance.
    double lat1 = toRadians(A.point.latitude);
    double long1 = toRadians(A.point.longitude);
    double lat2 = toRadians(B.point.latitude);
    double long2 = toRadians(B.point.longitude);
    // Haversine Formula
    long double dlong = long2 - long1;
    long double dlat = lat2 - lat1;

    long double ans = pow(sin(dlat / 2), 2) +
                        cos(lat1) * cos(lat2) *
                        pow(sin(dlong / 2), 2);

    ans = 2 * asin(sqrt(ans));
    long double R = 6371;
    return ans * R;
}

double compute_weight_road(Data_road* road)
{
    return road->length / road->max_speed; // en heure decimal.
}

void fill_path_node(std::vector<Data_node>& vector_node, std::vector<Data_road>& road_vector, std::vector<Path_node>& graph)
{
    for(int idx_node = 0; idx_node < vector_node.size(); idx_node++)
    {
        Path_node new_path_node(vector_node[idx_node].node_ID, &vector_node[idx_node]);
        graph.push_back(new_path_node);
    }
    for(int idx_road = 0; idx_road < road_vector.size(); idx_road++)
    {
        if(road_vector[idx_road].available)
        {
            for(int idx_path = 0; idx_path < graph.size(); idx_path++)
            {
                if(graph[idx_path].index_node == road_vector[idx_road].A->node_ID)
                {
                    // 1 FOUND PATH NODE CONNECTION
                    for(int idx_path2 = 0; idx_path2 < graph.size(); idx_path2++)
                    {
                        if(graph[idx_path2].index_node == road_vector[idx_road].B->node_ID)
                        {
                            graph[idx_path].connection_data.push_back(&graph[idx_path2]);
                            graph[idx_path2].connection_data.push_back(&graph[idx_path]);
                            // 2 COMPUTE WEIGHT
                            graph[idx_path].connection_weight.push_back(compute_weight_road(&road_vector[idx_road]));
                            graph[idx_path2].connection_weight.push_back(compute_weight_road(&road_vector[idx_road]));
                        }
                    }
                }
            }
        }
    }
}

void compute_navigation_path(int idx_start, int idx_endof, std::vector<Path_node>& graph, std::vector<Data_road>& road_vector, std::vector<Data_road*>& path_road_vector)
{
    // GOOD INDEX.
    int detection = 0;
    for(int i = 0; i < graph.size(); i++)
    {
        if(graph[i].index_node == idx_start || graph[i].index_node == idx_endof) detection++;
    }
    if(detection < 2) return;

    // INIT A* PARAM.
    for(int i = 0; i < graph.size(); i++)
    {
        graph[i].closet = false;
        graph[i].gscore = 99999;
        graph[i].fscore = 99999;
        graph[i].come_from = NULL;
    }

    std::priority_queue<TuplePath,std::vector<TuplePath>,std::greater<TuplePath>> openList;   

    // INITIALISATION START
    Path_node* start_node;
    Path_node* endof_node;

    for(int idx_graph = 0; idx_graph < graph.size(); idx_graph++)
    {
        if(graph[idx_graph].index_node == idx_start)
        {
            start_node = &graph[idx_graph];
            start_node->gscore = 0;
        }
        if(graph[idx_graph].index_node == idx_endof)
        {
            endof_node = &graph[idx_graph];
        }
    }
    
    TuplePath start_tuple(0.0, start_node);
    openList.emplace(start_tuple);

    while(!openList.empty())
    {
        TuplePath p = openList.top();
        std::get<1>(p)->closet = true;
        openList.pop();

        // POUR CHAQUE VOISIN
        for(int idx_voisin = 0; idx_voisin < std::get<1>(p)->connection_weight.size(); idx_voisin++)
        {
            // // VERIFIER SI ON EST A DESTINATION
            if(std::get<1>(p)->connection_data[idx_voisin]->index_node == endof_node->index_node)
            {
                std::get<1>(p)->connection_data[idx_voisin]->come_from = std::get<1>(p);
                Path_node* next_node = std::get<1>(p)->connection_data[idx_voisin];

                // double distance_km = 0;
                // double time_total_decimal = 0;
                while(next_node->come_from != NULL && next_node->index_node != idx_start)
                {
                    // FOUND ROAD BETWEEN.
                    for(int i = 0; i < road_vector.size(); i++)
                    {
                        if(road_vector[i].A->node_ID == next_node->index_node && \
                        road_vector[i].B->node_ID == next_node->come_from->index_node)
                        {
                            // std::cout << " ROAD " << road_vector[i].road_ID << std::endl;
                            path_road_vector.push_back(&road_vector[i]);
                            // distance_km += road_vector[i].length;
                            // time_total_decimal += (road_vector[i].length/road_vector[i].max_speed);
                        }
                        if(road_vector[i].B->node_ID == next_node->index_node && \
                        road_vector[i].A->node_ID == next_node->come_from->index_node)
                        {
                            // std::cout << " ROAD " << road_vector[i].road_ID << std::endl;
                            path_road_vector.push_back(&road_vector[i]);
                            // distance_km += road_vector[i].length;
                            // time_total_decimal += (road_vector[i].length/road_vector[i].max_speed);
                        }
                    }

                    next_node = next_node->come_from;
                }
                // std::cout << std::setprecision(3);
                // std::cout << "DISTANCE TOTAL : " << distance_km << " KM (" << std::ceil(time_total_decimal*60) << "Min)" << std::endl;


                return;
            }

            //  // CALCULER LE TENTATIVE_GSCORE = GSCORE[CURRENT] + DEPLACEMENT[CURRENT<>VOISIN]
            double tentative_gscode = std::get<1>(p)->gscore + std::get<1>(p)->connection_weight[idx_voisin];

            //  // IF TENTATIVE_GSCORE < GSCORE[VOISIN] //SETUP A 99999 DE BASE
            if(tentative_gscode < std::get<1>(p)->connection_data[idx_voisin]->gscore)
            {
                //  //  // VOISIN PARENT = CURRENT
                std::get<1>(p)->connection_data[idx_voisin]->come_from = std::get<1>(p);

                //  //  // GSCORE[VOISIN] = TENTATIVE_GSCORE
                std::get<1>(p)->connection_data[idx_voisin]->gscore = tentative_gscode;

                //  //  // FSCORE[VOISIN] = TENTATIVE_GSCORE + DISTANCE[VOISIN<>DESTINATION]
                std::get<1>(p)->connection_data[idx_voisin]->fscore = tentative_gscode + compute_distance_to_end(*endof_node->n, *std::get<1>(p)->connection_data[idx_voisin]->n);

                //  //  // IF VOISIN N'A PAS ENCORE ETAIT VISITER [CLOSET=FALSE]
                if(!std::get<1>(p)->connection_data[idx_voisin]->closet)
                {
                    //  //  //  // AJOUT VOISIN DANS OPENLIST
                    openList.emplace(std::get<1>(p)->connection_data[idx_voisin]->fscore, std::get<1>(p)->connection_data[idx_voisin]);
                }
            }
        }
    }

    return;
}
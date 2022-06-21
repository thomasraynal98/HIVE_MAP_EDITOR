/*
    This software only allows to visualize and edite the navigation map of 
    our robots.
    Thomas Raynal for HIVE ROBOTICS. 
*/

#include <opencv2/highgui.hpp>
#include <thread>
#include <chrono>
#include <bits/stdc++.h>

#include "useful.h"

std::thread thread_display, thread_keyboard;

std::vector<Geographic_point> ref_border;
std::vector<Data_node> node_vector;
std::vector<Data_road> road_vector;
cv::Mat map_current, map_current_copy, map_data; 

int selection      = -1;
int selection_ID   = -1;
int selection_ID_2 = -1;
int phase_Value    = 0;
Opencv_area validation_area(-1,-1,-1,-1);

void mouseHandler(int event, int x, int y, int flags, void* param)
{
    switch(event){
    case cv::EVENT_LBUTTONDOWN:
        unsigned short intensity = map_data.at<unsigned short>(y, x);

        if(phase_Value == 2)
        {
            if(validation_area.is_in(x, y))
            {
                // Create new road.
                // 1. Found the good ID.
                int i = 1001; bool found = false;
                while(!found)
                {
                    found = true;
                    for(auto road : road_vector)
                    {
                        if(road.road_ID == i)
                        {
                            i += 1;
                            found = false;
                            break;
                        }
                    }
                }
                int ID_road = i;

                // 2. Add new road.
                Data_node* tempo_save;
                for(int i = 0; i < node_vector.size(); i++)
                {
                    if(selection_ID == node_vector[i].node_ID) { tempo_save = &node_vector[i]; break;}
                }
                for(int i = 0; i < node_vector.size(); i++)
                {
                    if(selection_ID_2 == node_vector[i].node_ID)
                    {
                        Data_road new_road(ID_road, tempo_save, &node_vector[i]);
                        road_vector.push_back(new_road);
                    }
                }
            }
        }

        if(intensity != 0)
        {
            selection = intensity;
            if(selection < 1000)
            {
                // Point selection.
                if(phase_Value == 0)
                {
                    selection_ID = selection;

                    // first point selection.
                    for(auto node : node_vector)
                    {
                        if(node.node_ID == selection_ID)
                        {
                            cv::circle(map_current_copy, cv::Point((int)(node.col_idx),(int)(node.row_idx)),8, cv::Scalar(255,255,0), cv::FILLED, 1,0);
                        }
                    }
                    phase_Value = 1;
                    break;
                }
                if(phase_Value == 1)
                {
                    // second point selection.
                    selection_ID_2 = selection;
                    if(selection_ID == selection_ID_2)
                    {
                        map_current_copy = map_current.clone();
                        Init_data_map(map_current, map_data);

                        Project_all_element(ref_border, node_vector, map_current_copy, map_data, road_vector); // add road.

                        selection_ID   = -1;
                        selection_ID_2 = -1;
                        phase_Value    = 0;
                        break;
                    }
                    else
                    {
                        // Show the second point.
                        int idx_col2, idx_row2, idx_col, idx_row;
                        for(auto node : node_vector)
                        {
                            if(node.node_ID == selection_ID_2)
                            {
                                idx_row2 = node.row_idx;
                                idx_col2 = node.col_idx;
                            }
                            if(node.node_ID == selection_ID)
                            {
                                idx_row = node.row_idx;
                                idx_col = node.col_idx;
                            }
                        }

                        // Show the possible new road.
                        cv::line(map_current_copy, cv::Point((int)(idx_col),(int)(idx_row)), cv::Point((int)(idx_col2),(int)(idx_row2)), cv::Scalar(150,150,0), 4, cv::LINE_8);
                        cv::circle(map_current_copy, cv::Point((int)(idx_col),(int)(idx_row)),8, cv::Scalar(255,255,0), cv::FILLED, 1,0);
                        cv::circle(map_current_copy, cv::Point((int)(idx_col2),(int)(idx_row2)),8, cv::Scalar(255,255,0), cv::FILLED, 1,0);

                        // Add validation area
                        cv::rectangle(map_current_copy, cv::Point((int)((idx_col+idx_col2)/2),(int)((idx_row+idx_row2)/2)), cv::Point((int)((idx_col+idx_col2)/2+30),(int)((idx_row+idx_row2)/2+30)), cv::Scalar(0, 252, 124), -1, cv::LINE_8);
                        validation_area.fullfill((int)((idx_col+idx_col2)/2), (int)((idx_row+idx_row2)/2), (int)((idx_col+idx_col2)/2+30), (int)((idx_row+idx_row2)/2+30));

                        phase_Value = 2;
                        break;
                    }
                }
            }
        }
        else
        {
            // Generate new normal image.
            map_current_copy = map_current.clone();
            Init_data_map(map_current, map_data);
            Project_all_element(ref_border, node_vector, map_current_copy, map_data, road_vector); // add road.

            selection      = -1;
            selection_ID   = -1;
            selection_ID_2 = -1;
            phase_Value    = 0;
        }
    }
}

void function_thread_display()
{
    // THREAD DESCRIPTION: this thread display the map

    //
    int frequency       = 20;
    double time_of_loop = 1000/frequency;                  // en milliseconde.
    std::chrono::high_resolution_clock::time_point last_loop_time = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point x              = std::chrono::high_resolution_clock::now();
    auto next = std::chrono::high_resolution_clock::now();
    //

    int mouseParam = cv::EVENT_FLAG_LBUTTON;
    cv::namedWindow( "HIVE MAP EDITOR", 4);
    cv::setMouseCallback("HIVE MAP EDITOR",mouseHandler, NULL);

    while(true)
    {
        //
        x                          = std::chrono::high_resolution_clock::now();         
        last_loop_time             = x;
        next                       += std::chrono::milliseconds((int)time_of_loop);
        std::this_thread::sleep_until(next);
        //

        cv::imshow("HIVE MAP EDITOR", map_current_copy);
        char d =(char)cv::waitKey(25);
    }
}

void function_thread_keyboard()
{
    std::string input_user;

    while(true)
    {
        std::cin >> input_user;
        if(input_user.compare("SAVE") == 0)
        {
            std::cout << "Sauvegarde de la session." << std::endl;
            Write_XLSX_file("../data/Hive_Map_Database2.xlsx", node_vector, road_vector);
        }
    }
}

int main()
{
    // Initialisation software.
    std::cout << std::setprecision(10);

    // STEP 1 : Read initialisation param.
    Read_YAML_file("../data/map_information.yaml", &ref_border);

    // STEP 2 : Collect "node" data from XLSX database.
    Read_XLSX_file("../data/Hive_Map_Database2.xlsx", &node_vector, &road_vector);

    // STEP 3 : Import map of the La Defense.
    Read_JPG_file("../data/02_LaDefense.jpg", map_current);
    map_current_copy = map_current.clone();

    // STEP 4 : Init the data map.
    Init_data_map(map_current, map_data);

    // STEP 5 : Project XLSX data on map_current and map_data.
    Project_all_element(ref_border, node_vector, map_current_copy, map_data, road_vector);

    // Thread run.
    thread_display  = std::thread(&function_thread_display);
    thread_keyboard = std::thread(&function_thread_keyboard);

    thread_display.join();
    thread_keyboard.join();
}
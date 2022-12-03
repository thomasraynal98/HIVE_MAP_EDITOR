/*
    This software only allows to visualize and edite the navigation map of 
    our robots.
    Thomas Raynal for HIVE ROBOTICS. 
*/

#include <opencv2/highgui.hpp>
#include <thread>
#include <chrono>
#include <bits/stdc++.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

#include "useful.h"

std::thread thread_display, thread_keyboard;

std::vector<Geographic_point> ref_border;
std::vector<Data_node> node_vector;
std::vector<Data_road> road_vector;
std::vector<Data_road*> path_road_vector;
std::vector<Path_node> graph;
cv::Mat map_current, map_current_copy, map_data; 

bool opt_speed_view = false;
int selection      = -1;
int selection_ID   = -1;
int selection_ID_2 = -1;
int phase_Value    = 0;
Opencv_area validation_area(-1,-1,-1,-1);
Opencv_area navigation_area(-1,-1,-1,-1);

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

                phase_Value = 0;
            }
            else if(navigation_area.is_in(x, y))
            {
                // Create new road.
                path_road_vector.clear();
                graph.clear();
                fill_path_node(node_vector, road_vector, graph); 
                compute_navigation_path(selection_ID, selection_ID_2, graph, road_vector, path_road_vector);

                // DRAW PATH IF ITS GOOD.
                if(path_road_vector.size() != 0)
                {
                    map_current_copy = map_current.clone();
                    Init_data_map(map_current, map_data);
                    Project_all_element(ref_border, node_vector, map_current_copy, map_data, road_vector, opt_speed_view); // add road.

                    double distance_km = 0;
                    double time_travel = 0;
                    std::cout << "Chemin entre le node " << selection_ID << " et le node " << selection_ID_2 << " : " << std::endl;
                    for(auto road : path_road_vector)
                    {
                        // Show the line.
                        cv::line(map_current_copy, cv::Point((int)(road->A->col_idx),(int)(road->A->row_idx)), cv::Point((int)(road->B->col_idx),(int)(road->B->row_idx)), cv::Scalar(105,0,0), 7, cv::LINE_8);

                        cv::circle(map_current_copy, cv::Point((int)(road->A->col_idx),(int)(road->A->row_idx)),5, cv::Scalar(0,0,255), cv::FILLED, 1,0);
                        cv::circle(map_current_copy, cv::Point((int)(road->B->col_idx),(int)(road->B->row_idx)),5, cv::Scalar(0,0,255), cv::FILLED, 1,0);

                        distance_km += road->length;
                        time_travel += (road->length / road->max_speed) * 60;
                        std::cout << road->road_ID << " > ";
                    }
                    std::cout << "END" << std::endl;
                    std::cout << "La distance total est de " << distance_km << " KM. Un temps de traject de " << std::ceil(time_travel) << " Min(s)." << std::endl;
                    for(int i = 0; i < node_vector.size(); i++)
                    {
                        if(node_vector[i].node_ID == selection_ID)
                        {
                            cv::circle(map_current_copy, cv::Point((int)(node_vector[i].col_idx),(int)(node_vector[i].row_idx)),8, cv::Scalar(0,255,0), cv::FILLED, 1,0);
                        }
                        if(node_vector[i].node_ID == selection_ID_2)
                        {
                            cv::circle(map_current_copy, cv::Point((int)(node_vector[i].col_idx),(int)(node_vector[i].row_idx)),8, cv::Scalar(255,255,0), cv::FILLED, 1,0);
                        }
                    }
                }
                else
                {
                    std::cout << "[!] Aucun chemin disponible entre le node " << selection_ID << " et le node " << selection_ID_2 << "." << std::endl;
                }  
            }
            else { phase_Value = 0; }         
        }

        if(phase_Value == 1 && selection_ID > 1000)
        {
            if(validation_area.is_in(x, y))
            {
                int option_selection = validation_area.option_selection(x, y);
                
                if(option_selection == 1)
                {
                    // CLOSE ROAD
                    for(int i = 0; i < road_vector.size(); i++)
                    {
                        if(road_vector[i].road_ID == selection_ID)
                        {
                            if(road_vector[i].available) road_vector[i].available = false;
                            else{road_vector[i].available = true;}
                            break;
                        }
                    }
                }
                if(option_selection == 2)
                {
                    // DELETE ROAD
                    int index = -1;
                    for(int i = 0; i < road_vector.size(); i++)
                    {
                        if(road_vector[i].road_ID == selection_ID) { index = i; break;}
                    }
                    road_vector.erase(road_vector.begin() + index);
                }
                if(option_selection == 3)
                {
                    // LOW ROAD
                    for(int i = 0; i < road_vector.size(); i++)
                    {
                        if(road_vector[i].road_ID == selection_ID) { road_vector[i].max_speed = 3.001; break;}
                    }
                }
                if(option_selection == 4)
                {
                    // MID ROAD
                    for(int i = 0; i < road_vector.size(); i++)
                    {
                        if(road_vector[i].road_ID == selection_ID) { road_vector[i].max_speed = 7.001; break;}
                    }
                }
                if(option_selection == 5)
                {
                    // HIGH ROAD
                    for(int i = 0; i < road_vector.size(); i++)
                    {
                        if(road_vector[i].road_ID == selection_ID) { road_vector[i].max_speed = 10.001; break;}
                    }
                }
                if(option_selection == 6)
                {
                    // INFO
                    for(int i = 0; i < road_vector.size(); i++)
                    {
                        if(road_vector[i].road_ID == selection_ID)
                        {
                            std::cout << "ROAD : " << road_vector[i].road_ID << " | " << "SPEED : " << road_vector[i].max_speed << " | " << \
                            "LENGTH : " << road_vector[i].length << " | " << " STATE : ";
                            if(road_vector[i].available) std::cout << "OPEN" << std::endl;
                            else { std::cout << "CLOSE" << std::endl; }
                            break;
                        }
                    }
                }
                if(option_selection == 7)
                {
                    // NO SURVEILLENCE
                    for(int i = 0; i < road_vector.size(); i++)
                    {
                        if(road_vector[i].road_ID == selection_ID)
                        {
                            road_vector[i].opt_auto = 0;
                            break;
                        }
                    }
                }
                if(option_selection == 8)
                {
                    // SURVEILLENCE DEMANDER
                    for(int i = 0; i < road_vector.size(); i++)
                    {
                        if(road_vector[i].road_ID == selection_ID)
                        {
                            road_vector[i].opt_auto = 1;
                            break;
                        }
                    }
                }
                if(option_selection == 9)
                {
                    // PILOTAGE REQUIS
                    for(int i = 0; i < road_vector.size(); i++)
                    {
                        if(road_vector[i].road_ID == selection_ID)
                        {
                            road_vector[i].opt_auto = 2;
                            break;
                        }
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

                        Project_all_element(ref_border, node_vector, map_current_copy, map_data, road_vector, opt_speed_view); // add road.

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

                        // Add Navigation area
                        cv::rectangle(map_current_copy, cv::Point((int)((idx_col+idx_col2)/2+30),(int)((idx_row+idx_row2)/2)), cv::Point((int)((idx_col+idx_col2)/2+60),(int)((idx_row+idx_row2)/2+30)), cv::Scalar(105, 0, 0), -1, cv::LINE_8);
                        navigation_area.fullfill((int)((idx_col+idx_col2)/2+30), (int)((idx_row+idx_row2)/2), (int)((idx_col+idx_col2)/2+60), (int)((idx_row+idx_row2)/2+30));

                        phase_Value = 2;
                        break;
                    }
                }
            }
            if(selection > 1000)
            {
                // ROAD GESTION.
                if(phase_Value == 0)
                {
                    selection_ID = selection;

                    // Show the second point.
                    for(auto road : road_vector)
                    {
                        if(road.road_ID == selection_ID)
                        {
                            // Show the line.
                            cv::line(map_current_copy, cv::Point((int)(road.A->col_idx),(int)(road.A->row_idx)), cv::Point((int)(road.B->col_idx),(int)(road.B->row_idx)), cv::Scalar(150,150,0), 4, cv::LINE_8);
                            cv::circle(map_current_copy, cv::Point((int)(road.A->col_idx),(int)(road.A->row_idx)),5, cv::Scalar(0,0,255), cv::FILLED, 1,0);
                            cv::circle(map_current_copy, cv::Point((int)(road.B->col_idx),(int)(road.B->row_idx)),5, cv::Scalar(0,0,255), cv::FILLED, 1,0);
                            // Show the option area

                            cv::rectangle(map_current_copy, cv::Point((int)((road.A->col_idx+road.B->col_idx)/2),    (int)((road.A->row_idx+road.B->row_idx)/2)), cv::Point((int)((road.A->col_idx+road.B->col_idx))/2+30 ,(int)((road.A->row_idx+road.B->row_idx)/2+30)), cv::Scalar(255, 0, 127), -1, cv::LINE_8); // CLOSE ROAD
                            cv::rectangle(map_current_copy, cv::Point((int)((road.A->col_idx+road.B->col_idx)/2+30), (int)((road.A->row_idx+road.B->row_idx)/2)), cv::Point((int)((road.A->col_idx+road.B->col_idx))/2+60 ,(int)((road.A->row_idx+road.B->row_idx)/2+30)), cv::Scalar(0, 0, 255), -1, cv::LINE_8); // DELETE ROAD
                            cv::rectangle(map_current_copy, cv::Point((int)((road.A->col_idx+road.B->col_idx)/2+60), (int)((road.A->row_idx+road.B->row_idx)/2)), cv::Point((int)((road.A->col_idx+road.B->col_idx))/2+90 ,(int)((road.A->row_idx+road.B->row_idx)/2+30)), cv::Scalar(76, 153, 0), -1, cv::LINE_8); // DELETE ROAD

                            cv::rectangle(map_current_copy, cv::Point((int)((road.A->col_idx+road.B->col_idx)/2), (int)((road.A->row_idx+road.B->row_idx)/2+30)), cv::Point((int)((road.A->col_idx+road.B->col_idx))/2+30 ,(int)((road.A->row_idx+road.B->row_idx)/2+60)), cv::Scalar(180, 180, 180), -1, cv::LINE_8); // SLOW ROAD
                            cv::rectangle(map_current_copy, cv::Point((int)((road.A->col_idx+road.B->col_idx)/2+30), (int)((road.A->row_idx+road.B->row_idx)/2+30)), cv::Point((int)((road.A->col_idx+road.B->col_idx))/2+60,(int)((road.A->row_idx+road.B->row_idx)/2+60)), cv::Scalar(100, 100, 100), -1, cv::LINE_8); // STANDARD ROAD
                            cv::rectangle(map_current_copy, cv::Point((int)((road.A->col_idx+road.B->col_idx)/2+60),(int)((road.A->row_idx+road.B->row_idx)/2+30)), cv::Point((int)((road.A->col_idx+road.B->col_idx))/2+90,(int)((road.A->row_idx+road.B->row_idx)/2+60)), cv::Scalar(30, 30, 30), -1, cv::LINE_8); // HIGH ROAD
                            
                            // Option de suivit du robot
                            cv::rectangle(map_current_copy, cv::Point((int)((road.A->col_idx+road.B->col_idx)/2), (int)((road.A->row_idx+road.B->row_idx)/2+60)), cv::Point((int)((road.A->col_idx+road.B->col_idx))/2+30 ,(int)((road.A->row_idx+road.B->row_idx)/2+90)), cv::Scalar(224, 224, 224), -1, cv::LINE_8); // NO ASSISTANCE
                            cv::rectangle(map_current_copy, cv::Point((int)((road.A->col_idx+road.B->col_idx)/2+30), (int)((road.A->row_idx+road.B->row_idx)/2+60)), cv::Point((int)((road.A->col_idx+road.B->col_idx))/2+60,(int)((road.A->row_idx+road.B->row_idx)/2+90)), cv::Scalar(153, 255, 255), -1, cv::LINE_8); // SURVEILLENCE REQUIRED
                            cv::rectangle(map_current_copy, cv::Point((int)((road.A->col_idx+road.B->col_idx)/2+60),(int)((road.A->row_idx+road.B->row_idx)/2+60)), cv::Point((int)((road.A->col_idx+road.B->col_idx))/2+90,(int)((road.A->row_idx+road.B->row_idx)/2+90)), cv::Scalar(51, 51, 255), -1, cv::LINE_8); // NEED HUMAIN

                            validation_area.fullfill((int)((road.A->col_idx+road.B->col_idx)/2), (int)((road.A->row_idx+road.B->row_idx)/2), (int)(road.A->col_idx+road.B->col_idx/2+90), (int)((road.A->row_idx+road.B->row_idx)/2+90));

                            phase_Value = 1;
                            break;
                        }
                    }
                }
            }
        }
        else
        {
            // Generate new normal image.
            if(phase_Value != 2)
            {
                map_current_copy = map_current.clone();
                Init_data_map(map_current, map_data);
                Project_all_element(ref_border, node_vector, map_current_copy, map_data, road_vector, opt_speed_view); // add road.

                selection      = -1;
                selection_ID   = -1;
                selection_ID_2 = -1;
                phase_Value    = 0;
            }
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
        std::cout << "> ";
        std::cin >> input_user;
        if(input_user.compare("SAVE") == 0)
        {
            std::cout << "Sauvegarde de la session." << std::endl;
            // Write_XLSX_file("../data/Hive_Map_Database2.xlsx", node_vector, road_vector);
            Write_TXT_file("../data/COURDIMANCHE.txt", node_vector, road_vector);
            // system("make push-git-feature"); 
            system("cp ../data/COURDIMANCHE.txt ../data/HMD.txt");
            system("git add ../."); 
            system("git commit -m \"new MAP uplod.\"");
            system("git pull");
            system("git push origin feature/branch");

            system("git checkout main");
            system("git pull");
            system("git merge feature/branch");
            system("git push origin main");
        }
        if(input_user.compare("CLEAR") == 0)
        {
            std::cout << "Suppression de toute les routes." << std::endl;
            clear_road_vector(road_vector);
            map_current_copy = map_current.clone();
            Init_data_map(map_current, map_data);
            Project_all_element(ref_border, node_vector, map_current_copy, map_data, road_vector, opt_speed_view); // add road.
        }
        if(input_user.compare("OPT_SPEED") == 0)
        {
            std::cout << "Activer/Desactiver le mode vitesse." << std::endl;
            opt_speed_view = !opt_speed_view;
            map_current_copy = map_current.clone();
            Init_data_map(map_current, map_data);
            Project_all_element(ref_border, node_vector, map_current_copy, map_data, road_vector, opt_speed_view); // add road.
        }
        if(input_user[0] == '1')
        {
            bool found = false;
            for(auto road : road_vector)
            {
                if(road.road_ID == std::stoi(input_user))
                {
                    found = true;
                    map_current_copy = map_current.clone();
                    Init_data_map(map_current, map_data);
                    Project_all_element(ref_border, node_vector, map_current_copy, map_data, road_vector, opt_speed_view); // add road.
                    // Show the line.
                    cv::line(map_current_copy, cv::Point((int)(road.A->col_idx),(int)(road.A->row_idx)), cv::Point((int)(road.B->col_idx),(int)(road.B->row_idx)), cv::Scalar(0,255,255), 10, cv::LINE_8);

                    cv::circle(map_current_copy, cv::Point((int)(road.A->col_idx),(int)(road.A->row_idx)),5, cv::Scalar(0,0,255), cv::FILLED, 1,0);
                    cv::circle(map_current_copy, cv::Point((int)(road.B->col_idx),(int)(road.B->row_idx)),5, cv::Scalar(0,0,255), cv::FILLED, 1,0);

                    // SHOW INFORMATION 
                    std::cout << "ROAD : " << road.road_ID << " | " << "SPEED : " << road.max_speed << " | " << \
                    "LENGTH : " << road.length << " | " << " STATE : ";
                    if(road.available) std::cout << "OPEN" << std::endl;
                    else { std::cout << "CLOSE" << std::endl; }

                    break;
                }
            }
            if(!found) std::cout << "[!] La route " << std::stoi(input_user) << " n'existe pas." << std::endl;
        }
        if(input_user.compare("ALL_SPEED_LOW") == 0)
        {
            std::cout << "Toute les routes sont passé en vitesse LOW." << std::endl;
            for(int i = 0; i < road_vector.size(); i++)
            {
                road_vector[i].max_speed = 3.001;
            }
            map_current_copy = map_current.clone();
            Init_data_map(map_current, map_data);
            Project_all_element(ref_border, node_vector, map_current_copy, map_data, road_vector, opt_speed_view); // add road.
        }
        if(input_user.compare("ALL_SPEED_STANDARD") == 0)
        {
            std::cout << "Toute les routes sont passé en vitesse STANDARD." << std::endl;
            for(int i = 0; i < road_vector.size(); i++)
            {
                road_vector[i].max_speed = 7.001;
            }
            map_current_copy = map_current.clone();
            Init_data_map(map_current, map_data);
            Project_all_element(ref_border, node_vector, map_current_copy, map_data, road_vector, opt_speed_view); // add road.
        }
        if(input_user.compare("ALL_SPEED_HIGH") == 0)
        {
            std::cout << "Toute les routes sont passé en vitesse HIGH." << std::endl;
            for(int i = 0; i < road_vector.size(); i++)
            {
                road_vector[i].max_speed = 10.001;
            }
            map_current_copy = map_current.clone();
            Init_data_map(map_current, map_data);
            Project_all_element(ref_border, node_vector, map_current_copy, map_data, road_vector, opt_speed_view); // add road.
        }
        if(input_user.compare("PATH") == 0)
        {
            std::cout << "(GREEN) START > ";
            int index_start = -1;
            std::cin >> index_start;
            for(int i = 0; i < node_vector.size(); i++)
                {
                    if(node_vector[i].node_ID == index_start)
                    {
                        cv::circle(map_current_copy, cv::Point((int)(node_vector[i].col_idx),(int)(node_vector[i].row_idx)),8, cv::Scalar(0,255,0), cv::FILLED, 1,0);
                    }
                }
            std::cout << "(CYAN)  ENDOF > ";
            int index_endof = -1;
            std::cin >> index_endof;

            path_road_vector.clear();
            graph.clear();
            fill_path_node(node_vector, road_vector, graph); 
            compute_navigation_path(index_start, index_endof, graph, road_vector, path_road_vector);

            // DRAW PATH IF ITS GOOD.
            if(path_road_vector.size() != 0)
            {
                map_current_copy = map_current.clone();
                Init_data_map(map_current, map_data);
                Project_all_element(ref_border, node_vector, map_current_copy, map_data, road_vector, opt_speed_view); // add road.

                double distance_km = 0;
                double time_travel = 0;
                std::cout << "Chemin entre le node " << index_start << " et le node " << index_endof << " : " << std::endl;
                for(auto road : path_road_vector)
                {
                    // Show the line.
                    cv::line(map_current_copy, cv::Point((int)(road->A->col_idx),(int)(road->A->row_idx)), cv::Point((int)(road->B->col_idx),(int)(road->B->row_idx)), cv::Scalar(105,0,0), 7, cv::LINE_8);

                    cv::circle(map_current_copy, cv::Point((int)(road->A->col_idx),(int)(road->A->row_idx)),5, cv::Scalar(0,0,255), cv::FILLED, 1,0);
                    cv::circle(map_current_copy, cv::Point((int)(road->B->col_idx),(int)(road->B->row_idx)),5, cv::Scalar(0,0,255), cv::FILLED, 1,0);

                    distance_km += road->length;
                    time_travel += (road->length / road->max_speed) * 60;
                    std::cout << road->road_ID << " > ";
                }
                std::cout << "END" << std::endl;
                std::cout << "La distance total est de " << distance_km << " KM. Un temps de traject de " << std::ceil(time_travel) << " Min(s)." << std::endl;
                for(int i = 0; i < node_vector.size(); i++)
                {
                    if(node_vector[i].node_ID == index_start)
                    {
                        cv::circle(map_current_copy, cv::Point((int)(node_vector[i].col_idx),(int)(node_vector[i].row_idx)),8, cv::Scalar(0,255,0), cv::FILLED, 1,0);
                    }
                    if(node_vector[i].node_ID == index_endof)
                    {
                        cv::circle(map_current_copy, cv::Point((int)(node_vector[i].col_idx),(int)(node_vector[i].row_idx)),8, cv::Scalar(255,255,0), cv::FILLED, 1,0);
                    }
                }
            }
            else
            {
                std::cout << "[!] Aucun chemin disponible entre le node " << index_start << " et le node " << index_endof << "." << std::endl;
            }
        }
        if(input_user.compare("PROXI") == 0)
        {
            std::cout << "(COORDINATE POINT IN FORMAT :LONG|LAT| > ";
            std::string input_str = "";
            std::cin >> input_str;

            std::vector<std::string> vect_string;
            get_multi_str(input_str, vect_string);

            Robot_position input_position = Robot_position();
            input_position.point->longitude = std::stod(vect_string[0]);
            input_position.point->latitude  = std::stod(vect_string[1]);

            Geographic_point project_point_debug = Geographic_point(0.0,0.0);
            int index_road = get_road_ID_from_pos2(road_vector, input_position.point, &project_point_debug);

            for(auto road : road_vector)
            {
                if(road.road_ID == index_road)
                {
                    map_current_copy = map_current.clone();
                    Init_data_map(map_current, map_data);
                    Project_all_element(ref_border, node_vector, map_current_copy, map_data, road_vector, opt_speed_view); // add road.
                    // Show the line.
                    cv::line(map_current_copy, cv::Point((int)(road.A->col_idx),(int)(road.A->row_idx)), cv::Point((int)(road.B->col_idx),(int)(road.B->row_idx)), cv::Scalar(0,255,255), 10, cv::LINE_8);

                    cv::circle(map_current_copy, cv::Point((int)(road.A->col_idx),(int)(road.A->row_idx)),5, cv::Scalar(0,0,255), cv::FILLED, 1,0);
                    cv::circle(map_current_copy, cv::Point((int)(road.B->col_idx),(int)(road.B->row_idx)),5, cv::Scalar(0,0,255), cv::FILLED, 1,0);

                    // SHOW INFORMATION 
                    std::cout << "ROAD : " << road.road_ID << " | " << "SPEED : " << road.max_speed << " | " << \
                    "LENGTH : " << road.length << " | " << " STATE : ";
                    if(road.available) std::cout << "OPEN" << std::endl;
                    else { std::cout << "CLOSE" << std::endl; }

                    Project_tempo_point(ref_border, map_current_copy, input_position.point->longitude, input_position.point->latitude, project_point_debug.longitude, project_point_debug.latitude);

                    break;
                }
            }
        }
    }
}

int main()
{
    // Initialisation software.
    std::cout << std::setprecision(3);

    // STEP 1 : Read initialisation param.
    Read_YAML_file("../data/map_information.yaml", &ref_border);

    // STEP 2 : Collect "node" data from XLSX database.
    // Read_XLSX_file("../data/Hive_Map_Database3.xlsx", node_vector, road_vector);
    // Read_TXT_file("../data/HMD_PARKVESINET_220912.txt", node_vector, road_vector);
    Read_TXT_file("../data/COURDIMANCHE.txt", node_vector, road_vector);

    // STEP 3 : Import map of the La Defense.
    // Read_JPG_file("../data/carte_parc_vesinet_page-0001.jpg", map_current);
    Read_JPG_file("../data/COURDIMANCHE.jpg", map_current);
    map_current_copy = map_current.clone();

    // STEP 4 : Init the data map.
    Init_data_map(map_current, map_data);

    // STEP 5 : Project XLSX data on map_current and map_data.
    map_current_copy = map_current.clone();
    Init_data_map(map_current, map_data);
    Project_all_element(ref_border, node_vector, map_current_copy, map_data, road_vector, opt_speed_view);

    // Thread run.
    thread_display  = std::thread(&function_thread_display);
    thread_keyboard = std::thread(&function_thread_keyboard);

    thread_display.join();
    thread_keyboard.join();
}
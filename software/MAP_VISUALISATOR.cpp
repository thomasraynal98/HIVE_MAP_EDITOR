/*
    This software only allows to visualize the navigation map of our robots.
    Thomas Raynal for HIVE ROBOTICS. 
*/

#include <opencv2/highgui.hpp>

#include "useful.h"

std::vector<Geographic_point> ref_border;
std::vector<Data_node> node_vector;
cv::Mat map_current;

void mouseHandler(int event, int x, int y, int flags, void* param)
{
    switch(event){
    case cv::EVENT_LBUTTONDOWN:
        if(flags & cv::EVENT_FLAG_CTRLKEY) 
            printf("Left button down with CTRL pressed\n");
            std::cout << "\nPixel(" << x << "," << y << ")" << std::endl;
        break;
    case cv::EVENT_LBUTTONUP:
        printf("Left button up\n");
        break;
    }
}

int main()
{
    std::cout << std::setprecision(10);

    // STEP 1 : Read initialisation param.
    Read_YAML_file("../data/map_information.yaml", &ref_border);

    // STEP 2 : Collect "node" data from XLSX database.
    // Read_XLSX_file("../data/Hive_Map_Database2.xlsx", &node_vector);

    // STEP 3 : Import map of the La Defense.
    Read_JPG_file("../data/02_LaDefense.jpg", map_current);

    // STEP 5 : Get the pixel localisation of click.

    int mouseParam = cv::EVENT_FLAG_LBUTTON;
    cv::namedWindow( "HIVE MAP VISUALISATOR", 4);
    cv::setMouseCallback("HIVE MAP VISUALISATOR",mouseHandler,&mouseParam);

    // STEP 4 : Try to project point on this jpg.

    for(auto node : node_vector)
    {   
        node.col_idx = ((node.point.longitude - ref_border[0].longitude) * (double)(map_current.cols)) / (ref_border[1].longitude - ref_border[0].longitude);
        node.row_idx = (double)(map_current.rows) - (((node.point.latitude - ref_border[1].latitude) * (double)(map_current.rows)) / (ref_border[0].latitude - ref_border[1].latitude));
        
        cv::circle(map_current, cv::Point((int)(node.col_idx),(int)(node.row_idx)),5, cv::Scalar(0,0,250), cv::FILLED, 1,0);
    }

    cv::imshow("HIVE MAP VISUALISATOR", map_current);
    int k = cv::waitKey(0); // Wait for a keystroke in the window

    return 0;
}

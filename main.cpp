#include <memory>
#include <fstream>
#include <cassert>
#include <iomanip>
#include <ctime>
#include <sys/timeb.h>
#include <string>
#include <iostream>
#include <algorithm>
#include <vector>
#include <numeric>
#include <cmath>
#include "map.h"

using namespace std;

struct LaneId{
    int road_id;
    int seg_id;
    int local_id;
};

std::shared_ptr<zjlmap::Map> Gmap;

zjlmap::LaneId pre_lane_id;

int sign(int x)
{
    if (x > 0) return 1;
    else if (x==0) return 0;
    else return -1;
}

bool GetCurve(zjlmap::SLZ slz, double &k, double &dL, double &x, double &y, double &z, double &hdg)
{
    std::vector<zjlmap::TracePoint> lane_points_info;
    double s1 = slz.s - 0.1;
    double s2 = slz.s + 0.1;
    zjlmap::ErrorCode ec = Gmap->calc_lane_center_line_curv(slz.lane_id, s1, s2, 0.1, lane_points_info);
    if ((ec == zjlmap::ErrorCode::kOK) && (lane_points_info.size() > 1))
    {
        k = -sign(slz.lane_id.local_id)*lane_points_info[1].curv;
        dL = -sign(slz.lane_id.local_id)*(slz.l-lane_points_info[1].l);
        x = lane_points_info[1].x;
        y = lane_points_info[1].y;
        z = lane_points_info[1].z;
        hdg = lane_points_info[1].hdg;
        return true;       
    } 
    else
    {
        printf("calc lane center line failed.\n");
        return false;
    }
}

bool GetLeftLane(zjlmap::SLZ slz, int &leftlane)
{
    // get current lane linkage information
    zjlmap::LaneLinkage lane_linkage;
    zjlmap::ErrorCode ec = Gmap->query_lane_linkage(slz.lane_id, lane_linkage); 
    if (ec != zjlmap::ErrorCode::kOK)
    {
        printf("query lane linkage failed!\n");
        return false;
    }

    if (lane_linkage.left_neighbor_valid == -1)
    {
        leftlane = -1;
    }
    else{
        leftlane = 0;
    }

    return true;
}


int main(int argc, char **argv)
{
    // map 
    Gmap = std::make_shared<zjlmap::Map>();
    std::string mapName = "ZhejiangLabPark_V0102_211111.xodr";
    // 加载地图
    int map_hanle = 0;
    zjlmap::ErrorCode ec = Gmap->unload(map_hanle);
    if (zjlmap::ErrorCode::kOK == ec)
    {
        std::string configPath("./data/");
        auto mapPath = configPath + mapName;
        ec = Gmap->load(mapPath.data(), map_hanle);
        if (zjlmap::ErrorCode::kOK!=ec)
        {
            printf("map load is failed, ec=%d\n", ec);
            return false;
        }
        else
        {
            printf("map load is success!!\n");
        }
    }
    else
    {
        printf("map unload is failed, ec=%d\n", ec);
        return false;
    }

    // 获取车道序列数据
    vector<LaneId> lane_seqs;
    ifstream laneseqs_ifile("/home/toby/Nutstore/jupyter_ws/zhejianglabugv/splitpath/laneseqs.txt", std::ios::in);
    if (laneseqs_ifile.is_open() == false)
    {
        printf("open laneseqs file failed.\n");
        return 0;
    }
    while (!laneseqs_ifile.eof())
    {
        LaneId lane_id;
        laneseqs_ifile>>lane_id.road_id>>lane_id.seg_id>>lane_id.local_id;
        laneseqs_ifile.get();
        lane_seqs.push_back(lane_id);
    }
    laneseqs_ifile.close();

    // 获取 轨迹数据文件
    ifstream localization_ifile("/home/toby/Codes/record_parser/localization/localization1.txt", std::ios::in);
    // 导出包含地图道路信息的文件
    ofstream laneinfo_ofile("/home/toby/Codes/record_parser/localization/laneinfo.txt", std::ios::out);
    if (localization_ifile.is_open() == false)
    {
        printf("load localization file failed.\n");
        return 0;
    }

    int index = 0;

    while (!localization_ifile.eof())
    {
        long long int timestamp;
        double x, y, z, phi;
        localization_ifile>>timestamp>>x>>y>>z>>phi;
        localization_ifile.get();
        zjlmap::SLZ slz = zjlmap::EmptySLZ;

        if (lane_seqs[index].road_id > 0 && Gmap->is_point_in_road_range(lane_seqs[index].road_id, {x, y, z}, 5.0, slz))
        {
            slz.lane_id.road_id = lane_seqs[index].road_id;
            slz.lane_id.section_idx = lane_seqs[index].seg_id;
            slz.lane_id.local_id = lane_seqs[index].local_id;
            double k, dL, cx, cy, cz, chdg;
            int left_lane = 0;
            if (!GetCurve(slz, k, dL, cx, cy, cz, chdg))
            {
                laneinfo_ofile<<std::setprecision(10)<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "
                                                        <<0<<" "<<0<<" "<<0<<" "
                                                        <<0<<" "<<0<<endl;
                continue;
            }

            if (!GetLeftLane(slz, left_lane))
            {
                printf("road id %d, seg id %d, local id %d, s %f, l %f\n", slz.lane_id.road_id, slz.lane_id.section_idx, slz.lane_id.local_id, slz.s, slz.l);
                laneinfo_ofile<<std::setprecision(10)<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "
                                                        <<0<<" "<<0<<" "<<0<<" "
                                                        <<0<<" "<<0<<endl;
                continue;
            }

                //保存 曲率  横向偏移距离  左车道存在与否
            laneinfo_ofile<<std::setprecision(10)<<timestamp<<" "<<k<<" "<<dL<<" "<<left_lane<<" "<<cx<<" "<<cy<<" "<<cz<<" "<<chdg<<" "
                                                    <<slz.lane_id.road_id<<" "<<slz.lane_id.section_idx<<" "<<slz.lane_id.local_id<<" "
                                                    <<slz.s<<" "<<slz.l<<endl;   
            continue;                    
        }

        if (index+1 < lane_seqs.size())
        {
            if (lane_seqs[index+1].road_id == 0)
            {
                index += 1;
                printf("road id %d, seg id %d, local id %d, s %f, l %f\n", slz.lane_id.road_id, slz.lane_id.section_idx, slz.lane_id.local_id, slz.s, slz.l);
                laneinfo_ofile<<std::setprecision(10)<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "
                                                        <<0<<" "<<0<<" "<<0<<" "
                                                        <<0<<" "<<0<<endl;
                continue;                
            }

            if (Gmap->is_point_in_road_range(lane_seqs[index+1].road_id, {x, y, z}, 5.0, slz))
            {
                index += 1;
                slz.lane_id.road_id = lane_seqs[index].road_id;
                slz.lane_id.section_idx = lane_seqs[index].seg_id;
                slz.lane_id.local_id = lane_seqs[index].local_id;
                double k, dL, cx, cy, cz, chdg;
                int left_lane = 0;
                if (!GetCurve(slz, k, dL, cx, cy, cz, chdg))
                {
                    laneinfo_ofile<<std::setprecision(10)<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "
                                                            <<0<<" "<<0<<" "<<0<<" "
                                                            <<0<<" "<<0<<endl;
                    continue;
                }

                if (!GetLeftLane(slz, left_lane))
                {
                    printf("road id %d, seg id %d, local id %d, s %f, l %f\n", slz.lane_id.road_id, slz.lane_id.section_idx, slz.lane_id.local_id, slz.s, slz.l);
                    laneinfo_ofile<<std::setprecision(10)<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "
                                                            <<0<<" "<<0<<" "<<0<<" "
                                                            <<0<<" "<<0<<endl;
                    continue;
                }

                    //保存 曲率  横向偏移距离  左车道存在与否
                laneinfo_ofile<<std::setprecision(10)<<timestamp<<" "<<k<<" "<<dL<<" "<<left_lane<<" "<<cx<<" "<<cy<<" "<<cz<<" "<<chdg<<" "
                                                        <<slz.lane_id.road_id<<" "<<slz.lane_id.section_idx<<" "<<slz.lane_id.local_id<<" "
                                                        <<slz.s<<" "<<slz.l<<endl;   

                continue;                
            }

            printf("road id %d, seg id %d, local id %d, s %f, l %f\n", slz.lane_id.road_id, slz.lane_id.section_idx, slz.lane_id.local_id, slz.s, slz.l);
            laneinfo_ofile<<std::setprecision(10)<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "
                                                    <<0<<" "<<0<<" "<<0<<" "
                                                    <<0<<" "<<0<<endl;
            continue;                
        }
        printf("road id %d, seg id %d, local id %d, s %f, l %f\n", slz.lane_id.road_id, slz.lane_id.section_idx, slz.lane_id.local_id, slz.s, slz.l);
        laneinfo_ofile<<std::setprecision(10)<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "
                                                <<0<<" "<<0<<" "<<0<<" "
                                                <<0<<" "<<0<<endl;
    }
    localization_ifile.close();
    laneinfo_ofile.close();
    printf("exchange finished.\n");
}



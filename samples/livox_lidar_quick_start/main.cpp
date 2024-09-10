//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#include <unistd.h>
#include <arpa/inet.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <deque>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <math.h>
#include <mutex>
#include <iomanip>


#define NUM_SECTORS 8

#define NUM_LAYERS 5

#define MIN_PROX_DIST 0

struct timed_point {
  std::chrono::milliseconds time_ms;
  LivoxLidarSpherPoint data;
};

std::deque<timed_point> data_points[NUM_SECTORS][NUM_LAYERS];

struct {
  uint16_t yaw;
  uint16_t pitch;
  uint16_t roll;
} rotation;

struct {
  uint16_t x;
  uint16_t y;
  uint16_t z;
} translation;

float sin_yaw;
float sin_pitch;
float sin_roll;

float cos_yaw;
float cos_pitch;
float cos_roll;

uint8_t sensor_id;

std::mutex mut[NUM_SECTORS][NUM_LAYERS];

void processPoints() {
  std::chrono::milliseconds now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
  system("clear");
  std::cout << "Distances: " << std::endl;
  std::cout << "          Layer 0  Layer 1  Layer 2  Layer 3  Layer 4" << std::endl;
  for (uint8_t sector = 0; sector < NUM_SECTORS; sector++) {
    std::cout << std::fixed << std::setprecision(4) << "Sector " << std::to_string(sector) << ": ";
    for (uint8_t layer = 0; layer < NUM_LAYERS; layer++) {
      // std::cout << "checking layer: " << std::to_string(layer) << std::endl;
      while (true) {
        if (data_points[sector][layer].empty()) {
          break;
        }
        if (data_points[sector][layer].front().time_ms + (std::chrono::milliseconds)100 < now) {
          mut[sector][layer].lock();
          data_points[sector][layer].pop_front();
          mut[sector][layer].unlock();
        }
        else {
          // All points in queue are newer than the checked point, 
          // so we do not need to check them
          break;
        }
      }
      bool high_confidence = false;
      LivoxLidarSpherPoint closest_point;
      uint32_t closest_point_index = 0;
      closest_point.depth = UINT32_MAX;
      // Find closest point with highest confidence
      // std::cout << "sector: " << std::to_string(sector) << " Layer: " << std::to_string(layer) << " Size: " << std::to_string(data_points[sector][layer].size()) << std::endl;
      for (uint32_t i = 0; i < data_points[sector][layer].size(); i++) {
        timed_point point = data_points[sector][layer][i];
        if (high_confidence){
          if (point.data.tag == 0 && point.data.depth < closest_point.depth && point.data.depth > MIN_PROX_DIST) {
            closest_point = point.data;
            closest_point_index = i;
          }
        }
        else {
          if (point.data.tag == 0 && point.data.depth > MIN_PROX_DIST) {
            high_confidence = true;
            closest_point = point.data;
            closest_point_index = i;
            
          }
          else {
             if (point.data.tag & 0x3 <= 1 && (point.data.tag >> 2) & 0x3 <= 1&& (point.data.tag >> 4) & 0x3 <= 1 && point.data.depth < closest_point.depth && point.data.depth > MIN_PROX_DIST) {
              // Confidence is not worse than medium for any of the 3 tags
              closest_point = point.data;
              closest_point_index = i;
            }
          }
        }
      }
      mut[sector][layer].lock();
      data_points[sector][layer].erase(data_points[sector][layer].begin(), data_points[sector][layer].begin() + closest_point_index);
      mut[sector][layer].unlock();
      
      if (closest_point.depth != UINT32_MAX) {
        std::cout << ((float)closest_point.depth) / 1000 << "m  ";
      }
      else {
        std::cout << "MAX      ";
      }
    }
    std::cout << std::endl;
  }
}


void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
  if (data == nullptr) {
    return;
  }
  // printf("point cloud handle: %u, data_num: %d, data_type: %d, length: %d, frame_counter: %d\n",
  //     handle, data->dot_num, data->data_type, data->length, data->frame_cnt);

  // if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
  //   LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;
  //   for (uint32_t i = 0; i < data->dot_num; i++) {
  //     if (p_point_data[i].x == 0 && p_point_data[i].y == 0 && p_point_data[i].z == 0) {
  //       continue;
  //     }

  //     float transformed_x = (cos_yaw * cos_pitch * (float)p_point_data[i].x) 
  //                           + ((cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll) * (float)p_point_data[i].y) 
  //                           + ((cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll) * (float)p_point_data[i].z)
  //                           + (translation.x * 1);

  //     float transformed_y = (sin_yaw * cos_roll * (float)p_point_data[i].x)
  //                           + ((sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll) * (float)p_point_data[i].y)
  //                           + ((sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll) * (float)p_point_data[i].z)
  //                           + (translation.y * 1);
      
  //     float transformed_z = (-sin_pitch * (float)p_point_data[i].x)
  //                           + (cos_pitch * sin_roll * (float)p_point_data[i].y)
  //                           + (cos_pitch * cos_roll * (float)p_point_data[i].z)
  //                           + (translation.z * 1);

  //     timed_point new_point;
  //     new_point.time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
  //     new_point.data.depth = std::sqrt(std::pow(transformed_x, 2) + std::pow(transformed_y, 2) + std::pow(transformed_z, 2));
  //     new_point.data.theta = (uint16_t) ((std::atan(transformed_y / transformed_x) * 180 / M_PI) * 100);
  //     new_point.data.phi = (uint16_t) ((std::sqrt(std::pow(transformed_x, 2) + std::pow(transformed_y, 2)) / transformed_z) * 100);
  //     new_point.data.tag = p_point_data[i].tag;
  //     new_point.data.reflectivity = p_point_data[i].reflectivity;
  //   }
  // }

  if (data->data_type == kLivoxLidarSphericalCoordinateData) {
    LivoxLidarSpherPoint *p_point_data = (LivoxLidarSpherPoint *)data->data;
    for (uint32_t i = 0; i < data->dot_num; i++) {
      if (p_point_data[i].depth == 0) {
        continue;
      }

      uint8_t layer = 0;
      uint8_t sector = 0;

      sector = (8 - ((p_point_data[i].phi + 2250) / 4500)) % 8;

      layer = (std::min(std::max(p_point_data[i].theta, (uint16_t)1500), (uint16_t)16500) - 1500) / 3000;

      timed_point new_point;
      new_point.time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
      new_point.data = p_point_data[i];

      // if (layer > 0) {
      //   std::cout << std::to_string(layer);
      //   // std::cout << "phi: " << std::to_string(p_point_data[i].phi) << " theta: " << p_point_data[i].theta << std::endl;
      // }
      mut[sector][layer].lock();
      data_points[sector][layer].push_back(new_point);
      mut[sector][layer].unlock();
    }
  }
}

void ImuDataCallback(uint32_t handle, const uint8_t dev_type,  LivoxLidarEthernetPacket* data, void* client_data) {
  if (data == nullptr) {
    return;
  } 
  printf("Imu data callback handle:%u, data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n",
      handle, data->dot_num, data->data_type, data->length, data->frame_cnt);
}

// void OnLidarSetIpCallback(livox_vehicle_status status, uint32_t handle, uint8_t ret_code, void*) {
//   if (status == kVehicleStatusSuccess) {
//     printf("lidar set ip slot: %d, ret_code: %d\n",
//       slot, ret_code);
//   } else if (status == kVehicleStatusTimeout) {
//     printf("lidar set ip number timeout\n");
//   }
// }

void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  // printf("WorkModeCallack, status:%u, handle:%u, ret_code:%u, error_key:%u",
  //    status, handle, response->ret_code, response->error_key);

}

void RebootCallback(livox_status status, uint32_t handle, LivoxLidarRebootResponse* response, void* client_data) {
  if (response == nullptr) {
    return;
  }
  // printf("RebootCallback, status:%u, handle:%u, ret_code:%u",
  //    status, handle, response->ret_code);
}

void SetIpInfoCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  // printf("LivoxLidarIpInfoCallback, status:%u, handle:%u, ret_code:%u, error_key:%u",
  //    status, handle, response->ret_code, response->error_key);

  if (response->ret_code == 0 && response->error_key == 0) {
    LivoxLidarRequestReboot(handle, RebootCallback, nullptr);
  }
}

void QueryInternalInfoCallback(livox_status status, uint32_t handle, 
    LivoxLidarDiagInternalInfoResponse* response, void* client_data) {
  if (status != kLivoxLidarStatusSuccess) {
    printf("Query lidar internal info failed.\n");
    QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
    return;
  }

  if (response == nullptr) {
    return;
  }

  uint8_t host_point_ipaddr[4] {0};
  uint16_t host_point_port = 0;
  uint16_t lidar_point_port = 0;

  uint8_t host_imu_ipaddr[4] {0};
  uint16_t host_imu_data_port = 0;
  uint16_t lidar_imu_data_port = 0;

  uint16_t off = 0;
  for (uint8_t i = 0; i < response->param_num; ++i) {
    LivoxLidarKeyValueParam* kv = (LivoxLidarKeyValueParam*)&response->data[off];
    if (kv->key == kKeyLidarPointDataHostIpCfg) {
      memcpy(host_point_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
      memcpy(&(host_point_port), &(kv->value[4]), sizeof(uint16_t));
      memcpy(&(lidar_point_port), &(kv->value[6]), sizeof(uint16_t));
    } else if (kv->key == kKeyLidarImuHostIpCfg) {
      memcpy(host_imu_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
      memcpy(&(host_imu_data_port), &(kv->value[4]), sizeof(uint16_t));
      memcpy(&(lidar_imu_data_port), &(kv->value[6]), sizeof(uint16_t));
    }
    off += sizeof(uint16_t) * 2;
    off += kv->length;
  }

  // printf("Host point cloud ip addr:%u.%u.%u.%u, host point cloud port:%u, lidar point cloud port:%u.\n",
  //    host_point_ipaddr[0], host_point_ipaddr[1], host_point_ipaddr[2], host_point_ipaddr[3], host_point_port, lidar_point_port);

  // printf("Host imu ip addr:%u.%u.%u.%u, host imu port:%u, lidar imu port:%u.\n",
  //  host_imu_ipaddr[0], host_imu_ipaddr[1], host_imu_ipaddr[2], host_imu_ipaddr[3], host_imu_data_port, lidar_imu_data_port);

}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
  if (info == nullptr) {
    printf("lidar info change callback failed, the info is nullptr.\n");
    return;
  } 
  // printf("LidarInfoChangeCallback Lidar handle: %u SN: %s\n", handle, info->sn);
  
  // set the work mode to kLivoxLidarNormal, namely start the lidar
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);

  QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);

  // LivoxLidarIpInfo lidar_ip_info;
  // strcpy(lidar_ip_info.ip_addr, "192.168.1.10");
  // strcpy(lidar_ip_info.net_mask, "255.255.255.0");
  // strcpy(lidar_ip_info.gw_addr, "192.168.1.1");
  // SetLivoxLidarLidarIp(handle, &lidar_ip_info, SetIpInfoCallback, nullptr);
}

void LivoxLidarPushMsgCallback(const uint32_t handle, const uint8_t dev_type, const char* info, void* client_data) {
  struct in_addr tmp_addr;
  tmp_addr.s_addr = handle;  
  // std::cout << "handle: " << handle << ", ip: " << inet_ntoa(tmp_addr) << ", push msg info: " << std::endl;
  std::cout << info << std::endl;
  return;
}

int main(int argc, const char *argv[]) {
  if (argc != 2) {
    printf("Params Invalid, must input config path.\n");
    return -1;
  }
  const std::string path = argv[1];

  // REQUIRED, to init Livox SDK2
  if (!LivoxLidarSdkInit(path.c_str())) {
    printf("Livox Init Failed\n");
    LivoxLidarSdkUninit();
    return -1;
  }
  
  // REQUIRED, to get point cloud data via 'PointCloudCallback'
  SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
  
  // OPTIONAL, to get imu data via 'ImuDataCallback'
  // some lidar types DO NOT contain an imu component
  // SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);
  
  SetLivoxLidarInfoCallback(LivoxLidarPushMsgCallback, nullptr);
  
  // REQUIRED, to get a handle to targeted lidar and set its work mode to NORMAL
  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

  while(true) {
    processPoints();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  LivoxLidarSdkUninit();
  printf("Livox Quick Start Demo End!\n");
  return 0;
}

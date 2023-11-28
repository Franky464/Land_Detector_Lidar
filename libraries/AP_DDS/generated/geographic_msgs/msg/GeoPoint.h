// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*!
 * @file GeoPoint.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _GeoPoint_H_
#define _GeoPoint_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

typedef struct geographic_msgs_msg_GeoPoint
{
    double latitude;
    double longitude;
    double altitude;
} geographic_msgs_msg_GeoPoint;

struct ucdrBuffer;

bool geographic_msgs_msg_GeoPoint_serialize_topic(struct ucdrBuffer* writer, const geographic_msgs_msg_GeoPoint* topic);
bool geographic_msgs_msg_GeoPoint_deserialize_topic(struct ucdrBuffer* reader, geographic_msgs_msg_GeoPoint* topic);
uint32_t geographic_msgs_msg_GeoPoint_size_of_topic(const geographic_msgs_msg_GeoPoint* topic, uint32_t size);


#ifdef __cplusplus
}
#endif

#endif // _GeoPoint_H_
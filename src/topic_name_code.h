// Copyright 2018 Billy
#ifndef SRC_TOPIC_NAME_CODE_H_
#define SRC_TOPIC_NAME_CODE_H_
#include <map>
#include <string>
using std::string;
using std::map;

enum StringCode { eImu, eEncoderLeft, eEncoderRight, eGnss };

struct TopicKeyMapper {
    TopicKeyMapper(const string& imu_topic, const string& encoder_left_topic,
                   const string& encoder_right_topic,
                   const string& gnss_topic) {
        s_mapStringToStringCode_ = {{imu_topic, eImu},
                                    {encoder_left_topic, eEncoderLeft},
                                    {encoder_right_topic, eEncoderRight},
                                    {gnss_topic, eGnss}};
    }
    map<string, StringCode> s_mapStringToStringCode_;
};

#endif  // SRC_TOPIC_NAME_CODE_H_

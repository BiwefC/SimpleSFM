#ifndef __LOOP_HPP__
#define __LOOP_HPP__

#include "SLAMBase.hpp"

class Loop
{
public:
    Loop(void);

    // 往数据库里增加一条frame记录
    void add(Frame::Ptr& frame);
    // 获取可能的loops
    vector<Frame::Ptr> getPossibleLoops(Frame::Ptr& frame);


protected:
    DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>    vocab;     //字典文件
    vector<Frame::Ptr>      frames;
    float   min_sim_score;
    float   min_interval;
};

#endif
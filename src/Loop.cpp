#include "Loop.hpp"

Loop::Loop(void)
{
    string  vocab_file = "../ORBvoc.txt";
    cout<<"loading vocabulary file, this may take a while..."<<endl;
    vocab.load(vocab_file);
    cout<<"load ok."<<endl;

    min_sim_score = 0.015;
    min_interval = 60;
}

void Loop::add(Frame::Ptr& frame)
{
    vector<cv::Mat> desps = frame->DescriptorVector();
    DBoW2::FeatureVector featVec;
    vocab.transform(desps, frame->bowVec, featVec, 4);
    frames.push_back(frame);
}

vector<Frame::Ptr> Loop::getPossibleLoops(Frame::Ptr& frame)
{
    vector<Frame::Ptr>  result;
    for (size_t i=0; i<frames.size(); i++)
    {
        Frame::Ptr pf = frames[i];
        double  score = vocab.score(frame->bowVec, pf->bowVec);
        if (score > min_sim_score && abs(pf->frameID - frame->frameID) > min_interval )
        {
            result.push_back(pf);
        }
    }
    return result;
}
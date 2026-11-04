#include "AimedDebouncer.h"

#include <algorithm>

AimedDebouncer::AimedDebouncer(unsigned int requiredSamples)
    : requiredSamples_(std::max(1U, requiredSamples)),
      candidateSamples_(0),
      stableAimed_(false),
      candidateAimed_(false)
{
}

bool AimedDebouncer::update(bool aimed)
{
    if (aimed == stableAimed_)
    {
        candidateAimed_ = aimed;
        candidateSamples_ = 0;
        return stableAimed_;
    }

    if (aimed != candidateAimed_)
    {
        candidateAimed_ = aimed;
        candidateSamples_ = 1;
    }
    else
    {
        ++candidateSamples_;
    }

    if (candidateSamples_ >= requiredSamples_)
    {
        stableAimed_ = candidateAimed_;
        candidateSamples_ = 0;
    }

    return stableAimed_;
}

void AimedDebouncer::reset(bool aimed)
{
    stableAimed_ = aimed;
    candidateAimed_ = aimed;
    candidateSamples_ = 0;
}

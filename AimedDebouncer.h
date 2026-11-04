#ifndef AIMED_DEBOUNCER_H_
#define AIMED_DEBOUNCER_H_

class AimedDebouncer
{
public:
    explicit AimedDebouncer(unsigned int requiredSamples = 3);

    bool update(bool aimed);
    void reset(bool aimed = false);

private:
    unsigned int requiredSamples_;
    unsigned int candidateSamples_;
    bool stableAimed_;
    bool candidateAimed_;
};

#endif // AIMED_DEBOUNCER_H_

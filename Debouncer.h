#ifndef DEBOUNCER_H_
#define DEBOUNCER_H_

class Debouncer
{
public:
    explicit Debouncer(unsigned int requiredSamples = 8);

    bool update(bool aimed);
    void reset(bool aimed = false);

private:
    unsigned int requiredSamples_;
    unsigned int candidateSamples_;
    bool stableAimed_;
    bool candidateAimed_;
};

#endif // AIMED_DEBOUNCER_H_

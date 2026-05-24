#pragma once
#include <cstddef>
#include <stdexcept>

namespace mpc {

class MpcLayout {
public:
    explicit MpcLayout(std::size_t horizon = 20, double dt = 0.1, double lf = 0.5)
        : _N(horizon), _dt(dt), _Lf(lf) {
        if (_N < 2) {
            throw std::invalid_argument("N must be >= 2");
        }
        if (_dt <= 0.0 || _Lf <= 0.0) {
            throw std::invalid_argument("dt and Lf must be > 0");
        }
    }

    std::size_t N() const { return _N; }
    double dt() const { return _dt; }
    double Lf() const { return _Lf; }

    std::size_t stateSize() const { return 4 * _N; }
    std::size_t controlSize() const { return 2 * (_N - 1); }
    std::size_t totalSize() const { return stateSize() + controlSize(); }

    std::size_t xStart() const { return 0; }
    std::size_t yStart() const { return _N; }
    std::size_t psiStart() const { return 2 * _N; }
    std::size_t vStart() const { return 3 * _N; }
    std::size_t deltaStart() const { return 4 * _N; }
    std::size_t aStart() const { return 5 * _N - 1; }

    std::size_t idxX(std::size_t t) const { return xStart() + t; }
    std::size_t idxY(std::size_t t) const { return yStart() + t; }
    std::size_t idxPsi(std::size_t t) const { return psiStart() + t; }
    std::size_t idxV(std::size_t t) const { return vStart() + t; }
    std::size_t idxDelta(std::size_t t) const { return deltaStart() + t; }
    std::size_t idxA(std::size_t t) const { return aStart() + t; }

private:
    std::size_t _N;
    double _dt;
    double _Lf;
};

} // namespace mpc

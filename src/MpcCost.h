#pragma once
#include <dense/Matrix.h>
#include "MpcLayout.h"

namespace mpc {

class MpcCost {
public:
    explicit MpcCost(const MpcLayout& layout)
        : _layout(layout),
          Q(4, 1, nullptr, true),
          R(2, 1, nullptr, true) {
        auto q = Q.getColumnManipulator();
        q(0) = 1.0; // x
        q(1) = 1.0; // y
        q(2) = 1.0; // psi
        q(3) = 1.0; // v

        auto r = R.getColumnManipulator();
        r(0) = 1.0; // delta
        r(1) = 1.0; // a
    }

    const MpcLayout& layout() const { return _layout; }

    dense::DblMatrix Q; // state weights (placeholder)
    dense::DblMatrix R; // control weights (placeholder)

private:
    MpcLayout _layout;
};

} // namespace mpc

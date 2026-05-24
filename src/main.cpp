#include <mu/Application.h>
#include <dense/Matrix.h>
#include <sparse/IMatrix.h>
#include <sparse/Format.h>
#include <iostream>
#include "MpcSolverStub.h"

static void testSparseMatrixCreate() {
    std::cout << "Sparse test: createDblMatrix" << std::endl;
    sparse::IDblMatrix* pMatrix = sparse::createDblMatrix(5, 4, 10);
    if (!pMatrix) {
        std::cout << "Sparse test: createDblMatrix returned null" << std::endl;
        return;
    }

    pMatrix->addTriple(0, 0, 1);
    pMatrix->addTriple(1, 1, 11);
    pMatrix->addTriple(2, 2, 22);
    pMatrix->addTriple(3, 3, 33);
    pMatrix->addTriple(1, 0, 10);
    pMatrix->addTriple(1, 2, 12);
    pMatrix->addTriple(2, 1, 21);
    pMatrix->addTriple(2, 3, 23);
    pMatrix->addTriple(3, 1, 31);
    pMatrix->addTriple(4, 1, 41);

    pMatrix->serialize("H", std::cout, sparse::Format::Cpp);
    std::cout << "Sparse test: nnz=" << pMatrix->getNoOfNonZero() << std::endl;
    pMatrix->release();
    std::cout << "Sparse test: done" << std::endl;
}

int main(int argc, const char* argv[]) {
    mu::Application app(argc, argv);

    std::cout << "MpcCore start" << std::endl;

    testSparseMatrixCreate();

    dense::DblMatrix initial(4, 1, nullptr, true);
    auto init = initial.getColumnManipulator();
    init(0) = 0.0; // x
    init(1) = 0.0; // y
    init(2) = 0.0; // psi
    init(3) = 1.0; // v

    mpc::MpcSolverStub solver;
    dense::DblMatrix trajectory;
    solver.Solve(initial, trajectory);

    std::cout << "MpcCore end" << std::endl;

    return 0;
}

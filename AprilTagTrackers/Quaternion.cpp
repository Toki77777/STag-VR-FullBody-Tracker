//****************************************************
//* quaternion.c++                                   *
//*                                                  *
//* Implementaion for a generalized quaternion class *
//*                                                  *
//* Written 1.25.00 by Angela Bennett                *
//****************************************************

#include "Quaternion.hpp"

#include "utils/Test.hpp"

#include <cmath>

TEST_CASE("Quaternion inverse produces the multiplicative identity")
{
    Quaternion<double> quaternion{2.0, -1.0, 3.0, 0.5};
    const Quaternion<double> product = quaternion * quaternion.inverse();

    CHECK(product.w == doctest::Approx(1.0).epsilon(1e-12));
    CHECK(std::abs(product.x) < 1e-12);
    CHECK(std::abs(product.y) < 1e-12);
    CHECK(std::abs(product.z) < 1e-12);
}

TEST_CASE("Quaternion rotates a vector by a known angle")
{
    const double halfSqrtTwo = std::sqrt(0.5);
    Quaternion<double> rotation{halfSqrtTwo, 0.0, 0.0, halfSqrtTwo};
    double vector[3]{1.0, 0.0, 0.0};

    rotation.QuatRotation(vector);

    CHECK(std::abs(vector[0]) < 1e-12);
    CHECK(vector[1] == doctest::Approx(1.0).epsilon(1e-12));
    CHECK(std::abs(vector[2]) < 1e-12);
}

#ifndef INVKIN_HPP_
#define INVKIN_HPP_

#include <eeros/math/Matrix.hpp>
#include <eeros/control/Blockio.hpp>

using namespace eeros::control;

class InvKin : public Block
{
public:

    InvKin(double B)
        : B(B),
          WJR([&, B]()
              {
              eeros::math::Matrix<2,2> M = {1.0, 1.0, -B/2.0, B/2.0};
              eeros::math::Vector2 v = {WJR.getIn(0).getSignal().getValue(), WJR.getIn(1).getSignal().getValue()};
              WJR.getOut().getSignal().setValue(M*v);
              WJR.getOut().getSignal().setTimestamp(WJR.getIn(0).getSignal().getTimestamp()); })
    {
        // Name all blocks
        WJR.setName("InvKin->WJR");

        // Name all signals
        WJR.getOut().getSignal().setName("vW [m/s]");
    }

    Input<> &getInRvRx() { return WJR.getIn(0); }

    Input<> &getInOmegaR() { return WJR.getIn(1); }

    Output<eeros::math::Vector2> &getOut() { return WJR.getOut(); }

    virtual void run()
    {
        WJR.run();
    }

protected:
    double B;
    Blockio<2, 1, double, eeros::math::Vector2> WJR;
};

#endif // INVKIN_HPP_
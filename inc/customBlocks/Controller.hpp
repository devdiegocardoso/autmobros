#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/InputSub.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/I.hpp>

using namespace eeros::control;

template <typename T = double>
class Controller : public Block
{
public:

    Controller(double om0, double D, double M, T eLimit)
        : qd(this),
          KP(2.0 * D * om0),
          KI(om0 * om0),
          M(M)
    {
        init(eLimit);
    }

    Controller(double fTask, double D, double s, double M, T eLimit)
        : qd(this),
          KP(fTask / s),
          KI(fTask / 2.0 / s / D * fTask / 2.0 / s / D),
          M(M)
    {
        init(eLimit);
    }

    virtual Input<T> &getIn(uint8_t index)
    {
        if (index == 0)
        {
            return ed.getIn(0);
        }
        else if (index == 1)
        {
            return qd;
        }
        else
        {
            throw IndexOutOfBoundsFault("Trying to get inexistent element of input vector in block '" + this->getName() + "'");
        }
    }

    virtual Output<T> &getOut(uint8_t index)
    {
        if (index == 0)
        {
            return M.getOut();
        }
        else if (index == 1)
        {
            return qd;
        }
        else
        {
            throw IndexOutOfBoundsFault("Trying to get inexistent element of input vector in block '" + this->getName() + "'");
        }
    }

    virtual void run()
    {
        ed.run();
        KP.run();
        e.run();
        KI.run();
        qddC.run();
        M.run();
    }

    void enable()
    {
        e.enable();
    }

    void disable()
    {
        e.disable();
    }

    void setELimit(T eLimit)
    {
        e.setLimit(eLimit, -eLimit);
    }

protected:
    InputSub<T> qd;
    Sum<2, T> ed, qddC;
    Gain<T> KP, KI, M;
    I<T> e;

private:

    void init(T eLimit)
    {
        // Name all blocks
        ed.setName("controller->ed");
        KP.setName("controller->KP");
        e.setName("controller->e");
        KI.setName("controller->KI");
        qddC.setName("controller->qddC");
        M.setName("controller->M");

        // Name all signals
        ed.getOut().getSignal().setName("ed [rad/s]");
        KP.getOut().getSignal().setName("qddCP [rad/s^2]");
        e.getOut().getSignal().setName("e [rad]");
        KI.getOut().getSignal().setName("qddCI [rad/s^2]");
        qddC.getOut().getSignal().setName("qddC [rad/s^2]");
        M.getOut().getSignal().setName("Q [Nm]");

        // Connect signals
        ed.getIn(1).connect(qd);
        ed.negateInput(1);
        KP.getIn().connect(ed.getOut());
        e.getIn().connect(ed.getOut());
        KI.getIn().connect(e.getOut());
        qddC.getIn(0).connect(KP.getOut());
        qddC.getIn(1).connect(KI.getOut());
        M.getIn().connect(qddC.getOut());

        // Additional configuration
        T eInit = 0.0;
        e.setInitCondition(eInit);
        e.setLimit(eLimit, -eLimit);
    }
};

#endif // CONTROLLER_HPP_
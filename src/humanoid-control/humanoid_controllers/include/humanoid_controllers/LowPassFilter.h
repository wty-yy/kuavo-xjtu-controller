#pragma once
#include <Eigen/Dense>
#include <iostream>
class LowPassFilter
{
public:
    LowPassFilter()
    {
    }
    LowPassFilter(const Eigen::VectorXd &dt, const Eigen::VectorXd &cutoff_freq)
    {
        setParams(dt, cutoff_freq);
    }
    void setParams(double dt, const Eigen::VectorXd &cutoff_freq)
    {
        Eigen::VectorXd dt_s = Eigen::VectorXd::Ones(cutoff_freq.size()) * dt;
        setParams(dt_s, cutoff_freq);
    }
    void setParams(const Eigen::VectorXd &dt, const Eigen::VectorXd &cutoff_freq)
    {
        alpha_ = Eigen::VectorXd::Zero(dt.size());
        y_prev_ = Eigen::VectorXd::Zero(dt.size());
        y_ = Eigen::VectorXd::Zero(dt.size());

        for (int i = 0; i < dt.size(); ++i)
        {
            double RC = 1.0 / (cutoff_freq(i) * 2 * M_PI);
            alpha_(i) = dt(i) / (RC + dt(i));
        }
    }

    Eigen::VectorXd update(const Eigen::VectorXd &input)
    {
        for (int i = 0; i < input.size(); ++i)
        {
            y_(i) = alpha_(i) * input(i) + (1.0 - alpha_(i)) * y_prev_(i);
            y_prev_(i) = y_(i);
        }
        return y_;
    }

private:
    Eigen::VectorXd alpha_;
    Eigen::VectorXd y_prev_;
    Eigen::VectorXd y_;
};

class LowPassFilter2ndOrder
{
public:
    LowPassFilter2ndOrder() {}

    LowPassFilter2ndOrder(double dt, const Eigen::VectorXd &cutoff_freq)
    {
        setParams(dt, cutoff_freq);
    }

    void setParams(double dt, const Eigen::VectorXd &cutoff_freq)
    {
        double sample_freq = 1.0 / dt;
        if (sample_freq <= 0.0 || cutoff_freq.minCoeff() <= 0.0 || cutoff_freq.maxCoeff() >= sample_freq / 2.0)
        {
            disable();
            return;
        }

        _sample_freq = sample_freq;
        _cutoff_freq = cutoff_freq;

        _delay_element_1 = Eigen::VectorXd::Zero(cutoff_freq.size());
        _delay_element_2 = Eigen::VectorXd::Zero(cutoff_freq.size());

        _b0 = Eigen::VectorXd::Zero(cutoff_freq.size());
        _b1 = Eigen::VectorXd::Zero(cutoff_freq.size());
        _b2 = Eigen::VectorXd::Zero(cutoff_freq.size());
        _a1 = Eigen::VectorXd::Zero(cutoff_freq.size());
        _a2 = Eigen::VectorXd::Zero(cutoff_freq.size());

        for (int i = 0; i < cutoff_freq.size(); ++i)
        {
            double fr = sample_freq / cutoff_freq(i);
            double ohm = tan(M_PI / fr);
            double c = 1.0 + 2.0 * cos(M_PI / 4.0) * ohm + ohm * ohm;

            _b0(i) = ohm * ohm / c;
            _b1(i) = 2.0 * _b0(i);
            _b2(i) = _b0(i);

            _a1(i) = 2.0 * (ohm * ohm - 1.0) / c;
            _a2(i) = (1.0 - 2.0 * cos(M_PI / 4.0) * ohm + ohm * ohm) / c;

            if (!std::isfinite(_b0(i)) || !std::isfinite(_b1(i)) || !std::isfinite(_b2(i)) ||
                !std::isfinite(_a1(i)) || !std::isfinite(_a2(i)))
            {
                std::cout << "b0: " << _b0(i) << " b1: " << _b1(i) << " b2: " << _b2(i) << " a1: " << _a1(i) << " a2: " << _a2(i) << std::endl;
                disable();
                return;
            }
        }
    }

    Eigen::VectorXd update(const Eigen::VectorXd &input)
    {
        Eigen::VectorXd output(input.size());

        for (int i = 0; i < input.size(); ++i)
        {
            double delay_element_0 = input(i) - _delay_element_1(i) * _a1(i) - _delay_element_2(i) * _a2(i);
            output(i) = delay_element_0 * _b0(i) + _delay_element_1(i) * _b1(i) + _delay_element_2(i) * _b2(i);
            _delay_element_2(i) = _delay_element_1(i);
            _delay_element_1(i) = delay_element_0;
        }

        return output;
    }

    void reset()
    {
        _delay_element_1.setZero();
        _delay_element_2.setZero();
    }

    void disable()
    {
        std::cout << "LowPassFilter2ndOrder: Invalid parameters, disabling filter." << std::endl;
        _sample_freq = 0.0;
        _cutoff_freq.setZero();

        _b0.setOnes();
        _b1.setZero();
        _b2.setZero();

        _a1.setZero();
        _a2.setZero();

        _delay_element_1.setZero();
        _delay_element_2.setZero();
    }

private:
    double _sample_freq = 0.0;
    Eigen::VectorXd _cutoff_freq;

    Eigen::VectorXd _b0;
    Eigen::VectorXd _b1;
    Eigen::VectorXd _b2;
    Eigen::VectorXd _a1;
    Eigen::VectorXd _a2;

    Eigen::VectorXd _delay_element_1;
    Eigen::VectorXd _delay_element_2;
};

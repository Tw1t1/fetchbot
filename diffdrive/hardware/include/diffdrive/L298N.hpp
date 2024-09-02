#ifndef L298N_HPP
#define L298N_HPP

#include <gpiod.hpp>
#include <thread>
#include <atomic>
#include <stdexcept>
#include <algorithm>

class L298N {
private:
    gpiod::chip chip;
    gpiod::line en;
    gpiod::line in1;
    gpiod::line in2;
    int defaultDutyCycle;
    std::atomic<int> dutyCycleValue;
    std::thread pwmThread;
    std::atomic<bool> running;
    std::atomic<bool> initialized;

    void pwmFunction() {
        while (running) {
            if (en.is_requested()) {
                en.set_value(1);
                std::this_thread::sleep_for(std::chrono::microseconds(dutyCycleValue * 100));
                en.set_value(0);
                std::this_thread::sleep_for(std::chrono::microseconds((100 - dutyCycleValue) * 100));
            }
        }
    }

public:
    L298N() : defaultDutyCycle(30), dutyCycleValue(30), running(false), initialized(false) {}

    ~L298N() {
        if (initialized) {
            running = false;
            if (pwmThread.joinable()) {
                pwmThread.join();
            }
        }
    }

    void initialize(int enPin, int in1Pin, int in2Pin, int defaultDutyCycle = 30) {
        if (initialized) {
            throw std::runtime_error("Motor already initialized");
        }

        chip = gpiod::chip("gpiochip0");
        en = chip.get_line(enPin);
        in1 = chip.get_line(in1Pin);
        in2 = chip.get_line(in2Pin);

        en.request({"L298N", gpiod::line_request::DIRECTION_OUTPUT, 0});
        in1.request({"L298N", gpiod::line_request::DIRECTION_OUTPUT, 0});
        in2.request({"L298N", gpiod::line_request::DIRECTION_OUTPUT, 0});

        this->defaultDutyCycle = std::max(0, std::min(defaultDutyCycle, 100));
        this->dutyCycleValue = this->defaultDutyCycle;

        running = true;
        pwmThread = std::thread(&L298N::pwmFunction, this);

        initialized = true;
    }

    void forward() {
        if (!initialized) {
            throw std::runtime_error("Motor not initialized");
        }
        in1.set_value(0);
        in2.set_value(1);
    }

    void backward() {
        if (!initialized) {
            throw std::runtime_error("Motor not initialized");
        }
        in1.set_value(1);
        in2.set_value(0);
    }

    void set_duty_cycle(int dc) {
        if (!initialized) {
            throw std::runtime_error("Motor not initialized");
        }
        dutyCycleValue = std::max(0, std::min(dc, 100));
    }

    void setVelocity(double vel){
        if (vel > 0) {
            set_duty_cycle(static_cast<int>(vel));
            forward();
        } else if (vel < 0) {
            set_duty_cycle(static_cast<int>(-vel));
            backward();
        } else {
            stop();
        }
    }

    void stop() {
        if (!initialized) {
            throw std::runtime_error("Motor not initialized");
        }
        in1.set_value(0);
        in2.set_value(0);
    }
};

#endif // L298N_HPP
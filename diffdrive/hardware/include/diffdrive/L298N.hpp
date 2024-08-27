#ifndef L298N_HPP
#define L298N_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include <stdexcept>
#include <algorithm>

class L298N {
private:
    int en;  // represent ENA or ENB pins
    int in1; // represent IN1 or IN3 pins
    int in2; // represent IN2 or IN4 pins
    int defaultDutyCycle;
    std::atomic<int> dutyCycleValue;
    std::thread pwmThread;
    std::atomic<bool> running;
    std::atomic<bool> initialized;

    void exportPin(int pin) {
        std::ofstream exportFile("/sys/class/gpio/export");
        exportFile << pin;
        exportFile.close();
    }

    void unexportPin(int pin) {
        std::ofstream unexportFile("/sys/class/gpio/unexport");
        unexportFile << pin;
        unexportFile.close();
    }

    void setDirection(int pin, const std::string& direction) {
        std::ofstream directionFile("/sys/class/gpio/gpio" + std::to_string(pin) + "/direction");
        directionFile << direction;
        directionFile.close();
    }

    void setValue(int pin, int value) {
        std::ofstream valueFile("/sys/class/gpio/gpio" + std::to_string(pin) + "/value");
        valueFile << value;
        valueFile.close();
    }

    void pwmFunction() {
        while (running) {
            if (en != -1) {
                setValue(en, 1);
                std::this_thread::sleep_for(std::chrono::microseconds(dutyCycleValue * 100));
                setValue(en, 0);
                std::this_thread::sleep_for(std::chrono::microseconds((100 - dutyCycleValue) * 100));
            }
        }
    }

public:
    L298N() : en(-1), in1(-1), in2(-1), defaultDutyCycle(30), dutyCycleValue(30), running(false), initialized(false) {}

    L298N(int en, int in1, int in2, int defaultDutyCycle = 30) : running(false), initialized(false) {
        initialize(en, in1, in2, defaultDutyCycle);
    }

    ~L298N() {
        if (initialized) {
            running = false;
            if (pwmThread.joinable()) {
                pwmThread.join();
            }
            if (in1 != -1) unexportPin(in1);
            if (in2 != -1) unexportPin(in2);
            if (en != -1) unexportPin(en);
        }
    }

    void initialize(int en, int in1, int in2, int defaultDutyCycle = 30) {
        if (initialized) {
            throw std::runtime_error("Motor already initialized");
        }

        this->en = en;
        this->in1 = in1;
        this->in2 = in2;
        this->defaultDutyCycle = std::max(0, std::min(defaultDutyCycle, 100));
        this->dutyCycleValue = this->defaultDutyCycle;

        if (in1 != -1) {
            exportPin(in1);
            setDirection(in1, "out");
        }
        if (in2 != -1) {
            exportPin(in2);
            setDirection(in2, "out");
        }
        if (en != -1) {
            exportPin(en);
            setDirection(en, "out");
            running = true;
            pwmThread = std::thread(&L298N::pwmFunction, this);
        }

        setValue(in1, 0);
        setValue(in2, 0);

        initialized = true;
    }

    void forward() {
        if (!initialized) {
            throw std::runtime_error("Motor not initialized");
        }
        setValue(in1, 0);
        setValue(in2, 1);
    }

    void backward() {
        if (!initialized) {
            throw std::runtime_error("Motor not initialized");
        }
        setValue(in1, 1);
        setValue(in2, 0);
    }

    void set_duty_cycle(int dc) {
        if (!initialized) {
            throw std::runtime_error("Motor not initialized");
        }
        dutyCycleValue = std::max(0, std::min(dc, 100));
    }

    void stop() {
        if (!initialized) {
            throw std::runtime_error("Motor not initialized");
        }
        setValue(in1, 0);
        setValue(in2, 0);
    }

    void setVelocity(double vel){
        if (vel > 0) {
            forward();
            set_duty_cycle(static_cast<int>(vel));
        } else if (vel < 0) {
            backward();
            set_duty_cycle(static_cast<int>(-vel));
        } else {
            stop();
        }
    }
};

#endif // L298N_HPP
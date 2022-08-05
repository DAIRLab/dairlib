#pragma once

enum BLEND_FUNC { SIGMOID, EXP };

double blend_sigmoid(double t, double tau, double window);

double blend_exp(double t, double tau, double window);
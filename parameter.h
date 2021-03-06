#pragma once
#include <vector>
#include <string>

using namespace std;

struct ConfigTrace {
	double x;
	double y;
	ConfigTrace(double x, double y) :x(x), y(y) {}
};
struct Measurements {
	double x;
	double y;
	double v;
	double phi;
	unsigned long t;
	Measurements(double x, double y, double v, double phi, unsigned long t): x(x), y(y), v(v), phi(phi), t(t) {};
};

 class Parameter
{
	
public:
	void setParameter(string fileNameConfig, string fileNameMeasurements);
	vector<ConfigTrace>getVConfigTrace(void);
	vector<Measurements>getVMeasurements(void);
private:
	vector<ConfigTrace>vConfigTrace;
	vector<Measurements>vMeasurements;
};
#pragma once
#include "parameter.h"
#include <string>
#include <fstream>
#include <iostream>
#include <exception>
vector<ConfigTrace>Parameter::getVConfigTrace() {
	return Parameter::vConfigTrace;
};
vector<Measurements>Parameter::getVMeasurements() {
	return Parameter::vMeasurements;
};

void Parameter::setParameter(string fileNameConfig, string fileNameMeasurements)
{
	double x, y;

	try {
		ifstream fin(fileNameConfig);
		if (fin.is_open())
			{
				while (fin >> x >> y) {
					vConfigTrace.push_back(ConfigTrace(x, y));
				}
				fin.close();
			}
			else
			{
				cout << "File " << fileNameConfig << " not found" << endl;
			}
	}catch(exception &e)
	{
		cout << "File " << fileNameConfig << " not opened" << endl;
	}

	double v, phi;
	unsigned long t;

	ifstream fin1(fileNameMeasurements);
	try
	{
	if (fin1.is_open())
	{
		while (fin1 >> x >> y >> v >> phi >> t) {
			vMeasurements.push_back(Measurements(x, y, v, phi, t));
		}
		fin1.close();
	}
	else
	{
		cout << "File " << fileNameMeasurements << " not found" << endl;
	}
	}catch (exception& e)
	{
	cout << "File " << fileNameMeasurements << " not opened" << endl;
	}
};
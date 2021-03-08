// ARRIVAL_TEST.cpp : Этот файл содержит функцию "main". Здесь начинается и заканчивается выполнение программы.
//
#pragma once
#include <iostream>
#include "projection.h"
#include "parameter.h"

int main(int arg, char* argv[])
{
	std::cout << "Hello, ARRIVAL!\n";
	vector<ProjectionDot> projection;
	Parameter param;
	param.setParameter("configTrace.txt", "measurements.txt");

	Projection p(&param);
	try {
		projection = p.getProjection();

		if (!projection.empty())
		{
			cout << "==================Projection: =================" << endl;
			for (unsigned int i = 0; i < projection.size(); i++)
			{
				cout << projection[i].xy.x << " | " << projection[i].xy.y << endl;
			}
		}
	}
	catch (exception& e)
	{
		cout << "Exception of work for the Projection" << endl;
	}

	system("pause");
};
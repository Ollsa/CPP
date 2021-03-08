#include "parameter.h"

#define M_PI			3.14159265358979323846
#define LEN_INTERVAL	1
#define EPSILON			0.000000001

//������������ ���������� ��������
#define FILTER_COOF_LEN		20.0		// ����������� ���������� ����� �������� ����� �� �������
#define FILTER_COOF_SEG_NUM	1			// ����������� ����������� ������ ��������
#define FILTER_COOF_ANGLE	30.0		// ����������� ���������� ������� �� �������� ������ � ��������

struct CoordinatsDecartDot
{
	double x;
	double y;
};

struct CoordinatsDecartSection
{
	CoordinatsDecartDot beg;
	CoordinatsDecartDot end;
};

struct ProjectionDot
{
	CoordinatsDecartDot xy;
	double l;
	double alpha;
	unsigned int numSegment;
};

struct Straight //������ � ���� ������ ��������� Ax + By + C = 0
{
	double A;
	double B;
	double C;
};

class Projection
{
private:
	vector<ConfigTrace> configTraces;
	vector<Measurements> m;

public:
	Projection(Parameter *param);
	Projection();
	vector<ProjectionDot> allProjectionForMeasurement(vector<ConfigTrace>* trace, unsigned int currentIndex, Measurements* m);
	vector<ProjectionDot> filterProjection(vector<ProjectionDot>* prForM);
	vector<ProjectionDot>getProjection();
};
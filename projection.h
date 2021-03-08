#include "parameter.h"

#define M_PI			3.14159265358979323846
#define LEN_INTERVAL	1
#define EPSILON			0.000000001

//������������ ���������� ��������
#define FILTER_COOF_LEN		20		// ����������� ���������� ����� �������� ����� �� �������
#define FILTER_COOF_SEG_NUM	1		// ����������� ����������� ������ ��������
#define FILTER_COOF_ANGLE	30		// ����������� ���������� ������� �� �������� ������ � ��������

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

class Projection
{
private:
	vector<ConfigTrace> configTraces;
	vector<Measurements> m;

public:
	Projection(Parameter *param);
	vector<ProjectionDot> allProjectionForMeasurement(vector<ConfigTrace>* trace, unsigned int currentIndex, Measurements* m);
	vector<ProjectionDot> filterProjection(vector<ProjectionDot>* prForM);
	vector<ProjectionDot>getProjection();
};
#include "pch.h"
#include "../ARRIVAL_TEST/parameter.h"
#include "../ARRIVAL_TEST/projection.h"

TEST(TestCaseName, TestName) {
  EXPECT_EQ(1, 1);
  EXPECT_TRUE(true);
}

TEST(TEST_PARAM, TestFailFile) {
	testing::internal::FilePath testName_path;
	Parameter param;
	param.setParameter(testName_path.string(), testName_path.string());
	EXPECT_TRUE(param.getVConfigTrace().empty());
	EXPECT_TRUE(param.getVMeasurements().empty());
}

TEST(TEST_PROJECTION, TestVoidTrace) {
	Projection p;
	vector<ConfigTrace> cTrace = { ConfigTrace(0.0,0.0) };
	Measurements m = Measurements(0.0, 0.0, 5, 90, 5);
	EXPECT_TRUE(p.allProjectionForMeasurement(&cTrace, 0, &m).empty());
}

TEST(TEST_PROJECTION, TestUnidirectionalProjection) {
	Projection p;
	vector<ConfigTrace> cTrace = { ConfigTrace(0.0,0.0),
								   ConfigTrace(10.0,10.0) };
	Measurements m = Measurements(0.0, 0.0, 5, 45, 5);
	vector<ProjectionDot> resP = p.allProjectionForMeasurement(&cTrace, 0, &m);
	EXPECT_FALSE(resP.empty());
	EXPECT_EQ(resP.size(), 3);

	// Совпадает с начальной точкой
	EXPECT_EQ(resP[0].xy.x, 0);
	EXPECT_EQ(resP[0].xy.y, 0);
	EXPECT_EQ(resP[0].l, 0);
	EXPECT_EQ(resP[0].numSegment, 0);
	EXPECT_EQ(resP[0].alpha, 0);
}

TEST(TEST_PROJECTION, Test45Projection) {
	Projection p;
	vector<ConfigTrace> cTrace = { ConfigTrace(0.0,0.0),
								   ConfigTrace(10.0,10.0) };
	Measurements m = Measurements(0.0, 0.0, 5, 90, 5);
	vector<ProjectionDot> resP = p.allProjectionForMeasurement(&cTrace, 0, &m);
	EXPECT_FALSE(resP.empty());
	EXPECT_EQ(resP.size(), 4);

	EXPECT_EQ(resP[0].xy.x, 0);
	EXPECT_EQ(resP[0].xy.y, 0);
	EXPECT_EQ(resP[0].l, 0);
	EXPECT_EQ(resP[0].numSegment, 0);
	EXPECT_EQ(resP[0].alpha, 45.0);

	EXPECT_EQ(resP[1].xy.x, 2.5000000000000004);
	EXPECT_EQ(resP[1].xy.y, 2.5000000000000004);
	EXPECT_EQ(resP[1].l, 3.5355339059327373);
	EXPECT_EQ(resP[1].numSegment, 0);
	EXPECT_EQ(resP[1].alpha, 45.0);

	EXPECT_EQ(resP[2].xy.x, 5.0000000000000009);
	EXPECT_EQ(resP[2].xy.y, 5.0000000000000009);
	EXPECT_EQ(resP[2].l, 7.0710678118654746);
	EXPECT_EQ(resP[2].numSegment, 0);
	EXPECT_EQ(resP[2].alpha, 45.0);

	EXPECT_EQ(resP[3].xy.x, 7.5000000000000018);
	EXPECT_EQ(resP[3].xy.y, 7.5000000000000018);
	EXPECT_EQ(resP[3].l, 10.606601717798211);
	EXPECT_EQ(resP[3].numSegment, 0);
	EXPECT_EQ(resP[3].alpha, 45.0);
}

TEST(TEST_PROJECTION, Test90Projection) {
	Projection p;
	vector<ConfigTrace> cTrace = { ConfigTrace(0.0,0.0),
								   ConfigTrace(10.0,10.0) };
	Measurements m = Measurements(0.0, 0.0, 5, 180, 5); // 
	vector<ProjectionDot> resP = p.allProjectionForMeasurement(&cTrace, 0, &m);
	EXPECT_FALSE(resP.empty());
	EXPECT_EQ(resP.size(), 1);

	EXPECT_EQ(resP[0].xy.x, 0);
	EXPECT_EQ(resP[0].xy.y, 0);
	EXPECT_EQ(resP[0].l, 0);
	EXPECT_EQ(resP[0].numSegment, 0);
	EXPECT_EQ(resP[0].alpha, 135.00000000000000);

}

TEST(TEST_PROJECTION, TestMinus355Projection) {
	Projection p;
	vector<ConfigTrace> cTrace = { ConfigTrace(0.0,0.0),
								   ConfigTrace(5.0,0.0) };
	Measurements m = Measurements(0.0, 0.0, 5, 355.0, 5); // 
	
	vector<ProjectionDot> resP = p.allProjectionForMeasurement(&cTrace, 0, &m);
	EXPECT_FALSE(resP.empty());
	EXPECT_EQ(resP.size(), 2);

	EXPECT_EQ(resP[0].xy.x, 0);
	EXPECT_EQ(resP[0].xy.y, 0);
	EXPECT_EQ(resP[0].l, 0);
	EXPECT_EQ(resP[0].numSegment, 0);
	EXPECT_EQ(resP[0].alpha, m.phi);
}

TEST(TEST_PROJECTION, TestFilterLenProjection) {
	Projection p;
	vector<ConfigTrace> cTrace = { ConfigTrace(0.0,0.0) };
	Measurements m = Measurements(0.0, 0.0, 5, 90, 5);

	vector<ProjectionDot> prForDotInput = { ProjectionDot{ {0,0},0,0,0 },
									   ProjectionDot{ {5,5},double(FILTER_COOF_LEN + 1), 0, 0 }
									 };
	vector<ProjectionDot> result = p.filterProjection(&prForDotInput);
	EXPECT_FALSE(result.empty());
	EXPECT_EQ(result.size(), 1);
	EXPECT_EQ(result[0].l, prForDotInput[0].l);
}

TEST(TEST_PROJECTION, TestFilterSegProjection) {
	Projection p;
	vector<ConfigTrace> cTrace = { ConfigTrace(0.0,0.0) };
	Measurements m = Measurements(0.0, 0.0, 5, 90, 5);

	vector<ProjectionDot> prForDotInput = { ProjectionDot{ {0,0},0,0,0 },
									   ProjectionDot{ {5,5},0,0,FILTER_COOF_SEG_NUM + 1}
	};
	vector<ProjectionDot> result = p.filterProjection(&prForDotInput);
	EXPECT_FALSE(result.empty());
	EXPECT_EQ(result.size(), 1);
	EXPECT_EQ(result[0].numSegment, prForDotInput[0].numSegment);
}

TEST(TEST_PROJECTION, TestFilterAngleProjection) {
	Projection p;
	vector<ConfigTrace> cTrace = { ConfigTrace(0.0,0.0) };
	Measurements m = Measurements(0.0, 0.0, 5, 90, 5);

	vector<ProjectionDot> prForDotInput = { ProjectionDot{ {0,0},0,0,0 },
									   ProjectionDot{ {5,5},0,double(FILTER_COOF_ANGLE + 1),0}
	};
	vector<ProjectionDot> result = p.filterProjection(&prForDotInput);
	EXPECT_FALSE(result.empty());
	EXPECT_EQ(result.size(), 1);
	EXPECT_EQ(result[0].alpha, prForDotInput[0].alpha);
}
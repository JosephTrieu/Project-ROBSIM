// ProgrammingDemo.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdio.h>
#include <conio.h>
#include "ensc-488.h"
#include "StdAfx.h"
#include <iostream>
#include <string>
#include <cmath>
#include <cstdio>
#include <algorithm>
using namespace std;

// pre-declare these here because INVKIN uses them
double a[6] = { 0, 0, 0, 0, 0, 0 };
double d[6] = { 0, 0, 0, 0, 0, 0 };

typedef struct vec4 {
	double data[4];
};

// Robot link lengths
const double L2 = 195.0;  // in mm
const double L3 = 142.0;  // in mm

// KIN: Compute tool pose (x,y,z,phi) given joint vector q
void KIN(const JOINT& q, double& x, double& y, double& z, double& phi)
{
    double t1 = DEG2RAD(q[0]);
    double t2 = DEG2RAD(q[1]);
    double t4 = DEG2RAD(q[3]);

    // Forward kinematics equations
    x = L2 * cos(t1) + L3 * cos(t1 + t2);
    y = L2 * sin(t1) + L3 * sin(t1 + t2);
    z = -q[2];             // prismatic joint for vertical motion
    phi = t1 + t2 + t4;    // orientation of end-effector
}

void WHERE(const JOINT& q)
{
    double x, y, z, phi;
    KIN(q, x, y, z, phi);

    printf("\nTool Frame Pose (x ; y ; z ; phi):\n");
    printf("(%.2f ; %.2f ; %.2f ; %.2f deg)\n", x, y, z, RAD2DEG(phi));

    // Display robot graphically (simulator)
    DisplayConfiguration(const_cast<JOINT&>(q));
}

double evaluateSolution(const JOINT& solution, const JOINT& current, bool& valid) {
	double maxDelta = 0;
	
	if (abs(solution[0]) >= 150) { cout << "joint 1 limits violated\n"; valid = false;};
	if (abs(solution[1]) >= 100) { cout << "joint 2 limits violated\n"; valid = false;};
	if ((solution[2] >= -100) || (solution[2] <= -200)) { cout << "joint 3 limits violated\n"; valid = false;};
	if (abs(solution[3]) >= 160) { cout << "joint 4 limits violated\n"; valid = false;};


	if (valid) {
		maxDelta = max({
		abs(current[0] - solution[0]),
		abs(current[1] - solution[1]),
		abs(current[2] - solution[2]),
		abs(current[3] - solution[3])});
	}
	return maxDelta;
}

bool INVKIN(const vec4 &goal, const JOINT &start, JOINT &out, bool print) {

	double x= goal.data[0], y= goal.data[1], z = goal.data[2], phi = goal.data[3];

	JOINT curr_joint = { start[0],start[1],start[2],start[3] };

	JOINT sol1 = { 0, 0, 0, 0 }, sol2 = { 0, 0, 0, 0 }; //Elbow up and Elbow down solutions

	double r = sqrt(x * x + y * y);

	double xy_angle = RAD2DEG(atan2(y, x));
	double offset_angle = (r < 337) ? RAD2DEG(acos((a[2] * a[2] + r * r - a[3] * a[3]) / (2 * a[2] * r))) : 0;
	double elbow_angle = (r < 337) ? 180 - RAD2DEG(acos((a[2] * a[2] + a[3] * a[3] - r * r) / (2 * a[2] * a[3]))) : 0;
	double height = d[0] + d[2] - d[4] - d[5] - z;

	sol1[0] = xy_angle - offset_angle;
	sol2[0] = xy_angle + offset_angle;

	sol1[1] = elbow_angle;
	sol2[1] = -elbow_angle;

	sol1[2] = sol2[2] = height;

	sol1[3] = -phi + sol1[0] + sol1[1];
	sol2[3] = -phi + sol2[0] + sol2[1];

	//normalize to -180 to 180
	sol1[0] = (sol1[0] < -180) ? sol1[0] + 360 : (sol1[0] > 180) ? sol1[0] - 360 : sol1[0];
	sol2[0] = (sol2[0] < -180) ? sol2[0] + 360 : (sol2[0] > 180) ? sol2[0] - 360 : sol2[0];

	sol1[3] = (sol1[3] > 180) ? sol1[3] - 360 : (sol1[3] < -180) ? sol1[3] + 360 : sol1[3];
	sol2[3] = (sol2[3] > 180) ? sol2[3] - 360 : (sol2[3] < -180) ? sol2[3] + 360 : sol2[3];

	//evaluate the solutions
	bool valid1 = true, valid2 = true;
	double bad_score1 = evaluateSolution(sol1, curr_joint, valid1);
	double bad_score2 = evaluateSolution(sol2, curr_joint, valid2);

	// Choose the best solution

	if (valid1 && valid2) {
		if (bad_score1 < bad_score2) {
			if (print)cout << "\nFast solution:\ntheta 1: " << sol1[0] << "\ntheta 2: " << sol1[1] << "\nd3: " << sol1[2] << "\ntheta 4: " << sol1[3] << endl;
			if (print)cout << "\nSlow solution:\ntheta 1: " << sol2[0] << "\ntheta 2: " << sol2[1] << "\nd3: " << sol2[2] << "\ntheta 4: " << sol2[3] << endl;
			out[0] = sol1[0]; out[1] = sol1[1]; out[2] = sol1[2]; out[3] = sol1[3];
		}
		else {
			if (print)cout << "\nFast solution:\ntheta 1: " << sol2[0] << "\ntheta 2: " << sol2[1] << "\nd3: " << sol2[2] << "\ntheta 4: " << sol2[3] << endl;
			if (print)cout << "\nSlow solution:\ntheta 1: " << sol1[0] << "\ntheta 2: " << sol1[1] << "\nd3: " << sol1[2] << "\ntheta 4: " << sol1[3] << endl;
			out[0] = sol2[0]; out[1] = sol2[1]; out[2] = sol2[2]; out[3] = sol2[3];
		}
	}
	else if (valid1) {
		if (print)cout << "One solution found\ntheta 1: " << sol1[0] << "\ntheta 2: " << sol1[1] << "\nd3: " << sol1[2] << "\ntheta 4: " << sol1[3] << endl;
		out[0] = sol1[0]; out[1] = sol1[1]; out[2] = sol1[2]; out[3] = sol1[3];
	}
	else if (valid2) {
		if (print)cout << "One solution found\ntheta 1: " << sol2[0] << "\ntheta 2: " << sol2[1] << "\nd3: " << sol2[2] << "\ntheta 4: " << sol2[3] << endl;
		out[0] = sol2[0]; out[1] = sol2[1]; out[2] = sol2[2]; out[3] = sol2[3];
	}
	else {
		if (print)cout << "No valid solutions within joint limits." << endl;
		return false;
	}
	return true;
}

// TEST HARNESS MAIN
/*
* Strategy: compute a bunch of verified correct test cases and use those to verify the correctness of the KIN, WHERE, INVKIN
*/

//OLD MAIN FOR ACTUAL DEMO
int main(int argc, char* argv[])
{
	// joint vectors q are of the form {theta1, theta2, d3, theta4}
	JOINT q1 = {0, 0, -100, 0};
	JOINT q2 = {90, 90, -200, 45};
	printf("Keep this window in focus, and...\n");
	
	char ch;
	int c;

	const int ESC = 27;
	
	printf("1Press any key to continue \n");
	printf("2Press ESC to exit \n");

	c = _getch() ;

	//---- test for INVKIN and KIN ----
	// we will use kin to get the tool position in space relative to the base frame for joint vector q2
	// this will be the goal that INVKIN must get a joint vector for, relative to a starting position of q1

	// goal tool position for joint vector q2
	double x, y, z, phi;
	KIN(q2, x, y, z, phi);

	// assign goal to a vec4 such that it can be passed to INVKIN
	vec4 goal;
	goal.data[0] = x;
	goal.data[1] = y;
	goal.data[2] = z;
	goal.data[3] = phi;

	// link lengths and joint offsets for transform matrices
	a[0] = 0; a[1] = 195; a[2] = 142; a[3] = 0; a[4] = 0; a[5] = 0; 
	d[0] = 405; d[1] = 70; d[2] = 0; d[3] = 410; d[4] = 130; d[5] = 0;

	// use INVKIN to solve for a joint vector, which should be q2, and put it into invkin_sol
	JOINT invkin_sol;
	INVKIN(goal, q1, invkin_sol, 1);

	// print q2 and the invkin soluion to see if they match
	printf("goal: x= %.2f, y= %.2f, z= %.2f, phi= %.2f", x, y, z, phi);
	printf("\nq2: theta1= %.2f, theta2= %.2f, d3= %.2f, theta4= %.2f\n", q2[0], q2[1], q2[2], q2[3]);
	printf("invkin solution: theta1= %.2f, theta2= %.2f, d3= %.2f, theta4= %.2f\n\n", invkin_sol[0], invkin_sol[1], invkin_sol[2], invkin_sol[3]);

	while (1)
	{
		
		if (c != ESC)
		{
			printf("Press '1' or '2' \n");
			ch = _getch();

			if (ch == '1')
			{
				MoveToConfiguration(q1);
				DisplayConfiguration(q1);
				WHERE(q1);
			}
			else if (ch == '2')
			{
				MoveToConfiguration(q2);
				DisplayConfiguration(q2);
				WHERE(q2);
			}

			printf("Press any key to continue \n");
			printf("Press q to exit \n");
			c = _getch();
		}
		else
			break;
	}
	

	return 0;
}

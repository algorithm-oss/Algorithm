#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <time.h>
#include <string>
#include <windows.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <ilcplex/ilocplex.h>

using namespace std;
ILOSTLBEGIN

typedef IloArray<IloNumArray>NumMatrix;
typedef IloArray<IloNumVarArray>NumVarMatrix;
typedef IloArray<NumVarMatrix>NumVar3Matrix;
typedef IloArray<NumVar3Matrix>NumVar4Matrix;

//Problem Notation
int T;//the length of planning horizon
int C;//truck capacity
int Setup_cost;// K
int Trans_cost;// A
vector<int>Order_quan;// M_i
vector<vector<int>> Demand_order;// q_i^k
vector<int> Inven_cost;// h_t
vector<vector<vector<int>>> Wait_cost;// w_it^k

//Time variables
double run_time_CPLEX;
double run_time_PDH;
double run_time;
time_t begin_time;
time_t begin_time2;
time_t finish_time_CPLEX;
time_t finish_time_PDH;
//the objective value of the offline problem
double obj_value_offline;
//the value of the Primal-Dual heuristic
double value_PDH;
//the production cost
double value_PC;
//the waiting cost
double value_WC;
//the holding cost
double value_HC;
//the transportation cost
double value_TC;
//Competitive ratio
double C_ratio;

//reading data from the file
static void Read_Data(int instance);
//cplex procedure
static void Integer_Program(int instance);
//Primal-Dual heuristic
static void Primal_Dual(int instance);
//find all subsets
//void Find_subset(vector<int>& a, vector<int>& t, vector<vector<int>>& ans, int index, int length, int n);
//output results
static void report_result(int instance);

//main program//
int main()
{
	for (int instance = 1; instance <= 150; instance++)
	{
		//instance = 95;
		cout << "instance = " << instance << endl;
		//reading data from the file
		Read_Data(instance);
		begin_time = clock();
		//cplex procedure
		Integer_Program(instance);
		//Primal-Dual heuristic
		Primal_Dual(instance);
		//output results
		report_result(instance);
		cout << endl;
	}
	cout << "This procedure ends!" << endl;
	return 0;
}

//reading data from the file
static void Read_Data(int instance)
{
	string string1 = "F:\\management of paper\\one manufacturer and one retailer\\C++\\Data\\instance";
	string string2 = ".txt";
	string input_location = string1 + to_string(instance) + string2;
	char* o_file = (char*)input_location.data();
	ifstream instuf(o_file);
	if (!instuf) {
		cerr << "ERROR: could not open file " << endl;
		abort();
	}

	//reset
	C_ratio = obj_value_offline = value_PDH = 9999999;

	//input data 
	instuf >> T;
	instuf >> C;
	instuf >> Setup_cost;
	instuf >> Trans_cost;

	Order_quan.resize(T + 1, 0);
	for (int i = 1; i <= T; i++)
	{
		instuf >> Order_quan[i];
	}

	Demand_order.resize(T + 1, vector<int>());
	for (int i = 1; i <= T; i++)
	{
		Demand_order[i].resize(6, 0);
	}
	for (int i = 1; i <= T; i++)
	{
		for (int k = 1; k <= Order_quan[i]; k++)
		{
			instuf >> Demand_order[i][k];
		}
	}

	Inven_cost.resize(T + 1, 0);
	for (int t = 1; t <= T; t++)
		instuf >> Inven_cost[t];

	Wait_cost.resize(T + 1, vector<vector<int>>(6, vector<int>()));
	for (int i = 1; i <= T; i++)
	{
		for (int k = 1; k <= Order_quan[i]; k++)
		{
			Wait_cost[i][k].resize(T + 1, 0);
		}
	}
	for (int i = 1; i <= T; i++)
	{
		for (int k = 1; k <= Order_quan[i]; k++)
		{
			for (int t = 1; t <= T; t++)
			{
				instuf >> Wait_cost[i][k][t];
			}
		}
	}		
    instuf.close();
	cout << endl;
}

static void Integer_Program(int instance)
{
	//output log
	string string3 = "F:\\management of paper\\one manufacturer and one retailer\\C++\\Results\\Log\\";
	string string4 = ".txt";
	string output_location = string3 + to_string(instance) + string4;
	char* o_file = (char*)output_location.data();
	ofstream logfile(o_file);
	//ofstream logfile;
	//logfile.open(o_file, ios::app);

	// environment to organize the data and build the model of the problem //
	IloEnv IP_env;
	try
	{
		// parameters //
		IloModel IP_Model(IP_env);
		//extract the model
		IloCplex IP_Solver(IP_Model);
		//do not want to display any output on the screen
		//IP_Solver.setOut(IP_env.getNullStream());
		//Using a template to add objects
		IloObjective IP_Obj = IloAdd(IP_Model, IloMinimize(IP_env));

		//control output log
		IP_Solver.setOut(logfile);
		IP_Solver.setWarning(logfile);
		IP_Solver.setError(logfile);
		//set the limited running time
		IP_Solver.setParam(IloCplex::TiLim, 600);
		//find hidden feasible solutions
		IP_Solver.setParam(IloCplex::MIPEmphasis, 4);

		// define decision variables //
		NumVar3Matrix x(IP_env, T + 1);
		for (int i = 1; i <= T; i++)
		{
			x[i] = NumVarMatrix(IP_env, Order_quan[i] + 1);
			for (int k = 1; k <= Order_quan[i]; k++)
			{
				x[i][k] = IloNumVarArray(IP_env, T + 1);
			for (int t = i; t <= T; t++)
				{
					x[i][k][t] = IloNumVar(IP_env, 0, IloInfinity, ILOINT);
				}
			}
		}
		
		NumVar3Matrix y(IP_env, T + 1);
		for (int i = 1; i <= T; i++)
		{
			y[i] = NumVarMatrix(IP_env, Order_quan[i] + 1);
			for (int k = 1; k <= Order_quan[i]; k++)
			{
				y[i][k] = IloNumVarArray(IP_env, T + 1);
				for (int t = i; t <= T; t++)
				{
					y[i][k][t] = IloNumVar(IP_env, 0, 1, ILOBOOL);
				}
			}
		}
		
		IloNumVarArray z(IP_env, T + 1);
		for (int t = 1; t <= T; t++)
		{
			z[t] = IloNumVar(IP_env, 0, IloInfinity, ILOINT);
		}
		
		IloNumVarArray u(IP_env, T + 1);
		for (int t = 1; t <= T; t++)
		{
			u[t] = IloNumVar(IP_env, 0, 1, ILOBOOL);
		}

		// objective value //
		IloExpr obj(IP_env);

		IloExpr obj_setup(IP_env);
		for (int t = 1; t <= T; t++)
		{
			obj_setup += Setup_cost * u[t];
		}

		IloExpr obj_wait(IP_env);
		for (int t = 1; t <= T; t++)
		{
			for (int i = 1; i <= t; i++)
			{
				for (int k = 1; k <= Order_quan[i]; k++)
				{
					IloExpr sum_y(IP_env);
					for (int s = i; s <= t; s++)
					{
						sum_y += y[i][k][s];
					}
					obj_wait += (1 - sum_y) * Demand_order[i][k] * Wait_cost[i][k][t];
					sum_y.end();
				}
			}
		}
		
		IloExpr obj_inven(IP_env);
		for (int i = 1; i <= T; i++)
		{
			for (int k = 1; k <= Order_quan[i]; k++)
			{
				for (int t = i; t <= T; t++)
				{
					for (int s = i; s <= t; s++)
					{
						obj_inven += Inven_cost[t] * (x[i][k][s] - Demand_order[i][k] * y[i][k][s]);
					}	
				}
			}
		}

		IloExpr obj_trans(IP_env);
		for (int t = 1; t <= T; t++)
		{
			obj_trans += Trans_cost * z[t];
		}
		obj = obj_setup + obj_wait + obj_inven + obj_trans;
		IP_Obj.setExpr(obj);

		// constraints //
		for (int t = 1; t <= T; t++)
		{
			IloExpr sum_x(IP_env);
			for (int i = 1; i <= t; i++)
			{
				for (int k = 1; k <= Order_quan[i]; k++)
				{
					sum_x += x[i][k][t];
				}
			}
			IP_Model.add(sum_x <= 99999999 * u[t]);
			sum_x.end();
		}
		
		for (int i = 1; i <= T; i++)
		{
			for (int k = 1; k <= Order_quan[i]; k++)
			{
				IloExpr sum_x(IP_env);
				for (int t = i; t <= T; t++)
				{
					sum_x += x[i][k][t];
				}
				IP_Model.add(sum_x == Demand_order[i][k]);
				sum_x.end();
			}
		}
		
		for (int i = 1; i <= T; i++)
		{
			for (int k = 1; k <= Order_quan[i]; k++)
			{
				for (int t = i; t <= T; t++)
				{
					IloExpr sum_x(IP_env);
					for (int s = i; s <= t; s++)
					{
						sum_x += x[i][k][s];
					}
					IloExpr sum_y(IP_env);
					for (int ss = i; ss <= t; ss++)
					{
						sum_y += y[i][k][ss] * Demand_order[i][k];
					}
					IP_Model.add(sum_x >= sum_y);
					sum_x.end();
					sum_y.end();
				}
			}
		}
		
		for (int i = 1; i <= T; i++)
		{
			for (int k = 1; k <= Order_quan[i]; k++)
			{
				IloExpr sum_y(IP_env);
				for (int t = i; t <= T; t++)
				{
					sum_y += y[i][k][t];
				}
				IP_Model.add(sum_y == 1);
				sum_y.end();
			}
		}
		
		for (int t = 1; t <= T; t++)
		{
			IloExpr sum_qy(IP_env);
			for (int i = 1; i <= t; i++)
			{
				for (int k = 1; k <= Order_quan[i]; k++)
				{
					sum_qy += Demand_order[i][k] * y[i][k][t];
				}
			}
			IP_Model.add((z[t] - 1) * C <= sum_qy);
			IP_Model.add(sum_qy <= z[t] * C);
			sum_qy.end();
		}

		//extract the model
		IP_Solver.extract(IP_Model);
		//output the model
		//IP_Solver.exportModel("IP_Model.lp");

		cout << "IP starts!" << endl;
		//solve the model
		IP_Solver.solve();
		cout << "IP ends!" << endl;
		IP_env.out() << "Solution status: " << IP_Solver.getStatus() << endl;

		//run time
		finish_time_CPLEX = clock();
		run_time_CPLEX = double(finish_time_CPLEX - begin_time) / CLOCKS_PER_SEC;
		//the objective value of this model
		obj_value_offline = IP_Solver.getObjValue();
 

		obj_setup.end(); obj_inven.end(); obj_trans.end();
		obj.end();
		//free memory
		//x.end(); y.end(); z.end(); u.end();
		//IP_Obj.end();
		//IP_Model.end();
		//IP_Solver.end();
	}
	catch (IloException& e) {
		std::cerr << "IloException: " << e << std::endl;
		logfile << "Concert exception caught: " << e << endl;
	}
	catch (std::exception& e) {
		std::cerr << "Standard exception: " << e.what() << std::endl;
		logfile << "Standard exception: " << e.what() << endl;
	}
	catch (...) {
		std::cerr << "Some other exception!" << std::endl;
		logfile << "Unknown exception caught" << endl;
	}
	IP_env.end();
	logfile.close();
}

//heuristic
static void Primal_Dual(int instance)
{
	cout << endl;
	cout << "PDH starts!" << endl;
	begin_time2 = clock();

	vector<vector<int>> PT_time(T + 1, vector<int>(6, 99999));//production and transportation time
	vector<vector<vector<double>>> a(T + 1, vector<vector<double>>(6, vector<double>(T + 1, 0)));//>=¦Á_is^k
	vector<vector<int>> U(T + 1, vector<int>(6, 0)); //replace U_t
	
	for (int t = 1; t <= T; t++)
	{
		for (int k = 1; k <= Order_quan[t]; k++)
		{
			U[t][k] = 1;
		}

		for (int i = 1; i <= t; i++)
		{
			for (int k = 1; k <= Order_quan[i]; k++)
			{
				if (U[i][k] == 1)
				{
					a[i][k][t] = Demand_order[i][k] * Wait_cost[i][k][t];
				}
			}
		}
		
		for (int i = 1; i <= t; i++)
		{
			double sum_a = 0;
			for (int ii = 1; ii <= i; ii++)
			{
				for (int k = 1; k <= Order_quan[ii]; k++)
				{
					if (U[ii][k] == 1)
					{
						for (int s = i; s <= T; s++)
						{
							sum_a += a[ii][k][s];
						}

					}
				}
			}
			if (sum_a >= Setup_cost + Trans_cost)
			{
				for (int i2 = 1; i2 <= t; i2++)
				{
					for (int k = 1; k <= Order_quan[i2]; k++)
					{
						if (U[i2][k] == 1)
						{
							U[i2][k] = 0;
							PT_time[i2][k] = t;
						}
					}
				}
				break;
			}
		}
	}
	int sum_U = 0;
	for (int i = 1; i <= T; i++)
	{
		for (int k = 1; k <= Order_quan[i]; k++)
		{
			sum_U += U[i][k];
		}
	}
	cout << "PDH ends!" << endl;
	if (sum_U <= 0)
	{
		cout << "PDH right!!" << endl;
	}
	else {
		cout << "PDH error......" << endl;
	}

	//objective value obtained by Algorithm 1
	int obj_setup = 0;
	for (int t = 1; t <= T; t++)
	{
		int temp = 0;
		for (int i = 1; i <= t; i++)
		{
			for (int k = 1; k <= Order_quan[i]; k++)
			{
				if (PT_time[i][k] == t)
				{
					obj_setup += Setup_cost;
					temp = 1;
					break;
				}
			}
			if (temp == 1)break;
		}
	}

	int obj_wait = 0;
	for (int i = 1; i <= T; i++)
	{
		for (int k = 1; k <= Order_quan[i]; k++)
		{
			int sum_w = 0;
			for (int t = i; t <= PT_time[i][k] - 1; t++)
			{
				sum_w += Wait_cost[i][k][t];
			}
			obj_wait += Demand_order[i][k] * sum_w;
		}
	}

	int obj_trans = 0;
	for (int t = 1; t <= T; t++)
	{
		double sum_q = 0;
		for (int i = 1; i <= t; i++)
		{
			for (int k = 1; k <= Order_quan[i]; k++)
			{
				if (PT_time[i][k] == t)
				{
					sum_q += Demand_order[i][k];
				}
			}
		}
		obj_trans += ceil(sum_q / C) * Trans_cost;
	}

	value_PDH = obj_setup + obj_wait + obj_trans;
	value_PC = obj_setup;
	value_WC = obj_wait;
	value_TC = obj_trans;

	//run time
	finish_time_PDH = clock();
	run_time_PDH = double(finish_time_PDH - begin_time2) / CLOCKS_PER_SEC;
	run_time = run_time_CPLEX + run_time_PDH;

	//Competitive ratio
	C_ratio = value_PDH / obj_value_offline;
}

//output results
static void report_result(int instance)
{
	string string5 = "F:\\management of paper\\one manufacturer and one retailer\\C++\\Results\\";
	string string6 = "Algorithm 1.txt";
	string output_location = string5 + string6;
	char* o_file = (char*)output_location.data();
	//ofstream outstuf(o_file);
	ofstream outstuf;
	outstuf.open(o_file, ios::app);
	if (!outstuf)
	{
		cerr << "file could not be open." << endl;
	}
	outstuf << instance << '\t' << obj_value_offline << '\t' << value_PDH << '\t' << value_PC << '\t' << value_WC << '\t' << value_TC <<
		'\t' << C_ratio << '\t' << run_time_CPLEX << '\t' << run_time_PDH << '\t' << run_time << endl;
	outstuf.close();
}
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <time.h>
#include <string>
#include <windows.h>
#include <math.h>
#include <vector>
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
vector<int>Prod_cost;// p_t
vector<int> Inven_cost;// h_t
vector<vector<vector<int>>> Wait_cost;// w_it^k
vector<int>Prod_cap;// L_t

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

//read data from the file
static void Read_Data(int instance)
{
	string string1 = "F:\\management of paper\\one manufacturer and one retailer\\C++\\Extension\\Data\\instance";
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
		Demand_order[i].resize(Order_quan[i]+1, 0);
	}
	for (int i = 1; i <= T; i++)
	{
		for (int k = 1; k <= Order_quan[i]; k++)
		{
			instuf >> Demand_order[i][k];
		}
	}	

	Inven_cost.resize(T + 2, 0);
	for (int t = 1; t <= T+1; t++)
		instuf >> Inven_cost[t];

	Wait_cost.resize(T + 1, vector<vector<int>>(6, vector<int>()));
	for (int i = 1; i <= T; i++)
	{
		for (int k = 1; k <= Order_quan[i]; k++)
		{
			Wait_cost[i][k].resize(T + 2, 0);
		}
	}
	for (int i = 1; i <= T; i++)
	{
		for (int k = 1; k <= Order_quan[i]; k++)
		{
			for (int t = 1; t <= T+1; t++)
			{
				instuf >> Wait_cost[i][k][t];
			}
		}
	}

	Prod_cost.resize(T + 2, 0);
	for (int t = 1; t <= T + 1; t++)
		instuf >> Prod_cost[t];

	Prod_cap.resize(T + 2, 0);
	for (int t = 1; t <= T + 1; t++)
		instuf >> Prod_cap[t];

	instuf.close();

	cout << endl;
}

//IP,x_{i,t}^k,y_{i,t}^k,z_t,u_t
static void Integer_Program(int instance)
{
	//output log
	string string3 = "F:\\management of paper\\one manufacturer and one retailer\\C++\\Extension\\Results\\Log\\";
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
		//seek hidden feasible solutions
		IP_Solver.setParam(IloCplex::MIPEmphasis, 4);

		// define decision variables //
		NumVar3Matrix x(IP_env, T + 1);
		for (int i = 1; i <= T; i++)
		{
			x[i] = NumVarMatrix(IP_env, Order_quan[i] + 1);
			for (int k = 1; k <= Order_quan[i]; k++)
			{
				x[i][k] = IloNumVarArray(IP_env, T + 2);
				for (int t = i; t <= T+1; t++)
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
				y[i][k] = IloNumVarArray(IP_env, T + 2);
				for (int t = i; t <= T+1; t++)
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

		IloExpr obj_prod(IP_env);
		for (int i = 1; i <= T; i++)
		{
			for (int k = 1; k <= Order_quan[i]; k++)
			{
				for (int t = i; t <= T + 1; t++)
				{
					obj_prod += Prod_cost[t] * x[i][k][t];
				}
			}
		}

		IloExpr obj_wait(IP_env);
		for (int i = 1; i <= T; i++)
		{
			for (int k = 1; k <= Order_quan[i]; k++)
			{
				for (int t = i; t <= T+1; t++)
				{
					IloExpr sum_w(IP_env);
					for (int s = i; s <= t-1; s++)
					{
						sum_w += Demand_order[i][k] * Wait_cost[i][k][s];
					}
					obj_wait += sum_w * y[i][k][t];
					sum_w.end();
				}
			}
		}

		IloExpr obj_inven(IP_env);
		for (int i = 1; i <= T; i++)
		{
			for (int k = 1; k <= Order_quan[i]; k++)
			{
				for (int t = i; t <= T+1; t++)
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
		obj = obj_setup + obj_prod + obj_wait + obj_inven + obj_trans;
		IP_Obj.setExpr(obj);

		// constraints //
		for (int i = 1; i <= T; i++)
		{
			for (int k = 1; k <= Order_quan[i]; k++)
			{
				IloExpr sum_x(IP_env);
				for (int t = i; t <= T+1; t++)
				{
					sum_x += x[i][k][t];
				}
				IP_Model.add(sum_x == Demand_order[i][k]);
				sum_x.end();
			}
		}

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
			IP_Model.add(sum_x <= Prod_cap[t] * u[t]);
			sum_x.end();
		}

		for (int i = 1; i <= T; i++)
		{
			for (int k = 1; k <= Order_quan[i]; k++)
			{
				for (int t = i; t <= T+1; t++)
				{
					IloExpr sum_x(IP_env);
					for (int s = i; s <= t; s++)
					{
						sum_x += x[i][k][s];
					}
					IloExpr sum_y(IP_env);
					for (int aa = i; aa <= t; aa++)
					{
						sum_y += Demand_order[i][k] * y[i][k][aa];
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
				for (int t = i; t <= T+1; t++)
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
		if(IP_Solver.getStatus() == IloAlgorithm::Optimal)
		{
			obj_value_offline = IP_Solver.getObjValue();
		}
	        else if(IP_Solver.getStatus() == IloAlgorithm::Feasible)
		{
			obj_value_offline = IP_Solver.getBestObjValue();
		}
		
		obj_setup.end(); obj_prod.end(); obj_wait.end(); obj_inven.end(); obj_trans.end();
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

	vector<vector<int>> e(T + 1, vector<int>(6, 99999));//e_i^k
	vector<int> u(T + 2, 0);// u_t
	vector<vector<vector<int>>> x(T + 1, vector<vector<int>>(6, vector<int>(T + 2, 0)));//x_it^k
	vector<vector<vector<int>>> y(T + 1, vector<vector<int>>(6, vector<int>(T + 2, 0)));//y_it^k
	vector<vector<int>>temp_d(T + 1, vector<int>(6, 0));//d_i^k=q_i^k
	vector<int>temp_L(T+2, 0);//L_t
	vector<vector<int>> U(T + 1, vector<int>(6, 0));//replace U_t 
	vector<vector<int>> I(T + 1, vector<int>(6, 0));//replace I_t
	for (int t = 1; t <= T+1; t++)
	{
		temp_L[t] = Prod_cap[t];
	}
	for (int t = 1; t <= T+1; t++)
	{
		if (t <= T)
		{
			for (int k = 1; k <= Order_quan[t]; k++)
			{
				temp_d[t][k] = Demand_order[t][k];
				U[t][k] = 1;
			}
			double sum_wd = 0, sum_dp = 0;
			for (int i = 1; i <= t; i++)
			{
				for (int k = 1; k <= Order_quan[i]; k++)
				{
					if (U[i][k] == 1)
					{
						for (int s = i; s <= t; s++)
						{
							sum_wd += Wait_cost[i][k][s] * temp_d[i][k];
						}
						sum_dp += temp_d[i][k] * Prod_cost[t];
					}
				}
			}
			if (2 * sum_wd >= Setup_cost + sum_dp)
			{
				u[t] = 1;
				int temp = 0;
				for (int i = 1; i <= t; i++)
				{
					for (int k = 1; k <= Order_quan[i]; k++)
					{
						if (U[i][k] == 1)
						{
							x[i][k][t] = min(temp_d[i][k], temp_L[t]);
							temp_d[i][k] -= x[i][k][t];
							temp_L[t] -= x[i][k][t];
							if (temp_d[i][k] == 0)
							{
								U[i][k] = 0;
								I[i][k] = 1;
								e[i][k] = t;
							}
							if (temp_L[t] == 0)
							{
								temp = 1;
								break;
							}
						}
					}
					if (temp == 1) break;
				}
			}
			double sum_ah = 0, sum_q = 0;
			for (int i = 1; i <= t; i++)
			{
				for (int k = 1; k <= Order_quan[i]; k++)
				{
					if (I[i][k] == 1)
					{
						for (int s = i; s <= e[i][k]; s++)
						{
							double sum_hw = 0;
							for (int ss = s; ss <= t; ss++)
							{
								sum_hw += Inven_cost[ss] + Wait_cost[i][k][ss];
							}
							sum_ah += x[i][k][s] * sum_hw;
						}
						sum_q += Demand_order[i][k];
					}
				}
			}
			if (sum_ah >= ceil(sum_q / C) * Trans_cost)
			{
				for (int i = 1; i <= t; i++)
				{
					for (int k = 1; k <= Order_quan[i]; k++)
					{
						if (I[i][k] == 1)
						{
							y[i][k][t] = 1;
							I[i][k] = 0;
						}
					}
				}
			}
		}
		else if (t == T + 1)
		{
			for (int i = 1; i <= T; i++)
			{
				for (int k = 1; k <= Order_quan[i]; k++)
				{
					double sum_x = 0;
					for (int s = i; s <= T; s++)
					{
						sum_x += x[i][k][s];
					}
					x[i][k][T + 1] = Demand_order[i][k] - sum_x;
					if (x[i][k][T + 1] > 0)
					{
						e[i][k] = T + 1;
						y[i][k][T + 1] = 1;
						U[i][k] = 0;
					}
				}
			}
			for (int i = 1; i <= T; i++)
			{
				for (int k = 1; k <= Order_quan[i]; k++)
				{
					if (I[i][k] == 1)
					{
						y[i][k][T + 1] = 1;
						I[i][k] = 0;
					}
				}
			}
		}  
	}
	int sum_U = 0, sum_I = 0;
	for (int i = 1; i <= T; i++)
	{
		for (int k = 1; k <= Order_quan[i]; k++)
		{
			sum_U += U[i][k];
			sum_I += I[i][k];
		}
	}
	cout << "PDH ends!" << endl;
	if (sum_U <= 0 && sum_I <= 0)
	{
		cout << "PDH right!!" << endl;
	}
	else {
		cout << "PDH error......" << endl;
	}

	//objective value obtained by Algorithm 2
	int obj_setup = 0;
	for (int t = 1; t <= T; t++)
	{
		obj_setup += Setup_cost * u[t];
	}

	int obj_prod = 0;
	for (int i = 1; i <= T; i++)
	{
		for (int k = 1; k <= Order_quan[i]; k++)
		{
			for (int t = i; t <= T + 1; t++)
			{
				obj_prod += x[i][k][t] * Prod_cost[t];
			}
		}
	}

	int obj_wait = 0;
	for (int i = 1; i <= T; i++)
	{
		for (int k = 1; k <= Order_quan[i]; k++)
		{
			for (int t = i; t <= T+1; t++)
			{
				int sum_w = 0;
				for (int s = i; s <= t-1; s++)
				{
					sum_w += Demand_order[i][k] * Wait_cost[i][k][s];
				}
				obj_wait += y[i][k][t] * sum_w;
			}
		}
	}

	int obj_inven = 0;
	for (int i = 1; i <= T; i++)
	{
		for (int k = 1; k <= Order_quan[i]; k++)
		{
			for (int t = i; t <= T + 1; t++)
			{
				for (int s = i; s <= t; s++)
				{
					obj_inven += (x[i][k][s] - Demand_order[i][k] * y[i][k][s]) * Inven_cost[t];
				}
			}
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
				sum_q += Demand_order[i][k] * y[i][k][t];
			}
			
		}
		obj_trans += ceil(sum_q / C) * Trans_cost;
	}

	value_PDH = obj_setup + obj_prod + obj_wait + obj_inven + obj_trans;

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
	string string5 = "F:\\management of paper\\one manufacturer and one retailer\\C++\\Extension\\Results\\";
	string string6 = "Algorithm 2.txt";
	string output_location = string5 + string6;
	char* o_file = (char*)output_location.data();
	//ofstream outstuf(o_file);
	ofstream outstuf;
	outstuf.open(o_file, ios::app);
	if (!outstuf)
	{
		cerr << "file could not be open." << endl;
	}
	outstuf << instance << '\t' << obj_value_offline << '\t' << value_PDH << '\t' << 
		C_ratio << '\t' << run_time_CPLEX << '\t' << run_time_PDH << '\t' << run_time << endl;
	outstuf.close();
}

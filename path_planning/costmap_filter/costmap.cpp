#include <iostream>
#include <ctime>

using namespace std;

int main()
{
	const int filter_size{7};

	int** map = new int*[200];
	for (int i{0}; i<200; i++)
	{
		map[i] = new int[200];
	}

	for (int i{0}; i<200; i++)
	{
		for (int j{0}; j<200; j++)
		{
			map[i][j] = 1;
		}
	}

	cout << "hello\n";


	int time{static_cast<int>(clock())};
	

	for (int i{0}; i<200; i++)
	{
		for (int j{0}; j<200; j++)
		{
			if (i<=filter_size/2)
			{
				for (int _i{0}; _i<=i+filter_size/2; _i++)
				{
					if (j<=filter_size/2)
					{
						for (int _j{0}; _j<=j+filter_size/2; _j++)
						{
							map[_i][_j] += map[i][j];
						}

					}
					else if (j>=200-filter_size/2)
					{
						for (int _j{199}; _j>=j-filter_size/2; _j--)
						//	map[_i][_j] += map[i][j];
						{
							map[_i][_j] += map[i][j];
						}
					}
					else
					{
						for (int _j{j-filter_size/2}; _j<=j+filter_size/2; _j++)
						//	map[_i][_j] += map[i][j];
						{
							map[_i][_j] += map[i][j];
						}
					}
				}
			}
			else if (i>=200-filter_size/2)
			{
				for (int _i{199}; _i>=i-filter_size/2; _i--)
				{
					if (j<=filter_size/2)
					{
						for (int _j{0}; _j<=j+filter_size/2; _j++)
						//	map[_i][_j] += map[i][j];
						{
							map[_i][_j] += map[i][j];
						}
					}
					else if (j>200-filter_size/2)
					{
						for (int _j{199}; _j>=j-filter_size/2; _j--)
						//	map[_i][_j] += map[i][j];
						{
							map[_i][_j] += map[i][j];
						}
					}
					else
					{
						for (int _j{j-filter_size/2}; _j<=j+filter_size/2; _j++)
						//	map[_i][_j] += map[i][j];
						{
							map[_i][_j] += map[i][j];
						}
					}
				}
			}
			else	
			{
				for (int _i{i-filter_size/2}; _i<=i+filter_size/2; _i++)
				{
					for (int _j{j-filter_size/2}; _j<=j+filter_size/2; _j++)
					{
						map[_i][_j] += map[i][j];
					}
				}
			}
		}
	}

	cout << "duration time : " << (clock()-time)/(double)CLOCKS_PER_SEC << endl;

	return 0;
}

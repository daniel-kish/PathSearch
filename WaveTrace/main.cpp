#include <iostream>
#include <vector>
#include <queue>
#include <iomanip>
#include <algorithm>
#include <sstream>

enum node { empty = -2, banned = -1, zero = 0, one = 1, two = 2, on_path=5 };

node operator+(node const& n, int i)
{
	int v = int(n);
	v = ((v + i) % 3 + 3) % 3;
	return node(v);
}

std::ostream& operator<<(std::ostream& os, node const& n)
{
	if (n == node::on_path)
		return os << std::setw(2) << 'p';
	return os << std::setw(2) << int(n);
}

struct field {
	std::vector<node> data;
	int cols, rows;
	field(int c, int r): data(r*c, node::empty), cols(c), rows(r)
	{}
	node& operator()(int col, int row)
	{
		return data[row*cols + col];
	}
	const node& operator()(int col, int row) const
	{
		return data[row*cols + col];
	}
};

std::ostream& operator<<(std::ostream& os, field const& f)
{
	for (int row = f.rows - 1; row >= 0; row--)
	{
		for (int col = 0; col < f.cols; col++)
		{
			os << f(col, row) << ' ';
		}
		os << '\n';
	}
	return os;
}

void bfs_enumerate(field& f, int row, int col)
{
	f(row, col) = node::zero;
	std::queue<std::pair<int,int>> q;
	q.push({row,col});

	while (!q.empty())
	{
		auto head = q.front(); q.pop();

		node head_val = f(head.first, head.second);

		if (head.first < f.cols - 1 && f(head.first + 1, head.second) == node::empty) {
			f(head.first + 1, head.second) = head_val + 1;
			q.push({head.first + 1,head.second});
		}
		if (head.first > 0 && f(head.first - 1, head.second) == node::empty) {
			f(head.first - 1, head.second) = head_val + 1;
			q.push({head.first - 1,head.second});
		}
		if (head.second < f.rows - 1 && f(head.first, head.second + 1) == node::empty) {
			f(head.first, head.second + 1) = head_val + 1;
			q.push({head.first,head.second + 1});
		}
		if (head.second > 0 && f(head.first, head.second - 1) == node::empty) {
			f(head.first, head.second - 1) = head_val + 1;
			q.push({head.first,head.second - 1});
		}
	}
}

std::vector<std::pair<int, int>> build_path(field const& f, int C, int R, int c, int r)
{
	std::vector<std::pair<int, int>> path;
	node target_val = f(c, r);
	path.push_back({c,r});

	node next_val = target_val + (-1);

	while (true)
	{
		if (c == C && r == R) break;
		if (c < f.cols - 1 && f(c + 1, r) == next_val)
			path.push_back({c + 1, r});
		else if (c > 0 && f(c - 1, r) == next_val)
			path.push_back({c - 1, r});
		else if (r < f.rows-1 && f(c, r+1) == next_val)
			path.push_back({c, r+1});
		else if (r > 0 && f(c, r - 1) == next_val)
			path.push_back({c, r - 1});

		c = path.back().first;
		r = path.back().second;
		next_val = f(c,r) + (-1);
	}
	return path;
}

int main()
{	
	field f(5, 6);

	f(1, 0) = node::banned;
	f(1, 1) = node::banned;
	f(1, 2) = node::banned;
	f(1, 3) = node::banned;


	bfs_enumerate(f, 0, 3);
	std::cout << f << '\n';

	auto path = build_path(f, 0, 3, 4, 0);
	std::reverse(path.begin(), path.end());
	for (auto const& p : path)
		f(p.first, p.second) = node::on_path;

	std::cout << f << '\n';
	
}